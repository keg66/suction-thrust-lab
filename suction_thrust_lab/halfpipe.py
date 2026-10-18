"""Wheel-driven curved climb and thruster-controlled free flight (no pose servo)."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .model import make_xml
from .simulation import Simulation, load_config


def demo_config():
    return load_config(Path(__file__).with_name('halfpipe.json'))


def ramp_xml(c):
    root = ET.fromstring(make_xml(c))
    world = root.find('worldbody')
    for name in ['ceiling', 'wall']:
        world.remove(world.find(f"geom[@name='{name}']"))
    floor = world.find("geom[@name='floor']")
    floor.set('size', '2 1 .025')
    floor.set('rgba', '.14 .18 .22 1')
    r, segments = c['ramp']['radius'], c['ramp']['segments']
    da = np.pi / segments
    for i in range(segments):
        a = (i + .5) * da
        normal = np.array([-np.sin(a), 0, np.cos(a)])
        # Chord surface is the same geometry used for collision and ray seals.
        p = np.array([r*np.sin(a), 0, r*(1-np.cos(a))]) - .008 * normal
        ET.SubElement(world, 'geom', name=f'ramp_{i}', type='box',
                      pos=' '.join(map(str,p)), size=f'{r*np.sin(da/2)+.00003} .15 .008',
                      euler=f'0 {-a} 0', rgba='.36 .42 .48 1', friction='1.2 .001 .00001')
        for y in [-.151, .151]:
            p[1] = y
            ET.SubElement(world, 'geom', type='box', pos=' '.join(map(str,p)),
                          size=f'{r*np.sin(da/2)} .002 .009', euler=f'0 {-a} 0',
                          rgba='.95 .15 .08 1', contype='0', conaffinity='0', group='2')
    ET.SubElement(world, 'geom', name='landing_pad', type='box', pos='-.65 0 .0001',
                  size='.16 .18 .0001', rgba='.05 .48 .43 1', contype='0', conaffinity='0', group='2')
    body = world.find("body[@name='car']")
    body.set('pos', '-.16 0 .0205')
    # Explicit chassis inertia; four independently driven wheels contribute separately.
    body.find('inertial').set('mass', str(c['mass']-.004))
    actuator = ET.SubElement(root, 'actuator')
    for i, geom in enumerate(list(body.findall('geom'))):
        if not geom.get('name', '').startswith('wheel_'):
            continue
        body.remove(geom)
        wheel = ET.SubElement(body, 'body', name=f'wheel_body_{i}', pos=geom.get('pos'))
        ET.SubElement(wheel, 'joint', name=f'wheel_joint_{i}', type='hinge', axis='0 1 0', damping='.000002', armature='.0000001')
        geom.set('pos', '0 0 0')
        geom.set('mass', '.001')
        geom.set('friction', '1.2 .001 .00001')
        wheel.append(geom)
        ET.SubElement(actuator, 'motor', name=f'drive_{i}', joint=f'wheel_joint_{i}', ctrllimited='true', ctrlrange='-.012 .012')
    for geom in body.iter('geom'):
        geom.set('group', '1')
    return ET.tostring(root, encoding='unicode')


class HalfpipeSimulation(Simulation):
    def __init__(self, config=None, xml=None):
        c = config or demo_config()
        super().__init__(c, xml=xml or ramp_xml(c))
        self.length_scale = c.get("scaling", {}).get("length", 1.)
        self.time_scale = c.get("scaling", {}).get("time", 1.)
        self.torque_scale = c.get("scaling", {}).get("torque", 1.)
        self.force_scale = (c["mass"]+self.gas.mass) / .16597335476304644
        self.const_data = mujoco.MjData(self.model)
        self.dry_inertia = self.model.body_inertia[self.car].copy()
        self.initial_gas = self.gas.mass
        self.drive_energy = self.drive_power = 0.
        self.stage = 'climb'
        self.stage_time = 0.
        self.flight_start = None
        self.flight_origin = None
        self.flip_start_angle = 0.
        self.landing_speed = None
        self.touchdown_time = None
        self.max_touchdown_speed = 0.
        self.previous_floor_contact = True
        self.failure = ''
        self.allocation_error = 0.
        self.wheel_dofs = self.model.jnt_dofadr[1:]
        self.ray_groups = np.array([1,0,0,0,0,0], dtype=np.uint8)
        self.suction.speed = 1.
        self.update_mass()
        self.controller = FlightController(self)

    @property
    def total_mass(self):
        return self.c['mass'] + self.gas.mass

    def update_mass(self):
        # Gas treated as a symmetric distribution with the chassis inertia shape.
        dry = self.c['mass'] - 4*self.c.get('wheel_mass', .001)
        self.model.body_mass[self.car] = dry + self.gas.mass
        self.model.body_inertia[self.car] = self.dry_inertia * (dry + self.gas.mass) / dry
        # Scratch data keeps the actual trajectory untouched by setConst's qpos0 work.
        mujoco.mj_setConst(self.model, self.const_data)
        mujoco.mj_forward(self.model, self.data)

    def seal(self):
        s = self.c['suction']
        rot = self.data.xmat[self.car].reshape(3,3)
        hx, hy = s['half_extents']
        direction = -rot[:,2]
        gaps, normals, names = [], [], []
        # Corners and centre prevent spanning a missing portion of track.
        for x,y in [(-hx,-hy),(-hx,hy),(hx,-hy),(hx,hy),(0,0)]:
            p = self.data.xpos[self.car] + rot @ [x,y,s['bottom_z']+.01*self.length_scale]
            hit = np.array([-1], dtype=np.int32)
            normal = np.zeros(3)
            distance = mujoco.mj_ray(self.model,self.data,p,direction,self.ray_groups,True,-1,hit,normal)
            gap = distance - .01*self.length_scale
            if distance < 0 or gap < -.004*self.length_scale or gap > self.c['ramp']['skirt_travel'] or normal @ rot[:,2] < .94:
                return '', None, np.zeros(3)
            gaps.append(gap)
            normals.append(normal)
            names.append(self.model.geom(hit[0]).name)
        # Flexible skirt conforms within its finite travel; remaining leakage is min_gap.
        normal = np.mean(normals,axis=0)
        normal /= np.linalg.norm(normal)
        return ('ramp' if any(n.startswith('ramp') for n in names) else 'floor'), max(0., max(gaps)-self.c['ramp']['skirt_travel']), -normal

    def step(self, fan=0., commands=None):
        super().step(fan,commands)
        dt = self.model.opt.timestep
        # Motor electrical input: mechanical output / eta + resistive torque losses.
        torques = self.data.actuator_force
        omega = self.data.qvel[self.wheel_dofs]
        self.drive_power = float(np.maximum(torques*omega,0).sum()/.65 + 1000/(self.torque_scale*self.time_scale)*np.dot(torques,torques))
        used = min(self.suction.energy, self.drive_power*dt)
        self.suction.energy -= used
        self.drive_energy += used
        if np.any(self.thrust):
            self.update_mass()

    def advance(self):
        fan, commands, drive = self.controller.control()
        omega = self.data.qvel[self.wheel_dofs]
        self.data.ctrl[:] = np.clip(.003*self.torque_scale*self.time_scale*(drive/(.013*self.length_scale) - omega),-.012*self.torque_scale,.012*self.torque_scale) if self.suction.energy > 0 else 0
        if self.stage not in ['climb','landed']:
            self.data.ctrl[:] = np.clip(-.00003*self.torque_scale*self.time_scale*omega,-.001*self.torque_scale,.001*self.torque_scale)
        before_speed = float(np.linalg.norm(self.data.qvel[:3]))
        self.step(fan, commands)
        floor = self.model.geom('floor').id
        touching = any(floor in (contact.geom1, contact.geom2) for contact in self.data.contact)
        if self.stage != 'climb' and touching and not self.previous_floor_contact:
            self.max_touchdown_speed=max(self.max_touchdown_speed,before_speed)
            if self.touchdown_time is None:
                self.touchdown_time = self.data.time
                self.landing_speed = before_speed
            if self.stage not in ['descend','landed']:
                self.failure = 'Premature floor contact before the descent stage'
            elif before_speed > self.c.get('flight',{}).get('touchdown_limit',.15):
                self.failure = f"Touchdown speed exceeded {self.c.get('flight',{}).get('touchdown_limit',.15)} m/s"
        self.previous_floor_contact=touching
        if self.stage not in ['climb','landed'] and self.gas.mass <= self.gas.residual_mass+1e-10:
            self.failure = 'Gas exhausted during flight'
        if self.flight_start is not None and self.stage != 'landed' and self.data.time-self.flight_start > self.c.get('flight',{}).get('timeout',8.):
            self.failure = 'Flight timed out before a stable landing'

    def row(self):
        row = super().row()
        rot = self.data.xmat[self.car].reshape(3,3)
        launch = self.controller.launch_rotation
        relative = np.eye(3) if launch is None else launch.T@rot
        row.update(stage=self.stage, total_mass_kg=self.total_mass, drive_power_W=self.drive_power,
                   drive_energy_J=self.drive_energy, allocation_error=self.allocation_error,
                   touchdown_speed_m_s='' if self.landing_speed is None else self.landing_speed,
                   max_touchdown_speed_m_s=self.max_touchdown_speed,
                   roll_from_launch_deg=np.degrees(np.arctan2(relative[2,1],relative[1,1])),
                   pitch_deg=np.degrees(np.arctan2(self.data.xmat[self.car].reshape(3,3)[2,0],self.data.xmat[self.car].reshape(3,3)[0,0])),
                   failure=self.failure)
        return row


class FlightController:
    def __init__(self, sim):
        self.s = sim
        self.L, self.T = sim.length_scale, sim.time_scale
        self.flight = sim.c.get('flight',{})
        self.recover_duration = self.flight.get('recover_duration',2.8*self.T)/self.T
        self.descend_duration = self.flight.get('descend_duration',3.*self.T)/self.T
        self.A = np.array([np.r_[t['direction'],np.cross(t['position'],t['direction'])] for t in sim.c['thrusters']]).T
        reference = demo_config()['thrusters']
        shape=np.array(sim.c['dimensions'])/np.array([.147,.09,.041])
        expected = np.array([np.r_[t['direction'],np.cross(np.array(t['position'])*shape,t['direction'])] for t in reference]).T
        if self.A.shape != expected.shape or not np.allclose(self.A,expected):
            raise ValueError('Halfpipe allocator requires the documented 7-nozzle geometry/order')
        self.last_control = -np.inf
        self.commands = {}
        self.launch_rotation = None
        self.recovery_height = None
        self.recovery_origin = None

    def allocate(self, wrench):
        s = self.s
        ratio = max(0.,(s.gas.pressure-s.gas.c['ambient_pressure'])/(s.gas.c['initial_pressure']-s.gas.c['ambient_pressure']))
        limits = np.array([t['max_thrust'] for t in s.c['thrusters']])*ratio
        desired = np.zeros(7)
        # Four unilateral bottom jets: prefer attitude torque over requested lift.
        # Differential forces generate roll/pitch; a common bias prevents negatives.
        xs=np.array([t['position'][0] for t in s.c['thrusters'][:4]])
        ys=np.array([t['position'][1] for t in s.c['thrusters'][:4]])
        for i,t in enumerate(s.c['thrusters'][:4]):
            x,y,_ = t['position']
            desired[i] = max(0.,wrench[2])/4 + wrench[3]*y/np.dot(ys,ys) - wrench[4]*x/np.dot(xs,xs)
        desired[:4] -= min(0.,desired[:4].min())
        # The two front lateral jets couple yaw and lateral force: tau_z=.055 F_y.
        lateral = wrench[5]/s.c['thrusters'][4]['position'][0]
        desired[4 if lateral<0 else 5] = abs(lateral)
        desired[6] = max(0.,wrench[0])
        actual = np.minimum(desired,limits)
        s.allocation_error = float(np.linalg.norm(self.A@actual-wrench))
        return {t['name']:float(actual[i]/limits[i]) if limits[i]>0 else 0. for i,t in enumerate(s.c['thrusters'])}

    def transition(self, stage):
        self.s.stage = stage
        self.s.stage_time = self.s.data.time

    def attitude_torque(self, desired, rate=None, accel=None):
        s=self.s
        rot=s.data.xmat[s.car].reshape(3,3)
        qerr=np.zeros(4)
        mujoco.mju_mat2Quat(qerr,(rot.T@desired).ravel())
        if qerr[0]<0: qerr *= -1
        angle=2*np.arctan2(np.linalg.norm(qerr[1:]),max(0.,qerr[0]))
        error=qerr[1:]*angle/max(np.linalg.norm(qerr[1:]),1e-12)
        omega=s.data.qvel[3:6]*self.T
        inertia=s.model.body_inertia[s.car]
        rate=np.zeros(3) if rate is None else rate
        accel=np.zeros(3) if accel is None else accel
        feedback_time=self.flight.get('attitude_response_time',self.T)
        alpha=450/feedback_time**2*error+36/feedback_time*(rate-omega)/self.T+accel/self.T**2
        return inertia*alpha+np.cross(omega,inertia*omega)/self.T**2

    def control(self):
        s=self.s
        t=s.data.time
        pos=s.data.qpos[:3]/self.L
        velocity=s.data.qvel[:3]*self.T/self.L
        rot=s.data.xmat[s.car].reshape(3,3)
        if s.stage=='climb':
            name,_,_=s.seal()
            if pos[2]>.7 and name=='':
                self.transition('depart')
                s.flight_start=t
                s.flight_origin=s.data.qpos[:3].copy()
                s.flip_start_angle=np.arctan2(rot[2,0],rot[0,0])%(2*np.pi)
                self.launch_rotation=rot.copy()
            else:
                return 1.,{},s.c['ramp']['drive_speed']*min(1.,t/(.5*self.T))
        if s.stage=='landed': return 0.,{},0.
        if t-self.last_control < min(.002*self.T,.01)-1e-8:
            return 0.,self.commands,0.
        self.last_control=t
        elapsed=(t-s.flight_start)/self.T
        if elapsed < .08:
            # Rear jet clears the lip while the inverted car falls under gravity.
            torque=self.attitude_torque(self.launch_rotation)
            wrench=np.r_[.4*s.force_scale,0.,0.,torque]
        elif elapsed < .33:
            if s.stage!='flip':self.transition('flip')
            u=np.clip((elapsed-.08)/.25,0,1)
            phi=np.pi*(3*u*u-2*u**3)
            rate=np.pi*6*u*(1-u)/.25
            acceleration=np.pi*(6-12*u)/.25**2
            cr,sr=np.cos(phi),np.sin(phi)
            rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
            desired=self.launch_rotation@rx
            transform=rot.T@desired
            torque=self.attitude_torque(desired,transform@np.array([rate,0,0]),transform@np.array([acceleration,0,0]))
            # No upward supporting jet exists while inverted. Delay braking until
            # almost upright, to avoid a large lateral impulse during the roll.
            lift=0.
            if rot[2,2]>.96:
                lift=np.clip(s.total_mass*(s.c['gravity']-14*velocity[2]*self.L/self.T**2) / rot[2,2],0,9.*s.force_scale)
            wrench=np.r_[0.,0.,lift,torque]
        else:
            if s.stage=='flip':
                self.transition('recover')
                self.recovery_height=max(.30,pos[2])
                self.recovery_origin=pos.copy()
            if s.stage=='recover' and elapsed>.33+self.recover_duration:
                self.transition('descend')
            if s.stage=='recover':
                u=np.clip((elapsed-.33)/self.recover_duration,0,1)
                blend=10*u**3-15*u**4+6*u**5
                rate=(30*u**2-60*u**3+30*u**4)/self.recover_duration
                accel_rate=(60*u-180*u**2+120*u**3)/self.recover_duration**2
                delta=np.array([-.65,0,self.recovery_height])-self.recovery_origin
                target=self.recovery_origin+delta*blend
                target[2]=self.recovery_height
                target_v=delta*rate
                target_v[2]=0.
                target_a=delta*accel_rate
                target_a[2]=0.
            else:
                u=np.clip((t-s.stage_time)/(self.T*self.descend_duration),0,1)
                blend=10*u**3-15*u**4+6*u**5
                rate=(30*u**2-60*u**3+30*u**4)/self.descend_duration
                accel_rate=(60*u-180*u**2+120*u**3)/self.descend_duration**2
                target=np.array([-.65,0,self.recovery_height+(.0195-self.recovery_height)*blend])
                target_v=np.array([0,0,(.0195-self.recovery_height)*rate])
                target_a=np.array([0,0,(.0195-self.recovery_height)*accel_rate])
                floor=s.model.geom('floor').id
                touching=any(floor in (contact.geom1,contact.geom2) for contact in s.data.contact)
                settle_speed=self.flight.get('settle_speed',.06*self.L/self.T)
                if s.touchdown_time is not None and touching and t-s.touchdown_time>.15*self.T and rot[2,2]>.99 and np.linalg.norm(s.data.qvel[:3])<settle_speed:
                    self.transition('landed')
                    return 0.,{},0.
            gain=self.T/self.flight.get('position_response_time',self.T)
            acc=12*gain**2*(target-pos)+7*gain*(target_v-velocity)
            acc[2]=22*gain**2*(target[2]-pos[2])+10*gain*(target_v[2]-velocity[2])
            if self.flight.get('acceleration_feedforward',False): acc+=target_a
            acc=acc*self.L/self.T**2
            horizontal_limit=self.flight.get('horizontal_acceleration_limit',float('inf'))
            norm=np.linalg.norm(acc[:2])
            if norm>horizontal_limit: acc[:2]*=horizontal_limit/norm
            force=s.total_mass*(acc+[0,0,s.c['gravity']])
            force[2]=max(.15*s.force_scale,force[2])
            # Underactuated flight: tilt the four bottom jets toward the required
            # world force. Side jets are used for yaw, not fictitious independent Fy.
            z_axis=force/np.linalg.norm(force)
            heading=np.array([-1.,0,0])
            y_axis=np.cross(z_axis,heading);y_axis/=np.linalg.norm(y_axis)
            x_axis=np.cross(y_axis,z_axis)
            desired=np.column_stack([x_axis,y_axis,z_axis])
            torque=self.attitude_torque(desired)
            wrench=np.r_[0.,0.,max(0.,force@rot[:,2]),torque]
        self.commands=self.allocate(wrench)
        return 0.,self.commands,0.


def draw_exhaust(sim, scene):
    """Blue arrows depict actual exhaust direction, with length scaled by thrust."""
    rot = sim.data.xmat[sim.car].reshape(3,3)
    for nozzle,force in zip(sim.c['thrusters'],sim.thrust):
        if force < .015*sim.force_scale or scene.ngeom >= scene.maxgeom:
            continue
        start = sim.data.xpos[sim.car] + rot@np.array(nozzle['position'])
        end = start - rot@np.array(nozzle['direction'])*sim.length_scale*min(.13,.035+.035*force/sim.force_scale)
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.1,.7,1.,.8]))
        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,.003*sim.length_scale,start,end)
        scene.ngeom += 1
