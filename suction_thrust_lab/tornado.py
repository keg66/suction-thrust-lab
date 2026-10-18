"""Miniature straight-line launch, ballistic multi-roll and rolling touchdown.

The stunt car uses seven one-way jets and driven wheels, with no pose writes.
Six visual pace cars follow prescribed motion for the overtaking sequence.
"""
from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np


from .halfpipe import HalfpipeSimulation, FlightController, demo_config as halfpipe_config, ramp_xml


# Scripted pace cars: visual race traffic, excluded from the stunt's dynamics.
RIVALS = [
    (2., -.30, '.95 .12 .08 1'), (2.8, -.10, '1 .65 .03 1'),
    (3.6, .10, '.15 .8 .25 1'), (4.4, .30, '.65 .15 .9 1'),
    (5.2, -.30, '.1 .8 .85 1'), (6., -.10, '.95 .25 .6 1'),
]


def race_camera(sim, camera):
    """Start with the whole field, then track the overtakes at car level."""
    blend = np.clip(sim.data.time/.85, 0, 1)
    scale = sim.settings['race_scale']
    camera.lookat[:] = [sim.data.qpos[0]+(3*(1-blend)+.45*blend)*scale, 0, .25*scale]
    camera.distance = (7*(1-blend)+2.8*blend)*scale
    camera.azimuth, camera.elevation = 100, -28


def demo_config():
    c = halfpipe_config()
    c['tornado'] = dict(top_speed=5., runup_timeout=4., cruise_duration=.35,
                        lift_duration=.075, lift_acceleration=40., roll_duration=3.,
                        turns=14, descend_duration=.28,
                        touchdown_limit=.20, drag_coefficient=.004, opponent_speed=3.,
                        race_scale=1., roll_height=.15, roll_arch=.45, roll_ramp=.12)
    return c


def track_xml(c):
    if c.get('vehicle') == 'full_size':
        from .full_size import full_size_xml
        root = ET.fromstring(full_size_xml(c))
    else:
        root = ET.fromstring(ramp_xml(c))
    settings = demo_config()['tornado'] | c.get('tornado', {})
    scale = settings['race_scale']
    width_scale = c['dimensions'][1]/.09
    ground = c['dimensions'][2]/2
    root.set('model', 'Full-size Tornado' if c.get('vehicle') == 'full_size' else 'Mini Tornado')
    # A long runway must not enlarge MuJoCo's near clip past the miniature car.
    ET.SubElement(root.find('visual'), 'map', znear='.0001')
    world = root.find('worldbody')
    # Keep the driven chassis, replacing the half-pipe with a runway.
    for geom in list(world.findall('geom')):
        if geom.get('name') != 'floor':
            world.remove(geom)
    world.find("geom[@name='floor']").set('size', f'{50*scale} {2*width_scale} .025')
    world.find("geom[@name='floor']").set('pos', '0 0 -.025')
    world.find("body[@name='car']").set('pos', f'0 0 {ground}')
    for x in range(-2, 50):
        ET.SubElement(world, 'geom', type='box', pos=f'{x*scale} 0 .0001',
                      size=f'{.015*scale} {.035*width_scale} .0001', rgba='.85 .88 .90 1', contype='0', conaffinity='0', group='2')
    for y in [-.8, .8]:
        ET.SubElement(world, 'geom', type='box', pos=f'{24*scale} {y*width_scale} .0001',
                      size=f'{26*scale} {.008*width_scale} .0001', rgba='.95 .25 .08 1', contype='0', conaffinity='0', group='2')
    car = world.find("body[@name='car']")
    for index, (x, y, color) in enumerate(RIVALS):
        rival = ET.SubElement(world, 'body', name=f'rival_{index}', mocap='true',
                              pos=f'{x*scale} {y*width_scale} {ground}')
        # Same vehicle silhouette, with a distinct body color for each rival.
        for source in car.findall('geom'):
            geom = deepcopy(source)
            geom.attrib.pop('name', None)
            geom.set('mass', '0')
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            geom.set('group', '1')
            rival.append(geom)
        # Driven wheel bodies are one level below the chassis.
        for wheel in car.findall('body'):
            if not wheel.get('name', '').startswith('wheel_body_'):
                continue
            geom = deepcopy(wheel.find('geom'))
            geom.attrib.pop('name', None)
            geom.set('pos', wheel.get('pos'))
            geom.set('mass', '0')
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            geom.set('group', '1')
            rival.append(geom)
        for source in car.find("body[@name='visuals']").findall('geom'):
            geom = deepcopy(source)
            geom.attrib.pop('name', None)
            geom.set('mass', '0')
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            geom.set('group', '1')
            if geom.get('pos') != '0 0 -.002':
                geom.set('rgba', color)
            rival.append(geom)
    return ET.tostring(root, encoding='unicode')


class TornadoSimulation(HalfpipeSimulation):
    def __init__(self, config=None):
        c = config or demo_config()
        defaults = demo_config()['tornado']
        self.settings = defaults | c.get('tornado', {})
        for key, value in self.settings.items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'tornado.{key} must be finite and positive')
        if int(self.settings['turns']) != self.settings['turns']:
            raise ValueError('tornado.turns must be an integer for upright recovery')
        super().__init__(c, xml=track_xml(c))
        self.stage = 'accelerate'
        self.suction.speed = 0.
        self.controller = TornadoController(self)
        self.roll_angle = 0.
        self.peak_speed = 0.
        self.launch_speed = None
        self.touchdown_forward_speed = None
        self.rival_bodies = [self.model.body(f'rival_{i}').id for i in range(len(RIVALS))]
        self.rival_mocaps = self.model.body_mocapid[self.rival_bodies]
        self.rival_starts = self.model.body_pos[self.rival_bodies].copy()
        self.wheel_radius = .013*self.length_scale
        self.ground_height = c['dimensions'][2]/2

    def apply_environment_forces(self):
        # Only visual pace cars use prescribed motion; the stunt car is still
        # integrated entirely from wheel, gas, gravity and aerodynamic forces.
        self.data.mocap_pos[self.rival_mocaps] = self.rival_starts
        self.data.mocap_pos[self.rival_mocaps, 0] += self.settings['opponent_speed']*(
            self.data.time+self.model.opt.timestep)
        velocity = self.data.qvel[:3]
        drag = -self.settings['drag_coefficient'] * np.linalg.norm(velocity) * velocity
        self.apply_force(drag, [0, 0, 0])

    def advance(self):
        fan, commands, drive = self.controller.control()
        omega = self.data.qvel[self.wheel_dofs]
        self.data.ctrl[:] = np.clip(.003*self.torque_scale*self.time_scale * (drive/self.wheel_radius-omega),
                                    -.012*self.torque_scale, .012*self.torque_scale) if self.suction.energy > 0 else 0.
        # Free wheels keep their forward rolling speed, avoiding a pitch impulse
        # from braking them in flight and a skid on touchdown.
        if self.stage in ['lift', 'roll', 'descend']:
            self.data.ctrl[:] = 0.
        before_vz = abs(float(self.data.qvel[2]))
        before_vx = float(self.data.qvel[0])
        self.step(fan, commands)
        self.roll_angle += self.data.qvel[3] * self.model.opt.timestep
        self.peak_speed = max(self.peak_speed, self.data.qvel[0])
        floor = self.model.geom('floor').id
        touching = any(floor in (c.geom1, c.geom2) for c in self.data.contact)
        if self.flight_start is not None and touching and not self.previous_floor_contact:
            self.max_touchdown_speed = max(self.max_touchdown_speed, before_vz)
            if self.touchdown_time is None:
                self.touchdown_time = self.data.time
                self.landing_speed = before_vz
                self.touchdown_forward_speed = before_vx
            rot = self.data.xmat[self.car].reshape(3, 3)
            if self.stage not in ['descend', 'landed'] or rot[2, 2] < .98:
                self.failure = 'Floor contact before upright descent'
            elif before_vz > self.settings['touchdown_limit']:
                self.failure = 'Vertical touchdown speed exceeded limit'
        self.previous_floor_contact = touching
        if self.flight_start is not None and self.stage != 'landed':
            if self.gas.mass <= self.gas.residual_mass + 1e-10:
                self.failure = 'Gas exhausted during flight'
            if self.data.time-self.flight_start > 7.:
                self.failure = 'Flight timed out before landing'
        elif self.flight_start is None and self.data.time > self.settings['runup_timeout']:
            self.failure = 'Failed to reach configured top speed'

    @property
    def race_position(self):
        return 1+int(np.count_nonzero(self.data.xpos[self.rival_bodies, 0] > self.data.qpos[0]))

    @property
    def overtaken(self):
        return int(np.count_nonzero(self.data.qpos[0]-self.data.xpos[self.rival_bodies, 0]
                                    > self.c['dimensions'][0]))

    def row(self):
        row = super().row()
        row.update(race_position=self.race_position, overtaken=self.overtaken, roll_total_deg=np.degrees(self.roll_angle), peak_speed_m_s=self.peak_speed,
                   launch_speed_m_s='' if self.launch_speed is None else self.launch_speed,
                   touchdown_forward_speed_m_s='' if self.touchdown_forward_speed is None else self.touchdown_forward_speed)
        return row


class TornadoController(FlightController):
    def __init__(self, sim):
        super().__init__(sim)
        # This scenario specifies rates and durations in physical seconds.
        self.T = 1.

    def control(self):
        s = self.s
        p = s.settings
        t = s.data.time
        elapsed = t-s.stage_time
        rot = s.data.xmat[s.car].reshape(3, 3)
        pos, vel = s.data.qpos[:3], s.data.qvel[:3]
        if s.stage == 'accelerate':
            if vel[0] >= .98*p['top_speed']:
                self.transition('cruise')
            return 0., {}, p['top_speed']
        if s.stage == 'cruise':
            if elapsed < p['cruise_duration']:
                return 0., {}, p['top_speed']
            self.transition('lift')
            s.flight_start = t
            s.launch_speed = float(vel[0])
            self.launch_rotation = np.eye(3)
            elapsed = 0.
        if s.stage == 'landed':
            return 0., {}, p['top_speed']
        if s.stage == 'lift' and elapsed >= p['lift_duration']:
            self.transition('roll')
            elapsed = 0.
        if s.stage == 'roll' and elapsed >= p['roll_duration']:
            self.transition('descend')
            # Continue the falling trajectory directly into touchdown; no hover
            # or climb back to a recovery altitude after the final turn.
            self.recovery_height = float(pos[2])
            self.descent_velocity = float(vel[2])
            elapsed = 0.
        if s.stage == 'descend' and s.touchdown_time is not None and s.previous_floor_contact:
            if t-s.touchdown_time > .20 and rot[2, 2] > .99:
                self.transition('landed')
                return 0., {}, p['top_speed']
        boost = np.clip(s.total_mass*4*(p['top_speed']-vel[0]) + p['drag_coefficient']*vel[0]**2, 0, s.c['thrusters'][6]['max_thrust'])
        desired = np.eye(3)
        rate, accel = np.zeros(3), np.zeros(3)
        if s.stage == 'lift':
            lift = s.total_mass*(s.c['gravity']+p['lift_acceleration'])
        elif s.stage == 'roll':
            # Sustain fast rolls with short, smooth entry/exit ramps.
            # The same underside jets provide roll torque and pulsed support.
            u = np.clip(elapsed/p['roll_duration'], 0, 1)
            duration, angle = p['roll_duration'], 2*np.pi*p['turns']
            ramp = min(p['roll_ramp'], duration/4)
            peak_rate = angle/(duration-ramp)
            if elapsed < ramp:
                r = elapsed/ramp
                phi = peak_rate*ramp*(r**3-.5*r**4)
                rate[0] = peak_rate*(3*r*r-2*r**3)
                accel[0] = peak_rate*(6*r-6*r*r)/ramp
            elif elapsed > duration-ramp:
                r = (duration-elapsed)/ramp
                phi = angle-peak_rate*ramp*(r**3-.5*r**4)
                rate[0] = peak_rate*(3*r*r-2*r**3)
                accel[0] = -peak_rate*(6*r-6*r*r)/ramp
            else:
                phi = peak_rate*(elapsed-ramp/2)
                rate[0] = peak_rate
            cr, sr = np.cos(phi), np.sin(phi)
            desired = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            # Pulse jets when their rotating axis aligns with the requested
            # world force, sustaining altitude and correcting lateral drift.
            target_h = p['roll_height']+p['roll_arch']*np.sin(np.pi*u)
            target_v = p['roll_arch']*np.pi/duration*np.cos(np.pi*u)
            target_a = -p['roll_arch']*(np.pi/duration)**2*np.sin(np.pi*u)
            force_z = s.total_mass*max(0., s.c['gravity']+target_a
                +25*(target_h-pos[2])+7*(target_v-vel[2]))
            # At speed the upward half-cycle averages cos(phi)^2 to 1/4;
            # during slow entry/exit use direct support instead of over-lifting.
            blend = np.clip(abs(rate[0])/min(12., peak_rate), 0, 1)
            support = 1.-.75*blend
            force_y = s.total_mass*(-12*pos[1]-7*vel[1])
            lift = max(0., force_y*rot[1, 2]+force_z*rot[2, 2])/support
        else:
            duration = p['descend_duration']
            u = np.clip(elapsed/duration, 0, 1)
            # Quintic matches the current downward velocity and finishes with a
            # small downward speed, so contact happens without a hovering tail.
            h0, v0 = self.recovery_height, self.descent_velocity*duration
            hf, vf = s.ground_height-.002*max(1., s.length_scale/10), -.08*duration
            delta = hf-h0
            a3 = 10*delta-6*v0-4*vf
            a4 = -15*delta+8*v0+7*vf
            a5 = 6*delta-3*v0-3*vf
            height = h0+v0*u+a3*u**3+a4*u**4+a5*u**5
            vz = (v0+3*a3*u**2+4*a4*u**3+5*a5*u**4)/duration
            az = (6*a3*u+12*a4*u**2+20*a5*u**3)/duration**2
            if elapsed > duration:
                height += -.08*(elapsed-duration)
            # Stay level during the short touchdown instead of tilting to
            # remove lateral displacement when little lift is available.
            force_z = s.total_mass*max(.5, s.c['gravity']+az
                +40*(height-pos[2])+12*(vz-vel[2]))
            lift = max(0., force_z*rot[2, 2])
        transform = rot.T@desired
        torque = self.attitude_torque(desired, transform@rate, transform@accel)
        return 0., self.allocate(np.r_[boost, 0., lift, torque]), 0.
