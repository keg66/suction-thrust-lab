import json
from pathlib import Path

import mujoco
import numpy as np

from .model import make_xml
from .physics import GasTank, Suction


def load_config(path=None):
    c = json.loads(Path(path or Path(__file__).with_name('default.json')).read_text())
    # Reject invalid physical inputs early, instead of silently making energy/forces.
    for key in ['timestep', 'mass', 'gravity', 'ceiling_height']:
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    s, g = c['suction'], c['gas']
    for group, keys in [(s, ['area', 'max_pressure', 'max_flow', 'volume', 'air_density', 'ambient_pressure', 'response_time', 'max_power', 'max_gap']),
                        (g, ['volume', 'initial_pressure', 'temperature', 'gas_constant', 'ambient_pressure', 'exhaust_velocity'])]:
        if any(not np.isfinite(group[k]) or group[k] <= 0 for k in keys):
            raise ValueError('Physical parameters must be finite and positive')
    if g['initial_pressure'] <= g['ambient_pressure']:
        raise ValueError('Tank pressure must exceed ambient pressure')
    if not 0 <= s['min_gap'] <= s['max_gap'] or not 0 < s['alignment_cos'] <= 1 or s['battery_energy'] < 0:
        raise ValueError('Invalid seal/battery parameters')
    if not np.allclose(c['dimensions'], [.147, .09, .041]):
        raise ValueError('Phase 1 primitive geometry has fixed dimensions; edit model.py when resizing')
    if len({t['name'] for t in c['thrusters']}) != len(c['thrusters']):
        raise ValueError('Thruster names must be unique')
    for t in c['thrusters']:
        if len(t['position']) != 3 or len(t['direction']) != 3 or not np.all(np.isfinite(t['position'] + t['direction'])):
            raise ValueError('Thruster vectors must contain three finite values')
        if not np.isclose(np.linalg.norm(t['direction']), 1) or t['max_thrust'] < 0:
            raise ValueError('Thruster directions must be unit vectors and thrust nonnegative')
    return c


class Simulation:
    def __init__(self, config=None, demo='lift'):
        self.c = config or load_config()
        self.model = mujoco.MjModel.from_xml_string(make_xml(self.c))
        self.data = mujoco.MjData(self.model)
        self.car = self.model.body('car').id
        self.suction = Suction(self.c['suction'])
        self.gas = GasTank(self.c['gas'], self.c['thrusters'])
        self.thrust = np.zeros(len(self.c['thrusters']))
        self.surface = ''
        self.gap = None
        # Only scenario initialization writes pose. Steps never write qpos/qvel.
        if demo == 'ceiling':
            self.data.qpos[:3] = [0, 0, self.c['ceiling_height'] - .0205]
            self.data.qpos[3:7] = [0, 1, 0, 0]
            # Fan already spinning when released at the ceiling, pressure starts at 0.
            # A cold fan cannot catch the falling car before its skirt loses seal.
            self.suction.speed = 1.0
        elif demo == 'air':
            self.data.qpos[:3] = [0, 0, .5]
        mujoco.mj_forward(self.model, self.data)

    def seal(self):
        """All four skirt corners must face and lie within a finite surface."""
        s = self.c['suction']
        rotation = self.data.xmat[self.car].reshape(3, 3)
        hx, hy = s['half_extents']
        local = np.array([[x, y, s['bottom_z']] for x in [-hx, hx] for y in [-hy, hy]])
        corners = local @ rotation.T + self.data.xpos[self.car]
        h = self.c['ceiling_height']
        surfaces = [('floor', np.array([0, 0, 1.]), np.zeros(3), (0, 1), ((-1, 1), (-1, 1))),
                    ('ceiling', np.array([0, 0, -1.]), np.array([0, 0, h]), (0, 1), ((-1, 1), (-1, 1))),
                    ('wall', np.array([-1., 0, 0]), np.array([1., 0, 0]), (1, 2), ((-1, 1), (0, h)))]
        for name, normal, point, axes, bounds in surfaces:
            distances = (corners - point) @ normal
            in_bounds = all(np.all((corners[:, a] >= low) & (corners[:, a] <= high)) for a, (low, high) in zip(axes, bounds))
            if rotation[:, 2] @ normal >= s['alignment_cos'] and in_bounds and distances.min() >= -.001 and distances.max() <= s['max_gap']:
                return name, max(0., float(distances.mean())), -normal
        return '', None, np.zeros(3)

    def apply_force(self, force, local_point):
        rotation = self.data.xmat[self.car].reshape(3, 3)
        point = self.data.xpos[self.car] + rotation @ np.asarray(local_point)
        mujoco.mj_applyFT(self.model, self.data, np.asarray(force), np.zeros(3), point, self.car, self.data.qfrc_applied)

    def step(self, fan=0., commands=None):
        dt = self.model.opt.timestep
        self.data.qfrc_applied[:] = 0
        self.surface, self.gap, direction = self.seal()
        force = self.suction.step(dt, fan, self.gap)
        self.apply_force(direction * force, [0, 0, self.c['suction']['bottom_z']])
        self.thrust = self.gas.step(dt, commands or {})
        rotation = self.data.xmat[self.car].reshape(3, 3)
        for t, magnitude in zip(self.c['thrusters'], self.thrust):
            self.apply_force(rotation @ np.array(t['direction']) * magnitude, t['position'])
        mujoco.mj_step(self.model, self.data)
        # Refresh transforms/contact forces at the reported post-step pose.
        mujoco.mj_forward(self.model, self.data)

    def row(self):
        d = self.data
        row = {'time_s': d.time}
        for names, values in [('x_m y_m z_m qw qx qy qz', d.qpos),
                              ('vx_world_m_s vy_world_m_s vz_world_m_s wx_body_rad_s wy_body_rad_s wz_body_rad_s', d.qvel)]:
            row.update(zip(names.split(), values))
        wrench = np.zeros(6)
        normal = 0.
        for i in range(d.ncon):
            contact = d.contact[i]
            if self.car in self.model.geom_bodyid[[contact.geom1, contact.geom2]]:
                mujoco.mj_contactForce(self.model, d, i, wrench)
                normal += wrench[0]
        row.update(suction_N=self.suction.force, delta_pressure_Pa=self.suction.pressure,
                   fan_flow_m3_s=self.suction.flow, leak_flow_m3_s=self.suction.leak,
                   fan_power_W=self.suction.power, battery_J=self.suction.energy,
                   gas_kg=self.gas.mass, tank_Pa=self.gas.pressure, exhaust_energy_J=self.gas.exhaust_energy,
                   contacts=d.ncon, normal_load_N=normal, seal_surface=self.surface,
                   seal_gap_m='' if self.gap is None else self.gap)
        row.update({t['name'] + '_N': f for t, f in zip(self.c['thrusters'], self.thrust)})
        return row


def controls(sim, demo, fan_off=1.5, burn=.3):
    if demo == 'ceiling':
        return float(sim.data.time < fan_off), {}
    if demo == 'downforce':
        return float(.5 <= sim.data.time < fan_off), {}
    return 0., {t['name']: float(sim.data.time < burn) for t in sim.c['thrusters'] if t['name'].startswith('lift_')}
