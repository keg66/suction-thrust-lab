import mujoco
import numpy as np
import pytest

from suction_thrust_lab.physics import GasTank, Suction
from suction_thrust_lab.simulation import Simulation, controls, load_config


@pytest.mark.parametrize('pressure, expected', [(118, 1.18), (235, 2.35)])
def test_pressure_area(pressure, expected):
    c = load_config()['suction']
    c['min_gap'] = 0
    fan = Suction(c)
    fan.pressure = pressure
    assert fan.step(.001, 0, 0) == pytest.approx(expected, abs=1e-9)


def test_sealed_pressure_retained_without_fan():
    c = load_config()['suction']
    c['min_gap'] = 0
    fan = Suction(c)
    fan.pressure = 235
    for _ in range(1000):
        fan.step(.001, 0, 0)
    assert fan.pressure == pytest.approx(235, abs=1e-6)


def test_leak_equilibrium_and_fan_curve():
    fan = Suction(load_config()['suction'])
    for _ in range(300):
        fan.step(.001, 1, .0001)
    assert 0 < fan.pressure < fan.c['max_pressure']
    assert fan.flow == pytest.approx(fan.leak, rel=1e-5)
    assert fan.flows(fan.c['max_pressure'], 0)[0] == pytest.approx(0, abs=1e-12)
    assert fan.flows(0, 0)[0] == pytest.approx(fan.c['max_flow'], rel=1e-9)
    assert fan.pressure * fan.flow <= fan.power


@pytest.mark.parametrize('factor', [1., 1.5])
def test_gravity_and_actual_thrusters(factor):
    sim = Simulation(demo='air')
    weight = sim.c['mass'] * sim.c['gravity']
    commands = {t['name']: factor * weight / 4 / t['max_thrust'] for t in sim.c['thrusters'] if t['name'].startswith('lift_')}
    sim.step(commands=commands)
    expected = (factor - 1) * sim.c['gravity']
    assert sim.data.qacc[2] == pytest.approx(expected, abs=1e-8)
    assert sim.data.qvel[2] == pytest.approx(expected * sim.model.opt.timestep, abs=1e-8)


def test_offset_thruster_torque():
    sim = Simulation(demo='air')
    sim.step(commands={'lift_fl': 1})
    torque = np.cross([.045, .025, -.012], [0, 0, .45])
    np.testing.assert_allclose(sim.data.qfrc_applied[3:], torque, atol=1e-9)
    # First integration starts at rest; post-step acceleration includes gyroscopic torque.
    np.testing.assert_allclose(sim.data.qvel[3:] / sim.model.opt.timestep,
                               torque / sim.model.body_inertia[sim.car], rtol=1e-5, atol=1e-8)


def test_gas_mass_balance_pressure_and_exhaustion():
    c = load_config()
    tank = GasTank(c['gas'], c['thrusters'])
    initial = tank.mass
    commands = {t['name']: 1 for t in c['thrusters']}
    force = tank.step(.01, commands)
    assert initial - tank.mass == pytest.approx(force.sum() * .01 / c['gas']['exhaust_velocity'])
    assert tank.pressure < c['gas']['initial_pressure']
    tank.step(1000, commands)  # requested impulse cannot consume more than usable gas
    assert tank.mass == pytest.approx(tank.residual_mass, abs=1e-15)
    assert tank.step(.001, commands).sum() == pytest.approx(0, abs=1e-12)
    assert tank.exhaust_energy == pytest.approx(.5 * tank.used * c['gas']['exhaust_velocity']**2)


@pytest.mark.parametrize('position,quat', [([0,0,.5], [1,0,0,0]),
    ([0,0,.9795], [1,0,0,0]), ([1.1,0,.0205], [1,0,0,0]),
    ([0,0,.023], [1,0,0,0]), ([0,0,.0205], [.999, .0447101778,0,0])])
def test_no_valid_surface_no_suction(position, quat):
    sim = Simulation()
    sim.data.qpos[:3] = position
    sim.data.qpos[3:7] = quat
    mujoco.mj_forward(sim.model, sim.data)
    sim.suction.pressure = 235
    sim.step(fan=1)
    assert sim.suction.force == 0


def test_wall_seal_direction():
    sim = Simulation()
    sim.data.qpos[:3] = [.9795, 0, .5]
    sim.data.qpos[3:7] = [np.sqrt(.5), 0, -np.sqrt(.5), 0]
    mujoco.mj_forward(sim.model, sim.data)
    name, gap, direction = sim.seal()
    assert name == 'wall'
    assert gap == pytest.approx(0, abs=1e-15)
    np.testing.assert_allclose(direction, [1,0,0])


def test_battery_exhaustion():
    c = load_config()['suction']
    c['battery_energy'] = .00001
    fan = Suction(c)
    for _ in range(20):
        fan.step(.001, 1, .0001)
    assert fan.energy == 0
    assert fan.power == 0
    assert fan.force < 1e-6


def test_ceiling_hold_then_release():
    sim = Simulation(demo='ceiling')
    held = []
    for _ in range(2200):
        sim.step(*controls(sim, 'ceiling'))
        if .3 < sim.data.time < 1.5:
            held.append(sim.data.qpos[2])
    assert max(abs(z - .9795) for z in held) < .0005
    assert sim.data.qpos[2] < .1


def test_lift_demo_rises_and_uses_gas():
    sim = Simulation(demo='lift')
    peak = 0.
    for _ in range(800):
        sim.step(*controls(sim, 'lift'))
        peak = max(peak, sim.data.qpos[2])
    assert peak > .05
    assert sim.gas.used > 0


def test_downforce_increases_contact_load():
    sim = Simulation()
    for _ in range(200):
        sim.step()
    off = sim.row()['normal_load_N']
    for _ in range(300):
        sim.step(fan=1)
    on = sim.row()['normal_load_N']
    assert off == pytest.approx(sim.c['mass'] * sim.c['gravity'], abs=.01)
    assert on - off == pytest.approx(sim.suction.force, abs=.02)


def test_control_authority_is_not_six_dof():
    c = load_config()
    allocation = np.array([np.r_[t['direction'], np.cross(t['position'], t['direction'])] for t in c['thrusters']]).T
    assert np.linalg.matrix_rank(allocation) == 5  # yaw absent, unilateral thrust also limits authority


def test_cold_fan_cannot_catch_falling_car():
    sim = Simulation(demo='ceiling')
    sim.suction.speed = 0
    for _ in range(100):
        sim.step(fan=1)
    assert sim.surface == ''
    assert sim.data.qpos[2] < .95


def test_hover_requires_pressure_compensation_over_time():
    sim = Simulation(demo='air')
    u = sim.c['mass'] * sim.c['gravity'] / 1.8
    commands = {t['name']: u for t in sim.c['thrusters'] if t['name'].startswith('lift_')}
    for _ in range(100):
        sim.step(commands=commands)
    assert sim.data.qvel[2] < -.01  # constant valve command is not a hover controller
