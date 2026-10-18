import numpy as np
import pytest

from suction_thrust_lab.tornado import TornadoSimulation, demo_config


@pytest.fixture(scope='module')
def flight():
    sim = TornadoSimulation()
    samples, transitions = [], []
    previous = sim.stage
    for step in range(6000):
        old = sim.data.qpos[:3].copy()
        sim.advance()
        # Integration, never a scripted translation or pose reset.
        np.testing.assert_allclose(sim.data.qpos[:3]-old, .001*sim.data.qvel[:3], atol=1e-9)
        if sim.stage != previous:
            transitions.append(sim.stage)
            previous = sim.stage
        if step % 10 == 0:
            samples.append((sim.stage, sim.data.qpos.copy(), sim.data.qvel.copy(),
                            sim.roll_angle, sim.gas.mass, sim.suction.energy, sim.data.time, sim.race_position,
                            sim.overtaken, sim.data.xpos[sim.rival_bodies, 0].copy()))
    return sim, samples, transitions


def test_top_speed_multi_roll_and_rolling_landing(flight):
    sim, samples, transitions = flight
    assert transitions == ['cruise', 'lift', 'roll', 'descend', 'landed']
    assert not sim.failure
    assert sim.c['dimensions'] == [.147, .09, .041]
    assert sim.launch_speed >= .98*sim.settings['top_speed']
    roll = [x for x in samples if x[0] == 'roll']
    assert min(x[1][2] for x in roll) > .10
    assert min(x[2][0] for x in roll) > .95*sim.settings['top_speed']
    assert max(x[2][3] for x in roll) > 25  # sustained high-speed roll
    assert np.degrees(roll[-1][3]) == pytest.approx(5040, abs=5)
    assert roll[0][6]-sim.flight_start < .09
    descent = [x for x in samples if x[0] == 'descend']
    assert descent[0][6]-roll[0][6] == pytest.approx(3., abs=.02)
    assert sim.touchdown_time-descent[0][6] < .30
    assert sum(x[2][3] > 25 for x in roll) > .9*len(roll)
    assert all(x[2][2] < 0 for x in descent if x[6] < sim.touchdown_time)
    assert sim.landing_speed < sim.settings['touchdown_limit']
    assert sim.touchdown_forward_speed > .95*sim.settings['top_speed']
    assert sim.data.xmat[sim.car].reshape(3, 3)[2, 2] > .99
    assert sim.data.qpos[2] == pytest.approx(.0205, abs=.001)
    assert sim.data.qvel[0] == pytest.approx(sim.settings['top_speed'], abs=.02)
    assert np.linalg.norm(sim.data.qvel[3:6]) < .01


def test_finite_resources_and_mass(flight):
    sim, samples, _ = flight
    assert 0 < sim.gas.used < sim.initial_gas-sim.gas.residual_mass
    assert all(b[4] <= a[4] and b[5] <= a[5] for a, b in zip(samples, samples[1:]))
    assert sim.model.body_mass.sum() == pytest.approx(sim.total_mass)
    assert sim.drive_energy > 0
    assert sim.suction.energy > 0


def test_gas_exhaustion_cannot_report_success():
    sim = TornadoSimulation()
    for _ in range(1200):
        sim.advance()
    assert sim.stage == 'roll'
    sim.gas.mass = sim.gas.residual_mass
    sim.update_mass()
    sim.advance()
    assert sim.failure == 'Gas exhausted during flight'
    assert sim.thrust.sum() == 0


def test_invalid_roll_count():
    c = demo_config()
    c['tornado']['turns'] = 2.5
    with pytest.raises(ValueError, match='integer'):
        TornadoSimulation(c)


def test_overtake_the_whole_moving_field_during_tornado(flight):
    sim, samples, _ = flight
    ranks = list(dict.fromkeys(x[7] for x in samples))
    assert ranks == [7, 6, 5, 4, 3, 2, 1]
    assert samples[0][8] == 0
    assert sim.race_position == 1 and sim.overtaken == 6
    assert any(x[0] == 'roll' and x[8] == 6 for x in samples)
    assert all(b[8] >= a[8] for a, b in zip(samples, samples[1:]))
    assert all(x[7] == 1 for x in samples if x[0] == 'landed')
    for a, b in zip(samples, samples[1:]):
        np.testing.assert_allclose((b[9]-a[9])/(b[6]-a[6]),
                                   sim.settings['opponent_speed'], atol=1e-8)
