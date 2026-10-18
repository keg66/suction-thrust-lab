import numpy as np
import pytest

from suction_thrust_lab.tornado_full_size import FullSizeTornadoSimulation


@pytest.fixture(scope='module')
def race():
    sim = FullSizeTornadoSimulation()
    samples = []
    transitions = []
    previous = sim.stage
    for step in range(6000):
        before = sim.data.qpos[:3].copy()
        sim.advance()
        np.testing.assert_allclose(sim.data.qpos[:3]-before,
                                   sim.model.opt.timestep*sim.data.qvel[:3], atol=1e-8)
        if sim.stage != previous:
            transitions.append((sim.stage, sim.data.time))
            previous = sim.stage
        if step % 10 == 0:
            samples.append(dict(stage=sim.stage, time=sim.data.time,
                rank=sim.race_position, passed=sim.overtaken,
                velocity=sim.data.qvel[:6].copy(), roll=sim.roll_angle,
                position=sim.data.qpos[:3].copy(), gas=sim.gas.mass,
                battery=sim.suction.energy))
    return sim, samples, dict(transitions)


def test_full_size_race_roll_and_immediate_touchdown(race):
    sim, samples, times = race
    assert list(times) == ['cruise', 'lift', 'roll', 'descend', 'landed']
    assert not sim.failure
    np.testing.assert_allclose(sim.c['dimensions'], [4.536, 1.971, 1.26])
    assert sim.c['mass'] == 1600.
    assert times['roll']-times['lift'] == pytest.approx(.15, abs=.002)
    assert times['descend']-times['roll'] == pytest.approx(3., abs=.002)
    assert 0 < sim.touchdown_time-times['descend'] < .35
    assert sim.landing_speed < .5 and sim.max_touchdown_speed < .5
    roll = [x for x in samples if x['stage'] == 'roll']
    assert np.degrees(roll[-1]['roll']) == pytest.approx(2160, abs=5)
    assert max(x['velocity'][3] for x in roll) > 12
    assert min(x['velocity'][0] for x in roll) > 59.
    assert sim.touchdown_forward_speed > 59.
    assert sim.data.qpos[2] == pytest.approx(.63, abs=.01)
    assert sim.data.xmat[sim.car].reshape(3, 3)[2, 2] > .99
    assert sim.data.qvel[0] > 59.


def test_all_six_overtakes_happen_during_flight(race):
    sim, samples, _ = race
    assert list(dict.fromkeys(x['rank'] for x in samples)) == [7, 6, 5, 4, 3, 2, 1]
    assert any(x['stage'] == 'roll' and x['passed'] == 6 for x in samples)
    assert all(x['rank'] == 1 for x in samples if x['stage'] == 'landed')
    assert sim.race_position == 1 and sim.overtaken == 6


def test_existing_hardware_and_finite_resources(race):
    sim, samples, _ = race
    assert len(sim.c['thrusters']) == 7
    assert all(t['max_thrust'] == 60000 for t in sim.c['thrusters'][:4])
    assert sim.gas.residual_mass < sim.gas.mass < sim.initial_gas
    assert sim.model.body_mass.sum() == pytest.approx(sim.total_mass)
    assert 0 < sim.drive_energy < 36e6
    assert all(b['gas'] <= a['gas'] and b['battery'] <= a['battery']
               for a, b in zip(samples, samples[1:]))
