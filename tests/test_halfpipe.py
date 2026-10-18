import mujoco
import numpy as np
import pytest

from suction_thrust_lab.halfpipe import HalfpipeSimulation, demo_config


@pytest.fixture(scope='module')
def flight():
    s = HalfpipeSimulation()
    samples = []
    transitions = []
    previous = s.stage
    for step in range(22000):
        old = s.data.qpos[:3].copy()
        s.advance()
        np.testing.assert_allclose(s.data.qpos[:3]-old,s.model.opt.timestep*s.data.qvel[:3],atol=1e-9)
        if s.stage != previous:
            transitions.append((s.stage, s.data.time))
            previous = s.stage
        if step % 20 == 0:
            samples.append(dict(time=s.data.time, stage=s.stage, pos=s.data.qpos[:3].copy(),
                                velocity=s.data.qvel[:6].copy(), suction=s.suction.force,
                                gas=s.gas.mass, pitch=s.row()['pitch_deg'], battery=s.suction.energy,
                                allocation=s.allocation_error, seal=s.surface,
                                forward=s.data.xmat[s.car].reshape(3,3)[:,0].copy(),
                                roll=s.row()['roll_from_launch_deg']))
    return s, samples, transitions


def test_continuous_climb_flip_and_soft_landing(flight):
    s, samples, transitions = flight
    assert [stage for stage,t in transitions] == ['depart','flip','recover','descend','landed']
    assert not s.failure
    assert s.stage == 'landed'
    assert 0 <= s.landing_speed < .15
    np.testing.assert_allclose(s.data.qpos[:3],[-.65,0,.0205],atol=.005)
    assert np.linalg.norm(s.data.qvel[:6]) < .01
    climb = [x for x in samples if x['stage']=='climb' and x['time']>1]
    assert max(x['pos'][2] for x in climb) > .85
    assert all(x['suction'] > 3 for x in climb)
    assert all(x['seal'] for x in climb)
    assert 165 < np.degrees(s.flip_start_angle) < 180
    flip = [x for x in samples if x['stage']=='flip']
    assert any(60 < x['roll'] < 120 for x in flip)
    assert all(x['forward']@s.controller.launch_rotation[:,0] > .97 for x in flip)
    assert abs(s.row()['roll_from_launch_deg']) > 170
    assert s.data.xmat[s.car].reshape(3,3)[0,0] < -.99  # nose remains pointing away from ramp
    assert all(x['suction']==0 for x in samples if x['stage'] in ['depart','flip','recover','descend'])
    assert min(x['pos'][2] for x in samples if x['stage']=='recover') > .1
    assert max(x['allocation'] for x in samples) > 0  # constrained layout cannot realize every wrench


def test_consumption_and_mass_conservation(flight):
    s, samples, _ = flight
    assert .025 < s.gas.used < .04
    assert s.total_mass == pytest.approx(s.c['mass']+s.gas.mass)
    assert s.model.body_mass.sum() == pytest.approx(s.total_mass,abs=1e-10)
    assert s.gas.mass > s.gas.residual_mass
    assert all(b['gas'] <= a['gas'] for a,b in zip(samples,samples[1:]))
    assert all(b['battery'] <= a['battery'] for a,b in zip(samples,samples[1:]))
    assert s.drive_energy > 0
    assert s.suction.energy > 0


def test_mass_update_does_not_modify_trajectory():
    s = HalfpipeSimulation()
    s.data.qvel[:6] = [.1,.2,.3,.4,.5,.6]
    q,v = s.data.qpos.copy(),s.data.qvel.copy()
    s.gas.mass *= .7
    s.update_mass()
    np.testing.assert_array_equal(s.data.qpos,q)
    np.testing.assert_array_equal(s.data.qvel,v)


def test_restricted_allocation_and_saturation():
    s = HalfpipeSimulation()
    assert np.linalg.matrix_rank(s.controller.A) == 5
    for side in [4,5]:
        feasible=np.array([.3,.4,.5,.6,0,0,.2])
        feasible[side]=.15
        wrench=s.controller.A@feasible
        cmds = s.controller.allocate(wrench)
        force = np.array([cmds[t['name']]*t['max_thrust'] for t in s.c['thrusters']])
        np.testing.assert_allclose(s.controller.A@force,wrench,atol=1e-10)
        assert min(cmds.values()) >= 0
        assert max(cmds.values()) <= 1
    cmds = s.controller.allocate(np.array([0,0,100,0,0,0]))
    assert max(cmds.values()) <= 1
    assert s.allocation_error > 80
    commands=s.controller.allocate(np.array([0,0,-1.,0,0,0]))
    assert sum(commands.values())==0  # no top-mounted jet can supply negative body-z force
    s.controller.allocate(np.array([0,0,0,0,0,.02]))
    assert s.allocation_error > .3  # yaw necessarily creates lateral force
    s.gas.mass = s.gas.residual_mass
    assert sum(s.controller.allocate(np.ones(6)).values()) == 0
    assert s.allocation_error > 0


def test_only_requested_seven_nozzles():
    s=HalfpipeSimulation()
    t=s.c['thrusters']
    assert len(t)==7
    assert all(n['position'][2]<0 and n['direction']==[0,0,1] for n in t[:4])
    assert all(n['position'][0]>0 and n['position'][2]==0 for n in t[4:6])
    assert t[4]['direction']==[0,-1,0] and t[5]['direction']==[0,1,0]
    assert t[6]['position'][0]<0 and t[6]['direction']==[1,0,0]
    # At fully inverted attitude every available vertical force is downward.
    inverted=np.diag([-1.,1.,-1.])
    assert all((inverted@np.array(n['direction']))[2]<=0 for n in t)


def test_curved_seal_and_no_track_beyond_edge():
    s = HalfpipeSimulation()
    r = s.c['ramp']['radius']
    for angle in [.5,1.,2.,2.8]:
        n = np.array([-np.sin(angle),0,np.cos(angle)])
        s.data.qpos[:3] = [r*np.sin(angle),0,r*(1-np.cos(angle))]+.0232*n
        s.data.qpos[3:7] = [np.cos(angle/2),0,-np.sin(angle/2),0]
        mujoco.mj_forward(s.model,s.data)
        name,gap,direction = s.seal()
        assert name == 'ramp'
        assert direction@n < -.99
    s.data.qpos[:3] = [-.12,0,.88]
    s.data.qpos[3:7] = [0,0,-1,0]
    mujoco.mj_forward(s.model,s.data)
    assert s.seal()[0] == ''
    s.suction.pressure=350
    s.step(fan=1)
    assert s.suction.force == 0


def test_no_drive_force_without_wheel_contact():
    s = HalfpipeSimulation()
    s.data.qpos[:3]=[-.6,0,.5]
    mujoco.mj_forward(s.model,s.data)
    s.data.ctrl[:]=.01
    s.step()
    # Motor reaction rotates the chassis about the whole-system COM. Check total
    # momentum, not the chassis origin, which is offset from the wheel masses.
    mujoco.mj_subtreeVel(s.model,s.data)
    np.testing.assert_allclose(s.data.subtree_linvel[s.car],[0,0,-9.81*.001],atol=1e-8)


def test_exhausted_gas_cannot_report_success():
    s = HalfpipeSimulation()
    # An isolated failure experiment may initialize in mid-air.
    s.data.qpos[:3]=[-.65,0,.6]
    s.stage='depart'
    s.flight_start=0.
    s.flight_origin=s.data.qpos[:3].copy()
    s.gas.mass=s.gas.residual_mass
    s.update_mass()
    s.controller.launch_rotation=s.data.xmat[s.car].reshape(3,3).copy()
    for _ in range(500):
        s.advance()
    assert s.failure
    assert s.thrust.sum()==0
    assert s.landing_speed is not None and s.landing_speed>.15
