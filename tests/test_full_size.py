"""Full-size conditions and scale-independent gravity, experimental parameters only."""
import mujoco
import numpy as np
import pytest
from suction_thrust_lab.halfpipe import HalfpipeSimulation
from suction_thrust_lab.full_size import FullSizeSimulation, full_size_config


@pytest.fixture(scope='module')
def full_size_run():
    s=FullSizeSimulation()
    stages=[]
    min_clearance=float('inf')
    for i in range(35000):
        before=s.data.qpos[:3].copy()
        s.advance()
        np.testing.assert_allclose(s.data.qpos[:3]-before,s.data.qvel[:3]*.001,atol=1e-9)
        if not stages or stages[-1]!=s.stage:stages.append(s.stage)
        if s.stage in ['flip','recover']:min_clearance=min(min_clearance,s.data.qpos[2])
    return s,stages,min_clearance


def test_full_size_continuous_roll_and_landing(full_size_run):
    s,stages,minimum=full_size_run
    assert stages==['climb','depart','flip','recover','descend','landed']
    assert not s.failure
    assert 0<s.landing_speed<.5
    assert s.max_touchdown_speed<.5
    assert 6<minimum<10  # metres, rather than the small model's ~0.16 m
    assert 26<s.flight_origin[2]<28
    assert abs(s.row()['roll_from_launch_deg'])>170
    np.testing.assert_allclose(s.data.qpos[:3],[-.65*s.length_scale,0,.63],atol=.05)
    assert np.linalg.norm(s.data.qvel[:6])<.03


def test_full_size_finite_resource_and_mass_accounting(full_size_run):
    s,_,_=full_size_run
    assert 400<s.gas.used<470
    assert s.gas.mass>s.gas.residual_mass
    assert s.model.body_mass.sum()==pytest.approx(s.total_mass)
    assert s.total_mass>1600
    assert 0<s.drive_energy<3e6
    assert s.suction.energy>0
    assert s.model.opt.gravity[2]==-9.81


def test_real_dimensions_nozzle_positions_and_rank():
    s=FullSizeSimulation()
    np.testing.assert_allclose(s.c['dimensions'],[4.536,1.971,1.26])
    assert s.model.body_mass.sum()==pytest.approx(2082.720225,rel=1e-8)
    assert len(s.c['thrusters'])==7
    assert np.linalg.matrix_rank(s.controller.A)==5
    left=s.c['thrusters'][4]
    assert left['position'][0]>1.5 and left['position'][1]>.7
    assert all(n['direction'][2]>=0 for n in s.c['thrusters'])
    assert s.model.body_inertia[s.car][0]>500


def test_free_fall_does_not_depend_on_vehicle_mass_or_size():
    drops=[]
    for s in [HalfpipeSimulation(),FullSizeSimulation()]:
        s.data.qpos[:3]=[-30,0,40]
        mujoco.mj_forward(s.model,s.data)
        for _ in range(250):s.step()
        drops.append(40-s.data.qpos[2])
        assert s.data.qvel[2]==pytest.approx(-9.81*.25,abs=1e-9)
    assert drops[0]==pytest.approx(drops[1],abs=1e-9)
    assert drops[0]==pytest.approx(.5*9.81*.25**2,abs=.002)
