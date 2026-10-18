"""Run: python -m suction_thrust_lab --demo ceiling [--headless]."""
import argparse
import csv
from contextlib import nullcontext
from pathlib import Path
import time

import numpy as np

from .simulation import Simulation, controls, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', choices=['ceiling', 'lift', 'downforce', 'halfpipe', 'roadster'], default='ceiling')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--cold-fan', action='store_true', help='start ceiling fan at rest (expected seal failure)')
    parser.add_argument('--duration', type=float, help='default: 40 s roadster, 24 s halfpipe, otherwise 3 s')
    parser.add_argument('--fan-off', type=float, default=1.5)
    parser.add_argument('--burn', type=float, default=.3)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args()
    if args.duration is None:
        args.duration = 40. if args.demo == 'roadster' else 24. if args.demo == 'halfpipe' else 3.
    if not np.isfinite(args.duration) or args.duration <= 0 or args.burn < 0 or args.fan_off < 0:
        parser.error('duration must be positive; burn and fan-off must be nonnegative')
    if args.demo == 'roadster':
        from .roadster import RoadsterSimulation
        sim = RoadsterSimulation(load_config(args.config) if args.config else None)
    elif args.demo == 'halfpipe':
        from .halfpipe import HalfpipeSimulation
        sim = HalfpipeSimulation(load_config(args.config) if args.config else None)
    else:
        sim = Simulation(load_config(args.config), args.demo)
    if args.cold_fan:
        sim.suction.speed = 0.
    path = args.csv or Path('output') / (args.demo + '.csv')
    path.parent.mkdir(parents=True, exist_ok=True)
    viewer_context = nullcontext(None)
    if not args.headless:
        import mujoco.viewer
        viewer_context = mujoco.viewer.launch_passive(sim.model, sim.data)
    zmin = zmax = sim.data.qpos[2]
    lost_seal = False
    with viewer_context as viewer, path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=sim.row().keys())
        writer.writeheader()
        writer.writerow(sim.row())
        if viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = [0, 0, .5]
                viewer.cam.distance = 1.8
                viewer.cam.azimuth = 130
                viewer.cam.elevation = -15
                if args.demo in ['halfpipe','roadster']:
                    viewer.cam.lookat[:] = np.array([-.15, 0, .45])*sim.length_scale
                    viewer.cam.distance = 2.1*sim.length_scale
                    viewer.cam.azimuth = 100
                    viewer.cam.elevation = -12
        frame_steps = max(1, round(1 / 60 / sim.model.opt.timestep))
        wall_start = time.monotonic()
        for step in range(int(np.ceil(args.duration / sim.model.opt.timestep))):
            if viewer and not viewer.is_running():
                break
            with viewer.lock() if viewer else nullcontext():
                if args.demo in ['halfpipe','roadster']:
                    sim.advance()
                else:
                    sim.step(*controls(sim, args.demo, args.fan_off, args.burn))
                writer.writerow(sim.row())
                zmin, zmax = min(zmin, sim.data.qpos[2]), max(zmax, sim.data.qpos[2])
                if args.demo == 'ceiling' and .05 < sim.data.time < args.fan_off:
                    lost_seal |= sim.surface != 'ceiling'
            if viewer and step % frame_steps == 0:
                if args.demo in ['halfpipe','roadster']:
                    from .halfpipe import draw_exhaust
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        draw_exhaust(sim, viewer.user_scn)
                viewer.sync()
                time.sleep(max(0., sim.data.time - (time.monotonic() - wall_start)))
    gas_text=f'{sim.gas.used:.4f} kg' if args.demo=='roadster' else f'{sim.gas.used*1000:.4f} g'
    print(f'{args.demo}: z=[{zmin:.5f}, {zmax:.5f}] m, gas used={gas_text}; CSV: {path}')
    if args.demo in ['halfpipe','roadster']:
        print(f'Stage: {sim.stage}; touchdown speed: {sim.landing_speed}; failure: {sim.failure or "none"}')
        if sim.failure or sim.stage != 'landed':
            raise SystemExit('Halfpipe demo did not complete a verified soft landing.')
    if args.demo == 'lift' and zmax < .03:
        print('離陸条件を満たせませんでした。総推力、燃焼時間、重量、ガス圧を確認してください。')
    if lost_seal:
        print('ファン指令中に天井のシールを失いました。事前回転、隙間、流量・圧力と重量の条件を確認してください。')
    if args.demo == 'ceiling' and sim.suction.c['max_pressure'] * sim.suction.c['area'] <= sim.c['mass'] * sim.c['gravity']:
        print('最大吸着力が重量以下です。天井保持は成立しません。')


if __name__ == '__main__':
    main()
