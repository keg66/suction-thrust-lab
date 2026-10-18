"""Render the miniature tornado demo with tracking and roll-axis cameras.

Run: MUJOCO_GL=egl python scripts/record_tornado.py [--demo tornado_full_size]
Requires system ffmpeg. Output includes real time and a 0.25x flight replay.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np

from suction_thrust_lab.tornado import TornadoSimulation, race_camera
from suction_thrust_lab.halfpipe import draw_exhaust
from record_demos import timestamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', choices=['tornado', 'tornado_full_size'], default='tornado')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    args.output = args.output or Path('output/videos/suction-thrust-lab')/(args.demo+'.mp4')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    raw = args.output.with_name(args.output.stem+'-raw.mp4')
    subtitle = args.output.with_suffix('.srt')
    if args.demo == 'tornado_full_size':
        from suction_thrust_lab.tornado_full_size import FullSizeTornadoSimulation
        sim = FullSizeTornadoSimulation()
    else:
        sim = TornadoSimulation()
    scale = sim.settings['race_scale']
    title = (f'FULL SIZE TORNADO / 4.54 m / {sim.settings["turns"]} ROLLS'
             if args.demo == 'tornado_full_size' else 'MINI TORNADO / LAST TO FIRST / 14 ROLLS')
    follow, axial, overview = [mujoco.MjvCamera() for _ in range(3)]
    follow.distance, follow.azimuth, follow.elevation = .65, 110, -18
    axial.distance, axial.azimuth, axial.elevation = .30*sim.length_scale, 0, -8
    overview.distance, overview.azimuth, overview.elevation = 7., 90, -45
    fps, duration = 60, 6
    labels = dict(accelerate='FULL THROTTLE', cruise='TOP SPEED', lift='LIFT OFF',
                  roll=f'TORNADO / {sim.settings["turns"]} ROLLS',
                  descend='ROLLING TOUCHDOWN', landed='LEADER / FULL THROTTLE')
    captions = []
    proc = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x720', '-r', str(fps),
        '-i', '-', '-an', '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p', str(raw)], stdin=subprocess.PIPE)
    try:
        with mujoco.Renderer(sim.model, 720, 960) as renderer, mujoco.Renderer(sim.model, 240, 320) as detail:
            for frame in range(duration*fps):
                target = frame/fps
                while sim.data.time+1e-9 < target:
                    sim.advance()
                race_camera(sim, follow)
                axial.lookat[:] = sim.data.qpos[:3]
                field_x = np.r_[sim.data.qpos[0], sim.data.xpos[sim.rival_bodies, 0]]
                overview.lookat[:] = [(field_x.min()+field_x.max())/2, 0, .15*scale]
                overview.distance = max(4.5*scale, float(np.ptp(field_x))*1.5+scale)
                renderer.update_scene(sim.data, camera=follow)
                draw_exhaust(sim, renderer.scene)
                pixels = np.full((720, 1280, 3), 24, dtype=np.uint8)
                pixels[:, :960] = renderer.render()
                for camera, start in [(axial, 120), (overview, 430)]:
                    detail.update_scene(sim.data, camera=camera)
                    draw_exhaust(sim, detail.scene)
                    pixels[start:start+240, 960:] = detail.render()
                proc.stdin.write(pixels.tobytes())
                if frame % 2 == 0:
                    text = (f'{labels[sim.stage]} | POSITION {sim.race_position}/7 | PASSED {sim.overtaken}/6\n'
                            f't={target:.2f}s | Speed={sim.data.qvel[0]:.2f} m/s | Roll={np.degrees(sim.roll_angle):.0f} deg')
                    captions.append(f'{len(captions)+1}\n{timestamp(target)} --> {timestamp(target+2/fps)}\n{text}\n')
    finally:
        proc.stdin.close()
        code = proc.wait()
    if code:
        raise RuntimeError('ffmpeg failed')
    if sim.failure or sim.stage != 'landed' or sim.race_position != 1 or sim.overtaken != 6:
        raise RuntimeError(f'Demo failed: {sim.stage}: {sim.failure}')
    subtitle.write_text('\n'.join(captions))
    filters = (f"subtitles={subtitle}:force_style='FontSize=16,Outline=2,MarginV=12',"
        f"drawtext=text='{title}':x=24:y=20:fontsize=26:fontcolor=white,"
        "drawtext=text='REAL TIME 1x':x=24:y=60:fontsize=20:fontcolor=yellow,"
        "drawtext=text='ROLL AXIS':x=980:y=88:fontsize=20:fontcolor=white,"
        "drawtext=text='RACE OVERVIEW':x=980:y=398:fontsize=20:fontcolor=white,split[a][b];"
        "[a]setpts=PTS-STARTPTS[normal];"
        f"[b]trim=start={sim.flight_start-.15}:end={sim.touchdown_time+.25},setpts=4*(PTS-STARTPTS),fps=60,"
        "drawbox=x=20:y=55:w=430:h=32:color=black:t=fill,"
        "drawtext=text='FLIGHT REPLAY 0.25x':x=24:y=60:fontsize=20:fontcolor=yellow[slow];"
        "[normal][slow]concat=n=2:v=1:a=0[out]")
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(raw),
        '-filter_complex', filters, '-map', '[out]', '-c:v', 'libx264', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(args.output)], check=True)
    raw.unlink()
    print(f'{args.output}\nVertical touchdown: {sim.landing_speed:.4f} m/s; forward: {sim.touchdown_forward_speed:.2f} m/s')


if __name__ == '__main__':
    main()
