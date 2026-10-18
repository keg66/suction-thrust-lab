"""Record the unchanged Phase 1 physics using MuJoCo EGL and system ffmpeg.
Run from the project root: MUJOCO_GL=egl .venv/bin/python scripts/record_demos.py
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from suction_thrust_lab.simulation import Simulation, controls

OUT = Path(__file__).resolve().parents[1] / 'output' / 'videos' / 'suction-thrust-lab'
FPS = 60
WIDTH, HEIGHT = 960, 720


def timestamp(seconds):
    ms = round(seconds * 1000)
    return f'{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}'


def record(demo):
    sim = Simulation(demo=demo)
    sim.model.vis.global_.offwidth = WIDTH
    sim.model.vis.global_.offheight = HEIGHT
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, .5 if demo == 'ceiling' else .12]
    cam.distance = 1.6 if demo == 'ceiling' else .65
    cam.azimuth, cam.elevation = 100, -8 if demo == 'ceiling' else -18
    close = mujoco.MjvCamera()
    close.distance, close.azimuth, close.elevation = .32, 130, 20
    raw = OUT / f'{demo}-raw.mp4'
    proc = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}', '-r', str(FPS),
        '-i', '-', '-an', '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p', str(raw)], stdin=subprocess.PIPE)
    captions = []
    try:
        with mujoco.Renderer(sim.model, HEIGHT, WIDTH) as renderer:
            for frame in range(3 * FPS):
                target = frame / FPS
                while sim.data.time + 1e-9 < target:
                    sim.step(*controls(sim, demo))
                renderer.update_scene(sim.data, camera=cam)
                pixels = renderer.render().copy()
                if demo == 'ceiling':
                    close.lookat[:] = sim.data.qpos[:3]
                    renderer.update_scene(sim.data, camera=close)
                    inset = renderer.render()[::3, ::3]
                    pixels[87:333, 627:953] = 220
                    pixels[90:330, 630:950] = inset
                proc.stdin.write(pixels.tobytes())
                if frame % 6 == 0:
                    state = ('FAN ON / CEILING HOLD' if target < 1.5 else 'FAN OFF / FALL') if demo == 'ceiling' else ('THRUST ON' if sim.thrust.sum() > 0 else 'THRUST OFF')
                    text = f'{state}   t={target:.1f}s   height={sim.data.qpos[2]*1000:.1f} mm\nSuction={sim.suction.force:.2f} N   Thrust={sim.thrust.sum():.2f} N   Gas used={sim.gas.used*1000:.2f} g'
                    captions.append(f'{len(captions)+1}\n{timestamp(target)} --> {timestamp(target+.1)}\n{text}\n')
    finally:
        proc.stdin.close()
    if proc.wait():
        raise RuntimeError('ffmpeg encoding failed')
    subtitle = OUT / f'{demo}.srt'
    subtitle.write_text('\n'.join(captions))
    title = 'SUCTION & THRUST LAB / CEILING SUCTION' if demo == 'ceiling' else 'SUCTION & THRUST LAB / COLD GAS LIFT'
    note = 'Starts inverted, fan pre-spun | inset follows car' if demo == 'ceiling' else '0.3 s burn | finite gas | no position animation'
    # Caption physical time first, then duplicate frames for a clearly labelled slow replay.
    filters = (f"[0:v]subtitles={subtitle}:force_style='FontSize=16,Outline=2,MarginV=14',"
               f"drawtext=text='{title}':x=24:y=20:fontsize=27:fontcolor=white:box=1:boxcolor=black@0.6,"
               f"drawtext=text='{note}':x=24:y=56:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.6,split[a][b];"
               "[a]drawtext=text='REAL TIME 1x':x=24:y=100:fontsize=22:fontcolor=yellow[normal];"
               "[b]setpts=4*(PTS-STARTPTS),fps=60,drawtext=text='SLOW REPLAY 0.25x':x=24:y=100:fontsize=22:fontcolor=yellow[slow];"
               "[normal][slow]concat=n=2:v=1:a=0[out]")
    final = OUT / f'{demo}.mp4'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(raw),
                    '-filter_complex',filters,'-map','[out]','-c:v','libx264','-crf','20',
                    '-pix_fmt','yuv420p','-movflags','+faststart',str(final)], check=True)
    raw.unlink()
    print(final, flush=True)


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ['ceiling', 'lift']:
        record(name)
    listing = OUT / 'concat.txt'
    listing.write_text("file 'ceiling.mp4'\nfile 'lift.mp4'\n")
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0',
                    '-i',str(listing),'-c','copy','-movflags','+faststart',str(OUT/'suction-thrust-lab-demos.mp4')], check=True)
