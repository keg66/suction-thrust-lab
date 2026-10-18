"""Render the continuous halfpipe demo at real-time speed; needs system ffmpeg."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import csv
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from suction_thrust_lab.halfpipe import HalfpipeSimulation, draw_exhaust
from record_demos import timestamp


def main():
    out=Path(__file__).resolve().parents[1]/'output'/'videos'
    out.mkdir(parents=True,exist_ok=True)
    s=HalfpipeSimulation()
    fps,width,height=60,960,720
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[-.12,0,.45]
    camera.distance=1.95
    camera.azimuth,camera.elevation=95,-7
    close=mujoco.MjvCamera()
    close.distance,close.azimuth,close.elevation=.29,145,-12
    raw=out/'halfpipe-roll-raw.mp4'
    proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo',
        '-pix_fmt','rgb24','-s',f'{width+320}x{height}','-r',str(fps),'-i','-',
        '-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(raw)],stdin=subprocess.PIPE)
    captions=[]
    with (out/'halfpipe-roll.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=s.row().keys())
        writer.writeheader()
        writer.writerow(s.row())
        try:
            with mujoco.Renderer(s.model,height,width) as renderer:
                for frame in range(24*fps):
                    target=frame/fps
                    while s.data.time+1e-9<target:
                        s.advance()
                        writer.writerow(s.row())
                    renderer.update_scene(s.data,camera=camera)
                    draw_exhaust(s,renderer.scene)
                    pixels=np.full((height,width+320,3),24,dtype=np.uint8)
                    pixels[:,:width]=renderer.render()
                    close.lookat[:]=s.data.qpos[:3]
                    renderer.update_scene(s.data,camera=close)
                    draw_exhaust(s,renderer.scene)
                    inset=renderer.render()[::3,::3]
                    pixels[117:363,957:]=210
                    pixels[120:360,960:]=inset
                    proc.stdin.write(pixels.tobytes())
                    if frame%2==0:
                        state={'climb':'SUCTION + WHEEL CLIMB','depart':'EDGE DEPARTURE',
                               'flip':'BOTTOM-JET ROLL 180','recover':'BRAKE + SLOW FLIGHT','descend':'CONTROLLED DESCENT','landed':'LANDED'}[s.stage]
                        row=s.row()
                        text=(f'{state}   t={target:.1f}s   z={s.data.qpos[2]:.3f} m\n'
                              f'Suction {s.suction.force:.2f} N | Gas used {s.gas.used*1000:.1f} g | Roll {row["roll_from_launch_deg"]:.0f} deg')
                        captions.append(f'{len(captions)+1}\n{timestamp(target)} --> {timestamp(target+2/fps)}\n{text}\n')
                    if frame%(5*fps)==0:
                        print(f'Rendered {target:.0f}s: {s.stage}',flush=True)
        finally:
            proc.stdin.close()
    if proc.wait(): raise RuntimeError('ffmpeg failed')
    if s.stage!='landed' or s.failure:
        raise RuntimeError(f'Demo failed: {s.stage} {s.failure}')
    subtitle=out/'halfpipe-roll.srt'
    subtitle.write_text('\n'.join(captions))
    filters=(f"subtitles={subtitle}:force_style='FontSize=15,Outline=2,MarginV=15',"
             "drawtext=text='SUCTION & THRUST LAB / 7 JETS / ROLL 180':x=24:y=18:fontsize=27:fontcolor=white:box=1:boxcolor=black@0.6,"
             "drawtext=text='4 underside + 2 front-side + 1 rear | Blue arrows = exhaust':x=24:y=55:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.6,"
             "drawtext=text='REAL TIME 1x':x=24:y=90:fontsize=20:fontcolor=yellow,"
             "drawtext=text='TRACKING CLOSE-UP':x=980:y=88:fontsize=20:fontcolor=white")
    final=out/'halfpipe-roll.mp4'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(raw),'-vf',filters,
                    '-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(final)],check=True)
    raw.unlink()
    # Replay the brief gravity-driven roll without changing the physical trajectory.
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss','14.4','-t','1.5',
        '-i',str(final),'-vf',"setpts=4*(PTS-STARTPTS),fps=60,drawbox=x=20:y=85:w=300:h=32:color=black:t=fill,drawtext=text='SLOW REPLAY 0.25x':x=24:y=90:fontsize=20:fontcolor=yellow",
        '-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'halfpipe-roll-slow.mp4')],check=True)
    print(f'{final}\nTouchdown: {s.landing_speed:.4f} m/s; gas used: {s.gas.used*1000:.2f} g')


if __name__=='__main__':
    main()
