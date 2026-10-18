"""Render the continuous halfpipe demo at real-time speed; needs system ffmpeg."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import csv
import argparse
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from suction_thrust_lab.halfpipe import HalfpipeSimulation, draw_exhaust
from record_demos import timestamp


def side_camera(sim):
    camera=mujoco.MjvCamera()
    camera.distance=.29*sim.length_scale
    camera.azimuth,camera.elevation=90,0  # camera lies along y, outside the ramp's x-z plane
    camera.lookat[:]=sim.data.qpos[:3]
    return camera


def preview_visibility(sim,out,prefix):
    """Check real rendered pixels at five poses sampled from the prior physical run."""
    source=out/(prefix+'.csv')
    if not source.exists():
        source=out/(('full_size' if sim.c.get('vehicle')=='full_size' else 'halfpipe-roll')+'.csv')
    targets=[0,6,11.5,16,19.1] if sim.length_scale>1 else [0,3,7.5,11.5,14.3]
    poses={}
    with source.open() as stream:
        for row in csv.DictReader(stream):
            t=float(row['time_s'])
            for target in targets:
                if target not in poses or abs(t-target)<abs(float(poses[target]['time_s'])-target):
                    poses[target]=row
    frames=[]
    car_geoms=np.flatnonzero(sim.model.body_rootid[sim.model.geom_bodyid]==sim.car)
    counts=[]
    # Segmentation IDs must not be averaged across antialiased edges.
    sim.model.vis.quality.offsamples=0
    with mujoco.Renderer(sim.model,240,320) as renderer:
        for target in targets:
            row=poses[target]
            sim.data.qpos[:7]=[float(row[k]) for k in ['x_m','y_m','z_m','qw','qx','qy','qz']]
            mujoco.mj_forward(sim.model,sim.data)
            camera=side_camera(sim)
            renderer.update_scene(sim.data,camera=camera)
            frames.append(renderer.render().copy())
            renderer.enable_segmentation_rendering()
            segmentation=renderer.render()
            mask=(segmentation[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(segmentation[:,:,0],car_geoms)
            counts.append(int(mask.sum()))
            renderer.disable_segmentation_rendering()
    strip=np.concatenate(frames,axis=1)
    path=out/(prefix+'-camera-check.png')
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo',
        '-pix_fmt','rgb24','-s','1600x240','-i','-','-frames:v','1',str(path)],input=strip.tobytes(),check=True)
    print(f'{path}: visible car pixels at {targets}s = {counts}',flush=True)
    if min(counts)<1000:raise RuntimeError('Car is obscured or too small in the side camera')


def main():
    out=Path(__file__).resolve().parents[1]/'output'/'videos'/'suction-thrust-lab'
    out.mkdir(parents=True,exist_ok=True)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',choices=['halfpipe','full_size'],default='halfpipe')
    parser.add_argument('--preview',action='store_true',help='check the side camera against an existing simulation CSV')
    args=parser.parse_args()
    if args.demo=='full_size':
        from suction_thrust_lab.full_size import FullSizeSimulation
        s=FullSizeSimulation()
    else:
        s=HalfpipeSimulation()
    full_size=args.demo=='full_size'
    prefix=('full_size' if full_size else 'halfpipe-roll')+'-side'
    if args.preview:
        preview_visibility(s,out,prefix)
        return
    duration=40 if full_size else 24
    fps,width,height=30 if full_size else 60,960,720
    L=s.length_scale
    camera=mujoco.MjvCamera()
    camera.lookat[:]=np.array([-.12,0,.45])*L
    camera.distance=1.95*L
    camera.azimuth,camera.elevation=95,-7
    close=side_camera(s)
    front=mujoco.MjvCamera()
    front.distance,front.azimuth,front.elevation=.29*L,0,-8
    raw=out/(prefix+'-raw.mp4')
    proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo',
        '-pix_fmt','rgb24','-s',f'{width+320}x{height}','-r',str(fps),'-i','-',
        '-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(raw)],stdin=subprocess.PIPE)
    captions=[]
    with (out/(prefix+'.csv')).open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=s.row().keys())
        writer.writeheader()
        writer.writerow(s.row())
        try:
            with mujoco.Renderer(s.model,height,width) as renderer, mujoco.Renderer(s.model,240,320) as detail, mujoco.Renderer(s.model,200,320) as axial:
                for frame in range(duration*fps):
                    target=frame/fps
                    while s.data.time+1e-9<target:
                        s.advance()
                        writer.writerow(s.row())
                    renderer.update_scene(s.data,camera=camera)
                    draw_exhaust(s,renderer.scene)
                    pixels=np.full((height,width+320,3),24,dtype=np.uint8)
                    pixels[:,:width]=renderer.render()
                    close.lookat[:]=s.data.qpos[:3]
                    detail.update_scene(s.data,camera=close)
                    draw_exhaust(s,detail.scene)
                    inset=detail.render()
                    pixels[117:363,957:]=210
                    pixels[120:360,960:]=inset
                    if s.stage not in ['climb','landed']:
                        front.lookat[:]=s.data.qpos[:3]
                        axial.update_scene(s.data,camera=front)
                        draw_exhaust(s,axial.scene)
                        pixels[387:593,957:]=210
                        pixels[390:590,960:]=axial.render()
                    proc.stdin.write(pixels.tobytes())
                    if frame%2==0:
                        state={'climb':'SUCTION + WHEEL CLIMB','depart':'EDGE DEPARTURE',
                               'flip':'BOTTOM-JET ROLL 180','recover':'BRAKE + SLOW FLIGHT','descend':'CONTROLLED DESCENT','landed':'LANDED'}[s.stage]
                        row=s.row()
                        gas_text=f'{s.gas.used:.1f} kg' if full_size else f'{s.gas.used*1000:.1f} g'
                        suction_text=f'{s.suction.force/1000:.1f} kN' if full_size else f'{s.suction.force:.2f} N'
                        text=(f'{state}   t={target:.1f}s   z={s.data.qpos[2]:.3f} m\n'
                              f'Suction {suction_text} | Gas used {gas_text} | Roll {row["roll_from_launch_deg"]:.0f} deg')
                        captions.append(f'{len(captions)+1}\n{timestamp(target)} --> {timestamp(target+2/fps)}\n{text}\n')
                    if frame%(5*fps)==0:
                        print(f'Rendered {target:.0f}s: {s.stage}',flush=True)
        finally:
            proc.stdin.close()
    if proc.wait(): raise RuntimeError('ffmpeg failed')
    if s.stage!='landed' or s.failure:
        raise RuntimeError(f'Demo failed: {s.stage} {s.failure}')
    subtitle=out/(prefix+'.srt')
    subtitle.write_text('\n'.join(captions))
    title='SUCTION & THRUST LAB / FULL SIZE / 7 JETS' if full_size else 'SUCTION & THRUST LAB / MINI / ROLL 180'
    filters=(f"subtitles={subtitle}:force_style='FontSize=15,Outline=2,MarginV=15',"
             f"drawtext=text='{title}':x=24:y=18:fontsize=27:fontcolor=white:box=1:boxcolor=black@0.6,"
             "drawtext=text='4 underside + 2 front-side + 1 rear | Blue arrows = exhaust':x=24:y=55:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.6,"
             "drawtext=text='REAL TIME 1x':x=24:y=90:fontsize=20:fontcolor=yellow,"
             "drawtext=text='SIDE TRACKING VIEW':x=980:y=88:fontsize=20:fontcolor=white,"
             "drawtext=text='ROLL AXIS VIEW (AIRBORNE)':x=968:y=365:fontsize=17:fontcolor=white")
    realtime=out/(prefix+'-realtime.mp4')
    final=out/(prefix+'.mp4')
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(raw),'-vf',filters,
                    '-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(realtime)],check=True)
    raw.unlink()
    # Replay the entire airborne sequence through stable landing, with physical time captions.
    slow=out/(prefix+'-slow.mp4')
    replay_start=max(0.,s.flight_start-.2)
    replay_duration=min(duration,s.stage_time+.4)-replay_start
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss',str(replay_start),'-t',str(replay_duration),
        '-i',str(realtime),'-vf',"setpts=4*(PTS-STARTPTS),fps=60,drawbox=x=20:y=85:w=470:h=32:color=black:t=fill,drawtext=text='AIR CONTROL REPLAY 0.25x':x=24:y=90:fontsize=20:fontcolor=yellow",
        '-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(slow)],check=True)
    # Explicitly unify frame rates so the 30-fps full-size recording and 60-fps replay
    # concatenate without a timestamp gap. Playback speed changes only in the replay.
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(realtime),'-i',str(slow),
        '-filter_complex','[0:v]fps=60,setpts=PTS-STARTPTS[a];[1:v]fps=60,setpts=PTS-STARTPTS[b];[a][b]concat=n=2:v=1:a=0[out]',
        '-map','[out]','-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(final)],check=True)
    gas_text=f'{s.gas.used:.2f} kg' if full_size else f'{s.gas.used*1000:.2f} g'
    print(f'{final}\nTouchdown: {s.landing_speed:.4f} m/s; gas used: {gas_text}')


if __name__=='__main__':
    main()
