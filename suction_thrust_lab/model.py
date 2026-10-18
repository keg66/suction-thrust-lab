"""Separate visual geometry from the fixed collision/inertia approximation."""
from html import escape
import numpy as np


def visual_geoms():
    # Replace this function with mesh geoms/assets to install licensed STL/OBJ art.
    parts = ['<geom type="box" size=".060 .029 .008" pos="0 0 -.002" rgba=".92 .94 1 1"/>',
             '<geom type="ellipsoid" size=".027 .022 .012" pos="-.006 0 .008" rgba=".03 .18 .75 1"/>',
             '<geom type="box" size=".015 .022 .003" pos=".047 0 .007" euler="0 .15 0" rgba=".04 .28 .95 1"/>',
             '<geom type="box" size=".009 .041 .003" pos="-.064 0 .0175" rgba=".06 .25 .9 1"/>']
    for y in [-.019, .019]:
        parts.append(f'<geom type="box" size=".047 .003 .001" pos=".011 {y} .008" rgba="1 .12 .05 1"/>')
    return '\n'.join(parts)


def make_xml(c):
    length, width, height = c['dimensions']
    inertia = c['mass'] / 12 * np.array([width**2 + height**2, length**2 + height**2, length**2 + width**2])
    wheels = []
    for x in [-.048, .048]:
        for y in [-.0355, .0355]:
            wheels.append(f'<geom name="wheel_{x}_{y}" type="cylinder" size=".013 .0095" pos="{x} {y} -.0075" euler="1.57079632679 0 0" rgba=".06 .06 .07 1"/>')
    sites = '\n'.join(f'<site name="{escape(t["name"], quote=True)}" pos="{" ".join(map(str,t["position"]))}" size=".002" rgba="1 .4 0 1"/>' for t in c['thrusters'])
    return f'''<mujoco model="Suction &amp; Thrust Lab">
    <compiler angle="radian"/>
    <option timestep="{c['timestep']}" gravity="0 0 -{c['gravity']}" integrator="implicitfast"/>
    <visual><headlight ambient=".5 .5 .5"/><global offwidth="960" offheight="720"/></visual>
    <default><geom friction=".8 .005 .0001" solref=".003 1" solimp=".95 .99 .001"/></default>
    <worldbody>
      <light pos="0 -1 2"/>
      <camera name="overview" pos=".9 -1.4 .8" xyaxes=".84 .54 0 -.16 .25 .95"/>
      <geom name="floor" type="box" pos="0 0 -.025" size="1 1 .025" rgba=".3 .35 .4 1"/>
      <geom name="ceiling" type="box" pos="0 0 {c['ceiling_height']+.025}" size="1 1 .025" rgba=".5 .65 .75 .35"/>
      <geom name="wall" type="box" pos="1.025 0 {c['ceiling_height']/2}" size=".025 1 {c['ceiling_height']/2}" rgba=".4 .5 .6 .4"/>
      <body name="car" pos="0 0 .0205">
        <freejoint/>
        <inertial pos="0 0 0" mass="{c['mass']}" diaginertia="{' '.join(map(str, inertia))}"/>
        <geom name="chassis" type="box" size=".060 .029 .008" pos="0 0 -.002" rgba=".9 .9 .95 1"/>
        {' '.join(wheels)}
        <body name="visuals"><geom type="sphere" size=".0001" contype="0" conaffinity="0" mass="0" rgba="0 0 0 0"/>
        {visual_geoms().replace('<geom ', '<geom contype="0" conaffinity="0" mass="0" ')}
        </body>
        {sites}
      </body>
    </worldbody></mujoco>'''
