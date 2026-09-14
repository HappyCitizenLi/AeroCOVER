#!/usr/bin/env python3
"""OPEN: fast rounded star, slow straight observer, and continuous rising circle."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from design_four_scenes import scene, waypoint
from design_four_scenes_v3 import high_speed_contract


def open_19m_scene():
    vertices=[]
    for i in range(10):
        angle=math.radians(126-36*i)
        scale=1. if i%2==0 else .15
        vertices.append(scale*np.array([27.5*math.cos(angle),2.5*math.sin(angle)]))
    vertices=np.array(vertices)
    # Quadratic corner cuts retain the five-point star without stopping
    # at every vertex. Arc-length retiming limits both acceleration terms.
    cut=.03
    incoming=vertices+cut*(np.roll(vertices,1,axis=0)-vertices)
    outgoing=vertices+cut*(np.roll(vertices,-1,axis=0)-vertices)
    points=[]
    for i in range(len(vertices)):
        for q in np.linspace(0,1,101,endpoint=False):
            points.append((1-q)**2*incoming[i]+2*q*(1-q)*vertices[i]+q*q*outgoing[i])
        for q in np.linspace(0,1,301,endpoint=False):
            points.append((1-q)*outgoing[i]+q*incoming[(i+1)%len(vertices)])
    outline=np.array(points+[points[0]])
    entry=np.linspace([25.,outline[0,1]],outline[0],2001)
    points=np.vstack((entry,outline[1:],outline[1:]))
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    curve=CubicSpline(s,points,bc_type='natural')
    grid=np.linspace(0,s[-1],int(math.ceil(s[-1]/.01))+1)
    curvature=np.linalg.norm(curve(grid,2),axis=1)
    speed=np.minimum(7.1,np.sqrt(1.5/np.maximum(curvature,1e-8)))
    speed[0]=speed[-1]=0.
    for i in range(1,len(grid)):
        speed[i]=min(speed[i],math.sqrt(speed[i-1]**2+2*2.7*(grid[i]-grid[i-1])))
    for i in range(len(grid)-2,-1,-1):
        speed[i]=min(speed[i],math.sqrt(speed[i+1]**2+2*2.7*(grid[i+1]-grid[i])))
    stamps=np.r_[0.,np.cumsum(2*np.diff(grid)/np.maximum(speed[:-1]+speed[1:],1e-8))]
    stamps/=1.025
    duration=round(float(stamps[-1]),6)
    times=np.linspace(0,duration,int(math.ceil(duration/.02))+1)
    progress=CubicSpline(stamps,grid,bc_type=((1,0.),(1,0.)))(times)
    star=curve(progress)+[0.,-11.1]
    star=gaussian_filter1d(star,.25/np.median(np.diff(times)),axis=0,mode='mirror')
    observer=[waypoint(t,1.+.45*p[0],0.,3.) for t,p in zip(times,star)]
    target_star=[waypoint(t,1.+p[0],p[1],6.) for t,p in zip(times,star)]
    target_circle=[]
    for t in times:
        q=t/duration
        turn=q-math.sin(2*math.pi*q)/(2*math.pi)
        angle=4*math.pi*turn
        height=.7+.3*(10*q**3-15*q**4+6*q**5)
        target_circle.append(waypoint(t,1.+4.8*math.cos(angle),7.8+4.8*math.sin(angle),3+height))
    config=high_speed_contract(scene('OPEN','PW_open',duration,observer,[target_circle,target_star]))
    config.update(seed=42,maximum_target_range_m=19.,minimum_observer_target_distance_m=2.,
                  purpose='Fast entry then rounded five-point outline; straight observer; rising circle',
                  geometry_design=dict(observer_star_x_ratio=.45,observer_center_x_m=1.,
                      helix_center_xy=[1.,7.8],helix_radius_m=4.8,helix_relative_height_m=[.7,1.],
                      star_center_xyz=[1.,-11.1,6.],star_points=5,
                      star_outer_radii_xy_m=[27.5,2.5],star_inner_fraction=.15,
                      star_path='outline_with_entry',star_entry_start_xy=[26.,float(outline[0,1]-11.1)],
                      star_corner_cut_fraction=cut,star_smoothing_s=.25,star_laps=2,
                      time_speedup=1.025,cruise_before_retiming_mps=7.1))
    return config


if __name__=='__main__':
    import argparse
    from pathlib import Path
    import yaml
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(yaml.safe_dump(open_19m_scene(),sort_keys=False))
