#!/usr/bin/env python3
"""High-speed scene revision. Speed reference is MRS fast horizontal 8 m/s."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from design_four_scenes import ROOT, scene, waypoint
from design_four_scenes_v2 import chase_scene


def high_speed_contract(config):
    config.update(trajectory_execution='mrs_trajectory', speed_reference_mps=8.,
                  planned_attitude_from_acceleration=True,
                  geometry_sample_period_s=.02,
                  maximum_truth_gap_s=.05,
                  required_peak_speed_mps=7.2 if config['scenario_id'] in ('OPEN','MT') else 4.,
                  maximum_allowed_speed_mps=8., maximum_allowed_acceleration_mps2=4.,
                  maximum_target_range_m=20., maximum_mutual_occlusion_s=1.)
    return config


def shuttle_profile(amplitude=23.8):
    # Reduce acceleration while the star is near the forward field edge;
    # attain the peak near the line midpoint instead. No attitude clamp.
    x=np.linspace(-amplitude,amplitude,int(math.ceil(2*amplitude/.01))+1)
    def smooth(q):
        q=np.clip(q,0,1)
        return 10*q**3-15*q**4+6*q**5
    a=1.5-.6*smooth((x+17)/2)+.6*smooth((x+2)/2)
    braking=1.4-.4*smooth((x-21)/2)
    v=np.full_like(x,7.85)
    v[0]=v[-1]=0.
    for i in range(1,len(x)):
        v[i]=min(v[i],math.sqrt(v[i-1]**2+2*a[i]*(x[i]-x[i-1])))
    for i in range(len(x)-2,-1,-1):
        v[i]=min(v[i],math.sqrt(v[i+1]**2+2*braking[i]*(x[i+1]-x[i])))
    t=np.r_[0.,np.cumsum(2*np.diff(x)/np.maximum(v[:-1]+v[1:],1e-9))]
    duration=float(t[-1])
    return CubicSpline(t,x,bc_type=((1,0.),(1,0.))),duration


def open_scene():
    amplitude=23.8
    shuttle,leg=shuttle_profile(amplitude)
    return_leg=80.
    period=leg+return_leg
    duration=round(2*period,6)
    vertices=[]
    for i in range(11):
        angle=math.radians(126-36*i)
        radius=9.4 if i%2==0 else .15
        vertices.append([radius*math.cos(angle),radius*math.sin(angle)])
    vertices[-1]=vertices[0]
    vertices=np.asarray(vertices)
    return_x=np.array([amplitude,16.,12.,6.,0.,-6.,-12.,-16.,-amplitude])
    return_segments=len(vertices)-3
    def blend(q):
        q=np.clip(q,0,1)
        return 10*q**3-15*q**4+6*q**5
    observer,helix,star_path=[],[],[]
    for t in np.linspace(0,duration,int(math.ceil(duration/.02))+1):
        phase=min(t,duration-1e-9)%period
        outward=phase<=leg
        if outward:
            x=float(shuttle(phase))
            q=phase/leg
            if q<=.4:
                b=blend(q/.4);xy=(1-b)*vertices[0]+b*vertices[1]
            elif q<.6: xy=vertices[1]
            else:
                b=blend((q-.6)/.4);xy=(1-b)*vertices[1]+b*vertices[2]
            turn=q-math.sin(2*math.pi*q)/(2*math.pi)
            beta=math.atan2(9.,amplitude)
            angle=-math.pi/2-beta+(math.pi+2*beta)*turn
            hx,hy=6*math.sin(angle),9+6*math.cos(angle)
        else:
            q=(phase-leg)/return_leg*return_segments
            index=min(int(q),return_segments-1);b=blend(q-index)
            x=(1-b)*return_x[index]+b*return_x[index+1]
            xy=(1-b)*vertices[index+2]+b*vertices[index+3]
            norm=math.hypot(x,9.)
            hx,hy=6*x/norm,9-54/norm
        q=t/duration
        height=.4+.6*(10*q**3-15*q**4+6*q**5)
        observer.append(waypoint(t,x,0,3.))
        helix.append(waypoint(t,hx,hy,3+height))
        star_path.append(waypoint(t,xy[0],-10+xy[1],6.))
    result=scene('OPEN','PW_open',duration,observer,[helix,star_path])
    result.update(seed=42,minimum_observer_target_distance_m=2.,
                  purpose='Straight shuttle observer; radius-6 rising helix and five-point star with smooth stops at vertices',
                  geometry_design=dict(observer_half_length_m=amplitude,helix_center_xy=[0.,9.],helix_radius_m=6.,
                                       helix_relative_height_m=[.4,1.],
                                       star_center_xyz=[0.,-10.,6.],star_points=5,
                                       star_outer_radius_m=9.4,star_inner_radius_m=.15))
    return high_speed_contract(result)


def mt_scene():
    duration=52.
    times=np.linspace(0,duration,521)
    def angle(t):
        q=t/duration
        return 4*math.pi*(q-math.sin(2*math.pi*q)/(2*math.pi))
    # Preserve startup shell margin, then compensate the fast-turn sensor tilt.
    observer=[waypoint(t,12*math.cos(angle(t)-.1),12*math.sin(angle(t)-.1),
                       3.1-.25*math.sin(math.pi*t/duration)**2) for t in times]
    # Requested clearance is from the body envelope to the pillar surface.
    radius=1.2+.43+1.5
    targets=[[waypoint(t,radius,0,2.7) for t in (0,duration)],
             [waypoint(t,-radius,0,3.8) for t in (0,duration)]]
    # Keep the original uav4 path; remove uav5/uav6 without retuning geometry.
    offset = -.15
    targets.append([waypoint(t,16*math.cos(angle(t)+offset),16*math.sin(angle(t)+offset),3.9) for t in times])
    result=scene('MT','PW_mt',duration,observer,targets)
    result.update(allow_static_occlusion_targets=['uav2','uav3'],moving_target_ids=['uav4'],
                  los_only_target_ids=['uav2','uav3'],
                  maximum_moving_mutual_occlusion_s=1.,
                  geometry_design=dict(hover_body_clearance_m=1.5,observer_radius_m=12.,
                                       observer_midcourse_dip_m=.25,formation_radius_m=16.))
    return high_speed_contract(result)


def designs_v3():
    routes=ROOT/'config/routes'
    office=chase_scene('OFFICE',routes/'office.npy',speed_limit=4.6,
                       acceleration_limit=1.6,deceleration_limit=.7,entry_extension_m=4.,straight_gap_m=4.)
    forest=chase_scene('FOREST',routes/'forest.npy',speed_limit=4.6,
                       acceleration_limit=1.2,deceleration_limit=1.2,entry_extension_m=18.)
    return dict(OPEN=open_scene(),MT=mt_scene(),OFFICE=high_speed_contract(office),FOREST=high_speed_contract(forest))


if __name__=='__main__':
    import argparse,yaml
    from pathlib import Path
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    for name,config in designs_v3().items():
        (args.output/(name+'.yaml')).write_text(yaml.safe_dump(config,sort_keys=False))
        print(name,config['duration_s'],flush=True)
