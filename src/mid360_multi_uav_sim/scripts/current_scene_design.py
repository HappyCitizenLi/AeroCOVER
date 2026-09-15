#!/usr/bin/env python3
"""Current MT and flowing OFFICE/FOREST construction helpers."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from design_four_scenes import ROOT, scene, waypoint

def high_speed_contract(config):
    config.update(trajectory_execution='mrs_trajectory', speed_reference_mps=8.,
                  planned_attitude_from_acceleration=True,
                  geometry_sample_period_s=.02,
                  maximum_truth_gap_s=.05,
                  required_peak_speed_mps=7.2 if config['scenario_id'] in ('OPEN','MT') else 4.,
                  maximum_allowed_speed_mps=8., maximum_allowed_acceleration_mps2=4.,
                  maximum_target_range_m=20., maximum_mutual_occlusion_s=1.)
    return config

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
    # The single moving target uav4 follows the 16 m periodic orbit.
    offset = -.15
    targets.append([waypoint(t,16*math.cos(angle(t)+offset),16*math.sin(angle(t)+offset),3.9) for t in times])
    result=scene('MT','PW_mt',duration,observer,targets)
    result.update(allow_static_occlusion_targets=['uav2','uav3'],moving_target_ids=['uav4'],
                  los_only_target_ids=['uav2','uav3'],
                  maximum_moving_mutual_occlusion_s=1.,
                  geometry_design=dict(hover_body_clearance_m=1.5,observer_radius_m=12.,
                                       observer_midcourse_dip_m=.25,formation_radius_m=16.))
    return high_speed_contract(result)

def chase_scene(name, route, speed_limit=None, acceleration_limit=None,
                deceleration_limit=None, entry_extension_m=0., straight_gap_m=None,
                flowing=False):
    states=np.load(route)
    if straight_gap_m is not None:
        states=states.copy()
        states[:,3]=np.minimum(states[:,3],straight_gap_m)
    if entry_extension_m:
        first=states[0].copy()
        first[:2]-=entry_extension_m*np.array([math.cos(first[2]),math.sin(first[2])])
        states=np.vstack((np.linspace(first,states[0],int(entry_extension_m/.5)+1)[:-1],states))
    length=states[:,3] if states.shape[1]>3 else np.full(len(states),2.6 if name=='OFFICE' else 4.5)
    delta=length[:,None]/2*np.column_stack([np.cos(states[:,2]),np.sin(states[:,2])])
    points=np.column_stack((states[:,:2]-delta,states[:,:2]+delta))
    steps=np.maximum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1),
                     np.linalg.norm(np.diff(points[:,2:],axis=0),axis=1))
    keep=np.r_[True,steps>1e-6]
    points=points[keep]
    ds=np.maximum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1),np.linalg.norm(np.diff(points[:,2:],axis=0),axis=1))
    s=np.r_[0.,np.cumsum(ds)]
    spline=CubicSpline(s,points,bc_type='natural')
    grid=np.linspace(0,s[-1],int(math.ceil(s[-1]/.04))+1)
    derivative=spline(grid,2)
    curvature=np.maximum(np.linalg.norm(derivative[:,:2],axis=1),np.linalg.norm(derivative[:,2:],axis=1))
    normal_limit=.32 if name=='OFFICE' else .55
    tangent_limit=.20 if name=='OFFICE' else .25
    cruise=.65+.65*np.sin(np.pi*grid/s[-1])**2
    if speed_limit is not None: cruise=np.full_like(grid,speed_limit)
    if acceleration_limit is not None: tangent_limit=acceleration_limit
    acceleration=np.full_like(grid,tangent_limit)
    deceleration=np.full_like(grid,deceleration_limit or tangent_limit)
    if straight_gap_m is not None:
        slow=np.mean(spline(grid)[:,[0,2]],axis=1)>8
        cruise[slow]=1.4 if flowing else .8
        acceleration[slow]=deceleration[slow]=.16 if flowing else .04
        normal_limit=np.where(slow,.25 if flowing else .08,normal_limit)
    elif entry_extension_m:
        slow=np.mean(spline(grid)[:,[0,2]],axis=1)>=-4
        cruise[slow]=((1.5+.5*np.sin(np.pi*grid/s[-1])**2) if flowing else
                      (.65+.65*np.sin(np.pi*grid/s[-1])**2))[slow]
        acceleration[slow]=deceleration[slow]=.3 if flowing else .15
        normal_limit=np.where(slow,.55 if flowing else .35,normal_limit)
    velocity=np.minimum(cruise,np.sqrt(normal_limit/np.maximum(curvature,1e-6)))
    velocity[0]=velocity[-1]=0
    for i in range(1,len(grid)):
        velocity[i]=min(velocity[i],math.sqrt(velocity[i-1]**2+2*acceleration[i]*(grid[i]-grid[i-1])))
    for i in range(len(grid)-2,-1,-1):
        velocity[i]=min(velocity[i],math.sqrt(velocity[i+1]**2+2*deceleration[i]*(grid[i+1]-grid[i])))
    timestamps=np.r_[0.,np.cumsum(2*np.diff(grid)/np.maximum(velocity[:-1]+velocity[1:],1e-5))]
    # Narrow office turns leave little elevation/clearance reserve. Retiming
    # preserves the route and gap schedule while reducing bank-angle demand.
    if name=='OFFICE' and speed_limit is None: timestamps *= 1.5
    duration=round(float(timestamps[-1]+5),6)
    sample_t=np.linspace(0,duration,int(math.ceil(duration/(.2 if speed_limit is None else .02)))+1)
    progress=(np.interp(sample_t,timestamps,grid) if speed_limit is None else
              CubicSpline(timestamps,grid,bc_type=((1,0.),(1,0.)))(np.minimum(sample_t,timestamps[-1])))
    positions=spline(progress)
    observer_z,target_z=(.8,1.) if name=='OFFICE' else (2.,2.65)
    observer=[waypoint(t,*p[:2],observer_z) for t,p in zip(sample_t,positions)]
    target=[waypoint(t,*p[2:],target_z) for t,p in zip(sample_t,positions)]
    result=scene(name,'PW_office' if name=='OFFICE' else 'PW_forest_seed0',duration,observer,[target])
    if name=='OFFICE': result['minimum_observer_target_distance_m']=2.
    result['purpose']='Joint visible-pair path through corridors' if name=='OFFICE' else 'Full west-to-east forest traverse with near-obstacle passes'
    return result

def chase_scenes():
    routes=ROOT/'config/routes'
    office=chase_scene('OFFICE',routes/'office.npy',speed_limit=4.6,
                      acceleration_limit=1.6,deceleration_limit=.7,
                      entry_extension_m=4.,straight_gap_m=4.,flowing=True)
    forest=chase_scene('FOREST',routes/'forest.npy',speed_limit=4.6,
                      acceleration_limit=1.2,deceleration_limit=1.2,
                      entry_extension_m=18.,flowing=True)
    return dict(MT=mt_scene(),
                OFFICE=high_speed_contract(office),FOREST=high_speed_contract(forest))
