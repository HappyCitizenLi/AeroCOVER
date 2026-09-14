#!/usr/bin/env python3
"""Square-range sweep and collision-checked joint chase trajectories."""
import math
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from design_four_scenes import scene, waypoint, ROOT


def square(s, radius=.7):
    side = 5-2*radius
    quarter = side+math.pi*radius/2
    k = min(3, int(s/quarter)); u = s-k*quarter
    if u <= side:
        p, tangent = np.array([-2.5+radius+u,-2.5]), np.array([1.,0.])
    else:
        angle = -math.pi/2+(u-side)/radius
        p = np.array([2.5-radius,-2.5+radius])+radius*np.array([math.cos(angle),math.sin(angle)])
        tangent = np.array([-math.sin(angle),math.cos(angle)])
    a = k*math.pi/2
    rotation = np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    return rotation @ p, rotation @ tangent


def open_scene():
    radius = .7
    length = .875*(4*(5-2*radius)+2*math.pi*radius)
    targets, now = [], 0.
    centers = [(10+5*i, 0., 3.+2*i) for i in range(5)]
    speeds = [.8, 1., 1.1, 1.2, 1.4]
    for i, (center, speed) in enumerate(zip(centers, speeds)):
        duration = length/speed+(1 if i in (0,4) else 0)
        for elapsed in np.linspace(0,duration,int(math.ceil(duration/.2))+1):
            if i == 0:
                distance = speed*(elapsed*elapsed/4 if elapsed < 2 else elapsed-1)
            elif i == 4 and elapsed > duration-2:
                u = elapsed-(duration-2)
                distance = speed*(duration-2+u-u*u/4)
            else:
                distance = speed*elapsed
            xy, _ = square(min(length,distance),radius)
            if targets and abs(now+elapsed-targets[-1]['t']) < 1e-5:
                continue
            targets.append(waypoint(now+elapsed, center[0]+xy[0], xy[1], center[2]))
        now += duration
        if i < 4:
            xy, tangent = square(length,radius)
            first = np.array([center[0]+xy[0],xy[1],center[2]])
            xy, next_tangent = square(0,radius)
            second = np.array([centers[i+1][0]+xy[0],xy[1],centers[i+1][2]])
            first_v = np.r_[speed*tangent,0.]
            second_v = np.r_[speeds[i+1]*next_tangent,0.]
            transfer = 4.
            for elapsed in np.linspace(0,transfer,21)[1:]:
                u=elapsed/transfer
                h=10*u**3-15*u**4+6*u**5
                a=u-6*u**3+8*u**4-3*u**5
                b=-4*u**3+7*u**4-3*u**5
                p=(1-h)*first+h*second+transfer*(a*first_v+b*second_v)
                targets.append(waypoint(now+elapsed,*p))
            now += transfer
    now = round(now+12.,6)
    targets.append(waypoint(now,targets[-1]['x'],targets[-1]['y'],targets[-1]['z']))
    observer=[]
    for t in np.linspace(0,now,int(math.ceil(now/.2))+1):
        theta=1.1+.8/3*(t*t/4 if t<2 else t-1)
        observer.append(waypoint(t,3*math.cos(theta),3*math.sin(theta),3.))
    theta=1.1+.8/3*(now+.2-1)
    observer.append(waypoint(now+.2,3*math.cos(theta),3*math.sin(theta),3.))
    result=scene('OPEN','PW_open',now,observer,[targets])
    result['purpose']='Continuous observer circle; five 5 m rounded squares, 7/8 perimeter each, then hover'
    result['square_centers_relative_height']=[[10+5*i,2*i] for i in range(5)]
    result['square_corner_radius_m']=radius
    return result


def mt_scene():
    duration=120.
    times=np.linspace(0,duration,601)
    def angle(t):
        q=t/duration
        return 4*math.pi*(q-math.sin(2*math.pi*q)/(2*math.pi))
    observer=[waypoint(t,10*math.cos(angle(t)-.35),10*math.sin(angle(t)-.35),3.1) for t in times]
    targets=[[waypoint(t,x,0,5.) for t in (0,duration)] for x in (-16.,16.)]
    for offset,z in zip((-.4,0.,.4),(3.5,4.,4.5)):
        targets.append([waypoint(t,6*math.cos(angle(t)+offset),6*math.sin(angle(t)+offset),z) for t in times])
    result=scene('MT','PW_mt',duration,observer,targets)
    result['allow_static_occlusion_targets']=['uav2','uav3']
    result['moving_target_ids']=['uav4','uav5','uav6']
    result['maximum_moving_mutual_occlusion_s']=1.
    return result


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


def designs_v2():
    routes=ROOT/'config/routes'
    return dict(OPEN=open_scene(),MT=mt_scene(),
                OFFICE=chase_scene('OFFICE',routes/'office.npy'),
                FOREST=chase_scene('FOREST',routes/'forest.npy'))


if __name__ == '__main__':
    import argparse
    import yaml
    parser=argparse.ArgumentParser()
    parser.add_argument('scene',choices=['OPEN','MT','OFFICE','FOREST'])
    parser.add_argument('--route',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=open_scene() if args.scene=='OPEN' else mt_scene() if args.scene=='MT' else chase_scene(args.scene,args.route)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(yaml.safe_dump(config,sort_keys=False))
    print(args.scene,config['duration_s'],args.output,flush=True)
