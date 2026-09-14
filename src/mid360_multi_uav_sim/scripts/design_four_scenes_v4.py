#!/usr/bin/env python3
"""Default flowing scenes; OPEN flight speed/FOV exceptions accepted by user."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from design_four_scenes import ROOT, waypoint
from design_four_scenes_v2 import chase_scene
from design_four_scenes_v3 import open_scene, mt_scene, high_speed_contract


def flowing_open():
    config=open_scene()
    entities=[config['observer']]+config['targets']
    t=np.array([w['t'] for w in entities[0]['waypoints']])
    p=np.array([[[w[k] for k in ('x','y','z')] for w in e['waypoints']] for e in entities])
    period=config['duration_s']/2
    leg=period-80.
    phase=np.minimum(t,t[-1]-1e-9)%period
    vertices=np.array([(9.4 if i%2==0 else .15)*
                       np.array([math.cos(math.radians(126-36*i)),
                                 math.sin(math.radians(126-36*i))]) for i in range(11)])
    edge=np.where(phase<=leg,phase/leg*2,2+(phase-leg)/80*8)
    for k in range(2):
        p[2,:,k]=np.interp(edge,np.arange(11),vertices[:,k])+(0 if k==0 else -10)
    returning=phase>leg
    return_x=CubicSpline(np.linspace(leg,period,9),
                        [23.8,16.,12.,6.,0.,-6.,-12.,-16.,-23.8],bc_type='clamped')
    x=return_x(phase[returning])
    p[0,returning,0]=x
    norm=np.hypot(x,9.)
    p[1,returning,0]=6*x/norm
    p[1,returning,1]=9-54/norm
    # Compress slow sections, then round the resulting stops in time. Keep
    # the existing fast leg unchanged rather than creating a new speed peak.
    speed=np.linalg.norm(np.diff(p,axis=1),axis=2).max(axis=0)/np.diff(t)
    rate=np.clip(1.5/np.maximum(speed,.01),1.,3.)
    rate[(t[:-1]%period)<period-80.+1.]=1.
    stamps=np.r_[0.,np.cumsum(np.diff(t)/rate)]
    duration=round(float(stamps[-1]),6)
    sample_t=np.linspace(0,duration,int(math.ceil(duration/.02))+1)
    positions=CubicSpline(stamps,p.transpose(1,0,2),axis=0)(sample_t)
    positions=gaussian_filter1d(positions,.4/np.median(np.diff(sample_t)),axis=0,mode='mirror')
    circle=positions[:,1,:2]-[0.,9.]
    positions[:,1,:2]=[0.,9.]+6*circle/np.linalg.norm(circle,axis=1)[:,None]
    for i,e in enumerate(entities):
        e['waypoints']=[waypoint(t,*p) for t,p in zip(sample_t,positions[:,i])]
    config.update(duration_s=duration,score_end_s=duration-1,
                  purpose='Continuous rounded five-point star and rising circle; straight shuttle without intermediate dwell')
    config['geometry_design']['star_rounding_time_s']=.4
    return config


def designs_v4():
    routes=ROOT/'config/routes'
    office=chase_scene('OFFICE',routes/'office.npy',speed_limit=4.6,
                      acceleration_limit=1.6,deceleration_limit=.7,
                      entry_extension_m=4.,straight_gap_m=4.,flowing=True)
    forest=chase_scene('FOREST',routes/'forest.npy',speed_limit=4.6,
                      acceleration_limit=1.2,deceleration_limit=1.2,
                      entry_extension_m=18.,flowing=True)
    return dict(OPEN=flowing_open(),MT=mt_scene(),
                OFFICE=high_speed_contract(office),FOREST=high_speed_contract(forest))


if __name__=='__main__':
    import argparse
    from pathlib import Path
    import yaml
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    for name,config in designs_v4().items():
        (args.output/(name+'.yaml')).write_text(yaml.safe_dump(config,sort_keys=False))
        print(name,config['duration_s'],flush=True)
