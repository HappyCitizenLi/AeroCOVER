#!/usr/bin/env python3
"""Plan a visible pair of UAVs using the existing world collision geometry."""
import argparse
import heapq
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree

from four_scene_geometry import office_triangles

ROOT = Path(__file__).resolve().parents[1]
RES = .1
OBSERVER_Z, TARGET_Z = 1., 1.2


def mesh_samples(spacing=.08):
    samples = []
    for tri in office_triangles():
        a, b, c = tri
        count = max(1, int(math.ceil(max(np.linalg.norm(b-a), np.linalg.norm(c-a), np.linalg.norm(c-b))/spacing)))
        u, v = np.meshgrid(np.arange(count+1)/count, np.arange(count+1)/count)
        valid = u+v <= 1+1e-12
        samples.append(a+u[valid, None]*(b-a)+v[valid, None]*(c-a))
    return np.concatenate(samples)


def fields(world, output):
    path = output/f'{world}_fields.npz'
    if path.exists():
        return dict(np.load(path))
    if world == 'office':
        x = np.arange(-18., 40.+RES/2, RES)
        y = np.arange(-21., 25.+RES/2, RES)
        tree = cKDTree(mesh_samples())
        xx, yy = np.meshgrid(x, y)
        layers = {}
        for key, z in (('observer', OBSERVER_Z), ('target', TARGET_Z), ('sight', (OBSERVER_Z+.168+TARGET_Z)/2)):
            points = np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, z)))
            layers[key] = tree.query(points, workers=4)[0].reshape(xx.shape)
    else:
        x = np.arange(-8., 58.+RES/2, RES)
        y = np.arange(-3., 53.+RES/2, RES)
        xx, yy = np.meshgrid(x, y)
        clearance = np.full(xx.shape, np.inf)
        model = ET.parse(ROOT/'worlds/PW_forest_seed0.world').find("world/model[@name='planning_forest_seed0']")
        for link in model.findall('link'):
            p = np.fromstring(link.findtext('pose'), sep=' ')
            radius = float(link.findtext('collision/geometry/cylinder/radius'))
            clearance = np.minimum(clearance, np.hypot(xx-p[0], yy-p[1])-radius)
        layers = dict(observer=clearance, target=clearance, sight=clearance)
    result = dict(x=x, y=y, **layers)
    np.savez_compressed(path, **result)
    return result


def pair_validity(data, length, angles=64):
    yy, xx = np.indices(data['sight'].shape, dtype=float)
    valid = np.empty((angles, *xx.shape), dtype=bool)
    for k in range(angles):
        theta = 2*math.pi*k/angles
        dx, dy = math.cos(theta)*length/2/RES, math.sin(theta)*length/2/RES
        valid[k] = (map_coordinates(data['observer'], [yy-dy, xx-dx], order=1, mode='constant', cval=-1) > .82)
        valid[k] &= (map_coordinates(data['target'], [yy+dy, xx+dx], order=1, mode='constant', cval=-1) > .82)
        for f in np.linspace(-1, 1, int(math.ceil(length/.15))+1):
            valid[k] &= map_coordinates(data['sight'], [yy+f*dy, xx+f*dx], order=1, mode='constant', cval=-1) > .20
    return valid


def plan(data, valid, start, goal):
    ntheta, ny, nx = valid.shape
    def cell(p):
        return (int(round(p[1]/(2*math.pi)*ntheta)) % ntheta,
                int(round((p[0][1]-data['y'][0])/RES)), int(round((p[0][0]-data['x'][0])/RES)))
    start = cell(start)
    target = np.asarray(goal)
    assert valid[start], ('invalid start', start)
    def heuristic(s):
        return float(np.hypot(data['x'][s[2]]-target[0], data['y'][s[1]]-target[1]))
    queue = [(heuristic(start), 0., start)]
    cost, previous = {start: 0.}, {}
    moves = [(dy, dx) for dy in (-2, 0, 2) for dx in (-2, 0, 2) if dy or dx]
    visited = 0
    while queue:
        _, distance, current = heapq.heappop(queue)
        if distance != cost.get(current):
            continue
        if heuristic(current) < .24:
            path = [current]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return list(reversed(path))
        k, y, x = current
        neighbors = []
        for dy, dx in moves:
            q = (k, y+dy, x+dx)
            if not (0 <= q[1] < ny and 0 <= q[2] < nx):
                continue
            if valid[q] and valid[k, y+dy//2, x+dx//2]:
                neighbors.append((q, RES*math.hypot(dx, dy)))
        for dk in (-1, 1):
            q = ((k+dk) % ntheta, y, x)
            if valid[q]:
                neighbors.append((q, .22*64/ntheta))
        for q, change in neighbors:
            value = distance+change
            if value < cost.get(q, math.inf):
                cost[q], previous[q] = value, current
                heapq.heappush(queue, (value+heuristic(q), value, q))
        visited += 1
        if visited % 100000 == 0:
            print('expanded', visited, 'queue', len(queue), flush=True)
    raise RuntimeError('No visible-pair path found')


def plot(data, valid, path, length, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(data['target'], origin='lower', extent=[data['x'][0],data['x'][-1],data['y'][0],data['y'][-1]], vmin=0, vmax=2, cmap='gray')
    if path:
        states = np.array([[data['x'][x],data['y'][y],k*2*math.pi/len(valid)] for k,y,x in path])
        delta = length/2*np.column_stack((np.cos(states[:,2]),np.sin(states[:,2])))
        for label, sign in (('observer',-1),('target',1)):
            pts=states[:,:2]+sign*delta
            ax.plot(pts[:,0],pts[:,1],label=label)
        np.save(output.with_suffix('.npy'), states)
        ax.legend()
    ax.set_aspect('equal'); ax.grid(); fig.savefig(output, dpi=140); plt.close(fig)


def expand_office_spacing(states, data):
    """Expand straight/open sections, retaining short links at narrow turns."""
    resolution=float(data['x'][1]-data['x'][0])
    xx=(states[:,0]-data['x'][0])/resolution
    yy=(states[:,1]-data['y'][0])/resolution
    limits=np.full(len(states),2.6)
    alive=np.ones(len(states),dtype=bool)
    for length in np.arange(2.8,7.01,.2):
        dx=length/2*np.cos(states[:,2])/resolution
        dy=length/2*np.sin(states[:,2])/resolution
        alive &= map_coordinates(data['observer'],[yy-dy,xx-dx],order=1,mode='constant',cval=-1)>.83
        alive &= map_coordinates(data['target'],[yy+dy,xx+dx],order=1,mode='constant',cval=-1)>.83
        for f in np.linspace(-1,1,int(math.ceil(length/.15))+1):
            alive &= map_coordinates(data['sight'],[yy+f*dy,xx+f*dx],order=1,mode='constant',cval=-1)>.21
        limits[alive]=length
    theta=np.unwrap(states[:,2])
    ds=np.hypot(*np.diff(states[:,:2],axis=0).T)+1.3*np.abs(np.diff(theta))
    for i in range(1,len(limits)):
        limits[i]=min(limits[i],limits[i-1]+.4*ds[i-1])
    for i in range(len(limits)-2,-1,-1):
        limits[i]=min(limits[i],limits[i+1]+.4*ds[i])
    return np.column_stack((states,limits))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('world', choices=('office','forest'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plan', action='store_true')
    parser.add_argument('--start', type=float, nargs=3, metavar=('X','Y','YAW_DEG'))
    parser.add_argument('--goal', type=float, nargs=2, metavar=('X','Y'))
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--angles', type=int, default=64)
    parser.add_argument('--observer-height', type=float, default=1.)
    parser.add_argument('--target-height', type=float, default=1.2)
    parser.add_argument('--length', type=float)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    RES = args.resolution
    OBSERVER_Z,TARGET_Z=args.observer_height,args.target_height
    data = fields(args.world, args.output)
    length = 4.15 if args.world == 'office' else 4.5
    if args.length is not None: length=args.length
    valid_path = args.output/(f'{args.world}_valid_L{length:g}_{args.angles}.npy' if args.length is not None else
                             f'{args.world}_valid.npy' if args.angles==64 else f'{args.world}_valid_{args.angles}.npy')
    if valid_path.exists(): valid = np.load(valid_path)
    else:
        valid = pair_validity(data,length,args.angles); np.save(valid_path,valid)
    print('valid states',int(valid.sum()),flush=True)
    path = None
    if args.plan:
        start, goal = (((-4,25),0),(54,25)) if args.world=='forest' else (((-7,8.2),0),(30,-5))
        if args.start: start = (args.start[:2], math.radians(args.start[2]))
        if args.goal: goal = args.goal
        path = plan(data,valid,start,goal)
    plot(data,valid,path,length,args.output/f'{args.world}_route.png')
