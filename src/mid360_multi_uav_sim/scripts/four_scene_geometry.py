#!/usr/bin/env python3
"""Geometry checks for the four-scene suite, using the actual collision mesh."""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@lru_cache(maxsize=1)
def office_triangles():
    path = Path.home()/'.gazebo/models/willowgarage/meshes/willowgarage_collision.dae'
    root = ET.parse(path).getroot()
    for element in root.iter():
        element.tag = element.tag.split('}')[-1]
    assert root.find('asset/up_axis').text == 'Z_UP'
    assert not any(list(root.iter(tag)) for tag in ('matrix', 'translate', 'rotate', 'scale')), 'mesh transforms require explicit handling'
    scale = float(root.find('asset/unit').get('meter'))
    triangles = []
    for mesh in root.findall('library_geometries/geometry/mesh'):
        sources = {s.get('id'): np.fromstring(s.findtext('float_array'), sep=' ').reshape(
                       -1, int(s.find('technique_common/accessor').get('stride')))
                   for s in mesh.findall('source')}
        vertices = {v.get('id'): v.find("input[@semantic='POSITION']").get('source')[1:]
                    for v in mesh.findall('vertices')}
        for block in mesh.findall('triangles'):
            inputs = block.findall('input')
            stride = max(int(i.get('offset')) for i in inputs) + 1
            vertex = block.find("input[@semantic='VERTEX']")
            indices = np.fromstring(block.findtext('p'), sep=' ', dtype=int).reshape(-1, stride)
            points = sources[vertices[vertex.get('source')[1:]]]
            triangles.extend(points[indices[:, int(vertex.get('offset'))]].reshape(-1, 3, 3))
    return np.asarray(triangles) * scale + np.array([-20., -20., 0.])


def mesh_distance(point, triangles):
    """Point/triangle distance, including face interiors and all three edges."""
    p = np.asarray(point)
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b-a, c-a, p-a
    normal = np.cross(ab, ac)
    n2 = np.einsum('ij,ij->i', normal, normal)
    dot = np.einsum('ij,ij->i', ap, normal)
    projection = p - normal * (dot / np.maximum(n2, 1e-30))[:, None]
    inside = np.ones(len(a), dtype=bool)
    best = np.full(len(a), np.inf)
    for first, second in ((a, b), (b, c), (c, a)):
        edge = second-first
        side = np.einsum('ij,ij->i', np.cross(edge, projection-first), normal)
        inside &= side >= -1e-12
        fraction = np.clip(np.einsum('ij,ij->i', p-first, edge) /
                           np.maximum(np.einsum('ij,ij->i', edge, edge), 1e-30), 0, 1)
        best = np.minimum(best, np.linalg.norm(p-first-edge*fraction[:, None], axis=1))
    best = np.where(inside & (n2 > 1e-20), np.abs(dot)/np.sqrt(np.maximum(n2, 1e-30)), best)
    return float(best.min())


def mesh_los(origin, endpoint, triangles):
    """Moller-Trumbore, double-sided open segment; no back-face rejection."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    direction = np.asarray(endpoint)-origin
    e1, e2 = b-a, c-a
    h = np.cross(direction, e2)
    determinant = np.einsum('ij,ij->i', e1, h)
    valid = np.abs(determinant) > 1e-12
    inverse = np.zeros_like(determinant)
    inverse[valid] = 1/determinant[valid]
    s = origin-a
    u = inverse*np.einsum('ij,ij->i', s, h)
    q = np.cross(s, e1)
    v = inverse*np.einsum('j,ij->i', direction, q)
    t = inverse*np.einsum('ij,ij->i', e2, q)
    return not np.any(valid & (u >= 0) & (v >= 0) & (u+v <= 1) & (t > 0) & (t < 1))


def mesh_segment_distance(origin, endpoint, triangles):
    if not len(triangles):
        return float('inf')
    if not mesh_los(origin, endpoint, triangles):
        return 0.
    best = min(mesh_distance(origin, triangles), mesh_distance(endpoint, triangles))
    u = np.asarray(endpoint)-origin
    a = float(u @ u)
    if a < 1e-20:
        return best
    for index in range(3):
        first, second = triangles[:, index], triangles[:, (index+1) % 3]
        v, w = second-first, origin-first
        b = v @ u
        c = np.einsum('ij,ij->i', v, v)
        d = w @ u
        e = np.einsum('ij,ij->i', v, w)
        determinant = a*c-b*b
        safe = np.where(determinant > 1e-20, determinant, 1.)
        s, t = (b*e-c*d)/safe, (a*e-b*d)/safe
        interior = (determinant > 1e-20) & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
        if interior.any():
            best = min(best, float(np.linalg.norm(
                w[interior]+s[interior, None]*u-t[interior, None]*v[interior], axis=1).min()))
        s = np.clip(-d/a, 0, 1)
        best = min(best, float(np.linalg.norm(w+s[:, None]*u, axis=1).min()))
    return best


if __name__ == '__main__':
    triangle = np.array([[[0., 0., 0.], [2., 0., 0.], [0., 2., 0.]]])
    assert abs(mesh_distance([.5, .5, 1], triangle)-1) < 1e-12
    assert abs(mesh_distance([3, 0, 0], triangle)-1) < 1e-12
    assert not mesh_los(np.array([.5, .5, -1]), [.5, .5, 1], triangle)
    assert mesh_los(np.array([3., 3., -1]), [3, 3, 1], triangle)
    assert abs(mesh_segment_distance(np.array([.5, .5, 1]), [1, .5, 1], triangle)-1) < 1e-12
    print('geometry checks PASS; office triangles:', len(office_triangles()))
