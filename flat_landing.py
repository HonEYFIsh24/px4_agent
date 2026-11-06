"""Narrow static flat-ground support proof, independent of semantic target labels.

Python 3.8 executor module. Reads only hash-registered collision geometry and
the authorized ground manifest. No model/instance names or target truth used.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from mission import PathUnavailable
from static_navigation import transform_pose, apply
from stereo_localization import matmul, matvec, transpose, vector


class LandingUnavailable(PathUnavailable):
    pass


def flat_support_proof(world_path, scene, navigator, hover_enu, *, footprint_radius_m=1.35):
    if (scene.get('verified') is not True or scene.get('world_sha256') != navigator.world_sha256
            or scene.get('ground_plane', {}).get('known') is not True
            or scene.get('navigation', {}).get('method') != 'static_3d'
            or navigator.registration_id is None):
        raise LandingUnavailable('flat landing needs an authorized registered static world')
    point = vector(hover_enu, 3, 'landing hover')
    if not 1. <= point[2] <= 20. or footprint_radius_m != 1.35:
        raise LandingUnavailable('landing hover/footprint is outside the verified profile')
    raw = Path(world_path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != scene['world_sha256']:
        raise LandingUnavailable('landing support world hash changed')
    node = ET.fromstring(raw).find('world')
    if node is None or any(i.findtext('uri') != 'model://sun' for i in node.findall('include')):
        raise LandingUnavailable('landing support has unverified included geometry')
    models = node.findall('model')
    if any(m.findtext('static') != 'true' or m.findall('model') for m in models):
        raise LandingUnavailable('first landing profile requires a wholly static unnested scene')
    ground = scene['ground_plane']
    z = float(ground['z_m'])
    world = navigator.to_world(point)
    radius = footprint_radius_m
    bounds = ground['bounds_xy_m']
    if not (bounds[0]+radius <= world[0] <= bounds[2]-radius
            and bounds[1]+radius <= world[1] <= bounds[3]-radius):
        raise LandingUnavailable('whole aircraft footprint lies outside declared support')
    support_found = False
    for model in models:
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                transforms = [transform_pose(e) for e in (collision, link, model)]
                rotation = matmul(matmul(transforms[2][0], transforms[1][0]), transforms[0][0])
                if abs(rotation[2][0])+abs(rotation[2][1]) > 1e-7 or rotation[2][2] < 1.-1e-7:
                    continue
                centre = [0., 0., 0.]
                for transform in transforms:
                    centre = apply(transform, centre)
                box, plane = geometry.find('box'), geometry.find('plane')
                if box is not None:
                    size = vector([float(v) for v in box.findtext('size').split()], 3, 'support collision box')
                    surface_z = centre[2]+size[2]/2
                    half = [size[0]/2, size[1]/2]
                elif plane is not None:
                    if plane.findtext('normal', '0 0 1').split() != ['0', '0', '1']:
                        continue
                    size = vector([float(v) for v in plane.findtext('size').split()], 2, 'support collision plane')
                    surface_z = centre[2];half = [v/2 for v in size]
                else:
                    continue
                if abs(surface_z-z) > .005 or any(v <= 0 for v in half):
                    continue
                local = matvec(transpose(rotation), [world[i]-centre[i] for i in range(3)])
                if abs(local[0])+radius <= half[0] and abs(local[1])+radius <= half[1]:
                    support_found = True
    if not support_found:
        raise LandingUnavailable('no actual horizontal collision support covers the declared landing footprint')
    floor_enu = z-navigator.origin[2]+.05
    if not navigator.vertical_safe(point, [point[0], point[1], floor_enu]):
        raise LandingUnavailable('full landing descent intersects inflated static geometry')
    # A continuous column/prism check, not sparse ray samples. Existing 1 m
    # inflation is retained; an extra 0.35 m covers hover/descent tracking drift.
    drift = .35
    low = [world[0]-drift, world[1]-drift, z+.03]
    high = [world[0]+drift, world[1]+drift, world[2]+drift]
    if any(all(low[i] <= b[i] and high[i] >= a[i] for i in range(3)) for a, b in navigator.boxes):
        raise LandingUnavailable('continuous descent tracking corridor lacks static clearance')
    if any(world[i]-drift < navigator.bounds[0][i] or world[i]+drift > navigator.bounds[1][i] for i in (0, 1)):
        raise LandingUnavailable('descent corridor exceeds navigation bounds')
    return {'schema_version': 1, 'basis': 'hash_verified_unlabeled_static_collision_support',
            'world_sha256': navigator.world_sha256, 'sortie_id': navigator.registration_id,
            'centre_enu_m': [point[0], point[1], floor_enu-.05], 'hover_enu_m': point,
            'footprint_radius_m': radius, 'tracking_corridor_margin_m': drift,
            'existing_collision_inflation_m': navigator.clearance,
            'horizontal_support_verified': True, 'continuous_descent_verified': True,
            'dynamic_objects_supported': False, 'visual_ground_texture_is_support_proof': False}
