#!/usr/bin/env python3
"""Import an explicitly flat test derivative of Clearpath's orchard.

Preserve upstream visual meshes/UVs/layout, flatten only the ground. Tree trunks
AND leaves are enclosed by conservative spatial-bin boxes, including all source
triangle AABBs. This changes collision geometry conservatively; it is not an
evaluation of navigation over the original uneven ground.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import shlex
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission import MAX_ALTITUDE
from static_navigation import transform_pose

ASSETS = ROOT / 'scene/external_worlds'
PREPARED = ASSETS / 'prepared/orchard_flat'
OUTPUT = ROOT / 'scene/test_environments'
SHIFT = np.array([2., -12., 0.])
# Rotate the whole orchard, not the aircraft/VIO reference, toward nearby rows.
YAW = math.pi
WORLD_ROTATION = np.diag([-1., -1., 1.])
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def add(parent, tag, text=None, **attrs):
    n = ET.SubElement(parent, tag, attrs)
    if text is not None:
        n.text = str(text)
    return n


def values(items):
    return ' '.join(format(float(v), '.12g') for v in items)


def load_dump(path):
    meshes = []
    with Path(path).open('rb') as f:
        count, = struct.unpack('<I', f.read(4))
        if not 0 < count <= 1000:
            raise ValueError('invalid native mesh count')
        for _ in range(count):
            nv, ni = struct.unpack('<II', f.read(8))
            if not 0 < nv < 10000000 or not 0 < ni < 30000000 or ni % 3:
                raise ValueError('invalid triangle mesh size')
            vertices = np.fromfile(f, dtype='<f8', count=nv*3).reshape(nv, 3)
            indices = np.fromfile(f, dtype='<u4', count=ni).reshape(-1, 3)
            if not np.isfinite(vertices).all() or indices.max() >= nv:
                raise ValueError('invalid mesh data')
            meshes.append((vertices, indices))
        if f.read(1):
            raise ValueError('trailing native mesh data')
    return meshes


def triangle_boxes(triangles, cell=2.):
    """Enclose each triangle's whole AABB by a union of clipped grid-bin boxes.

    Not just a vertex test: a triangle crossing bins is represented in EVERY
    intersected bin, even when that bin contains no original triangle vertex.
    No source triangle may be dropped. The 5 mm padding is conservative.
    """
    t = np.asarray(triangles, dtype=float)
    if t.ndim != 3 or t.shape[1:] != (3, 3) or not len(t) or not np.isfinite(t).all():
        raise ValueError('invalid triangles')
    if not math.isfinite(cell) or cell <= 0:
        raise ValueError('invalid cell size')
    bins = {}
    for lo, hi in zip(t.min(axis=1), t.max(axis=1)):
        for ix in range(math.floor(lo[0]/cell), math.floor(hi[0]/cell)+1):
            for iy in range(math.floor(lo[1]/cell), math.floor(hi[1]/cell)+1):
                a, b = lo.copy(), hi.copy()
                a[:2] = np.maximum(a[:2], [ix*cell, iy*cell])
                b[:2] = np.minimum(b[:2], [(ix+1)*cell, (iy+1)*cell])
                key = (ix, iy)
                if key in bins:
                    a = np.minimum(a, bins[key][0])
                    b = np.maximum(b, bins[key][1])
                bins[key] = (a, b)
    return [(lo-.005, hi+.005) for _, (lo, hi) in sorted(bins.items())]


def flatten_ground(source, target):
    tree = ET.parse(source)
    root = tree.getroot()
    if root.findtext('c:asset/c:up_axis', namespaces=NS) != 'Z_UP':
        raise ValueError('unexpected ground up axis')
    if root.find('c:asset/c:unit', NS).get('meter') != '1':
        raise ValueError('unexpected ground units')
    for matrix in root.findall('.//c:visual_scene//c:matrix', NS):
        if not np.array_equal(np.fromstring(matrix.text, sep=' ').reshape(4, 4), np.eye(4)):
            raise ValueError('ground has nonidentity node transform')
    for geometry in root.findall('c:library_geometries/c:geometry', NS):
        mesh = geometry.find('c:mesh', NS)
        position_ids = {n.get('source')[1:] for n in mesh.findall('c:vertices/c:input', NS)
                        if n.get('semantic') == 'POSITION'}
        normal_ids = {n.get('source')[1:] for n in mesh.findall('.//c:input', NS)
                      if n.get('semantic') == 'NORMAL'}
        if not position_ids:
            raise ValueError('no ground positions')
        for source_node in mesh.findall('c:source', NS):
            key = source_node.get('id')
            if key not in position_ids | normal_ids:
                continue
            array = source_node.find('c:float_array', NS)
            accessor = source_node.find('c:technique_common/c:accessor', NS)
            if accessor.get('stride') != '3':
                raise ValueError('unexpected ground vertex stride')
            data = np.fromstring(array.text, sep=' ').reshape(-1, 3)
            if not np.isfinite(data).all():
                raise ValueError('invalid ground vertices')
            if key in position_ids:
                data[:, 2] = 0.
            else:
                data[:] = [0., 0., 1.]
            array.text = values(data.flat)
    for image in root.findall('c:library_images/c:image/c:init_from', NS):
        texture = (Path(source).parent / image.text).resolve()
        if not texture.is_file():
            raise ValueError('missing original ground texture')
        image.text = str(texture)
    ET.register_namespace('', NS['c'])
    tree.write(target, encoding='utf-8', xml_declaration=True)


def build(output=OUTPUT):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    PREPARED.mkdir(parents=True, exist_ok=True)
    source = ASSETS / 'prepared/orchard/orchard.world'
    source_world = ET.parse(source).getroot().find('world')
    source_model = source_world.find('model')
    if (len(source_world.findall('model')) != 1 or source_model.findtext('static') not in ('true', '1')
            or source_model.findall('.//plugin')):
        raise ValueError('unexpected upstream orchard structure')
    # The converted upstream model has one physical link, with explicit poses.
    links = source_model.findall('link')
    if len(links) != 1:
        raise ValueError('unexpected orchard links')
    source_link = links[0]
    helper = PREPARED / 'mesh_dump'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    subprocess.run(['c++', '-std=c++17', str(ROOT/'scene/gazebo_mesh_dump.cc'), '-o', str(helper), *flags], check=True)
    sources, triangles, ground_triangles = {}, [], None
    visuals = source_link.findall('visual')
    if len(visuals) != 4:
        raise ValueError('unexpected upstream visual mesh count')
    for idx, visual in enumerate(visuals):
        mesh = visual.find('geometry/mesh')
        uri = mesh.findtext('uri')
        if not uri.startswith('file://'):
            raise ValueError('expected resolved local mesh')
        path = Path(uri[7:])
        sources[str(path.relative_to(ROOT))] = sha(path)
        binary = PREPARED / ('mesh_%d.bin' % idx)
        subprocess.run([str(helper), str(path), str(binary)], check=True, stderr=subprocess.DEVNULL)
        scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise ValueError('invalid mesh scale')
        parts = []
        for v, indices in load_dump(binary):
            v = v * scale
            for element in (visual, source_link, source_model):
                rotation, offset = transform_pose(element)
                v = v @ np.asarray(rotation).T + offset
            parts.append(((v + SHIFT) @ WORLD_ROTATION.T)[indices])
        points = np.concatenate(parts)
        if path.name == 'orchard_world.dae':
            ground_triangles = points
            flat = PREPARED / 'flat_ground.dae'
            flatten_ground(path, flat)
            visual.find('geometry/mesh/uri').text = flat.as_uri()
            sources[str(flat.relative_to(ROOT))] = sha(flat)
        else:
            triangles.append(points)
        # Pin diffuse/normal textures referenced inside original Collada files.
        for image in ET.parse(path).findall('c:library_images/c:image/c:init_from', NS):
            texture = (path.parent / image.text).resolve()
            sources[str(texture.relative_to(ROOT))] = sha(texture)
    if ground_triangles is None:
        raise ValueError('ground mesh missing')
    # Collision meshes can be larger than visuals (e.g. the tripod's STL).
    # Include every original non-ground collision as well, even if redundant.
    for idx, collision in enumerate(source_link.findall('collision')):
        mesh = collision.find('geometry/mesh')
        if mesh is None or not mesh.findtext('uri', '').startswith('file://'):
            raise ValueError('unexpected upstream collision geometry')
        path = Path(mesh.findtext('uri')[7:])
        if path.name == 'orchard_world.dae':
            continue  # Explicitly replaced by the user-approved flat support.
        sources[str(path.relative_to(ROOT))] = sha(path)
        binary = PREPARED / ('collision_%d.bin' % idx)
        subprocess.run([str(helper), str(path), str(binary)], check=True, stderr=subprocess.DEVNULL)
        scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise ValueError('invalid collision mesh scale')
        for v, indices in load_dump(binary):
            v = v * scale
            for element in (collision, source_link, source_model):
                rotation, offset = transform_pose(element)
                v = v @ np.asarray(rotation).T + offset
            triangles.append(((v + SHIFT) @ WORLD_ROTATION.T)[indices])
    ground_low = ground_triangles.min(axis=(0, 1))
    ground_high = ground_triangles.max(axis=(0, 1))
    boxes = triangle_boxes(np.concatenate(triangles))
    # Reuse the successful baseline's SITL physics and geomagnetic reference.
    tree = ET.parse(ROOT/'scene/worlds/obstacle_yard.world')
    world = tree.getroot().find('world')
    world.set('name', 'orchard_flat_test')
    for model in world.findall('model'):
        world.remove(model)
    for tag in ('scene', 'gui'):
        existing = world.find(tag)
        if existing is not None:
            world.remove(existing)
        world.append(copy.deepcopy(source_world.find(tag)))
    world.find('gui/camera/pose').text = '-15 -12 22 0 .72 .85'
    ground = add(world, 'model', name='yard_ground')
    add(ground, 'static', 'true')
    ground_link = add(ground, 'link', name='ground')
    collision = add(ground_link, 'collision', name='flat_support')
    center = (ground_low + ground_high)/2
    center[2] = -.05
    size = ground_high-ground_low
    size[2] = .1
    add(collision, 'pose', values([*center, 0, 0, 0]))
    add(add(add(collision, 'geometry'), 'box'), 'size', values(size))
    model = add(world, 'model', name='orchard_visual_geometry')
    add(model, 'static', 'true')
    link = add(model, 'link', name='geometry')
    for visual in visuals:
        # Converted upstream model/link are identity; reject a future revision that isn't.
        for parent in (source_model, source_link):
            rotation, offset = transform_pose(parent)
            if not np.allclose(rotation, np.eye(3), atol=1e-12) or not np.allclose(offset, 0, atol=1e-12):
                raise ValueError('unexpected enclosing mesh transform')
        p = np.fromstring(visual.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        p[:3] = WORLD_ROTATION @ (p[:3] + SHIFT)
        p[5] += YAW
        visual.find('pose').text = values(p)
        (ground_link if 'flat_ground.dae' in visual.findtext('geometry/mesh/uri') else link).append(copy.deepcopy(visual))
    for i, (lo, hi) in enumerate(boxes):
        collision = add(link, 'collision', name='conservative_%04d' % i)
        add(collision, 'pose', values([*((lo+hi)/2), 0, 0, 0]))
        add(add(add(collision, 'geometry'), 'box'), 'size', values(hi-lo))
    path = output/'orchard_flat_test.world'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    bounds = [*(ground_low[:2] + .3), *(ground_high[:2] - .3)]
    manifest = {'id': 'orchard_flat_test', 'world_sha256': sha(path),
        'ground_plane': {'known': True, 'z_m': 0., 'bounds_xy_m': [*ground_low[:2], *ground_high[:2]]},
        'flight_region': {'frame': 'gazebo_world', 'bounds_min_m': [*bounds[:2], -.2],
                         'bounds_max_m': [*bounds[2:], MAX_ALTITUDE+.6], 'margin_m': .6},
        'takeoff_pad': {'center_world_m': [0, 0, 0], 'radius_m': 1., 'vertical_clearance_m': MAX_ALTITUDE},
        'navigation': {'enabled': True, 'method': 'static_3d', 'bounds_xy_m': bounds,
                       'flight_altitude_m': 3., 'minimum_clearance_m': 1.},
        'external_assets_sha256': sources}
    manifest_path = output/'orchard_flat_test_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    index_path = output/'index.json'
    registry = json.loads(index_path.read_text()) if index_path.exists() else {'schema_version': 1, 'scenes': []}
    entry = {'id': manifest['id'], 'world': path.name, 'manifest': manifest_path.name,
             'world_sha256': sha(path), 'asset_sha256': {'world': sha(path), 'manifest': sha(manifest_path)}}
    registry['scenes'] = [e for e in registry['scenes'] if e.get('id') != entry['id']] + [entry]
    index_path.write_text(json.dumps(registry, indent=2)+'\n')
    report = {'world': str(path), 'world_sha256': sha(path), 'proxy_boxes': len(boxes),
              'obstacle_triangles': sum(len(t) for t in triangles),
              'source_ground_z_range': [float(ground_low[2]), float(ground_high[2])],
              'test_ground_z': 0., 'translation_before_rotation_m': SHIFT.tolist(), 'world_rotation_rad': YAW,
              'collision_method': 'union of clipped 2m XY triangle-AABB bins, padded 5mm',
              'source_assets_sha256': sources,
              'limitations': 'flat derivative; conservative boxes occlude more than original foliage'}
    (PREPARED/'import-report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    print(json.dumps(build(), indent=2))
