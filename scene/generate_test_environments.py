#!/usr/bin/env python3
"""Create an independently registered static courtyard; no target truth in runtime metadata."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_reliability_worlds import geometry_bounds, resize_wall, indent

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'test_environments'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def generate(output=OUTPUT):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT/'worlds/obstacle_yard.world')
    world = tree.getroot().find('world')
    originals = {m.get('name'): copy.deepcopy(m) for m in world.findall('model')}
    for model in world.findall('model'):
        world.remove(model)
    world.append(originals['yard_ground'])
    # New loading-court layout. These positions are evaluation/generator data only.
    placements = [
        ('red_shipping_container', 11., 5., 0.),
        ('white_delivery_van', 9., -2.5, 0.),
        ('traffic_cone_1', 7.5, -.3, 0.), ('traffic_cone_2', 8.4, -.3, 0.),
        ('blue_drum', 14.5, 5., 0.),
        ('brick_warehouse', 12., -9., 0.),
        ('park_office', 13., 12., 0.),
        ('yard_tree_1', -9., -8., 0.), ('yard_tree_2', -10., 8., 0.),
        ('street_lamp_south', 14., -5., 0.), ('street_lamp_north', 15., 7., 0.),
        ('north_fence', 0., 19., 0.), ('south_fence', 0., -19., 0.),
        ('east_fence', 19.94, 0., 0.), ('west_fence', -19., 0., 0.)]
    for name, x, y, yaw in placements:
        model = originals[name]
        model.find('pose').text = f'{x} {y} 0.003 0 0 {yaw}'
        world.append(model)
    wall = originals['obstacle_wall_1']
    wall.set('name', 'courtyard_screen')
    resize_wall(wall, 4., -4.5, 1.5)
    for element in wall.findall('./link/visual')+wall.findall('./link/collision'):
        pose = [float(x) for x in element.findtext('pose').split()]
        pose[2] *= 3.5/5.5
        element.find('pose').text = ' '.join(f'{x:.9g}' for x in pose)
        size = element.find('geometry/box/size')
        values = [float(x) for x in size.text.split()]
        values[2] *= 3.5/5.5
        size.text = ' '.join(f'{x:.9g}' for x in values)
    world.append(wall)
    world.find('gui/camera/pose').text = '-14 -19 20 0 .65 .82'
    indent(tree.getroot())
    destination = output/'courtyard_delivery.world'
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    manifest = json.loads((ROOT/'obstacle_manifest.json').read_text())
    manifest.update(id='courtyard_delivery', world_sha256=sha(destination))
    manifest['flight_region'].update(bounds_min_m=[-4., -9., -.2], bounds_max_m=[17., 9., 10.6])
    manifest['navigation'].update(method='static_3d', bounds_xy_m=[-4., -9., 17., 9.], flight_altitude_m=3.)
    manifest['takeoff_pad']['vertical_clearance_m'] = 10.
    manifest_path = output/'courtyard_delivery_manifest.json'
    write(manifest_path, manifest)
    # Offline geometry certificate: never read by the agent or its public telemetry.
    objects = [{'id': m.get('name'), 'collision_bounds': geometry_bounds(m, collisions_only=True)}
               for m in world.findall('model') if m.get('name') != 'yard_ground']
    write(output/'courtyard_delivery_layout.json', {'evaluation_only': True, 'objects': objects,
          'target_ids': ['white_delivery_van', 'traffic_cone_1', 'traffic_cone_2'],
          'changes': 'new screen height, vehicle/cargo/cone/building layout; baseline worlds untouched'})
    registry = output/'index.json'
    existing = json.loads(registry.read_text()) if registry.exists() else {'scenes': []}
    retained = [s for s in existing['scenes'] if s['id'] != 'courtyard_delivery']
    write(registry, {'schema_version': 1, 'scenes': retained + [{
        'id': 'courtyard_delivery', 'world': destination.name, 'manifest': manifest_path.name,
        'world_sha256': sha(destination), 'asset_sha256': {'world': sha(destination), 'manifest': sha(manifest_path)}}]})
    return destination


if __name__ == '__main__':
    print(generate())
