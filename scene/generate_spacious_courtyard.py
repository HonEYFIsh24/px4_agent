#!/usr/bin/env python3
"""Expand the existing courtyard for observation/approach tests; preserve baselines.

Object identities and clearance measurements are offline authoring/evaluation
data. Only the hash-registered geometry/flight volume reaches navigation.
"""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_reliability_worlds import geometry_bounds, indent, resize_wall

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'test_environments'


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(output=OUTPUT):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(OUTPUT/'courtyard_delivery.world')
    world = tree.getroot().find('world')
    models = {m.get('name'): m for m in world.findall('model')}
    placements = {'red_shipping_container': (14., 9.), 'white_delivery_van': (18., -5.5),
                  'traffic_cone_1': (15., -3.5), 'traffic_cone_2': (16., -3.5),
                  'blue_drum': (23., 9.), 'brick_warehouse': (24., -19.),
                  'park_office': (25., 19.), 'yard_tree_1': (-15., -17.),
                  'yard_tree_2': (-18., 17.), 'street_lamp_south': (22., -12.),
                  'street_lamp_north': (24., 13.), 'north_fence': (0., 36.),
                  'south_fence': (0., -36.), 'east_fence': (36., 0.), 'west_fence': (-36., 0.)}
    for name, (x,y) in placements.items():
        pose = models[name].find('pose'); values = pose.text.split()
        values[:2] = [str(x),str(y)]; pose.text = ' '.join(values)
    resize_wall(models['courtyard_screen'], 6., -3., 3.)
    ground = models['yard_ground']
    for element in ground.findall('./link/visual')+ground.findall('./link/collision'):
        name = element.get('name','')
        size = element.find('geometry/box/size')
        if name in ('solid_ground','solid_ground_collision'):
            size.text = '76 76 0.1'
        elif name.startswith('road_edge_'):
            values = element.find('pose').text.split();values[0]='0'
            values[1] = '-11' if name.endswith('0') else '11'
            element.find('pose').text = ' '.join(values); size.text = '68 0.1 0.0001'
        elif name in ('west_lawn','north_lawn','south_lawn'):
            pose = element.find('pose').text.split()
            pose[0:2] = ['-27','0'] if name=='west_lawn' else ['6','31' if name=='north_lawn' else '-31']
            element.find('pose').text = ' '.join(pose)
            element.find('geometry/plane/size').text = '16 70' if name=='west_lawn' else '52 10'
    # Keep the authored materials, buildings, vehicles and vegetation. Extend
    # fence rails/posts in local coordinates rather than stretching mesh scale.
    for name in ('north_fence','south_fence','east_fence','west_fence'):
        for element in models[name].findall('./link/visual')+models[name].findall('./link/collision'):
            pose = element.find('pose')
            if pose is not None:
                values = list(map(float,pose.text.split()));values[0] *= 1.8
                pose.text = ' '.join(map(str,values))
            size = element.find('geometry/box/size')
            if size is not None:
                values=list(map(float,size.text.split()))
                if values[0] > 10.:values[0] *= 1.8
                size.text=' '.join(map(str,values))
    # More ground texture/markings outside the old yard, with no new obstacles.
    line = next(v for v in ground.findall('./link/visual') if v.get('name','').startswith('lane_dash_'))
    for i,x in enumerate(range(-28,31,4)):
        marker=copy.deepcopy(line);marker.set('name','spacious_lane_'+str(i))
        marker.find('pose').text=f'{x} -10.25 0.0033 0 0 0'
        ground.find('link').append(marker)
    world.find('gui/camera/pose').text='-29 -38 30 0 .58 .78'
    indent(tree.getroot())
    destination=output/'courtyard_spacious.world';tree.write(destination,encoding='utf-8',xml_declaration=True)
    manifest=json.loads((OUTPUT/'courtyard_delivery_manifest.json').read_text())
    manifest.update(id='courtyard_spacious',world_sha256=sha(destination))
    manifest['ground_plane']['bounds_xy_m']=[-38.,-38.,38.,38.]
    manifest['flight_region'].update(bounds_min_m=[-34.,-34.,-.2],bounds_max_m=[34.,34.,20.6])
    manifest['navigation']['bounds_xy_m']=[-34.,-34.,34.,34.]
    manifest['takeoff_pad']['vertical_clearance_m']=20.
    path=output/'courtyard_spacious_manifest.json';write(path,manifest)
    objects=[{'id':m.get('name'),'collision_bounds':geometry_bounds(m,collisions_only=True)}
             for m in world.findall('model') if m.get('name')!='yard_ground']
    write(output/'courtyard_spacious_layout.json',{'evaluation_only':True,'objects':objects,
        'changes':'76m ground; 68m flight box; original visual assets; separated vehicle/screen/buildings',
        'target_ids':['white_delivery_van']})
    registry=output/'index.json';existing=json.loads(registry.read_text()) if registry.exists() else {'scenes':[]}
    entries=[e for e in existing['scenes'] if e['id']!='courtyard_spacious']
    entries.append({'id':'courtyard_spacious','world':destination.name,'manifest':path.name,
                    'world_sha256':sha(destination),'asset_sha256':{'world':sha(destination),'manifest':sha(path)}})
    write(registry,{'schema_version':1,'scenes':entries})
    return destination


if __name__=='__main__':
    print(generate())
