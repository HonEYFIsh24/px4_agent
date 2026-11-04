#!/usr/bin/env python3
"""Reproducible local forest rescue world; no runtime semantic truth."""
import hashlib
import json
import math
import random
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'test_environments'


def node(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None: e.text = str(text)
    return e


def part(link, name, size, position, color, shape='box', collision=True, roll=0., texture=None):
    for tag in ('visual', 'collision') if collision else ('visual',):
        p = node(link, tag, name=name+'_'+tag)
        node(p, 'pose', ' '.join(map(str, [*position, roll, 0, 0])))
        g = node(node(p, 'geometry'), shape)
        if shape == 'box': node(g, 'size', ' '.join(map(str, size)))
        elif shape == 'sphere': node(g, 'radius', size[0])
        else:
            node(g, 'radius', size[0]); node(g, 'length', size[1])
        if tag == 'visual':
            m = node(p, 'material')
            if texture:
                s = node(m, 'script')
                for folder in ('scripts', 'textures'): node(s, 'uri', 'file://'+str(ROOT/'materials'/folder))
                node(s, 'name', 'Yard/'+texture)
            else:
                for channel in ('ambient','diffuse'): node(m, channel, ' '.join(map(str,color)))


def generate(output=OUTPUT):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT/'worlds/obstacle_yard.world'); world = tree.getroot().find('world')
    for m in world.findall('model'): world.remove(m)
    world.set('name', 'forest_rescue')
    world.find('scene/ambient').text = '.65 .68 .62 1'
    world.find('scene/background').text = '.55 .72 .86 1'
    world.find('gui/camera/pose').text = '-27 -36 32 0 .60 .82'
    def model(name,x,y):
        m=node(world,'model',name=name); node(m,'static','true');node(m,'pose',f'{x} {y} .003 0 0 0')
        return node(m,'link',name='structure')
    ground=model('yard_ground',0,0)
    part(ground,'earth',(100,80,.1),(0,0,-.05),None,texture='grass')
    part(ground,'trail',(75,3,.012),(0,-5,.008),None,collision=False,texture='concrete')
    part(ground,'stream',(5,78,.014),(23,0,.01),(.08,.35,.52,1),collision=False)
    # Ground markings and paths are visual layers over the verified flat ground.
    part(ground,'pad',(3,3,.012),(0,0,.012),None,collision=False,texture='wood')
    rng=random.Random(17)
    for i,(x,y) in enumerate([(-7,6),(5,7),(11,-6),(-8,-10),(15,9)]+[
            (rng.uniform(-43,43),rng.choice([-1,1])*rng.uniform(16,35)) for _ in range(25)]):
        link=model(f'pine_{i:02d}',x,y); height=rng.uniform(4.5,8.)
        part(link,'trunk',(.28,height),(0,0,height/2),(.24,.13,.07,1),'cylinder')
        for j in range(3):
            part(link,f'canopy{j}',(1.5-.3*j,),(0,0,height-1.1+j*.7),(.10+.025*j,.25+.03*j,.11,1),'sphere')
    for i,(x,y,sx,sy,h) in enumerate([(4,1,2.8,2.4,2.2),(-4,5,2,3,1.6),(12,7,2.7,2,3.2)]+[
            (rng.uniform(-42,42),rng.choice([-1,1])*rng.uniform(18,32),rng.uniform(2,5),rng.uniform(2,4),rng.uniform(1,4)) for _ in range(12)]):
        link=model(f'rock_{i:02d}',x,y)
        part(link,'stone',(sx,sy,h),(0,0,h/2),None,texture='concrete')
        part(link,'cap',(sx*.7,sy*.75,h*.3),(.1,.1,h),(.36,.38,.35,1))
    for i,(x,y) in enumerate([(16,-12),(-18,14),(-26,-18)]):
        link=model(f'woodland_cabin_{i}',x,y)
        part(link,'walls',(6,4,3),(0,0,1.5),None,texture='wood')
        for side in (-1,1): part(link,'roof'+str(side),(6.6,2.8,.18),(0,side*1.05,3.65),(.22,.13,.08,1),roll=-side*.5)
        part(link,'door',(1,.03,2),(0,-2.02,1),(.12,.08,.04,1),collision=False)
        for xw in (-2,2): part(link,'window'+str(xw),(1,.04,1),(xw,-2.03,1.8),(.17,.40,.48,1),collision=False)
    for name,x,y,color in [('orange_rescue_tent',8,-2,(1,.32,.03,1)),('blue_camp_tent',-15,-12,(.08,.22,.65,1)),('green_camp_tent',-20,-10,(.15,.35,.18,1))]:
        link=model(name,x,y)
        # A-frame canvas: actual sloped box collisions, no invisible enclosing cube.
        for side in (-1,1): part(link,'canvas'+str(side),(3.4,2.2,.08),(0,side*.75,.84),color,roll=-side*.78)
        part(link,'floor',(3.4,3,.08),(0,0,.04),(.26,.20,.12,1))
        part(link,'ridge',(.045,3.5),(0,0,1.65),(.15,.15,.15,1),'cylinder',roll=math.pi/2)
    bridge=model('footbridge',23,-5)
    part(bridge,'deck',(7,3,.35),(0,0,.20),None,texture='wood')
    for side in (-1,1):
        part(bridge,'rail'+str(side),(7,.14,.15),(0,side*1.5,1.3),None,texture='wood')
        for i in range(5): part(bridge,f'post{side}_{i}',(.15,.15,1.3),(-3+i*1.5,side*1.5,.65),None,texture='wood')
    for i in range(6):
        l=model('ruin_pillar_'+str(i),-22+i*3,4)
        part(l,'pillar',(1,1,2+i%3),(0,0,(2+i%3)/2),None,texture='brick')
    from generate_reliability_worlds import indent
    indent(tree.getroot())
    path=output/'forest_rescue.world'; tree.write(path,encoding='utf-8',xml_declaration=True)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=json.loads((ROOT/'test_environments/courtyard_delivery_manifest.json').read_text())
    manifest.update(id='forest_rescue',world_sha256=sha(path))
    manifest['ground_plane']['bounds_xy_m']=[-50,-40,50,40]
    manifest['flight_region'].update(bounds_min_m=[-48,-38,-.2],bounds_max_m=[48,38,10.6])
    manifest['navigation']['bounds_xy_m']=[-48,-38,48,38]
    dest=output/'forest_rescue_manifest.json';dest.write_text(json.dumps(manifest,indent=2)+'\n')
    registry=output/'index.json';data=json.loads(registry.read_text()) if registry.exists() else {'schema_version':1,'scenes':[]}
    data['scenes']=[s for s in data['scenes'] if s['id']!='forest_rescue']
    data['scenes'].append({'id':'forest_rescue','world':path.name,'manifest':dest.name,'world_sha256':sha(path),'asset_sha256':{'world':sha(path),'manifest':sha(dest)}})
    registry.write_text(json.dumps(data,indent=2)+'\n')
    (output/'forest_rescue_layout.json').write_text(json.dumps({'evaluation_only':True,'seed':17,'dimensions_m':[100,80], 'target':{'id':'orange_rescue_tent','position_world_m':[8,-2,.003]},'models':len(world.findall('model')),'scope':'local 12m search in larger static environment; stream is visual only, no water physics'},indent=2)+'\n')
    return path

if __name__=='__main__': print(generate())
