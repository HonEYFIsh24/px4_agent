#!/usr/bin/env python3
"""Prepare/check/preview pinned upstream worlds without starting the flight stack.

Assets stay in ignored scene/external_worlds. This does not register a world as
safe for autonomous navigation. No PX4, ROS master, VIO or Agent is launched.
"""
import argparse
import copy
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'scene/external_worlds'
CATALOG = ROOT / 'scene/external_worlds.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')


def verify_source(spec):
    folder = ASSETS / spec['directory']
    marker = folder / '.source_revision.json'
    if marker.exists():
        actual = json.loads(marker.read_text()).get('commit')
    else:
        actual = subprocess.check_output(
            ['git', '-C', str(folder), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != spec['commit']:
        raise RuntimeError('upstream revision mismatch: ' + spec['directory'])
    if digest(folder / spec['world']) != spec['world_sha256']:
        raise RuntimeError('upstream world changed: ' + spec['world'])
    return folder


def prepare(name, spec):
    source = verify_source(spec)
    output = ASSETS / 'prepared' / name
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source / spec['world'])
    world = tree.getroot().find('world')
    changes = []
    # Saved simulation state is not a fresh-start definition.
    for state in world.findall('state'):
        world.remove(state)
        changes.append('removed upstream saved simulation state')
    if spec.get('orchard_xacro'):
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = str(source) + ':/opt/ros/noetic/share'
        env['PYTHONPATH'] = '/opt/ros/noetic/lib/python3/dist-packages'
        urdf = subprocess.check_output(
            ['/opt/ros/noetic/bin/xacro', str(source / spec['orchard_xacro'])],
            env=env, text=True, timeout=30)
        robot = ET.fromstring(urdf)
        for mesh in robot.iter('mesh'):
            uri = mesh.get('filename', '')
            if uri.startswith('package://'):
                file = source / uri[len('package://'):]
                if not file.is_file():
                    raise RuntimeError('missing orchard mesh: ' + str(file))
                mesh.set('filename', file.as_uri())
        urdf_file = output / 'orchard.urdf'
        ET.ElementTree(robot).write(urdf_file, encoding='utf-8', xml_declaration=True)
        sdf = subprocess.check_output(['gz', 'sdf', '-p', str(urdf_file)],
                                      text=True, timeout=30)
        model = ET.fromstring(sdf).find('model')
        if model is None:
            raise RuntimeError('Gazebo URDF conversion produced no orchard model')
        world.append(copy.deepcopy(model))
        changes.append('converted upstream orchard xacro using Gazebo; no ground robot spawned')
    allowed = set(spec.get('omit_preview_plugins', []))
    for parent in list(world.iter()):
        for plugin in list(parent.findall('plugin')):
            filename = plugin.get('filename')
            if filename not in allowed:
                raise RuntimeError('unreviewed world plugin: ' + str(filename))
            parent.remove(plugin)
            changes.append('preview omits plugin ' + filename)
    if spec.get('camera_pose'):
        gui = world.find('gui')
        if gui is None:
            gui = ET.SubElement(world, 'gui')
        camera = gui.find('camera')
        if camera is None:
            camera = ET.SubElement(gui, 'camera', name='user_camera')
        pose = camera.find('pose')
        if pose is None:
            pose = ET.SubElement(camera, 'pose')
        pose.text = spec['camera_pose']
        changes.append('set preview overview camera pose')
    result = output / (name + '.world')
    tree.write(result, encoding='utf-8', xml_declaration=True)
    write_json(output / 'provenance.json', {
        'source': spec, 'prepared_sha256': digest(result), 'changes': changes,
        'flight_approved': False,
        'note': 'Upstream visuals/collisions retained; not registered for autonomous navigation.'})
    return result


def check_resources(world, roots, resource_roots=()):
    """Follow referenced models, including nested includes; fail on missing URIs.

    This checks SDF resource paths, not mesh correctness, flight clearance, or
    every texture reference inside DAE/material scripts. Real rendering is also
    required. Included model plugins are deliberately refused in preview mode.
    """
    seen, missing, plugins = set(), set(), set()

    def scan(path):
        path = Path(path).resolve()
        if path in seen:
            return
        seen.add(path)
        tree = ET.parse(path)
        plugins.update(n.get('filename', '') for n in tree.iter('plugin'))
        # Heightmap diffuse/normal paths are not named <uri> in SDF.
        nodes = list(tree.iter('uri'))
        nodes += [n for h in tree.iter('heightmap') for tag in ('diffuse', 'normal')
                  for n in h.iter(tag)]
        for node in nodes:
            uri = (node.text or '').strip()
            if uri.startswith('model://'):
                target = next((r / uri[8:] for r in roots if (r / uri[8:]).exists()), None)
            elif uri.startswith('file://'):
                rel = Path(uri[7:])
                target = rel if rel.is_absolute() else next(
                    (r / rel for r in resource_roots if (r / rel).exists()), None)
            elif '://' in uri:
                missing.add(uri)
                continue
            else:
                target = path.parent / uri
            if target is None or not target.exists():
                missing.add(uri)
                continue
            config = target / 'model.config' if target.is_dir() else None
            if config is not None and config.is_file():
                # Gazebo 11 supports all SDF versions used by these pinned assets.
                sdf_nodes = ET.parse(config).findall('sdf')
                if not sdf_nodes:
                    missing.add(str(config) + ': no SDF entry')
                else:
                    sdf_file = target / sdf_nodes[0].text.strip()
                    if not sdf_file.is_file():
                        missing.add(str(sdf_file))
                    else:
                        scan(sdf_file)
    scan(world)
    return {'sdf_files_checked': len(seen), 'missing_resources': sorted(missing),
            'unreviewed_plugins': sorted(plugins), 'flight_approved': False}


def stop(process, first_signal=signal.SIGINT):
    if process is None or process.poll() is not None:
        return
    process.send_signal(first_signal)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def preview(world, roots, resource_roots, name, port, duration):
    # Never attach a new client to an existing instance on the chosen port.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', port))
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    log = ROOT / 'logs/external-worlds' / (name + '-' + stamp)
    log.mkdir(parents=True)
    env = os.environ.copy()
    gazebo_log = log / 'gazebo'
    gazebo_log.mkdir()
    env.update(GAZEBO_MASTER_URI='http://127.0.0.1:' + str(port),
               GAZEBO_MODEL_PATH=os.pathsep.join(str(p) for p in roots),
               GAZEBO_RESOURCE_PATH=os.pathsep.join(str(p) for p in resource_roots),
               GAZEBO_LOG_PATH=str(gazebo_log),
               GAZEBO_MODEL_DATABASE_URI='', LP_NUM_THREADS='4')
    server = client = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    old_handlers = {sig: signal.signal(sig, interrupted)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with (log / 'server.log').open('w') as server_log, (log / 'client.log').open('w') as client_log:
            server = subprocess.Popen(['gzserver', '--verbose', '--seed', '17', str(world)],
                                      env=env, stdout=server_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while True:
                if server.poll() is not None:
                    raise RuntimeError('Gazebo server exited; inspect ' + str(log))
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.3):
                        break
                except OSError:
                    if time.monotonic() > deadline:
                        raise RuntimeError('Gazebo master startup timed out')
                    time.sleep(.2)
            client = subprocess.Popen(['gzclient', '--verbose'], env=env,
                                      stdout=client_log, stderr=subprocess.STDOUT)
            write_json(log / 'instance.json', {'world': str(world), 'sha256': digest(world),
                'port': port, 'server_pid': server.pid, 'client_pid': client.pid,
                'flight_stack_started': False})
            print('Preview only. Close Gazebo or press Ctrl+C. Logs: ' + str(log), flush=True)
            start = time.monotonic()
            while client.poll() is None and server.poll() is None:
                if duration and time.monotonic() - start >= duration:
                    break
                time.sleep(.25)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop our GUI while its server still exists, then our server only.
        stop(client, signal.SIGTERM)
        stop(server)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        result = {'server_exit': server.returncode if server else None,
                  'client_exit': client.returncode if client else None}
        write_json(log / 'exit.json', result)
    if any(code not in (None, 0, -signal.SIGINT, -signal.SIGTERM)
           for code in result.values()):
        raise RuntimeError('Gazebo exited abnormally; inspect ' + str(log))


def main():
    catalog = json.loads(CATALOG.read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', choices=sorted(catalog['worlds']))
    parser.add_argument('--check', action='store_true', help='prepare/check without launching')
    parser.add_argument('--port', type=int, default=11365)
    parser.add_argument('--duration', type=float, default=0, help='auto-close after seconds; 0 waits for GUI close')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535 or args.duration < 0:
        parser.error('invalid port or duration')
    spec = catalog['worlds'][args.world]
    world = prepare(args.world, spec)
    roots = [ASSETS / p for p in spec['model_paths']]
    roots += [Path.home() / '.gazebo/models', Path('/usr/share/gazebo-11/models')]
    resource_roots = [ASSETS / spec['directory'], Path('/usr/share/gazebo-11')]
    for model in spec.get('texture_search_models', []):
        folder = next((r / model / 'materials/textures' for r in roots
                       if (r / model / 'materials/textures').is_dir()), None)
        if folder is None:
            raise RuntimeError('missing texture search directory: ' + model)
        resource_roots.append(folder)
    result = check_resources(world, roots, resource_roots)
    write_json(world.parent / 'resource-check.json', result)
    print(json.dumps({'world': str(world), **result}, ensure_ascii=False), flush=True)
    if result['missing_resources'] or result['unreviewed_plugins']:
        raise RuntimeError('preview dependency validation failed')
    if not args.check:
        preview(world, roots, resource_roots, args.world, args.port, args.duration)


if __name__ == '__main__':
    main()
