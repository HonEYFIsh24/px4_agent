#!/usr/bin/env python3
"""Read-only gate for an explicitly approved private Gazebo physics library.

The candidate intentionally has no approved manifest digest. Default is PLAN;
--verify cannot succeed until root reviews the real build and freezes that digest.
No library is loaded, no process is started except bounded readelf, no ROS calls.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

PROJECT = Path('/home/dut/px4_agent')
RUNTIME_PART = Path('vio/gazebo_sync')
# Pending real private build, deployment and root approval; never infer approval.
FROZEN_MANIFEST_SHA256 = 'ffab589c5960c551f1ce8fc4916368483382bd36c0430cb87aa4a9b6f35bfd72'
SOURCE_PATCH_SHA256 = '50b7c8db8a6a4de6b26f56d5f57c3a2be78b5231db879252d2f6dc3a887cd3b8'
OFFICIAL_ARCHIVE_SHA256 = '77a952971eabfd79e16d2582a355ee017a2e22cc369b68453323c0984b725126'
LIBRARY = 'libgazebo_physics.so.11.15.1'
SONAME = 'libgazebo_physics.so.11'
LINKS = {SONAME: LIBRARY, 'libgazebo_physics.so': SONAME}
CHECKS = ('no_rpath_runpath', 'ldd_r_ok', 'rtld_now_ok',
          'all_nonphysics_gazebo_deps_installed',
          'public_physics_abi_exports_preserved', 'private_only_not_installed')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def owned_directory(path, private=True):
    value = path.lstat()
    require(stat.S_ISDIR(value.st_mode) and value.st_uid == os.getuid(),
            'Library path directory must be same-UID and nonsymlink')
    # Existing shared workspace ancestors can be 0775. Do not chmod them.
    if private:
        require(not value.st_mode & 0o022,
                'Private library directory must not be writable by others')


def pinned_bytes(path, maximum):
    """No symlink final component, stable inode/bytes, bounded regular file."""
    before = path.lstat()
    require(stat.S_ISREG(before.st_mode) and before.st_uid == os.getuid()
            and not before.st_mode & 0o022, 'Evidence/library must be an owned regular nonsymlink file')
    descriptor = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        initial = os.fstat(descriptor)
        require((initial.st_dev, initial.st_ino) == (before.st_dev, before.st_ino)
                and 0 < initial.st_size <= maximum, 'Library/evidence inode or size invalid')
        data = bytearray()
        while len(data) < initial.st_size:
            chunk = os.read(descriptor, min(1024*1024, initial.st_size-len(data)))
            require(chunk, 'Library/evidence shrank')
            data.extend(chunk)
        final, final_path = os.fstat(descriptor), path.lstat()
        for value in (final, final_path):
            require((value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
                    == (initial.st_dev, initial.st_ino, initial.st_size, initial.st_mtime_ns, initial.st_ctime_ns),
                    'Library/evidence changed while reading')
        return bytes(data)
    finally:
        os.close(descriptor)


def dynamic_tags(path):
    # readelf interprets bytes only: it does not execute ELF initializers or ldd.
    result = subprocess.run(['/usr/bin/readelf', '-h', '-d', str(path)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            env=dict(os.environ, LC_ALL='C'), timeout=2, check=False)
    require(result.returncode == 0 and len(result.stdout) <= 128*1024,
            'Bounded readelf dynamic-tag inspection failed')
    value = result.stdout.decode('utf-8', errors='strict')
    require(re.search(r'Class:\s+ELF64', value) and re.search(r'Data:.*little endian', value)
            and re.search(r'Type:\s+DYN\b', value)
            and re.search(r'Machine:\s+Advanced Micro Devices X86-64', value),
            'Private library must be the actual local ELF64 x86-64 shared object')
    sonames = re.findall(r'\(SONAME\).*\[([^\]]+)\]', value)
    needed = re.findall(r'\(NEEDED\).*\[([^\]]+)\]', value)
    require(sonames == [SONAME] and needed and all('/' not in name for name in needed),
            'Unexpected SONAME or absolute dependency')
    require(not re.search(r'\((?:RPATH|RUNPATH)\)', value),
            'Private physics library must not redirect dependency resolution using RPATH/RUNPATH')
    return {'soname': sonames[0], 'needed': needed, 'no_rpath_runpath': True}


def validate(project=PROJECT, frozen_manifest_sha256=FROZEN_MANIFEST_SHA256):
    require(isinstance(frozen_manifest_sha256, str)
            and re.fullmatch(r'[a-f0-9]{64}', frozen_manifest_sha256),
            'No real built/deployed library manifest has been approved and frozen')
    runtime = project / RUNTIME_PART
    library_directory = runtime / 'lib'
    for path in (project, project/'vio'):
        owned_directory(path, private=False)
    for path in (runtime, library_directory):
        owned_directory(path)
    manifest_path = runtime / 'manifest.json'
    manifest_bytes = pinned_bytes(manifest_path, 1024*1024)
    require(hashlib.sha256(manifest_bytes).hexdigest() == frozen_manifest_sha256,
            'Actual manifest differs from the reviewed frozen digest')
    manifest = json.loads(manifest_bytes)
    require(type(manifest.get('schema_version')) is int and manifest['schema_version'] == 1
            and manifest.get('status') == 'READY_PRIVATE_LIBRARY_ONLY'
            and manifest.get('gazebo_version') == '11.15.1'
            and manifest.get('soname') == SONAME
            and manifest.get('library_dir') == str(library_directory)
            and manifest.get('library_name') == LIBRARY
            and manifest.get('source_patch_sha256') == SOURCE_PATCH_SHA256
            and manifest.get('official_archive_sha256') == OFFICIAL_ARCHIVE_SHA256
            and manifest.get('soname_links') == LINKS,
            'Private library manifest schema, prefix, source or aliases mismatch')
    require(all((manifest.get('checks') or {}).get(name) is True for name in CHECKS),
            'Real dependency/ABI/private-build evidence is incomplete')
    record = manifest.get('library') or {}
    path = library_directory / LIBRARY
    require(record.get('path') == str(path) and type(record.get('size_bytes')) is int
            and re.fullmatch(r'[a-f0-9]{64}', record.get('sha256', '')),
            'Actual library record missing or invalid')
    require(set(p.name for p in library_directory.iterdir()) == {LIBRARY, *LINKS},
            'Private LD directory must contain only the audited physics ELF and two aliases')
    for name, target in LINKS.items():
        link = library_directory / name
        details = link.lstat()
        require(stat.S_ISLNK(details.st_mode) and details.st_uid == os.getuid()
                and os.readlink(str(link)) == target,
                'Physics aliases must be same-UID exact relative links without path escape')
    binary = pinned_bytes(path, 512*1024*1024)
    actual_digest = hashlib.sha256(binary).hexdigest()
    require(actual_digest == record['sha256'] and len(binary) == record['size_bytes'],
            'Actual private physics ELF does not match the frozen manifest')
    tags = dynamic_tags(path)
    # Refuse an observed change during separate readelf execution; no dlopen here.
    require(pinned_bytes(path, 512*1024*1024) == binary
            and pinned_bytes(manifest_path, 1024*1024) == manifest_bytes,
            'Library or manifest changed during dynamic inspection')
    for name, target in LINKS.items():
        require(os.readlink(str(library_directory/name)) == target,
                'Physics alias changed during inspection')
    return {'library_directory': str(library_directory),
            'manifest_sha256': frozen_manifest_sha256, 'library_sha256': actual_digest,
            'library_size_bytes': len(binary), 'dynamic_tags': tags,
            'library_loaded': False, 'runtime_or_ROS_calls': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args(argv)
    if not args.verify:
        print(json.dumps({'status': 'PLAN', 'actual_library_verified': False,
                          'approved_manifest_present_in_candidate': bool(FROZEN_MANIFEST_SHA256),
                          'fixed_library_directory': str(PROJECT/RUNTIME_PART/'lib')}))
        return 0
    try:
        evidence = validate()
        # stdout is a single fixed path consumed by the shell; safe evidence on stderr.
        print(json.dumps(evidence), file=sys.stderr)
        print(evidence['library_directory'])
        return 0
    except (ValueError, TypeError, KeyError, OSError, subprocess.SubprocessError) as exc:
        print('Experimental Gazebo library rejected: '+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
