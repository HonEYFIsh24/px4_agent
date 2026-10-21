"""Download only the official VIO packages and verify every Git blob hash."""
import hashlib
import json
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DESTINATION = ROOT.parent / 'vio_ws' / 'src' / 'VINS-Fusion'


def main():
    manifest = json.loads((ROOT / 'upstream_manifest.json').read_text())
    marker = DESTINATION / '.download_complete'
    if marker.exists():
        print('Official sources are already present; preserving local compatibility patches.')
        return

    def download(item):
        destination = (DESTINATION / item['path']).resolve()
        if DESTINATION.resolve() not in destination.parents:
            raise RuntimeError('invalid upstream path')
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not item['url'].startswith('https://raw.githubusercontent.com/HKUST-Aerial-Robotics/VINS-Fusion/'):
            raise RuntimeError('unexpected upstream host')
        command = ['curl', '-L', '--fail', '--silent', '--show-error', '--connect-timeout', '8',
                   '--max-time', '30', '--retry', '1']
        raw_ip = os.getenv('VINS_RAW_IP')
        if raw_ip:
            command += ['--resolve', 'raw.githubusercontent.com:443:' + raw_ip]
        command += [item['url'], '-o', str(destination)]
        subprocess.run(command, check=True)
        content = destination.read_bytes()
        digest = hashlib.sha1(('blob %d\0' % len(content)).encode() + content).hexdigest()
        if digest != item['sha']:
            raise RuntimeError('Official source hash mismatch: ' + item['path'])
        return item['path']

    with ThreadPoolExecutor(max_workers=4) as pool:
        pending = [pool.submit(download, item) for item in manifest['files']]
        for count, future in enumerate(as_completed(pending), 1):
            future.result()
            if count % 10 == 0:
                print('Verified %d/%d official source files' % (count, len(pending)), flush=True)
    marker.write_text(json.dumps(manifest, indent=2))
    print('Official VINS-Fusion VIO source download complete.', flush=True)


if __name__ == '__main__':
    main()
