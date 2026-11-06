#!/usr/bin/env python3
"""Read-only real-image CPU matcher replay. No model, ROS, truth or flight IO."""
import argparse
import base64
import copy
import hashlib
import json
import resource
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from task_runtime.association import PatchMatcher, labels_compatible


def replay(result_path):
    archive = json.loads(result_path.read_text())
    command = archive.get('command', archive)
    directory = Path(archive['capture_directory'])
    matcher = PatchMatcher()
    observations, inputs, durations = [], {}, []
    for report in command.get('perceptions', []):
        identifier = report['capture_id']
        if Path(identifier).name != identifier:
            raise ValueError('Invalid capture basename')
        path = directory/(identifier+'.semantic-map.json')
        snapshot = json.loads(path.read_text())
        jpeg = directory/(identifier+'.jpg')
        raw = jpeg.read_bytes()
        metadata_path = directory/(identifier+'.json')
        metadata = json.loads(metadata_path.read_text())
        hashes = metadata['snapshot']['stereo_image_sha256']
        if hashlib.sha256(raw).hexdigest() != hashes['left']:
            raise ValueError('Archived original image hash mismatch')
        inputs[str(metadata_path)] = hashlib.sha256(metadata_path.read_bytes()).hexdigest()
        inputs[str(jpeg)] = hashlib.sha256(raw).hexdigest()
        inputs[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        for landmark in snapshot['landmarks']:
            for o in landmark['observations']:
                if o['capture_id'] != identifier:
                    continue
                o = copy.deepcopy(o)
                if o.get('image_sha256', hashes) != hashes:
                    raise ValueError('Map and original capture image identity differ')
                # Old maps predate per-target hashes; use the actual matching
                # capture archive, never fill a measurement or write it back.
                o['image_sha256'] = hashes
                start = time.perf_counter()
                matcher.remember({'image_base64': base64.b64encode(raw).decode()}, o)
                durations.append((time.perf_counter()-start)*1000)
                observations.append((landmark['label'], o))
    pairs = []
    for i, (label, a) in enumerate(observations):
        later = [(l, b) for l, b in observations[i+1:] if b['capture_id'] != a['capture_id']
                 and labels_compatible(label, l)]
        if not later:
            continue
        other_label, b = later[0]
        start = time.perf_counter()
        result = matcher.compare(a, b)
        pairs.append({'source_capture_id': a['capture_id'], 'current_capture_id': b['capture_id'],
                      'reported_labels': [label, other_label], **result,
                      'comparison_ms': (time.perf_counter()-start)*1000})
    unchanged = all(hashlib.sha256(Path(path).read_bytes()).hexdigest() == sha for path, sha in inputs.items())
    return {'schema_version': 1, 'backend': matcher.name, 'real_roi_count': len(observations),
            'real_pair_count': len(pairs), 'pairs': pairs, 'feature_extraction_total_ms': sum(durations),
            'feature_extraction_max_ms': max(durations, default=0.),
            'process_peak_rss_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            'gpu_used': False, 'weights_downloaded': False, 'input_hashes': inputs,
            'original_evidence_unchanged': unchanged, 'identity_ground_truth_available': False,
            'claims_accuracy_or_dino_equivalence': False, 'flight_or_model_calls': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay(args.result)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in result.items() if k not in ('pairs', 'input_hashes')}, ensure_ascii=False))


if __name__ == '__main__':main()
