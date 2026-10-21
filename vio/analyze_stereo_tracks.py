#!/usr/bin/env python3
"""Calibrate real tracked-stereo window statistics; no truth enters the fit."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def quantile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))]


def analyze(path):
    raw = []
    for line in path.read_text().splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get('kind') == 'native_health' and record.get('value', {}).get('stereo_tracks'):
            raw.append(record['value'])
    histories = {}
    fits = []
    previous_stamp = None
    for row in raw:
        stamp = row['stamp']
        if previous_stamp is not None and (stamp <= previous_stamp or stamp - previous_stamp > .200001):
            histories = {}
        previous_stamp = stamp
        current, flows, jitter = {}, [], []
        for track in row['stereo_tracks']:
            samples = histories.get(track['id'], []) + [(stamp, track['left_uv'], track['right_uv'])]
            samples = [s for s in samples if s[0] >= stamp - 1 - 1e-6]
            current[track['id']] = samples
            if len(samples) < 8 or samples[-1][0] - samples[0][0] < .98:
                continue
            mean_t = sum(s[0] - stamp for s in samples) / len(samples)
            tt = sum((s[0] - stamp - mean_t) ** 2 for s in samples)
            speeds, residuals = [], []
            for eye in (1, 2):
                mean = [sum(s[eye][d] for s in samples) / len(samples) for d in (0, 1)]
                slope = [sum((s[0] - stamp - mean_t) * (s[eye][d] - mean[d]) for s in samples) / tt for d in (0, 1)]
                speeds.append(math.hypot(*slope))
                residuals.append(math.sqrt(sum(sum((s[eye][d] - mean[d] - (s[0] - stamp - mean_t) * slope[d]) ** 2
                                                        for d in (0, 1)) for s in samples) / len(samples)))
            flows.append(max(speeds))
            jitter.append(max(residuals))
        histories = current
        if flows:
            fits.append({'stamp': stamp, 'supported_tracks': len(flows), 'flow_p90_px_s': quantile(flows, .9),
                         'flow_median_px_s': quantile(flows, .5), 'fit_jitter_p90_px': quantile(jitter, .9),
                         'instant_flow_p90_px_s': row['flow_p90_px_s'],
                         'instant_flow_median_px_s': row['flow_median_px_s']})
    statistics = {}
    if fits:
        for key in fits[0]:
            values = [row[key] for row in fits]
            statistics[key] = {'minimum': min(values), 'median': quantile(values, .5),
                               'p95': quantile(values, .95), 'maximum': max(values)}
    passed = [row for row in fits if row['supported_tracks'] >= 20 and row['flow_p90_px_s'] <= .5
              and row['flow_median_px_s'] <= .25 and row['fit_jitter_p90_px'] <= .06
              and row['instant_flow_p90_px_s'] <= 2 and row['instant_flow_median_px_s'] <= .75]
    return {'source': str(path.resolve()), 'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'native_frames': len(raw), 'complete_window_frames': len(fits), 'visual_gate_pass_frames': len(passed),
            'source_semantics': 'actual_feature_measurement_stamps_raw_two_eye_pixels_no_ground_truth_in_fit',
            'window_s': 1, 'minimum_span_s': .98, 'minimum_samples': 8,
            'minimum_near_stereo_tracks': 20, 'maximum_flow_p90_px_s': .5, 'maximum_flow_median_px_s': .25,
            'maximum_fit_jitter_p90_px': .06, 'maximum_instant_flow_p90_px_s': 2,
            'maximum_instant_flow_median_px_s': .75, 'statistics': statistics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = analyze(args.samples)
    encoded = json.dumps(result, indent=2) + '\n'
    if args.output:
        # Evidence artifacts must not silently replace a previous analysis.
        with args.output.open('x') as stream:
            stream.write(encoded)
    print(encoded, end='')


if __name__ == '__main__':
    main()
