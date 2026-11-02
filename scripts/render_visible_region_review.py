#!/usr/bin/env python3
"""Render archived VLM region hypotheses and optional temporal inliers, without calling models."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle


def render(region_files, feature_file, output):
    features = json.loads(feature_file.read_text())
    inliers = [x for x in features['matches'] if x['geometry'] is not None]
    reviews = [json.loads(path.read_text()) for path in region_files]
    if len(reviews) < 2 or [r['capture_id'] for r in reviews[:2]] != features['capture_ids']:
        raise ValueError('First two region reviews must match the temporal image pair')
    output.mkdir(parents=True, exist_ok=False)
    fig, axes = plt.subplots(1, len(reviews), figsize=(6*len(reviews), 5), squeeze=False)
    summary = []
    for j, (path, d, ax) in enumerate(zip(region_files, reviews, axes[0])):
        raw = (path.parent/'source.jpg').read_bytes()
        if hashlib.sha256(raw).hexdigest() != d['image_sha256']:
            raise ValueError('Region source image hash mismatch')
        if j < 2 and d['image_sha256'] != features['image_sha256'][j]:
            raise ValueError('Region and temporal evidence use different image bytes')
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape[:2] != (480, 640) or d['status'] != 'ok':
            raise ValueError('Invalid image or region review')
        box = d['original_bbox']; points = d['result']['polygon']
        polygon = np.array([[x*640, y*480] for x, y in points], dtype=np.float32).reshape(-1, 2)
        ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        ax.add_patch(Rectangle((box[0]*640, box[1]*480), (box[2]-box[0])*640,
                               (box[3]-box[1])*480, fill=False, edgecolor='yellow', linewidth=2))
        if len(polygon) >= 3:
            ax.add_patch(Polygon(polygon, facecolor='cyan', alpha=.25, edgecolor='cyan', linewidth=2))
        inside = []
        if j < 2:
            for index, item in enumerate(inliers):
                point = item[('source_pixel', 'current_pixel')[j]]
                hit = len(polygon) >= 3 and cv2.pointPolygonTest(polygon, tuple(point), False) >= 0
                inside.append(hit); ax.scatter(*point, c='lime' if hit else 'red', s=30)
                ax.text(point[0]+3, point[1]-3, str(index+1), color='yellow')
        area = abs(cv2.contourArea(polygon)) if len(polygon) >= 3 else 0.
        ratio = area/((box[2]-box[0])*640*(box[3]-box[1])*480)
        summary.append({'case': path.parent.name, 'model_visible': d['result']['visible'],
                        'polygon_bbox_area_ratio': round(ratio, 3),
                        'inlier_points_inside': sum(inside), 'inlier_points_total': len(inside)})
        ax.set_title(path.parent.name); ax.axis('off')
    fig.tight_layout(); fig.savefig(output/'region-overlay.png', dpi=120); plt.close(fig)
    result = {'cases': summary, 'note': 'Model polygons are hypotheses, not pixel ground truth.'}
    (output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--regions', type=Path, nargs='+', required=True)
    parser.add_argument('--features', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(render(args.regions, args.features, args.output), ensure_ascii=False, indent=2))
