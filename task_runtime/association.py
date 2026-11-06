"""CPU image-patch correspondence plus metric/bearing gates, not DINO.

Pixels are read only from hash-verified acquired JPEGs. Appearance is an identity
hint, never a source of depth, semantic confirmation or flight permission.
Thresholds below are conservative engineering defaults, not calibrated scores.
"""
import base64
import hashlib
import io
import math
import re
from collections import OrderedDict

import numpy as np
from PIL import Image


ALIASES = {
    'van': ('厢式货车', '厢式车', '货车', '面包车', 'van', 'delivery truck', 'cargo truck'),
    'tent': ('帐篷', 'tent'), 'barrel': ('油桶', '圆桶', 'barrel', 'drum'),
    'container': ('集装箱', 'container'), 'cone': ('交通锥', '路锥', 'cone'),
}
COLORS = {'white': ('白色', 'white'), 'red': ('红色', 'red'), 'blue': ('蓝色', 'blue'),
          'orange': ('橙色', 'orange'), 'yellow': ('黄色', 'yellow'),
          'black': ('黑色', 'black'), 'green': ('绿色', 'green'),
          'gray': ('灰色', 'gray', 'grey')}


def semantic_key(label, attributes=None):
    normalized = ' '.join(str(label).casefold().strip().split())
    category = next((key for key, names in ALIASES.items() if any(
        name in normalized if not name.isascii() else re.search(r'\b'+re.escape(name)+r'\b', normalized)
        for name in names)), normalized)
    color_text = normalized+' '+str((attributes or {}).get('color', '')).casefold()
    colors = {key for key, names in COLORS.items() if any(name in color_text for name in names)}
    return category, colors


def labels_compatible(a, b, attributes_a=None, attributes_b=None):
    ka, ca = semantic_key(a, attributes_a)
    kb, cb = semantic_key(b, attributes_b)
    return ka == kb and not (ca and cb and ca.isdisjoint(cb))


class PatchMatcher:
    """Harris-like corners, normalized local patches, reciprocal matches + affine consensus.

    Small fixed ROIs and a bounded cache keep this off the flight thread. No
    downloaded weights, fabricated embeddings or feature vectors in model context.
    """
    name = 'cpu_harris_patch_affine_v1'

    def __init__(self, capacity=192):
        self.cache, self.capacity = OrderedDict(), capacity

    @staticmethod
    def key(observation):
        return (observation['image_sha256']['left'], tuple(observation['bbox']))

    def remember(self, frame, observation):
        key = self.key(observation)
        if key in self.cache:
            return
        encoded = frame.get('image_base64')
        if not encoded:
            return  # Controlled/old archives without pixels have no fake features.
        raw = base64.b64decode(encoded, validate=True)
        if hashlib.sha256(raw).hexdigest() != key[0]:
            raise ValueError('Association JPEG hash differs from acquired frame')
        box = observation['bbox']
        with Image.open(io.BytesIO(raw)) as image:
            if image.size != (640, 480):
                raise ValueError('Association expects calibrated original 640x480 image')
            region = (round(box[0]*640), round(box[1]*480), round(box[2]*640), round(box[3]*480))
            crop = image.convert('L').crop(region)
            if min(crop.size) < 20:
                features = None
            else:
                # Never upsample a tiny object to manufacture texture.
                crop.thumbnail((128, 128), Image.Resampling.BILINEAR)
                features = self._features(np.asarray(crop, dtype=np.float32)/255.)
        self.cache[key] = features
        while len(self.cache) > self.capacity:
            self.cache.popitem(last=False)

    @staticmethod
    def _features(gray):
        gy, gx = np.gradient(gray)
        # Local structure tensor over 3x3 neighborhoods.
        def boxsum(a):
            padded = np.pad(a, 1, mode='edge')
            return sum(padded[y:y+a.shape[0], x:x+a.shape[1]] for y in range(3) for x in range(3))
        xx, yy, xy = boxsum(gx*gx), boxsum(gy*gy), boxsum(gx*gy)
        response = xx*yy-xy*xy-.04*(xx+yy)**2
        response[:6] = response[-6:] = 0
        response[:, :6] = response[:, -6:] = 0
        if response.max() <= 1e-7:
            return None
        points, descriptors = [], []
        for flat in np.argsort(response, axis=None)[::-1]:
            y, x = np.unravel_index(flat, response.shape)
            if response[y, x] < max(1e-7, response.max()*.02):
                break
            if any((x-u)**2+(y-v)**2 < 25 for u, v in points):
                continue
            patch = gray[y-4:y+5, x-4:x+5].reshape(-1)
            descriptor = patch-patch.mean()
            length = np.linalg.norm(descriptor)
            if length < .08:
                continue
            points.append((x, y));descriptors.append(descriptor/length)
            if len(points) == 64:
                break
        if len(points) < 8:
            return None
        return (np.asarray(points, dtype=float)/[gray.shape[1], gray.shape[0]], np.asarray(descriptors))

    def compare(self, a, b):
        fa, fb = self.cache.get(self.key(a)), self.cache.get(self.key(b))
        result = {'backend': self.name, 'status': 'unavailable', 'supports_identity': False,
                  'calibrated_probability': False}
        if fa is None or fb is None:
            return result
        if self.key(a)[0] == self.key(b)[0]:
            return {**result, 'reason': 'same_original_is_not_an_independent_view'}
        pa, da = fa;pb, db = fb
        distance = np.maximum(0., 2.-2.*da@db.T)
        order = np.argsort(distance, axis=1)
        pairs = [(i, int(ids[0])) for i, ids in enumerate(order)
                 if distance[i, ids[0]] < .6 and distance[i, ids[0]] < .65*distance[i, ids[1]]
                 and int(np.argmin(distance[:, ids[0]])) == i]
        if len(pairs) < 8:
            return {**result, 'status': 'insufficient', 'matches': len(pairs)}
        x = np.array([pa[i] for i, _ in pairs]);y = np.array([pb[j] for _, j in pairs])
        design = np.column_stack((x, np.ones(len(x))))
        # Deterministic bounded consensus: at most 80 well-spread triplets.
        best = 0
        from itertools import combinations
        for count, triple in enumerate(combinations(range(len(pairs)), 3)):
            if count >= 80:
                break
            d = design[list(triple)]
            if abs(np.linalg.det(d)) < .002:
                continue
            affine = np.linalg.solve(d, y[list(triple)])
            if not .08 <= abs(np.linalg.det(affine[:2])) <= 12.:
                continue
            best = max(best, int(np.sum(np.linalg.norm(design@affine-y, axis=1) < .065)))
        clipped = any(a['bbox'][i] <= .005 or b['bbox'][i] <= .005 for i in (0, 1)) or any(
            a['bbox'][i] >= .995 or b['bbox'][i] >= .995 for i in (2, 3))
        supported = best >= 8 and best/len(pairs) >= .6 and not clipped
        return {**result, 'status': 'matched' if supported else 'hypothesis', 'matches': len(pairs),
                'affine_inliers': best, 'clipped_roi': clipped, 'supports_identity': supported}


class CrossViewAssociation:
    def __init__(self):
        self.matcher = PatchMatcher()

    def decide(self, label, attributes, observation, landmarks):
        """Unique joint evidence only. Ambiguous alternatives stay separate."""
        from semantic_map import point_in_box
        from target_confirmation import pair_geometry, CONFIRMATION_GEOMETRY_POLICY
        candidates = []
        for landmark in landmarks.values():
            if not labels_compatible(label, landmark.label, attributes,
                                     landmark.attributes.get('reported_attributes')):
                continue
            if any(o['capture_id'] == observation['capture_id'] for o in landmark.observations):
                continue
            for old in landmark.observations[-3:]:
                dt = observation['stamp_sim_s']-old['stamp_sim_s']
                if not 0 < dt <= 120. or observation['coordinate_epoch'] != old['coordinate_epoch']:
                    continue
                if old['image_sha256']['left'] == observation['image_sha256']['left']:
                    continue
                a, b = old.get('position_enu_m'), observation.get('position_enu_m')
                if a is not None and b is not None:
                    geometric = {'compatible': math.dist(a, b) <= 6. and point_in_box(a, observation)
                                 and point_in_box(b, old), 'association_basis': 'stereo_unique'}
                else:
                    geometric = pair_geometry(landmark, old, landmark, observation, CONFIRMATION_GEOMETRY_POLICY)
                appearance = self.matcher.compare(old, observation)
                # Failure to match texture is not evidence of different identity
                # (e.g. two valid views of a plain white vehicle). Preserve the
                # existing unique reciprocal two-depth geometric criterion;
                # appearance is required to link weaker bearing-only evidence.
                metric_only = a is not None and b is not None
                eligible = geometric['compatible'] and (appearance['supports_identity'] or metric_only)
                candidates.append({'target_id': landmark.id, 'source_capture_id': old['capture_id'],
                    'geometry': geometric, 'appearance': appearance, 'eligible': bool(eligible)})
        eligible = {c['target_id'] for c in candidates if c['eligible']}
        return {'target_id': next(iter(eligible)) if len(eligible) == 1 else None,
                'status': 'unique' if len(eligible) == 1 else 'ambiguous' if len(eligible) > 1 else 'unresolved',
                'candidates': candidates[-24:], 'appearance_alone_authorizes_navigation': False}
