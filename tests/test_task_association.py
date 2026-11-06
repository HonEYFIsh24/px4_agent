"""Association uses real pixel operations here; flight/model remain offline fixtures."""
import base64
import copy
import hashlib
import io
import unittest

import numpy as np
from PIL import Image

from task_runtime.association import PatchMatcher, labels_compatible
from task_runtime.memory import SharedTargetMemory
from test_semantic_map import observation, target, navigator, OWNER
from semantic_map import SemanticMap


def pixels(frame, geometry, image):
    buffer = io.BytesIO();image.save(buffer, 'JPEG', quality=95)
    raw = buffer.getvalue();sha = hashlib.sha256(raw).hexdigest()
    frame['image_base64'] = base64.b64encode(raw).decode()
    frame['snapshot']['stereo_image_sha256']['left'] = sha
    geometry['image_sha256']['left'] = sha


def textured(seed=17):
    # Deterministic test texture, NOT visual recognition or a Gazebo photograph.
    rng = np.random.default_rng(seed)
    small = Image.fromarray(rng.integers(0, 255, (60, 80), dtype=np.uint8)).convert('RGB')
    return small.resize((640, 480), Image.Resampling.BILINEAR)


class PixelAssociationTests(unittest.TestCase):
    def test_real_patch_matching_hash_cache_and_nonmatching_image(self):
        matcher = PatchMatcher()
        n = navigator()
        a, _, ga = observation(n)
        b, _, gb = observation(n, 'new-view', [0., 1., 3.], 2.)
        pixels(a, ga, textured());pixels(b, gb, textured())
        # Distinct original bytes simulate an exposure change, not a new independent image claim.
        image = textured();arr = np.asarray(image).copy();arr[0, 0] = [255, 0, 0]
        pixels(b, gb, Image.fromarray(arr))
        oa = {'bbox': [.2, .2, .8, .8], 'image_sha256': a['snapshot']['stereo_image_sha256']}
        ob = {'bbox': [.2, .2, .8, .8], 'image_sha256': b['snapshot']['stereo_image_sha256']}
        matcher.remember(a, oa);matcher.remember(b, ob)
        match = matcher.compare(oa, ob)
        self.assertTrue(match['supports_identity'], match)
        c, _, gc = observation(n, 'other', [0., 2., 3.], 3.)
        pixels(c, gc, textured(21))
        oc = {**ob, 'image_sha256': c['snapshot']['stereo_image_sha256']}
        matcher.remember(c, oc)
        self.assertFalse(matcher.compare(oa, oc)['supports_identity'])
        corrupt = copy.deepcopy(a);corrupt['image_base64'] = b['image_base64']
        with self.assertRaises(ValueError):PatchMatcher().remember(corrupt, oa)

    def test_clipping_no_texture_and_duplicate_original_never_supply_independent_identity(self):
        matcher = PatchMatcher();n = navigator()
        f, _, g = observation(n);pixels(f, g, Image.new('RGB', (640, 480), 'white'))
        o = {'bbox': [0., .2, .8, .8], 'image_sha256': f['snapshot']['stereo_image_sha256']}
        matcher.remember(f, o)
        self.assertFalse(matcher.compare(o, o)['supports_identity'])
        self.assertEqual(matcher.compare(o, o)['status'], 'unavailable')

    def test_aliases_preserve_category_attributes_without_merging_all_white_vehicles(self):
        self.assertTrue(labels_compatible('货车', '白色厢式货车'))
        self.assertTrue(labels_compatible('橙色帐篷', 'orange tent'))
        self.assertFalse(labels_compatible('白色货车', '蓝色货车'))
        self.assertFalse(labels_compatible('白色货车', '白色集装箱'))

    def test_explicit_gray_vehicle_stays_in_map_but_is_not_a_white_target_candidate(self):
        from target_confirmation import observations_for
        for label in ('灰色厢式货车', 'gray van', 'grey delivery truck'):
            with self.subTest(label=label):
                n = navigator();f, r, g = observation(n)
                r['targets'] = [target(f, [5.,0.,1.], label)]
                m = SemanticMap(OWNER, g);m.enable_joint_association();m.update_observation(f, r, g)
                self.assertEqual(len(m.landmarks), 1)
                self.assertEqual(observations_for(m, '白色货车', f['capture_id']), [])
                self.assertEqual(SharedTargetMemory(m).matching('white van'), [])
                self.assertEqual(len(observations_for(m, '货车', f['capture_id'])), 1)
                self.assertTrue(labels_compatible('货车', '白色货车'))  # Unknown remains unknown.
                self.assertTrue(labels_compatible(label, '灰色货车'))

    def test_bearing_promoted_to_metric_keeps_id_actual_observations_and_uncertainty(self):
        n = navigator();f, r, g = observation(n)
        pixels(f, g, textured())
        r['targets'] = [{'label': '货车', 'bbox': [.2, .2, .8, .8]}]
        m = SemanticMap(OWNER, g);m.enable_joint_association();m.update_observation(f, r, g)
        identifier = next(iter(m.landmarks))
        f2, r2, g2 = observation(n, 'new', [0., 1., 3.], 2.)
        image = np.asarray(textured()).copy();image[0, 0] = [255, 0, 0]
        pixels(f2, g2, Image.fromarray(image))
        r2['targets'] = [target(f2, [5., 0., 1.], '白色厢式货车', [.2, .2, .8, .8])]
        m.update_observation(f2, r2, g2)
        self.assertEqual(list(m.landmarks), [identifier])
        view = SharedTargetMemory(m).view(identifier)
        self.assertEqual(view['target_revision'], 2)
        self.assertEqual(view['observation_refs'], ['capture-1', 'new'])
        self.assertEqual(m.landmarks[identifier].status, 'candidate')
        self.assertEqual(m.memory.targets[0]['measurement_count'], 1)
        f3, r3, g3 = observation(n, 'occluded', [0., 2., 3.], 3.)
        m.update_observation(f3, r3, g3)
        lost = SharedTargetMemory(m).view(identifier)
        self.assertEqual(lost['visibility'], 'lost')
        self.assertEqual(lost['missed_observations'], 1)
        self.assertEqual(lost['target_id'], identifier)
        m.invalidate('epoch reset')
        with self.assertRaises(ValueError):SharedTargetMemory(m).view(identifier)

    def test_same_class_cooccurrence_and_ambiguous_reappearance_do_not_merge(self):
        n = navigator();f, r, g = observation(n)
        r['targets'] = [target(f, [5., 0., 1.], '白色货车'), target(f, [5., 1., 1.], '白色货车')]
        m = SemanticMap(OWNER, g);m.enable_joint_association();m.update_observation(f, r, g)
        self.assertEqual(len(m.landmarks), 2)
        f2, r2, g2 = observation(n, 'ambiguous', [0., 1., 3.], 2.)
        r2['targets'] = [target(f2, [5., .5, 1.], '货车')]
        m.update_observation(f2, r2, g2)
        self.assertEqual(len(m.landmarks), 3)
        association = list(m.landmarks.values())[-1].observations[-1]['association']
        self.assertEqual(association['status'], 'ambiguous')

    def test_strong_review_can_link_bearing_to_metric_without_inventing_a_metric_view(self):
        n = navigator();f, r, g = observation(n)
        r['targets'] = [{'label': '白色货车', 'bbox': [.1, .1, .9, .9]}]
        m = SemanticMap(OWNER, g);m.enable_joint_association();m.update_observation(f, r, g)
        stable = next(iter(m.landmarks))
        f2, r2, g2 = observation(n, 'metric', [0., 1., 3.], 2.)
        r2['targets'] = [target(f2, [5., 0., 1.], '白色厢式货车')]
        m.update_observation(f2, r2, g2)
        metric_id = next(l.id for l in m.landmarks.values() if l.position_enu_m is not None)
        proof = {'compatible': True, 'association_basis': 'single_depth_projection'}
        review = {'source_supported': True, 'current_supported': True, 'same_object': True}
        with self.assertRaises(ValueError):m.link_reviewed_instance(stable, metric_id, 'capture-1', 'metric', proof,
            {**review, 'source_supported': False})
        self.assertEqual(len(m.landmarks), 2)
        actual = m.link_reviewed_instance(stable, metric_id, 'capture-1', 'metric', proof, review)
        self.assertEqual(actual, stable)
        self.assertEqual(list(m.landmarks), [stable])
        self.assertEqual(m.memory.targets[0]['measurement_count'], 1)
        self.assertEqual(m.memory.targets[0]['distinct_viewpoint_count'], 1)
        self.assertEqual(m.landmarks[stable].status, 'candidate')
        self.assertEqual(SharedTargetMemory(m).view(stable)['observation_refs'], ['capture-1', 'metric'])


if __name__ == '__main__':unittest.main()
