"""Conflicting depth is not evidence of a second object or permission to fuse."""
import copy
import unittest

from landmark_association import duplicate_hypotheses, display_groups
from semantic_map import SemanticMap, point_in_box
from target_confirmation import pair_geometry, CONFIRMATION_GEOMETRY_POLICY
from test_semantic_map import navigator, observation, target, OWNER


def conflicting_map(extra_source=False):
    # Controlled approximation of the archived tent failure; no scene truth.
    n = navigator()
    f, r, g = observation(n)
    r['targets'] = [target(f, [7., -2., .7], '橙色帐篷', [.52, .40, .74, .58])]
    if extra_source:
        r['targets'].append(target(f, [8., -2., .7], '橙色帐篷', [.52, .40, .74, .58]))
    m = SemanticMap(OWNER, g, query='橙色帐篷')
    m.update_observation(f, r, g)
    f, r, g = observation(n, 'second', [-5.7, 0., 1.55], 2.)
    r['targets'] = [target(f, [3.9, -1.8, -.13], '橙色帐篷', [.52, .32, .66, .44])]
    m.update_observation(f, r, g)
    return m


class DuplicateHypothesisTests(unittest.TestCase):
    def test_conflicting_surfaces_share_only_a_display_hypothesis(self):
        m = conflicting_map()
        before = copy.deepcopy(m.memory.reports())
        snapshot = m.snapshot()
        hypotheses = snapshot['landmark_association_hypotheses']
        self.assertEqual(len(hypotheses), 1)
        h = hypotheses[0]
        self.assertEqual(h['member_ids'], ['target_01', 'target_02'])
        self.assertEqual(h['status'], 'possible_duplicate')
        self.assertTrue(h['evidence']['source_surface_in_current_box'])
        self.assertFalse(h['evidence']['current_surface_in_source_box'])
        self.assertFalse(h['identity_confirmed'])
        self.assertFalse(h['metric_evidence_merged'])
        self.assertFalse(h['usable_for_confirmation'])
        self.assertEqual(len(display_groups(snapshot)), 1)
        self.assertEqual(m.memory.reports(), before)
        self.assertEqual(len(m.landmarks), 2)
        self.assertFalse(snapshot['landmark_count_is_object_count'])
        a, b = m.landmarks.values()
        geometry = pair_geometry(a, a.observations[0], b, b.observations[0], CONFIRMATION_GEOMETRY_POLICY)
        self.assertFalse(geometry['compatible'], 'display grouping must never upgrade confirmation')

    def test_two_plausible_sources_remain_ambiguous_not_a_transitive_object(self):
        m = conflicting_map(extra_source=True)
        h = m.snapshot()['landmark_association_hypotheses']
        self.assertEqual(len(h), 2)
        self.assertTrue(all(x['status'] == 'ambiguous' for x in h))
        self.assertEqual(display_groups(m.snapshot()), [])
        self.assertEqual(len(m.memory.targets), 3)

    def test_same_frame_different_class_and_no_projection_are_not_grouped(self):
        m = conflicting_map()
        a, b = m.landmarks.values()
        for variant in ('same_frame', 'different_class', 'distant', 'no_baseline', 'unlocalized'):
            with self.subTest(variant=variant):
                x, y = copy.deepcopy((a, b))
                if variant == 'same_frame': y.observations[0]['capture_id'] = x.observations[0]['capture_id']
                if variant == 'different_class': y.label = '蓝色帐篷'
                if variant == 'distant': y.observations[0]['position_enu_m'] = [20., 0., 0.]
                if variant == 'no_baseline': y.observations[0]['body_pose_enu'] = x.observations[0]['body_pose_enu']
                if variant == 'unlocalized': y.observations[0]['position_enu_m'] = None
                self.assertEqual(duplicate_hypotheses([x, y], point_in_box), [])
        self.assertEqual(duplicate_hypotheses([a, b], lambda p, o: False), [])
        self.assertEqual(duplicate_hypotheses([a, b], lambda p, o: True), [],
                         'do not resolve ambiguous strong associations through a weak display link')

    def test_later_cooccurrence_removes_hypothesis_instead_of_permanent_alias(self):
        m = conflicting_map()
        a, b = m.landmarks.values()
        # Same image seeing both is counter-evidence: the link must disappear.
        x = copy.deepcopy(a.observations[0]); x['capture_id'] = 'later'
        y = copy.deepcopy(b.observations[0]); y['capture_id'] = 'later'
        a.observations.append(x); b.observations.append(y)
        self.assertEqual(m.snapshot()['landmark_association_hypotheses'], [])

    def test_invalid_map_hides_links_and_old_snapshots_still_render(self):
        m = conflicting_map(); snapshot = m.snapshot()
        old = copy.deepcopy(snapshot); old.pop('landmark_association_hypotheses')
        self.assertEqual(display_groups(old), [])
        m.invalidate('coordinate epoch changed')
        self.assertEqual(m.snapshot()['landmark_association_hypotheses'], [])
        for field in ('identity_confirmed', 'metric_evidence_merged', 'usable_for_confirmation'):
            forged = copy.deepcopy(snapshot)
            forged['landmark_association_hypotheses'][0][field] = True
            self.assertEqual(display_groups(forged), [])


if __name__ == '__main__': unittest.main()
