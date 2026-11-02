"""Adversarial archived execution evidence; all observations are controlled fixtures."""
import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_nbv_evidence import audit_record, snapshot_revision
from semantic_map import SemanticMap
from viewpoint_geometry import registered_identity
from test_semantic_map import navigator, observation, OWNER, EPOCH


class EvidenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.nav = navigator()
        frame, first_report, geometry = observation(self.nav)
        model = SemanticMap(OWNER, geometry, query='蓝色油桶')
        model.update_observation(frame, first_report, geometry)
        first = model.snapshot()
        goal = [2., 0., 3.]
        second_frame, second_report, second_geometry = observation(self.nav, 'capture-2', goal, 2.)
        model.update_observation(second_frame, second_report, second_geometry)
        self.snapshots = {'capture-1': first, 'capture-2': model.snapshot()}
        identity = registered_identity(self.nav, EPOCH)
        revision = snapshot_revision(first)
        scope = {'owner': list(OWNER), 'identity': identity, 'origin_xy_enu_m': [0., 0.],
                 'search_radius_m': 12., 'map_revision': revision, 'batch_id': 'b'*32, 'revoked': False}
        plan = self.nav.preview_path([0., 0., 3.], goal, [0., 0.])
        mission = {'id': 'ticket-flight', 'status': 'running', 'search_constraint': scope,
                   'viewpoint_candidate_id': 'view_001',
                   'steps': [{'action': 'goto', 'x': 2., 'y': 0., 'altitude': 3.}]}
        finished = {**copy.deepcopy(mission), 'status': 'succeeded',
                    'events': [{'action': 'goto', 'position_enu_m': goal}],
                    'navigation_plans': [{'goal_enu_m': goal, 'plan': plan}]}
        event = {'owner': list(OWNER), 'batch_id': 'b'*32, 'map_revision': revision,
                 'source_capture_id': 'capture-1', 'identity': identity,
                 'reference': {'start_enu_m': [0., 0., 3.]},
                 'search_mode': 'nbv_geometry', 'config': {'mode': 'geometry'},
                 'selected_id': 'view_001', 'ranked_candidates': [{'id': 'view_001',
                    'position_enu_m': goal, 'path_verified': True, 'coverage_gain_m2': 10.}]}
        self.record = {'flight': {'armed': False, 'landed_state': 1}, 'command': {
            'id': OWNER[1], 'status': 'succeeded', 'nbv_events': [event],
            'perceptions': [first_report, second_report],
            'flight_missions': [{'accepted_via': '/navigation/execute-viewpoint', 'mission': mission}],
            'flight_mission_results': [{'mission': finished}]}}

    def audit(self, safe=None):
        return audit_record(self.record, self.snapshots.__getitem__,
                            safe or (lambda identity, a, b: self.nav.segment_safe(a, b)))

    def test_owned_execution_has_measurable_geometric_gain_and_planned_length(self):
        result = self.audit()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['moves'][0]['actual_planned_length_m'], 2.)
        self.assertGreater(result['moves'][0]['realized_geometric_gain_m2'], 0.)
        self.assertFalse(result['semantic_labels_independently_verified'])

    def test_heading_candidate_requires_owned_rotation_and_later_actual_image(self):
        event = self.record['command']['nbv_events'][0]
        event['config']['heading_mode'] = 'discrete'
        event['reference']['fixed_yaw_enu_rad'] = 0.
        event['ranked_candidates'][0].update(heading_step=1, target_yaw_enu_rad=math.pi/4)
        for key in ('flight_missions', 'flight_mission_results'):
            self.record['command'][key][0]['mission']['steps'].append({'action': 'rotate', 'yaw_rad': math.pi/4})
        self.record['command']['flight_mission_results'][0]['mission']['events'].append(
            {'action': 'rotate', 'rotation': {'status': 'reached', 'target_rad': math.pi/4,
                                              'actual_rad': math.pi/4, 'stamp_s': 1.5}})
        observation = self.snapshots['capture-2']['observations'][-1]
        observation['orientation_xyzw'] = [0., 0., math.sin(math.pi/8), math.cos(math.pi/8)]
        self.assertTrue(self.audit()['passed'], self.audit())
        for change in ('old_image', 'wrong_heading', 'tampered_goal', 'missing_arrival'):
            with self.subTest(change=change):
                record, snapshots = copy.deepcopy(self.record), copy.deepcopy(self.snapshots)
                if change == 'old_image': snapshots['capture-2']['observations'][-1]['stamp_sim_s'] = 1.4
                if change == 'wrong_heading': snapshots['capture-2']['observations'][-1]['orientation_xyzw'] = [0., 0., 0., 1.]
                if change == 'tampered_goal': record['command']['nbv_events'][0]['ranked_candidates'][0]['target_yaw_enu_rad'] = 1.
                if change == 'missing_arrival': record['command']['flight_mission_results'][0]['mission']['events'].pop()
                result = audit_record(record, snapshots.__getitem__, lambda identity, a, b: self.nav.segment_safe(a, b))
                self.assertFalse(result['passed'])

    def test_prose_success_or_landing_without_nbv_movement_cannot_pass(self):
        self.record['command']['nbv_events'] = []
        result = self.audit()
        self.assertFalse(result['passed'])
        self.assertFalse(result['checks']['has_executed_nbv_move'])

    def test_semantic_mode_requires_nonzero_field_and_score_before_executed_move(self):
        def audit():
            return audit_record(self.record, self.snapshots.__getitem__,
                lambda identity, a, b: self.nav.segment_safe(a, b), require_semantic_guidance=True)
        self.assertFalse(audit()['passed'])
        event = self.record['command']['nbv_events'][0]
        event.update(search_mode='nbv_semantic', config={'mode': 'semantic'})
        event['ranked_candidates'][0].update(semantic_gain_m2=2., score_terms={'semantic': .12})
        # Selecting semantic mode alone does not establish prior semantic evidence.
        self.assertFalse(audit()['passed'])
        self.snapshots['capture-1']['semantic_field_active'] = True
        revision = snapshot_revision(self.snapshots['capture-1'])
        event['map_revision'] = revision
        for name in ('flight_missions', 'flight_mission_results'):
            self.record['command'][name][0]['mission']['search_constraint']['map_revision'] = revision
        result = audit()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['semantic_guided_move_count'], 1)

    def test_missing_or_replaced_completion_cannot_pass(self):
        for change in ('missing', 'wrong_id', 'failed', 'duplicate'):
            with self.subTest(change=change):
                self.setUp()
                entries = self.record['command']['flight_mission_results']
                if change == 'missing': entries.clear()
                elif change == 'wrong_id': entries[0]['mission']['id'] = 'another-task'
                elif change == 'failed': entries[0]['mission']['status'] = 'failed'
                else: entries.append(copy.deepcopy(entries[0]))
                self.assertFalse(self.audit()['passed'])

    def test_false_safe_flag_cannot_hide_out_of_radius_path_or_collision(self):
        plan = self.record['command']['flight_mission_results'][0]['mission']['navigation_plans'][0]['plan']
        plan['waypoints'].insert(0, [13., 0., 3.])
        self.assertFalse(self.audit()['passed'])
        self.setUp()
        self.assertFalse(self.audit(safe=lambda *args: False)['passed'])

    def test_owner_epoch_revision_mode_and_route_length_changes_cannot_pass(self):
        for change in ('owner', 'epoch', 'revision', 'mode', 'length', 'ticket', 'generic'):
            with self.subTest(change=change):
                self.setUp()
                command = self.record['command']
                event = command['nbv_events'][0]
                done = command['flight_mission_results'][0]['mission']
                if change == 'owner': event['owner'][1] = 'another-task'
                elif change == 'epoch': done['search_constraint']['identity']['coordinate_epoch']['vio_epoch'] += 1
                elif change == 'revision': self.snapshots['capture-1']['cells'][0]['coverage'] = 1.
                elif change == 'mode': event['search_mode'] = 'nbv_semantic'
                elif change == 'length': done['navigation_plans'][0]['plan']['path_length_m'] += 1.
                elif change == 'ticket': event['ranked_candidates'][0]['execution_ticket'] = 'private-token'
                else: command['flight_missions'][0]['accepted_via'] = '/mission'
                self.assertFalse(self.audit()['passed'])

    def test_repeated_capture_wrong_view_and_nonmonotonic_map_do_not_count_as_progress(self):
        for change in ('repeat', 'wrong_view', 'lost_history', 'shrink'):
            with self.subTest(change=change):
                self.setUp()
                after = self.snapshots['capture-2']
                if change == 'repeat': self.record['command']['perceptions'][1]['capture_id'] = 'capture-1'
                elif change == 'wrong_view': after['observations'][-1]['position_enu_m'] = [8., 0., 3.]
                elif change == 'lost_history': after['observations'].pop(0)
                else:
                    first = self.snapshots['capture-1']
                    index = next(i for i, c in enumerate(first['cells']) if c['coverage'] > 0)
                    after['cells'][index]['coverage'] = 0.
                self.assertFalse(self.audit()['passed'])


if __name__ == '__main__':
    unittest.main()
