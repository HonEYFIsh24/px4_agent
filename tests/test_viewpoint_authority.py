"""Executor tickets must remain owned, single-use, fresh and tied to observed epochs."""
import copy
import math
import unittest

from mission import MissionError
from viewpoint_authority import ViewpointAuthority
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from test_semantic_map import navigator, observation, EPOCH


class AuthorityTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.authority = ViewpointAuthority(clock=lambda: self.now)
        nav = navigator()
        self.nav = nav
        frame, _, _ = observation(nav)
        snapshot = frame['snapshot']
        self.identity = registered_identity(nav, EPOCH)
        self.context = {'owner': ['thread', 'task'], 'map_revision': 'b'*64, 'batch_id': 'a'*32}
        self.metadata = {'stamp_sim_s': 1., 'body_pose_enu': snapshot['body_pose_enu']}
        self.authority.bind(self.context, 'pair-1', self.metadata, self.identity, [0., 0.])
        evaluator = StaticViewpointEvaluator(nav, snapshot['body_pose_enu'], snapshot['camera_intrinsics'], EPOCH)
        self.response = evaluator.query('paths', [0., 0.], self.identity,
            [{'id': 'view_001', 'position_enu_m': [2., 0., 3.]},
             {'id': 'view_002', 'position_enu_m': [0., 2., 3.]}])
        self.authority.issue(self.context, self.response)
        self.ticket = self.response['results'][0]['execution_ticket']

    def consume(self, **changes):
        args = dict(owner=self.context['owner'], batch_id=self.context['batch_id'],
            revision=self.context['map_revision'], candidate_id='view_001', ticket=self.ticket,
            identity=self.identity, position=[0., 0., 3.], yaw=0.)
        args.update(changes)
        return self.authority.consume(**args)

    def test_valid_ticket_resolves_cached_goal_and_invalidates_all_alternatives(self):
        result = self.consume()
        self.assertEqual(result['goal_enu_m'], [2., 0., 3.])
        self.assertEqual(result['origin_xy_enu_m'], [0., 0.])
        self.assertEqual(self.authority.tickets, {})
        with self.assertRaises(MissionError): self.consume()

    def test_cross_owner_revision_batch_candidate_pose_yaw_and_epoch_are_rejected(self):
        identity = copy.deepcopy(self.identity)
        identity['coordinate_epoch']['vio_epoch'] += 1
        for changes in ({'owner': ['thread', 'other']}, {'batch_id': 'c'*32}, {'revision': 'c'*64},
                        {'candidate_id': 'view_999'}, {'position': [.36, 0., 3.]}, {'yaw': .01},
                        {'identity': identity}, {'ticket': []}):
            with self.subTest(changes=changes):
                with self.assertRaises(MissionError): self.consume(**changes)
        self.assertEqual(self.consume()['candidate_id'], 'view_001')

    def test_expired_ticket_never_executes(self):
        self.now = 90.001
        with self.assertRaisesRegex(MissionError, 'expired'):
            self.consume()

    def test_new_observation_invalidates_prior_batch_and_cannot_rewind(self):
        context = {**self.context, 'map_revision': 'c'*64, 'batch_id': 'd'*32}
        metadata = {**self.metadata, 'stamp_sim_s': 2.}
        self.authority.bind(context, 'pair-2', metadata, self.identity, [0., 0.])
        with self.assertRaises(MissionError): self.consume()
        with self.assertRaisesRegex(MissionError, 'older'):
            self.authority.bind(self.context, 'pair-1', self.metadata, self.identity, [0., 0.])

    def test_operator_revocation_requires_a_new_task_even_after_clearing_scope(self):
        self.authority.revoke()
        with self.assertRaises(MissionError): self.consume()
        self.authority.clear()
        with self.assertRaisesRegex(MissionError, 'operator stopped'):
            self.authority.bind(self.context, 'pair-1', self.metadata, self.identity, [0., 0.])
        fresh = {**self.context, 'owner': ['thread', 'new-task']}
        self.authority.bind(fresh, 'pair-1', self.metadata, self.identity, [0., 0.])
        self.assertEqual(self.authority.constraint()['owner'], fresh['owner'])

    def test_origin_cannot_be_fabricated_or_moved_during_same_task(self):
        with self.assertRaisesRegex(MissionError, 'search origin'):
            self.authority.bind(self.context, 'pair-1', self.metadata, self.identity, [2., 0.])
        self.authority.clear()
        with self.assertRaisesRegex(MissionError, 'initial observation'):
            self.authority.bind(self.context, 'pair-1', self.metadata, self.identity, [2., 0.])

    def test_path_from_other_identity_cannot_issue_an_execution_ticket(self):
        self.response['identity']['sortie_id'] = 'another sortie'
        with self.assertRaisesRegex(MissionError, 'active search identity'):
            self.authority.issue(self.context, self.response)

    def test_heading_is_server_derived_and_bound_to_single_use_ticket(self):
        nav = self.nav
        frame, _, _ = observation(nav)
        snap = frame['snapshot']
        evaluator = StaticViewpointEvaluator(nav, snap['body_pose_enu'], snap['camera_intrinsics'], EPOCH, allow_yaw=True)
        response = evaluator.query('paths', [0., 0.], self.identity,
            [{'id': 'view_003', 'position_enu_m': [1., 0., 3.], 'heading_step': 1}])
        self.authority.issue(self.context, response)
        ticket = response['results'][0]['execution_ticket']
        response['results'][0]['target_yaw_enu_rad'] = -2.
        accepted = self.consume(candidate_id='view_003', ticket=ticket)
        self.assertAlmostEqual(accepted['target_yaw_enu_rad'], math.pi/4)
        with self.assertRaises(MissionError):
            self.consume(candidate_id='view_003', ticket=ticket)

    def test_heading_candidates_require_opt_in_and_discrete_values(self):
        nav = self.nav
        frame, _, _ = observation(nav)
        snap = frame['snapshot']
        fixed = StaticViewpointEvaluator(nav, snap['body_pose_enu'], snap['camera_intrinsics'], EPOCH)
        enabled = StaticViewpointEvaluator(nav, snap['body_pose_enu'], snap['camera_intrinsics'], EPOCH, allow_yaw=True)
        candidate = {'id': 'view_001', 'position_enu_m': [1., 0., 3.], 'heading_step': 1}
        with self.assertRaises(MissionError): fixed.query('paths', [0., 0.], self.identity, [candidate])
        for value in (True, 0, 2, 1., '1', None):
            with self.subTest(value=value), self.assertRaises(MissionError):
                enabled.query('paths', [0., 0.], self.identity, [{**candidate, 'heading_step': value}])
        with self.assertRaises(MissionError):
            enabled.query('paths', [0., 0.], self.identity, [{**candidate, 'target_yaw_enu_rad': 2.}])
