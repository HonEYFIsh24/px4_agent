"""Terminal-pose geometry and uncertainty tests; all image/depth fixtures are synthetic."""
import copy
import math
import unittest

from task_runtime.contracts import GoalSpec
from task_runtime.goal_planner import GoalPosePlanner, camera_facing_yaw
from task_runtime.object_frame import estimate_object_frame
from semantic_map import SemanticMap
from goal_pose_geometry import query_goal_poses, validate_goal_context
from viewpoint_geometry import StaticViewpointEvaluator, registered_identity
from viewpoint_authority import ViewpointAuthority
from mission import MissionError
from coverage_search import project_ground_point
from test_semantic_map import navigator, observation, OWNER, EPOCH
from test_target_observation import relative_target, measured_stereo


def region(frame, point):
    p = project_ground_point(frame['snapshot']['camera_intrinsics'], frame['snapshot']['body_pose_enu'], point, 20.)['pixel_uv']
    return [p[0]/640-.035, p[1]/480-.045, p[0]/640+.035, p[1]/480+.045]


def relative_fixture():
    n = navigator()
    f, r, g = observation(n)
    f['snapshot']['body_pose_enu']['orientation_covariance_rad2'] = [[.003, 0., 0.], [0., .003, 0.], [0., 0., .003]]
    g['body_pose_enu'] = copy.deepcopy(f['snapshot']['body_pose_enu'])
    r['targets'] = [relative_target(f, [5., 0., 1.], '白色货车', [.1, .1, .9, .9])]
    m = SemanticMap(OWNER, g);m.update_observation(f, r, g)
    target = m.landmarks['target_01'];obs = target.observations[-1]
    front, rear = [5., -1., 1.], [5., 1., 1.]
    boxes = [region(f, front), region(f, rear)]
    parts = [measured_stereo(f, p) for p in (front, rear)]
    orientation = estimate_object_frame(f, obs, *boxes, parts, ['实际前脸表面（测试夹具）', '实际车尾表面（测试夹具）'])
    target.attributes.update(object_orientation=orientation, object_frame_revision=1)
    reference = {'target_id': target.id, 'capture_id': f['capture_id'], 'confirmation_level': 'multiview'}
    return n, f, m, reference, boxes, parts


class ObjectFrameTests(unittest.TestCase):
    def test_front_axis_is_derived_from_metric_parts_not_image_left_or_body_yaw(self):
        n, frame, m, ref, boxes, parts = relative_fixture()
        orientation = m.landmarks[ref['target_id']].attributes['object_orientation']
        self.assertIsNotNone(orientation)
        self.assertAlmostEqual(orientation['yaw_enu_rad'], -math.pi/2)
        self.assertEqual(orientation['front_surface_enu_m'], parts[0]['position_enu_m'])
        self.assertGreater(orientation['yaw_variance_rad2'], .003)
        self.assertFalse(orientation['uncertainty_statistics_calibrated'])
        self.assertFalse(orientation['complete_extent_known'])

    def test_missing_depth_or_pose_covariance_keeps_frame_unknown(self):
        n, f, m, ref, boxes, parts = relative_fixture();obs = m.landmarks[ref['target_id']].observations[-1]
        weak = copy.deepcopy(parts);weak[0] = {'status': 'unavailable'}
        self.assertIsNone(estimate_object_frame(f, obs, *boxes, weak, ['front', 'rear']))
        del f['snapshot']['body_pose_enu']['orientation_covariance_rad2']
        self.assertIsNone(estimate_object_frame(f, obs, *boxes, parts, ['front', 'rear']))

    def test_wrong_hash_clipped_roi_and_forged_low_covariance_are_rejected(self):
        n, f, m, ref, boxes, parts = relative_fixture();obs = m.landmarks[ref['target_id']].observations[-1]
        forged = copy.deepcopy(parts);forged[0]['covariance_camera_optical_m2'] = [[1e-12, 0., 0.], [0., 1e-12, 0.], [0., 0., 1e-12]]
        with self.assertRaises(ValueError):estimate_object_frame(f, obs, *boxes, forged, ['front', 'rear'])
        stale = copy.deepcopy(parts);stale[0]['image_sha256']['left'] = 'f'*64
        with self.assertRaises(ValueError):estimate_object_frame(f, obs, *boxes, stale, ['front', 'rear'])
        with self.assertRaises(ValueError):estimate_object_frame(f, obs, [0., .2, .3, .4], boxes[1], parts, ['front', 'rear'])


class GoalPoseTests(unittest.TestCase):
    def test_surface_visibility_uses_measured_covariance_but_requires_fov_and_clear_prefix(self):
        n, f, m, ref, *_ = relative_fixture()
        point=[5.,-1.,1.]
        n.raw_boxes.append(([4.,-1.03,0.],[6.,1.,2.]))
        # A front surface point measured 3cm inside its raw collision surface.
        candidate={'id':'goal_001','position_enu_m':[5.,-6.,3.],
                   'yaw_enu_rad':camera_facing_yaw([5.,-6.,3.],point)}
        matrix=[[.0016,0.,0.],[0.,.0016,0.],[0.,0.,.0016]]
        backend=StaticViewpointEvaluator(n,f['snapshot']['body_pose_enu'],f['snapshot']['camera_intrinsics'],EPOCH,
            probe_points_enu_m=[point],probe_covariances_enu_m2=[matrix])
        identity=registered_identity(n,EPOCH)
        result=query_goal_poses(backend,'paths',[0.,0.],identity,[candidate])['results'][0]
        self.assertTrue(result['reachable'],result)
        probe=result['probe_visibility'][0]
        self.assertFalse(probe['unoccluded'])
        self.assertTrue(probe['surface_visibility_possible'])
        self.assertAlmostEqual(probe['endpoint_uncertainty_m'],.08)
        self.assertTrue(probe['requires_fresh_visual_verification'])
        self.assertEqual(backend.probe_covariances,[matrix])
        strict=StaticViewpointEvaluator(n,f['snapshot']['body_pose_enu'],f['snapshot']['camera_intrinsics'],EPOCH,
            probe_points_enu_m=[point])
        self.assertFalse(query_goal_poses(strict,'paths',[0.,0.],identity,[candidate])['results'][0]['reachable'])
        wrong={**candidate,'yaw_enu_rad':-math.pi/2}
        self.assertFalse(query_goal_poses(backend,'paths',[0.,0.],identity,[wrong])['results'][0]['reachable'])
        n.raw_boxes.append(([4.,-3.,0.],[6.,-2.95,4.]))
        self.assertFalse(query_goal_poses(backend,'paths',[0.,0.],identity,[candidate])['results'][0]['reachable'])

    def test_surface_probe_covariance_is_optional_and_malformed_values_are_rejected(self):
        n, f, m, ref, *_ = relative_fixture()
        args=(n,f['snapshot']['body_pose_enu'],f['snapshot']['camera_intrinsics'],EPOCH)
        for covariances in ([],[[[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]]],
                             [[[.1,.2,0.],[.2,.1,0.],[0.,0.,.1]]]):
            with self.subTest(covariances=covariances),self.assertRaises(ValueError):
                StaticViewpointEvaluator(*args,probe_points_enu_m=[[5.,-1.,1.]],probe_covariances_enu_m2=covariances)
        backend=StaticViewpointEvaluator(*args,probe_points_enu_m=[[5.,-1.,1.]])
        self.assertIsNone(backend.probe_covariances)

    def test_front_goal_candidates_face_actual_front_and_keep_uncertainty(self):
        n, f, m, ref, *_ = relative_fixture()
        spec = GoalSpec(ref['target_id'], 'object_front', 'relative_hover')
        result = GoalPosePlanner().generate(spec, m, ref, f['snapshot']['body_pose_enu'], [0., 0.], 12.)
        self.assertEqual(result['status'], 'candidates')
        for c in result['candidates']:
            self.assertLess(c['position_enu_m'][1], -3.)
            self.assertAlmostEqual(c['yaw_enu_rad'], camera_facing_yaw(c['position_enu_m'], [5., -1., 1.]))
            self.assertGreater(c['score_terms']['world_position_margin_m'], 0.)
            self.assertEqual(c['hard_constraints']['path'], 'unknown')
            self.assertIsNone(c['score'])

    def test_unknown_orientation_requests_observation_and_stale_target_cannot_be_used(self):
        n, f, m, ref, *_ = relative_fixture();del m.landmarks[ref['target_id']].attributes['object_orientation']
        spec = GoalSpec(ref['target_id'], 'object_front', 'relative_hover')
        planner = GoalPosePlanner()
        missing = planner.generate(spec, m, ref, f['snapshot']['body_pose_enu'], [0., 0.], 12.)
        self.assertEqual(missing['need']['kind'], 'REVEAL_FRONT')
        m.update_observation(*observation(n, 'lost', [0., 1., 3.], 2.))
        missing = planner.generate(spec, m, ref, f['snapshot']['body_pose_enu'], [0., 0.], 12.)
        self.assertEqual(missing['need']['kind'], 'RECOVER_OCCLUDED')
        m.invalidate('actual reset')
        with self.assertRaises(ValueError):planner.generate(spec, m, ref, f['snapshot']['body_pose_enu'], [0., 0.], 12.)

    def test_goal_yaw_path_and_visibility_use_real_static_geometry_adapter(self):
        n, f, m, ref, *_ = relative_fixture()
        identity = registered_identity(n, EPOCH)
        backend = StaticViewpointEvaluator(n, f['snapshot']['body_pose_enu'], f['snapshot']['camera_intrinsics'], EPOCH,
                                          probe_points_enu_m=[[5., -1., 1.]], allow_yaw=True)
        point = [5., -6., 3.];yaw = camera_facing_yaw(point, [5., -1., 1.])
        c = {'id': 'goal_001', 'position_enu_m': point, 'yaw_enu_rad': yaw}
        good = query_goal_poses(backend, 'paths', [0., 0.], identity, [c])['results'][0]
        self.assertTrue(good['reachable'], good)
        self.assertTrue(good['target_visible'])
        bad = query_goal_poses(backend, 'paths', [0., 0.], identity, [{**c, 'yaw_enu_rad': -math.pi/2}])['results'][0]
        self.assertFalse(bad['reachable'])
        self.assertIsNone(n.last_plan)
        # Continuous target yaw is only accepted through the new versioned
        # adapter; the old NBV schema has not been loosened.
        with self.assertRaises(MissionError):backend.query('paths', [0., 0.], identity, [c])

    def test_high_score_cannot_override_collision_or_missing_facing_postcondition(self):
        n, f, m, ref, *_ = relative_fixture();planner = GoalPosePlanner()
        generated = planner.generate(GoalSpec(ref['target_id'], 'object_front', 'relative_hover'), m, ref,
                                     f['snapshot']['body_pose_enu'], [0., 0.], 12.)
        blocked = planner.evaluate(generated, lambda c, anchor: {'reachable': False, 'target_visible': True})
        self.assertEqual(blocked['status'], 'no_safe_goal')
        c = generated['candidates'][0]
        self.assertFalse(planner.verify_arrival(c, c['position_enu_m'], c['yaw_enu_rad']+.3, EPOCH))
        self.assertTrue(planner.verify_arrival(c, c['position_enu_m'], c['yaw_enu_rad'], EPOCH))
        wrong = copy.deepcopy(EPOCH);wrong['vio_epoch'] += 1
        self.assertFalse(planner.verify_arrival(c, c['position_enu_m'], c['yaw_enu_rad'], wrong))

    def test_goal_ticket_binds_stage_target_version_and_cannot_use_old_nbv_route(self):
        n, f, m, ref, *_ = relative_fixture();a = ViewpointAuthority()
        identity = registered_identity(n, EPOCH)
        context = {'owner': list(OWNER), 'map_revision': 'a'*64, 'batch_id': 'b'*32}
        metadata = {'stamp_sim_s': 1., 'body_pose_enu': f['snapshot']['body_pose_enu']}
        a.bind(context, 'pair', metadata, identity, [0., 0.]);a.advance_stage(OWNER, 'relative', 1, identity)
        # A controlled stage transition permits semantic revision on the same
        # source image, but cannot change permission centre/world/epoch.
        context.update(map_revision='c'*64, batch_id='d'*32)
        a.bind(context, 'pair', metadata, identity, [0., 0.])
        goal = {'schema_version': 1, 'stage_id': 'relative', 'stage_revision': 1,
                'plan_revision': 1, 'target_id': 'target_01', 'target_revision': 2, 'relation': 'object_front', 'terminal_action': 'relative_hover'}
        backend = StaticViewpointEvaluator(n, f['snapshot']['body_pose_enu'], f['snapshot']['camera_intrinsics'], EPOCH,
                                          probe_points_enu_m=[[5., -1., 1.]])
        c = {'id': 'goal_001', 'position_enu_m': [5., -6., 3.], 'yaw_enu_rad': math.pi/2}
        response = query_goal_poses(backend, 'paths', [0., 0.], identity, [c])
        a.issue(context, response, goal_context=goal)
        ticket = response['results'][0]['execution_ticket']
        args = (OWNER, context['batch_id'], context['map_revision'], c['id'], ticket, identity, [0., 0., 3.], 0.)
        with self.assertRaises(MissionError):a.consume(*args)
        wrong = {**goal, 'target_revision': 3}
        with self.assertRaises(MissionError):a.consume(*args, goal_context=wrong)
        accepted = a.consume(*args, goal_context=goal)
        self.assertEqual(accepted['target_yaw_enu_rad'], math.pi/2)
        self.assertEqual(accepted['goal_context'], goal)
        with self.assertRaises(MissionError):a.consume(*args, goal_context=goal)


if __name__ == '__main__':unittest.main()
