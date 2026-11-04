"""Bearing-only geometry contracts; no provider, simulator, or flight calls."""
import copy
import json
import math
import unittest

from photo_framing import photo_bearing_feedback


def inputs(yaw=0., bbox=None):
    camera = dict(side='left', width=640, height=480, stamp_sim_s=5., image_sha256='a'*64)
    info = dict(width=640,height=480,K=[320.,0.,320.,0.,320.,240.,0.,0.,1.],
                distortion_coefficients=[0.]*5,R=[1.,0.,0.,0.,1.,0.,0.,0.,1.])
    pose = dict(frame_id='map',body_reference='base_link',stamp_sim_s=5.,
                orientation_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
    frame = dict(capture_id='image',camera=camera,snapshot=dict(side='left',stamp_sim_s=5.,
                 camera_intrinsics=info,body_pose_enu=pose))
    report = dict(status='ok',capture_id='image',camera=copy.deepcopy(camera),
                  targets=[dict(label='visible subject',bbox=bbox or [.6,.35,.9,.65])])
    return frame,report


def effects(result):
    return {(x['direction'],x['angle_deg']): x for x in result['targets'][0]['turn_bearing_effects']}


class BearingTests(unittest.TestCase):
    def test_zero_turn_recovers_box_and_does_not_change_inputs(self):
        frame,report = inputs(); saved=copy.deepcopy((frame,report))
        result=photo_bearing_feedback(frame,report)
        self.assertEqual(result['status'],'ok')
        zero=effects(result)['unchanged',0.]
        for a,b in zip(zero['projected_known_bbox_direction_bounds_normalized'],report['targets'][0]['bbox']):
            self.assertAlmostEqual(a,b)
        self.assertEqual(zero['sampled_known_bbox_bearings_inside'],9)
        self.assertEqual((frame,report),saved)

    def test_camera_right_object_moves_left_on_right_yaw_and_reverse(self):
        for box in ([.6,.35,.9,.65],[.1,.35,.4,.65]):
            rows=effects(photo_bearing_feedback(*inputs(bbox=box)))
            center=rows['unchanged',0.]['bbox_center_direction_normalized'][0]
            self.assertLess(rows['right',15.]['bbox_center_direction_normalized'][0],center)
            self.assertGreater(rows['left',15.]['bbox_center_direction_normalized'][0],center)

    def test_world_heading_and_pi_wrap_do_not_change_relative_projection(self):
        reference=photo_bearing_feedback(*inputs())['targets']
        for yaw in (math.pi,-math.pi,math.pi-.01,-math.pi+.01,math.pi/2):
            actual=photo_bearing_feedback(*inputs(yaw=yaw))['targets']
            for a,b in zip(actual[0]['turn_bearing_effects'],reference[0]['turn_bearing_effects']):
                for u,v in zip(a['bbox_center_direction_normalized'],b['bbox_center_direction_normalized']):
                    self.assertAlmostEqual(u,v)

    def test_centered_box_does_not_always_favor_right_turn(self):
        rows=effects(photo_bearing_feedback(*inputs(bbox=[.4,.4,.6,.6])))
        self.assertAlmostEqual(rows['unchanged',0.]['bbox_center_direction_normalized'][0],.5)
        for direction in ('left','right'):
            self.assertGreater(abs(rows[direction,15.]['bbox_center_direction_normalized'][0]-.5),.1)

    def test_no_depth_or_world_position_is_inferred_even_if_surface_point_exists(self):
        frame,report=inputs(); before=photo_bearing_feedback(frame,report)
        report['targets'][0]['stereo_location']=dict(status='ok',position_enu_m=[10,20,30])
        result=photo_bearing_feedback(frame,report)
        self.assertEqual(result,before)
        self.assertFalse(result['usable_for_flight_authorization'])
        self.assertFalse(result['predicts_whole_subject_complete'])
        self.assertNotIn('position_enu_m',json.dumps(result))
        self.assertIn('lever-arm',' '.join(result['limitations']))

    def test_missing_invalid_stale_or_foreign_geometry_provides_no_predictions(self):
        mutations = [lambda f,r:f['snapshot'].pop('camera_intrinsics'),
            lambda f,r:r.update(capture_id='other'),
            lambda f,r:r['camera'].update(image_sha256='b'*64),
            lambda f,r:r['camera'].update(stamp_sim_s=4.9),
            lambda f,r:f['snapshot']['body_pose_enu'].update(stamp_sim_s=4.9),
            lambda f,r:f['snapshot']['body_pose_enu'].update(body_reference='camera'),
            lambda f,r:f['snapshot']['body_pose_enu'].update(orientation_xyzw=[0,0,0,0]),
            lambda f,r:f['snapshot']['camera_intrinsics'].update(K=[True,0,320,0,320,240,0,0,1]),
            lambda f,r:f['snapshot']['camera_intrinsics'].update(K=[1e-320,0,320,0,320,240,0,0,1]),
            lambda f,r:f['snapshot']['camera_intrinsics'].update(distortion_coefficients=[.1]),
            lambda f,r:f['camera'].update(side='right'),
            lambda f,r:r.update(targets=[]),
            lambda f,r:r['targets'][0].update(bbox=[.1,0.,float('nan'),1.]),
            lambda f,r:r['targets'][0].update(bbox=[.6,0.,.4,1.])]
        for mutate in mutations:
            frame,report=inputs(); mutate(frame,report)
            with self.subTest(mutate=mutate):
                result=photo_bearing_feedback(frame,report)
                self.assertEqual(result['status'],'unavailable')
                self.assertNotIn('targets',result)
                self.assertFalse(result['usable_for_flight_authorization'])

    def test_wide_and_near_horizon_rays_do_not_emit_nan_or_claim_complete(self):
        frame,report=inputs(bbox=[0.,0.,1.,1.])
        result=photo_bearing_feedback(frame,report)
        self.assertEqual(result['status'],'ok')
        json.dumps(result,allow_nan=False)
        self.assertFalse(result['predicts_whole_subject_complete'])


if __name__=='__main__': unittest.main()
