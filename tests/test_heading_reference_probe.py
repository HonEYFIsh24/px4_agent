"""Ground-only heading evidence is never a flight authorization."""
import copy
import math
import unittest
from vio.heading_reference_probe import GroundHeadingProbe


def sample(t):
    return {'clock_sim_s': t+.02, 'visual_stamp_sim_s': t, 'pose_stamp_sim_s':t-.01,
            'visual_frame':'vio_local','visual_child_frame':'base_link','pose_frame':'map',
            'visual_q_xyzw':[0.,0.,0.,1.], 'pose_q_xyzw':[0.,0.,math.sin(.4),math.cos(.4)],
            'visual_yaw_variance':.01}


def state(t):
    return {'simulation_only':True,'ready':True,'connected':True,'armed':False,'landed_state':1,
            'scene':{'world_sha256':'a'*64}, 'external_vision':{'healthy':True,'used_for_flight_control':True,
            'adapter':{'epoch':1,'ready':True,'forwarding':True,'reset_latched':False},
            'ekf':{'ready':True,'fusion_active':True,'primary_instance':0,'observed_sim_s':t,
                   'local_position_source_stamp_sim_s':t-.02,
                   'local_position_reset_counters':{'xy':1,'z':1,'heading':1},
                   'local_velocity_reset_counters':{'vxy':1,'vz':1},
                   'mag_alignment':{'yaw_aligned':True,'in_air':False},
                   'mag_reference':{'heading_fusion':True,'three_axis_fusion':False,
                                    'fault':False,'field_disturbed':False,'source_stamp_sim_s':t}}}}


class GroundHeadingProbeTests(unittest.TestCase):
    def test_candidate_requires_distinct_ground_samples_and_is_not_authorization(self):
        probe=GroundHeadingProbe()
        for i in range(41):
            t=10+i*.1
            result=probe.add(sample(t),state(t))
        self.assertEqual(result['status'],'candidate_only')
        self.assertAlmostEqual(result['offset_rad'],.8)
        self.assertIn('not flight authorization',result['scope'])
        self.assertFalse(result['uncertainty_validated'])
        self.assertFalse(result['reference_independent'])
        self.assertFalse(result['fusion_authorized'])

    def test_wraparound_and_untrusted_small_covariance(self):
        probe = GroundHeadingProbe()
        for i in range(41):
            t = 10 + i*.1
            x = sample(t)
            angle = math.pi - .001 if i % 2 else -math.pi + .001
            x['pose_q_xyzw'] = [0., 0., math.sin(angle/2), math.cos(angle/2)]
            x['visual_yaw_variance'] = 1e-30
            result = probe.add(x, state(t))
        self.assertEqual(result['status'], 'candidate_only')
        self.assertLess(abs(abs(result['offset_rad']) - math.pi), .002)
        self.assertLess(result['maximum_deviation_rad'], .003)
        self.assertFalse(result['uncertainty_validated'])
        self.assertFalse(result['reference_independent'])
        self.assertFalse(result['fusion_authorized'])

    def test_fault_or_identity_change_permanently_invalidates_window(self):
        scenarios=('vio_epoch','heading_reset','xy_reset','velocity_reset','primary','world','armed','mag_fault',
                   'missing_mag_evidence','stale_magnetic','future_magnetic','future_ekf','visual_future','pose_future','stale_pose',
                   'bad_quaternion','bad_covariance','missing_covariance','missing_frame','duplicate')
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                probe=GroundHeadingProbe();probe.add(sample(10),state(10))
                t=10.1;x=sample(t);s=state(t);ev=s['external_vision'];ekf=ev['ekf']
                if scenario=='vio_epoch':ev['adapter']['epoch']+=1
                elif scenario=='heading_reset':ekf['local_position_reset_counters']['heading']+=1
                elif scenario=='xy_reset':ekf['local_position_reset_counters']['xy']+=1
                elif scenario=='velocity_reset':ekf['local_velocity_reset_counters']['vxy']+=1
                elif scenario=='primary':ekf['primary_instance']=1
                elif scenario=='world':s['scene']['world_sha256']='b'*64
                elif scenario=='armed':s['armed']=True
                elif scenario=='mag_fault':ekf['mag_reference']['fault']=True
                elif scenario=='missing_mag_evidence':ekf['mag_reference']['fault']=None
                elif scenario=='stale_magnetic':ekf['mag_reference']['source_stamp_sim_s']=t-3
                elif scenario=='future_magnetic':ekf['mag_reference']['source_stamp_sim_s']=t+.01
                elif scenario=='future_ekf':ekf['local_position_source_stamp_sim_s']=t+.01
                elif scenario=='visual_future':x['clock_sim_s']=t-.1
                elif scenario=='pose_future':x['pose_stamp_sim_s']=t+.01
                elif scenario=='stale_pose':x['pose_stamp_sim_s']=t-.1
                elif scenario=='bad_quaternion':x['visual_q_xyzw']=[0.,0.,0.,0.]
                elif scenario=='bad_covariance':x['visual_yaw_variance']=0
                elif scenario=='missing_covariance':del x['visual_yaw_variance']
                elif scenario=='missing_frame':x['visual_frame']=''
                elif scenario=='duplicate':x=sample(10.)
                with self.assertRaises(ValueError):probe.add(x,s)
                self.assertEqual(probe.summary()['status'],'invalid')
                self.assertNotIn('offset_rad',probe.summary())
                with self.assertRaises(ValueError):probe.add(sample(10.2),state(10.2))

    def test_current_state_fault_invalidates_even_without_a_matched_image(self):
        probe=GroundHeadingProbe();probe.add(sample(10),state(10))
        current=state(10.1);current['external_vision']['ekf']['mag_reference']['fault']=True
        with self.assertRaises(ValueError):probe.check_state(current)
        self.assertEqual(probe.summary()['status'],'invalid')
        self.assertEqual(probe.summary()['count'],0)


if __name__=='__main__': unittest.main()
