"""Checks causal sample selection for the offline PX4 diagnostic."""
import unittest

from scripts.analyze_heading_ulog import (changes, previous_sample, compare_visual_yaw_to_groundtruth,
                                          replay_ground_heading_reference)


class ULogHeadingEvidenceTests(unittest.TestCase):
    def test_previous_sample_never_uses_a_future_flag(self):
        samples = {'timestamp_sample': [10, 20, 30]}
        self.assertIsNone(previous_sample(samples, 9))
        self.assertEqual(previous_sample(samples, 20), 1)
        self.assertEqual(previous_sample(samples, 29), 1)
        self.assertEqual(previous_sample(samples, 30), 2)
        with self.assertRaises(ValueError):
            previous_sample({'timestamp_sample': [10, 30, 20]}, 25)

    def test_counter_changes_include_wrap_and_skip_repeated_values(self):
        self.assertEqual(changes({'count': [254, 255, 255, 0, 1]}, 'count'), [1, 3, 4])
        self.assertEqual(changes({'count': []}, 'count'), [])

    def test_groundtruth_comparison_preserves_offset_and_rejects_reversed_time(self):
        import math
        def attitude(yaw):
            return {'q[0]':[math.cos(yaw/2)]*3, 'q[1]':[0.]*3,
                    'q[2]':[0.]*3, 'q[3]':[math.sin(yaw/2)]*3}
        visual = dict(attitude(0), timestamp_sample=[11000000,12000000,13000000], quality=[1,1,1])
        truth = dict(attitude(.5), timestamp=[11000000,12000000,13000000])
        result = compare_visual_yaw_to_groundtruth(visual, truth)
        self.assertAlmostEqual(result['median_offset_rad'], -.5)
        self.assertAlmostEqual(result['maximum_absolute_residual_rad'], 0.)
        visual['timestamp_sample'] = [11000000,13000000,12000000]
        with self.assertRaises(ValueError):
            compare_visual_yaw_to_groundtruth(visual, truth)

    def test_ground_reference_never_uses_truth_or_airborne_samples_to_calibrate(self):
        import math
        stamps = [10000000+i*100000 for i in range(101)]
        def attitude(angle):
            return {'q[0]':[math.cos(angle/2)]*len(stamps), 'q[1]':[0.]*len(stamps),
                    'q[2]':[0.]*len(stamps), 'q[3]':[math.sin(angle/2)]*len(stamps)}
        visual=dict(attitude(0),timestamp_sample=stamps,quality=[1]*len(stamps),reset_counter=[0]*len(stamps))
        px4=dict(attitude(.7),timestamp_sample=stamps,quat_reset_counter=[1]*len(stamps))
        truth={k:v[51:] for k,v in attitude(.6).items()}
        truth['timestamp']=stamps[51:]
        status={'timestamp':stamps,'arming_state':[1]*51+[2]*50}
        flags={'timestamp_sample':stamps,'cs_in_air':[False]*51+[True]*50,
               'cs_yaw_align':[True]*len(stamps),'cs_mag_hdg':[True]*len(stamps),
               'cs_mag_3d':[False]*len(stamps),'cs_mag_fault':[False]*len(stamps),
               'cs_mag_field_disturbed':[False]*len(stamps)}
        result=replay_ground_heading_reference(visual,truth,px4,status,flags)
        self.assertAlmostEqual(result['ground_px4_minus_visual_yaw_rad'],.7)
        self.assertAlmostEqual(result['heldout_absolute_truth_error_p95_rad'],.1)
        self.assertTrue(result['heldout_contains_airborne'])
        flags['cs_in_air']=[True]*len(stamps)
        with self.assertRaises(ValueError):
            replay_ground_heading_reference(visual,truth,px4,status,flags)

    def test_ground_reference_rejects_reset_unhealthy_or_stale_reference(self):
        import math
        stamps=[10000000+i*100000 for i in range(101)]
        def attitude():
            return {'q[0]':[1.]*len(stamps),'q[1]':[0.]*len(stamps),
                    'q[2]':[0.]*len(stamps),'q[3]':[0.]*len(stamps)}
        for fault in ('vision_reset','attitude_reset','mag_fault','stale_attitude'):
            with self.subTest(fault=fault):
                visual=dict(attitude(),timestamp_sample=stamps,quality=[1]*len(stamps),reset_counter=[0]*len(stamps))
                px4=dict(attitude(),timestamp_sample=stamps,quat_reset_counter=[1]*len(stamps))
                truth=dict(attitude(),timestamp=stamps)
                status={'timestamp':stamps,'arming_state':[1]*51+[2]*50}
                flags={'timestamp_sample':stamps,'cs_in_air':[False]*51+[True]*50,
                       'cs_yaw_align':[True]*len(stamps),'cs_mag_hdg':[True]*len(stamps),
                       'cs_mag_3d':[False]*len(stamps),'cs_mag_fault':[False]*len(stamps),
                       'cs_mag_field_disturbed':[False]*len(stamps)}
                if fault=='vision_reset': visual['reset_counter'][25:]=[1]*76
                elif fault=='attitude_reset': px4['quat_reset_counter'][25:]=[2]*76
                elif fault=='mag_fault': flags['cs_mag_fault']=[True]*len(stamps)
                else: px4['timestamp_sample']=[x-60000 for x in stamps]
                with self.assertRaises(ValueError):
                    replay_ground_heading_reference(visual,truth,px4,status,flags)


if __name__ == '__main__':
    unittest.main()
