import copy
import unittest

from compare_nbv_heading_runs import compare


class HeadingComparisonTests(unittest.TestCase):
    def setUp(self):
        base = {'search_mode': 'nbv_geometry', 'target_confirmation_mode': 'single_frame',
                'nbv_selector': 'deterministic', 'candidate_pool_mode': 'ranked', 'nbv_auto_prepare': True,
                'world_file': '/world', 'elapsed_seconds': 12.,
                'gazebo_seed_evidence': {'source': 'actual_gzserver_proc_cmdline', 'seed': 17,
                                         'lockstep': True, 'world_file': '/world', 'pid': 1},
                'command': {'prompt': 'find van', 'status': 'succeeded', 'perceptions': [{'capture_id': 'a'}],
                            'model_calls': [{'model': 'model'}], 'search_events': [],
                            'nbv_events': [{'config': {'mode': 'geometry', 'heading_mode': 'fixed'}}]}}
        snapshot = {'world_sha256': 'a'*64, 'registration': {'world_from_enu_origin_m': [0., 0., 0.],
                    'world_from_enu_yaw_rad': 0.}, 'observations': [{'position_enu_m': [0., 0., 3.],
                    'orientation_xyzw': [0., 0., 0., 1.]}],
                    'eligible_cell_ids': [0, 1], 'cells': [{'coverage': 1.}, {'coverage': 0.}]}
        audit = {'passed': True, 'moves': [{'heading_step': None, 'actual_planned_length_m': 1.}]}
        other = copy.deepcopy(base); other['heading_mode'] = 'discrete'; other['gazebo_seed_evidence']['pid'] = 2
        other['command']['nbv_events'][0]['config']['heading_mode'] = 'discrete'
        base['heading_mode'] = 'fixed'
        other_audit = copy.deepcopy(audit); other_audit['moves'][0]['heading_step'] = 1
        self.fixed, self.discrete = (base, audit, [snapshot]), (other, other_audit, [copy.deepcopy(snapshot)])

    def test_equal_conditions_and_mismatched_seed_or_configuration(self):
        result = compare(self.fixed, self.discrete)
        self.assertEqual(result['seed'], 17)
        self.assertEqual(result['discrete']['rotating_moves'], 1)
        self.assertFalse(result['statistics_or_causal_advantage_proven'])
        for change in ('seed', 'pid', 'world', 'model', 'position', 'registration', 'heading', 'budget', 'no_rotation'):
            with self.subTest(change=change):
                other = copy.deepcopy(self.discrete)
                if change == 'seed': other[0]['gazebo_seed_evidence']['seed'] = 18
                if change == 'pid': other[0]['gazebo_seed_evidence']['pid'] = 1
                if change == 'world': other[2][0]['world_sha256'] = 'b'*64
                if change == 'model': other[0]['command']['model_calls'][0]['model'] = 'other'
                if change == 'position': other[2][0]['observations'][0]['position_enu_m'][0] = 1.
                if change == 'registration': other[2][0]['registration']['world_from_enu_origin_m'][0] = 1.
                if change == 'heading': other[2][0]['observations'][0]['orientation_xyzw'] = [0., 0., .2, .98]
                if change == 'budget': other[0]['command']['nbv_events'][0]['config']['max_path_evaluations'] = 99
                if change == 'no_rotation': other[1]['moves'][0]['heading_step'] = None
                with self.assertRaises(ValueError): compare(self.fixed, other)


if __name__ == '__main__':
    unittest.main()
