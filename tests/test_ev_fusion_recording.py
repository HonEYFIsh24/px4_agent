"""Pure recording/evidence contracts; no simulator or ROS required."""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import unittest
from unittest import mock

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/record_ev_fusion.py'
SPEC = importlib.util.spec_from_file_location('ev_fusion_recording', SCRIPT)
recording = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recording)


def topic(name, fields, code=0):
    return {'kind': 'topic', 'topic': name, 'published': True, 'return_code': code, 'fields': fields}


def observations(instance=0, reject=False, fuse_times=(40, 41)):
    result = [topic('estimator_selector_status', {'primary_instance': instance}),
              topic('estimator_status_flags', {'cs_ev_pos': True, 'cs_ev_vel': False, 'cs_gps': True})]
    for stamp, fused in zip((50, 51), fuse_times):
        result.append(topic('estimator_aid_src_ev_pos', {'timestamp_sample': stamp,
            'time_last_fuse': fused, 'fusion_enabled': True, 'fused': True, 'innovation_rejected': reject}))
    return result


def explicit_observations(instance=2):
    records = observations(instance=instance)
    for record in records:
        record['instance'] = (instance if record['topic'] in (
            'estimator_status_flags', 'estimator_aid_src_ev_pos') else 0)
    return records


class ListenerSnapshotTests(unittest.TestCase):
    def test_single_instance_snapshot_is_instance_zero_only(self):
        text = 'TOPIC: vehicle_local_position\n vehicle_local_position\n    timestamp: 123\n    xy_valid: true\n'
        self.assertEqual(recording.parse_listener_snapshot(text, 0)['fields']['timestamp'], 123)
        self.assertFalse(recording.parse_listener_snapshot(text, 1)['published'])

    def test_multiple_instances_are_selected_without_field_leakage(self):
        text = ('TOPIC: estimator_status_flags 2 instances\n\nInstance 0:\n'
                ' estimator_status_flags\n    timestamp: 100\n    cs_ev_pos: false\n'
                '\nInstance 1:\n estimator_status_flags\n    timestamp: 200\n'
                '    cs_ev_pos: true\n    cs_ev_vel: true\n')
        result = recording.parse_listener_snapshot(text, 0)
        self.assertEqual(result['fields'], {'timestamp': 100, 'cs_ev_pos': False})
        self.assertEqual(recording.parse_listener_snapshot(text, 1)['fields']['timestamp'], 200)
        self.assertFalse(recording.parse_listener_snapshot(text, 2)['published'])

    def test_missing_or_ambiguous_snapshot_never_becomes_telemetry(self):
        for text in ('never published\n', '', 'TOPIC: estimator_status_flags 2 instances\n timestamp: 100\n'):
            self.assertFalse(recording.parse_listener_snapshot(text)['published'])


class FusionRecordingTests(unittest.TestCase):
    def test_topic_instances_follow_px4_selector_and_vehicle_publication(self):
        multi = {'estimator_status_flags', 'estimator_event_flags',
                 'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel',
                 'estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw'}
        single = {'estimator_selector_status', 'vehicle_visual_odometry',
                  'vehicle_local_position', 'vehicle_attitude', 'vehicle_status',
                  'failsafe_flags', 'timesync_status'}
        self.assertEqual(set(recording.TOPICS), multi | single)
        for instance in range(4):
            for name in multi | single:
                with self.subTest(topic=name, instance=instance):
                    self.assertEqual(recording.topic_instance(name, instance),
                                     instance if name in multi else 0)

    def test_nonzero_instance_dry_plan_queries_global_topics_at_zero(self):
        with mock.patch.object(recording.subprocess, 'run', side_effect=AssertionError('process forbidden')), \
                mock.patch.object(Path, 'mkdir', side_effect=AssertionError('write forbidden')), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(recording.main(['--instance', '2']), 0)
        specification = json.loads(capture.getvalue())
        commands = {command[1]: command for command in specification['commands']}
        for name in ('estimator_status_flags', 'estimator_event_flags',
                     'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel',
                     'estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw'):
            self.assertEqual(commands[name][-4:], ['-i', '2', '-n', '1'])
            self.assertEqual(specification['topic_instances'][name], 2)
        for name in ('estimator_selector_status', 'vehicle_visual_odometry',
                     'vehicle_local_position', 'vehicle_attitude', 'vehicle_status',
                     'failsafe_flags', 'timesync_status'):
            self.assertEqual(commands[name][-4:], ['-i', '0', '-n', '1'])
            self.assertEqual(specification['topic_instances'][name], 0)

    def test_usable_topic_requires_exact_success_publication_fields_and_instance(self):
        valid = topic('estimator_status_flags', {'cs_ev_pos': True})
        valid['instance'] = 0
        self.assertTrue(recording.usable_topic(valid, 0))
        changes = [('return_code', value) for value in (False, 0., '0', None, 1)]
        changes += [('published', value) for value in (1, 'true', None, False)]
        changes += [('fields', value) for value in (None, [], 'not fields')]
        changes += [('instance', value) for value in (False, 0., '0', None, -1, 4, 1)]
        for field, value in changes:
            with self.subTest(field=field, value=value):
                record = copy.deepcopy(valid)
                record[field] = value
                self.assertFalse(recording.usable_topic(record, 0))

    def test_explicit_instance_mismatch_cannot_supply_selected_fusion(self):
        self.assertTrue(recording.summary(explicit_observations(), 2)
                        ['components']['pos']['fusion_observed_in_selected_instance'])
        for index in range(4):
            with self.subTest(topic_index=index):
                records = explicit_observations()
                records[index]['instance'] = 2 if index == 0 else 0
                result = recording.summary(records, 2)
                self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])
                self.assertEqual(result['topic_statistics'][records[index]['topic']]['published_count'],
                                 1 if index in (2, 3) else 0)

    def test_missing_instance_remains_compatible_with_legacy_recordings(self):
        records = observations(instance=2)
        self.assertTrue(all('instance' not in record for record in records))
        self.assertTrue(recording.summary(records, 2)
                        ['components']['pos']['fusion_observed_in_selected_instance'])

    def test_invalid_published_primary_never_coerces_or_proves_selection(self):
        for value in (False, True, 0., 2., '0', None, -1, 4):
            with self.subTest(primary=value):
                records = observations() + [topic('estimator_selector_status', {'primary_instance': value})]
                result = recording.summary(records)
                self.assertEqual(result['primary_estimator_instances_observed'], [0])
                self.assertEqual(result['invalid_published_selector_count'], 1)
                self.assertFalse(result['requested_instance_is_observed_primary'])
                self.assertTrue(result['components']['pos']['fusion_observed_in_requested_instance'])
                self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])

    def test_failed_unpublished_or_wrong_instance_selector_is_not_proof(self):
        changes = [('return_code', 1), ('return_code', False), ('return_code', 0.),
                   ('return_code', '0'), ('published', False), ('published', 1),
                   ('fields', None), ('fields', []), ('instance', 2), ('instance', False)]
        for field, value in changes:
            with self.subTest(field=field, value=value):
                records = observations()
                records[0][field] = value
                result = recording.summary(records)
                self.assertEqual(result['primary_estimator_instances_observed'], [])
                self.assertEqual(result['invalid_published_selector_count'], 0)
                self.assertFalse(result['requested_instance_is_observed_primary'])
                self.assertTrue(result['components']['pos']['fusion_observed_in_requested_instance'])
                self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])

    def test_aid_measurement_and_fuse_times_must_be_positive_exact_integers(self):
        for field in ('timestamp_sample', 'time_last_fuse'):
            for value in (True, False, 50., '50', None, 0, -1):
                with self.subTest(field=field, value=value):
                    records = observations()
                    for record in records[2:]:
                        record['fields'][field] = value
                    result = recording.summary(records)
                    self.assertEqual(result['components']['pos']['distinct_successful_aid_sample_count'], 0)
                    self.assertFalse(result['components']['pos']['fusion_observed_in_requested_instance'])

    def test_aid_boolean_fields_cannot_be_truthy_numeric_values(self):
        for field, value in (('fusion_enabled', 1), ('fused', 1),
                             ('innovation_rejected', 0), ('innovation_rejected', None)):
            with self.subTest(field=field, value=value):
                records = observations()
                for record in records[2:]:
                    record['fields'][field] = value
                self.assertFalse(recording.summary(records)['components']['pos']
                                 ['fusion_observed_in_requested_instance'])

    def test_aid_transport_failure_or_invalid_record_shape_cannot_prove_fusion(self):
        changes = [('return_code', 1), ('return_code', False), ('return_code', 0.),
                   ('published', False), ('published', 1), ('fields', None), ('fields', []),
                   ('instance', 1), ('instance', False)]
        for field, value in changes:
            with self.subTest(field=field, value=value):
                records = observations()
                for record in records[2:]:
                    record[field] = value
                result = recording.summary(records)
                self.assertEqual(result['topic_statistics']['estimator_aid_src_ev_pos']['published_count'], 0)
                self.assertEqual(result['components']['pos']['distinct_successful_aid_sample_count'], 0)
                self.assertFalse(result['components']['pos']['fusion_observed_in_requested_instance'])

    def test_failed_or_unpublished_flags_do_not_claim_gps_or_ev_fusion(self):
        changes = [('return_code', 1), ('return_code', False), ('return_code', 0.),
                   ('published', False), ('published', 1), ('fields', None), ('fields', []),
                   ('instance', 1), ('instance', False)]
        for field, value in changes:
            with self.subTest(field=field, value=value):
                records = observations()
                records[1][field] = value
                result = recording.summary(records)
                self.assertFalse(result['cs_gps']['observed_true'])
                self.assertEqual(result['cs_gps']['poll_count'], 0)
                self.assertFalse(result['components']['pos']['control_flag_observed'])
                self.assertFalse(result['components']['pos']['fusion_observed_in_requested_instance'])

    def test_failed_reset_counters_and_wrong_global_instance_are_not_evidence(self):
        visual = topic('vehicle_visual_odometry', {'reset_counter': 3})
        visual['instance'] = 0
        local = topic('vehicle_local_position', {'xy_reset_counter': 2, 'z_reset_counter': 1})
        local['instance'] = 0
        changes = [('return_code', 1), ('return_code', False), ('return_code', 0.),
                   ('published', False), ('published', 1), ('instance', 2), ('instance', False)]
        for field, value in changes:
            with self.subTest(field=field, value=value):
                failed_visual, failed_local = copy.deepcopy(visual), copy.deepcopy(local)
                failed_visual['fields']['reset_counter'] = 99
                failed_local['fields'].update(xy_reset_counter=99, z_reset_counter=99)
                failed_visual[field] = failed_local[field] = value
                result = recording.summary([visual, local, failed_visual, failed_local], 2)
                self.assertEqual(result['visual_odometry_reset_counters'], [3])
                self.assertEqual(result['local_position_resets']['xy_reset_counter'], [2])
                self.assertEqual(result['local_position_resets']['z_reset_counter'], [1])

    def test_listener_numeric_bool_arrays_and_nonfinite_do_not_fabricate_values(self):
        result = recording.parse_listener('TOPIC: vehicle_visual_odometry\n'
            '\ttimestamp: 1234 (0.2 seconds ago)\n\tposition: [1.2, -3.0, nan]\n'
            '\tcs_ev_pos: 1\n\tfused: True\n\tinnovation_rejected: 0\n\tquality: 1\n')
        self.assertTrue(result['published'])
        self.assertEqual(result['fields']['timestamp'], 1234)
        self.assertEqual(result['fields']['position'], [1.2, -3., None])
        self.assertIs(result['fields']['quality'], 1)
        self.assertIs(result['fields']['cs_ev_pos'], True)
        self.assertIs(result['fields']['fused'], True)
        self.assertIs(result['fields']['innovation_rejected'], False)

    def test_never_published_is_missing_evidence(self):
        self.assertFalse(recording.parse_listener('never published\n')['published'])
        result = recording.summary([])
        self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])

    def test_autopilot_flags_use_message_boolean_types_but_navigation_remains_integer(self):
        result = recording.parse_listener('TOPIC: vehicle_status\n\tnav_state: 12\n'
            '\tarming_state: 2\n\tfailsafe: 1\n\tlocal_altitude_invalid: 0\n'
            '\tattitude_invalid: 0\n\tangular_velocity_invalid: 0\n'
            '\tlocal_position_invalid: 1\n\toffboard_control_signal_lost: 1\n')
        fields = result['fields']
        self.assertIs(fields['failsafe'], True)
        self.assertIs(fields['local_altitude_invalid'], False)
        self.assertIs(fields['local_position_invalid'], True)
        self.assertIs(fields['offboard_control_signal_lost'], True)
        self.assertIs(type(fields['nav_state']), int)
        self.assertIs(type(fields['arming_state']), int)

    def test_parameter_values_are_actual_source_not_defaults(self):
        result = recording.parse_parameters('x + EKF2_EV_CTRL [12,13] : 5\n'
            '  EKF2_GPS_CTRL [14,15] : 0\n EKF2_EV_DELAY : 12.500\n')
        self.assertEqual(result, {'EKF2_EV_CTRL': 5, 'EKF2_GPS_CTRL': 0, 'EKF2_EV_DELAY': 12.5})
        self.assertEqual(recording.parse_parameters('no match'), {})

    def test_flag_and_received_ev_alone_never_prove_fusion(self):
        records = [topic('estimator_status_flags', {'cs_ev_pos': True}),
                   topic('vehicle_visual_odometry', {'timestamp': 90, 'timestamp_sample': 70, 'reset_counter': 3})]
        result = recording.summary(records)
        self.assertTrue(result['components']['pos']['control_flag_observed'])
        self.assertFalse(result['components']['pos']['fusion_observed_in_requested_instance'])
        self.assertEqual(result['visual_odometry_reset_counters'], [3])

    def test_two_actual_distinct_aid_sources_prove_observed_selected_fusion(self):
        result = recording.summary(observations())
        self.assertTrue(result['components']['pos']['fusion_observed_in_selected_instance'])
        self.assertFalse(result['components']['vel']['fusion_observed_in_selected_instance'])
        self.assertTrue(result['cs_gps']['observed_true'])

    def test_duplicate_stale_aid_source_does_not_prove_fusion(self):
        records = observations()
        records[-1] = dict(records[-2])
        result = recording.summary(records)
        self.assertFalse(result['components']['pos']['fusion_observed_in_requested_instance'])

    def test_nonadvancing_fuse_time_does_not_prove_fusion(self):
        result = recording.summary(observations(fuse_times=(40, 40)))
        self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])

    def test_rejected_or_failed_listener_samples_do_not_prove_fusion(self):
        result = recording.summary(observations(reject=True))
        self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])
        self.assertEqual(result['components']['pos']['innovation_rejected_poll_count'], 2)
        records = observations()
        records[-1]['return_code'] = 1
        self.assertFalse(recording.summary(records)['components']['pos']['fusion_observed_in_requested_instance'])

    def test_other_or_changing_primary_does_not_prove_selected_fusion(self):
        result = recording.summary(observations(instance=1))
        self.assertTrue(result['components']['pos']['fusion_observed_in_requested_instance'])
        self.assertFalse(result['components']['pos']['fusion_observed_in_selected_instance'])
        records = observations()+[topic('estimator_selector_status', {'primary_instance': 1})]
        self.assertFalse(recording.summary(records)['requested_instance_is_observed_primary'])

    def test_timeout_captures_partial_evidence_without_retry(self):
        runner = mock.Mock(side_effect=subprocess.TimeoutExpired(['px4-listener'], 1.5, output=b'partial'))
        result = recording.bounded_command(['px4-listener', 'vehicle_visual_odometry'], 1.5, runner)
        self.assertEqual(result['error_type'], 'TimeoutExpired')
        self.assertEqual(result['stdout'], 'partial')
        self.assertEqual(runner.call_count, 1)
        self.assertLessEqual(runner.call_args.kwargs['timeout'], 2)

    def test_timeout_outside_bound_rejects_before_process(self):
        runner = mock.Mock()
        with self.assertRaises(ValueError):
            recording.bounded_command(['px4-listener'], 3, runner)
        runner.assert_not_called()

    def test_dry_run_does_not_call_process_or_write_output(self):
        with mock.patch.object(recording.subprocess, 'run', side_effect=AssertionError('process forbidden')), \
                mock.patch.object(Path, 'mkdir', side_effect=AssertionError('write forbidden')), \
                contextlib.redirect_stdout(io.StringIO()) as capture:
            self.assertEqual(recording.main(['--output', '/tmp/must-not-be-written']), 0)
        result = json.loads(capture.getvalue())
        self.assertFalse(result['record_requested'])
        self.assertFalse(result['flight_command_sent'])
        self.assertTrue(all(command[0].endswith('px4-listener') for command in result['commands']))

    def test_process_scheduling_state_is_not_identity(self):
        self.assertEqual(recording.stable_px4_identity({'pid': 1, 'state': 'S'}),
                         recording.stable_px4_identity({'pid': 1, 'state': 'R'}))


if __name__ == '__main__':
    unittest.main()
