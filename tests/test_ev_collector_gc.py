"""Collector GC lifecycle contracts with fake ROS; no ROS, flight or IO."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'ev_collector_gc_contracts', ROOT / 'scripts/test_ev_shadow.py')
shadow = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(shadow)


class FakeGC:
    """Observe lifecycle without changing this test process's real GC state."""
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.operations = []

    def isenabled(self):
        return self.enabled

    def disable(self):
        self.operations.append('disable')
        self.enabled = False

    def enable(self):
        self.operations.append('enable')
        self.enabled = True

    def collect(self, *args):
        self.operations.append('collect')
        return 0


class FakeSubscription:
    def __init__(self, topic, gc_state, failure=None):
        self.topic = topic
        self.gc_state = gc_state
        self.failure = failure
        self.unregister_states = []

    def unregister(self):
        self.unregister_states.append(self.gc_state.enabled)
        if self.failure is not None:
            raise self.failure


class FakeROS:
    def __init__(self, gc_state, fail_construction_at=None, emit_clock=True,
                 emit_timing=True, ros_now=10., timing_source='producer_ros_clock_at_publish'):
        self.gc_state = gc_state
        self.fail_construction_at = fail_construction_at
        self.subscriptions = []
        self.creation_gc_states = []
        self.modules = {}
        self.callbacks = {}
        self.emit_clock = emit_clock
        self.emit_timing = emit_timing
        self.ros_now = ros_now
        self.timing_source = timing_source
        rospy = types.ModuleType('rospy')
        rospy.init_node = mock.Mock()
        rospy.Subscriber = self.subscribe
        rospy.Time = types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_sec=lambda: self.ros_now))
        self.modules['rospy'] = rospy
        for package, names in (
                ('gazebo_msgs', ('ModelStates',)),
                ('nav_msgs', ('Odometry',)),
                ('sensor_msgs', ('LaserScan',)),
                ('std_msgs', ('Bool', 'String')),
                ('rosgraph_msgs', ('Clock',))):
            parent = types.ModuleType(package)
            messages = types.ModuleType(package + '.msg')
            for name in names:
                setattr(messages, name, type(name, (), {}))
            parent.msg = messages
            self.modules[package] = parent
            self.modules[package + '.msg'] = messages

    def subscribe(self, topic, *args, **kwargs):
        self.creation_gc_states.append(self.gc_state.enabled)
        if len(self.creation_gc_states) == self.fail_construction_at:
            raise RuntimeError('Synthetic subscriber construction failure')
        subscription = FakeSubscription(topic, self.gc_state)
        self.subscriptions.append(subscription)
        self.callbacks[topic] = args[1]
        if topic == '/clock' and self.emit_clock:
            self.deliver_clock()
        elif topic == shadow.TOPICS['timing'] and self.emit_timing:
            self.deliver_timing()
        return subscription

    def deliver_clock(self):
        self.callbacks['/clock'](types.SimpleNamespace(
            clock=types.SimpleNamespace(to_sec=lambda: 10.)))

    def deliver_timing(self):
        self.callbacks[shadow.TOPICS['timing']](types.SimpleNamespace(data=json.dumps({
            'stamp_ns': 10000000000, 'stamp_sim_s': 10.,
            'actual_publish_clock_sim_s': 10., 'measurement_age_at_publish_sim_s': 0.,
            'source': self.timing_source, 'source_timestamp_unchanged': True})))


class CollectorGarbageCollectionTests(unittest.TestCase):
    def environment(self, gc_state, fail_construction_at=None, **ros_options):
        ros = FakeROS(gc_state, fail_construction_at, **ros_options)
        return ros, mock.patch.dict(sys.modules, ros.modules), mock.patch.object(
            shadow, 'gc', gc_state, create=True)

    def test_pause_starts_after_all_subscriptions_and_restores_after_close(self):
        gc_state = FakeGC(True)
        ros, modules, garbage = self.environment(gc_state)
        with modules, garbage:
            collector = shadow.RosObservations()
            try:
                self.assertGreater(len(ros.subscriptions), 1)
                self.assertTrue(all(ros.creation_gc_states))
                self.assertFalse(gc_state.enabled)
                self.assertEqual(gc_state.operations, ['disable'])
            finally:
                collector.close()
            self.assertTrue(gc_state.enabled)
            self.assertEqual(gc_state.operations, ['disable', 'enable'])
            self.assertTrue(all(s.unregister_states == [False]
                                for s in ros.subscriptions))
            self.assertNotIn('collect', gc_state.operations)

    def test_preexisting_disabled_state_is_preserved(self):
        gc_state = FakeGC(False)
        ros, modules, garbage = self.environment(gc_state)
        with modules, garbage:
            collector = shadow.RosObservations()
            self.assertFalse(gc_state.enabled)
            collector.close()
            self.assertFalse(gc_state.enabled)
            self.assertNotIn('enable', gc_state.operations)
            self.assertNotIn('collect', gc_state.operations)
            self.assertTrue(all(s.unregister_states == [False]
                                for s in ros.subscriptions))

    def test_failed_initialization_does_not_pause_garbage_collection(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                gc_state = FakeGC(enabled)
                ros, modules, garbage = self.environment(gc_state, 4)
                with modules, garbage:
                    with self.assertRaisesRegex(RuntimeError, 'construction failure'):
                        shadow.RosObservations()
                self.assertEqual(gc_state.enabled, enabled)
                self.assertEqual(gc_state.operations, [])
                self.assertTrue(all(s.unregister_states == [enabled]
                                    for s in ros.subscriptions))

    def test_close_is_idempotent_and_does_not_reenable_again(self):
        gc_state = FakeGC(True)
        ros, modules, garbage = self.environment(gc_state)
        with modules, garbage:
            collector = shadow.RosObservations()
            collector.close()
            operations = list(gc_state.operations)
            collector.close()
            self.assertEqual(gc_state.operations, operations)
            self.assertTrue(all(s.unregister_states == [False]
                                for s in ros.subscriptions))

    def test_subscriber_cleanup_failure_still_restores_original_state(self):
        gc_state = FakeGC(True)
        ros, modules, garbage = self.environment(gc_state)
        with modules, garbage:
            collector = shadow.RosObservations()
            ros.subscriptions[1].failure = RuntimeError('Synthetic unsubscribe failure')
            try:
                collector.close()
            except RuntimeError:
                pass
            self.assertTrue(gc_state.enabled)
            self.assertTrue(all(s.unregister_states == [False]
                                for s in ros.subscriptions))
            self.assertNotIn('collect', gc_state.operations)
            evidence = collector.recording_session.evidence()
            self.assertTrue(evidence['closed'])
            self.assertTrue(evidence['previous_gc_enabled'])
            self.assertTrue(evidence['source_stamps_and_age_limits_unchanged'])
            self.assertEqual(evidence['cleanup_errors'], ['Synthetic unsubscribe failure'])

    def test_closing_restores_start_state_even_if_other_code_changed_it(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                gc_state = FakeGC(enabled)
                ros, modules, garbage = self.environment(gc_state)
                with modules, garbage:
                    collector = shadow.RosObservations()
                    gc_state.enabled = not enabled
                    collector.close()
                    self.assertEqual(gc_state.enabled, enabled)
                    self.assertNotIn('collect', gc_state.operations)

    def test_actual_clock_and_timing_receipts_precede_measurement_subscriptions(self):
        gc_state = FakeGC(True)
        ros, modules, garbage = self.environment(gc_state)
        with modules, garbage:
            collector = shadow.RosObservations()
            try:
                self.assertEqual([s.topic for s in ros.subscriptions[:5]],
                    ['/clock', shadow.TOPICS['timing'], shadow.TOPICS['shadow'],
                     shadow.TOPICS['optimized'], shadow.TOPICS['propagated']])
                self.assertEqual([e['kind'] for e in collector.events],
                                 ['clock', 'timing', 'recorder_startup_ready'])
                ready = collector.events[-1]
                self.assertEqual(ready['clock_stamp_sim_s'], 10.)
                self.assertEqual(ready['timing_stamp_ns'], 10000000000)
                self.assertTrue(ready['all_startup_events_retained'])
            finally:
                collector.close()

    def test_waits_for_actual_receipts_without_discarding_early_evidence(self):
        gc_state = FakeGC(True)
        ros, modules, garbage = self.environment(gc_state, emit_clock=False)
        elapsed = [0.]
        def sleep(delay):
            self.assertEqual([s.topic for s in ros.subscriptions],
                             ['/clock', shadow.TOPICS['timing']])
            elapsed[0] += delay
            ros.deliver_clock()
        fake_time = types.SimpleNamespace(monotonic=lambda: elapsed[0],
                                         time=lambda: 100.+elapsed[0], sleep=sleep)
        with modules, garbage, mock.patch.object(shadow, 'time', fake_time):
            collector = shadow.RosObservations()
            try:
                self.assertEqual([e['kind'] for e in collector.events],
                                 ['timing', 'clock', 'recorder_startup_ready'])
                self.assertGreater(elapsed[0], 0.)
                self.assertFalse(gc_state.enabled)
            finally:
                collector.close()

    def test_missing_or_invalid_startup_receipts_timeout_and_clean_partial_subscriptions(self):
        options = ({'emit_clock': False}, {'emit_timing': False},
                   {'ros_now': 0.}, {'timing_source': 'not_producer_clock'})
        for option in options:
            with self.subTest(option=option):
                gc_state = FakeGC(True)
                ros, modules, garbage = self.environment(gc_state, **option)
                elapsed = [0.]
                def sleep(delay): elapsed[0] += delay
                fake_time = types.SimpleNamespace(monotonic=lambda: elapsed[0],
                    time=lambda: 100.+elapsed[0], sleep=sleep)
                with modules, garbage, mock.patch.object(shadow, 'time', fake_time):
                    with self.assertRaises(TimeoutError):
                        shadow.RosObservations()
                self.assertEqual(elapsed[0], 2.)
                self.assertEqual([s.topic for s in ros.subscriptions],
                                 ['/clock', shadow.TOPICS['timing']])
                self.assertTrue(all(s.unregister_states == [True] for s in ros.subscriptions))
                self.assertEqual(gc_state.operations, [])

    def test_previous_first_missing_timing_still_fails_original_evaluator(self):
        rows = [
            {'kind': 'shadow', 'stamp_sim_s': 9.708, 'stamp_ns': 9708000000,
             'receipt_sim_s': 0., 'receipt_monotonic_s': 1.},
            {'kind': 'clock', 'stamp_sim_s': 9.708, 'receipt_monotonic_s': 1.000117688},
            {'kind': 'shadow', 'stamp_sim_s': 9.728, 'stamp_ns': 9728000000,
             'receipt_sim_s': 9.728, 'receipt_monotonic_s': 1.06},
            {'kind': 'timing', 'value': {'stamp_sim_s': 9.728, 'stamp_ns': 9728000000,
             'actual_publish_clock_sim_s': 9.728, 'measurement_age_at_publish_sim_s': 0.,
             'source': 'producer_ros_clock_at_publish', 'source_timestamp_unchanged': True}}]
        result = shadow.timestamp_assessment(rows)
        self.assertFalse(result['passed'])
        self.assertEqual(result['missing_count'], 1)
        self.assertEqual(result['timing_match_count'], 1)
        self.assertFalse(result['consumer_clock_reordering'][0]['proved'])


if __name__ == '__main__':
    unittest.main()
