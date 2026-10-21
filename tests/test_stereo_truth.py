"""Independent recorder tests use fake ROS/processes; no live requests."""
import contextlib
import copy
import gc
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest import mock

from scripts import record_stereo_truth as truth
import test_stereo_positive as pair_fixture


def pose(p=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return NS(position=NS(x=p[0], y=p[1], z=p[2]), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))


def row(stamp, p=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return truth.model_states_body(NS(name=['iris'], pose=[pose(p, q)]), stamp)


def archived_pair(directory, identifier='a'*32):
    metadata = next(iter(pair_fixture.evidence()[-1].values()))
    metadata.pop('verified_archive_image_sha256')
    files = Path(__file__).resolve().parent/'fixtures'
    hashes = {}
    directory.mkdir(parents=True, exist_ok=True)
    for side, suffix in (('left','.jpg'), ('right','.right.jpg')):
        data = (files/('stereo_'+side+'.jpg')).read_bytes()
        hashes[side] = hashlib.sha256(data).hexdigest()
        (directory/(identifier+suffix)).write_bytes(data)
    metadata['capture_id'] = identifier
    metadata['camera']['image_sha256'] = hashes['left']
    metadata['snapshot']['stereo_image_sha256'] = copy.deepcopy(hashes)
    metadata['stereo_evidence']['image_sha256'] = copy.deepcopy(hashes)
    for side, suffix in (('left','.jpg'), ('right','.right.jpg')):
        member = metadata['stereo_evidence'][side]
        member['image_sha256'] = hashes[side]
        member['jpeg_filename'] = identifier+suffix
    (directory/(identifier+'.json')).write_text(json.dumps(metadata))
    return metadata


class FakeOwner:
    def __init__(self, directory):
        self.manager = {'run_directory': str(directory)}
        self.calls = 0
        self.foreign = False

    def check(self):
        self.calls += 1
        if self.foreign:
            raise RuntimeError('Pinned manager identity changed')

    def evidence(self):
        return {'manager': self.manager, 'identity_check_count': self.calls}


class FakeCollector:
    def __init__(self, samples):
        self.samples = list(samples)
        self.pending = list(samples)
        self.error = None
        self.closed = False

    def flush(self, stream):
        for sample in self.pending:
            stream.write(json.dumps(sample)+'\n')
        self.pending = []

    def snapshot(self):
        return list(self.samples)

    def evidence(self):
        return {'closed': self.closed, 'fake_ROS': True, 'record_count': len(self.samples)}

    def close(self):
        self.closed = True


class StereoTruthTests(unittest.TestCase):
    def test_truth_receipt_does_not_fake_source_and_local_pose_uses_actual_header(self):
        sample = row(10.)
        self.assertIsNone(sample['source_stamp_sim_s'])
        self.assertIsNone(sample['source_stamp_ns'])
        self.assertEqual(sample['receipt_ros_clock_sim_s'], 10.)
        self.assertEqual(sample['timestamp_semantics'], 'receipt_ros_clock_no_source_header')
        stamp = NS(to_sec=lambda: 9.99, to_nsec=lambda: 9990000000)
        actual = truth.local_pose_row(NS(header=NS(stamp=stamp, frame_id='map'), pose=pose((1.,2.,3.))), 10.)
        self.assertEqual(actual['source_stamp_sim_s'], 9.99)
        self.assertEqual(actual['source_stamp_ns'], 9990000000)
        self.assertEqual(actual['receipt_ros_clock_sim_s'], 10.)
        self.assertIsNone(truth.model_states_body(NS(name=['another'], pose=[pose()]), 10.))

    def test_strict_small_receipt_bracket_is_approximate_and_preserves_endpoints(self):
        before, after = row(9.98, (0.,0.,0.)), row(10.02, (4.,2.,0.), (0.,0.,0.,-1.))
        bracket = truth.receipt_truth_bracket([before, after], 10.)
        self.assertAlmostEqual(bracket['gap_sim_s'], .04)
        self.assertEqual(bracket['before'], before)
        self.assertEqual(bracket['after'], after)
        self.assertEqual(bracket['body_pose_world_approximate']['position_m'], [2.,1.,0.])
        self.assertEqual(bracket['body_pose_world_approximate']['orientation_xyzw'], [0.,0.,0.,1.])
        self.assertFalse(bracket['exact_source_time_ground_truth'])
        self.assertIn('no source header', bracket['limitation'])
        # Duplicates outside this capture's endpoints are preserved; they do
        # not silently force a sort or invalidate unrelated later brackets.
        previous=[row(8.),row(8.),before,after]
        self.assertEqual(truth.receipt_truth_bracket(previous,10.)['before'],before)
        scaled_a=row(9.98,q=(0.,0.,0.,1.002))
        scaled_b=row(10.02,q=(0.,0.,math.sqrt(.5)*.998,math.sqrt(.5)*.998))
        normalized=truth.receipt_truth_bracket([scaled_a,scaled_b],10.)['body_pose_world_approximate']['orientation_xyzw']
        self.assertAlmostEqual(normalized[2],math.sin(math.pi/8))
        self.assertAlmostEqual(normalized[3],math.cos(math.pi/8))
        self.assertEqual(scaled_a['orientation_xyzw'],[0.,0.,0.,1.002])

    def test_no_extrapolation_nonmonotonic_nan_fake_source_or_wide_truth_bracket(self):
        fake_source = row(10.02); fake_source['source_stamp_sim_s'] = 10.02
        scenarios = ([row(9.98)], [row(9.97),row(10.03)], [row(10.02),row(9.98)],
                     [row(9.98),row(9.98),row(10.02)], [row(9.98),fake_source])
        for samples in scenarios:
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                truth.receipt_truth_bracket(samples,10.)
        samples=[row(9.98),row(10.02)];samples[1]['receipt_ros_clock_sim_s']=float('nan')
        with self.assertRaises(ValueError): truth.receipt_truth_bracket(samples,10.)
        with self.assertRaises(ValueError): truth.receipt_truth_bracket([row(9.98),row(10.02)],10.,.1)

    def test_optical_world_transform_uses_actual_body_truth_mounting_and_ignores_enu_gauge(self):
        angle = .35
        value = {'position_m':[10.,20.,30.], 'orientation_xyzw':[0.,0.,0.,1.],
                 'frame_id':'gazebo_world','body_reference':'base_link',
                 'position_enu_m':[-100.,80.,60.], 'initial_body_yaw_enu':1.3}
        point = truth.optical_surface_to_world([0.,0.,2.],value)
        self.assertAlmostEqual(point[0],10.+.12+2.*math.cos(angle))
        self.assertAlmostEqual(point[1],20.+.06)
        self.assertAlmostEqual(point[2],30.+.18-2.*math.sin(angle))
        value['position_enu_m']=[999.,999.,999.];value['initial_body_yaw_enu']=-2.
        self.assertEqual(truth.optical_surface_to_world([0.,0.,2.],value),point)
        value['frame_id']='local_enu'
        with self.assertRaises(ValueError):truth.optical_surface_to_world([0.,0.,2.],value)

    def test_pair_archive_reads_exact_left_right_hash_stamps_and_keeps_no_fake_nanoseconds(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager=Path(temporary)/'manager';source=manager/'captures';metadata=archived_pair(source)
            destination=Path(temporary)/'new';destination.mkdir()
            owner=FakeOwner(manager)
            actual=truth.archive_pair('a'*32,owner,destination)
            self.assertEqual(actual['source_stamp_sim_s'],10.)
            self.assertIsNone(actual['source_stamp_ns'])
            self.assertEqual(actual['verified_image_sha256'],metadata['stereo_evidence']['image_sha256'])
            self.assertEqual(actual['stereo_evidence']['body_pose_enu'],metadata['snapshot']['body_pose_enu'])
            self.assertEqual(actual['stereo_evidence']['coordinate_epoch'],metadata['snapshot']['coordinate_epoch'])
            for suffix in ('.jpg','.right.jpg','.json'):
                self.assertEqual((destination/('a'*32+suffix)).read_bytes(),(source/('a'*32+suffix)).read_bytes())
            self.assertNotIn('image_base64',json.dumps(actual))
            self.assertGreaterEqual(owner.calls,2)
            corrupt=Path(temporary)/'corrupt';corrupt.mkdir()
            (source/('a'*32+'.right.jpg')).write_bytes(b'invalid')
            with self.assertRaises(ValueError):truth.archive_pair('a'*32,owner,corrupt)
            self.assertEqual(list(corrupt.iterdir()),[])

    def test_legacy_manifest_does_not_claim_pair_and_foreign_owner_during_read_never_copies(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager=Path(temporary)/'manager';source=manager/'captures';metadata=archived_pair(source)
            destination=Path(temporary)/'legacy';destination.mkdir()
            metadata.pop('stereo_evidence');(source/('a'*32+'.json')).write_text(json.dumps(metadata))
            owner=FakeOwner(manager)
            result=truth.archive_pair('a'*32,owner,destination)
            self.assertEqual(result['kind'],'captured_pair_unavailable')
            self.assertNotIn('source_stamp_sim_s',result)
            self.assertEqual(list(destination.iterdir()),[])
            archived_pair(source)
            read=truth._read_bounded
            def change_owner(path,limit):
                raw=read(path,limit)
                if path.name.endswith('.right.jpg'):owner.foreign=True
                return raw
            with mock.patch.object(truth,'_read_bounded',side_effect=change_owner),self.assertRaisesRegex(RuntimeError,'identity changed'):
                truth.archive_pair('a'*32,owner,destination)
            self.assertEqual(list(destination.iterdir()),[])

    def test_foreign_owner_fails_before_ros_subscriptions_and_never_performs_recovery(self):
        with tempfile.TemporaryDirectory() as temporary:
            owner=FakeOwner(Path(temporary)/'manager');owner.foreign=True
            constructor=mock.Mock(side_effect=AssertionError('ROS must not start'))
            result=truth.record(Path(temporary)/'new',duration=.2,ownership_factory=lambda *a:owner,
                                collector_factory=constructor)
            self.assertFalse(result['recording_completed'])
            self.assertIn('identity changed',result['error']['message'])
            constructor.assert_not_called()
            self.assertFalse(result['actual_control_requests'])
            self.assertFalse(result['actual_model_requests'])

    def test_bounded_fake_record_captures_only_new_manifest_and_keeps_fail_on_owner_change(self):
        for foreign in (False,True):
            with self.subTest(foreign=foreign),tempfile.TemporaryDirectory() as temporary:
                manager=Path(temporary)/'manager';source=manager/'captures'
                archived_pair(source,'b'*32)
                owner=FakeOwner(manager);now=[0.]
                collector=FakeCollector([row(9.98),row(10.02)])
                def sleep(seconds):
                    now[0]+=seconds
                    if now[0]>=.1:
                        if not (source/('a'*32+'.json')).exists():archived_pair(source)
                        if foreign:owner.foreign=True
                output=Path(temporary)/'new'
                result=truth.record(output,duration=.3,poll=.1,ownership_factory=lambda *a:owner,
                                    collector_factory=lambda:collector,monotonic=lambda:now[0],sleep=sleep)
                self.assertTrue(collector.closed)
                self.assertEqual(result['recording_completed'],not foreign)
                self.assertEqual(result['preexisting_capture_count_excluded'],1)
                self.assertFalse((output/'captures'/('b'*32+'.jpg')).exists())
                self.assertFalse(result['ground_truth_exposed_to_agent'])
                self.assertFalse(result['enu_to_world_gauge_estimated'])
                self.assertTrue((output/'observations.jsonl').read_text())
                if not foreign:
                    self.assertEqual(result['captured_pairs'],1)
                    self.assertTrue(result['all_captured_pairs_have_receipt_brackets'])
                    association=json.loads((output/'associations.json').read_text())['captures'][0]
                    self.assertEqual(association['coordinate_epoch'],pair_fixture.search_fixture.EPOCH)
                else:
                    self.assertEqual(list((output/'captures').iterdir()),[])
                with self.assertRaises(FileExistsError):truth.record(output,duration=.2)

    def test_final_poll_retains_last_committed_pair_and_no_missing_bracket_success_claim(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager=Path(temporary)/'manager';owner=FakeOwner(manager);now=[0.]
            collector=FakeCollector([row(9.98)])
            def sleep(seconds):
                now[0]+=seconds
                archived_pair(manager/'captures')
            output=Path(temporary)/'new'
            result=truth.record(output,duration=.1,poll=.1,ownership_factory=lambda *a:owner,
                                collector_factory=lambda:collector,monotonic=lambda:now[0],sleep=sleep)
            self.assertTrue(result['recording_completed'])
            self.assertEqual(result['captured_pairs'],1)
            self.assertFalse(result['all_captured_pairs_have_receipt_brackets'])
            self.assertFalse(result['metric_accuracy_ground_truth_verified'])
            association=json.loads((output/'associations.json').read_text())['captures'][0]
            self.assertEqual(association['truth']['status'],'unavailable')
            self.assertIn('both sides',association['truth']['reason'])

    def test_nearby_enu_samples_remain_raw_source_evidence_without_world_gauge_or_interpolation(self):
        def sample(stamp,point):
            return truth.local_pose_row(NS(header=NS(stamp=NS(to_sec=lambda:stamp),frame_id='map'),
                                           pose=pose(point)),10.)
        a,b,c=sample(9.99,(1.,0.,3.5)),sample(10.02,(2.,0.,3.5)),sample(11.,(9.,0.,3.5))
        result=truth.actual_enu_neighbors([a,b,c],10.)
        self.assertEqual(result,[a,b]);self.assertIsNone(a['source_stamp_ns'])
        a['timestamp_semantics']='receipt_ros_clock_no_source_header'
        with self.assertRaises(ValueError):truth.actual_enu_neighbors([a,b],10.)

    def test_actual_ros_constructor_subscribes_only_light_topics_queue_one_and_restores_gc(self):
        modules={name:ModuleType(name) for name in ('rospy','gazebo_msgs','gazebo_msgs.msg',
                    'geometry_msgs','geometry_msgs.msg','rosgraph_msgs','rosgraph_msgs.msg')}
        rospy=modules['rospy'];rospy.init_node=mock.Mock();rospy.Time=NS(now=lambda:NS(to_sec=lambda:10.))
        rospy.Publisher=mock.Mock(side_effect=AssertionError('publishing forbidden'))
        rospy.ServiceProxy=mock.Mock(side_effect=AssertionError('controls forbidden'))
        for package,klass in (('gazebo_msgs','ModelStates'),('geometry_msgs','PoseStamped'),('rosgraph_msgs','Clock')):
            setattr(modules[package+'.msg'],klass,type(klass,(),{}))
        subscriptions=[]
        def subscribe(topic,kind,callback,**options):
            entry=NS(topic=topic,callback=callback,options=options,unregister=mock.Mock())
            subscriptions.append(entry);return entry
        rospy.Subscriber=subscribe
        previous=gc.isenabled()
        with mock.patch.dict(sys.modules,modules):
            collector=truth.RosTruthObservations()
            self.assertFalse(gc.isenabled())
            self.assertEqual([s.topic for s in subscriptions],['/clock','/gazebo/model_states','/mavros/local_position/pose'])
            self.assertTrue(all(s.options['queue_size']==1 and s.options['tcp_nodelay'] for s in subscriptions))
            subscriptions[1].callback(NS(name=['iris'],pose=[pose()]))
            self.assertEqual(collector.snapshot()[0]['source_stamp_sim_s'],None)
            collector.close();collector.close()
            self.assertEqual(gc.isenabled(),previous)
            self.assertTrue(all(s.unregister.call_count==1 for s in subscriptions))
            rospy.Publisher.assert_not_called();rospy.ServiceProxy.assert_not_called()

    def test_partial_subscribe_failure_closes_already_created_subscriptions_without_changing_gc(self):
        modules={name:ModuleType(name) for name in ('rospy','gazebo_msgs','gazebo_msgs.msg',
                    'geometry_msgs','geometry_msgs.msg','rosgraph_msgs','rosgraph_msgs.msg')}
        rospy=modules['rospy'];rospy.init_node=mock.Mock()
        for package,klass in (('gazebo_msgs','ModelStates'),('geometry_msgs','PoseStamped'),('rosgraph_msgs','Clock')):
            setattr(modules[package+'.msg'],klass,type(klass,(),{}))
        first=NS(unregister=mock.Mock());rospy.Subscriber=mock.Mock(side_effect=[first,RuntimeError('subscribe failure')])
        previous=gc.isenabled()
        with mock.patch.dict(sys.modules,modules),self.assertRaisesRegex(RuntimeError,'subscribe failure'):
            truth.RosTruthObservations()
        self.assertEqual(gc.isenabled(),previous)
        first.unregister.assert_called_once()

    def test_local_master_default_plan_creates_no_ros_owner_or_files(self):
        for uri in ('http://foreign:11311','http://user:secret@localhost:11311',
                    'http://localhost:11312','http://localhost:11311/?key=x'):
            with self.subTest(uri=uri),self.assertRaises(ValueError):truth.local_ros_master(uri)
        self.assertEqual(truth.local_ros_master('http://localhost:11311'),'http://localhost:11311')
        with mock.patch.object(truth,'record',side_effect=AssertionError('record forbidden')),\
             mock.patch.object(truth,'LocalSimulationOwner',side_effect=AssertionError('owner read forbidden')),\
             mock.patch.object(truth,'RosTruthObservations',side_effect=AssertionError('ROS forbidden')),\
             mock.patch.object(Path,'mkdir',side_effect=AssertionError('file write forbidden')),\
             contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(truth.main([]),0)
        plan=json.loads(stream.getvalue())
        self.assertFalse(plan['actual_requests']);self.assertFalse(plan['ros_subscriptions_started'])
        self.assertEqual(plan['image_topic_subscriptions'],[])


if __name__=='__main__':
    unittest.main()
