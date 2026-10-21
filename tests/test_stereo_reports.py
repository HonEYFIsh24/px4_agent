import copy
import threading
import types
import unittest
from unittest import mock
from stereo_cache import StereoCache
from stereo_reports import StereoReports


def pair():
    info={'stamp_sim_s':10.,'width':640,'height':480,'K':[320.,0.,320.,0.,320.,240.,0.,0.,1.]}
    return {'now_sim_s':10.1,'left':{'stamp_sim_s':10.,'age_seconds':.1,'camera_intrinsics':copy.deepcopy(info)},
            'right':{'stamp_sim_s':10.,'age_seconds':.1,'camera_intrinsics':copy.deepcopy(info)},
            'body_pose_enu':{'stamp_sim_s':10.,'age_seconds':.1,'frame_id':'map','body_reference':'base_link',
                'position_m':[0.,0.,3.5],'orientation_xyzw':[0.,0.,0.,1.]}}


class HistoricalStereoTests(unittest.TestCase):
    def test_capture_freshness_and_expiration_are_distinct(self):
        now=[0.]
        store=StereoReports(clock=lambda:now[0],ttl=180)
        item=pair(); key=store.put(item)
        item['body_pose_enu']['position_m'][0]=50
        self.assertEqual(store.frames[key]['pair']['body_pose_enu']['position_m'][0],0)
        now[0]=181
        with self.assertRaisesRegex(ValueError,'expired'):
            store.localize(key,[])
        future=pair(); future['now_sim_s']=9.9
        with self.assertRaises(ValueError):
            store.put(future)

    def test_invalid_capture_and_target_budget_do_not_access_geometry(self):
        store=StereoReports()
        with mock.patch('stereo_localization.compute_disparities') as cv:
            for boxes in (None,[[]]*9):
                with self.assertRaises(ValueError):
                    store.localize('unowned',boxes)
            with self.assertRaises(ValueError):
                store.localize('unowned',[])
            cv.assert_not_called()

    def test_empty_target_report_uses_saved_frame_without_cv_or_depth(self):
        store=StereoReports()
        key=store.put(pair())
        with mock.patch('stereo_localization.compute_disparities') as cv:
            result=store.localize(key,[])
        self.assertEqual(result['targets'],[])
        self.assertTrue(result['historical_observation'])
        cv.assert_not_called()

    def test_pairing_requires_exact_source_stamp_and_limits_pending(self):
        entered=threading.Event(); release=threading.Event()
        def encode(m):
            entered.set(); release.wait(1)
            return {'jpeg':b'frame','width':640,'height':480}
        cache=StereoCache(encode,lambda:10.1,wall_clock=lambda:100.)
        self.addCleanup(cache.close)
        def image(stamp):
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                to_nsec=lambda:int(stamp*1e9),to_sec=lambda:stamp)))
        cache.push('left',image(10.)); cache.push('right',image(10.02))
        self.assertFalse(entered.is_set())
        for n in range(10):
            cache.push('left',image(9.+n*.004))
        self.assertLessEqual(len(cache.pending['left']),8)
        cache.push('left',image(10.)); cache.push('right',image(10.))
        self.assertTrue(entered.wait(1))
        release.set()
