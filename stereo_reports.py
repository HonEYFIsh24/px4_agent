"""Immutable historical stereo observations; no flight or estimator writes."""
import copy
import threading
import time
import uuid
from search_tools import coordinate_epoch


class StereoReports:
    def __init__(self, clock=time.monotonic, ttl=180, capacity=8):
        self.clock, self.ttl, self.capacity = clock, ttl, capacity
        self.lock = threading.Lock()
        self.frames = {}

    def put(self, pair):
        from stereo_localization import validate_pair
        validate_pair(pair, pair['now_sim_s'])
        key = uuid.uuid4().hex
        with self.lock:
            now = self.clock()
            self.frames = {k: v for k, v in self.frames.items() if now-v['created'] <= self.ttl}
            while len(self.frames) >= self.capacity:
                del self.frames[next(iter(self.frames))]
            self.frames[key] = {'pair': copy.deepcopy(pair), 'created': now}
        return key

    def localize(self, identifier, bboxes):
        from stereo_localization import (validate_pair, calibration_from_camera_info,
            decode_frame, compute_disparities, estimate_from_disparities, StereoLocalizationError)
        if not isinstance(bboxes, list) or len(bboxes) > 8:
            raise ValueError('at most eight target boxes are supported')
        with self.lock:
            frame = self.frames.get(identifier)
            if not frame or self.clock()-frame['created'] > self.ttl:
                raise ValueError('stereo observation is absent or expired')
            pair = copy.deepcopy(frame['pair'])
            elapsed = self.clock()-frame['created']
        # This is the already-validated acquisition instant. Do not relabel the
        # historical image or pose with the model response time.
        stamp, limits = validate_pair(pair, pair['now_sim_s'])
        if not bboxes:
            return {'status': 'ok', 'pair_id': identifier, 'targets': [],
                    'historical_observation': True, 'report_delay_wall_s': elapsed}
        calibration = calibration_from_camera_info(pair['left']['camera_intrinsics'], pair['right']['camera_intrinsics'], .12)
        left, left_hash = decode_frame(pair['left'])
        right, right_hash = decode_frame(pair['right'])
        dl, dr = compute_disparities(left, right)
        results = []
        for box in bboxes:
            try:
                result = estimate_from_disparities(left, dl, dr, box, calibration, pair['body_pose_enu'], stamp, limits)
                result.update(image_sha256={'left': left_hash, 'right': right_hash},
                              coordinate_epoch=coordinate_epoch(pair.get('external_vision')),
                              capture_pair_id=identifier, historical_observation=True,
                              report_delay_wall_s=elapsed, usable_as_flight_waypoint=False)
            except StereoLocalizationError as exc:
                result = {'status': 'unavailable', 'reason': str(exc), 'image_stamp_sim_s': stamp,
                          'capture_pair_id': identifier, 'usable_as_flight_waypoint': False}
            results.append(result)
        return {'status': 'ok', 'pair_id': identifier, 'targets': results,
                'historical_observation': True, 'report_delay_wall_s': elapsed}
