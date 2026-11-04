"""Read-only bearing feedback for photo planning, not geometry or flight authority.

Re-expresses image bbox rays under hypothetical ENU yaw. No depth, scene truth,
synthetic image, coverage score, waypoint, or whole-subject extent is inferred.
"""
import math

from stereo_localization import (finite, iris_left_optical_to_body, matmul, matvec,
                                 quaternion_rotation, transpose, vector)


LIMITATIONS = [
    'Only observed bbox bearing directions; boxes are uncalibrated visual model outputs, not segmentation or complete subject extent.',
    'Orientation-only far-field approximation: ignores camera lever-arm translation during body yaw and attitude settling.',
    'Does not predict unseen content, depth, occlusion, actual new pixels, or whole-subject completeness.',
    'No translation/altitude prediction without reliable 3D extent. A representative stereo surface point is not object extent.',
    'No safety or execution authorization; next action still requires current pose, coordinate identity and executor validation.',
]


def photo_bearing_feedback(frame, report):
    """Optional diagnostic: invalid/missing inputs never invent a projection.

    Caller must supply its owned, delivered, hash-verified frame and saved report.
    This function additionally checks their identities and capture-time geometry.
    Unavailable feedback is not a flight fault and grants no completion or move.
    """
    result = dict(schema_version=1, status='unavailable', capture_id=frame.get('capture_id'),
        source='actual_report_bbox_camera_calibration_body_pose',
        usable_for_flight_authorization=False, predicts_whole_subject_complete=False,
        limitations=list(LIMITATIONS))
    try:
        if (not frame.get('capture_id') or report.get('capture_id') != frame['capture_id']
                or report.get('status') != 'ok'):
            raise ValueError('frame/report identity mismatch')
        camera = frame['camera']; snapshot = frame['snapshot']
        digest = camera.get('image_sha256')
        if not isinstance(digest, str) or len(digest) != 64 or report['camera'].get('image_sha256') != digest:
            raise ValueError('frame/report image hash mismatch')
        if camera.get('side') != 'left' or snapshot.get('side') != 'left':
            raise ValueError('only calibrated Iris left camera is supported')
        info = snapshot['camera_intrinsics']; pose = snapshot['body_pose_enu']
        if (info.get('width'), info.get('height')) != (640, 480) or (camera.get('width'), camera.get('height')) != (640, 480):
            raise ValueError('unsupported camera dimensions')
        k = vector(info['K'], 9, 'camera K')
        if (k[0] <= 0 or k[4] <= 0 or abs(k[1])+abs(k[3]) > 1e-9
                or k[6:] != [0., 0., 1.] or not 0 <= k[2] <= 640 or not 0 <= k[5] <= 480):
            raise ValueError('invalid rectified pinhole intrinsics')
        distortion = info.get('distortion_coefficients', info.get('D'))
        if not isinstance(distortion, (list, tuple)) or any(abs(finite(v, 'distortion')) > 1e-9 for v in distortion):
            raise ValueError('missing or nonzero distortion')
        if vector(info['R'], 9, 'rectification R') != [1.,0.,0.,0.,1.,0.,0.,0.,1.]:
            raise ValueError('unsupported rectification')
        if pose.get('frame_id') != 'map' or pose.get('body_reference') != 'base_link':
            raise ValueError('expected ENU base_link pose')
        stamp = finite(camera['stamp_sim_s'], 'capture stamp')
        if (abs(stamp-finite(snapshot['stamp_sim_s'], 'snapshot stamp')) > 1e-6
                or abs(stamp-finite(pose['stamp_sim_s'], 'pose stamp')) > .05
                or abs(stamp-finite(report['camera']['stamp_sim_s'], 'report stamp')) > 1e-6):
            raise ValueError('capture and geometry timestamps differ')
        optical = matmul(quaternion_rotation(pose['orientation_xyzw']), iris_left_optical_to_body()['rotation'])
        targets = report['targets']
        if not isinstance(targets, list) or not 1 <= len(targets) <= 8:
            raise ValueError('no bounded visual target boxes')
        rows = []
        for target in targets:
            label = target['label']
            if not isinstance(label, str) or not label.strip() or len(label) > 200:
                raise ValueError('invalid visual label')
            x1,y1,x2,y2 = vector(target['bbox'], 4, 'bbox')
            if not 0 <= x1 < x2 <= 1 or not 0 <= y1 < y2 <= 1:
                raise ValueError('invalid normalized bbox')
            rays = [[(x*640-k[2])/k[0], (y*480-k[5])/k[4], 1.]
                    for y in (y1,(y1+y2)/2,y2) for x in (x1,(x1+x2)/2,x2)]
            if not all(math.isfinite(v) for ray in rays for v in ray):
                raise ValueError('nonfinite calibrated bearing')
            predictions = []
            for angle in (-45.,-30.,-15.,0.,15.,30.,45.):
                d = math.radians(angle); c,s = math.cos(d),math.sin(d)
                hypothetical = matmul([[c,-s,0.],[s,c,0.],[0.,0.,1.]], optical)
                relative = matmul(transpose(hypothetical), optical)
                points = []
                for ray in rays:
                    x,y,z = matvec(relative, ray)
                    point = [(k[0]*x/z+k[2])/640, (k[4]*y/z+k[5])/480] if z > 1e-6 else None
                    if point is not None and not all(math.isfinite(v) for v in point):
                        raise ValueError('nonfinite projected bearing')
                    points.append(point)
                bounds = ([min(p[0] for p in points),min(p[1] for p in points),
                           max(p[0] for p in points),max(p[1] for p in points)]
                          if all(p is not None for p in points) else None)
                predictions.append(dict(direction='left' if angle > 0 else 'right' if angle < 0 else 'unchanged',
                    angle_deg=abs(angle), projected_known_bbox_direction_bounds_normalized=bounds,
                    bbox_center_direction_normalized=points[4], total_sampled_bearings=len(points),
                    sampled_known_bbox_bearings_inside=sum(p is not None and all(-1e-9 <= v <= 1+1e-9 for v in p) for p in points)))
            rows.append(dict(label=label, source_bbox_normalized=[x1,y1,x2,y2], turn_bearing_effects=predictions))
        result.update(status='ok', image_sha256=digest, stamp_sim_s=stamp, yaw_positive='left_ENU', targets=rows)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        result['reason'] = str(exc)
    return result
