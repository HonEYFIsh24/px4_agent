"""Views and reviews of existing semantic-map evidence; no second target memory.

Stereo validates surface geometry, not the VLM's class. Bearing samples below
are explicitly hypotheses for visibility prediction and never become target XYZ.
"""
import base64
import hashlib
import io
import math

from semantic_map import bbox, point_in_box

PAIRED_REVIEW_PROMPT = ('复核同一任务两个实际视角。图像依次为源完整图、源框裁图、当前完整图、'
    '各当前候选框裁图。裁图不是新视角。先客观描述每个框内像素，再判断对象及属性。'
    '不能把原标签、置信度、双目成功、几何假设或用户希望目标存在当成证明。'
    '框若主要覆盖路面、墙体或遮挡物，不能把框外的目标当框内证据；局部可见就明确局部。'
    'supported表示框内有可辨认的对象证据，不要求整个物体完整入镜；若局部的颜色和结构足以识别可为true，'
    '不可仅因裁切、不完整而一律否定，也不可只凭普通白色平面猜测车辆。'
    'current_target_id只能选列出的当前候选，未见对应对象时为空且current_supported=false。'
    '若源框或当前框像素不足以判断，应分别设置source_uncertain或current_uncertain为true，'
    '对应的supported为false；不确定不等于证明目标不存在。'
    '分别给出source_supported/current_supported/same_object以及源图和当前图的实际内容与理由。')


def prepare_review_images(source, current, source_box, current_boxes, evidence_dir=None):
    """Exact bounded paired-review images shared by the agent and offline replay."""
    from pathlib import Path
    if len(current_boxes) > 8 or len({i for i, _ in current_boxes}) != len(current_boxes):
        raise ValueError('Current review candidates must be unique and bounded')
    prefix = source['capture_id']+'.'+current['capture_id']
    if Path(prefix).name != prefix:
        raise ValueError('Invalid paired review image basename')
    def crop_for(frame, box, suffix):
        crop = review_crop(frame, box)
        metadata = {k: v for k, v in crop.items() if k != 'image_base64'}
        if evidence_dir:
            filename = prefix+suffix
            path = Path(evidence_dir)/filename
            raw = base64.b64decode(crop['image_base64'])
            if path.exists():
                if hashlib.sha256(path.read_bytes()).hexdigest() != crop['crop_sha256']:
                    raise ValueError('Archived confirmation ROI differs from actual model input')
            else:
                with path.open('xb') as handle:
                    handle.write(raw)
            metadata['archive_filename'] = filename
        return crop, metadata
    crop, source_metadata = crop_for(source, source_box, '.confirmation-roi.jpg')
    image_block = lambda data: {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,'+data}}
    content = [image_block(source['image_base64']), image_block(crop['image_base64']), image_block(current['image_base64'])]
    metadata = {'derived_roi': source_metadata, 'current_rois': []}
    for index, (identifier, box) in enumerate(current_boxes):
        crop, item = crop_for(current, box, '.current-'+str(index)+'.confirmation-roi.jpg')
        item.update(target_id=identifier, bbox=list(box))
        metadata['current_rois'].append(item)
        content.extend([{'type': 'text', 'text': '当前候选框：'+str(identifier)+'；bbox='+str(box)}, image_block(crop['image_base64'])])
    return content, metadata


def box_region_geometry(source, current):
    """Finite epipolar-box compatibility for clipped objects, not point identity.

    A clipped box centre need not represent the same surface in another view.
    Check overlap of calibrated viewing regions instead. This is deliberately
    only an association hypothesis; it never creates a position measurement.
    """
    from stereo_localization import iris_left_optical_to_body, matvec, quaternion_rotation
    boxes = [bbox(o['bbox']) for o in (source, current)]
    if not any(any(b[i] <= .005 for i in (0, 1)) or any(b[i] >= .995 for i in (2, 3)) for b in boxes):
        return {'compatible': False, 'reason': 'region correspondence is only available for image-clipped boxes'}
    def rays(observation, box):
        k = observation['camera_info']['K']
        mount = iris_left_optical_to_body()['rotation']
        attitude = quaternion_rotation(observation['body_pose_enu']['orientation_xyzw'])
        result = []
        for fx in (.1, .3, .5, .7, .9):
            for fy in (.1, .3, .5, .7, .9):
                pixel = [(box[0]+fx*(box[2]-box[0]))*640, (box[1]+fy*(box[3]-box[1]))*480]
                ray = matvec(attitude, matvec(mount, [(pixel[0]-k[2])/k[0], (pixel[1]-k[5])/k[4], 1.]))
                norm = math.sqrt(sum(v*v for v in ray))
                result.append(([v/norm for v in ray], (fx, fy)))
        return result
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    origins = [o['bearing_origin_enu_m'] for o in (source, current)]
    w = [a-b for a, b in zip(*origins)]
    supports = []
    for u, uv in rays(source, boxes[0]):
        for v, vv in rays(current, boxes[1]):
            cosine = dot(u, v)
            denominator = 1-cosine*cosine
            if denominator < math.sin(math.radians(3.))**2:
                continue
            du, dv = dot(u, w), dot(v, w)
            t, s = (cosine*dv-du)/denominator, (dv-cosine*du)/denominator
            if not .5 <= t <= 20. or not .5 <= s <= 20.:
                continue
            a = [o+d*t for o, d in zip(origins[0], u)]
            b = [o+d*s for o, d in zip(origins[1], v)]
            midpoint = [(x+y)/2 for x, y in zip(a, b)]
            # Pixel/angular residual, not a permissive fixed 0.5m cone gap.
            if math.dist(a, b) > min(t, s)*math.tan(math.radians(.5)):
                continue
            if point_in_box(midpoint, source) and point_in_box(midpoint, current):
                supports.append((uv, vv))
    spread = all(len({p[side][axis] for p in supports}) >= 2 for side in (0, 1) for axis in (0, 1))
    compatible = len(supports) >= 4 and spread
    return {'compatible': compatible, 'basis': 'clipped_box_epipolar_region_not_localization',
            'support_pair_count': len(supports), 'support_spans_both_box_axes': spread,
            'target_xyz_estimated': False, 'association_hypothesis_only': True,
            'reason': 'calibrated clipped-box regions have distributed compatible rays' if compatible else
                      'clipped-box regions lack distributed positive-depth epipolar support'}


def observations_for(semantic_map, query, capture_id):
    return [(landmark, observation) for landmark in semantic_map.landmarks.values()
            if landmark.label == query for observation in landmark.observations
            if observation['capture_id'] == capture_id]


def observation_for(semantic_map, target_id, capture_id):
    landmark = semantic_map.landmarks.get(target_id)
    if landmark is None:
        raise ValueError('Confirmation target is not in the owned semantic map')
    matches = [o for o in landmark.observations if o['capture_id'] == capture_id]
    if len(matches) != 1:
        raise ValueError('Confirmation requires one actual target observation in its source image')
    return landmark, matches[0]


def confirmation_goal(semantic_map, target_id, capture_id):
    landmark, observation = observation_for(semantic_map, target_id, capture_id)
    origin = observation['bearing_origin_enu_m']
    if observation['position_enu_m'] is not None:
        points = [list(observation['position_enu_m'])]
        # A measured curved surface can lie inside its enclosing raw AABB.
        # Add a bounded, camera-facing uncertainty sample, explicitly a probe,
        # rather than exempting the target's obstacle from line-of-sight checks.
        length = math.dist(origin, points[0])
        if length > .5:
            covariance = landmark.covariance_enu_m2
            uncertainty = min(.6, max(.1, math.sqrt(max(covariance[i][i] for i in range(3)))))
            points.append([p+(o-p)*uncertainty/length for p, o in zip(points[0], origin)])
        basis = 'actual_stereo_surface_with_bounded_uncertainty_probe'
    else:
        direction = observation['bearing_direction_enu']
        ground = semantic_map.grid['ground_enu_z_m']
        points = [[o+d*distance for o, d in zip(origin, direction)] for distance in (2., 4., 6., 8., 10.)]
        points = [p for p in points if p[2] >= ground-.15]
        basis = 'bearing_range_hypotheses_not_localization'
    if not points:
        raise ValueError('No geometrically valid bearing hypotheses for a confirmation view')
    return {'target_id': target_id, 'source_capture_id': capture_id, 'label': landmark.label,
            'source_observer_enu_m': list(observation['body_pose_enu']['position_m']),
            'source_camera_enu_m': list(origin), 'source_bbox': list(observation['bbox']),
            'probe_points_enu_m': points, 'probe_basis': basis,
            'hypotheses_are_target_locations': False}


def pair_geometry(source_landmark, source, current_landmark, current, policy='clipped_regions_v2'):
    """Association check only; it does not establish object existence or class."""
    if policy not in ('center_v1', 'clipped_regions_v2', 'clipped_regions_v3'):
        raise ValueError('Unknown confirmation geometry policy')
    if (source['capture_id'] == current['capture_id']
            or current['stamp_sim_s'] <= source['stamp_sim_s']
            or math.dist(source['body_pose_enu']['position_m'], current['body_pose_enu']['position_m']) < .5):
        return {'compatible': False, 'reason': 'requires newer image and observer baseline of at least 0.5m'}
    a, b = source['position_enu_m'], current['position_enu_m']
    if a is not None and b is not None:
        compatible = (source_landmark.id == current_landmark.id
                      and point_in_box(a, current) and point_in_box(b, source))
        return {'compatible': compatible, 'basis': 'shared_stereo_target_memory',
                'reason': 'same uniquely associated landmark with mutual image projections' if compatible else
                          'stereo observations are not uniquely associated to the same landmark'}
    if a is not None or b is not None:
        compatible = point_in_box(a, current) if a is not None else point_in_box(b, source)
        return {'compatible': compatible, 'basis': 'one_metric_surface_projects_to_other_image',
                'reason': 'actual measured surface projects into the other target box' if compatible else
                          'measured surface does not project into the other target box'}
    u, v = source['bearing_direction_enu'], current['bearing_direction_enu']
    boxes = [source['bbox'], current['bbox']]
    if policy in ('clipped_regions_v2', 'clipped_regions_v3') and any(any(b[i] <= .005 for i in (0, 1)) or any(b[i] >= .995 for i in (2, 3)) for b in boxes):
        return box_region_geometry(source, current)
    w = [a-b for a, b in zip(source['bearing_origin_enu_m'], current['bearing_origin_enu_m'])]
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    cosine = dot(u, v)
    denominator = 1.-cosine*cosine
    if denominator < math.sin(math.radians(3.))**2:
        return {'compatible': False, 'reason': 'bearing rays have insufficient parallax'}
    du, dv = dot(u, w), dot(v, w)
    t, s = (cosine*dv-du)/denominator, (dv-cosine*du)/denominator
    if not .5 <= t <= 20. or not .5 <= s <= 20.:
        return {'compatible': False, 'reason': 'bearing correspondence requires unsupported or negative depth'}
    a = [o+d*t for o, d in zip(source['bearing_origin_enu_m'], u)]
    b = [o+d*s for o, d in zip(current['bearing_origin_enu_m'], v)]
    midpoint = [(x+y)/2 for x, y in zip(a, b)]
    # Ephemeral intersection is only a compatibility test, never a localization.
    compatible = math.dist(a, b) <= .5 and point_in_box(midpoint, source) and point_in_box(midpoint, current)
    return {'compatible': compatible, 'basis': 'bearing_intersection_not_localization',
            'reason': 'distinct-view rays have a geometrically compatible intersection' if compatible else
                      'bearing rays or projected boxes do not correspond',
            'target_xyz_estimated': False}


def review_crop(frame, box, padding=.025):
    """Derived pixel crop of an owned JPEG; original image is never overwritten."""
    from PIL import Image
    box = bbox(box)
    raw = base64.b64decode(frame['image_base64'], validate=True)
    expected = frame['camera']['image_sha256']
    if hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError('Confirmation JPEG differs from its acquired image hash')
    with Image.open(io.BytesIO(raw)) as original:
        if original.size != (640, 480):
            raise ValueError('Confirmation expects calibrated 640x480 source images')
        region = [max(0, int((box[0]-padding)*640)), max(0, int((box[1]-padding)*480)),
                  min(640, math.ceil((box[2]+padding)*640)), min(480, math.ceil((box[3]+padding)*480))]
        crop = original.convert('RGB').crop(tuple(region))
        if min(crop.size) < 2:
            raise ValueError('Confirmation crop is too small')
        scale = min(4., max(1., 224./max(crop.size)))
        if scale > 1.:
            crop = crop.resize(tuple(max(2, round(v*scale)) for v in crop.size), Image.BICUBIC)
        buffer = io.BytesIO()
        crop.save(buffer, format='JPEG', quality=90)
    encoded = buffer.getvalue()
    return {'image_base64': base64.b64encode(encoded).decode(), 'source_capture_id': frame['capture_id'],
            'source_image_sha256': expected, 'crop_pixel_bounds': region,
            'crop_sha256': hashlib.sha256(encoded).hexdigest(), 'derived_not_new_capture': True}
