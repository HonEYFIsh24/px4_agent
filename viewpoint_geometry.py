"""Read-only fixed-heading candidate visibility/path queries, without semantic truth."""
import copy
import hashlib
import json
import math
import re

from mission import MAX_ALTITUDE, MissionError
from semantic_map import epoch_identity
from stereo_localization import vector, finite, quaternion_rotation, covariance_matrix
from search_limits import grid_dimensions


def registered_identity(navigator, coordinate_epoch):
    if not navigator.registration_id or navigator.origin is None:
        raise MissionError('viewpoints require a registered static map')
    if not re.fullmatch(r'[0-9a-f]{64}', navigator.world_sha256 or ''):
        raise MissionError('viewpoints require a verified world SHA256')
    return {'world_sha256': navigator.world_sha256, 'sortie_id': navigator.registration_id,
            'coordinate_epoch': epoch_identity(coordinate_epoch),
            'registration': {'world_from_enu_origin_m': list(navigator.origin),
                             'world_from_enu_yaw_rad': navigator.yaw,
                             'ground_world_z_m': navigator.ground_world_z}}


class StaticViewpointEvaluator:
    """Evaluates predictions; no coverage update, mission creation or last_plan write."""
    def __init__(self, navigator, pose, camera_info, coordinate_epoch, fixed_yaw=None,
                 reference_kind='archived_capture', probe_points_enu_m=None, allow_yaw=False,
                 search_radius_m=12., grid_resolution_m=.5, endpoint_tracking_reserve_m=0.,
                 probe_covariances_enu_m2=None):
        grid_dimensions(search_radius_m,grid_resolution_m)
        self.search_radius, self.grid_resolution = search_radius_m,grid_resolution_m
        self.navigator = navigator
        self.endpoint_tracking_reserve = finite(endpoint_tracking_reserve_m, 'endpoint tracking reserve')
        if not 0 <= self.endpoint_tracking_reserve <= .5:
            raise MissionError('endpoint tracking reserve must be within 0..0.5m')
        self.allow_yaw = allow_yaw is True
        self.pose = copy.deepcopy(pose)
        self.camera_info = copy.deepcopy(camera_info)
        self.coordinate_epoch = copy.deepcopy(coordinate_epoch)
        self.position = vector(pose['position_m'], 3, 'viewpoint source position')
        if probe_points_enu_m is not None and (not isinstance(probe_points_enu_m, list)
                                               or not 1 <= len(probe_points_enu_m) <= 16):
            raise MissionError('confirmation visibility requires 1..16 bounded probe points')
        self.probes = None if probe_points_enu_m is None else [vector(p, 3, 'confirmation probe') for p in probe_points_enu_m]
        self.probe_covariances = None
        if probe_covariances_enu_m2 is not None:
            if (not self.probes or not isinstance(probe_covariances_enu_m2, list)
                    or len(probe_covariances_enu_m2) != len(self.probes)):
                raise MissionError('surface probe covariances must match the actual probe points')
            self.probe_covariances = [covariance_matrix(c, 'actual surface probe covariance')
                                      for c in probe_covariances_enu_m2]
            if any(any(c[i][i] <= 0 for i in range(3)) for c in self.probe_covariances):
                raise MissionError('surface probes require positive actual position variances')
        if self.probes and any(math.dist(p, self.position) > 20. for p in self.probes):
            raise MissionError('confirmation probe exceeds actual observation range')
        if pose.get('frame_id') not in ('map', 'local_enu') or pose.get('body_reference') != 'base_link':
            raise MissionError('viewpoints require an ENU base_link source pose')
        rotation = quaternion_rotation(pose['orientation_xyzw'])
        self.yaw = (math.atan2(rotation[1][0], rotation[0][0]) if fixed_yaw is None
                    else finite(fixed_yaw, 'fixed commanded yaw'))
        if reference_kind not in ('archived_capture', 'live_enu_pose'):
            raise MissionError('unknown viewpoint pose provenance')
        self.reference_kind = reference_kind
        if (camera_info.get('width') != 640 or camera_info.get('height') != 480
                or len(camera_info.get('K', [])) != 9):
            raise MissionError('unsupported viewpoint camera calibration')
        k = vector(camera_info['K'], 9, 'viewpoint intrinsics')
        if k[0] <= 0 or k[4] <= 0:
            raise MissionError('invalid viewpoint focal length')

    def endpoint_has_tracking_room(self, point, origin):
        """Keep the whole endpoint tolerance box outside existing inflation."""
        from static_navigation import intersects
        reserve = self.endpoint_tracking_reserve
        if not reserve:
            return True
        if (math.dist(point[:2], origin)+reserve > self.search_radius+1e-9
                or point[2]-reserve < 1. or point[2]+reserve > MAX_ALTITUDE):
            return False
        world = self.navigator.to_world(point)
        if any(not self.navigator.bounds[0][i]+reserve <= world[i] <= self.navigator.bounds[1][i]-reserve
               for i in range(3)):
            return False
        return not any(intersects(world, world, [v-reserve for v in low], [v+reserve for v in high])
                       for low, high in self.navigator.boxes)

    def query(self, operation, origin_xy, expected_identity, candidates):
        if operation not in ('context', 'visibility', 'paths'):
            raise MissionError('unknown viewpoint query operation')
        identity = registered_identity(self.navigator, self.coordinate_epoch)
        if identity != expected_identity:
            raise MissionError('viewpoint world, sortie, registration or epoch changed')
        origin = vector(origin_xy, 2, 'viewpoint search origin')
        limit = 8 if operation == 'paths' else 64
        if (not isinstance(candidates, list) or len(candidates) > limit
                or (operation == 'context' and candidates)):
            raise MissionError('invalid viewpoint query batch')
        identifiers, validated = set(), []
        for candidate in candidates:
            fields = set(candidate) if isinstance(candidate, dict) else set()
            if fields not in ({'id', 'position_enu_m'}, {'id', 'position_enu_m', 'heading_step'}):
                raise MissionError('candidate requires id, position and optional bounded heading step')
            heading_step = candidate.get('heading_step')
            if 'heading_step' in candidate and (not self.allow_yaw or type(heading_step) is not int
                                                 or heading_step not in (-1, 1)):
                raise MissionError('candidate heading requires enabled discrete rotation')
            identifier = candidate['id']
            if (not isinstance(identifier, str) or not re.fullmatch(r'view_[0-9]{3}', identifier)
                    or identifier in identifiers):
                raise MissionError('invalid or duplicate viewpoint candidate ID')
            identifiers.add(identifier)
            validated.append((identifier, vector(candidate['position_enu_m'], 3, 'candidate position'), heading_step))
        response = {'source': 'verified_static_3d_viewpoint_prediction', 'prediction_only': True,
                    'execution_authorized': False, 'requires_executor_revalidation': True,
                    'identity': identity, 'start_enu_m': list(self.position),
                    'origin_xy_enu_m': origin, 'search_radius_m': self.search_radius,
                    'fixed_yaw_enu_rad': self.yaw, 'source_pose_kind': self.reference_kind,
                    'camera_calibration_sha256': hashlib.sha256(json.dumps(self.camera_info,
                        sort_keys=True, allow_nan=False).encode()).hexdigest(), 'results': []}
        if self.endpoint_tracking_reserve:
            response['endpoint_tracking_reserve_m'] = self.endpoint_tracking_reserve
        if self.probes is not None:
            response['probe_points_enu_m'] = copy.deepcopy(self.probes)
        if self.probe_covariances is not None:
            response['probe_covariances_enu_m2'] = copy.deepcopy(self.probe_covariances)
        for identifier, point, heading_step in validated:
            result = {'id': identifier, 'position_enu_m': point, 'reachable': False}
            candidate_yaw = self.yaw
            if heading_step is not None:
                candidate_yaw = math.atan2(math.sin(self.yaw + heading_step*math.pi/4),
                                           math.cos(self.yaw + heading_step*math.pi/4))
                result.update(heading_step=heading_step, target_yaw_enu_rad=candidate_yaw)
            if (not 1. <= point[2] <= MAX_ALTITUDE or math.dist(point[:2], origin) > self.search_radius+1e-9
                    or not self.navigator.segment_safe(point, point)):
                result['reason'] = 'candidate lacks search-volume or inflated flight clearance'
            elif not self.endpoint_has_tracking_room(point, origin):
                result['reason'] = 'candidate lacks endpoint tracking reserve outside existing flight clearance'
            elif operation == 'visibility':
                pose = {'position_m': point, 'frame_id': 'map', 'body_reference': 'base_link',
                        'orientation_xyzw': [0., 0., math.sin(candidate_yaw/2), math.cos(candidate_yaw/2)]}
                visible = self.navigator.observation_visibility(pose, self.camera_info, origin,
                    radius=self.search_radius,resolution=self.grid_resolution)
                result.update(reachable=True, point_clearance_only=True,
                              visible_cell_ids=visible['visible_cell_ids'])
                if self.probes is not None:
                    from coverage_search import project_ground_point
                    result['probe_visibility'] = []
                    for index, probe in enumerate(self.probes):
                        projection = project_ground_point(self.camera_info, pose, probe, 20.)
                        row = {
                            'point_enu_m': list(probe), 'inside_fov_and_range': projection['inside_fov_and_range'],
                            'unoccluded': self.navigator.raw_line_of_sight(visible['camera_origin_enu_m'], probe),
                            'pixel_uv': projection['pixel_uv'], 'camera_origin_enu_m': visible['camera_origin_enu_m']}
                        if self.probe_covariances is not None:
                            delta = [b-a for a,b in zip(visible['camera_origin_enu_m'], probe)]
                            distance = math.sqrt(sum(v*v for v in delta))
                            direction = [v/distance for v in delta] if distance > 0 else [0., 0., 0.]
                            matrix = self.probe_covariances[index]
                            variance = sum(direction[i]*matrix[i][j]*direction[j] for i in range(3) for j in range(3))
                            margin = 2*math.sqrt(max(0., variance))
                            row.update(surface_visibility_possible=self.navigator.raw_surface_visibility_possible(
                                visible['camera_origin_enu_m'], probe, margin),
                                endpoint_uncertainty_m=margin,
                                surface_visibility_basis='actual_covariance_projected_2sigma_endpoint_only',
                                prediction_only=True, requires_fresh_visual_verification=True)
                        result['probe_visibility'].append(row)
            elif operation == 'paths':
                try:
                    plan = self.navigator.preview_path(self.position, point, origin,self.search_radius)
                except MissionError as exc:
                    result['reason'] = str(exc)
                else:
                    result.update(reachable=True, path=plan)
            response['results'].append(result)
        return response


class RemoteViewpointEvaluator:
    """Internal client adapter; candidates are not coordinates generated by the VLM."""
    def __init__(self, client, pair_id, search_context=None, probe_points_enu_m=None, probe_covariances_enu_m2=None):
        self.client, self.pair_id = client, pair_id
        self.search_context = copy.deepcopy(search_context)
        self.probes = copy.deepcopy(probe_points_enu_m)
        self.probe_covariances = copy.deepcopy(probe_covariances_enu_m2)

    def query(self, operation, origin_xy, expected_identity, candidates):
        payload = {
            'operation': operation, 'pair_id': self.pair_id, 'origin_xy_enu_m': origin_xy,
            'expected_identity': expected_identity, 'candidates': candidates}
        if self.search_context is not None:
            payload['search_context'] = self.search_context
        if self.probes is not None:
            payload['probe_points_enu_m'] = self.probes
        if self.probe_covariances is not None:
            payload['probe_covariances_enu_m2'] = self.probe_covariances
        from search_diagnostics import stage
        with stage('remote_viewpoints', payload) as diagnostic:
            response = self.client.request('/navigation/viewpoints', payload, timeout=30)
            diagnostic['result'] = response
            return response
