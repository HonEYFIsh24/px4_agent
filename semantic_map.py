"""Task-scoped geometric observation coverage and visual semantic evidence.

No target names/poses enter from the static map. Coverage is centre-sampled
geometric opportunity, never a calibrated detection probability or absence proof.
"""
import copy
import math
import re
from dataclasses import dataclass, field, asdict

from coverage_search import (TargetMemory, CoverageSearchError, project_ground_point,
                             validate_observation_capture, _validated_coordinate_epoch)
from stereo_localization import (vector, finite, quaternion_rotation,
                                matvec, iris_left_optical_to_body)


class SemanticMapError(CoverageSearchError):
    pass


def explicit_semantic_relation(instruction, landmark_label, query):
    """Return the existing positive task relation, or None; not a visual finding."""
    start = instruction.find(landmark_label)
    finish = instruction.find(query, start+len(landmark_label)) if query else -1
    link = instruction[start+len(landmark_label):finish] if start >= 0 and finish >= 0 else ''
    prefix = instruction[max(0, start-5):start] if start >= 0 else ''
    match = re.search(r'(附近|旁边|边上|周围|旁)', link)
    return match.group(0) if match and not any(x in prefix+link for x in ('不', '远离', '并非')) else None


@dataclass
class SemanticLandmark:
    id: str
    label: str
    attributes: dict = field(default_factory=dict)
    position_enu_m: list = None
    covariance_enu_m2: list = None
    observations: list = field(default_factory=list)
    status: str = 'unlocalized'
    position_basis: str = 'visible_surface_representative_not_object_center'
    semantic_class_independently_verified: bool = False


def epoch_identity(value):
    result = _validated_coordinate_epoch(value)
    if result is None or result['vio_epoch'] is None or result['ekf_counters'] is None:
        raise SemanticMapError('Semantic map requires complete VIO epoch and EKF reset counters')
    return result


def bbox(value):
    box = vector(value, 4, 'visual bbox')
    if not 0 <= box[0] < box[2] <= 1 or not 0 <= box[1] < box[3] <= 1:
        raise SemanticMapError('Invalid visual bbox')
    return box


def point_in_box(point, observation, padding=.03):
    projected = project_ground_point(observation['camera_info'], observation['body_pose_enu'], point, 20.)
    pixel = projected['pixel_uv']
    box = observation['bbox']
    return bool(pixel and projected['inside_fov_and_range']
        and box[0]-padding <= pixel[0]/640 <= box[2]+padding
        and box[1]-padding <= pixel[1]/480 <= box[3]+padding)


class SemanticMap:
    def __init__(self, owner, geometry, query='', instruction='', relation_scale_m=3.):
        if not isinstance(owner, tuple) or len(owner) != 2 or not all(isinstance(v, str) and v for v in owner):
            raise SemanticMapError('Semantic map needs the actual thread/turn owner')
        self.owner, self.query, self.instruction = owner, query, instruction
        if not isinstance(query, str) or len(query) > 1500 or not isinstance(instruction, str) or len(instruction) > 2000:
            raise SemanticMapError('Invalid actual task text')
        self.world_sha256 = geometry['world_sha256']
        if not re.fullmatch('[0-9a-f]{64}', self.world_sha256 or ''):
            raise SemanticMapError('Invalid world SHA256')
        self.sortie_id = geometry['sortie_id']
        if not isinstance(self.sortie_id, str) or not 0 < len(self.sortie_id) <= 128:
            raise SemanticMapError('Missing sortie registration identity')
        self.coordinate_epoch = epoch_identity(geometry['coordinate_epoch'])
        self.registration = copy.deepcopy(geometry['registration'])
        self.grid = copy.deepcopy(geometry['grid'])
        if (self.grid.get('rows') != 48 or self.grid.get('cols') != 48
                or self.grid.get('resolution_m') != .5 or self.grid.get('radius_m') != 12.):
            raise SemanticMapError('MVP requires a 48x48 ground grid at 0.5m resolution')
        vector(self.grid['origin_xy_enu_m'], 2, 'grid origin')
        finite(self.grid['ground_enu_z_m'], 'ground height')
        self.eligible = self._indices(geometry['eligible_cell_ids'])
        self.cells = [{'coverage': 0., 'last_capture_id': None, 'last_seen_sim_s': None,
                       'observation_count': 0, 'semantic_score': 0., 'candidate_target_ids': []}
                      for _ in range(2304)]
        self.memory = TargetMemory()
        self.landmarks, self.observations, self.capture_ids, self.image_pairs = {}, [], set(), set()
        self.view_positions = {i: [] for i in self.eligible}
        self.status, self.invalid_reason = 'valid', None
        self.relation_scale = finite(relation_scale_m, 'language relation scale')
        if not .5 <= self.relation_scale <= 8:
            raise SemanticMapError('Relation scale must be configured separately from measurement uncertainty')

    @staticmethod
    def _indices(indices):
        if not isinstance(indices, list) or any(type(i) is not int or not 0 <= i < 2304 for i in indices):
            raise SemanticMapError('Invalid visibility cell IDs')
        if len(set(indices)) != len(indices):
            raise SemanticMapError('Duplicate geometry cell IDs')
        return set(indices)

    def invalidate(self, reason):
        self.status, self.invalid_reason = 'invalid', str(reason)

    def _identity(self, geometry):
        if self.status != 'valid':
            raise SemanticMapError('Semantic map is invalid: '+str(self.invalid_reason))
        try:
            epoch = epoch_identity(geometry['coordinate_epoch'])
        except (ValueError, KeyError):
            self.invalidate('geometry coordinate evidence is missing or malformed')
            raise SemanticMapError(self.invalid_reason)
        if (geometry['world_sha256'] != self.world_sha256 or geometry['sortie_id'] != self.sortie_id
                or epoch != self.coordinate_epoch):
            self.invalidate('world, sortie or coordinate epoch changed')
            raise SemanticMapError(self.invalid_reason)
        if (geometry['registration'] != self.registration or geometry['grid'] != self.grid
                or self._indices(geometry['eligible_cell_ids']) != self.eligible):
            self.invalidate('grid or geometric permission changed')
            raise SemanticMapError(self.invalid_reason)

    def centre(self, index):
        row, col = divmod(index, 48)
        x, y = self.grid['origin_xy_enu_m']
        return [x-12+(col+.5)*.5, y-12+(row+.5)*.5, self.grid['ground_enu_z_m']]

    def _associate(self, label, point, evidence):
        candidates = []
        for target in self.memory.targets:
            if target['label'].casefold() != label.casefold(): continue
            landmark = self.landmarks[target['id']]
            if any(o['capture_id'] == evidence['capture_id'] for o in landmark.observations): continue
            if math.dist(target['position_enu_m'], point) > 6.: continue
            if any(point_in_box(point, old) and point_in_box(old['position_enu_m'], evidence)
                   for old in landmark.observations if old.get('position_enu_m') is not None):
                candidates.append(target['id'])
        # Overlapping/ambiguous detections are not forced into the nearest object.
        return candidates[0] if len(candidates) == 1 else None

    def update_observation(self, frame, report, geometry):
        self._identity(geometry)
        snapshot = frame['snapshot']
        if (frame.get('owner') != self.owner or frame.get('delivered') is not True
                or report.get('status') != 'ok' or report.get('capture_id') != frame.get('capture_id')):
            raise SemanticMapError('Map update needs an owned delivered image and successfully saved report')
        capture = {key: snapshot[key] for key in ('stamp_sim_s', 'age_seconds', 'measurement_age_seconds',
                   'body_pose_enu', 'coordinate_epoch', 'image_quality')}
        capture.update(capture_id=frame['capture_id'], image_sha256=snapshot['stereo_image_sha256'])
        checked = validate_observation_capture(capture)
        epoch = epoch_identity(checked['coordinate_epoch'])
        if epoch != self.coordinate_epoch:
            self.invalidate('capture coordinate epoch changed')
            raise SemanticMapError(self.invalid_reason)
        if (geometry['capture_pair_id'] != snapshot.get('stereo_pair_id')
                or abs(geometry['stamp_sim_s']-checked['stamp_sim_s']) > 1e-6
                or geometry['image_sha256'] != checked['hashes']
                or geometry['body_pose_enu'] != checked['pose']
                or geometry['camera_intrinsics'] != snapshot['camera_intrinsics']):
            raise SemanticMapError('Visibility geometry belongs to another actual image')
        pair = tuple(checked['hashes'][s] for s in ('left', 'right'))
        if checked['capture_id'] in self.capture_ids or pair in self.image_pairs:
            raise SemanticMapError('Duplicate capture cannot update coverage or target evidence')
        if self.observations and checked['stamp_sim_s'] <= self.observations[-1]['stamp_sim_s']:
            raise SemanticMapError('Capture simulation time did not advance')
        if len(self.observations) >= 128:
            raise SemanticMapError('Semantic map observation capacity reached')
        visible = self._indices(geometry['visible_cell_ids'])
        if not visible <= self.eligible:
            raise SemanticMapError('Visible cells exceed geometric permission')
        pending_memory, pending_landmarks = copy.deepcopy(self.memory), copy.deepcopy(self.landmarks)
        # Association reads the provisional batch too, while same-frame objects stay separate.
        original_memory, original_landmarks = self.memory, self.landmarks
        self.memory, self.landmarks = pending_memory, pending_landmarks
        try:
            for target in report.get('targets', []):
                label = target['label']
                if not isinstance(label, str) or not 0 < len(label.strip()) <= 80:
                    raise SemanticMapError('Invalid visual target label')
                box = bbox(target['bbox'])
                mounting = iris_left_optical_to_body()
                k = snapshot['camera_intrinsics']['K']
                if (snapshot['camera_intrinsics'].get('width') != 640 or snapshot['camera_intrinsics'].get('height') != 480
                        or len(k) != 9 or not all(math.isfinite(x) for x in k) or k[0] <= 0 or k[4] <= 0):
                    raise SemanticMapError('Unsupported or invalid camera calibration')
                ray = [((box[0]+box[2])/2*640-k[2])/k[0], ((box[1]+box[3])/2*480-k[5])/k[4], 1.]
                direction = matvec(quaternion_rotation(checked['pose']['orientation_xyzw']), matvec(mounting['rotation'], ray))
                norm = math.sqrt(sum(x*x for x in direction));direction = [x/norm for x in direction]
                evidence = {'capture_id': checked['capture_id'], 'stamp_sim_s': checked['stamp_sim_s'],
                    'body_pose_enu': copy.deepcopy(checked['pose']), 'camera_info': copy.deepcopy(snapshot['camera_intrinsics']),
                    'bbox': box, 'bearing_origin_enu_m': list(geometry['camera_origin_enu_m']), 'bearing_direction_enu': direction,
                    'uncalibrated_visual_score': target.get('confidence'), 'position_enu_m': None}
                metric = target.get('stereo_location') or {}
                if metric.get('status') == 'ok':
                    if (metric.get('image_sha256') != checked['hashes']
                            or abs(metric.get('image_stamp_sim_s', -1)-checked['stamp_sim_s']) > 1e-6
                            or epoch_identity(metric.get('coordinate_epoch')) != self.coordinate_epoch):
                        raise SemanticMapError('Stereo measurement belongs to another image or epoch')
                    point = vector(metric['position_enu_m'], 3, 'stereo surface position')
                    association = self._associate(label, point, evidence)
                    accepted = self.memory.observe(label, metric, checked['capture_id'], checked['pose'],
                        association_target_id=association, allow_auto_association=False)
                    if not accepted['accepted']: raise SemanticMapError(accepted['reason'])
                    identifier = accepted['target_id']
                    evidence['position_enu_m'] = point
                    landmark = self.landmarks.setdefault(identifier, SemanticLandmark(identifier, label))
                    landmark.attributes['reported_attributes'] = copy.deepcopy(target.get('attributes') or {})
                    landmark.observations.append(evidence)
                else:
                    if sum(k.startswith('bearing_') for k in self.landmarks) >= 64:
                        raise SemanticMapError('Unlocalized clue capacity reached')
                    identifier = 'bearing_{:03d}'.format(1+sum(k.startswith('bearing_') for k in self.landmarks))
                    self.landmarks[identifier] = SemanticLandmark(identifier, label,
                        attributes={'localization_failure': metric.get('reason', 'no valid stereo location')}, observations=[evidence])
            for fused in self.memory.reports():
                landmark = self.landmarks[fused['id']]
                landmark.position_enu_m = fused['position_enu_m']
                landmark.covariance_enu_m2 = fused['covariance_enu_m2']
                landmark.status = 'confirmed' if fused['distinct_viewpoint_count'] >= 2 else 'candidate'
                landmark.attributes.update(distinct_viewpoint_count=fused['distinct_viewpoint_count'],
                    measurement_count=fused['measurement_count'], uncertainty_statistics_calibrated=False,
                    confirmation_basis='multi-view geometric consistency only; semantic class not independently verified')
        except Exception:
            self.memory, self.landmarks = original_memory, original_landmarks
            raise
        position = checked['position']
        for i in visible:
            cell = self.cells[i]
            cell.update(coverage=1., last_capture_id=checked['capture_id'], last_seen_sim_s=checked['stamp_sim_s'])
            if all(math.dist(position, old) >= .5 for old in self.view_positions[i]):
                self.view_positions[i].append(list(position));cell['observation_count'] += 1
        self.capture_ids.add(checked['capture_id']);self.image_pairs.add(pair)
        self.observations.append({'capture_id': checked['capture_id'], 'stamp_sim_s': checked['stamp_sim_s'],
            'position_enu_m': list(position), 'orientation_xyzw': checked['pose']['orientation_xyzw'],
            'visible_cell_ids': sorted(visible), 'image_sha256': checked['hashes']})
        self._semantic_scores()
        return self.get_context()

    def _semantic_scores(self):
        for cell in self.cells:
            cell['semantic_score'], cell['candidate_target_ids'] = 0., []
        for landmark in self.landmarks.values():
            if landmark.position_enu_m is None: continue
            # Only explicit user language linking this observed landmark to the query activates a field.
            explicit = explicit_semantic_relation(self.instruction, landmark.label, self.query) is not None
            for i in self.eligible:
                point = self.centre(i)
                distance = math.dist(point[:2], landmark.position_enu_m[:2])
                envelope = max(.5, 3*math.sqrt(max(landmark.covariance_enu_m2[i][i] for i in (0,1))))
                if landmark.label == self.query and distance <= envelope:
                    self.cells[i]['candidate_target_ids'].append(landmark.id)
                if explicit:
                    score = math.exp(-distance*distance/(2*self.relation_scale*self.relation_scale))
                    self.cells[i]['semantic_score'] = max(self.cells[i]['semantic_score'], score)

    def coverage_gain(self, visible_cells):
        if self.status != 'valid': raise SemanticMapError('Cannot query invalid map')
        return sum((1-self.cells[i]['coverage'])*.25 for i in self._indices(list(visible_cells)) & self.eligible)

    def semantic_gain(self, visible_cells, query=None):
        if self.status != 'valid': raise SemanticMapError('Cannot query invalid map')
        if query is not None and query != self.query: raise SemanticMapError('Semantic field is scoped to its actual task query')
        return sum((1-self.cells[i]['coverage'])*self.cells[i]['semantic_score']*.25
                   for i in self._indices(list(visible_cells)) & self.eligible)

    def insufficiently_observed_cells(self):
        if self.status != 'valid': raise SemanticMapError('Cannot query invalid map')
        return [i for i in sorted(self.eligible) if self.cells[i]['coverage'] < 1.]

    def semantic_regions(self, threshold=.2, limit=64):
        if self.status != 'valid': raise SemanticMapError('Cannot query invalid map')
        if not 0 <= threshold <= 1 or type(limit) is not int or not 1 <= limit <= 2304:
            raise SemanticMapError('Invalid semantic region query')
        cells = [i for i in self.eligible if self.cells[i]['semantic_score'] >= threshold and self.cells[i]['coverage'] < 1.]
        cells.sort(key=lambda i: (-self.cells[i]['semantic_score'], i))
        return [{'cell_id': i, 'centre_enu_m': self.centre(i), 'semantic_score': self.cells[i]['semantic_score']}
                for i in cells[:limit]]

    def get_context(self):
        observed = sum(self.cells[i]['coverage'] > 0 for i in self.eligible)
        return {'status': self.status, 'invalid_reason': self.invalid_reason,
            'world_sha256': self.world_sha256, 'sortie_id': self.sortie_id,
            'coordinate_epoch': copy.deepcopy(self.coordinate_epoch), 'observation_count': len(self.observations),
            'eligible_cells': len(self.eligible), 'geometrically_observed_cells': observed,
            'geometric_coverage_fraction': observed/len(self.eligible) if self.eligible else 0.,
            'localized_landmarks': len(self.memory.targets), 'unlocalized_bearing_clues': sum(k.startswith('bearing_') for k in self.landmarks),
            'semantic_field_active': any(self.cells[i]['semantic_score'] > 0 for i in self.eligible),
            'coverage_is_recognition_probability': False, 'strategy_changed': False}

    def snapshot(self):
        return {**self.get_context(), 'owner': list(self.owner), 'query': self.query, 'grid': copy.deepcopy(self.grid),
            'registration': copy.deepcopy(self.registration),
            'eligible_cell_ids': sorted(self.eligible), 'cells': copy.deepcopy(self.cells),
            'landmarks': [asdict(x) for x in self.landmarks.values()], 'observations': copy.deepcopy(self.observations),
            'relation_scale_m': self.relation_scale}
