"""Deterministic fixed-heading NBV baseline; predictions never become observations."""
from mission import MAX_ALTITUDE
import copy
import hashlib
import json
import math
import re
from dataclasses import dataclass, asdict

from stereo_localization import vector, finite, quaternion_rotation


class ViewpointError(ValueError):
    pass


@dataclass(frozen=True)
class ViewpointConfig:
    mode: str = 'semantic'
    candidate_pool_mode: str = 'ranked'
    heading_mode: str = 'fixed'
    coverage_weight: float = 1.
    semantic_weight: float = 1.5
    motion_weight: float = .25
    repeat_weight: float = .2
    target_visibility_weight: float = 1.
    parallax_weight: float = .5
    coverage_scale_m2: float = 100.
    semantic_scale_m2: float = 25.
    motion_scale_s: float = 10.
    nominal_speed_m_s: float = .65
    repeat_sigma_m: float = 1.5
    max_path_evaluations: int = 12
    path_batch_size: int = 4
    source_motion_tolerance_m: float = .35

    def __post_init__(self):
        if self.mode not in ('geometry', 'semantic'):
            raise ViewpointError('unknown NBV baseline mode')
        if self.candidate_pool_mode not in ('ranked', 'diverse'):
            raise ViewpointError('unknown NBV candidate pool mode')
        if self.heading_mode not in ('fixed', 'discrete'):
            raise ViewpointError('unknown NBV heading mode')
        for key, value in asdict(self).items():
            if key in ('mode', 'candidate_pool_mode', 'heading_mode'): continue
            if key in ('max_path_evaluations', 'path_batch_size'):
                if type(value) is not int or not 1 <= value <= (64 if key == 'max_path_evaluations' else 8):
                    raise ViewpointError('invalid NBV path budget')
            elif finite(value, key) < 0 or (key.endswith(('_m2', '_s', '_m')) and value == 0):
                raise ViewpointError('invalid NBV normalization, weight or tolerance')
        if self.coverage_weight == 0 or self.source_motion_tolerance_m > .5:
            raise ViewpointError('NBV requires coverage gain and bounded source-motion tolerance')


def map_revision(semantic_map):
    if semantic_map.status != 'valid':
        raise ViewpointError('cannot plan or select with an invalid semantic map')
    # Includes evidence and semantic state, not merely the number of saved photos.
    snapshot = semantic_map.snapshot()
    return hashlib.sha256(json.dumps(snapshot, ensure_ascii=False, sort_keys=True,
                                     allow_nan=False).encode()).hexdigest()


def map_identity(semantic_map):
    return {key: copy.deepcopy(getattr(semantic_map, key)) for key in
            ('world_sha256', 'sortie_id', 'coordinate_epoch', 'registration')}


POOL_GROUP_ORDER = tuple(altitude+':'+direction for altitude, direction in (
    ('rise', 'vertical'), ('rise', 'front'), ('level', 'front'),
    ('level', 'left'), ('level', 'right'), ('rise', 'left'), ('rise', 'right'),
    ('descend', 'front'), ('descend', 'left'), ('descend', 'right'),
    ('rise', 'back'), ('level', 'back'), ('descend', 'back'), ('descend', 'vertical')))


def viewpoint_group(candidate, reference):
    """Relative heading/height group; no scene semantics or target truth."""
    delta = [a-b for a, b in zip(candidate['position_enu_m'], reference['start_enu_m'])]
    altitude = 'rise' if delta[2] > 1e-8 else 'descend' if delta[2] < -1e-8 else 'level'
    horizontal = math.hypot(*delta[:2])
    yaw = reference['fixed_yaw_enu_rad']
    forward = delta[0]*math.cos(yaw)+delta[1]*math.sin(yaw)
    left = -delta[0]*math.sin(yaw)+delta[1]*math.cos(yaw)
    direction = ('vertical' if horizontal < .25 else
                 'front' if forward/horizontal > .38 else
                 'back' if forward/horizontal < -.38 else 'left' if left > 0 else 'right')
    return altitude+':'+direction


class ViewpointPlanner:
    def __init__(self, config=None):
        self.config = config or ViewpointConfig()
        self._result = None
        self.confirmation_goal = None

    def candidate_pool(self, ranked, reference, fill=False):
        if self.config.candidate_pool_mode == 'ranked' or not ranked:
            return [p['id'] for p in ranked[:8]]
        # Retain the numerical winner, then the best verified representative
        # of each group. Both selectors receive this identical pool contract.
        pool = [ranked[0]]
        represented = {viewpoint_group(ranked[0], reference)}
        for group in POOL_GROUP_ORDER:
            candidate = next((p for p in ranked if viewpoint_group(p, reference) == group), None)
            if group not in represented and candidate is not None:
                pool.append(candidate)
                represented.add(group)
            if len(pool) == 8:
                break
        if fill:
            included = {p['id'] for p in pool}
            pool.extend(p for p in ranked if p['id'] not in included)
        return [p['id'] for p in pool[:8]]

    def generate_candidates(self, pose, origin_xy, fixed_yaw=None):
        position = vector(pose['position_m'], 3, 'NBV position')
        origin = vector(origin_xy, 2, 'NBV origin')
        if not 1. <= position[2] <= MAX_ALTITUDE or math.dist(position[:2], origin) > 12.+1e-9:
            raise ViewpointError('NBV source is outside the search volume')
        if fixed_yaw is None:
            rotation = quaternion_rotation(pose['orientation_xyzw'])
            fixed_yaw = math.atan2(rotation[1][0], rotation[0][0])
        yaw = finite(fixed_yaw, 'NBV fixed yaw')
        points, seen = [], set()
        # Keep observation goals inside the hard 1..20 m volume. The 0.5 m
        # buffer exceeds the executor's 0.35 m rotation tracking allowance.
        heights = dict.fromkeys(max(1.5, min(MAX_ALTITUDE-.5, z)) for z in
                                (position[2], position[2]+2, position[2]-2))
        for z in heights:
            offsets = [(distance*math.cos(i*math.pi/4), distance*math.sin(i*math.pi/4))
                       for distance in (2., 4.) for i in range(8)] + [(0., 0.)]
            for dx, dy in offsets:
                point = [position[0]+dx, position[1]+dy, z]
                key = tuple(round(x, 8) for x in point)
                if (key in seen or math.dist(point, position) < .5
                        or math.dist(point[:2], origin) > 12.+1e-9):
                    continue
                seen.add(key)
                if math.hypot(dx, dy) < 1e-8:
                    direction = '原地'
                else:
                    angle = (math.atan2(dy, dx)-yaw) % (2*math.pi)
                    direction = ('前', '左前', '左', '左后', '后', '右后', '右', '右前')[int(angle/(math.pi/4)+.5) % 8]
                direction += '升高' if z > position[2]+1e-8 else '下降' if z < position[2]-1e-8 else '平移'
                points.append({'id': 'view_{:03d}'.format(len(points)+1),
                               'position_enu_m': point, 'fixed_yaw_enu_rad': yaw, 'direction': direction})
        if self.config.heading_mode == 'discrete':
            base = points + ([{'position_enu_m': list(position), 'direction': '原地', 'fixed_yaw_enu_rad': yaw}]
                             if 1.5 <= position[2] <= MAX_ALTITUDE-.5 else [])
            for candidate in base:
                for step in (-1, 1):
                    points.append({**candidate, 'id': 'view_{:03d}'.format(len(points)+1),
                                   'heading_step': step,
                                   'direction': candidate['direction'] + ('向右转45度' if step < 0 else '向左转45度')})
        return points

    def _score(self, coverage, semantic, seconds, repeat, target_visibility=0., parallax=0.):
        c = self.config
        if self.confirmation_goal is not None:
            terms = {'target_visibility': c.target_visibility_weight*target_visibility,
                     'parallax': c.parallax_weight*parallax,
                     'motion': -c.motion_weight*seconds/c.motion_scale_s,
                     'repeat': -c.repeat_weight*repeat}
            return sum(terms.values()), terms
        terms = {'coverage': c.coverage_weight*coverage/c.coverage_scale_m2,
                 'semantic': (c.semantic_weight if c.mode == 'semantic' else 0.)*semantic/c.semantic_scale_m2,
                 'motion': -c.motion_weight*seconds/c.motion_scale_s,
                 'repeat': -c.repeat_weight*repeat}
        return sum(terms.values()), terms

    def evaluate(self, semantic_map, evaluator, confirmation_goal=None):
        self._result = None
        self.confirmation_goal = copy.deepcopy(confirmation_goal)
        if confirmation_goal is not None:
            if (not isinstance(confirmation_goal.get('probe_points_enu_m'), list)
                    or not 1 <= len(confirmation_goal['probe_points_enu_m']) <= 16):
                raise ViewpointError('confirmation requires bounded actual-evidence probes')
            vector(confirmation_goal['source_camera_enu_m'], 3, 'confirmation source camera')
            vector(confirmation_goal['source_observer_enu_m'], 3, 'confirmation source observer')
            for point in confirmation_goal['probe_points_enu_m']:
                vector(point, 3, 'confirmation hypothesis')
            avoid = confirmation_goal.get('avoid_observer_positions', [])
            if not isinstance(avoid, list) or len(avoid) > 8:
                raise ViewpointError('confirmation previous observers must be a bounded list')
            for point in avoid:
                vector(point, 3, 'previous confirmation observer')
        revision = map_revision(semantic_map)
        if not semantic_map.observations:
            raise ViewpointError('NBV needs at least one saved actual observation')
        identity, origin = map_identity(semantic_map), list(semantic_map.grid['origin_xy_enu_m'])
        reference = None

        def query(operation, candidates):
            nonlocal reference
            payload = [{**{'id': p['id'], 'position_enu_m': list(p['position_enu_m'])},
                        **({'heading_step': p['heading_step']} if 'heading_step' in p else {})} for p in candidates]
            response = evaluator.query(operation, list(origin), copy.deepcopy(identity), payload)
            if (confirmation_goal is not None
                    and response.get('probe_points_enu_m') != confirmation_goal['probe_points_enu_m']):
                raise ViewpointError('confirmation visibility probes changed during query')
            if map_revision(semantic_map) != revision:
                raise ViewpointError('map evidence changed while evaluating viewpoints')
            if (response.get('source') != 'verified_static_3d_viewpoint_prediction'
                    or response.get('prediction_only') is not True or response.get('identity') != identity
                    or response.get('execution_authorized') is not False
                    or response.get('requires_executor_revalidation') is not True
                    or response.get('origin_xy_enu_m') != origin or response.get('search_radius_m') != 12.):
                raise ViewpointError('candidate prediction identity or search permission changed')
            if (not isinstance(response.get('camera_calibration_sha256'), str)
                    or not re.fullmatch('[0-9a-f]{64}', response['camera_calibration_sha256'])
                    or response.get('source_pose_kind') not in ('archived_capture', 'live_enu_pose')):
                raise ViewpointError('candidate prediction lacks calibration or pose provenance')
            start = vector(response['start_enu_m'], 3, 'prediction source')
            yaw = finite(response['fixed_yaw_enu_rad'], 'prediction yaw')
            if reference is not None:
                if (math.dist(start, reference['start_enu_m']) > self.config.source_motion_tolerance_m
                        or abs(math.atan2(math.sin(yaw-reference['fixed_yaw_enu_rad']),
                                          math.cos(yaw-reference['fixed_yaw_enu_rad']))) > 1e-9
                        or response.get('camera_calibration_sha256') != reference.get('camera_calibration_sha256')):
                    raise ViewpointError('source pose, heading or calibration changed during candidate evaluation')
            results = response.get('results')
            if (not isinstance(results, list) or len(results) != len(payload)
                    or any(not isinstance(r, dict) for r in results)
                    or [r.get('id') for r in results] != [p['id'] for p in payload]
                    or any(r.get('position_enu_m') != p['position_enu_m'] for r, p in zip(results, payload))):
                raise ViewpointError('candidate response does not match the requested IDs/positions')
            for candidate, result in zip(payload, results):
                if 'heading_step' in candidate:
                    target = math.atan2(math.sin(yaw + candidate['heading_step']*math.pi/4),
                                        math.cos(yaw + candidate['heading_step']*math.pi/4))
                    if (result.get('heading_step') != candidate['heading_step']
                            or type(result.get('target_yaw_enu_rad')) not in (int, float)
                            or not math.isfinite(result['target_yaw_enu_rad'])
                            or abs(math.atan2(math.sin(result['target_yaw_enu_rad']-target),
                                              math.cos(result['target_yaw_enu_rad']-target))) > 1e-9):
                        raise ViewpointError('candidate heading changed during prediction')
                elif 'target_yaw_enu_rad' in result or 'heading_step' in result:
                    raise ViewpointError('unexpected candidate heading')
            return response

        reference = query('context', [])
        candidates = self.generate_candidates({'position_m': reference['start_enu_m']}, origin,
                                               reference['fixed_yaw_enu_rad'])
        visibility_results = []
        for start in range(0, len(candidates), 64):
            visibility_results.extend(query('visibility', candidates[start:start+64])['results'])
        useful, rejected = [], []
        for candidate, result in zip(candidates, visibility_results):
            if result.get('reachable') is not True:
                rejected.append({'id': candidate['id'], 'stage': 'endpoint', 'reason': result.get('reason')})
                continue
            visible = result.get('visible_cell_ids')
            if not isinstance(visible, list) or not semantic_map._indices(visible) <= semantic_map.eligible:
                raise ViewpointError('candidate visibility exceeds the map permission')
            coverage = semantic_map.coverage_gain(visible)
            semantic = semantic_map.semantic_gain(visible)
            if coverage <= 0 and confirmation_goal is None:
                rejected.append({'id': candidate['id'], 'stage': 'visibility', 'reason': 'no new geometric coverage'})
                continue
            target_visibility, parallax = 0., 0.
            if confirmation_goal is not None:
                probes = result.get('probe_visibility')
                expected_probes = confirmation_goal['probe_points_enu_m']
                if not isinstance(probes, list) or len(probes) != len(expected_probes):
                    raise ViewpointError('confirmation prediction lacks target visibility evidence')
                box = confirmation_goal['source_bbox']
                pixels = min((box[2]-box[0])*640, (box[3]-box[1])*480)
                for probe, expected in zip(probes, expected_probes):
                    if (probe.get('point_enu_m') != expected
                            or type(probe.get('inside_fov_and_range')) is not bool
                            or type(probe.get('unoccluded')) is not bool):
                        raise ViewpointError('confirmation visibility does not match evidence probes')
                    camera = vector(probe['camera_origin_enu_m'], 3, 'predicted camera')
                    old_ray = [a-b for a, b in zip(expected, confirmation_goal['source_camera_enu_m'])]
                    new_ray = [a-b for a, b in zip(expected, camera)]
                    old_range = math.sqrt(sum(v*v for v in old_ray))
                    new_range = math.sqrt(sum(v*v for v in new_ray))
                    if old_range < .1 or new_range < .5 or not (probe['inside_fov_and_range'] and probe['unoccluded']):
                        continue
                    size_quality = min(1., pixels*old_range/new_range/24.)
                    pixel = vector(probe['pixel_uv'], 2, 'predicted target pixel')
                    # FOV membership alone can preserve an unusable edge sliver.
                    # Approximate the observed box extent at the new range and
                    # favour enough image margin for its visible surface.
                    half_width = (box[2]-box[0])*640*old_range/new_range/2.
                    half_height = (box[3]-box[1])*480*old_range/new_range/2.
                    framing = max(0., min(1., min(pixel[0], 640-pixel[0])/(half_width+64.),
                                           min(pixel[1], 480-pixel[1])/(half_height+48.)))
                    size_quality *= framing
                    cosine = max(-1., min(1., sum(a*b for a, b in zip(old_ray, new_ray))/old_range/new_range))
                    angle = math.acos(cosine)
                    angle_gain = min(1., math.sin(angle)/math.sin(math.radians(20.))) if angle <= math.radians(70.) else 0.
                    target_visibility += size_quality/len(probes)
                    parallax += size_quality*angle_gain/len(probes)
                if (math.dist(candidate['position_enu_m'], confirmation_goal['source_observer_enu_m']) < .5
                        or any(math.dist(candidate['position_enu_m'], old) < .75
                               for old in confirmation_goal.get('avoid_observer_positions', []))
                        or target_visibility <= 0 or parallax < .05):
                    rejected.append({'id': candidate['id'], 'stage': 'confirmation',
                                     'reason': 'insufficient predicted target visibility or distinct-view parallax'})
                    continue
            target_yaw = result.get('target_yaw_enu_rad', reference['fixed_yaw_enu_rad'])
            def repetition(o):
                position_term = math.dist(candidate['position_enu_m'], o['position_enu_m'])**2 / (2*self.config.repeat_sigma_m**2)
                if self.config.heading_mode == 'fixed':
                    return math.exp(-position_term)
                rotation = quaternion_rotation(o['orientation_xyzw'])
                old_yaw = math.atan2(rotation[1][0], rotation[0][0])
                delta = math.atan2(math.sin(target_yaw-old_yaw), math.cos(target_yaw-old_yaw))
                return math.exp(-position_term-delta**2/(2*(math.pi/4)**2))
            repeat = max((repetition(o) for o in semantic_map.observations), default=0.)
            # Lower bound remains valid when live source pose changes within the allowed tolerance.
            lower_length = max(0., math.dist(reference['start_enu_m'], candidate['position_enu_m'])
                               - self.config.source_motion_tolerance_m)
            # Rotation cost is omitted from this upper bound; the estimated cost
            # below is not a guaranteed lower bound on real arrival time.
            upper, _ = self._score(coverage, semantic, lower_length/self.config.nominal_speed_m_s, repeat,
                                   target_visibility, parallax)
            useful.append({**candidate, 'visible_cell_ids': visible, 'coverage_gain_m2': coverage,
                           'semantic_gain_m2': semantic, 'repeat_cost': repeat, 'optimistic_score': upper,
                           'target_visibility_score': target_visibility, 'parallax_score': parallax,
                           **({'target_yaw_enu_rad': target_yaw} if 'heading_step' in candidate else {})})
        useful.sort(key=lambda p: (-p['optimistic_score'], p['id']))
        remaining, evaluated, attempted, pruned = list(useful), [], 0, []
        attempted_ids = set()

        def evaluate_paths(batch):
            nonlocal attempted
            paths = query('paths', batch)
            attempted += len(batch)
            attempted_ids.update(p['id'] for p in batch)
            for candidate, result in zip(batch, paths['results']):
                if result.get('reachable') is not True:
                    rejected.append({'id': candidate['id'], 'stage': 'path', 'reason': result.get('reason')})
                    continue
                plan = result.get('path') or {}
                points = [vector(plan['start_enu_m'], 3, 'path start')]
                waypoints = plan.get('waypoints')
                if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 4096:
                    raise ViewpointError('candidate path is incomplete')
                points += [vector(p, 3, 'path waypoint') for p in waypoints]
                if (plan.get('source') != 'verified_static_collision_map_3d'
                        or plan.get('route_complete') is not True or plan.get('full_route_within_search_volume') is not True
                        or plan.get('search_origin_xy_enu_m') != origin or plan.get('search_radius_m') != 12.
                        or math.dist(points[-1], candidate['position_enu_m']) > 1e-8
                        or math.dist(points[0], paths['start_enu_m']) > 1e-8
                        or any(not 1.-1e-9 <= p[2] <= MAX_ALTITUDE+1e-9 or math.dist(p[:2], origin) > 12.+1e-9 for p in points)):
                    raise ViewpointError('candidate route exceeds its source, goal or full search-volume permission')
                length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
                if not math.isclose(length, finite(plan['path_length_m'], 'path length'), rel_tol=1e-9, abs_tol=1e-9):
                    raise ViewpointError('candidate route length does not match its waypoints')
                seconds = length/self.config.nominal_speed_m_s + ((math.pi/4/.5 + 1.) if 'heading_step' in candidate else 0.)
                score, terms = self._score(candidate['coverage_gain_m2'], candidate['semantic_gain_m2'], seconds,
                                           candidate['repeat_cost'], candidate['target_visibility_score'], candidate['parallax_score'])
                evaluated.append({**candidate, 'path': plan, 'motion_seconds': seconds,
                                  'score': score, 'score_terms': terms, 'path_verified': True,
                                  'execution_ticket': result.get('execution_ticket')})

        while remaining and attempted < self.config.max_path_evaluations:
            best = max((p['score'] for p in evaluated), default=-math.inf)
            if remaining[0]['optimistic_score'] < best-1e-12:
                pruned = [p['id'] for p in remaining];remaining = []
                break
            count = min(self.config.path_batch_size, self.config.max_path_evaluations-attempted)
            batch, remaining = remaining[:count], remaining[count:]
            evaluate_paths(batch)
        evaluated.sort(key=lambda p: (-p['score'], p['id']))
        if remaining and evaluated and remaining[0]['optimistic_score'] < evaluated[0]['score']-1e-12:
            pruned = [p['id'] for p in remaining];remaining = []
        diversity_checked = []
        if self.config.candidate_pool_mode == 'diverse' and evaluated:
            tried_groups = set()
            while attempted < self.config.max_path_evaluations:
                evaluated.sort(key=lambda p: (-p['score'], p['id']))
                if len(self.candidate_pool(evaluated, reference)) >= 8:
                    break
                represented = {viewpoint_group(p, reference) for p in evaluated}
                batch = []
                for group in POOL_GROUP_ORDER:
                    if group in represented or group in tried_groups:
                        continue
                    candidate = next((p for p in useful if p['id'] not in attempted_ids
                                      and viewpoint_group(p, reference) == group), None)
                    tried_groups.add(group)
                    if candidate is not None:
                        batch.append(candidate)
                    if len(batch) >= min(self.config.path_batch_size,
                                         self.config.max_path_evaluations-attempted,
                                         8-len(self.candidate_pool(evaluated, reference))):
                        break
                if not batch:
                    break
                evaluate_paths(batch)
                diversity_checked.extend(p['id'] for p in batch)
            # Extra checks use the original budget and identical path verifier.
            # An explicitly checked candidate is no longer upper-bound-pruned.
            pruned = [identifier for identifier in pruned if identifier not in attempted_ids]
            remaining = [p for p in remaining if p['id'] not in attempted_ids]
            evaluated.sort(key=lambda p: (-p['score'], p['id']))
        pool_ids = self.candidate_pool(evaluated, reference, fill=True)[:8]
        status = ('path_budget_limited' if remaining else 'planned') if evaluated else (
            'path_budget_exhausted' if remaining else 'no_reachable_view' if useful else 'no_useful_view')
        result = {'status': status, 'prediction_only': True, 'execution_authorized': False,
                  'requires_executor_revalidation': True, 'map_revision': revision, 'identity': identity,
                  'config': asdict(self.config), 'reference': reference, 'query': semantic_map.query,
                  'planning_mode': 'confirmation' if confirmation_goal is not None else 'exploration',
                  'confirmation_goal': copy.deepcopy(confirmation_goal),
                  'generated_count': len(candidates), 'useful_count': len(useful),
                  'path_evaluation_count': attempted, 'pruned_by_upper_bound_ids': pruned,
                  'unvalidated_candidate_ids': [p['id'] for p in remaining], 'rejected': rejected,
                  'ranked_candidates': evaluated, 'selected_id': evaluated[0]['id'] if evaluated else None,
                  'candidate_pool_ids': pool_ids, 'diversity_path_check_ids': diversity_checked,
                  'candidate_pool_groups': {p['id']: viewpoint_group(p, reference) for p in evaluated if p['id'] in pool_ids},
                  'useful_groups_without_verified_offer': sorted(
                      {viewpoint_group(p, reference) for p in useful}
                      - {viewpoint_group(p, reference) for p in evaluated if p['id'] in pool_ids}),
                  'limitations': ['heuristic score, not entropy or calibrated detection probability',
                                  'discrete heading; hypothetical level attitude, ground-centre AABB visibility' if self.config.heading_mode == 'discrete' else 'fixed commanded heading; hypothetical level attitude, ground-centre AABB visibility',
                                  'bounded path budget; exact executor revalidation required before any flight']}
        if map_revision(semantic_map) != revision:
            raise ViewpointError('map changed before selection')
        self._result = copy.deepcopy(result)
        return result

    def select(self, semantic_map, candidate_id=None):
        if self._result is None or map_revision(semantic_map) != self._result['map_revision']:
            raise ViewpointError('no current verified candidate batch; evaluate again')
        identifier = self._result['selected_id'] if candidate_id is None else candidate_id
        for candidate in self._result['ranked_candidates']:
            if candidate['id'] == identifier:
                return {**copy.deepcopy(candidate), 'map_revision': self._result['map_revision'],
                        'identity': copy.deepcopy(self._result['identity']), 'execution_authorized': False,
                        'requires_executor_revalidation': True}
        if identifier is None:
            return None
        raise ViewpointError('candidate ID was not path-verified in the current batch')
