"""Evidence-driven local observation / grounding / bounded global escalation.

Progress advances only on actual new map captures or a newly rejected candidate
batch. Repeated model invocations do not count as new failed viewpoints.
"""
from dataclasses import asdict
import math
from .contracts import ObservationNeed


class AdaptiveReasoning:
    max_global_replans = 2
    local_without_progress = 2

    @staticmethod
    def initial():
        return {'mode': 'LOCAL_OBSERVATION', 'need': None, 'last_evidence': None,
                'last_quality': None, 'stagnant_views': 0, 'global_replans': 0,
                'cooldown_until_observation': 0, 'events': [], 'target_id': None}

    def update(self, state, stage, semantic_map, child):
        from .memory import SharedTargetMemory
        if child.get('search_phase') == 'awaiting_review':
            state['mode'] = 'LOCAL_OBSERVATION'
            return
        if semantic_map is None or not semantic_map.observations:
            state.update(mode='LOCAL_GROUNDING', need={'kind': 'FIND_TARGET', 'target_ref': stage.target_ref})
            return
        if semantic_map.status != 'valid':
            raise ValueError('Reasoning cannot recover an invalid coordinate reference')
        latest = semantic_map.observations[-1]['capture_id']
        active = child.get('confirmation') or {}
        identifier = active.get('target_id') or state.get('target_id')
        memory = SharedTargetMemory(semantic_map)
        if identifier is None:
            visible = [v for v in memory.matching(child['query']) if v['last_capture_id'] == latest]
            if len(visible) == 1:
                identifier = visible[0]['target_id']
        view = memory.view(identifier) if identifier else None
        state['target_id'] = identifier
        missing, kind = [], 'FIND_TARGET'
        if view:
            if view['visibility'] == 'lost':
                kind, missing = 'RECOVER_OCCLUDED', ['fresh_same_instance_observation']
            elif stage.kind in ('approach', 'relative_hover', 'relative_land'):
                kind, missing = 'VISUAL_GOAL_REVIEW', ['current_image_and_model_relation_decision']
            elif (view.get('association_hypotheses') or {}).get('status') == 'ambiguous':
                kind, missing = 'RESOLVE_INSTANCE', ['unique_visual_metric_correspondence']
            elif view['position_enu_m'] is None:
                kind, missing = 'OBTAIN_DEPTH', ['fresh_valid_stereo_depth']
            elif stage.kind == 'close_photo':
                kind, missing = 'IMPROVE_FRAMING_AND_RANGE', ['same_frame_relative_range', 'usable_framing']
            else:
                kind, missing = 'RESOLVE_INSTANCE', ['explicit_multiview_review']
        else:
            missing = ['fresh_visual_candidate']
        state['need'] = asdict(ObservationNeed(kind, identifier, view['last_capture_id'] if view else None,
                                               missing, ['actual_fresh_owned_observation']))
        last = semantic_map.landmarks[identifier].observations[-1] if view else None
        distance = (math.dist(last['position_enu_m'], last['bearing_origin_enu_m'])
                    if last and last.get('position_enu_m') is not None else None)
        box_area = ((last['bbox'][2]-last['bbox'][0])*(last['bbox'][3]-last['bbox'][1]) if last else 0.)
        gap = max(0., distance-5.5, 2.5-distance) if distance is not None else None
        quality = {'target_id': identifier, 'metric': distance is not None, 'orientation': bool(view and view['orientation']),
                   'visible': bool(view and not view['missed_observations']), 'range_gap': gap, 'box_area': box_area}
        evidence = latest
        if evidence != state['last_evidence']:
            best = state['last_quality']
            improved = (best is None or identifier != best['target_id']
                or any(quality[k] and not best[k] for k in ('metric', 'orientation', 'visible'))
                or (gap is not None and best['range_gap'] is not None and best['range_gap']-gap >= .35)
                or box_area > max(.01, (best['box_area'] if best else 0.)*1.2))
            state['stagnant_views'] = 0 if improved else state['stagnant_views']+1
            if improved:
                state['last_quality'] = quality
            state['last_evidence'] = evidence
        count = len(semantic_map.observations)
        needs_global = (state['stagnant_views'] >= self.local_without_progress
                        or child.get('status') == 'no_reachable_view')
        old = state['mode']
        state['mode'] = ('GLOBAL_REASONING' if needs_global and count >= state['cooldown_until_observation']
                         and state['global_replans'] < self.max_global_replans else 'LOCAL_OBSERVATION')
        if child.get('pending') and state['mode'] != 'GLOBAL_REASONING':
            state['mode'] = 'LOCAL_GROUNDING'
        if old != state['mode']:
            state['events'].append({'mode': state['mode'], 'reason_code': kind,
                                    'capture_id': latest, 'observation_count': count})
            state['events'] = state['events'][-24:]

    def apply(self, state, strategy, semantic_map):
        if state['mode'] != 'GLOBAL_REASONING' or state['global_replans'] >= self.max_global_replans:
            raise ValueError('Global replanning is not currently authorized')
        if strategy not in ('reacquire_target', 'alternate_view', 'continue_search'):
            raise ValueError('Unknown bounded replan strategy')
        state['global_replans'] += 1
        state.update(mode='LOCAL_OBSERVATION', stagnant_views=0,
                     cooldown_until_observation=len(semantic_map.observations)+2)
        state['events'].append({'mode': 'LOCAL_OBSERVATION', 'reason_code': 'GLOBAL_PLAN_APPLIED',
                                'strategy': strategy, 'observation_count': len(semantic_map.observations)})
