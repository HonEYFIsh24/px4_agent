"""Adapters retain the existing search/map/binding/executor as authorities."""
import copy
from dataclasses import asdict
from .contracts import SkillResult, TargetState

from search_limits import MAX_OBSERVATIONS


class ExistingSkills:
    def __init__(self, search_manager):
        self.search = search_manager.adaptive

    def start_search(self, owner, target, original, strict, reserve):
        from .planning import target_query
        try:
            result = self.search.begin(owner, target_query(target), original_request=original, execution_mode='orchestrated')
        except Exception as exc:
            if hasattr(self.search.client,'record'):
                self.search.client.record('begin_target_search', {'status':'error','error':str(exc)})
            raise
        session = self.search.sessions[owner]
        if strict:
            session['target_confirmation_mode'] = 'multiview'
        session['observation_ceiling'] = MAX_OBSERVATIONS-reserve
        if hasattr(self.search.client,'record'):
            self.search.client.record('begin_target_search', self.search.summary(owner))

    def handoff(self, owner, stage_id, revision):
        return self.search.handoff(owner, stage_id, revision)

    def resume_photo(self, owner, stage_id, revision, need):
        prior = self.search.sessions[owner]['skill_handoff']
        if (need.kind != 'IMPROVE_FRAMING_AND_RANGE' or need.target_id != prior['target_id']
                or need.source_capture_id != prior['capture_id']):
            raise ValueError('Photo observation need must refer to the actual handed-off instance')
        self.search.resume_observation_skill(owner, stage_id, revision, need.target_id)
        self.search.sessions[owner]['observation_ceiling'] = MAX_OBSERVATIONS
        if self.search.sessions[owner].get('nbv_auto_prepare'):
            self.search.next_view(owner, 'auto', '父任务交接后准备目标近照视点',
                                  preparation_source='task_runtime_photo_stage')

    def report(self, owner):
        session = self.search.sessions[owner]
        self.search.validate_state(owner, self.search.client.state(), allow_skill_handoff=True)
        if not session['observations']:
            raise ValueError('Report stage requires an actual saved visual report')
        observation = session['observations'][-1]
        return SkillResult('completed', evidence_refs=[{'capture_id': observation['capture_id'],
            'summary': observation['summary'], 'basis':'actual_saved_visual_report', 'model_calls_added':0}])

    def finish(self, owner, complete):
        result = self.search.finish_normal(owner, parent_goal_complete=complete)
        session = self.search.sessions[owner]
        if session.get('nbv_bound') and not session.get('nbv_released'):
            self.search.client.request('/navigation/search-release', {'owner':list(owner)})
            session['nbv_released'] = True
        return result

    def target_view(self, owner, reference):
        """References/summary only; the authoritative observations stay in SemanticMap."""
        semantic = self.search.semantic_maps.maps[owner]
        if semantic.status != 'valid':
            return {'target_id': reference['target_id'], 'map_status': semantic.status,
                    'position_enu_m': None, 'invalid_reason': semantic.invalid_reason}
        from .memory import SharedTargetMemory
        return SharedTargetMemory(semantic).view(reference['target_id'], reference['confirmation_level'])
