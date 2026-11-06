"""One parent lifecycle, shared evidence/budget, verified skills and one normal ending."""
import copy
import json
import threading
import time
import uuid
from dataclasses import asdict

from pydantic import ValidationError

from mission import MAX_ALTITUDE
from search_limits import MAX_OBSERVATIONS, SEARCH_RADIUS_M
from task_contract import assess_task
from .contracts import TaskLimits, TaskResult, ObservationNeed, GoalSpec, TaskProtocolError
from .planning import compile_plan, target_query
from .skills import ExistingSkills
from .reasoning import AdaptiveReasoning


class TaskOrchestrator:
    goal_stage_types = frozenset(('approach', 'relative_hover', 'relative_land', 'pass_landmark', 'relative_turn', 'face_target'))
    def __init__(self, client, search_manager, clock=time.monotonic, frame_store=None):
        self.client, self.clock = client, clock
        self.skills = ExistingSkills(search_manager)
        self.reasoning = AdaptiveReasoning()
        from .execution import GoalExecution
        self.goal_execution = GoalExecution(self.skills.search)
        from .landing import LandingSkill
        self.landing_skill = LandingSkill(self.skills.search)
        self.frame_store = frame_store
        from .photography import PhotographySkill
        self.photography = PhotographySkill(self.skills.search, frame_store)
        self.sessions, self.schemas = {}, {}
        self.lock = threading.RLock()

    def ensure(self, owner, instruction):
        with self.lock:
            if owner not in self.sessions:
                if len(self.sessions) >= 64:
                    finished = next((o for o,s in self.sessions.items() if s.get('result')), None)
                    if finished is None:
                        raise ValueError('Task runtime capacity reached')
                    del self.sessions[finished]
                self.sessions[owner] = {'instruction': instruction, 'plan': None, 'index': 0,
                    'revision': 0, 'started': self.clock(), 'model_calls': 0, 'recoveries': 0,
                    'issued_calls': {}, 'targets': {}, 'result': None, 'observations': set(),
                    'reasoning': self.reasoning.initial(),
                    'limits': TaskLimits(MAX_OBSERVATIONS, 900., 60, 1, SEARCH_RADIUS_M, MAX_ALTITUDE)}
            elif self.sessions[owner]['instruction'] != instruction:
                raise ValueError('Parent task cannot change the original instruction')
            return self.sessions[owner]

    def emit(self, owner, event):
        state = self.snapshot(owner)
        if hasattr(self.client, 'record'):
            self.client.record('task_runtime', {'event': event, **state})
        return state

    def stage_contract(self, owner):
        s = self.sessions[owner]
        contract = assess_task(s['instruction'])
        return {**contract, 'admission': 'accepted', 'reason': None,
                'requires_target_search': not self.is_photo_stage(owner), 'target_observation_required': False,
                'policy_source': 'program_parent_stage_contract'}

    def define(self, owner, proposal):
        with self.lock:
            s = self.sessions[owner]
            if s['plan'] is not None or s['result']:
                raise ValueError('Plan can only be established once')
            self.require_live(owner)
            s['plan'] = compile_plan(proposal, owner, s['instruction'], s['limits'])
            strict = any(v.kind in ('close_photo','relative_hover','relative_land','pass_landmark') for v in s['plan'].stages)
            relational = any(v.kind in self.goal_stage_types for v in s['plan'].stages)
            for stage in s['plan'].stages:
                if stage.kind == 'search':
                    stage.evidence_profile = ('visual_candidate' if relational else 'strict_multiview' if strict or
                        self.skills.search.target_confirmation_mode == 'multiview' else 'single_frame_visual')
            self.start_stage(owner)
            return self.emit(owner, 'plan_established')

    def require_live(self, owner):
        s = self.sessions[owner]
        application = getattr(self.client, 'application', None)
        if application:
            with application.lock:
                command = application.current_command or {}
                if (command.get('id') != owner[1] or command.get('interrupted')
                        or command.get('flight_failure') or command.get('vision_failure')
                        or command.get('search_initialization_failure')):
                    raise ValueError('Parent task ownership/cancellation/failure barrier closed')
        child = self.skills.search.sessions.get(owner)
        if child and child.get('epoch_bound'):
            state = self.client.state()
            self.skills.search.validate_state(owner, state, allow_skill_handoff=True)
            if (state.get('mission') or {}).get('status') == 'running':
                raise ValueError('Parent cannot plan/correct while an executor flight is running')

    def stage(self, owner):
        s = self.sessions[owner]
        return s['plan'].stages[s['index']] if s['plan'] and s['index'] < len(s['plan'].stages) else None

    def start_stage(self, owner):
        s, stage = self.sessions[owner], self.stage(owner)
        s['revision'] += 1
        stage.status = 'running'
        if stage.kind == 'search':
            target = next(t for t in s['plan'].targets if t['ref'] == stage.target_ref)
            strict = stage.evidence_profile == 'strict_multiview'
            remaining = s['plan'].stages[s['index']+1:]
            reserve = min(4, 2*sum(v.kind in ({'search', 'close_photo'} | self.goal_stage_types) for v in remaining))
            if owner in self.skills.search.sessions:
                self.skills.search.resume_search_stage(owner, stage.stage_id, s['revision'], target_query(target), stage.evidence_profile)
                self.skills.search.sessions[owner]['observation_ceiling'] = s['limits'].max_observations-reserve
                if self.skills.search.sessions[owner].get('nbv_auto_prepare'):
                    self.skills.search.next_view(owner, 'auto', '开始父任务下一个地标阶段', preparation_source='task_next_target')
            else:
                self.skills.start_search(owner, target, s['instruction'], strict, reserve)
            self.skills.search.sessions[owner]['handoff_evidence_profile'] = stage.evidence_profile
            s['reasoning'] = {**self.reasoning.initial(), 'global_replans': s['reasoning']['global_replans']}
        elif stage.kind == 'close_photo':
            s['reasoning'] = {**self.reasoning.initial(), 'global_replans': s['reasoning']['global_replans']}
            target = s['targets'].get(stage.target_ref)
            if target is None:
                raise ValueError('Photo stage has no handed-off target evidence')
            need = ObservationNeed('IMPROVE_FRAMING_AND_RANGE', target['target_id'], target['capture_id'],
                ['same_frame_relative_range','usable_framing'],
                ['fresh_owned_stereo_measurement','matching_strong_multiview_review'])
            s['observation_need'] = need
            self.skills.resume_photo(owner, stage.stage_id, s['revision'], need)
        elif stage.kind in self.goal_stage_types:
            if stage.target_ref not in s['targets']:
                raise ValueError('Relative goal requires the actual target handoff')
            nav = self.client.state().get('navigation') or {}
            capability = 'target_landing_supported' if stage.kind == 'relative_land' else 'goal_pose_execution_supported'
            if nav.get(capability) is not True or stage.kind == 'relative_land' and stage.relation != 'object_front':
                stage.status, stage.reason_code = 'unavailable', 'EXECUTOR_RELATIVE_GOAL_PROFILE_UNAVAILABLE'
                return
            stage.conditions = {'execution_capability': 'true', 'object_orientation': 'unknown', 'goal_region': 'unknown'}
            stage.reason_code = None
            s['reasoning'] = {**self.reasoning.initial(), 'global_replans': s['reasoning']['global_replans']}
            s['goal_state'] = {'stage_id': stage.stage_id, 'phase': 'need_object_frame' if stage.relation in
                               ('object_front', 'object_behind') else 'plan', 'prepared': None}
        elif stage.kind == 'photo':
            subject = target_query(next((t for t in s['plan'].targets if t['ref'] == stage.target_ref), s['plan'].targets[0]))
            s['photo_state'] = self.photography.start(owner, s['instruction'], subject, stage.altitude_m or 3.)

    def is_photo_stage(self, owner):
        stage = self.stage(owner) if owner in self.sessions else None
        return bool(stage and stage.kind == 'photo' and not self.sessions[owner]['result'])

    def observe_photo(self, owner):
        with self.lock:
            self.require_live(owner)
            if not self.is_photo_stage(owner):
                raise ValueError('No active parent photography stage')
            return self.photography.capture(owner, self.sessions[owner]['photo_state'])

    def report_target_frame(self, owner, capture_id, front_box, rear_box, descriptions,
                            front_left_box=None, front_right_box=None):
        from .object_frame import estimate_object_frame, estimate_front_face_frame
        from target_confirmation import observation_for
        with self.lock:
            self.require_live(owner)
            s, stage = self.sessions[owner], self.stage(owner)
            goal = s.get('goal_state') or {}
            if not stage or goal.get('phase') != 'need_object_frame' or goal.get('stage_id') != stage.stage_id:
                raise ValueError('Object-frame report is not the current missing-evidence stage')
            reference = s['targets'][stage.target_ref]
            if capture_id != reference['capture_id'] or self.frame_store is None:
                raise ValueError('Object-frame report must refer to the handed-off actual image')
            frame = self.frame_store.get(owner, capture_id, require_delivered=True)
            semantic = self.skills.search.semantic_maps.maps[owner]
            landmark, observation = observation_for(semantic, reference['target_id'], capture_id)
            face_profile = front_left_box is not None or front_right_box is not None
            boxes = [front_box, front_left_box, front_right_box] if face_profile else [front_box, rear_box]
            metrics = {'status': 'ok', 'targets': []}
            if any(box is None for box in boxes):
                result, reason = None, 'MISSING_FRONT_FACE_LATERAL_REGIONS' if face_profile else 'MISSING_FRONT_OR_REAR_ROI'
            else:
                metrics = self.client.localize_stereo(frame['snapshot']['stereo_pair_id'], boxes)
                if metrics.get('status') != 'ok':
                    raise ValueError('Object-part stereo response failed its transport contract')
                if face_profile:
                    evidence = estimate_front_face_frame(frame, observation, boxes, metrics['targets'], descriptions[0])
                    result, reason = evidence['orientation'], evidence['reason_code']
                else:
                    result = estimate_object_frame(frame, observation, front_box, rear_box, metrics['targets'], descriptions)
                    reason = 'FRONT_REAR_AXIS_AVAILABLE' if result else 'FRONT_REAR_METRIC_AXIS_UNAVAILABLE'
            self.require_live(owner)  # Do not accept slow localization after cancel/reset.
            diagnostic = {'capture_id': capture_id, 'target_id': reference['target_id'],
                'profile': 'front_face_lateral' if face_profile else 'front_rear_axis', 'reason_code': reason,
                'boxes': copy.deepcopy(boxes), 'measurements': copy.deepcopy(metrics['targets']),
                'descriptions': descriptions, 'orientation_available': result is not None}
            goal.setdefault('object_frame_attempts', []).append({k: v for k, v in diagnostic.items() if k != 'measurements'})
            from search_diagnostics import emit
            emit({'event': 'task_object_frame_evidence', 'owner': list(owner), **diagnostic})
            if result is None:
                goal.update(phase='request_observation', need_kind=(
                    'OBTAIN_DEPTH' if face_profile and reason == 'FRONT_FACE_LATERAL_DEPTH_UNAVAILABLE'
                    else 'REVEAL_FRONT'))
                stage.conditions['object_orientation'] = 'unknown'
            else:
                landmark.attributes['object_orientation'] = result
                landmark.attributes['object_frame_revision'] = landmark.attributes.get('object_frame_revision', 0)+1
                stage.conditions['object_orientation'] = 'true'
                goal['phase'] = 'plan'
            s['revision'] += 1
            s['issued_calls'].clear()
            return self.emit(owner, 'object_frame_available' if result else 'object_frame_observation_required')

    def _review_original_reference(self, owner, reference, current):
        """Resolve a tentative handoff against an existing metric view before closing it.

        Bearing-only reviews can advance the observation hypothesis without
        proving its identity with the parent's original candidate. Reuse the
        two real images for the existing strict review; a shared label is not
        an instance link and no goal execution is authorized here.
        """
        from target_confirmation import observation_for
        s = self.sessions[owner]
        goal = s['goal_state']
        child = self.skills.search.sessions[owner]
        semantic = self.skills.search.semantic_maps.maps[owner]
        if (reference['confirmation_level'] != 'visual_candidate' or goal['phase'] != 'observing'
                or (goal.get('prepared') or {}).get('execution') is not None
                or not child.get('observations') or child.get('activity_closed') or self.frame_store is None):
            return False
        source_landmark, source = observation_for(semantic, reference['target_id'], reference['capture_id'])
        _, observation = current
        pair = [reference['capture_id'], observation['capture_id']]
        if (source_landmark.position_enu_m is not None or source.get('position_enu_m') is not None
                or observation.get('position_enu_m') is None or pair[0] == pair[1]
                or pair in goal.get('reference_review_pairs', [])
                or len(child['observations']) >= child.get('observation_ceiling', MAX_OBSERVATIONS)):
            return False
        # require_live already checked owner, epoch, current mission and faults.
        # Pin only an actually delivered original, never reconstruct pixels.
        self.frame_store.get(owner, pair[0], require_delivered=True)
        self.frame_store.get(owner, pair[1], require_delivered=True)
        self.frame_store.pin_confirmation_source(owner, pair[0])
        goal.setdefault('reference_review_pairs', []).append(pair)
        child.update(status='searching', search_phase='awaiting_review', target_visual_confirmed=False,
            target_focus_state='association_unresolved', confirmation={
                'target_id': reference['target_id'], 'source_capture_id': pair[0],
                'current_capture_id': pair[1], 'current_target_ids': [current[0].id],
                'purpose': 'parent_reference_association', 'review_attempts': 0})
        s['observation_need'] = ObservationNeed('ASSOCIATE_TARGET_REFERENCE', reference['target_id'], pair[0],
            ['identity_with_original_handoff'], ['strict_paired_visual_and_unique_metric_geometry'])
        s['revision'] += 1
        s['issued_calls'].clear()
        self.skills.search.confirmation_event(owner, 'review_required', reason='verify_original_parent_reference')
        self.emit(owner, 'original_reference_review_required')
        return True

    def _replace_refuted_relation_reference(self, owner, stage, reference, outcome):
        """A refuted query match is discarded, never linked to a new instance.

        Earlier multiview confidence cannot make a later explicit negative
        immutable. This is allowed only before any terminal flight, and the new
        object must independently satisfy the existing strict multiview handoff.
        """
        self.require_live(owner)
        s = self.sessions[owner];child = self.skills.search.sessions[owner];goal = s['goal_state']
        refuted = {'target_id': reference['target_id'], 'capture_id': reference['capture_id'], 'query': child['query']}
        if (refuted not in child.get('refuted_search_observations', [])
                or reference['query'] != child['query'] or outcome['query'] != child['query']
                or outcome['confirmation_level'] != 'multiview' or outcome['status'] != 'evidence_ready'
                or outcome['coordinate_epoch'] != child['epoch'] or outcome['stage_id'] != stage.stage_id
                or outcome['capture_id'] != child['observations'][-1]['capture_id']
                or (goal.get('prepared') or {}).get('execution') is not None):
            raise ValueError('Relation verification cannot substitute another same-class instance')
        replacement = {'reason_code': 'REFUTED_QUERY_REFERENCE_REPLACED',
            'previous': copy.deepcopy(reference), 'replacement': copy.deepcopy(outcome),
            'refutation': copy.deepcopy(refuted), 'same_object_claimed': False}
        stage.evidence_refs.append(replacement)
        s['reasoning']['target_id'] = outcome['target_id']
        s['targets'][stage.target_ref] = outcome
        goal['prepared'] = None
        goal.pop('rejected_positions', None)
        stage.conditions.update(object_orientation='unknown', goal_region='unknown',
                                fresh_target_verification='unknown', facing_target='unknown')
        self.emit(owner, 'refuted_candidate_replaced_with_reviewed_instance')

    def _advance_relative(self, owner, stage):
        from .goal_planner import GoalPosePlanner
        s = self.sessions[owner]
        child = self.skills.search.sessions[owner]
        goal = s['goal_state']
        reference = s['targets'][stage.target_ref]
        semantic = self.skills.search.semantic_maps.maps[owner]
        if goal['phase'] == 'route_verification':
            capture = goal['verification_capture_id']
            saved = next((o for o in child['observations'] if o['capture_id'] == capture), None)
            if saved is None:
                return False
            selected = next(c for c in goal['prepared']['candidates'] if c['id'] == goal['prepared']['selected_id'])
            state = self.client.state()
            if not GoalPosePlanner.verify_arrival(selected, state['position_enu_m'], state['yaw_enu_rad'], child['epoch']):
                raise ValueError('Route action lacks actual fresh position/heading arrival')
            if stage.kind == 'face_target':
                from target_confirmation import observations_for
                matches = observations_for(semantic, child['query'], capture)
                if (len(matches) != 1 or matches[0][0].id != reference['target_id']
                        or abs((matches[0][1]['bbox'][0]+matches[0][1]['bbox'][2])/2-.5) > .15):
                    # A completed rotation is not proof the target was seen.
                    # Continue the same target's bounded local observation skill.
                    child.update(search_phase='confirming', target_confirmation_mode='multiview', program_route_verification=False,
                        confirmation={'target_id': reference['target_id'], 'source_capture_id': reference['capture_id'], 'purpose': 'recovery'})
                    goal['phase'] = 'observing'
                    self.skills.search.next_view(owner, 'auto', '转向后目标关联或居中仍需新证据', preparation_source='task_route_recovery')
                    return False
                stage.conditions['fresh_target_horizontal_alignment'] = 'true'
            elif stage.kind == 'pass_landmark':
                anchor = goal['prepared']['target_anchor_enu_m'];axis = goal['prepared']['route_axis_xy_enu']
                margin = selected['score_terms']['world_position_margin_m']
                beyond = sum((state['position_enu_m'][i]-anchor[i])*axis[i] for i in (0, 1))-margin
                if beyond < 2.:
                    raise ValueError('Actual path endpoint did not pass the observed landmark reference plane')
                stage.conditions['passed_observed_landmark_plane'] = 'true'
            else:
                stage.conditions['actual_body_relative_heading'] = 'true'
            # A route report may deliberately contain no target (the landmark
            # is behind us). Preserve the real source, rather than relabelling
            # that empty frame as new target evidence for the next skill.
            route_reference = copy.deepcopy(reference)
            if stage.kind == 'face_target':
                route_reference['capture_id'] = capture
                s['targets'][stage.target_ref] = route_reference
            route_reference.update(kind='route_observation', verified_capture_id=capture,
                                   stage_id=stage.stage_id, status='route_evidence_ready')
            child.update(status='route_completed', activity_closed=True, pending=None, confirmation=None,
                         program_route_verification=False, nbv_private=None, nbv_public=None,
                         parent_stage_revision=s['revision'], skill_handoff=route_reference)
            stage.conditions['fresh_post_action_report'] = 'true'
            s.pop('goal_state')
            self.complete_stage(owner, [{'capture_id': capture, 'actual_mission_id': goal['prepared']['execution']['mission_id'],
                                         'goal_spec': selected['goal_spec'], 'position_enu_m': state['position_enu_m'],
                                         'yaw_enu_rad': state['yaw_enu_rad']}])
            return True
        if goal['phase'] in ('observing', 'verification'):
            progress = self.skills.search.summary(owner)
            if progress['search_status'] == 'target_found':
                from target_confirmation import observations_for
                current = observations_for(semantic, child['query'], child['observations'][-1]['capture_id'])
                refuted = {'target_id': reference['target_id'], 'capture_id': reference['capture_id'], 'query': child['query']}
                if (len(current) == 1 and current[0][0].id != reference['target_id']
                        and refuted not in child.get('refuted_search_observations', [])
                        and self._review_original_reference(owner, reference, current[0])):
                    return False
                outcome = self.skills.handoff(owner, stage.stage_id, s['revision'])
                if outcome['target_id'] != reference['target_id']:
                    self._replace_refuted_relation_reference(owner, stage, reference, outcome)
                s['targets'][stage.target_ref] = outcome
                if goal['phase'] == 'verification':
                    state = self.client.state()
                    selected = next(c for c in goal['prepared']['candidates'] if c['id'] == goal['prepared']['selected_id'])
                    if not GoalPosePlanner.verify_arrival(selected, state['position_enu_m'], state['yaw_enu_rad'],
                                                        child['epoch']):
                        raise ValueError('Fresh target review lacks actual goal pose arrival')
                    stage.conditions.update(goal_region='true', fresh_target_verification='true', facing_target='true')
                    goal['verified_outcome'] = outcome
                    if stage.relation == 'object_front':
                        goal['phase'] = 'need_relation_review'
                        return False
                    s.pop('goal_state', None)
                    self.complete_stage(owner, [outcome, {'goal_pose': copy.deepcopy({k: v for k, v in selected.items()
                        if k != 'execution_ticket'}), 'mission_id': goal['prepared']['execution']['mission_id']}])
                    return True
                from .object_frame import FRONT_FACE_BASIS
                orientation = semantic.landmarks[outcome['target_id']].attributes.get('object_orientation')
                refresh_face = (orientation and orientation.get('basis') == FRONT_FACE_BASIS
                                and orientation['source_capture_id'] != outcome['capture_id'])
                goal['phase'] = ('need_object_frame' if stage.relation in ('object_front', 'object_behind')
                    and (not orientation or refresh_face) else 'plan')
                return False
            if (goal['phase'] == 'verification' and progress['search_status'] == 'searching'
                    and child['observations'] and child['observations'][-1]['capture_id'] == goal.get('verification_capture_id')
                    and child.get('search_phase') != 'awaiting_review'):
                # A rejected/insufficient arrival view requests observation.
                # Once another observation flight moves away, later identity
                # confirmation cannot verify arrival at the *old* goal pose.
                # Replan a fresh goal after reacquiring, retaining the submitted
                # mission record and the parent's unchanged budgets.
                goal['phase'] = 'observing'
                self.emit(owner, 'goal_verification_needs_observation_then_replan')
            if progress['search_status'] in ('budget_exhausted', 'coverage_exhausted', 'no_reachable_view'):
                if progress.get('search_phase') == 'awaiting_review':
                    # The last acquired pair may still be reviewed within the
                    # unchanged parent model/time budget. No new view is allowed.
                    return False
                stage.reason_code = 'RELATION_EVIDENCE_EXHAUSTED'
                self.finish(owner, False)
                return False
            self.reasoning.update(s['reasoning'], stage, semantic, child)
            return False
        if goal['phase'] == 'request_observation':
            # A genuine missing object frame causes a new target view, not a
            # fresh parent/search, larger budget or a guessed vehicle heading.
            s['revision'] += 1
            self.skills.search.resume_observation_skill(owner, stage.stage_id, s['revision'], reference['target_id'],
                purpose='reveal_front' if goal.get('need_kind') == 'REVEAL_FRONT' else
                        'recovery' if goal.get('need_kind') == 'RECOVER_OCCLUDED' else
                        'approach' if goal.get('need_kind') == 'OBTAIN_DEPTH' else 'confirmation', require_close=False)
            child['observation_ceiling'] = s['limits'].max_observations-1
            child['target_confirmation_mode'] = 'multiview'
            goal['phase'] = 'observing'
            s['observation_need'] = ObservationNeed(goal.get('need_kind', 'REVEAL_FRONT'), reference['target_id'],
                reference['capture_id'], ['object_relation_geometry'], ['fresh_visual_regions_with_metric_depth'])
            self.skills.search.next_view(owner, 'auto', '补充目标方向或关系证据', preparation_source='task_relation_evidence')
            return False
        if goal['phase'] == 'plan':
            state = self.client.state()
            pose = state.get('body_pose_enu')
            if not pose or not 0 <= pose.get('age_seconds', 100.) <= 1.:
                raise ValueError('Goal planning requires fresh measured base_link ENU pose')
            if stage.kind in ('relative_turn', 'face_target'):
                from .route import heading_goal
                generated = heading_goal(semantic, reference, pose, child['boundary_origin'][:2], s['limits'].radius_m,
                                         stage.kind, stage.direction, stage.angle_deg)
            else:
                spec = GoalSpec(reference['target_id'], stage.relation, stage.kind,
                                reference_frame='route' if stage.kind == 'pass_landmark' else 'object')
                generated = GoalPosePlanner().generate(spec, semantic, reference, pose,
                                                       child['boundary_origin'][:2], s['limits'].radius_m)
            if goal.get('rejected_positions') and generated['status'] == 'candidates':
                import math
                generated['candidates'] = [c for c in generated['candidates'] if all(
                    math.dist(c['position_enu_m'], old) >= 1.5 for old in goal['rejected_positions'])]
                if not generated['candidates']:
                    generated['status'] = 'no_safe_goal'
            if generated['status'] == 'needs_observation':
                goal.update(phase='request_observation', need_kind=generated['need']['kind'])
                return False
            if generated['status'] == 'no_safe_goal':
                stage.reason_code = 'NO_VERIFIABLE_RELATION_GOAL'
                self.finish(owner, False)
                return False
            goal['prepared'] = self.goal_execution.prepare(owner, stage, s['plan'].plan_revision, generated)
            if goal['prepared']['status'] != 'ready':
                stage.reason_code = goal['prepared'].get('reason_code', 'NO_SAFE_GOAL_PATH')
                self.finish(owner, False)
                return False
            goal['phase'] = 'ready'
            self.emit(owner, 'goal_pose_ready')
        if goal['phase'] == 'authorize_landing':
            goal['landing'] = self.landing_skill.prepare(owner, stage, s['plan'].plan_revision, goal, reference)
            if goal['landing']['status'] == 'not_executed':
                if goal['landing'].get('reason_code') != 'FLAT_SUPPORT_UNAVAILABLE' or goal['landing'].get('mission_submitted') is not False:
                    raise ValueError('Unknown landing-plan rejection is not recoverable')
                selected = next(c for c in goal['prepared']['candidates'] if c['id'] == goal['prepared']['selected_id'])
                goal.setdefault('rejected_positions', []).append(selected['position_enu_m'])
                if len(goal['rejected_positions']) >= 2:
                    stage.reason_code = 'NO_VERIFIED_FLAT_FRONT_LANDING_AREA'
                    self.finish(owner, False)
                else:
                    goal['phase'] = 'plan'
                return False
            if goal['landing']['status'] != 'authorized':
                raise ValueError('Landing authority returned an invalid result')
            stage.conditions['landing_support_and_corridor'] = 'true'
            goal['phase'] = 'landing_ready'
        return False

    def review_goal_region(self, owner, capture_id, front_visible, front_box, description,
                           front_left_box=None, front_right_box=None):
        from .landing import verify_front_review
        from target_confirmation import observation_for
        with self.lock:
            self.require_live(owner)
            s, stage = self.sessions[owner], self.stage(owner)
            goal = s.get('goal_state') or {}
            reference = s['targets'][stage.target_ref]
            if goal.get('phase') != 'need_relation_review' or capture_id != reference['capture_id']:
                raise ValueError('Relation review needs the actual fresh post-goal verification image')
            frame = self.frame_store.get(owner, capture_id, require_delivered=True)
            semantic = self.skills.search.semantic_maps.maps[owner]
            landmark, observation = observation_for(semantic, reference['target_id'], capture_id)
            from .object_frame import FRONT_FACE_BASIS
            face_profile = (landmark.attributes.get('object_orientation') or {}).get('basis') == FRONT_FACE_BASIS
            boxes = [front_box, front_left_box, front_right_box] if face_profile else [front_box]
            metrics = (self.client.localize_stereo(frame['snapshot']['stereo_pair_id'], boxes)
                       if front_visible and all(b is not None for b in boxes) else {'status': 'ok', 'targets': []})
            if metrics.get('status') != 'ok':
                raise ValueError('Fresh relation stereo transport failed')
            result = verify_front_review(landmark, observation, frame, front_box if front_visible else None,
                                        metrics['targets'], description,
                                        side_boxes=[front_left_box, front_right_box] if face_profile else None)
            self.require_live(owner)
            s['revision'] += 1
            s['issued_calls'].clear()
            if result is None:
                goal.update(phase='request_observation', need_kind='REVEAL_FRONT')
                return self.emit(owner, 'fresh_front_relation_unverified')
            goal['relation_review'] = result
            observation['object_front_review'] = copy.deepcopy(result)
            landmark.attributes['object_frame_revision'] = landmark.attributes.get('object_frame_revision', 0)+1
            stage.conditions['fresh_object_front_relation'] = 'true'
            if stage.kind == 'relative_hover':
                selected = next(c for c in goal['prepared']['candidates'] if c['id'] == goal['prepared']['selected_id'])
                evidence = [goal['verified_outcome'], result, {'goal_pose': {k: copy.deepcopy(v) for k, v in selected.items()
                            if k != 'execution_ticket'}, 'mission_id': goal['prepared']['execution']['mission_id']}]
                s.pop('goal_state')
                self.complete_stage(owner, evidence)
            else:
                goal['phase'] = 'authorize_landing'
            return self.emit(owner, 'fresh_front_relation_verified')

    def land_goal(self, owner):
        with self.lock:
            self.require_live(owner)
            s, stage = self.sessions[owner], self.stage(owner)
            goal = s.get('goal_state') or {}
            if stage.kind != 'relative_land' or goal.get('phase') != 'landing_ready':
                raise ValueError('No current verified object-front landing authorization')
            s['revision'] += 1
            s['issued_calls'].clear()
            goal['phase'] = 'landing'
            result = self.landing_skill.execute(owner, goal['landing'])
            verified = result['relation_verified']
            stage.status = 'completed' if verified else 'incomplete'
            stage.reason_code = None if verified else 'ACTUAL_LANDING_RELATIVE_POSITION_UNVERIFIED'
            stage.conditions.update(actual_ground_disarmed='true', actual_relative_landing='true' if verified else 'unknown')
            stage.evidence_refs = [result]
            if verified:
                s['index'] += 1
            child = self.skills.search.sessions[owner]
            self.skills.search.close_activity(owner, 'normal_completion',
                'verified_target_relative_landing' if verified else 'landed_but_relative_position_unverified')
            child['termination'].update(flight_status='grounded_disarmed', policy_source='program_target_relative_landing',
                                        purpose='goal_completion' if verified else 'unverified_goal_landing')
            child['finish_result'] = result['actual_state']
            if child.get('nbv_bound') and not child.get('nbv_released'):
                self.client.request('/navigation/search-release', {'owner': list(owner)})
                child['nbv_released'] = True
            done = [v.stage_id for v in s['plan'].stages if v.status == 'completed']
            unmet = [{'stage_id': v.stage_id, 'kind': v.kind, 'relation': v.relation,
                      'reason_code': v.reason_code or 'NOT_EXECUTED'} for v in s['plan'].stages if v.status != 'completed']
            s['result'] = TaskResult('completed', 'satisfied' if verified and not unmet else 'partial', done, unmet,
                {'targets': copy.deepcopy(s['targets']), 'observations': len(child['observations']),
                 'relative_landing': result}, 'grounded_disarmed', 'normal_completion').snapshot()
            goal['phase'] = 'completed' if verified else 'grounded_unverified'
            return self.emit(owner, 'target_relative_landing_finalized')

    def execute_goal(self, owner, candidate_id):
        """Program-selected terminal pose, actual flight, then an actual fresh image."""
        with self.lock:
            self.require_live(owner)
            s, stage = self.sessions[owner], self.stage(owner)
            goal = s.get('goal_state') or {}
            if goal.get('phase') != 'ready' or goal.get('stage_id') != stage.stage_id:
                raise ValueError('No current ready terminal-pose candidate')
            if candidate_id != goal['prepared']['selected_id'] or self.frame_store is None:
                raise ValueError('Only the program-selected current goal may execute')
            reference = s['targets'][stage.target_ref]
            selected = next(c for c in goal['prepared']['candidates'] if c['id'] == candidate_id)
            point = selected['position_enu_m']
            source_capture = reference['capture_id']
            semantic = self.skills.search.semantic_maps.maps[owner]
            # A near/heading goal may be at the most recent observer location.
            # Reuse a real independent key view of this same instance for the
            # post-arrival comparison, not a second picture at the same pose.
            import math
            sources = [o for o in semantic.landmarks[reference['target_id']].observations
                       if math.dist(o['body_pose_enu']['position_m'], point) >= .75
                       and semantic.observations[-1]['stamp_sim_s']-o['stamp_sim_s'] <= 120.]
            if sources:
                source_capture = sources[-1]['capture_id']
            self.frame_store.pin_confirmation_source(owner, source_capture)
            result = self.goal_execution.execute(owner, goal['prepared'], candidate_id)
            state = self.client.state()
            child = self.skills.search.sessions[owner]
            child.update(status='searching', activity_closed=False, arrived=True,
                pending={'action': 'goto', 'x': point[0], 'y': point[1], 'altitude': point[2]},
                nbv_selected={'id': candidate_id}, search_phase='confirming',
                confirmation={'target_id': reference['target_id'], 'source_capture_id': source_capture},
                handoff_evidence_profile='strict_multiview')
            child['observation_ceiling'] = s['limits'].max_observations
            child['task_contract'] = {**child['task_contract'], 'target_observation_required': False}
            route = stage.kind in ('relative_turn', 'face_target', 'pass_landmark')
            goal['phase'] = 'route_verification' if route else 'verification'
            if route:
                child.update(search_phase='exploring', target_confirmation_mode='route_observation',
                             handoff_evidence_profile='route_observation', confirmation=None, program_route_verification=True)
            capture = self.frame_store.capture(owner, child['query'], minimum_stamp=state['stamp_sim_s'])
            goal['verification_capture_id'] = capture['capture_id']
            if hasattr(self.client, 'record'):
                self.client.record('capture_scene', capture)
            self.emit(owner, 'goal_arrived_awaiting_fresh_report')
            return capture

    def available_replan_strategies(self, owner):
        s = self.sessions[owner]
        if s['result'] or s['reasoning']['mode'] != 'GLOBAL_REASONING':
            return []
        identifier = s['reasoning'].get('target_id')
        semantic = self.skills.search.semantic_maps.maps.get(owner)
        child = self.skills.search.sessions.get(owner)
        if identifier and semantic is not None and child:
            from .memory import SharedTargetMemory
            actual = {v['target_id'] for v in SharedTargetMemory(semantic).matching(child['query'])}
            if identifier in actual:
                return ['continue_search', 'reacquire_target', 'alternate_view']
        return ['continue_search']

    def replan(self, owner, remaining_stage_ids, strategy, reason):
        """Reorder only unexecuted stages; permissions/evidence/budgets stay fixed."""
        from .contracts import PlanProposal
        with self.lock:
            self.require_live(owner)
            s = self.sessions[owner]
            if s['result'] or s['reasoning']['mode'] != 'GLOBAL_REASONING':
                raise ValueError('A global replan must be requested by evidence, not by the model')
            if strategy not in self.available_replan_strategies(owner):
                raise ValueError('Global strategy lacks its required actual target reference')
            remaining = s['plan'].stages[s['index']:]
            ids = [v.stage_id for v in remaining]
            if (len(remaining_stage_ids) != len(ids) or set(remaining_stage_ids) != set(ids)
                    or not ids or remaining_stage_ids[0] != ids[0]):
                raise ValueError('Replan must preserve every unmet requirement and the active stage')
            if not isinstance(reason, str) or not 0 < len(reason) <= 1000:
                raise ValueError('Replan needs a bounded semantic reason')
            by_id = {v.stage_id: v for v in remaining}
            reordered = s['plan'].stages[:s['index']]+[by_id[i] for i in remaining_stage_ids]
            candidate = {'targets': s['plan'].targets, 'stages': [
                {'kind': v.kind, 'target_ref': v.target_ref,
                 'relation': v.relation if v.kind not in ('relative_turn', 'face_target', 'pass_landmark') else None,
                 'direction': v.direction, 'angle_deg': v.angle_deg, 'altitude_m': v.altitude_m} for v in reordered]}
            compile_plan(PlanProposal.model_validate(candidate), owner, s['instruction'], s['limits'])
            semantic = self.skills.search.semantic_maps.maps[owner]
            # Decide recovery source from authoritative memory, never model IDs/coordinates.
            identifier = s['reasoning'].get('target_id')
            if strategy != 'continue_search' and identifier is None:
                raise ValueError('Cannot recover a target without an actual observation reference')
            self.reasoning.apply(s['reasoning'], strategy, semantic)
            s['plan'].stages = reordered
            s['plan'].plan_revision += 1
            s['revision'] += 1
            s['issued_calls'].clear()
            self.skills.search.apply_reasoning_strategy(owner, self.stage(owner).stage_id,
                strategy, identifier)
            return self.emit(owner, 'global_replan_applied')

    def program_call(self, owner):
        """Deterministic tool scheduling, explicitly audited as program, not a VLM call."""
        s = self.sessions[owner]
        if s['result'] or not s['plan'] or s.get('awaiting_report_capture_id'):
            return None
        stage = self.stage(owner)
        if stage and stage.kind == 'photo':
            if self.photography.progress(owner)['photo_status'] != 'awaiting_photo':
                return None
            call = {'id': 'program-'+uuid.uuid4().hex, 'name': 'observe_task_photo', 'args': {}}
            s['issued_calls'][call['id']] = (s['revision'], call['name'])
            self.emit(owner, 'program_photo_capture')
            return call
        goal = s.get('goal_state')
        if stage and stage.kind in self.goal_stage_types and goal and goal.get('phase') in ('ready', 'landing_ready'):
            name = 'execute_task_goal' if goal['phase'] == 'ready' else 'land_task_goal'
            call = {'id': 'program-'+uuid.uuid4().hex, 'name': name,
                    'args': {'candidate_id': goal['prepared']['selected_id']} if name == 'execute_task_goal' else {}}
            s['issued_calls'][call['id']] = (s['revision'], name)
            self.emit(owner, 'program_goal_execution')
            return call
        if (not stage or stage.kind not in ({'search', 'close_photo'} | self.goal_stage_types)
                or s['reasoning']['mode'] == 'GLOBAL_REASONING'):
            return None
        child = self.skills.search.sessions[owner]
        progress = self.skills.search.summary(owner)
        if progress['search_status'] != 'searching' or child.get('nbv_selector') != 'deterministic':
            return None
        name = progress['next_tool']
        if name not in ('observe_target_search_view', 'next_target_search_view'):
            return None
        self.require_live(owner)
        call = {'id': 'program-'+uuid.uuid4().hex, 'name': name,
                'args': {} if name == 'observe_target_search_view' else
                {'strategy': 'auto', 'reason': '程序根据当前信息缺口准备下一安全观察'}}
        s['issued_calls'][call['id']] = (s['revision'], name)
        child['offered_search_tools'] = child['accepted_search_tools'] = [name]
        self.emit(owner, 'program_local_grounding')
        return call

    def complete_stage(self, owner, evidence):
        s, stage = self.sessions[owner], self.stage(owner)
        stage.status = 'completed'
        stage.conditions.update(required_evidence='true')
        stage.evidence_refs = copy.deepcopy(evidence)
        s['index'] += 1
        self.emit(owner, 'stage_completed')
        if s['index'] < len(s['plan'].stages):
            self.start_stage(owner)

    def advance(self, owner):
        with self.lock:
            s = self.sessions[owner]
            if s['result'] or s['plan'] is None:
                return self.snapshot(owner)
            self.require_live(owner)
            child = self.skills.search.sessions.get(owner)
            if child:
                s['observations'].update(o['capture_id'] for o in child['observations'])
            if len(s['observations']) > s['limits'].max_observations:
                raise ValueError('Parent observation budget exceeded')
            if (self.clock()-s['started'] >= s['limits'].max_duration_seconds
                    or s['model_calls'] >= s['limits'].max_model_calls):
                if child is None:
                    raise ValueError('Parent duration exhausted before flight preparation')
                child.update(status='budget_exhausted',search_phase='exploring',confirmation=None)
                self.stage(owner).reason_code = 'PARENT_DURATION_EXHAUSTED'
                return self.finish(owner,False)
            # Only already available evidence advances synchronously. The loop
            # never generates images, semantic claims or unverified positions.
            for _ in range(len(s['plan'].stages)+1):
                stage = self.stage(owner)
                if stage is None:
                    return self.finish(owner, True)
                if stage.status == 'unavailable':
                    return self.finish(owner, False)
                if stage.kind in ('search','close_photo'):
                    progress = self.skills.search.summary(owner)
                    if progress['search_status'] in ('target_found', 'candidate_acquired'):
                        if (stage.kind == 'close_photo' and (child.get('target_observation') or {}).get('target_id')
                                != s['targets'][stage.target_ref]['target_id']):
                            raise ValueError('Photo stage cannot silently substitute another target instance')
                        outcome = self.skills.handoff(owner, stage.stage_id, s['revision'])
                        s['targets'][stage.target_ref] = outcome
                        self.complete_stage(owner, [outcome])
                        continue
                    if progress['search_status'] in ('searching', 'no_reachable_view'):
                        self.reasoning.update(s['reasoning'], stage, self.skills.search.semantic_maps.maps.get(owner), child)
                        if s['reasoning']['mode'] == 'GLOBAL_REASONING':
                            return self.emit(owner, 'global_reasoning_required')
                    if progress['search_status'] in ('budget_exhausted','coverage_exhausted','no_reachable_view'):
                        if progress.get('search_phase') == 'awaiting_review':
                            return self.emit(owner, 'last_acquired_pair_review_required')
                        stage.status, stage.reason_code = 'incomplete', 'EVIDENCE_BUDGET_OR_PATH_LIMIT'
                        return self.finish(owner, False)
                    if progress['search_status'] == 'failed':
                        raise ValueError('Child skill failed; parent cannot resume it')
                    return self.emit(owner, 'awaiting_skill_evidence')
                if stage.kind == 'report':
                    report = self.skills.report(owner)
                    if report.status != 'completed':
                        raise ValueError('Report skill did not complete')
                    self.complete_stage(owner, report.evidence_refs)
                    continue
                if stage.kind == 'photo':
                    photo = self.photography.progress(owner)
                    if photo['photo_status'] in ('met', 'limited', 'budget_exhausted'):
                        met = photo['photo_status'] == 'met' and s['photo_state']['area_extent_known'] is not False
                        child['status'] = 'photo_completed' if met else 'photo_incomplete'
                        if met:
                            self.complete_stage(owner, [photo['latest_observation']])
                            continue
                        stage.status, stage.reason_code = 'incomplete', ('AREA_EXTENT_UNKNOWN' if
                            s['photo_state']['area_extent_known'] is False else 'FRAMING_REQUIREMENT_UNMET')
                        stage.conditions['subject_extent_known'] = 'unknown' if s['photo_state']['area_extent_known'] is False else 'false'
                        stage.evidence_refs = [photo.get('latest_observation')]
                        s['index'] += 1
                        self.emit(owner, 'stage_incomplete_continue_report')
                        if self.stage(owner):
                            self.start_stage(owner)
                        continue
                    return self.emit(owner, 'awaiting_photo_evidence')
                if stage.kind in ('return_home_land','hold'):
                    return self.finish(owner, True)
                if stage.kind in self.goal_stage_types:
                    if self._advance_relative(owner, stage):
                        continue
                    if s.get('goal_state', {}).get('phase') in ('plan', 'request_observation', 'authorize_landing'):
                        continue
                    return self.emit(owner, 'relative_goal_progress')
                raise ValueError('No implemented adapter for declared task stage '+stage.kind)
            raise ValueError('Stage transition bound exceeded')

    def finish(self, owner, complete):
        s = self.sessions[owner]
        if s['result']:
            return self.snapshot(owner)
        self.require_live(owner)
        complete = complete and not any(v.status in ('incomplete', 'unavailable') for v in s['plan'].stages)
        s['revision'] += 1
        s['issued_calls'].clear()
        ending = self.stage(owner)
        child = self.skills.search.sessions.get(owner)
        if child is None:
            raise ValueError('Parent cannot claim flight completion without an owned skill')
        if complete and ending and ending.kind == 'hold':
            child['end_behavior'] = 'hold'
        result = self.skills.finish(owner, complete)
        if result is None:
            from search_tools import _navigation_permission
            state = self.client.state()
            _navigation_permission(state)
            if state.get('armed') is not True or state.get('landed_state') != 2:
                raise ValueError('Parent holding goal lacks actual airborne state')
        flight = 'grounded_disarmed' if result and result.get('armed') is False and result.get('landed_state') == 1 else 'holding'
        if ending and ending.kind in ('return_home_land','hold') and (complete or ending.kind == 'return_home_land' and flight == 'grounded_disarmed'):
            ending.status, ending.conditions = 'completed', {'actual_terminal_state': 'true'}
            ending.evidence_refs = [{'mission_id': (result or {}).get('mission',{}).get('id'), 'flight_status': flight}]
            s['index'] += 1
        done = [v.stage_id for v in s['plan'].stages if v.status == 'completed']
        unmet = [{'stage_id':v.stage_id,'kind':v.kind,'target_ref':v.target_ref,
                  'relation':v.relation,'reason_code':v.reason_code or 'NOT_EXECUTED'}
                 for v in s['plan'].stages if v.status != 'completed']
        s['result'] = TaskResult('completed', 'satisfied' if complete and not unmet else 'partial' if done else 'unmet',
            done, unmet, {'targets':copy.deepcopy(s['targets']), 'observations':len(s['observations'])},
            flight, 'normal_completion').snapshot()
        return self.emit(owner, 'task_finalized')

    def before_model_call(self, owner, expected_revision):
        s = self.sessions[owner]
        self.require_live(owner)
        if s['result'] or expected_revision != s['revision'] or s['model_calls'] >= s['limits'].max_model_calls:
            raise ValueError('Parent model budget or terminal barrier closed')
        s['model_calls'] += 1

    def register_response(self, owner, calls, expected_revision):
        s = self.sessions[owner]
        self.require_live(owner)
        if s['result'] or expected_revision != s['revision']:
            raise ValueError('Parent finalized or stage changed while the model was running')
        for call in calls:
            s['issued_calls'][call['id']] = (s['revision'], call['name'])

    def correct_unsubmitted_text(self, owner):
        s = self.sessions[owner]
        self.require_live(owner)
        if s['recoveries'] >= s['limits'].max_recoveries:
            raise TaskProtocolError('Parent protocol correction budget exhausted')
        s['recoveries'] += 1
        self.emit(owner,'unexecuted_text_correction')

    def _schema_correction(self, owner, name, arguments, errors):
        """One shared correction, before executing a tool or consuming a ticket."""
        from search_diagnostics import emit
        s = self.sessions[owner]
        emit({'event': 'task_tool_schema_error', 'owner': list(owner), 'tool': name,
              'revision': s['revision'], 'arguments': arguments, 'validation_errors': errors})
        if s['recoveries'] >= s['limits'].max_recoveries:
            raise TaskProtocolError('Parent schema correction budget exhausted')
        s['recoveries'] += 1
        return {'status': 'not_executed', 'reason_code': 'tool_schema_invalid', 'tool_executed': False,
                'mission_submitted': False, 'errors': errors, 'next_tool': name,
                'task_runtime': self.emit(owner, 'unexecuted_schema_correction')}

    def _region_argument_errors(self, owner, arguments):
        """Frame identity faults stay fatal; ROI formatting may use the shared credit."""
        import math
        from target_confirmation import observation_for
        s = self.sessions[owner];stage = self.stage(owner);reference = s['targets'][stage.target_ref]
        if arguments.get('capture_id') != reference['capture_id']:
            raise TaskProtocolError('Object-region arguments refer to a different owned image')
        semantic = self.skills.search.semantic_maps.maps[owner]
        _, observation = observation_for(semantic, reference['target_id'], reference['capture_id'])
        outer = observation['bbox'];errors = [];valid = {}
        for field in ('front_box', 'rear_box', 'front_left_box', 'front_right_box'):
            box = arguments.get(field)
            if box is None:
                continue
            finite = (isinstance(box, list) and len(box) == 4
                      and all(type(v) in (int, float) and math.isfinite(v) for v in box))
            if not finite or not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
                errors.append({'loc': [field], 'type': 'normalized_roi_invalid',
                               'msg': 'Use normalized [left,top,right,bottom], ordered and within 0..1'})
                continue
            valid[field] = box
            if not (outer[0] <= box[0] < box[2] <= outer[2] and outer[1] <= box[1] < box[3] <= outer[3]):
                errors.append({'loc': [field], 'type': 'roi_outside_reported_target',
                    'msg': 'Redraw inside the saved target bbox; use null if the region is not visible there',
                    'allowed_target_bbox': list(outer)})
        face = valid.get('front_box')
        for field in ('front_left_box', 'front_right_box'):
            box = valid.get(field)
            if box is not None and (face is None or not (
                    face[0] <= box[0] < box[2] <= face[2] and face[1] <= box[1] < box[3] <= face[3])):
                errors.append({'loc': [field], 'type': 'roi_outside_front_face',
                    'msg': 'Lateral regions must be inside the same visible front_box; unknown is null',
                    'allowed_front_bbox': face})
        return errors

    def guard_tool(self, owner, call):
        s = self.sessions[owner]
        self.require_live(owner)
        if s['result'] or s['issued_calls'].pop(call['id'], None) != (s['revision'], call['name']):
            raise TaskProtocolError('Stale, duplicate or unowned parent tool response')
        name = call['name']
        if name not in self.allowed_tools(owner):
            raise TaskProtocolError('Tool is outside the current parent stage')
        if name in ('report_target_frame', 'review_goal_region'):
            from search_diagnostics import emit
            emit({'event': 'task_region_tool_request', 'owner': list(owner), 'tool': name,
                  'revision': s['revision'], 'arguments': call.get('args')})
        schema = self.schemas.get(name)
        try:
            if schema is not None:
                schema.model_validate(call.get('args') or {})
        except ValidationError as exc:
            return self._schema_correction(owner, name, call.get('args'),
                exc.errors(include_input=False, include_url=False, include_context=False))
        if name in ('report_target_frame', 'review_goal_region'):
            errors = self._region_argument_errors(owner, call.get('args') or {})
            if errors:
                return self._schema_correction(owner, name, call.get('args'), errors)
        if name == 'revise_task_plan':
            from search_diagnostics import emit
            available = self.available_replan_strategies(owner)
            emit({'event': 'task_replan_request', 'owner': list(owner), 'revision': s['revision'],
                  'arguments': call.get('args'), 'available_strategies': available})
            if (call.get('args') or {}).get('strategy') not in available:
                if s['recoveries'] >= s['limits'].max_recoveries:
                    raise TaskProtocolError('Parent protocol correction budget exhausted')
                s['recoveries'] += 1
                return {'status': 'not_executed', 'reason_code': 'replan_strategy_unavailable',
                        'tool_executed': False, 'mission_submitted': False, 'available_strategies': available,
                        'next_tool': name, 'task_runtime': self.emit(owner, 'unexecuted_plan_correction')}
        return None

    def abort(self, owner, reason, source, flight_status):
        """Record external cancellation/fault; recovery never completes a goal."""
        s = self.sessions.get(owner)
        if s is None or s['result']:
            return
        s['revision'] += 1
        s['issued_calls'].clear()
        stages = s['plan'].stages if s['plan'] else []
        done = [v.stage_id for v in stages if v.status == 'completed']
        s['result'] = TaskResult('cancelled' if source == 'operator' else 'failed',
            'partial' if done else 'unmet', done,
            [{'stage_id':v.stage_id, 'kind':v.kind, 'reason_code':'EXTERNAL_FAILURE', 'reason':str(reason)[:500]}
             for v in stages if v.status != 'completed'] or [{'reason_code':'NO_COMPLETED_PLAN'}],
            {'targets':copy.deepcopy(s['targets']), 'observations':len(s['observations'])},
            flight_status, source).snapshot()
        self.emit(owner,'task_aborted')

    def allowed_tools(self, owner):
        s = self.sessions[owner]
        if s['result']:
            return set()
        if s['plan'] is None:
            return {'define_task_plan','get_drone_state'}
        if s.get('awaiting_report_capture_id'):
            return {'report_scene','get_drone_state','target_search_progress'}
        if s['reasoning']['mode'] == 'GLOBAL_REASONING':
            return {'revise_task_plan', 'get_drone_state', 'target_search_progress'}
        stage = self.stage(owner)
        if stage and stage.kind == 'photo':
            photo = self.photography.progress(owner)['photo_status']
            return {'get_drone_state', *({'observe_task_photo'} if photo == 'awaiting_photo' else
                {'report_scene'} if photo == 'awaiting_report' else {'review_photo'} if photo == 'awaiting_review' else
                {'adjust_photo_view', 'rotate_photo_view'} if photo == 'adjust' else set())}
        goal = s.get('goal_state')
        if stage and stage.kind in self.goal_stage_types and goal:
            if goal['phase'] == 'need_object_frame':
                return {'report_target_frame', 'get_drone_state'}
            if goal['phase'] == 'ready':
                return {'execute_task_goal', 'get_drone_state'}
            if goal['phase'] == 'need_relation_review':
                return {'review_goal_region', 'get_drone_state'}
            if goal['phase'] == 'landing_ready':
                return {'land_task_goal', 'get_drone_state'}
            if goal['phase'] in ('plan', 'request_observation'):
                return {'get_drone_state'}
        if stage and stage.kind in ('search','close_photo'):
            progress = self.skills.search.summary(owner)
            return {'get_drone_state','target_search_progress',progress['next_tool']}
        if stage and stage.kind in self.goal_stage_types:
            return {'get_drone_state', 'target_search_progress', self.skills.search.summary(owner)['next_tool']}
        return {'get_drone_state'}

    def snapshot(self, owner):
        s = self.sessions[owner]
        stage = self.stage(owner)
        child = self.skills.search.sessions.get(owner)
        need = (asdict(s['observation_need']) if stage and stage.kind == 'close_photo' and s.get('observation_need')
                else {'kind':'FIND_TARGET','target_ref':stage.target_ref} if stage and stage.kind == 'search' else None)
        return {'schema_version':1, 'owner':list(owner), 'original_instruction':s['instruction'],
                'plan':s['plan'].snapshot() if s['plan'] else None,
                'stage_revision':s['revision'],'current_stage':asdict(stage) if stage else None,
                'reasoning_mode':s['reasoning']['mode'], 'observation_need':s['reasoning'].get('need') or need,
                'reasoning_events':copy.deepcopy(s['reasoning']['events']),
                'available_replan_strategies': self.available_replan_strategies(owner),
                'goal_state':self.public_goal(s.get('goal_state')),
                'next_tool': self.next_tool(owner),
                'target_memory':{ref:self.skills.target_view(owner,value) for ref,value in s['targets'].items()},
                'budget': {**asdict(s['limits']),'model_calls_used':s['model_calls'],
                           'recoveries_used':s['recoveries'],'observations_used':len(s['observations']),
                           'global_replans_max':self.reasoning.max_global_replans,
                           'global_replans_used':s['reasoning']['global_replans']},
                'result':copy.deepcopy(s['result'])}

    def model_context(self, owner):
        """Current evidence summary; do not replay cumulative flight/metric audits to the model."""
        value = self.snapshot(owner)
        if value['plan']:
            for stage in value['plan']['stages']:
                stage.pop('evidence_refs', None)
        for target in value['target_memory'].values():
            for key in ('observation_refs', 'association_hypotheses'):
                target.pop(key, None)
            if target.get('orientation'):
                target['orientation'] = {k: target['orientation'].get(k) for k in
                    ('source_capture_id', 'yaw_enu_rad', 'yaw_variance_rad2', 'basis')}
        goal = value.get('goal_state')
        if goal and goal.get('prepared'):
            prepared = goal['prepared']
            goal['prepared'] = {k: prepared.get(k) for k in ('status', 'selected_id', 'target_revision')}
            goal['candidate_summary'] = [{k: c.get(k) for k in ('id', 'position_enu_m', 'yaw_enu_rad', 'score', 'hard_constraints')}
                                         for c in prepared.get('candidates', [])[:3]]
            for key in ('relation_review', 'landing', 'verified_outcome'):
                goal.pop(key, None)
        value['reasoning_events'] = value['reasoning_events'][-2:]
        return value

    def next_tool(self, owner):
        s = self.sessions[owner]
        if s['result']:
            return None
        if s['plan'] is None:
            return 'define_task_plan'
        if s.get('awaiting_report_capture_id'):
            return 'report_scene'
        if s['reasoning']['mode'] == 'GLOBAL_REASONING':
            return 'revise_task_plan'
        if self.is_photo_stage(owner):
            status = self.photography.progress(owner)['photo_status']
            return {'awaiting_photo': 'observe_task_photo', 'awaiting_report': 'report_scene',
                    'awaiting_review': 'review_photo', 'adjust': 'adjust_photo_view_or_rotate_photo_view'}.get(status)
        goal = s.get('goal_state')
        if goal:
            if goal['phase'] == 'need_object_frame':
                return 'report_target_frame'
            if goal['phase'] == 'ready':
                return 'execute_task_goal'
            if goal['phase'] == 'need_relation_review':
                return 'review_goal_region'
            if goal['phase'] == 'landing_ready':
                return 'land_task_goal'
        return self.skills.search.summary(owner)['next_tool'] if owner in self.skills.search.sessions else None

    @staticmethod
    def public_goal(value):
        if value is None:
            return None
        value = copy.deepcopy(value)
        for candidate in (value.get('prepared') or {}).get('candidates', []):
            candidate.pop('execution_ticket', None)
        if value.get('landing'):
            value['landing'].pop('ticket', None)
        return value

    @staticmethod
    def final_text(state):
        result = state['result']
        lead = {'satisfied':'用户任务已完成。','partial':'用户任务部分完成，仍有要求未满足。',
                'unmet':'用户任务未完成。'}[result['goal_status']]
        reports = []
        for stage in (state.get('plan') or {}).get('stages', []):
            if stage['kind'] == 'report' and stage['status'] == 'completed':
                for evidence in stage.get('evidence_refs', []):
                    if evidence.get('basis') == 'actual_saved_visual_report' and evidence.get('summary'):
                        reports.append('观察报告（'+evidence['capture_id']+'）：'+evidence['summary'])
        return lead + ('\n'+'\n'.join(reports) if reports else '')+'\n' + \
            '已完成阶段：'+', '.join(result['completed_stages'])+'；未完成要求：'+ \
            json.dumps(result['unmet_requirements'],ensure_ascii=False)+'；飞行状态：'+result['flight_status']
