"""Serial adapter to the executor's cached, versioned goal-pose tickets."""
import copy
import time
import uuid

from search_tools import coordinate_epoch
from viewpoint_planner import map_identity, map_revision
from .goal_planner import GoalPosePlanner


class GoalExecution:
    def __init__(self, search):
        self.search, self.client = search, search.client

    def prepare(self, owner, stage, plan_revision, generated):
        semantic = self.search.semantic_maps.maps[owner]
        child = self.search.sessions[owner]
        self.search.validate_state(owner, self.client.state(), allow_skill_handoff=True)
        if generated['map_revision'] != map_revision(semantic):
            raise ValueError('Goal map changed before preparation')
        self.search.semantic_maps.checkpoint(owner)
        if self.client.state().get('navigation', {}).get('goal_pose_execution_supported') is not True:
            return {'status': 'unavailable', 'reason_code': 'EXECUTOR_GOAL_POSE_VERSION_UNAVAILABLE', 'candidates': []}
        # Retire every old NBV candidate while preserving the actual scope.
        revision = child.get('authority_stage_revision', 0)+1
        self.client.request('/navigation/stage-transition',
            {'owner': list(owner), 'stage_id': stage.stage_id, 'revision': revision})
        child['authority_stage_revision'] = revision
        context = {'owner': list(owner), 'map_revision': generated['map_revision'], 'batch_id': uuid.uuid4().hex}
        goal_context = {'schema_version': 1, 'stage_id': stage.stage_id, 'stage_revision': revision,
            'plan_revision': plan_revision, 'target_id': stage_target_id(generated),
            'target_revision': generated['target_revision'], 'relation': stage.relation, 'terminal_action': stage.kind}
        payload = {'pair_id': child['last_stereo_pair_id'], 'origin_xy_enu_m': child['boundary_origin'][:2],
                   'expected_identity': map_identity(semantic), 'search_context': context,
                   'goal_context': goal_context, 'probe_points_enu_m': ([generated['target_anchor_enu_m']]
                    if generated['target_anchor_enu_m'] is not None else None)}
        self.client.request('/navigation/goal-poses', {**payload, 'operation': 'context', 'candidates': []})

        def evaluate(candidate, _anchor):
            response = self.client.request('/navigation/goal-poses', {**payload, 'operation': 'paths',
                'candidates': [{k: candidate[k] for k in ('id', 'position_enu_m', 'yaw_enu_rad')}]})
            if response['identity'] != payload['expected_identity']:
                raise ValueError('Goal geometry returned a different coordinate identity')
            result = response['results'][0]
            return {**result, 'batch_id': context['batch_id']}

        result = GoalPosePlanner().evaluate(generated, evaluate)
        return {**result, 'context': context, 'goal_context': goal_context,
                'prepared_at': time.monotonic(), 'execution': None}

    def execute(self, owner, prepared, candidate_id):
        """Record submission BEFORE transport. Unknown side effects are never retried."""
        semantic = self.search.semantic_maps.maps[owner]
        child = self.search.sessions[owner]
        self.search.validate_state(owner, self.client.state(), allow_skill_handoff=True)
        from .memory import SharedTargetMemory
        version = SharedTargetMemory(semantic).view(prepared['goal_context']['target_id'])['target_revision']
        if (prepared.get('execution') is not None or prepared['status'] != 'ready'
                or map_revision(semantic) != prepared['map_revision']
                or version != prepared['target_revision']
                or time.monotonic()-prepared['prepared_at'] > 60.):
            raise ValueError('Goal ticket already submitted, stale or target/map changed')
        candidate = next((c for c in prepared['candidates'] if c['id'] == candidate_id), None)
        if candidate is None or any(v is not True for v in candidate['hard_constraints'].values()):
            raise ValueError('Goal must be a current hard-constraint-verified candidate')
        record = {'status': 'submitting', 'candidate_id': candidate_id, 'started_wall_s': time.time(),
                  'source_capture_id': child['observations'][-1]['capture_id'], 'mission_id': None}
        prepared['execution'] = record
        try:
            accepted = self.client.request('/navigation/execute-goal-pose', {
                'owner': list(owner), 'batch_id': candidate['batch_id'], 'map_revision': prepared['map_revision'],
                'candidate_id': candidate_id, 'ticket': candidate['execution_ticket'],
                'goal_context': prepared['goal_context'], 'description': 'Task goal pose '+candidate_id})
            record.update(status='accepted', mission_id=accepted['mission']['id'])
            # The usual audited waiter records actual completed ownership and
            # retains the independent flight-failure path. No special retry.
            if hasattr(self.client, 'wait_for_photo_action'):
                result = self.client.wait_for_photo_action(accepted)
            else:
                result = self.client.wait_for(accepted)
            if (result.get('mission') or {}).get('status') != 'succeeded':
                raise ValueError('Accepted goal-pose flight did not succeed')
            self.search.validate_completed_flight(owner, result)
            state = self.client.state()
            yaw = state.get('yaw_enu_rad')
            if yaw is None:
                pose = state.get('body_pose_enu') or {}
                from stereo_localization import quaternion_rotation
                r = quaternion_rotation(pose.get('orientation_xyzw'))
                import math
                yaw = math.atan2(r[1][0], r[0][0])
            if not GoalPosePlanner.verify_arrival(candidate, state['position_enu_m'], yaw, coordinate_epoch(state)):
                raise ValueError('Goal flight succeeded without measured position/yaw arrival')
            record.update(status='arrived', result=copy.deepcopy(result))
            return result
        except Exception:
            record['status'] = 'failed_or_unknown_effect'
            raise


def stage_target_id(generated):
    identifiers = {c['goal_spec']['target_id'] for c in generated['candidates']}
    if len(identifiers) != 1:
        raise ValueError('Goal candidates require one authoritative target')
    return next(iter(identifiers))
