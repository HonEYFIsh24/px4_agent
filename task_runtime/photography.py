"""PhotoTask as a child skill, sharing the existing one-time flight preparation.

The preparation session owns epoch/boundary/observations; it is explicitly marked
photo_framing and never starts target NBV or pretends framing is target_found.
"""
from photo_task import PhotoTask, is_overview_request
from .association import semantic_key, ALIASES


class PhotographySkill:
    def __init__(self, search, frames):
        self.search, self.frames, self.client = search, frames, search.client

    def start(self, owner, query, subject, altitude):
        if self.frames is None or owner in self.frames.photo_tasks or owner in self.search.sessions:
            raise ValueError('Photography preparation requires the original parent owner once')
        try:
            self.search.begin(owner, subject, initial_altitude=altitude, original_request=query, execution_mode='orchestrated')
        except Exception as exc:
            if hasattr(self.client, 'record'):
                self.client.record('begin_target_search', {'status': 'error', 'error': str(exc), 'purpose': 'photo_framing_preparation'})
            raise
        child = self.search.sessions[owner]
        child.update(purpose='photo_framing', target_confirmation_mode='single_frame', nbv_auto_prepare=False)
        self.frames.photo_tasks[owner] = PhotoTask(query)
        unknown_area = is_overview_request(query) and semantic_key(subject)[0] not in ALIASES
        return {'last_report_mission_id': None, 'minimum_capture_stamp': None,
                'area_extent_known': False if unknown_area else None}

    def capture(self, owner, state):
        child = self.search.sessions[owner]
        task = self.frames.photo_tasks[owner]
        task.guard_tool('capture_scene', {})
        if task.status != 'awaiting_photo':
            raise ValueError('Photo skill cannot skip report/review/adjustment')
        pending = child.get('pending')
        if pending and pending['action'] == 'takeoff' and not child['arrived']:
            self.search.execute_pending(owner)
        live = self.client.state()
        self.search.validate_state(owner, live)
        mission = live.get('mission') or {}
        if mission.get('status') != 'succeeded' or mission.get('id') == state['last_report_mission_id']:
            raise ValueError('A new photograph needs an actual completed initial/adjustment flight, not another capture number')
        if child['observations']:
            point = live['position_enu_m']
            child.update(pending={'action': 'goto', 'x': point[0], 'y': point[1], 'altitude': point[2]}, arrived=True)
        minimum = state.get('minimum_capture_stamp')
        if minimum is None:
            minimum = child['binding']['stamp_sim_s']
        captured = self.frames.capture(owner, child['query'], minimum_stamp=minimum)
        task.captured(captured['capture_id'])
        state['last_report_mission_id'] = mission['id']
        if hasattr(self.client, 'record'):
            self.client.record('capture_scene', captured)
            self.client.record('photo_progress', task.summary())
        return captured

    def progress(self, owner):
        return self.frames.photo_tasks[owner].summary()

    def plan_adjustment(self, owner, operation, arguments, pose):
        from .goal_planner import GoalPosePlanner
        from search_limits import SEARCH_RADIUS_M
        child = self.search.sessions[owner]
        return GoalPosePlanner().framing_candidate(pose, operation, arguments, child['boundary_origin'][:2],
                                                  SEARCH_RADIUS_M)
