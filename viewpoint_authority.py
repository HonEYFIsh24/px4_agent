"""Executor-owned, task-bound candidate tickets and immutable flight search volume."""
import copy
import math
import re
import time
import uuid

from mission import MissionError
from stereo_localization import vector


def owner_key(value):
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or any(not isinstance(v, str) or not 1 <= len(v) <= 128 for v in value)):
        raise MissionError('viewpoint owner requires actual thread and task IDs')
    return tuple(value)


class ViewpointAuthority:
    """Called under the controller lock; never trusts client-provided flight coordinates."""
    def __init__(self, clock=time.monotonic, ttl_s=90., capacity=128):
        self.clock, self.ttl_s, self.capacity = clock, ttl_s, capacity
        self.active, self.tickets = None, {}
        self.revoked_owners = set()

    def bind(self, context, pair_id, metadata, identity, origin_xy):
        if not isinstance(context, dict) or set(context) != {'owner', 'map_revision', 'batch_id'}:
            raise MissionError('invalid viewpoint task context')
        owner = owner_key(context['owner'])
        if owner in self.revoked_owners:
            raise MissionError('operator stopped this viewpoint owner; a new task is required')
        if (not re.fullmatch(r'[0-9a-f]{64}', str(context['map_revision']))
                or not re.fullmatch(r'[0-9a-f]{32}', str(context['batch_id']))):
            raise MissionError('invalid map revision or candidate batch ID')
        origin = vector(origin_xy, 2, 'search scope origin')
        stamp = metadata['stamp_sim_s']
        old = self.active
        if old and tuple(old['owner']) == owner:
            if old['identity'] != identity or old['origin_xy_enu_m'] != origin:
                raise MissionError('existing search owner cannot change its identity or search origin')
            if (stamp < old['capture_stamp_sim_s']
                    or (pair_id != old['pair_id'] and stamp <= old['capture_stamp_sim_s'])
                    or (pair_id == old['pair_id'] and context['map_revision'] != old['map_revision'])):
                raise MissionError('search revision is older than its registered observation')
            if all(old[k] == context[k] for k in ('map_revision', 'batch_id')) and old['pair_id'] == pair_id:
                return
        elif math.dist(origin, vector(metadata['body_pose_enu']['position_m'], 3, 'initial observer')[:2]) > .5:
            raise MissionError('new search origin must match its actual initial observation')
        self.tickets.clear()
        self.active = {**copy.deepcopy(context), 'owner': list(owner), 'identity': copy.deepcopy(identity),
                       'origin_xy_enu_m': origin, 'search_radius_m': 12., 'pair_id': pair_id,
                       'capture_stamp_sim_s': stamp, 'revoked': False}

    def require_batch(self, context):
        active = self.active
        if (active is None or active.get('revoked') or owner_key(context.get('owner')) != tuple(active['owner'])
                or any(context.get(k) != active[k] for k in ('map_revision', 'batch_id'))):
            raise MissionError('candidate batch is not the active owner/map revision')
        return active

    def issue(self, context, response):
        active = self.require_batch(context)
        if response['identity'] != active['identity'] or response['origin_xy_enu_m'] != active['origin_xy_enu_m']:
            raise MissionError('candidate path response does not match active search identity')
        now = self.clock()
        self.tickets = {k: v for k, v in self.tickets.items() if now-v['created'] <= self.ttl_s}
        for result in response['results']:
            if result.get('reachable') is not True or not result.get('path'):
                continue
            ticket = uuid.uuid4().hex
            if len(self.tickets) >= self.capacity:
                del self.tickets[next(iter(self.tickets))]
            self.tickets[ticket] = {**copy.deepcopy(active), 'candidate_id': result['id'],
                'goal_enu_m': list(result['position_enu_m']), 'source_enu_m': list(response['start_enu_m']),
                'fixed_yaw_enu_rad': response['fixed_yaw_enu_rad'], 'created': now}
            if 'target_yaw_enu_rad' in result:
                self.tickets[ticket]['target_yaw_enu_rad'] = result['target_yaw_enu_rad']
            result['execution_ticket'] = ticket

    def consume(self, owner, batch_id, revision, candidate_id, ticket, identity, position, yaw):
        if not isinstance(ticket, str) or not re.fullmatch(r'[0-9a-f]{32}', ticket):
            raise MissionError('invalid viewpoint ticket')
        value = self.tickets.get(ticket)
        context = {'owner': owner, 'batch_id': batch_id, 'map_revision': revision}
        self.require_batch(context)
        if (value is None or self.clock()-value['created'] > self.ttl_s
                or owner_key(owner) != tuple(value['owner'])
                or value['batch_id'] != batch_id or value['map_revision'] != revision
                or value['candidate_id'] != candidate_id or value['identity'] != identity
                or math.dist(position, value['source_enu_m']) > .35
                or abs(math.atan2(math.sin(yaw-value['fixed_yaw_enu_rad']),
                                  math.cos(yaw-value['fixed_yaw_enu_rad']))) > 1e-9):
            raise MissionError('viewpoint ticket expired, changed owner/identity/source, or was not verified')
        # One accepted execution invalidates every alternative in that batch.
        self.tickets.clear()
        return copy.deepcopy(value)

    def constraint(self):
        if self.active is None:
            return None
        return {k: copy.deepcopy(self.active[k]) for k in
                ('owner', 'identity', 'origin_xy_enu_m', 'search_radius_m', 'map_revision', 'batch_id', 'revoked')}

    def revoke(self):
        self.tickets.clear()
        if self.active:
            self.revoked_owners.add(tuple(self.active['owner']))
            self.active['revoked'] = True
        if len(self.revoked_owners) > 128:
            self.revoked_owners = {tuple(self.active['owner'])} if self.active else set()

    def clear(self):
        self.active, self.tickets = None, {}
