"""Passive integration of saved VLM reports with task-local semantic maps."""
import json
import os
from pathlib import Path

from semantic_map import SemanticMap
from search_tools import coordinate_epoch


class SemanticMapRuntime:
    def __init__(self, client, capacity=8):
        self.client, self.capacity, self.maps = client, capacity, {}

    def observe(self, owner, frame, report, session=None, instruction='', *, after_update=None):
        state = self.client.state()
        nav, scene = state.get('navigation') or {}, state.get('scene') or {}
        existing = self.maps.get(owner)
        identity = session.get('spatial_map_identity') if session else None
        if existing and (scene.get('world_sha256') != existing.world_sha256
                         or coordinate_epoch(state) != existing.coordinate_epoch
                         or nav.get('registration_id') != existing.sortie_id):
            existing.invalidate('current world, sortie or epoch changed')
            if identity is not None: identity.update(status='invalid', reason=existing.invalid_reason)
            return {'status': 'invalid', 'reason': existing.invalid_reason}
        if session is not None:
            binding = session.get('binding')
            if (not session.get('epoch_bound') or frame.get('owner') != owner
                    or coordinate_epoch(state) != session.get('epoch')
                    or frame['snapshot'].get('coordinate_epoch') != session.get('epoch')
                    or scene.get('world_sha256') != session.get('scene') or (binding and
                    (nav.get('registration_id') != binding['registration_id']
                     or frame['snapshot'].get('navigation_registration_id') != binding['registration_id']
                     or frame['snapshot'].get('stamp_sim_s', -1) <= binding['stamp_sim_s']))):
                raise ValueError('观察与已绑定的搜索坐标或导航地图不一致')
        if nav.get('observation_geometry_supported') is not True or not nav.get('registration_id'):
            return {'status': 'unavailable', 'reason': 'no registered observation-geometry backend'}
        origin = (session or {}).get('boundary_origin') or (session or {}).get('origin') or state.get('home_enu_m') or frame['snapshot']['body_pose_enu']['position_m']
        geometry = self.client.request('/navigation/observation-geometry',
            {'pair_id': frame['snapshot']['stereo_pair_id'], 'origin_xy_enu_m': origin[:2]})
        if existing is None:
            existing = SemanticMap(owner, geometry, query=(session or {}).get('query', frame.get('query', '')),
                                   instruction=instruction)
            if len(self.maps) >= self.capacity:
                del self.maps[next(iter(self.maps))]
            self.maps[owner] = existing
        if session is not None:
            identity = session.setdefault('spatial_map_identity', {})
            identity.update(status=existing.status, world_sha256=existing.world_sha256, sortie_id=existing.sortie_id)
        try:
            context = existing.update_observation(frame, report, geometry)
            if after_update is not None:
                after_update(existing)
        finally:
            if identity is not None: identity.update(status=existing.status, reason=existing.invalid_reason)
        evidence_dir = os.getenv('UAV_CAPTURE_EVIDENCE_DIR')
        if evidence_dir:
            identifier = frame['capture_id']
            if Path(identifier).name != identifier or identifier in ('.', '..'):
                raise ValueError('Invalid capture evidence basename')
            directory = Path(evidence_dir)
            directory.mkdir(parents=True, exist_ok=True)
            for suffix, data in [('geometry', geometry), ('semantic-map', existing.snapshot())]:
                path = directory/(identifier+'.'+suffix+'.json')
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
                temporary.replace(path)
        if session is not None:
            session['last_stereo_pair_id'] = frame['snapshot']['stereo_pair_id']
        return {**context, 'capture_id': frame['capture_id'], 'passive_only': True}
