"""Read-only task views of the one authoritative SemanticMap observation store."""
import copy
from .association import labels_compatible


class SharedTargetMemory:
    def __init__(self, semantic_map):
        self.map = semantic_map

    def view(self, identifier, confirmation_level='unknown'):
        m = self.map
        if m.status != 'valid':
            raise ValueError('Cannot use target memory after coordinate/map invalidation')
        landmark = m.landmarks[identifier]
        observations = landmark.observations
        last = observations[-1]
        later = [o for o in m.observations if o['stamp_sim_s'] > last['stamp_sim_s']]
        ranked = sorted(observations, key=lambda o: (
            o.get('position_enu_m') is not None,
            min(o['bbox'][0], o['bbox'][1], 1-o['bbox'][2], 1-o['bbox'][3]) > .01,
            (o['bbox'][2]-o['bbox'][0])*(o['bbox'][3]-o['bbox'][1]), o['stamp_sim_s']), reverse=True)
        keys, hashes = [], set()
        for o in ranked:
            sha = o['image_sha256']['left']
            if sha in hashes:
                continue
            hashes.add(sha)
            keys.append({'capture_id': o['capture_id'], 'image_sha256': sha, 'bbox': o['bbox'],
                         'has_metric_depth': o.get('position_enu_m') is not None})
            if len(keys) == 3:
                break
        return copy.deepcopy({'target_id': identifier, 'label': landmark.label,
            'aliases': landmark.attributes.get('aliases', [landmark.label]),
            'target_revision': len(observations)+landmark.attributes.get('object_frame_revision', 0),
            'observation_refs': [o['capture_id'] for o in observations],
            'key_views': keys, 'last_capture_id': last['capture_id'], 'last_seen_sim_s': last['stamp_sim_s'],
            'missed_observations': len(later), 'visibility': 'recently_visible' if not later else 'lost',
            'bearing_origin_enu_m': last['bearing_origin_enu_m'], 'bearing_direction_enu': last['bearing_direction_enu'],
            'position_enu_m': landmark.position_enu_m, 'covariance_enu_m2': landmark.covariance_enu_m2,
            'coordinate_epoch': m.coordinate_epoch, 'world_sha256': m.world_sha256,
            'sortie_id': m.sortie_id, 'confirmation_level': confirmation_level,
            'orientation': landmark.attributes.get('object_orientation'), 'extent_known': False,
            'refuted_visual_observations': landmark.attributes.get('refuted_visual_observations', []),
            'association_hypotheses': last.get('association'), 'map_status': m.status,
            'schema_version': 1})

    def matching(self, query):
        return [self.view(l.id) for l in self.map.landmarks.values() if labels_compatible(l.label, query)
                and not any(r['capture_id'] == l.observations[-1]['capture_id'] and labels_compatible(r['query'], query)
                            for r in l.attributes.get('refuted_visual_observations', []))]
