"""Non-authoritative duplicate hypotheses between conflicting landmark tracks.

These links never merge metric evidence, establish object identity, or authorize
confirmation/navigation. A one-way projection can also arise from occlusion or
two different objects; preserve both tracks and expose that uncertainty.
"""
import math


def duplicate_hypotheses(landmarks, contains):
    proposals = []
    for index, a in enumerate(landmarks):
        for b in landmarks[index+1:]:
            if a.label.casefold() != b.label.casefold():
                continue
            ca = {o['capture_id'] for o in a.observations}
            cb = {o['capture_id'] for o in b.observations}
            # Co-occurrence is evidence against treating two detections as one.
            if ca & cb:
                continue
            support, mutual = None, False
            for oa in a.observations:
                for ob in b.observations:
                    old, new = sorted((oa, ob), key=lambda o: o['stamp_sim_s'])
                    x, y = old.get('position_enu_m'), new.get('position_enu_m')
                    if (x is None or y is None or math.dist(x, y) > 6.
                            or new['stamp_sim_s'] <= old['stamp_sim_s']
                            or math.dist(old['body_pose_enu']['position_m'], new['body_pose_enu']['position_m']) < .5):
                        continue
                    forward, backward = contains(x, new), contains(y, old)
                    mutual |= forward and backward
                    if forward != backward and support is None:
                        support = dict(source_capture_id=old['capture_id'], current_capture_id=new['capture_id'],
                            source_surface_in_current_box=forward, current_surface_in_source_box=backward,
                            surface_distance_m=math.dist(x, y))
            # Already-ambiguous reciprocal associations must not be laundered
            # into a unique display group by a weaker pair from another view.
            if support and not mutual:
                proposals.append(dict(member_ids=[a.id, b.id], label=a.label,
                    basis='one_way_projection_with_conflicting_metric_evidence', evidence=support,
                    association_hypothesis_only=True, identity_confirmed=False,
                    metric_evidence_merged=False, usable_for_confirmation=False))
    degree = {}
    for p in proposals:
        for identifier in p['member_ids']:
            degree[identifier] = degree.get(identifier, 0)+1
    for p in proposals:
        p['status'] = ('possible_duplicate' if all(degree[i] == 1 for i in p['member_ids']) else 'ambiguous')
    return proposals


def display_groups(snapshot):
    """Only unambiguous *hypotheses* get a shared visual label; no XYZ fusion."""
    landmarks = {x['id']: x for x in snapshot.get('landmarks', [])}
    groups, used = [], set()
    for h in snapshot.get('landmark_association_hypotheses', []):
        ids = h.get('member_ids', [])
        if (h.get('status') != 'possible_duplicate' or h.get('association_hypothesis_only') is not True
                or h.get('identity_confirmed') is not False or h.get('metric_evidence_merged') is not False
                or h.get('usable_for_confirmation') is not False or len(ids) != 2 or len(set(ids)) != 2
                or any(i not in landmarks or i in used for i in ids)):
            continue
        members = [landmarks[i] for i in ids]
        if (len({m['label'].casefold() for m in members}) != 1
                or any(m.get('position_enu_m') is None for m in members)):
            continue
        groups.append(dict(member_ids=ids, label=members[0]['label'], members=members, hypothesis=h))
        used.update(ids)
    return groups
