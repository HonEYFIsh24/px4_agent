"""Fail-closed, ground-only heading reference investigation; never drives PX4.

This is an evidence collector, not a flight-readiness or calibration authority.
"""
import math

from search_tools import coordinate_epoch


def _number(x):
    return type(x) in (int, float) and math.isfinite(x)


def _yaw(q):
    if not isinstance(q, (list, tuple)) or len(q) != 4 or not all(_number(x) for x in q):
        raise ValueError('invalid quaternion')
    norm = math.sqrt(sum(x*x for x in q))
    if abs(norm-1.) > .02:
        raise ValueError('non-unit quaternion')
    x, y, z, w = (v/norm for v in q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class GroundHeadingProbe:
    """A rejected sample invalidates the whole window; create a new probe to retry."""
    def __init__(self):
        self.identity = None
        self.rows = []
        self.invalid_reason = None

    def add(self, sample, state):
        try:
            return self._add(sample, state)
        except (KeyError, TypeError, ValueError) as exc:
            self.invalid_reason = str(exc)
            self.rows.clear()
            raise ValueError(self.invalid_reason) from None

    def check_state(self, state):
        try:
            return self._check_state(state)
        except (KeyError, TypeError, ValueError) as exc:
            self.invalid_reason = str(exc)
            self.rows.clear()
            raise ValueError(self.invalid_reason) from None

    def _check_state(self, state):
        if self.invalid_reason:
            raise ValueError('probe already invalidated: '+self.invalid_reason)
        if (state.get('simulation_only') is not True or state.get('ready') is not True
                or state.get('connected') is not True or state.get('armed') is not False
                or state.get('landed_state') != 1):
            raise ValueError('requires ready grounded disarmed simulation')
        ev = state.get('external_vision') or {}
        adapter = ev.get('adapter') or {}
        ekf = ev.get('ekf') or {}
        mag = ekf.get('mag_reference') or {}
        align = ekf.get('mag_alignment') or {}
        if (ev.get('healthy') is not True or ev.get('used_for_flight_control') is not True
                or adapter.get('ready') is not True or adapter.get('forwarding') is not True
                or adapter.get('reset_latched') is not False
                or ekf.get('ready') is not True or ekf.get('fusion_active') is not True
                or align.get('yaw_aligned') is not True or align.get('in_air') is not False
                or not (mag.get('heading_fusion') is True or mag.get('three_axis_fusion') is True)
                or mag.get('fault') is not False or mag.get('field_disturbed') is not False):
            raise ValueError('VIO, EKF or magnetic reference is not verifiably healthy')
        epoch = coordinate_epoch(state)
        primary = ekf.get('primary_instance')
        world = (state.get('scene') or {}).get('world_sha256')
        velocity_resets = ekf.get('local_velocity_reset_counters')
        if (not epoch or epoch.get('vio_epoch') is None or epoch.get('ekf_counters') is None
                or type(primary) is not int or not 0 <= primary <= 8
                or not isinstance(world, str) or len(world) != 64
                or any(c not in '0123456789abcdef' for c in world)
                or not isinstance(velocity_resets, dict) or set(velocity_resets) != {'vxy','vz'}
                or any(type(v) is not int or not 0 <= v <= 255 for v in velocity_resets.values())):
            raise ValueError('coordinate or scene identity missing')
        identity = (world, epoch['vio_epoch'], tuple(epoch['ekf_counters'][k] for k in ('xy','z','heading')),
                    tuple(velocity_resets[k] for k in ('vxy','vz')), primary)
        if self.identity is not None and identity != self.identity:
            raise ValueError('coordinate, estimator or scene identity changed')
        return identity

    def _add(self, sample, state):
        identity = self.check_state(state)
        ekf = state['external_vision']['ekf']
        mag = ekf['mag_reference']
        now, vs, ps, es, ms, ls = (sample.get('clock_sim_s'), sample.get('visual_stamp_sim_s'),
            sample.get('pose_stamp_sim_s'), ekf.get('observed_sim_s'), mag.get('source_stamp_sim_s'),
            ekf.get('local_position_source_stamp_sim_s'))
        if (not all(_number(v) and v > 0 for v in (now, vs, ps, es, ms, ls))
                or not 0 <= now-vs <= .25 or not 0 <= vs-ps <= .05
                or not -.05 <= now-es <= 1. or not 0 <= vs-ms <= 1.6
                or not 0 <= vs-ls <= 1.):
            raise ValueError('source time stale, future or not causally aligned')
        if sample.get('visual_frame') != 'vio_local' or sample.get('visual_child_frame') != 'base_link' or sample.get('pose_frame') != 'map':
            raise ValueError('unexpected coordinate frame')
        variance = sample.get('visual_yaw_variance')
        if not _number(variance) or variance <= 0:
            raise ValueError('visual yaw variance missing or invalid')
        if self.rows and (vs <= self.rows[-1]['visual_stamp_sim_s'] or vs-self.rows[-1]['visual_stamp_sim_s'] > .5):
            raise ValueError('visual source sequence repeated, regressed or has a gap')
        visual_yaw, px4_yaw = _yaw(sample['visual_q_xyzw']), _yaw(sample['pose_q_xyzw'])
        offset = (px4_yaw-visual_yaw+math.pi)%(2*math.pi)-math.pi
        self.identity = identity
        self.rows.append({'visual_stamp_sim_s': vs, 'pose_stamp_sim_s': ps, 'offset_rad': offset,
                          'visual_yaw_variance': variance})
        return self.summary()

    def summary(self):
        if self.invalid_reason:
            return {'status':'invalid', 'reason':self.invalid_reason, 'count':0}
        if not self.rows:
            return {'status':'insufficient', 'count':0}
        s = sum(math.sin(r['offset_rad']) for r in self.rows)
        c = sum(math.cos(r['offset_rad']) for r in self.rows)
        mean = math.atan2(s,c)
        spread = max(abs((r['offset_rad']-mean+math.pi)%(2*math.pi)-math.pi) for r in self.rows)
        span = self.rows[-1]['visual_stamp_sim_s']-self.rows[0]['visual_stamp_sim_s']
        sufficient = len(self.rows)>=30 and span>=3. and spread<=.05
        return {'status':'candidate_only' if sufficient else 'insufficient', 'count':len(self.rows),
                'span_sim_s':span, 'offset_rad':mean if sufficient else None, 'maximum_deviation_rad':spread,
                'visual_yaw_variance_max':max(r['visual_yaw_variance'] for r in self.rows),
                'uncertainty_validated':False, 'reference_independent':False,
                'fusion_authorized':False,
                'scope':'read-only ground experiment; not flight authorization or safe calibration'}
