"""Flight guard for fresh external-vision health and actual selected EKF data."""
import copy
import math


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _counters(value, keys):
    return (isinstance(value, dict) and set(value) == set(keys)
            and all(type(v) is int and 0 <= v <= 255 for v in value.values()))


class ExternalVisionFlightGuard:
    def __init__(self):
        self.adapter = self.ekf = None
        self.adapter_time = self.ekf_time = -1e9
        self.epoch = self.counters = None
        self.latched = None
        self.velocity_counters = self.primary = self._ekf_adapter_epoch = None
        self.alignment_pending = False
        self.heading_alignment_event = None
        self.heading_alignment_history = []
        self.pending_deadline_monotonic_s = None
        self._alignment_candidate = self._mag_previous = self._mag_transition = None
        self._alignment_used = False

    def observe_adapter(self, value, now, armed=False):
        if self.adapter and armed and value.get('epoch') != self.epoch:
            self.latched = 'VIO coordinate epoch changed during flight'
        self.adapter, self.adapter_time = value, now
        self.epoch = value.get('epoch')

    def _alignment_evidence(self, value):
        """Source times are measured PX4 times, never receipt-time substitutes."""
        if (value.get('source') != 'actual_owned_px4_uorb'
                or any(value.get(k) is not True for k in ('ready', 'fusion_active',
                    'position_fusion', 'velocity_fusion', 'local_position_valid', 'local_velocity_valid'))):
            return None
        source = value.get('local_position_source_stamp_sim_s')
        published = value.get('local_position_publish_stamp_sim_s')
        observed = value.get('observed_sim_s')
        mag = value.get('mag_alignment')
        if not isinstance(mag, dict):
            return None
        sample, flag_publish = mag.get('source_stamp_sim_s'), mag.get('publish_stamp_sim_s')
        if (not all(_number(v) and v > 0 for v in (source, published, observed, sample, flag_publish))
                or published < source or flag_publish < sample
                or not -.05 <= observed-source <= .5
                # PX4 publishes flags at 1 Hz, or immediately on a change.
                or not -.05 <= observed-flag_publish <= 1.5
                or not -.05 <= observed-sample <= 1.6
                or any(type(mag.get(k)) is not bool for k in ('aligned_in_flight', 'yaw_aligned', 'in_air'))
                or type(value.get('heading_good_for_control')) is not bool
                or not _number(value.get('heading_delta_ned_rad'))):
            return None
        return {'source': source, 'mag_source': sample, 'mag_publish': flag_publish,
                'aligned': mag['aligned_in_flight'], 'yaw_aligned': mag['yaw_aligned'],
                'in_air': mag['in_air']}

    def _remember_mag_transition(self, evidence, now):
        if evidence is None:
            return
        previous = self._mag_previous
        if (previous and previous['aligned'] is False and evidence['aligned'] is True
                and evidence['mag_source'] > previous['mag_source']
                and evidence['source'] >= previous['source'] and 0 <= now-previous['received'] <= 2):
            self._mag_transition = dict(evidence, received=now, epoch=self.epoch, primary=self.primary)
            self._mag_transition.update(false_source_stamp_sim_s=previous['mag_source'],
                                        false_observed_source_stamp_sim_s=previous['source'])
        elif evidence['aligned'] is False:
            self._mag_transition = None
        self._mag_previous = dict(evidence, received=now)

    def _accept_alignment(self, evidence, now, allow):
        candidate, transition = self._alignment_candidate, self._mag_transition
        if candidate is None or transition is None:
            return None
        if (not allow or not evidence['aligned'] or not evidence['yaw_aligned'] or not evidence['in_air']
                or not 0 <= now-transition['received'] <= 2
                or abs(transition['mag_source']-candidate['reset_source_stamp_sim_s']) > .5
                or transition['epoch'] != candidate['adapter_epoch']
                or transition['primary'] != candidate['primary_instance']):
            self.latched = 'PX4 heading reset lacks a matching fresh takeoff magnetic alignment'
            return None
        event = dict(candidate, schema_version=1, source='actual_owned_px4_uorb',
                     permitted_context='takeoff_before_horizontal_navigation',
                     mag_alignment_false_source_stamp_sim_s=transition['false_source_stamp_sim_s'],
                     mag_alignment_source_stamp_sim_s=transition['mag_source'],
                     accepted_at_monotonic_s=now,
                     stamp_semantics='first_observed_local_position_sample_after_heading_counter_change')
        self.heading_alignment_event = event
        self.heading_alignment_history.append(copy.deepcopy(event))
        self._alignment_used = True
        self._alignment_candidate = None
        return copy.deepcopy(event)

    def observe_ekf(self, value, now, armed=False, allow_takeoff_mag_alignment=False):
        event = None
        counters = value.get('local_position_reset_counters')
        valid = _counters(counters, ('xy', 'z', 'heading'))
        velocity = value.get('local_velocity_reset_counters')
        velocity_valid = _counters(velocity, ('vxy', 'vz'))
        primary = value.get('primary_instance')
        primary_valid = type(primary) is int and 0 <= primary <= 8
        evidence = self._alignment_evidence(value)
        self._remember_mag_transition(evidence, now)
        if armed and self.primary is not None and (not primary_valid or primary != self.primary):
            self.latched = 'PX4 selected estimator changed during flight'
        if armed and self.velocity_counters is not None and (not velocity_valid or velocity != self.velocity_counters):
            self.latched = 'PX4 local velocity reset during flight'
        if valid:
            if armed and self.counters is not None and counters != self.counters:
                only_heading = all(counters[k] == self.counters[k] for k in ('xy', 'z'))
                adapter_healthy = (self.adapter and self.adapter.get('mode') == 'fuse'
                    and self.adapter.get('ready') is True and self.adapter.get('forwarding') is True
                    and 0 <= now-self.adapter_time <= 1 and type(self.epoch) is int and self.epoch >= 0
                    and self.epoch == self._ekf_adapter_epoch)
                previous_evidence = self._alignment_evidence(self.ekf) if self.ekf else None
                previous_healthy = (previous_evidence is not None and 0 <= now-self.ekf_time <= 2
                    and evidence is not None and 0 < evidence['source']-previous_evidence['source'] <= .5)
                delta = value.get('heading_delta_ned_rad')
                if (not self.latched and allow_takeoff_mag_alignment and only_heading
                        and (counters['heading']-self.counters['heading']) % 256 == 1
                        and not self._alignment_used and not self.alignment_pending
                        and adapter_healthy and previous_healthy and evidence is not None
                        and _number(delta) and abs(delta) <= math.radians(3)
                        and velocity_valid and velocity == self.velocity_counters
                        and primary_valid and primary == self.primary
                        and evidence['yaw_aligned'] and evidence['in_air']
                        and (evidence['aligned'] is False or self._mag_transition is not None)):
                    self._alignment_candidate = {'delta_heading_ned_rad': delta,
                        'reset_source_stamp_sim_s': evidence['source'],
                        'previous_local_source_stamp_sim_s': previous_evidence['source'],
                        'old_heading_reset_counter': self.counters['heading'],
                        'new_heading_reset_counter': counters['heading'], 'adapter_epoch': self.epoch,
                        'primary_instance': primary, 'xy_reset_counter': counters['xy'],
                        'z_reset_counter': counters['z'], 'velocity_reset_counters': copy.deepcopy(velocity)}
                    self.alignment_pending = True
                    self.pending_deadline_monotonic_s = now+10
                else:
                    self.latched = 'PX4 local coordinate frame reset during flight'
            self.counters = copy.deepcopy(counters)
        elif armed and self.counters is not None:
            self.latched = 'PX4 reset counter evidence unavailable during flight'
        if velocity_valid:
            self.velocity_counters = copy.deepcopy(velocity)
        if primary_valid:
            self.primary = primary
        if self.alignment_pending and not self.latched:
            pending = self._alignment_candidate or self.heading_alignment_event
            if (evidence is None or not armed or now >= self.pending_deadline_monotonic_s
                    or (self._alignment_candidate is not None and not allow_takeoff_mag_alignment)):
                self.latched = 'PX4 takeoff heading alignment evidence missing or timed out'
            elif (not self.adapter or self.adapter.get('mode') != 'fuse'
                    or self.adapter.get('ready') is not True or self.adapter.get('forwarding') is not True
                    or not 0 <= now-self.adapter_time <= 1 or self.epoch != pending['adapter_epoch']
                    or value['heading_delta_ned_rad'] != pending['delta_heading_ned_rad']):
                self.latched = 'PX4 takeoff heading alignment adapter or reset evidence changed'
            elif evidence['source'] < pending['reset_source_stamp_sim_s']:
                self.latched = 'PX4 takeoff heading alignment source moved backwards'
            elif self._alignment_candidate is not None:
                event = self._accept_alignment(evidence, now, allow_takeoff_mag_alignment)
            if not self.latched and self.heading_alignment_event and self._alignment_candidate is None:
                if (evidence['source'] >= self.heading_alignment_event['reset_source_stamp_sim_s']
                        and value.get('heading_good_for_control') is True and evidence['aligned']
                        and evidence['yaw_aligned'] and evidence['in_air']):
                    self.alignment_pending = False
                    self.pending_deadline_monotonic_s = None
            if not self.latched and self._alignment_candidate is not None:
                # Counter/flag polling can be asynchronous, but a later flag
                # cannot excuse a reset after the finite .5-s source window.
                if evidence['source']-self._alignment_candidate['reset_source_stamp_sim_s'] > .5:
                    self.latched = 'PX4 heading reset magnetic alignment source window expired'
        if (not armed and not self.alignment_pending and not self.latched and evidence
                and evidence['aligned'] is False and evidence['in_air'] is False):
            self._alignment_used = False
        self.ekf, self.ekf_time = value, now
        self._ekf_adapter_epoch = self.epoch
        return event

    @property
    def required(self):
        return bool(self.adapter and self.adapter.get('mode') == 'fuse')

    def reason(self, now):
        if not self.required:
            return None
        if self.latched:
            return self.latched
        if self.alignment_pending and now >= self.pending_deadline_monotonic_s:
            self.latched = 'PX4 takeoff heading alignment timed out'
            return self.latched
        if now-self.adapter_time > 1:
            return 'VIO adapter status is stale'
        if self.adapter.get('ready') is not True or self.adapter.get('forwarding') is not True:
            return 'VIO adapter stopped: ' + ', '.join(self.adapter.get('reasons') or ['not forwarding'])
        if now-self.ekf_time > 2:
            return 'actual EKF evidence is stale'
        if not self.ekf or self.ekf.get('ready') is not True or self.ekf.get('fusion_active') is not True:
            return 'selected PX4 EKF does not demonstrate healthy EV position and velocity fusion'
        return None

    def snapshot(self, now):
        return {'required': self.required, 'healthy': self.reason(now) is None,
                'reason': self.reason(now), 'frame_reset_latched': self.latched,
                'adapter': self.adapter, 'ekf': self.ekf,
                'heading_alignment_pending': self.alignment_pending,
                'heading_alignment_event': copy.deepcopy(self.heading_alignment_event),
                'heading_alignment_history': copy.deepcopy(self.heading_alignment_history),
                'heading_alignment_candidate': copy.deepcopy(self._alignment_candidate),
                'pending_deadline_monotonic_s': self.pending_deadline_monotonic_s,
                'used_for_flight_control': bool(self.required and self.ekf and
                    now-self.ekf_time <= 2 and self.ekf.get('fusion_active') is True)}
