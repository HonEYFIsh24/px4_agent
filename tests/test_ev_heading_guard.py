"""Offline safety contracts for the one permitted takeoff magnetic alignment."""
import copy
import math
import unittest
from ev_health import ExternalVisionFlightGuard
from vio.ekf_monitor import heading_observation_fields


def telemetry(source=10., heading=1, aligned=False, good=False):
    return {'source':'actual_owned_px4_uorb','ready':True,'fusion_active':True,
            'position_fusion':True,'velocity_fusion':True,'local_position_valid':True,
            'local_velocity_valid':True,'primary_instance':0,'observed_sim_s':source+.01,
            'local_position_source_stamp_sim_s':source,'local_position_publish_stamp_sim_s':source,
            'local_position_reset_counters':{'xy':2,'z':1,'heading':heading},
            'local_velocity_reset_counters':{'vxy':2,'vz':2},
            'heading_delta_ned_rad':math.radians(1.),'heading_good_for_control':good,
            'mag_alignment':{'aligned_in_flight':aligned,'yaw_aligned':True,'in_air':True,
                             'source_stamp_sim_s':source-.1,'publish_stamp_sim_s':source-.02}}


def adapter(epoch=0):
    return {'mode':'fuse','epoch':epoch,'ready':True,'forwarding':True}


class HeadingGuardTests(unittest.TestCase):
    def initialized(self):
        guard=ExternalVisionFlightGuard();guard.observe_adapter(adapter(),100.)
        guard.observe_ekf(telemetry(),100.)
        return guard

    def accept(self,guard):
        guard.observe_adapter(adapter(),100.5,armed=True)
        return guard.observe_ekf(telemetry(10.2,2,True),100.5,armed=True,allow_takeoff_mag_alignment=True)

    def test_accept_emits_one_event_and_pending_is_independent_of_fatal_health(self):
        g=self.initialized();event=self.accept(g)
        self.assertEqual(event['delta_heading_ned_rad'],math.radians(1.))
        self.assertEqual(event['reset_source_stamp_sim_s'],10.2)
        self.assertEqual(event['stamp_semantics'],'first_observed_local_position_sample_after_heading_counter_change')
        self.assertTrue(g.alignment_pending);self.assertIsNone(g.reason(100.5))
        self.assertEqual(g.pending_deadline_monotonic_s,110.5)
        snap=g.snapshot(100.5);self.assertTrue(snap['heading_alignment_pending'])
        self.assertTrue(snap['healthy']);self.assertEqual(len(snap['heading_alignment_history']),1)
        event['delta_heading_ned_rad']=999;self.assertNotEqual(g.heading_alignment_event['delta_heading_ned_rad'],999)
        self.assertIsNone(g.observe_ekf(telemetry(10.3,2,True),100.6,armed=True,allow_takeoff_mag_alignment=True))
        self.assertEqual(len(g.heading_alignment_history),1)

    def test_pending_clears_only_from_new_actual_good_heading_not_receipt_time(self):
        g=self.initialized();self.accept(g)
        g.observe_adapter(adapter(),101.,armed=True)
        self.assertIsNone(g.observe_ekf(telemetry(10.4,2,True,False),101.,armed=True))
        self.assertTrue(g.alignment_pending)
        g.observe_adapter(adapter(),101.5,armed=True)
        self.assertIsNone(g.observe_ekf(telemetry(11.3,2,True,True),101.5,armed=True))
        self.assertFalse(g.alignment_pending);self.assertIsNone(g.pending_deadline_monotonic_s)
        self.assertIsNone(g.reason(101.5));self.assertEqual(len(g.heading_alignment_history),1)

    def test_default_or_outside_takeoff_does_not_exempt_heading(self):
        g=self.initialized()
        self.assertIsNone(g.observe_ekf(telemetry(10.2,2,True),100.5,armed=True))
        self.assertIn('reset',g.reason(100.5));self.assertFalse(g.alignment_pending)

    def test_counter_first_waits_for_real_fresh_false_true_then_emits_once(self):
        g=self.initialized();counter=telemetry(10.2,2,False)
        g.observe_adapter(adapter(),100.5,armed=True)
        self.assertIsNone(g.observe_ekf(counter,100.5,armed=True,allow_takeoff_mag_alignment=True))
        self.assertTrue(g.alignment_pending);self.assertIsNone(g.heading_alignment_event)
        self.assertIsNone(g.reason(100.5))
        event=g.observe_ekf(telemetry(10.3,2,True),100.8,armed=True,allow_takeoff_mag_alignment=True)
        self.assertIsNotNone(event);self.assertEqual(event['reset_source_stamp_sim_s'],10.2)
        self.assertAlmostEqual(event['mag_alignment_source_stamp_sim_s'],10.2)
        self.assertEqual(len(g.heading_alignment_history),1)

    def test_flags_first_transition_can_match_subsequent_counter(self):
        g=self.initialized()
        self.assertIsNone(g.observe_ekf(telemetry(10.1,1,True),100.2,armed=True,allow_takeoff_mag_alignment=True))
        event=g.observe_ekf(telemetry(10.2,2,True),100.4,armed=True,allow_takeoff_mag_alignment=True)
        self.assertIsNotNone(event);self.assertEqual(event['mag_alignment_source_stamp_sim_s'],10.)

    def test_unchanging_true_flag_cannot_invent_a_false_true_transition(self):
        g=ExternalVisionFlightGuard();g.observe_adapter(adapter(),100.)
        g.observe_ekf(telemetry(aligned=True),100.)
        self.assertIsNone(self.accept(g));self.assertIn('reset',g.reason(100.5))

    def test_accepted_once_per_flight_and_modulo_wrap_is_valid(self):
        g=self.initialized();g.counters['heading']=255
        g.ekf['local_position_reset_counters']['heading']=255
        event=g.observe_ekf(telemetry(10.2,0,True),100.5,armed=True,allow_takeoff_mag_alignment=True)
        self.assertIsNotNone(event)
        g.observe_adapter(adapter(),101.,armed=True)
        g.observe_ekf(telemetry(10.4,0,True,True),101.,armed=True)
        self.assertFalse(g.alignment_pending)
        self.assertIsNone(g.observe_ekf(telemetry(10.5,1,True,True),101.1,armed=True,allow_takeoff_mag_alignment=True))
        self.assertIn('reset',g.reason(101.1));self.assertEqual(len(g.heading_alignment_history),1)

    def test_second_alignment_in_same_flight_fails_even_with_new_false_true(self):
        g=self.initialized();self.accept(g)
        g.observe_ekf(telemetry(10.3,2,True,True),100.6,armed=True)
        g.observe_ekf(telemetry(10.4,2,False,True),100.7,armed=True)
        self.assertIsNone(g.observe_ekf(telemetry(10.5,3,True),100.8,armed=True,allow_takeoff_mag_alignment=True))
        self.assertIn('reset',g.reason(100.8));self.assertEqual(len(g.heading_alignment_history),1)

    def test_xy_z_velocity_primary_and_epoch_changes_remain_fatal(self):
        for change in ('xy','z','vxy','vz','primary','epoch'):
            g=self.initialized();value=telemetry(10.2,2,True)
            if change in ('xy','z'):value['local_position_reset_counters'][change]+=1
            if change in ('vxy','vz'):value['local_velocity_reset_counters'][change]+=1
            if change=='primary':value['primary_instance']=1
            if change=='epoch':g.observe_adapter(adapter(1),100.5,armed=True)
            with self.subTest(change=change):
                self.assertIsNone(g.observe_ekf(value,100.5,armed=True,allow_takeoff_mag_alignment=True))
                self.assertIsNotNone(g.reason(100.5));self.assertFalse(g.heading_alignment_history)

    def test_velocity_and_primary_change_are_fatal_even_without_heading_change(self):
        for change in ('velocity','primary'):
            g=self.initialized();value=telemetry(10.2)
            if change=='velocity':value['local_velocity_reset_counters']['vxy']+=1
            else:value['primary_instance']=1
            g.observe_ekf(value,100.5,armed=True)
            self.assertIsNotNone(g.reason(100.5))

    def test_typed_range_and_angle_boundaries(self):
        for delta in (math.radians(3)+1e-8,True,'0.01',None,float('nan'),float('inf')):
            g=self.initialized();value=telemetry(10.2,2,True);value['heading_delta_ned_rad']=delta
            self.assertIsNone(g.observe_ekf(value,100.5,armed=True,allow_takeoff_mag_alignment=True))
            self.assertIsNotNone(g.reason(100.5))
        for delta in (-math.radians(3),math.radians(3)):
            g=self.initialized();value=telemetry(10.2,2,True);value['heading_delta_ned_rad']=delta
            self.assertIsNotNone(g.observe_ekf(value,100.5,armed=True,allow_takeoff_mag_alignment=True))
        for counter in (True,2.,'2',None,256,-1,3):
            g=self.initialized();value=telemetry(10.2,2,True);value['local_position_reset_counters']['heading']=counter
            self.assertIsNone(g.observe_ekf(value,100.5,armed=True,allow_takeoff_mag_alignment=True))
            self.assertIsNotNone(g.reason(100.5))

    def test_missing_malformed_stale_future_and_unowned_source_cannot_be_excused(self):
        changes=('missing_sample','missing_flags','fake_source','old_local','future_local','old_flags',
                 'future_flags','sample_after_publish','invalid_flag','invalid_good','no_pos','no_vel',
                 'invalid_velocity','invalid_primary','old_baseline_gap','missing_baseline_velocity')
        for change in changes:
            g=self.initialized();value=telemetry(10.2,2,True)
            if change=='missing_sample':value.pop('local_position_source_stamp_sim_s')
            if change=='missing_flags':value.pop('mag_alignment')
            if change=='fake_source':value['source']='synthetic'
            if change=='old_local':value['observed_sim_s']=11.
            if change=='future_local':value['observed_sim_s']=9.9
            if change=='old_flags':value['mag_alignment'].update(source_stamp_sim_s=8.,publish_stamp_sim_s=8.)
            if change=='future_flags':value['mag_alignment'].update(source_stamp_sim_s=10.5,publish_stamp_sim_s=10.5)
            if change=='sample_after_publish':value['mag_alignment']['source_stamp_sim_s']=10.25
            if change=='invalid_flag':value['mag_alignment']['aligned_in_flight']=1
            if change=='invalid_good':value['heading_good_for_control']=0
            if change=='no_pos':value['position_fusion']=False
            if change=='no_vel':value['velocity_fusion']=False
            if change=='invalid_velocity':value['local_velocity_reset_counters']['vz']=True
            if change=='invalid_primary':value['primary_instance']=False
            if change=='old_baseline_gap':value=telemetry(10.7,2,True)
            if change=='missing_baseline_velocity':g.velocity_counters=None
            with self.subTest(change=change):
                self.assertIsNone(g.observe_ekf(value,100.5,armed=True,allow_takeoff_mag_alignment=True))
                self.assertIsNotNone(g.reason(100.5));self.assertFalse(g.heading_alignment_history)

    def test_shadow_unready_stale_or_changed_epoch_adapter_cannot_authorize_alignment(self):
        for change in ('shadow','ready','forwarding','stale','invalid_epoch','changed_epoch'):
            g=self.initialized();a=adapter();when=100.5
            if change=='shadow':a['mode']='shadow'
            if change in ('ready','forwarding'):a[change]=False
            if change=='stale':when=98.
            if change=='invalid_epoch':a['epoch']=False
            if change=='changed_epoch':a['epoch']=1
            g.observe_adapter(a,when,armed=True)
            self.assertIsNone(g.observe_ekf(telemetry(10.2,2,True),100.5,armed=True,allow_takeoff_mag_alignment=True))
            self.assertIsNotNone(g.latched);self.assertFalse(g.heading_alignment_history)

    def test_matching_source_window_and_transition_wall_age_are_strict(self):
        for mag_sample,received in ((9.69,100.5),(10.71,100.5),(10.1,103.)):
            g=self.initialized();value=telemetry(10.2,2,True)
            value['mag_alignment'].update(source_stamp_sim_s=mag_sample,publish_stamp_sim_s=max(10.18,mag_sample))
            if received==103.:
                g.observe_ekf(telemetry(10.1,1,True),100.2,armed=True,allow_takeoff_mag_alignment=True)
                g.observe_adapter(adapter(),received,armed=True)
            with self.subTest(mag_sample=mag_sample,received=received):
                self.assertIsNone(g.observe_ekf(value,received,armed=True,allow_takeoff_mag_alignment=True))
                self.assertIsNotNone(g.reason(received))

    def test_counter_first_cannot_wait_past_half_second_source_window(self):
        g=self.initialized()
        g.observe_ekf(telemetry(10.2,2,False),100.5,armed=True,allow_takeoff_mag_alignment=True)
        g.observe_adapter(adapter(),101.,armed=True)
        self.assertIsNone(g.observe_ekf(telemetry(10.71,2,False),101.,armed=True,allow_takeoff_mag_alignment=True))
        self.assertIn('source window expired',g.reason(101.))

    def test_pending_deadline_expires_without_callback_and_is_not_rescheduled(self):
        g=self.initialized();self.accept(g)
        deadline=g.pending_deadline_monotonic_s
        g.observe_adapter(adapter(),101.,armed=True)
        g.observe_ekf(telemetry(11.,2,True),101.,armed=True)
        self.assertEqual(g.pending_deadline_monotonic_s,deadline)
        self.assertIn('timed out',g.reason(deadline))
        g.observe_ekf(telemetry(11.1,2,True,True),deadline+.1,armed=True)
        self.assertIsNotNone(g.reason(deadline+.1));self.assertTrue(g.alignment_pending)

    def test_pending_missing_or_old_source_still_fails_and_never_clears_latch(self):
        for change in ('missing','backwards','fusion','epoch','xy','velocity','primary','disarmed'):
            g=self.initialized();self.accept(g);value=telemetry(10.3,2,True,True)
            if change=='missing':value.pop('local_position_source_stamp_sim_s')
            if change=='backwards':value=telemetry(10.1,2,True,True)
            if change=='fusion':value['position_fusion']=False
            if change=='epoch':g.observe_adapter(adapter(1),100.6,armed=True)
            if change=='xy':value['local_position_reset_counters']['xy']+=1
            if change=='velocity':value['local_velocity_reset_counters']['vxy']+=1
            if change=='primary':value['primary_instance']=1
            with self.subTest(change=change):
                self.assertIsNone(g.observe_ekf(value,100.6,armed=change!='disarmed'))
                self.assertIsNotNone(g.reason(100.6));self.assertTrue(g.alignment_pending)
                g.observe_ekf(telemetry(10.4,2,True,True),100.7,armed=True)
                self.assertIsNotNone(g.reason(100.7));self.assertTrue(g.alignment_pending)

    def test_budget_reopens_only_after_real_disarmed_ground_false_flag_and_history_retained(self):
        g=self.initialized();self.accept(g)
        g.observe_ekf(telemetry(10.3,2,True,True),100.6,armed=True)
        ground=telemetry(10.4,2,False);ground['mag_alignment']['in_air']=False
        g.observe_ekf(ground,100.7,armed=False)
        g.observe_adapter(adapter(),101.,armed=True)
        self.assertIsNotNone(g.observe_ekf(telemetry(10.5,3,True),101.,armed=True,allow_takeoff_mag_alignment=True))
        self.assertEqual(len(g.heading_alignment_history),2)

    def test_one_hz_unchanging_flags_remain_fresh_but_no_timestamp_is_rewritten(self):
        g=self.initialized();self.accept(g)
        value=telemetry(11.2,2,True,True)
        value['mag_alignment'].update(source_stamp_sim_s=10.1,publish_stamp_sim_s=10.18)
        original=copy.deepcopy(value)
        g.observe_adapter(adapter(),101.,armed=True)
        g.observe_ekf(value,101.,armed=True)
        self.assertFalse(g.alignment_pending);self.assertEqual(value,original)

    def test_pending_adapter_and_same_counter_delta_must_stay_consistent(self):
        for change in ('ready','forwarding','delta','stale'):
            g=self.initialized();self.accept(g);value=telemetry(10.3,2,True,True)
            a=adapter();when=100.6
            if change in ('ready','forwarding'):a[change]=False
            if change=='delta':value['heading_delta_ned_rad']=math.radians(2.)
            if change=='stale':when=98.
            g.observe_adapter(a,when,armed=True)
            with self.subTest(change=change):
                self.assertIsNone(g.observe_ekf(value,100.6,armed=True))
                self.assertIsNotNone(g.reason(100.6));self.assertTrue(g.alignment_pending)


class HeadingMonitorFieldsTests(unittest.TestCase):
    def test_actual_samples_publication_times_and_typed_flags_preserved(self):
        local={'timestamp_sample':72684000,'timestamp':72688000,'heading_good_for_control':False,
               'delta_heading':.01735,'vxy_reset_counter':2,'vz_reset_counter':2}
        flags={'timestamp_sample':72432000,'timestamp':72532000,'cs_mag_aligned_in_flight':True,
               'cs_yaw_align':True,'cs_in_air':True}
        result=heading_observation_fields(local,flags)
        self.assertEqual(result['local_position_source_stamp_sim_s'],72.684)
        self.assertEqual(result['local_position_publish_stamp_sim_s'],72.688)
        self.assertEqual(result['mag_alignment']['source_stamp_sim_s'],72.432)
        self.assertEqual(result['mag_alignment']['publish_stamp_sim_s'],72.532)
        self.assertEqual(result['heading_delta_ned_rad'],.01735)
        self.assertFalse(result['heading_good_for_control'])
        self.assertEqual(result['local_velocity_reset_counters'],{'vxy':2,'vz':2})

    def test_missing_or_malformed_actual_times_not_substituted_with_publication_or_receipt(self):
        for malformed in (True,0,-1,None,72.,'72000000'):
            result=heading_observation_fields({'timestamp_sample':malformed,'timestamp':72000000,
                 'heading_good_for_control':1},{'timestamp_sample':malformed,'timestamp':72000000,
                 'cs_mag_aligned_in_flight':1})
            self.assertIsNone(result['local_position_source_stamp_sim_s'])
            self.assertIsNone(result['mag_alignment']['source_stamp_sim_s'])
            self.assertIsNone(result['heading_good_for_control'])
            self.assertIsNone(result['mag_alignment']['aligned_in_flight'])
            self.assertEqual(result['local_position_publish_stamp_sim_s'],72.)
            self.assertIsNone(result['local_velocity_reset_counters']['vxy'])


if __name__=='__main__':unittest.main()
