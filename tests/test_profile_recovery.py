import contextlib
import copy
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

from scripts import configure_ev_profile as profiles
from scripts import recover_ev_profile as recovery


class ProfileRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.system = self.root / 'system'
        self.old_run, self.new_run = self.system / 'old', self.system / 'new'
        (self.old_run / 'ev-profile').mkdir(parents=True)
        self.new_run.mkdir()
        (self.new_run / 'ev-profile.txt').write_text('gps_fused\n')
        self.px4dir = self.root / 'PX4'
        self.script = str(recovery.ROOT / 'scripts/start_system.sh')
        self.old_manager = {'pid': 10001, 'start_time': '1234',
                            'run_directory': str(self.old_run), 'script': self.script}
        self.new_manager = {'pid': 20001, 'start_time': '2234',
                            'run_directory': str(self.new_run), 'script': self.script}
        binary = str(self.px4dir / 'build/px4_sitl_default/bin/px4')
        self.old_px4 = {'pid': 10002, 'parent_pid': 10003, 'start_time': '1235',
                        'ancestor_pids': [10002, 10003, 10001], 'binary': binary}
        self.new_px4 = {'pid': 20002, 'parent_pid': 20003, 'start_time': '2235',
                        'ancestor_pids': [20002, 20003, 20001], 'binary': binary, 'state': 'S'}
        self.baseline = profiles.target_profile('gps_fused', {'SDLOG_PROFILE': 3,
                                       'COM_OBL_RC_ACT': 0, 'COM_OF_LOSS_T': .17})
        self.baseline.update(EKF2_HGT_REF=1, EKF2_EV_CTRL=15, EKF2_EV_QMIN=0, SDLOG_PROFILE=3)
        self.parameters = profiles.target_profile('gps_fused', self.baseline)
        self.source = self.old_run / 'ev-profile/original.json'
        self.write_source()
        (self.old_run / 'controller.json').write_text(json.dumps(
            {k: self.old_manager[k] for k in ('pid', 'start_time', 'script')}))
        self.state = {'simulation_only': True, 'connected': True, 'armed': False, 'landed_state': 1,
                      'gazebo': {'available': True}, 'mission': None, 'ready': False,
                      'position_enu_m': None,
                      'landing_readiness': {'ready': True,
                          'source': 'actual_MAVROS_State_and_ExtendedState_receipts',
                          'connected': True, 'armed': False, 'landed_state': 1,
                          'heartbeat_age_wall_s': .1, 'extended_state_age_wall_s': .2}}
        self.client = types.SimpleNamespace(state=lambda: copy.deepcopy(self.state))
        self.calls = []
        self.probe = lambda pid: None
        self.manager_reader = lambda run: copy.deepcopy(self.new_manager)
        self.px4_reader = lambda manager, binary: copy.deepcopy(self.new_px4)

    def write_source(self):
        self.source.write_text(json.dumps({'manager': self.old_manager, 'px4': self.old_px4,
                                           'parameters': self.baseline}))

    def command(self, args, timeout):
        self.calls.append(list(args))
        self.assertEqual(timeout, 2)
        self.assertEqual(Path(args[0]).name, 'px4-param')
        action, name = args[1:3]
        self.assertIn(name, profiles.PARAMETERS)
        self.assertIn(action, ('show', 'set'))
        if action == 'set':
            self.parameters[name] = float(args[3])
        return {'command': args, 'return_code': 0, 'stdout': name + ' : ' + str(self.parameters[name]),
                'stderr': '', 'error': None}

    def validate(self):
        return recovery.validate_source(self.source, self.px4dir, self.system, self.probe)

    def run_recovery(self, **changes):
        kwargs = dict(client=self.client, manager_reader=self.manager_reader,
                      px4_reader=self.px4_reader, command_runner=self.command,
                      probe=self.probe, system_root=self.system)
        kwargs.update(changes)
        return recovery.recover(self.source, self.new_run, self.px4dir,
                                self.new_run / 'profile-recovery', **kwargs)

    def test_restores_only_actual_fixed_values_and_preserves_old_bytes(self):
        before = self.source.read_bytes()
        result = self.run_recovery()
        self.assertTrue(result['success'])
        self.assertEqual(result['actual'], self.baseline)
        self.assertEqual(self.source.read_bytes(), before)
        self.assertEqual((self.new_run / 'profile-recovery/recovered-original.json').read_bytes(), before)
        self.assertEqual(json.loads((self.new_run / 'profile-recovery/original.json').read_text())['parameters'],
                         profiles.target_profile('gps_fused', self.baseline))
        self.assertFalse(result['arming_requested'])
        self.assertFalse(result['arming_checks_changed'])

    def test_real_profile_main_captures_recovered_baseline_for_normal_exit(self):
        self.assertTrue(self.run_recovery()['success'])
        aliases = {'run_reliability_suite': types.SimpleNamespace(
                        process_identity=self.manager_reader, owned_px4_process=self.px4_reader),
                   'record_ev_fusion': types.SimpleNamespace(
                        bounded_command=self.command,
                        stable_px4_identity=lambda d: {k:v for k,v in d.items() if k != 'state'})}
        output = self.new_run / 'ev-profile'
        with mock.patch.object(profiles, 'BridgeClient', return_value=self.client), \
                mock.patch.dict(sys.modules, aliases), contextlib.redirect_stdout(io.StringIO()):
            code = profiles.main(['--profile', 'gps_fused', '--apply', '--run-directory', str(self.new_run),
                                  '--px4-directory', str(self.px4dir), '--output', str(output)])
        self.assertEqual(code, 0)
        current_original = json.loads((output / 'original.json').read_text())['parameters']
        self.assertEqual(current_original, self.baseline)
        self.assertEqual(self.parameters, profiles.target_profile('gps_fused', self.baseline))

    def test_old_live_owner_or_px4_and_reused_identity_refuse_zero_commands(self):
        for pid, live in ((10001, {'start_time': '1234', 'uid': recovery.os.getuid(), 'state': 'S'}),
                          (10002, {'start_time': '1235', 'uid': recovery.os.getuid(), 'state': 'R'}),
                          (10001, {'start_time': '9999', 'uid': recovery.os.getuid(), 'state': 'S'})):
            with self.subTest(pid=pid, live=live), self.assertRaises(RuntimeError):
                self.run_recovery(probe=lambda p: live if p == pid else None)
        self.assertEqual(self.calls, [])
        self.assertFalse((self.new_run / 'profile-recovery').exists())

    def test_stopped_zombie_with_exact_identity_is_dead(self):
        self.probe = lambda p: {'start_time': '1234' if p == 10001 else '1235',
                                'uid': recovery.os.getuid(), 'state': 'Z'}
        self.assertEqual(self.validate()['parameters'], self.baseline)

    def test_controller_or_px4_ancestry_mismatch_rejected(self):
        for mutate in ('controller', 'binary', 'ancestry'):
            with self.subTest(mutate=mutate):
                controller = self.old_run / 'controller.json'
                good = controller.read_bytes()
                if mutate == 'controller':
                    controller.write_text(json.dumps({'pid': 10009, 'start_time': '1234', 'script': self.script}))
                elif mutate == 'binary':
                    self.old_px4['binary'] = '/foreign/bin/px4'; self.write_source()
                else:
                    self.old_px4['ancestor_pids'] = [10002, 10003, 8888]; self.write_source()
                with self.assertRaises(RuntimeError): self.validate()
                controller.write_bytes(good)
                self.old_px4.update(binary=str(self.px4dir / 'build/px4_sitl_default/bin/px4'),
                                    ancestor_pids=[10002, 10003, 10001])
                self.write_source()

    def test_arbitrary_parameters_bool_nonfinite_or_shell_string_are_invalid(self):
        for poison in (True, float('nan'), float('inf'), '0; echo unexpected'):
            with self.subTest(poison=poison):
                self.baseline['EKF2_EV_CTRL'] = poison; self.write_source()
                with self.assertRaises(RuntimeError): self.validate()
        self.baseline['EKF2_EV_CTRL'] = 15
        self.baseline['COM_ARM_WO_GPS'] = 1; self.write_source()
        with self.assertRaises(RuntimeError): self.validate()

    def test_duplicate_record_keys_and_noncanonical_archive_refused(self):
        self.source.write_text('{"manager":{},"manager":{},"px4":{},"parameters":{}}')
        with self.assertRaises(RuntimeError): self.validate()
        other = self.root / 'unowned-original.json'; other.write_bytes(self.source.read_bytes())
        with self.assertRaises(RuntimeError):
            recovery.validate_source(other, self.px4dir, self.system, self.probe)

    def test_ground_and_fresh_actual_fcu_gate_refuses_without_commands(self):
        good = copy.deepcopy(self.state)
        bads = [{'armed': True}, {'connected': False}, {'landed_state': True},
                {'gazebo': {'available': False}}, {'mission': {'status': 'running'}},
                {'landing_readiness': {}}]
        for change in bads:
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                self.state = dict(good, **change); self.run_recovery()
        self.assertEqual(self.calls, [])
        self.assertFalse((self.new_run / 'profile-recovery').exists())

    def test_receipt_ages_fail_closed_and_do_not_require_xy(self):
        recovery.fresh_ground_guard(self.state)
        for field in ('heartbeat_age_wall_s', 'extended_state_age_wall_s'):
            for age in (None, True, float('nan'), float('inf'), -.001, 3.001):
                state = copy.deepcopy(self.state); state['landing_readiness'][field] = age
                with self.subTest(field=field, age=age), self.assertRaises(RuntimeError):
                    recovery.fresh_ground_guard(state)

    def test_lost_ground_after_first_command_stops_and_saves_first_error(self):
        def runner(args, timeout):
            result = self.command(args, timeout)
            self.state['armed'] = True
            return result
        result = self.run_recovery(command_runner=runner)
        self.assertFalse(result['success'])
        self.assertIn('grounded', result['error'])
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(json.loads((self.new_run / 'profile-recovery/result.json').read_text())['error'], result['error'])

    def test_identity_change_after_mutation_prevents_foreign_rollback(self):
        def runner(args, timeout):
            result = self.command(args, timeout)
            if args[1] == 'set': self.new_px4['start_time'] = '99999'
            return result
        result = self.run_recovery(command_runner=runner)
        self.assertFalse(result['success'])
        self.assertIn('identity changed', result['error'])
        self.assertIn('identity changed', result['rollback_error'])
        self.assertEqual(self.calls[-1][1], 'set')
        self.assertEqual(sum(c[1] == 'set' for c in self.calls), 1)

    def test_changed_old_source_is_detected_before_further_parameter_commands(self):
        def runner(args, timeout):
            result = self.command(args, timeout)
            self.baseline['EKF2_EV_DELAY'] = 1; self.write_source()
            return result
        result = self.run_recovery(command_runner=runner)
        self.assertFalse(result['success'])
        self.assertIn('old recovery source changed', result['error'])
        self.assertEqual(len(self.calls), 1)

    def test_only_fresh_boot_before_profile_capture_and_not_no_gps(self):
        (self.new_run / 'ev-profile.txt').write_text('no_gps\n')
        with self.assertRaisesRegex(RuntimeError, 'no_gps'): self.run_recovery()
        (self.new_run / 'ev-profile.txt').write_text('gps_fused\n')
        (self.new_run / 'ev-profile').mkdir()
        (self.new_run / 'ev-profile/original.json').write_text('{}')
        with self.assertRaisesRegex(RuntimeError, 'startup-only'): self.run_recovery()
        self.assertEqual(self.calls, [])

    def test_real_readback_mismatch_is_failure_and_verified_rollback(self):
        initial = copy.deepcopy(self.parameters)
        def runner(args, timeout):
            if args[1:] == ['set', 'EKF2_HGT_REF', '1']:
                self.calls.append(list(args))
                return {'command': args, 'return_code': 0, 'stdout': '', 'stderr': '', 'error': None}
            return self.command(args, timeout)
        result = self.run_recovery(command_runner=runner)
        self.assertFalse(result['success'])
        self.assertIn('readback mismatch', result['error'])
        self.assertTrue(result['rolled_back'])
        self.assertEqual(self.parameters, initial)

    def test_default_source_check_never_constructs_runtime_or_applies(self):
        with mock.patch.object(recovery, 'validate_source', return_value={'source': 'record'}), \
                mock.patch.object(recovery, 'recover') as apply, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(recovery.main(['--source', str(self.source)]), 0)
        apply.assert_not_called()

    def test_startup_help_and_no_gps_combination_fail_before_launch(self):
        script = recovery.ROOT / 'scripts/start_system.sh'
        help_result = subprocess.run(['bash', str(script), '--help'], capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0)
        self.assertIn('--recover-profile-from', help_result.stdout)
        rejected = subprocess.run(['bash', str(script), '--ev-profile', 'no_gps',
                                   '--recover-profile-from', str(self.source)], capture_output=True, text=True)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn('no_gps 冷启动', rejected.stderr)
        source = script.read_text()
        sequence = [source.index('while ! get_state || ! state_is ground_connected'),
                    source.index('-s HEARTBEAT -r 5'), source.index('-s EXTENDED_SYS_STATE -r 5'),
                    source.index('while ! get_state || ! state_is recovery_ground_connected'),
                    source.index('--source "$RECOVER_PROFILE_FROM" --apply'),
                    source.index('--profile "$EV_PROFILE" --apply')]
        self.assertEqual(sequence, sorted(sequence))
        self.assertLess(source.index('--profile "$EV_PROFILE" --apply'),
                        source.index('[[ "$EV_PROFILE" == no_gps ]] || PROFILE_RESTORE_FILE='))


if __name__ == '__main__':
    unittest.main()
