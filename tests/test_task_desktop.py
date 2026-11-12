"""Parent/goal presentation data tests; no flight or model IO."""
import copy
import os
import json
import tempfile
import unittest
from pathlib import Path

from desktop_search_data import task_runtime_view, task_runtime_text, goal_prediction, status_text

_QT_APPLICATION = None  # Keep native QApplication owned by the main test thread.


class AcceptanceObserverStartTests(unittest.TestCase):
    def fixture(self, directory):
        state={'armed':True,'landed_state':2,'position_enu_m':[-3.,-3.,1.6],
            'mission':{'id':'owned-setup','status':'succeeded'},'navigation':{'registration_id':'sortie'},
            'scene':{'world_sha256':'a'*64},'external_vision':{'healthy':True,'adapter':{'epoch':0},
                'ekf':{'local_position_reset_counters':{'xy':2,'z':2,'heading':2}}}}
        setup={'status':'succeeded','run':'run','target_truth_queried':False,'not_counted_as_goal_pose':True,
               'final_state':copy.deepcopy(state)}
        path=Path(directory)/'controlled-observer-setup.json';path.write_text(json.dumps(setup))
        return state,setup,path

    def test_normal_ground_start_remains_required_without_explicit_setup(self):
        from scripts.validate_workbench_ui import observer_start_scope
        with tempfile.TemporaryDirectory()as directory:
            state,_,_=self.fixture(directory)
            with self.assertRaises(ValueError):observer_start_scope({},state,Path(directory)/'live-ui','run')
            state.update(armed=False,landed_state=1)
            self.assertEqual(observer_start_scope({},state,Path(directory)/'live-ui','run'),{'mode':'grounded'})

    def test_explicit_actual_setup_is_recorded_without_goal_pose_credit(self):
        from scripts.validate_workbench_ui import observer_start_scope
        with tempfile.TemporaryDirectory()as directory:
            state,_,_=self.fixture(directory)
            value=observer_start_scope({'controlled_observer_setup':'controlled-observer-setup.json'},
                                       state,Path(directory)/'live-ui','run')
            self.assertEqual(value['mode'],'controlled_airborne')
            self.assertEqual(value['setup_mission_id'],'owned-setup')
            self.assertFalse(value['setup_is_goal_pose_evidence'])

    def test_changed_mission_pose_epoch_registration_or_world_cannot_reuse_setup(self):
        from scripts.validate_workbench_ui import observer_start_scope
        with tempfile.TemporaryDirectory()as directory:
            state,_,_=self.fixture(directory)
            for kind in ('mission','position','epoch','registration','world','unhealthy'):
                bad=copy.deepcopy(state)
                if kind=='mission':bad['mission']['id']='other'
                elif kind=='position':bad['position_enu_m'][0]+=1.
                elif kind=='epoch':bad['external_vision']['adapter']['epoch']+=1
                elif kind=='registration':bad['navigation']['registration_id']='other'
                elif kind=='world':bad['scene']['world_sha256']='b'*64
                else:bad['external_vision']['healthy']=False
                with self.subTest(kind=kind),self.assertRaises(ValueError):
                    observer_start_scope({'controlled_observer_setup':'controlled-observer-setup.json'},
                                         bad,Path(directory)/'live-ui','run')

    def test_setup_reference_and_run_must_match_and_failure_never_falls_back(self):
        from scripts.validate_workbench_ui import observer_start_scope
        with tempfile.TemporaryDirectory()as directory:
            state,setup,path=self.fixture(directory)
            for reference,run in (('../controlled-observer-setup.json','run'),('controlled-observer-setup.json','other')):
                with self.assertRaises(ValueError):observer_start_scope({'controlled_observer_setup':reference},state,
                                                                        Path(directory)/'live-ui',run)
            setup['status']='failed';path.write_text(json.dumps(setup))
            with self.assertRaises(ValueError):observer_start_scope({'controlled_observer_setup':path.name},state,
                                                                    Path(directory)/'live-ui','run')


def presentation():
    owner = ['thread', 'command'];request = '找到白色货车后降落在货车前方'
    epoch = {'vio_epoch': 0, 'ekf_counters': {'xy': 1, 'z': 1, 'heading': 2}}
    hashes = {'left': 'a'*64, 'right': 'b'*64}
    snapshot = {'owner': owner, 'world_sha256': 'c'*64, 'sortie_id': 'sortie', 'coordinate_epoch': epoch,
                'observations': [{'capture_id': 'frame', 'image_sha256': hashes}]}
    candidate = {'id': 'goal_001', 'position_enu_m': [5., -5., 3.], 'yaw_enu_rad': 1.57,
        'target_revision': 3, 'coordinate_epoch': epoch, 'path': {'path_length_m': 4., 'waypoints': [[5., -5., 3.]]},
        'evidence_refs': [{'capture_id': 'frame', 'image_sha256': hashes}],
        'goal_spec': {'relation': 'object_front', 'terminal_action': 'relative_land'},
        'hard_constraints': dict.fromkeys(('identity', 'object_relation', 'permission_volume', 'path', 'target_visibility'), True),
        'execution_ticket': 'private-ticket'}
    parent = {'schema_version': 1, 'owner': owner, 'original_instruction': request,
        'plan': {'owner': owner, 'original_instruction': request, 'stages': []}, 'stage_revision': 2,
        'current_stage': {'stage_id': 'stage-02', 'kind': 'relative_land'}, 'reasoning_mode': 'LOCAL_GROUNDING',
        'target_memory': {'van': {'target_id': 'target_01', 'label': '白色货车', 'target_revision': 3, 'map_status': 'valid',
                                **{k: snapshot[k] for k in ('world_sha256', 'sortie_id', 'coordinate_epoch')}}},
        'goal_state': {'phase': 'ready', 'prepared': {'selected_id': 'goal_001', 'candidates': [candidate],
            'context': {'batch_id': 'batch'}, 'goal_context': {'target_id': 'target_01'}}}, 'result': None}
    return {'id': 'command', 'prompt': request, 'status': 'running', 'task_runtime': parent}, snapshot


class TaskPresentationTests(unittest.TestCase):
    def test_current_goal_is_distinct_from_nbv_and_does_not_expose_ticket(self):
        command, snapshot = presentation()
        value = goal_prediction(command, snapshot)
        self.assertEqual(value['planning_mode'], 'goal_pose')
        self.assertEqual(value['ranked_candidates'][0]['id'], 'goal_001')
        self.assertNotIn('execution_ticket', value['ranked_candidates'][0])
        self.assertIn('终点路径已验证', status_text(command, {}))
        self.assertIn('target_01', task_runtime_text(command))

    def test_stale_owner_target_or_coordinate_identity_cannot_display_ready_goals(self):
        command, snapshot = presentation()
        bad = copy.deepcopy(command);bad['task_runtime']['owner'][1] = 'old-command'
        self.assertIsNone(task_runtime_view(bad))
        bad = copy.deepcopy(command);bad['task_runtime']['target_memory']['van']['target_revision'] += 1
        self.assertIsNone(goal_prediction(bad, snapshot))
        bad = copy.deepcopy(snapshot);bad['coordinate_epoch']['vio_epoch'] += 1
        self.assertIsNone(goal_prediction(command, bad))

    def test_partial_grounded_result_clears_pending_goal_without_claiming_relative_success(self):
        command, snapshot = presentation()
        command['status'] = 'failed'
        command['task_runtime']['result'] = {'process_status': 'completed', 'goal_status': 'partial',
            'flight_status': 'grounded_disarmed', 'unmet_requirements': [
                {'kind': 'relative_land', 'reason_code': 'ACTUAL_LANDING_RELATIVE_POSITION_UNVERIFIED'}]}
        self.assertIsNone(goal_prediction(command, snapshot))
        self.assertIn('部分完成', status_text(command, {}))
        text = task_runtime_text(command)
        self.assertIn('grounded_disarmed', text)
        self.assertIn('ACTUAL_LANDING_RELATIVE_POSITION_UNVERIFIED', text)

    @unittest.skipUnless(os.getenv('UAV_DESKTOP_UI_TEST') == '1', 'requires explicit Qt test')
    def test_parent_details_fit_existing_native_tabs_at_both_supported_window_sizes(self):
        global _QT_APPLICATION
        from PySide6.QtWidgets import QApplication
        from PySide6.QtGui import QFont
        from PySide6.QtCore import QCoreApplication, QEvent
        from desktop_workbench import SearchWorkbench
        app = _QT_APPLICATION = QApplication.instance() or QApplication([])
        app.setQuitOnLastWindowClosed(False)
        font = app.font();app.setFont(QFont(font.family(), 13));self.addCleanup(lambda: app.setFont(font))
        command, _ = presentation()
        for width, height in ((1220, 900), (960, 720)):
            widget = SearchWorkbench();widget.resize(width, height);widget.show()
            widget.update_data({'connected': True, 'armed': True, 'landed_state': 2}, command)
            widget.detail_tabs.setCurrentWidget(widget.task_details);app.processEvents()
            self.assertIn(command['prompt'], widget.task_details.toPlainText())
            self.assertGreater(widget.task_details.viewport().height(), 60)
            self.assertGreater(widget.map.width(), 250)
            self.assertFalse(widget.grab().isNull())
            widget.close()
            widget.deleteLater()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            app.processEvents()


if __name__ == '__main__':unittest.main()
