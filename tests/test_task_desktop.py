"""Parent/goal presentation data tests; no flight or model IO."""
import copy
import os
import unittest

from desktop_search_data import task_runtime_view, task_runtime_text, goal_prediction, status_text

_QT_APPLICATION = None  # Keep native QApplication owned by the main test thread.


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
