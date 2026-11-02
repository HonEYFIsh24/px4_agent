"""Offscreen native workbench checks; no model, ROS or flight."""
import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path

try:
    from PySide6.QtGui import QImage, QColor, QPixmap
    from PySide6.QtWidgets import QApplication
    from desktop_workbench import SearchWorkbench
except ImportError as exc:
    raise unittest.SkipTest(str(exc)) from exc


@unittest.skipUnless(os.environ.get('UAV_DESKTOP_UI_TEST')=='1','requires explicit Qt test')
class WorkbenchTests(unittest.TestCase):
    def test_owned_history_and_candidate_are_read_only_and_stale_data_disappears(self):
        app=QApplication.instance() or QApplication([]);app.setQuitOnLastWindowClosed(False)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);img=QImage(640,480,QImage.Format.Format_RGB32);img.fill(QColor('#b9c9d9'));self.assertTrue(img.save(str(root/'f.jpg'),'JPEG'))
            digest=hashlib.sha256((root/'f.jpg').read_bytes()).hexdigest();epoch={'vio_epoch':0,'ekf_counters':{'xy':1,'z':1,'heading':1}}
            snapshot={'status':'valid','owner':['thread','cmd'],'world_sha256':'world','sortie_id':'sortie','coordinate_epoch':epoch,
                'grid':{'rows':48,'cols':48,'resolution_m':.5,'radius_m':12.,'origin_xy_enu_m':[0.,0.]},
                'cells':[{'coverage':0.,'semantic_score':0.} for _ in range(2304)],'eligible_cell_ids':[1,2],
                'query':'蓝色油桶','semantic_field_active':True,
                'landmarks':[{'id':'landmark1','label':'红色集装箱','position_enu_m':[3.,2.,1.],
                              'covariance_enu_m2':[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],'status':'candidate',
                              'observations':[{'bbox':[.1,.1,.3,.3]}]},
                             {'id':'landmark2','label':'蓝色油桶','position_enu_m':[4.,2.,1.],
                              'covariance_enu_m2':[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],'status':'candidate',
                              'observations':[{'bbox':[.5,.4,.6,.6]}]}],
                'observations':[{'capture_id':'f','position_enu_m':[0.,0.,2.],'visible_cell_ids':[1],'image_sha256':{'left':digest}}], 'geometric_coverage_fraction':0.}
            (root/'f.semantic-map.json').write_text(json.dumps(snapshot))
            candidate={'id':'v1','position_enu_m':[1.,1.,2.],'direction':'右移','path_verified':True,'visible_cell_ids':[1],
                       'coverage_gain_m2':1.,'semantic_gain_m2':0.,'motion_seconds':2.,'path':{'start_enu_m':[0.,0.,2.],'waypoints':[[1.,1.,2.]],'path_length_m':1.4},'score_terms':{}}
            command={'id':'cmd','status':'running','prompt':'寻找红色集装箱旁边的蓝色油桶','perceptions':[{'status':'ok','capture_id':'f','query':'油桶','camera':{'image_sha256':digest},'targets':[{'label':'油桶','bbox':[.2,.2,.5,.5]}]}],
                'semantic_map_events':[{'status':'valid','capture_id':'f','world_sha256':'world','sortie_id':'sortie'}],
                'nbv_events':[{'owner':['thread','cmd'],'source_capture_id':'f','map_revision':hashlib.sha256(json.dumps(snapshot,ensure_ascii=False,sort_keys=True,allow_nan=False).encode()).hexdigest(),'identity':{'world_sha256':'world','sortie_id':'sortie','coordinate_epoch':epoch},'selected_id':'v1','batch_id':'b','ranked_candidates':[candidate]}]}
            widget=SearchWorkbench();widget.evidence_dir=tmp;widget.resize(900,600);widget.show();widget.update_data({},command);app.processEvents()
            self.assertEqual(widget.history.count(),1);self.assertEqual(widget.candidates.count(),1);self.assertEqual(widget.selected_candidate,'v1');self.assertIsNotNone(widget._last_image)
            self.assertEqual(len(widget.map.relations),1)
            self.assertIn('指令描述：红色集装箱 旁边 蓝色油桶',widget.landmark_details.toPlainText())
            self.assertIn('地面物体占据范围尚未测得',widget.landmark_details.toPlainText())
            self.assertIn('图像框：',widget.landmark_details.toPlainText())
            widget.detail_tabs.setCurrentIndex(1)
            widget.pair_status.setPlainText('这是用于检查长复核说明可滚动显示的文字。' * 30)
            for image in widget.pair_images:image.set_image(QPixmap.fromImage(img))
            app.processEvents()
            self.assertTrue(widget.pair_status.isVisible())
            self.assertGreater(widget.pair_status.verticalScrollBar().maximum(),0)
            for image in widget.pair_images:
                self.assertLessEqual(image.pixmap().width(),image.contentsRect().width())
                self.assertLessEqual(image.pixmap().height(),image.contentsRect().height())
                top=image.mapTo(widget,image.rect().topLeft());bottom=image.mapTo(widget,image.rect().bottomRight())
                text_top=widget.pair_status.mapTo(widget,widget.pair_status.rect().topLeft())
                self.assertLess(bottom.y(),text_top.y())
            command['semantic_map_events'].append({'status':'invalid'});widget.update_data({},command);app.processEvents()
            self.assertIsNone(widget.snapshot);self.assertEqual(widget.history.count(),0);self.assertEqual(widget.candidates.count(),0)
            widget.close()


if __name__=='__main__':unittest.main()
