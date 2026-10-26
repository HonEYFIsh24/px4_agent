import unittest
from vio.ekf_monitor import aid_fusion_accepted


class AidSchemaTests(unittest.TestCase):
    def aid(self, **extra):
        return dict(fused=True, innovation_rejected=False, time_last_fuse=123, **extra)

    def test_px4_114_requires_explicit_enabled_and_actual_control_flag(self):
        self.assertTrue(aid_fusion_accepted(self.aid(fusion_enabled=True),{'cs_ev_pos':True},'cs_ev_pos'))
        self.assertFalse(aid_fusion_accepted(self.aid(fusion_enabled=False),{'cs_ev_pos':True},'cs_ev_pos'))
        self.assertFalse(aid_fusion_accepted(self.aid(fusion_enabled=True),{},'cs_ev_pos'))

    def test_px4_115_requires_selected_instance_and_actual_control_flag(self):
        self.assertTrue(aid_fusion_accepted(self.aid(estimator_instance=0),{'cs_ev_vel':True},'cs_ev_vel'))
        for instance in (1,None,False,'0'):
            self.assertFalse(aid_fusion_accepted(self.aid(estimator_instance=instance),{'cs_ev_vel':True},'cs_ev_vel'))
        self.assertFalse(aid_fusion_accepted(self.aid(estimator_instance=0),{},'cs_ev_vel'))

    def test_missing_schema_or_rejected_sample_never_implies_fusion(self):
        self.assertFalse(aid_fusion_accepted(self.aid(),{'cs_ev_pos':True},'cs_ev_pos'))
        for field,value in (('fused',False),('innovation_rejected',True),('time_last_fuse',0)):
            aid=self.aid(estimator_instance=0); aid[field]=value
            self.assertFalse(aid_fusion_accepted(aid,{'cs_ev_pos':True},'cs_ev_pos'))
