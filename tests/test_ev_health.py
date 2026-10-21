import unittest
from ev_health import ExternalVisionFlightGuard


class EvGuardTest(unittest.TestCase):
    def healthy(self):
        g = ExternalVisionFlightGuard()
        g.observe_adapter(dict(mode='fuse', epoch=0, ready=True, forwarding=True), 10)
        g.observe_ekf(dict(ready=True, fusion_active=True, local_position_reset_counters={'xy': 0, 'z': 0, 'heading': 0}), 10)
        return g

    def test_shadow_does_not_claim_fusion(self):
        g = ExternalVisionFlightGuard()
        g.observe_adapter(dict(mode='shadow', epoch=0, ready=True), 1)
        self.assertFalse(g.snapshot(1)['used_for_flight_control'])

    def test_adapter_enable_does_not_prove_fusion(self):
        g = self.healthy()
        self.assertTrue(g.snapshot(10)['used_for_flight_control'])
        self.assertIsNotNone(g.reason(13))
        g.observe_ekf(dict(ready=False, fusion_active=False), 10)
        self.assertFalse(g.snapshot(10)['used_for_flight_control'])

    def test_flight_reset_latches_across_recovery(self):
        g = self.healthy()
        g.observe_adapter(dict(mode='fuse', epoch=1, ready=True, forwarding=True), 10, armed=True)
        g.observe_adapter(dict(mode='fuse', epoch=1, ready=True, forwarding=True), 11)
        self.assertIn('epoch', g.reason(11))
        g = self.healthy()
        g.observe_ekf(dict(ready=True, fusion_active=True, local_position_reset_counters={'xy': 1, 'z': 0, 'heading': 0}), 10, armed=True)
        self.assertIn('reset', g.reason(10))

    def test_drop_health_stops_new_motion(self):
        g = self.healthy()
        g.observe_adapter(dict(mode='fuse', epoch=0, ready=False, forwarding=False, reasons=['drop']), 10, armed=True)
        self.assertIn('drop', g.reason(10))
