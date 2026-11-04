import math
import unittest
import numpy as np
from analyze_landing_velocity import down_acceleration

class LandingVelocityAnalysisTests(unittest.TestCase):
    def test_level_stationary_and_downward_specific_force(self):
        self.assertAlmostEqual(float(down_acceleration([0,0,-9.80665],[1,0,0,0])),0.)
        self.assertAlmostEqual(float(down_acceleration([0,0,0],[1,0,0,0])),9.80665)
        self.assertGreater(float(down_acceleration([0,0,59.87],[1,0,0,0])),69.)

    def test_body_to_ned_rotation_not_raw_body_z(self):
        q=[math.sqrt(.5),0,math.sqrt(.5),0]
        self.assertAlmostEqual(float(down_acceleration([9.80665,0,0],q)),0.)
        q=np.array([[1,0,0,0],[-1,0,0,0]])
        np.testing.assert_allclose(down_acceleration([[0,0,-9.80665]]*2,q),[0,0])

    def test_invalid_measurements_are_rejected(self):
        for a,q in [([0,0,float('nan')],[1,0,0,0]),([0,0,0],[0,0,0,0])]:
            with self.assertRaises(ValueError):down_acceleration(a,q)
