import unittest

from vio.quality import invalid_reason


class VioQualityTests(unittest.TestCase):
    def test_fresh_tracking_pose(self):
        self.assertIsNone(invalid_reason([1, 2, 3, 0, 0, 0, 1, .2, 0, 0], .1, .1, True, .1))

    def test_rejects_reset_stale_clock_and_delayed_data(self):
        values = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        for receipt, stamp, tracking, age in [(0, 0, False, 0), (0, 0, True, 2),
                                              (2, 0, True, 0), (0, 2, True, 0),
                                              (0, -1, True, 0)]:
            with self.subTest(receipt=receipt, stamp=stamp, tracking=tracking, age=age):
                self.assertIsNotNone(invalid_reason(values, receipt, stamp, tracking, age))

    def test_rejects_diverged_estimates(self):
        for values in [[float('nan'), 0, 0, 0, 0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 1, 4, 0, 0],
                       [1000, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                       [1e308, 0, 0, 0, 0, 0, 1, 0, 0, 0]]:
            self.assertIsNotNone(invalid_reason(values, 0, 0, True, 0))


if __name__ == '__main__':
    unittest.main()
