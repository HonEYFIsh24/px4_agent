import math
import unittest

from mission import MissionError, validate_mission


class MissionValidationTests(unittest.TestCase):
    def test_complete_sortie(self):
        steps = [{"action": "takeoff", "altitude": 2},
                 {"action": "goto", "x": 2, "y": 0, "altitude": 2},
                 {"action": "hold", "duration": 1},
                 {"action": "return_home"}, {"action": "land"}]
        self.assertEqual(len(validate_mission(steps)), 5)

    def test_airborne_continuation(self):
        self.assertEqual(validate_mission([{"action": "return_home"}, {"action": "land"}],
                                          airborne=True, home_known=True)[0]["action"], "return_home")

    def test_out_of_bounds_is_rejected_before_takeoff(self):
        with self.assertRaises(MissionError):
            validate_mission([{"action": "takeoff", "altitude": 2},
                              {"action": "goto", "x": 25, "y": 25, "altitude": 2}])

    def test_nonfinite_and_boolean_altitudes(self):
        for value in (math.nan, math.inf, -math.inf, True, "2", None):
            with self.subTest(value=value), self.assertRaises(MissionError):
                validate_mission([{"action": "takeoff", "altitude": value}])

    def test_altitude_bounds(self):
        for value in (0, 0.9, 10.1, -1):
            with self.subTest(value=value), self.assertRaises(MissionError):
                validate_mission([{"action": "takeoff", "altitude": value}])

    def test_ground_movement_is_rejected(self):
        for step in ({"action": "goto", "x": 0, "y": 0, "altitude": 2},
                     {"action": "return_home"}, {"action": "hold", "duration": 1}):
            with self.subTest(step=step), self.assertRaises(MissionError):
                validate_mission([step])

    def test_unknown_home_is_rejected(self):
        with self.assertRaises(MissionError):
            validate_mission([{"action": "return_home"}], airborne=True)

    def test_duplicate_takeoff_is_rejected(self):
        with self.assertRaises(MissionError):
            validate_mission([{"action": "takeoff", "altitude": 2},
                              {"action": "takeoff", "altitude": 2}])

    def test_no_commands_after_land(self):
        with self.assertRaises(MissionError):
            validate_mission([{"action": "land"}, {"action": "takeoff", "altitude": 2}])

    def test_unknown_or_extra_fields_are_rejected(self):
        for step in ({"action": "shell", "command": "anything"},
                     {"action": "takeoff", "altitude": 2, "force": True},
                     {"action": ["land"]}, {"action": "goto", "altitude": 2}):
            with self.subTest(step=step), self.assertRaises(MissionError):
                validate_mission([step], airborne=True)

    def test_empty_and_oversized_missions_are_rejected(self):
        for steps in ([], None, "land", [{"action": "hold", "duration": 1}] * 25):
            with self.subTest(steps=steps), self.assertRaises(MissionError):
                validate_mission(steps, airborne=True)

    def test_idle_hold_is_bounded(self):
        for duration in (0, 61):
            with self.subTest(duration=duration), self.assertRaises(MissionError):
                validate_mission([{"action": "hold", "duration": duration}], airborne=True)


if __name__ == "__main__":
    unittest.main()

