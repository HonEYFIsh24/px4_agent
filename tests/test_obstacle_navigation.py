"""Offline sensor contracts and incremental S-route tests; no ROS/model needed."""
import math
import unittest

from obstacle_navigation import NavigationError, NavigationMap


BOUNDS = [-2.4, -7.4, 15.4, 7.4]
WALLS = [(4., -8., 5., 2.), (9., 0., 10., 8.)]


def ray_box(origin, direction, box):
    minimum, maximum = -float("inf"), float("inf")
    for position, velocity, low, high in zip(origin, direction, box[:2], box[2:]):
        if abs(velocity) < 1e-12:
            if position < low or position > high:
                return None
            continue
        first, second = (low - position) / velocity, (high - position) / velocity
        minimum, maximum = max(minimum, min(first, second)), min(maximum, max(first, second))
    if maximum < max(minimum, 0.):
        return None
    return max(minimum, 0.)


def measurement(position=(0., 0., 3.5), boxes=(), stamp=1., yaw=0., count=360, maximum=20.):
    ranges = []
    for index in range(count):
        heading = -math.pi + 2 * math.pi * index / count + yaw
        direction = (math.cos(heading), math.sin(heading))
        hits = [ray_box(position[:2], direction, box) for box in boxes]
        ranges.append(min([hit for hit in hits if hit is not None and hit <= maximum] + [maximum]))
    return ({"ranges": ranges, "angle_min": -math.pi, "angle_increment": 2 * math.pi / count,
             "range_min": .2, "range_max": maximum, "stamp": stamp},
            {"position_m": list(position), "orientation_xyzw": [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)],
             "stamp": stamp})


def load_scan(navigation, position=(0., 0., 3.5), boxes=(), stamp=1., receipt=10., **kwargs):
    scan, pose = measurement(position, boxes, stamp, **kwargs)
    navigation.update_scan(scan, pose, receipt)
    return scan, pose


def segment_wall_clearance(start, end, boxes):
    distance = float("inf")
    for index in range(201):
        point = (start[0] + (end[0] - start[0]) * index / 200., start[1] + (end[1] - start[1]) * index / 200.)
        for xmin, ymin, xmax, ymax in boxes:
            dx, dy = max(xmin - point[0], 0., point[0] - xmax), max(ymin - point[1], 0., point[1] - ymax)
            distance = min(distance, math.hypot(dx, dy))
    return distance


class SensorValidationTests(unittest.TestCase):
    def setUp(self):
        self.navigation = NavigationMap(BOUNDS)

    def test_max_range_and_positive_infinity_mean_observed_free(self):
        for value in (20., float("inf")):
            navigation = NavigationMap(BOUNDS)
            scan, pose = measurement()
            scan["ranges"] = [value] * 360
            result = navigation.update_scan(scan, pose, 10.)
            plan = navigation.plan([0., 0.], [14., 0.], 10.)
            self.assertEqual(result["hit_cells"], 0)
            self.assertTrue(plan["route_complete"])
            self.assertEqual(plan["waypoints"], [[14., 0.]])

    def test_nan_negative_infinity_and_short_ranges_never_clear(self):
        for value in (float("nan"), -float("inf"), .1, 21., True, "20"):
            scan, pose = measurement()
            scan["ranges"] = [value] * 360
            with self.subTest(value=value), self.assertRaises(NavigationError):
                self.navigation.update_scan(scan, pose, 10.)
            self.assertEqual(self.navigation.snapshot(10.)["free_cells"], 0)

    def test_small_invalid_ray_fraction_leaves_unknown_sector(self):
        scan, pose = measurement()
        scan["ranges"] = [float("nan")] * 40 + scan["ranges"][40:]
        self.navigation.update_scan(scan, pose, 10.)
        self.assertGreater(self.navigation.snapshot(10.)["unknown_cells"], 0)
        self.assertFalse(self.navigation.segment_safe([0., 0.], [-1.9, -.8], 10.))

    def test_one_too_close_ray_rejects_the_entire_scan_without_refresh(self):
        load_scan(self.navigation)
        for value in (-float("inf"), .1, -.1):
            scan, pose = measurement(stamp=2.)
            scan["ranges"][180] = value
            with self.subTest(value=value), self.assertRaisesRegex(NavigationError, "too-close"):
                self.navigation.update_scan(scan, pose, 10.9)
            self.assertEqual(self.navigation.snapshot(10.9)["scan_count"], 1)
            self.assertEqual(self.navigation.snapshot(10.9)["scan_stamp_sim_s"], 1.)
        self.assertFalse(self.navigation.segment_safe([0, 0], [1, 0], 11.01))

    def test_non360_and_quality_mismatch_are_rejected_without_state_change(self):
        scan, pose = measurement()
        for change in ({"angle_increment": math.pi / 360}, {"range_min": .5},
                       {"range_max": 2.}, {"stamp": float("nan")}, {"ranges": [20.] * 20}):
            sample = dict(scan, **change)
            with self.subTest(change=change), self.assertRaises(NavigationError):
                self.navigation.update_scan(sample, pose, 10.)
        self.assertEqual(self.navigation.snapshot(10.)["scan_count"], 0)

    def test_time_alignment_quaternion_tilt_and_height_are_checked(self):
        scan, pose = measurement()
        cases = [dict(pose, stamp=1.051), dict(pose, orientation_xyzw=[0, 0, 0, 0]),
                 dict(pose, orientation_xyzw=[math.sin(.15), 0, 0, math.cos(.15)]),
                 dict(pose, position_m=[float("nan"), 0, 3.5])]
        for bad_pose in cases:
            with self.subTest(pose=bad_pose), self.assertRaises(NavigationError):
                self.navigation.update_scan(scan, bad_pose, 10.)
        self.navigation.update_scan(scan, pose, 10.)
        high_scan, high_pose = measurement((0, 0, 4.), stamp=2.)
        with self.assertRaisesRegex(NavigationError, "altitude"):
            self.navigation.update_scan(high_scan, high_pose, 10.1)
        self.assertEqual(self.navigation.snapshot(10.1)["scan_count"], 1)

    def test_old_or_duplicate_scan_cannot_refresh_source(self):
        scan, pose = load_scan(self.navigation)
        for stamp in (1., .9):
            duplicate = dict(scan, stamp=stamp)
            with self.assertRaises(NavigationError):
                self.navigation.update_scan(duplicate, dict(pose, stamp=stamp), 10.9)
        self.assertFalse(self.navigation.segment_safe([0, 0], [1, 0], 11.01))
        with self.assertRaisesRegex(NavigationError, "stale"):
            self.navigation.plan([0, 0], [1, 0], 11.01)

    def test_combined_roll_pitch_tilt_exceeds_individual_euler_angles(self):
        scan, pose = measurement()
        angle = .15
        sine, cosine = math.sin(angle / 2), math.cos(angle / 2)
        pose["orientation_xyzw"] = [sine * cosine, cosine * sine, -sine * sine, cosine * cosine]
        with self.assertRaisesRegex(NavigationError, "horizontal"):
            self.navigation.update_scan(scan, pose, 10.)
        self.assertEqual(self.navigation.snapshot(10.)["scan_count"], 0)

    def test_reset_requires_a_new_measurement(self):
        load_scan(self.navigation)
        self.navigation.reset()
        self.assertFalse(self.navigation.segment_safe([0, 0], [1, 0], 10.))
        self.assertFalse(self.navigation.snapshot(10.)["fresh"])
        with self.assertRaisesRegex(NavigationError, "no validated"):
            self.navigation.plan([0, 0], [1, 0], 10.)

    def test_yaw_rotates_scan_to_enu(self):
        load_scan(self.navigation, boxes=[(4, -3, 5, 3)], yaw=math.pi / 2)
        self.assertFalse(self.navigation.segment_safe([0, 0], [8, 0], 10.))
        self.assertTrue(self.navigation.segment_safe([0, 0], [0, 3], 10.))


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.navigation = NavigationMap(BOUNDS)

    def test_constructor_rejects_small_physical_clearance_and_invalid_bounds(self):
        for arguments in (([0, 0, 0, 1],), (BOUNDS, .3, .9), (BOUNDS, .3, 1., 1.1),
                          (BOUNDS, .3, 1., 1.2, 3.)):
            with self.subTest(arguments=arguments), self.assertRaises(NavigationError):
                NavigationMap(*arguments)

    def test_unknown_and_outside_cells_never_authorize_motion(self):
        self.assertFalse(self.navigation.segment_safe([0, 0], [1, 0], 10.))
        load_scan(self.navigation)
        self.assertFalse(self.navigation.segment_safe([0, 0], [20, 0], 10.))
        with self.assertRaisesRegex(NavigationError, "rectangle"):
            self.navigation.plan([0, 0], [20, 0], 10.)

    def test_sensor_observed_wall_and_inflated_goal_are_rejected(self):
        load_scan(self.navigation, boxes=WALLS)
        self.assertFalse(self.navigation.segment_safe([0, 0], [14, 0], 10.))
        for goal in ([4.5, 0.], [3.1, 0.]):
            with self.subTest(goal=goal), self.assertRaisesRegex(NavigationError, "goal"):
                self.navigation.plan([0, 0], goal, 10.)

    def test_goal_seen_by_a_ray_but_with_unknown_neighbors_still_explores(self):
        load_scan(self.navigation, boxes=WALLS)
        # A narrow glimpse is still a measurement, but cannot authorize the
        # vehicle's footprint. A ray-cell update here isolates that condition.
        goal_cell = self.navigation._cell((14., 0.))
        self.navigation._scores[goal_cell] = -1
        self.navigation._passable_cache = None
        plan = self.navigation.plan([0, 0], [14, 0], 10.)
        self.assertTrue(plan["goal_known"])
        self.assertFalse(plan["route_complete"])
        self.assertTrue(plan["frontier"])

    def test_goal_near_operation_boundary_is_rejected(self):
        load_scan(self.navigation)
        with self.assertRaisesRegex(NavigationError, "boundary"):
            self.navigation.plan([0, 0], [15., 0.], 10.)

    def test_start_without_measured_clearance_is_rejected(self):
        load_scan(self.navigation, boxes=WALLS)
        with self.assertRaisesRegex(NavigationError, "start"):
            self.navigation.plan([3.5, 0], [14, 0], 10.)

    def test_hidden_goal_plans_only_to_a_reachable_observed_frontier(self):
        load_scan(self.navigation, boxes=WALLS)
        plan = self.navigation.plan([0, 0], [14, 0], 10.)
        self.assertFalse(plan["route_complete"])
        self.assertTrue(plan["frontier"])
        self.assertGreater(plan["path_length_m"], 1.2)
        position = [0, 0]
        for waypoint in plan["waypoints"]:
            self.assertTrue(self.navigation.segment_safe(position, waypoint, 10.))
            self.assertGreaterEqual(segment_wall_clearance(position, waypoint, WALLS), 1.)
            position = waypoint
        self.assertFalse(self.navigation.segment_safe(position, [14, 0], 10.))

    def test_incremental_scans_complete_s_route_and_return_home(self):
        self.check_s_route(20.)

    def test_five_meter_horizon_incrementally_explores_s_route_and_returns(self):
        replans = self.check_s_route(5.)
        self.assertGreaterEqual(replans, 4)
        self.assertLessEqual(replans, 16)

    def check_s_route(self, maximum):
        position, trajectory, replans = [0., 0.], [[0., 0.]], 0
        for goal in ([14., 0.], [0., 0.]):
            for attempt in range(16):
                replans += 1
                stamp, receipt = float(replans), 10. + replans * .1
                load_scan(self.navigation, tuple(position) + (3.5,), WALLS, stamp, receipt, maximum=maximum)
                plan = self.navigation.plan(position, goal, receipt)
                for waypoint in plan["waypoints"]:
                    self.assertTrue(self.navigation.segment_safe(position, waypoint, receipt))
                    self.assertGreaterEqual(segment_wall_clearance(position, waypoint, WALLS), 1.)
                    position = list(waypoint)
                    trajectory.append(list(position))
                if plan["route_complete"]:
                    self.assertEqual(position, goal)
                    break
            else:
                self.fail("incremental route did not reach {}".format(goal))
        self.assertTrue(any(point[1] > 3. for point in trajectory))
        self.assertTrue(any(point[1] < -1. for point in trajectory))
        self.assertGreater(replans, 2)
        return replans

    def test_entire_known_enclosure_has_no_exploration_route(self):
        enclosure = [(2., -4., 2.3, 4.), (-2., -4., -1.7, 4.), (-2., -4., 2.3, -3.7), (-2., 3.7, 2.3, 4.)]
        load_scan(self.navigation, boxes=enclosure)
        with self.assertRaisesRegex(NavigationError, "frontier"):
            self.navigation.plan([0, 0], [14, 0], 10.)

    def test_published_diagnostics_are_sensor_only_and_defensive_copies(self):
        load_scan(self.navigation)
        plan = self.navigation.plan([0, 0], [14, 0], 10.)
        plan["waypoints"][0][0] = 999
        result = self.navigation.snapshot(10.)
        self.assertEqual(result["last_plan"]["waypoints"], [[14., 0.]])
        self.assertEqual(result["source"], "horizontal_lidar_aligned_enu")
        self.assertEqual(result["latest_pose_enu_m"], [0., 0., 3.5])
        self.assertNotIn("objects", result)
        self.assertNotIn("ground_truth", result)
        self.assertNotIn("grid", result)
        self.assertEqual(result["free_cells"] + result["occupied_cells"] + result["unknown_cells"],
                         self.navigation.width * self.navigation.height)
        self.assertFalse(self.navigation.snapshot(11.1)["fresh"])


class ClearancePreferenceTests(unittest.TestCase):
    def measured_grid(self, occupied):
        """Synthetic measured free/hit cells, without a scene object API."""
        navigation = NavigationMap([0., 0., 12., 10.])
        navigation._scores = {(x, y): -6 for x in range(navigation.width)
                              for y in range(navigation.height)}
        navigation._scores.update({cell: 3 for cell in occupied})
        navigation._stamp, navigation._received, navigation._scan_count = 1., 10., 1
        return navigation

    def corner_map(self):
        return self.measured_grid({(x, y) for x in range(18, 22) for y in range(16)})

    def minimum_occupied_center_distance(self, navigation, points):
        occupied = [navigation._center(cell) for cell, score in navigation._scores.items() if score >= 2]
        result = float("inf")
        for start, end in zip(points, points[1:]):
            for index in range(201):
                point = (start[0] + (end[0] - start[0]) * index / 200.,
                         start[1] + (end[1] - start[1]) * index / 200.)
                result = min(result, min(math.hypot(point[0] - hit[0], point[1] - hit[1])
                                         for hit in occupied))
        return result

    def test_clearance_preference_moves_away_from_hits_with_same_hard_allowed_set(self):
        preferred, shortest = self.corner_map(), self.corner_map()
        allowed = set(preferred._passable())
        # A unit-cost field provides the previous shortest-path behaviour.
        shortest._clearance_weights_cache = {}
        start, goal = [3., 3.], [9., 3.]
        preferred_path = [start] + preferred.plan(start, goal, 10.)["waypoints"]
        shortest_path = [start] + shortest.plan(start, goal, 10.)["waypoints"]
        self.assertEqual(allowed, preferred._passable())
        self.assertEqual(allowed, shortest._passable())
        for first, second in zip(preferred_path, preferred_path[1:]):
            self.assertTrue(preferred.segment_safe(first, second, 10.))
        self.assertGreater(self.minimum_occupied_center_distance(preferred, preferred_path),
                           self.minimum_occupied_center_distance(shortest, shortest_path) + .2)

    def test_simplification_retains_clearance_cost_instead_of_cutting_wall_corner(self):
        navigation = self.corner_map()
        start, goal = [3., 3.], [9., 3.]
        start_cell, goal_cell = navigation._cell(start), navigation._cell(goal)
        allowed = navigation._passable()
        _, previous, _, _ = navigation._search(start_cell, goal_cell, allowed)
        cells, current = [goal_cell], goal_cell
        while current != start_cell:
            current = previous[current]
            cells.append(current)
        points = [start] + [navigation._center(cell) for cell in reversed(cells[:-1])]
        points[-1] = goal
        simplified = navigation._simplify(points, allowed)
        # Reproduce only the previous farthest safe LOS shortcut for comparison.
        greedy, index = [points[0]], 0
        while index < len(points) - 1:
            end = len(points) - 1
            while not navigation._line_safe(points[index], points[end], allowed):
                end -= 1
            greedy.append(points[end])
            index = end
        cost = lambda path: sum(navigation._weighted_line_cost(first, second)
                                for first, second in zip(path, path[1:]))
        self.assertLessEqual(cost(simplified), cost(points) * 1.01 + 1e-8)
        self.assertGreater(cost(greedy), cost(simplified) * 1.05)
        for first, second in zip(simplified, simplified[1:]):
            self.assertTrue(navigation._line_safe(first, second, allowed))

    def test_soft_cost_does_not_forbid_the_only_narrow_measured_corridor(self):
        # Two measured hit rows leave a single passable cell row. The corridor
        # carries a high soft cost everywhere but satisfies the existing gate.
        occupied = {(x, y) for x in range(40) for y in (10, 20)}
        navigation = self.measured_grid(occupied)
        start, goal = [2.25, 4.65], [9.75, 4.65]
        allowed = set(navigation._passable())
        self.assertGreater(navigation._clearance_weights().get(navigation._cell(start), 1.), 2.)
        plan = navigation.plan(start, goal, 10.)
        self.assertTrue(plan["route_complete"])
        self.assertEqual(plan["waypoints"], [goal])
        self.assertEqual(allowed, navigation._passable())
        self.assertTrue(navigation.segment_safe(start, goal, 10.))
        self.assertFalse(navigation.segment_safe(start, [6., 4.], 10.))

    def test_clearance_cache_reuses_unchanged_hits_and_invalidates_added_and_removed_hits(self):
        navigation = NavigationMap(BOUNDS)
        load_scan(navigation)
        empty = navigation._clearance_weights()
        self.assertEqual(empty, {})
        self.assertIs(navigation._clearance_weights(), empty)
        load_scan(navigation, boxes=[(4., -3., 5., 3.)], stamp=2., receipt=10.1)
        self.assertIsNone(navigation._clearance_weights_cache)
        populated = navigation._clearance_weights()
        self.assertTrue(populated)
        load_scan(navigation, boxes=[(4., -3., 5., 3.)], stamp=3., receipt=10.2)
        self.assertIs(navigation._clearance_weights(), populated)
        # Clearing rays must eventually remove old occupied membership and the
        # old preference too; no stale distance field survives that transition.
        for index in range(8):
            load_scan(navigation, stamp=4. + index, receipt=10.3 + index * .1)
        cleared = navigation._clearance_weights()
        self.assertIsNot(cleared, populated)
        self.assertEqual(cleared, {})
        navigation.reset()
        self.assertIsNone(navigation._clearance_weights_cache)


if __name__ == "__main__":
    unittest.main()
