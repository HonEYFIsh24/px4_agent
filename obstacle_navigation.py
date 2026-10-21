"""Conservative 2-D navigation from time-aligned horizontal LaserScan data.

Only sensor rays change this map.  It accepts neither simulation object poses
nor a scene occupancy map.  The permitted rectangle is an operating boundary,
not an assertion that anything inside it is clear.  Python 3.8 / stdlib only.
"""
import copy
import heapq
import math
import threading

from mission import MissionError


class NavigationError(MissionError):
    pass


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise NavigationError("{} must be a finite number".format(name))
    return float(value)


def _point(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise NavigationError("{} must contain x and y".format(name))
    return (_number(value[0], name), _number(value[1], name))


class NavigationMap:
    """An incremental, fixed-height, static-obstacle sensor occupancy map.

    ``bounds_xy`` is [xmin, ymin, xmax, ymax] in the same ENU frame as pose.
    ``update_scan`` expects LaserScan fields plus ``stamp`` and a pose with
    ``position_m``, ``orientation_xyzw``, and ``stamp``. Positive infinity means
    a measured no-return ray; NaN and above-range samples never clear cells.
    Negative infinity and below-range returns reject the whole measurement.
    Call ``reset`` after taking off to the demonstration
    height: measurements from a different altitude cannot be combined.
    """

    def __init__(self, bounds_xy, resolution=.3, inflation=1.0,
                 visible_clearance=1.2, freshness=1.0, max_alignment=.05,
                 max_altitude_change=.35, sensor_offset_xy=(0., 0.), sensor_yaw=0.):
        if not isinstance(bounds_xy, (tuple, list)) or len(bounds_xy) != 4:
            raise NavigationError("bounds_xy must contain xmin, ymin, xmax, ymax")
        self.bounds = tuple(_number(v, "navigation bounds") for v in bounds_xy)
        if self.bounds[0] >= self.bounds[2] or self.bounds[1] >= self.bounds[3]:
            raise NavigationError("navigation bounds must have positive area")
        self.resolution = _number(resolution, "resolution")
        self.inflation = _number(inflation, "inflation")
        self.visible_clearance = _number(visible_clearance, "visible clearance")
        self.freshness = _number(freshness, "freshness")
        self.max_alignment = _number(max_alignment, "maximum scan alignment")
        self.max_altitude_change = _number(max_altitude_change, "altitude change")
        self.sensor_offset = _point(sensor_offset_xy, "sensor offset")
        self.sensor_yaw = _number(sensor_yaw, "sensor yaw")
        if not .05 <= self.resolution <= 1.0 or self.inflation < 1.0:
            raise NavigationError("resolution must be .05..1 m and obstacle inflation at least 1 m")
        if self.visible_clearance < 1.2 or not 0 < self.freshness <= 2:
            raise NavigationError("visible clearance must be at least 1.2 m and freshness at most 2 s")
        if not 0 < self.max_alignment <= .05 or not 0 < self.max_altitude_change <= .5:
            raise NavigationError("scan alignment must be at most .05 s and fixed-height tolerance at most .5 m")
        self.width = int(math.ceil((self.bounds[2] - self.bounds[0]) / self.resolution))
        self.height = int(math.ceil((self.bounds[3] - self.bounds[1]) / self.resolution))
        if self.width * self.height > 200000:
            raise NavigationError("navigation grid is too large")
        # A ray endpoint can be anywhere in its cell. This extra diagonal keeps
        # the requested physical clearance after grid discretization.
        discretization = math.sqrt(2) * self.resolution
        self._occupied_offsets = self._offsets(self.inflation + discretization)
        # Unknown cells have no measured obstacle endpoint. Half the cell
        # diagonal is sufficient to conservatively cover a point inside the
        # current cell while retaining the requested 1.2 m visible space.
        unknown_discretization = math.sqrt(2) * self.resolution / 2
        self._unknown_offsets = self._offsets(max(self.inflation, self.visible_clearance) + unknown_discretization)
        self._frontier_offsets = self._offsets(max(self.inflation, self.visible_clearance) + unknown_discretization + 2 * self.resolution)
        # A soft preference outside the unchanged hard inflation. Its sparse
        # field is built from measured occupied cells, never scene geometry.
        # Precomputing these offsets avoids a nearest-obstacle search for every
        # A* expansion. A narrow but permitted corridor remains traversable.
        self._clearance_cost_band = .9
        self._clearance_cost_factor = 1.5
        hard_radius = self.inflation + discretization
        self._clearance_cost_offsets = tuple(
            (dx, dy, 1 + self._clearance_cost_factor * max(
                0., 1 - max(0., math.hypot(dx, dy) * self.resolution - hard_radius)
                / self._clearance_cost_band) ** 2)
            for dx, dy in self._offsets(hard_radius + self._clearance_cost_band))
        self._lock = threading.RLock()
        self.reset()

    def reset(self):
        with self._lock:
            self._scores = {}
            self._scan_count = 0
            self._stamp = None
            self._received = None
            self._altitude = None
            self._position = None
            self._nearest = None
            self._valid_fraction = None
            self._passable_cache = None
            self._clearance_weights_cache = None
            self._hit_evidence = {}
            self._last_plan = None

    def _offsets(self, radius):
        cells = int(math.ceil(radius / self.resolution))
        return tuple((x, y) for y in range(-cells, cells + 1) for x in range(-cells, cells + 1)
                     if math.hypot(x, y) * self.resolution <= radius + 1e-9)

    def _inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def _cell(self, point):
        if not (self.bounds[0] <= point[0] < self.bounds[2] and self.bounds[1] <= point[1] < self.bounds[3]):
            raise NavigationError("navigation point leaves the permitted ENU rectangle")
        return (int(math.floor((point[0] - self.bounds[0]) / self.resolution)),
                int(math.floor((point[1] - self.bounds[1]) / self.resolution)))

    def _center(self, cell):
        return (self.bounds[0] + (cell[0] + .5) * self.resolution,
                self.bounds[1] + (cell[1] + .5) * self.resolution)

    def _ray_cells(self, start, end):
        """Supercover traversal; both cells at a diagonal corner are included."""
        x, y = self._cell(start)
        dx, dy = end[0] - start[0], end[1] - start[1]
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            return [(x, y)]
        next_x = self.bounds[0] + (x + (1 if dx > 0 else 0)) * self.resolution
        next_y = self.bounds[1] + (y + (1 if dy > 0 else 0)) * self.resolution
        tx = (next_x - start[0]) / dx if abs(dx) > 1e-12 else float("inf")
        ty = (next_y - start[1]) / dy if abs(dy) > 1e-12 else float("inf")
        dtx = self.resolution / abs(dx) if abs(dx) > 1e-12 else float("inf")
        dty = self.resolution / abs(dy) if abs(dy) > 1e-12 else float("inf")
        output = [(x, y)]
        for _ in range(self.width + self.height + 4):
            if min(tx, ty) > 1 + 1e-10:
                break
            if abs(tx - ty) < 1e-10:
                for cell in ((x + sx, y), (x, y + sy)):
                    if self._inside(cell) and cell not in output[-3:]:
                        output.append(cell)
                x, y, tx, ty = x + sx, y + sy, tx + dtx, ty + dty
            elif tx < ty:
                x, tx = x + sx, tx + dtx
            else:
                y, ty = y + sy, ty + dty
            if not self._inside((x, y)):
                break
            if (x, y) not in output[-3:]:
                output.append((x, y))
        return output

    def update_scan(self, scan, pose_enu, received_monotonic):
        """Validate the whole measurement before modifying any map state."""
        if not isinstance(scan, dict) or not isinstance(pose_enu, dict):
            raise NavigationError("scan and aligned ENU pose must be objects")
        received = _number(received_monotonic, "scan receipt time")
        stamp = _number(scan.get("stamp"), "scan stamp")
        pose_stamp = _number(pose_enu.get("stamp"), "pose stamp")
        if stamp < 0 or abs(stamp - pose_stamp) > self.max_alignment:
            raise NavigationError("LiDAR scan and ENU pose are not time aligned")
        position = pose_enu.get("position_m")
        orientation = pose_enu.get("orientation_xyzw")
        if not isinstance(position, (tuple, list)) or len(position) != 3:
            raise NavigationError("scan pose must contain a 3-D ENU position")
        position = tuple(_number(value, "scan ENU position") for value in position)
        if not isinstance(orientation, (tuple, list)) or len(orientation) != 4:
            raise NavigationError("scan pose must contain a quaternion")
        qx, qy, qz, qw = (_number(value, "scan orientation") for value in orientation)
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if abs(norm - 1) > .025:
            raise NavigationError("scan orientation quaternion is not normalized")
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        # The body-Z / ENU-Z angle combines roll and pitch. Independent Euler
        # bounds would admit a larger physical scan-plane inclination.
        combined_tilt = math.acos(max(-1., min(1., 1 - 2 * (qx*qx + qy*qy))))
        if combined_tilt > .20:
            raise NavigationError("2-D LiDAR mapping requires a nearly horizontal aircraft")
        yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
        origin = (position[0] + math.cos(yaw) * self.sensor_offset[0] - math.sin(yaw) * self.sensor_offset[1],
                  position[1] + math.sin(yaw) * self.sensor_offset[0] + math.cos(yaw) * self.sensor_offset[1])
        self._cell(origin)
        angle = _number(scan.get("angle_min"), "minimum scan angle")
        increment = _number(scan.get("angle_increment"), "scan angle increment")
        minimum = _number(scan.get("range_min"), "minimum scan range")
        maximum = _number(scan.get("range_max"), "maximum scan range")
        ranges = scan.get("ranges")
        if not isinstance(ranges, (list, tuple)) or not 180 <= len(ranges) <= 4096:
            raise NavigationError("LiDAR scan requires 180..4096 angular samples")
        coverage = abs(increment) * len(ranges)
        if increment == 0 or abs(coverage - 2 * math.pi) > max(.10, 2 * abs(increment)):
            raise NavigationError("obstacle navigation requires a complete 360-degree LiDAR scan")
        if not 0 <= minimum <= .3 or not minimum < maximum <= 100 or maximum < 3:
            raise NavigationError("LiDAR range limits do not meet obstacle navigation requirements")
        valid = []
        for index, raw in enumerate(ranges):
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                continue
            value = float(raw)
            if value == -float("inf") or (math.isfinite(value) and value < minimum):
                # ROS/Gazebo uses -Inf for a return nearer than range_min.
                # A single too-close return is a safety fault even if all
                # remaining rays are valid; it must not refresh a free map.
                raise NavigationError("LiDAR reports a too-close obstacle or invalid short return")
            if value == float("inf"):
                valid.append((index, maximum, False))
            elif math.isfinite(value) and minimum <= value <= maximum:
                # Gazebo ray sensors encode no collision as range_max rather
                # than infinity. A return at the measurement horizon cannot
                # establish an occupied endpoint inside that horizon.
                valid.append((index, value, value < maximum - .01))
        valid_fraction = len(valid) / float(len(ranges))
        if valid_fraction < .85:
            raise NavigationError("LiDAR scan contains too many invalid or missing rays")
        free, hits, hit_evidence = set(), set(), {}
        nearest = min((distance for _, distance, hit in valid if hit), default=None)
        for index, distance, hit in valid:
            heading = yaw + self.sensor_yaw + angle + index * increment
            endpoint = (origin[0] + math.cos(heading) * distance, origin[1] + math.sin(heading) * distance)
            cells = self._ray_cells(origin, endpoint)
            if hit and (self.bounds[0] <= endpoint[0] < self.bounds[2] and self.bounds[1] <= endpoint[1] < self.bounds[3]):
                hit_cell = self._cell(endpoint)
                hits.add(hit_cell)
                hit_evidence[hit_cell] = {"stamp_sim_s": stamp, "origin_xy_enu_m": list(origin),
                                          "ray_index": index, "range_m": distance,
                                          "scan_angle_rad": angle + index * increment,
                                          "heading_enu_rad": heading, "tilt_rad": combined_tilt,
                                          "sensor_position_enu_m": list(position),
                                          "orientation_xyzw": [qx, qy, qz, qw],
                                          "endpoint_xy_enu_m": list(endpoint)}
                free.update(cell for cell in cells if cell != hit_cell)
            else:
                free.update(cells)
        free.difference_update(hits)  # An endpoint wins over crossing rays within this scan.
        with self._lock:
            if self._stamp is not None and stamp <= self._stamp:
                raise NavigationError("LiDAR scan is duplicate or out of order")
            if self._received is not None and received < self._received:
                raise NavigationError("LiDAR receipt time moved backwards")
            if self._altitude is not None and abs(position[2] - self._altitude) > self.max_altitude_change:
                raise NavigationError("LiDAR scan altitude differs from the fixed-height navigation map")
            occupancy_changed = False
            for cell in free:
                previous_score = self._scores.get(cell, 0)
                score = max(-6, previous_score - 1)
                self._scores[cell] = score
                occupancy_changed |= (previous_score >= 2) != (score >= 2)
            for cell in hits:
                previous_score = self._scores.get(cell, 0)
                score = min(8, max(0, previous_score) + 3)
                self._scores[cell] = score
                occupancy_changed |= (previous_score >= 2) != (score >= 2)
                self._hit_evidence[cell] = hit_evidence[cell]
            self._scan_count += 1
            self._stamp, self._received = stamp, received
            if self._altitude is None:
                self._altitude = position[2]
            self._position, self._nearest, self._valid_fraction = position, nearest, valid_fraction
            self._passable_cache = None
            if occupancy_changed:
                self._clearance_weights_cache = None
            return {"accepted": True, "valid_fraction": valid_fraction, "scan_count": self._scan_count,
                    "pose_alignment_s": abs(stamp - pose_stamp), "free_ray_cells": len(free), "hit_cells": len(hits)}

    def _fresh(self, now):
        now = _number(now, "navigation time")
        if self._received is None:
            raise NavigationError("no validated LiDAR scan is available")
        age = now - self._received
        if age < -.001 or age > self.freshness:
            raise NavigationError("LiDAR source is stale or its receipt time is invalid")
        return age

    def _passable(self):
        if self._passable_cache is not None:
            return self._passable_cache
        occupied = {cell for cell, score in self._scores.items() if score >= 2}
        free = {cell for cell, score in self._scores.items() if score <= -1}
        allowed = set()
        for cell in free:
            # The aircraft footprint and visible clearance must fit completely
            # in measured free cells, including at the operating boundary.
            if any((cell[0] + dx, cell[1] + dy) not in free for dx, dy in self._unknown_offsets):
                continue
            if any((cell[0] + dx, cell[1] + dy) in occupied for dx, dy in self._occupied_offsets):
                continue
            allowed.add(cell)
        self._passable_cache = allowed
        return allowed

    def _neighbors(self, cell, allowed):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor not in allowed:
                continue
            if dx and dy and ((cell[0] + dx, cell[1]) not in allowed or (cell[0], cell[1] + dy) not in allowed):
                continue
            yield neighbor, self.resolution * (math.sqrt(2) if dx and dy else 1)

    def _clearance_weights(self):
        """Sparse soft costs, cached until sensed occupied membership changes."""
        if self._clearance_weights_cache is not None:
            return self._clearance_weights_cache
        weights = {}
        for cell, score in self._scores.items():
            if score < 2:
                continue
            for dx, dy, weight in self._clearance_cost_offsets:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if self._inside(neighbor) and weight > weights.get(neighbor, 1.):
                    weights[neighbor] = weight
        self._clearance_weights_cache = weights
        return weights

    def _search(self, start, goal, allowed):
        distance, previous = {start: 0.}, {}
        # The weighted shortest path chooses clearance; frontier progress
        # still refers to actual metres, not inflated search cost.
        lengths = {start: 0.}
        weights = self._clearance_weights()
        counter = 0
        queue = [(0., counter, start)]
        expanded = set()
        while queue:
            _, _, current = heapq.heappop(queue)
            if current in expanded:
                continue
            expanded.add(current)
            if current == goal:
                break
            for neighbor, cost in self._neighbors(current, allowed):
                candidate = distance[current] + cost * (
                    weights.get(current, 1.) + weights.get(neighbor, 1.)) / 2
                if candidate < distance.get(neighbor, float("inf")):
                    distance[neighbor], previous[neighbor] = candidate, current
                    lengths[neighbor] = lengths[current] + cost
                    counter += 1
                    heuristic = (math.hypot(neighbor[0] - goal[0], neighbor[1] - goal[1]) * self.resolution
                                 if goal is not None else 0.)
                    heapq.heappush(queue, (candidate + heuristic, counter, neighbor))
        return distance, previous, expanded, lengths

    def _line_safe(self, start, end, allowed):
        try:
            self._cell(start)
            self._cell(end)
            return all(cell in allowed for cell in self._ray_cells(start, end))
        except NavigationError:
            return False

    def _weighted_line_cost(self, start, end):
        """Midpoint integral for preference only; _line_safe is the hard gate."""
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        weights = self._clearance_weights()
        if not weights or length == 0:
            return length
        samples = max(1, int(math.ceil(length / (self.resolution / 2))))
        total = 0.
        for index in range(samples):
            fraction = (index + .5) / samples
            cell = self._cell((start[0] + (end[0] - start[0]) * fraction,
                               start[1] + (end[1] - start[1]) * fraction))
            total += weights.get(cell, 1.)
        return length * total / samples

    def _simplify(self, points, allowed):
        # A pure farthest-line-of-sight shortcut would cut back toward wall
        # corners and discard the clearance preference selected by A*.
        costs = [0.]
        for start, end in zip(points, points[1:]):
            costs.append(costs[-1] + self._weighted_line_cost(start, end))
        simplified, index = [points[0]], 0
        while index < len(points) - 1:
            end = len(points) - 1
            while end > index + 1 and (
                    not self._line_safe(points[index], points[end], allowed)
                    or self._weighted_line_cost(points[index], points[end])
                    > (costs[end] - costs[index]) * 1.01):
                end -= 1
            if not self._line_safe(points[index], points[end], allowed):
                raise NavigationError("planned path contains an unsafe segment")
            simplified.append(points[end])
            index = end
        return simplified

    def plan(self, startxy, goalxy, now):
        """A* to a known goal, or a reachable observed frontier for a new scan.

        ``goal_known`` describes the map, whereas ``route_complete`` tells the
        caller whether these waypoints actually end at the requested goal.
        A frontier route requires incremental replanning after motion; it never
        authorizes a straight jump from its endpoint into unknown space.
        """
        start, goal = _point(startxy, "start"), _point(goalxy, "goal")
        with self._lock:
            self._fresh(now)
            start_cell, goal_cell = self._cell(start), self._cell(goal)
            allowed = self._passable()
            if start_cell not in allowed:
                raise NavigationError("start is blocked or lacks observed LiDAR clearance")
            boundary_margin = max(self.inflation, self.visible_clearance)
            if (min(goal[0] - self.bounds[0], self.bounds[2] - goal[0],
                    goal[1] - self.bounds[1], self.bounds[3] - goal[1]) < boundary_margin):
                raise NavigationError("goal lacks clearance from the permitted navigation boundary")
            goal_score = self._scores.get(goal_cell, 0)
            goal_known = goal_score <= -1 or goal_score >= 2
            if goal_score >= 2:
                raise NavigationError("goal is occupied in the LiDAR map")
            if any(self._scores.get((goal_cell[0] + dx, goal_cell[1] + dy), 0) >= 2
                   for dx, dy in self._occupied_offsets):
                raise NavigationError("goal lacks observed obstacle clearance")
            distance, previous, reachable, lengths = self._search(
                start_cell, goal_cell if goal_cell in allowed else None, allowed)
            complete = goal_cell in reachable
            endpoint = goal_cell if complete else None
            if endpoint is None:
                unknown = lambda cell: self._inside(cell) and not (self._scores.get(cell, 0) <= -1 or self._scores.get(cell, 0) >= 2)
                candidates = []
                for cell in reachable:
                    if lengths[cell] < self.visible_clearance:
                        continue
                    center = self._center(cell)
                    # Unknown space behind a measured solid wall is not an
                    # observable frontier.  A candidate must face unknown via
                    # a ray that has no already-occupied cells along it.
                    visible_unknown = False
                    for dx, dy in self._frontier_offsets:
                        neighbor = (cell[0] + dx, cell[1] + dy)
                        if (unknown(neighbor) and not any(self._scores.get(ray_cell, 0) >= 2
                                for ray_cell in self._ray_cells(center, self._center(neighbor)))):
                            visible_unknown = True
                            break
                    if not visible_unknown:
                        continue
                    candidates.append((math.hypot(center[0] - goal[0], center[1] - goal[1]) + .12 * distance[cell], cell))
                if not candidates:
                    raise NavigationError("no reachable observed frontier; hold instead of crossing unknown space")
                endpoint = min(candidates)[1]
            cells, current = [endpoint], endpoint
            while current != start_cell:
                current = previous[current]
                cells.append(current)
            cells.reverse()
            points = [start] + [self._center(cell) for cell in cells[1:]]
            if complete:
                if len(points) == 1:
                    points.append(goal)
                else:
                    points[-1] = goal
            elif len(points) == 1:
                raise NavigationError("frontier planning made no safe progress")
            points = self._simplify(points, allowed)
            result = {"waypoints": [[point[0], point[1]] for point in points[1:]],
                      "goal_known": goal_known, "route_complete": complete,
                      "frontier": not complete, "path_length_m": sum(math.hypot(b[0] - a[0], b[1] - a[1])
                            for a, b in zip(points, points[1:])),
                      "source": "horizontal_lidar_aligned_enu", "scan_count": self._scan_count,
                      "scan_stamp_sim_s": self._stamp, "source_age_s": self._fresh(now),
                      "obstacle_inflation_m": self.inflation, "visible_clearance_m": self.visible_clearance,
                      "goal_xy_m": list(goal), "start_xy_m": list(start)}
            self._last_plan = result
            return copy.deepcopy(result)

    def segment_safe(self, startxy, endxy, now):
        """Hard motion gate: a stale scan or any unknown/blocked cell returns False."""
        try:
            start, end = _point(startxy, "segment start"), _point(endxy, "segment end")
            with self._lock:
                self._fresh(now)
                return self._line_safe(start, end, self._passable())
        except NavigationError:
            return False

    def diagnostics(self, now, position=None, include_map=False):
        """Safe public telemetry; obstacle identities/ground truth are unavailable."""
        with self._lock:
            try:
                age, fresh, reason = self._fresh(now), True, None
            except NavigationError as exc:
                age = None if self._received is None else _number(now, "navigation time") - self._received
                fresh, reason = False, str(exc)
            free = [cell for cell, score in self._scores.items() if score <= -1]
            occupied = [cell for cell, score in self._scores.items() if score >= 2]
            nearest = self._nearest
            if position is not None:
                point = _point(position[:2], "diagnostic position")
                if occupied:
                    nearest = max(0., min(math.hypot(self._center(cell)[0] - point[0], self._center(cell)[1] - point[1])
                                          for cell in occupied) - math.sqrt(2) * self.resolution / 2)
            result = {"source": "horizontal_lidar_aligned_enu", "fresh": fresh, "source_age_s": age,
                      "reason": reason, "scan_count": self._scan_count, "scan_stamp_sim_s": self._stamp,
                      "map_altitude_enu_m": self._altitude, "bounds_xy_enu_m": list(self.bounds),
                      "latest_pose_enu_m": None if self._position is None else list(self._position),
                      "resolution_m": self.resolution, "obstacle_inflation_m": self.inflation,
                      "visible_clearance_m": self.visible_clearance, "nearest_obstacle_m": nearest,
                      "valid_ray_fraction": self._valid_fraction, "free_cells": len(free),
                      "occupied_cells": len(occupied), "known_cells": len(free) + len(occupied),
                      "unknown_cells": self.width * self.height - len(free) - len(occupied),
                      "observed_fraction": (len(free) + len(occupied)) / float(self.width * self.height),
                      "last_plan": copy.deepcopy(self._last_plan)}
            if include_map:
                # Optional sensed grid for diagnostics/rendering. Not a scene map.
                result["grid"] = {"width": self.width, "height": self.height,
                                  "free_cells": [list(cell) for cell in sorted(free)],
                                  "occupied_cells": [list(cell) for cell in sorted(occupied)]}
                result["grid"]["occupied_hit_evidence"] = [
                    {"cell": list(cell), "score": self._scores[cell], **copy.deepcopy(self._hit_evidence[cell])}
                    for cell in sorted(occupied) if cell in self._hit_evidence]
                result["grid"]["unresolved_hit_evidence"] = [
                    {"cell": list(cell), "score": score, **copy.deepcopy(self._hit_evidence[cell])}
                    for cell, score in sorted(self._scores.items())
                    if score in (0, 1) and cell in self._hit_evidence]
            return result

    def snapshot(self, now):
        return self.diagnostics(now)
