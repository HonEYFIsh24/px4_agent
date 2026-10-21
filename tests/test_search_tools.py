"""Search tools use owned measured reports, never silently execute a route."""
import copy
import json
import math
import unittest
from types import SimpleNamespace

from search_tools import CoverageSearchError, SearchManager, build_search_tools, coordinate_epoch

try:
    from langchain.agents import create_agent
    from langchain_core.language_models import BaseChatModel
    from langchain_core.messages import HumanMessage, AIMessage, ToolMessage
    from langchain_core.outputs import ChatGeneration, ChatResult
    from pydantic import Field
    HAS_AGENT = True
except ImportError:
    HAS_AGENT = False


OWNER = ("thread-a", "turn-a")
INFO = {"width": 640, "height": 480, "stamp_sim_s": 10.,
        "K": [320., 0., 320., 0., 320., 240., 0., 0., 1.], "D": [0., 0., 0., 0., 0.]}


class MetadataBridge:
    def __init__(self):
        self.telemetry = {"ready": True, "simulation_only": True, "armed": True, "landed_state": 2,
                          "position_enu_m": [1.5, -1., 3.5], "home_enu_m": [0., 0., 0.],
                          "scene": {"verified": True, "world_sha256": 'a'*64,
                                    "navigation": {"flight_altitude_m": 3.5, "bounds_xy_m": [90., 90., 120., 120.]}},
                          "navigation": {"enabled": True, "active": True, "fresh": True, "scan_count": 5,
                                         "source": "horizontal_lidar_aligned_enu", "bounds_xy_enu_m": [-8., -8., 16., 8.],
                                         "obstacle_inflation_m": 1., "visible_clearance_m": 1.2,
                                         "map_altitude_enu_m": 3.5}, "mission": {"id": "owned-goto", "status": "succeeded"}}
        self.records, self.reads = [], []

    def state(self):
        self.reads.append("/state")
        return copy.deepcopy(self.telemetry)

    def capture_stereo(self):
        self.reads.append("/camera/stereo")
        return {"status": "ok", "left": {"camera_intrinsics": copy.deepcopy(INFO)},
                "right": {"camera_intrinsics": copy.deepcopy(INFO)},
                "body_pose_enu": {"frame_id": "map", "body_reference": "base_link", "orientation_xyzw": [0., 0., 0., 1.]}}

    def record(self, name, result):
        self.records.append((name, copy.deepcopy(result)))
        return result

    def execute(self, *args, **kwargs):
        raise AssertionError("search tool attempted flight")

    def request(self, *args, **kwargs):
        raise AssertionError("search tool attempted arbitrary Bridge request")


def planned_manager():
    client = MetadataBridge()
    manager = SearchManager(client)
    manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
    return manager, client


def saved_frame(manager, capture_id="actual-capture", owner=OWNER):
    view = manager.status(OWNER, "next")["next_view"]
    body = {"position_m": list(view["position_enu_m"]), "orientation_xyzw": [0., 0., 0., 1.],
            "frame_id": "map", "body_reference": "base_link", "stamp_sim_s": 10., "age_seconds": .1}
    return {"owner": owner, "delivered": True, "capture_id": capture_id,
            "snapshot": {"stamp_sim_s": 10., "age_seconds": .1, "measurement_age_seconds": .03,
                         "body_pose_enu": body, "stereo_image_sha256": {"left": 'a'*64, "right": 'b'*64},
                         "coordinate_epoch": copy.deepcopy(manager.sessions[OWNER]["coordinate_epoch"]),
                         "image_quality": {"laplacian_variance": 100., "mean_luma": 120., "saturated_fraction": .1}}}


class ManagerTests(unittest.TestCase):
    def test_missing_or_invalid_actual_navigation_margin_refuses_before_camera_read(self):
        for field in ("obstacle_inflation_m", "visible_clearance_m"):
            for invalid in (None, True, float("nan"), -.1):
                with self.subTest(field=field, invalid=invalid):
                    client = MetadataBridge()
                    navigation = client.telemetry["navigation"]
                    navigation.get("map", navigation)[field] = invalid
                    with self.assertRaises(ValueError):
                        SearchManager(client).plan(OWNER, "blue barrel", [3., 0., 5., 2.], 1., 4)
                    self.assertNotIn("/camera/stereo", client.reads)

    def test_flight_gate_permits_only_current_sample_and_after_failure_direct_land(self):
        manager, client = planned_manager()
        point = manager.status(OWNER, "next")["next_view"]["position_enu_m"]
        current_goto = {"action": "goto", "x": point[0], "y": point[1], "altitude": point[2]}
        self.assertTrue(manager.check_mission(OWNER, [current_goto]))
        for steps in ([{"action": "goto", "x": 4.5, "y": -1., "altitude": 3.5}],
                      [{"action": "return_home"}], [{"action": "takeoff", "altitude": 3.5}],
                      [{"action": "goto", "x": 1.5, "y": -1., "altitude": 3.5}, {"action": "land"}]):
            with self.subTest(steps=steps), self.assertRaises(CoverageSearchError):
                manager.check_mission(OWNER, steps)
        manager.fail(OWNER, "actual report failed")
        self.assertTrue(manager.check_mission(OWNER, [{"action": "land"}]))
        with self.assertRaises(CoverageSearchError):
            manager.check_mission(OWNER, [current_goto])
        self.assertNotIn("/mission", client.reads)

    def test_planner_uses_actual_map_enu_bounds_not_scene_world_bounds_and_never_flies(self):
        manager, client = planned_manager()
        status = manager.status(OWNER, "next")
        self.assertEqual(status["next_view"]["target_center_enu_m"], [1.5, -1., 0.])
        self.assertLess(status["next_view"]["position_enu_m"][0], 0.)
        self.assertEqual(client.reads, ["/state", "/camera/stereo", "/state"])
        generated = manager.sessions[OWNER]["search"].plan
        self.assertEqual(generated["navigation_permission"]["navigation_map_bounds_enu_m"], [-8., -8., 16., 8.])
        for actual, expected in zip(generated["allowed_bounds_enu_m"], [-6.8, -6.8, 14.8, 6.8]):
            self.assertAlmostEqual(actual, expected)
        self.assertFalse(generated["navigation_permission"]["scene_flight_margin_applied_again"])
        self.assertFalse(client.records[0][1]["actual_flight"])

    def test_planner_uses_heading_from_same_pair_base_link_pose_and_refuses_missing_frame(self):
        client = MetadataBridge()
        original_capture = client.capture_stereo
        def turned_pair():
            pair = original_capture()
            pair["body_pose_enu"]["orientation_xyzw"] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
            return pair
        client.capture_stereo = turned_pair
        manager = SearchManager(client)
        result = manager.plan(OWNER, "blue barrel", [0., 0., 2., 2.], 1., 4)
        self.assertAlmostEqual(result["plan"]["nominal_heading_yaw_rad"], math.pi/2)
        point = result["plan"]["views"][0]["position_enu_m"]
        self.assertAlmostEqual(point[0], .56)
        self.assertAlmostEqual(point[1], -3.406283784352713)
        bad_client = MetadataBridge()
        bad_client.capture_stereo = lambda: {"status": "ok", "left": {"camera_intrinsics": INFO},
                                           "right": {"camera_intrinsics": INFO}}
        with self.assertRaisesRegex(CoverageSearchError, "same-frame"):
            SearchManager(bad_client).plan(OWNER, "blue barrel", [0., 0., 2., 2.], 1., 4)

    def test_missing_live_map_rejects_planning_instead_of_using_world_permission(self):
        client = MetadataBridge()
        client.telemetry["navigation"] = {"enabled": True, "active": False}
        with self.assertRaisesRegex(CoverageSearchError, "ENU"):
            SearchManager(client).plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
        self.assertEqual(client.reads, ["/state"])

    def test_other_turn_cannot_read_replan_or_consume_original_owners_search(self):
        manager, _ = planned_manager()
        with self.assertRaises(CoverageSearchError):
            manager.status(("thread-a", "new-turn"), "summary")
        with self.assertRaisesRegex(CoverageSearchError, "已有"):
            manager.plan(OWNER, "new target", [0., -2., 6., 2.], 3., 4)
        frame = saved_frame(manager)
        self.assertIsNone(manager.consume_report(("thread-a", "new-turn"), frame,
                                                {"status": "ok", "capture_id": frame["capture_id"], "targets": []}))

    def test_actual_saved_report_updates_one_point_and_no_report_means_zero_coverage(self):
        manager, _ = planned_manager()
        self.assertEqual(manager.status(OWNER, "summary")["search"]["observed_views"], 0)
        frame = saved_frame(manager)
        report = {"status": "ok", "capture_id": frame["capture_id"], "targets": []}
        result = manager.consume_report(OWNER, frame, report)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["search"]["observed_views"], 1)
        self.assertEqual(manager.status(OWNER, "next")["next_view"]["id"], "view_02")

    def test_undelivered_foreign_or_failed_report_terminates_without_counting_view(self):
        for alter in (lambda f, r: f.update(delivered=False),
                      lambda f, r: f.update(owner=("thread-a", "other-turn")),
                      lambda f, r: r.update(status="error")):
            with self.subTest(alter=alter):
                manager, _ = planned_manager()
                frame = saved_frame(manager)
                report = {"status": "ok", "capture_id": frame["capture_id"], "targets": []}
                alter(frame, report)
                result = manager.consume_report(OWNER, frame, report)
                self.assertEqual(result["status"], "error")
                self.assertEqual(result["search"]["observed_views"], 0)
                self.assertIsNone(manager.status(OWNER, "next")["next_view"])

    def test_failed_navigation_is_terminal_not_skipped_as_a_successful_view(self):
        manager, client = planned_manager()
        client.telemetry["mission"]["status"] = "failed"
        result = manager.status(OWNER, "next")
        self.assertEqual(result["search"]["status"], "blocked")
        self.assertEqual(result["search"]["observed_views"], 0)
        self.assertIsNone(result["next_view"])
        self.assertTrue(result["requires_stop_new_movement"])

    def test_coordinate_epoch_change_stops_old_target_and_point_memory(self):
        manager, client = planned_manager()
        client.telemetry["external_vision"] = {"adapter": {"epoch": 1}}
        result = manager.status(OWNER, "next")
        self.assertEqual(result["search"]["status"], "observation_failed")
        self.assertIsNone(result["next_view"])

    def test_gps_ekf_xy_z_and_heading_reset_each_abort_same_vio_epoch_search(self):
        for changed_key in ("xy", "z", "heading"):
            with self.subTest(changed_key=changed_key):
                client = MetadataBridge()
                client.telemetry["external_vision"] = {"adapter": {"epoch": 2}, "ekf": {
                    "local_position_reset_counters": {"xy": 0, "z": 0, "heading": 0}}}
                manager = SearchManager(client)
                manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
                client.telemetry["external_vision"]["ekf"]["local_position_reset_counters"][changed_key] = 1
                result = manager.status(OWNER, "next")
                self.assertEqual(result["search"]["status"], "observation_failed")
                self.assertIsNone(result["next_view"])
                self.assertEqual(result["search"]["observed_views"], 0)

    def test_missing_observer_is_none_but_new_or_lost_counter_evidence_is_not_accepted_mid_search(self):
        self.assertIsNone(coordinate_epoch({"external_vision": {"adapter": None, "ekf": None}}))
        for value in ({"xy": True, "z": 0, "heading": 0}, {"xy": 0, "z": 0},
                      {"xy": -1, "z": 0, "heading": 0}, {"xy": 0, "z": 0, "heading": 256}):
            self.assertIsNone(coordinate_epoch({"external_vision": {"ekf": {"local_position_reset_counters": value}}}))
        manager, client = planned_manager()
        client.telemetry["external_vision"] = {"ekf": {"local_position_reset_counters": {"xy": 0, "z": 0, "heading": 0}}}
        self.assertEqual(manager.status(OWNER, "next")["search"]["status"], "observation_failed")
        client = MetadataBridge()
        client.telemetry["external_vision"] = {"ekf": {"local_position_reset_counters": {"xy": 0, "z": 0, "heading": 0}}}
        manager = SearchManager(client)
        manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
        client.telemetry["external_vision"]["ekf"] = None
        self.assertEqual(manager.status(OWNER, "next")["search"]["status"], "observation_failed")

    def test_public_epoch_helper_accepts_envelope_preserves_zero_and_copies_observer_fields(self):
        envelope = {"adapter": {"epoch": 0}, "ekf": {"local_position_reset_counters": {"xy": 0, "z": 255, "heading": 0}}}
        expected = {"vio_epoch": 0, "ekf_counters": {"xy": 0, "z": 255, "heading": 0}}
        self.assertEqual(coordinate_epoch(envelope), expected)
        self.assertEqual(coordinate_epoch({"external_vision": envelope}), expected)
        result = coordinate_epoch(envelope)
        result["ekf_counters"]["z"] = 0
        self.assertEqual(envelope["ekf"]["local_position_reset_counters"]["z"], 255)
        for invalid_epoch in (True, -1, "0"):
            self.assertIsNone(coordinate_epoch({"adapter": {"epoch": invalid_epoch}}))

    def test_actual_fusion_requires_complete_vio_and_ekf_identity_before_camera_io(self):
        complete = {"required": True, "adapter": {"mode": "fuse", "epoch": 0},
                    "ekf": {"local_position_reset_counters": {"xy": 0, "z": 0, "heading": 0}}}
        for missing in ("adapter", "ekf", "counter"):
            with self.subTest(missing=missing):
                client = MetadataBridge()
                client.telemetry["external_vision"] = copy.deepcopy(complete)
                if missing == "counter":
                    del client.telemetry["external_vision"]["ekf"]["local_position_reset_counters"]["heading"]
                else:
                    client.telemetry["external_vision"][missing] = None
                with self.assertRaisesRegex(CoverageSearchError, "xy/z/heading"):
                    SearchManager(client).plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
                self.assertNotIn("/camera/stereo", client.reads)
        client = MetadataBridge()
        client.telemetry["external_vision"] = complete
        manager = SearchManager(client)
        manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
        frame = saved_frame(manager)
        metric = {"status": "ok", "method": "stereo_sgbm_robust_surface", "frame": "local_enu",
                  "position_enu_m": [5., 0., 0.], "covariance_enu_m2": [[.04, 0., 0.], [0., .04, 0.], [0., 0., .04]],
                  "image_sha256": frame["snapshot"]["stereo_image_sha256"], "image_stamp_sim_s": 10.,
                  "coordinate_epoch": coordinate_epoch(complete)}
        result = manager.consume_report(OWNER, frame, {"status": "ok", "capture_id": frame["capture_id"],
                                                       "targets": [{"label": "red container", "stereo_location": metric}]})
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["search"]["targets"][0]["coordinate_epoch"], coordinate_epoch(complete))

    def test_mission_gate_checks_actual_counter_without_previous_status_tool(self):
        client = MetadataBridge()
        client.telemetry["external_vision"] = {"ekf": {"local_position_reset_counters": {"xy": 0, "z": 0, "heading": 0}}}
        manager = SearchManager(client)
        manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
        client.telemetry["external_vision"]["ekf"]["local_position_reset_counters"]["xy"] = 1
        with self.assertRaisesRegex(CoverageSearchError, "epoch"):
            manager.check_mission(OWNER, [{"action": "goto", "x": 1.5, "y": -1., "altitude": 3.5}])
        self.assertTrue(manager.check_mission(OWNER, [{"action": "land"}]))
        self.assertEqual(manager.sessions[OWNER]["search"].status, "observation_failed")

    def test_partial_shadow_identity_cannot_silently_become_required_fusion(self):
        client = MetadataBridge()
        client.telemetry["external_vision"] = {"adapter": {"mode": "shadow", "epoch": 0}, "ekf": None}
        manager = SearchManager(client)
        manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
        client.telemetry["external_vision"]["adapter"]["mode"] = "fuse"
        self.assertEqual(manager.status(OWNER, "next")["search"]["status"], "observation_failed")

    def test_old_capture_or_metric_epoch_cannot_enter_unchanged_current_epoch_session(self):
        for mismatch in ("frame", "metric"):
            with self.subTest(mismatch=mismatch):
                client = MetadataBridge()
                client.telemetry["external_vision"] = {"adapter": {"epoch": 2}, "ekf": {
                    "local_position_reset_counters": {"xy": 1, "z": 0, "heading": 0}}}
                manager = SearchManager(client)
                manager.plan(OWNER, "red container", [0., -2., 6., 2.], 3., 4)
                frame = saved_frame(manager)
                current_epoch = coordinate_epoch(client.telemetry)
                old_epoch = {"vio_epoch": 2, "ekf_counters": {"xy": 0, "z": 0, "heading": 0}}
                frame["snapshot"]["coordinate_epoch"] = old_epoch if mismatch == "frame" else current_epoch
                metric = {"status": "ok", "method": "stereo_sgbm_robust_surface", "frame": "local_enu",
                          "position_enu_m": [5., 0., 0.], "covariance_enu_m2": [[.04, 0., 0.], [0., .04, 0.], [0., 0., .04]],
                          "image_sha256": frame["snapshot"]["stereo_image_sha256"], "image_stamp_sim_s": 10.,
                          "coordinate_epoch": old_epoch if mismatch == "metric" else current_epoch}
                result = manager.consume_report(OWNER, frame, {"status": "ok", "capture_id": frame["capture_id"],
                                                               "targets": [{"label": "red container", "stereo_location": metric}]})
                self.assertEqual(result["status"], "error")
                self.assertEqual(result["search"]["observed_views"], 0)
                self.assertEqual(result["search"]["targets"], [])

    def test_unavailable_stereo_target_does_not_invent_metric_memory(self):
        manager, _ = planned_manager()
        frame = saved_frame(manager)
        report = {"status": "ok", "capture_id": frame["capture_id"], "targets": [
            {"label": "red container", "stereo_location": {"status": "unavailable", "reason": "low texture"}}]}
        result = manager.consume_report(OWNER, frame, report)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["search"]["targets"], [])


if HAS_AGENT:
    class ScriptedModel(BaseChatModel):
        script: list

        @property
        def _llm_type(self):
            return "offline-search-contract"

        def bind_tools(self, tools, **kwargs):
            return self

        def _generate(self, messages, stop=None, run_manager=None, **kwargs):
            return ChatResult(generations=[ChatGeneration(message=self.script.pop(0))])


@unittest.skipUnless(HAS_AGENT, "Search ToolRuntime graphs require Agent Python")
class GraphTests(unittest.TestCase):
    def graph(self, arguments):
        client = MetadataBridge()
        tools, manager = build_search_tools(client)
        calls = [AIMessage(content="", tool_calls=[{"name": "plan_region_search", "args": arguments,
                                                   "id": "plan-call", "type": "tool_call"}]),
                 AIMessage(content="", tool_calls=[{"name": "search_observation_status", "args": {"view": "next"},
                                                   "id": "status-call", "type": "tool_call"}]),
                 AIMessage(content="read-only plan complete")]
        result = create_agent(ScriptedModel(script=calls), tools).invoke({"messages": [HumanMessage(
            "Search the explicitly requested ENU area for red container", id=OWNER[1])]},
            {"configurable": {"thread_id": OWNER[0]}, "max_concurrency": 1})
        return result, manager, client, tools

    def test_real_graph_injects_owned_runtime_and_only_reads_and_audits(self):
        result, manager, client, tools = self.graph({"query": "red container", "bounds": [0., -2., 6., 2.],
                                                    "spacing": 3., "max_observations": 4})
        messages = [message for message in result["messages"] if isinstance(message, ToolMessage)]
        self.assertTrue(all(message.status != "error" for message in messages))
        self.assertEqual(json.loads(messages[0].content)["status"], "ok")
        self.assertIn(OWNER, manager.sessions)
        self.assertNotIn("runtime", tools[0].tool_call_schema.model_json_schema()["properties"])
        self.assertFalse(tools[0].tool_call_schema.model_json_schema()["additionalProperties"])
        self.assertEqual([name for name, result in client.records], ["plan_region_search", "search_observation_status"])

    def test_extra_model_arguments_rejected_before_calibration_or_flight_io(self):
        result, manager, client, _ = self.graph({"query": "red container", "bounds": [0., -2., 6., 2.],
                                               "spacing": 3., "max_observations": 4, "example_parameter_1": ""})
        messages = [message for message in result["messages"] if isinstance(message, ToolMessage)]
        self.assertEqual(messages[0].status, "error")
        self.assertEqual(manager.sessions, {})
        self.assertNotIn("/camera/stereo", client.reads)


if __name__ == "__main__":
    unittest.main()
