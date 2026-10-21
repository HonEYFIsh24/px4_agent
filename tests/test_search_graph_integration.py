"""Production single-VLM region-search wiring with local deterministic doubles.

No ROS service, socket, provider request, or real flight is used. The synthetic
JPEG fixture is genuinely 640x480; stereo localization below is an explicit
transport double, while production tools, manager, middleware and web audits run.
"""
import base64
import contextlib
import copy
import hashlib
import io
import json
import os
import struct
import unittest
from unittest import mock

try:
    from langchain_core.messages import AIMessage
    from test_single_vlm import LocalScriptModel, encoded, image_urls, tool_call, tool_payload
    from test_vision import CONFIG
    from test_web_server import completed
    import agent
    import search_tools
    import single_vlm
    from web_server import WebApplication
except ImportError as exc:
    raise unittest.SkipTest("Production graph integration requires Agent Python LangChain") from exc

# Independently encoded grayscale checkerboard, 640x480 JPEG, quality 90.
JPEG_B64 = "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgFBQQEBQoHBwYIDAoMDAsKCwsNDhIQDQ4RDgsLEBYQERMUFRUVDA8XGBYUGBIUFRT/wAALCAHgAoABAREA/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/APmSiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooooooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKKKKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiiiiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooooooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKKKKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiiiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooooooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKKKKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiiiiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooooooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKKKKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiiiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooooooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKKKKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/SaiiivzZooor9JqKKK/Nmiiiv0mooor82aKKK/Saiiiv//Z"
QUALITY = json.loads("{\"laplacian_variance\":1508.0887451171875,\"mean_luma\":125,\"saturated_fraction\":0,\"source\":\"offline_synthetic_640x480_jpeg_fixture\",\"not_semantic_visibility_proof\":true}")
QUERY = "蓝色桶"
PROMPT = "起飞3.5米，在ENU x[3,5] y[0,2]区域按1米网格寻找蓝色桶，最多4观察点，逐点采图报告，最后返航降落。"


def calibration(stamp, side):
    return {"width": 640, "height": 480, "stamp_sim_s": stamp,
            "K": [320., 0., 320., 0., 320., 240., 0., 0., 1.],
            "R": [1., 0., 0., 0., 1., 0., 0., 0., 1.], "D": [0.] * 5,
            "P": [320., 0., 320., -38.4 if side == "right" else 0.,
                  0., 320., 240., 0., 0., 0., 1., 0.]}


class LocalStereoBridge:
    supports_stereo_localization = True

    def __init__(self):
        self.flight_requests, self.camera_reads, self.localizations = [], [], []
        self.pairs, self.counter, self.mission_count = {}, 0, 0
        self.telemetry = {"ready": True, "simulation_only": True, "connected": True,
            "armed": False, "landed_state": 1, "position_enu_m": [0., 0., 0.], "home_enu_m": [0., 0., 0.],
            "scene": {"verified": True, "world_sha256": "c" * 64,
                      "navigation": {"flight_altitude_m": 3.5, "bounds_xy_m": [90., 90., 120., 120.]}},
            "navigation": {"enabled": True, "active": False, "map": None},
            "vio": {"ready": True}, "camera": {s: {"available": True} for s in ("left", "right")},
            "external_vision": {"required": True, "healthy": True, "used_for_flight_control": True,
                "adapter": {"epoch": 0, "mode": "fuse", "ready": True, "forwarding": True, "sitl_verified": True},
                "ekf": {"ready": True, "fusion_active": True,
                        "local_position_reset_counters": {"xy": 1, "z": 2, "heading": 3}}},
            "mission": None}

    def state(self):
        return copy.deepcopy(self.telemetry)

    def request(self, path, payload=None):
        if path not in ("/mission", "/land") or payload is None:
            raise AssertionError("Unscripted or network-like Bridge request: " + path)
        steps = copy.deepcopy(payload.get("steps", [])) if path == "/mission" else [{"action": "land"}]
        self.flight_requests.append({"path": path, "steps": steps, "description": payload.get("description")})
        self.mission_count += 1
        for step in steps:
            action = step["action"]
            if action == "takeoff":
                if self.telemetry["armed"]:
                    raise AssertionError("Duplicate local takeoff")
                self.telemetry.update(armed=True, landed_state=2, position_enu_m=[0., 0., step["altitude"]])
                self.telemetry["navigation"] = {"enabled": True, "active": True, "map": {
                    "source": "horizontal_lidar_aligned_enu", "fresh": True, "scan_count": 5,
                    "bounds_xy_enu_m": [-2.4, -7.4, 15.4, 7.4], "map_altitude_enu_m": 3.5,
                    "obstacle_inflation_m": 1., "visible_clearance_m": 1.2}}
            elif action == "goto":
                self.telemetry["position_enu_m"] = [step["x"], step["y"], step["altitude"]]
            elif action == "return_home":
                self.telemetry["position_enu_m"] = [0., 0., self.telemetry["position_enu_m"][2]]
            elif action == "land":
                self.telemetry.update(armed=False, landed_state=1, position_enu_m=[0., 0., 0.])
            else:
                raise AssertionError("Unscripted local action " + action)
        self.telemetry["mission"] = {"id": "local-flight-{}".format(self.mission_count), "status": "running",
                                    "step": len(steps), "steps": steps, "events": []}
        return {"mission": copy.deepcopy(self.telemetry["mission"])}

    def wait_for(self, accepted, on_progress=None):
        if accepted["mission"]["id"] != self.telemetry["mission"]["id"]:
            raise AssertionError("Local mission replaced")
        self.telemetry["mission"]["status"] = "succeeded"
        return self.state()

    def capture_stereo(self):
        from stereo_localization import validate_pair
        self.counter += 1
        stamp = 42. + self.counter
        identifier = "local-pair-{}".format(self.counter)
        body = {"frame_id": "map", "body_reference": "base_link", "position_m": list(self.telemetry["position_enu_m"]),
                "orientation_xyzw": [0., 0., 0., 1.], "stamp_sim_s": stamp, "age_seconds": .1}
        pair = {"status": "ok", "synchronized": True, "pair_id": identifier, "stamp_sim_s": stamp,
                "now_sim_s": stamp + .02, "baseline_m": .12, "body_pose_enu": body,
                "external_vision": copy.deepcopy(self.telemetry["external_vision"]), "image_sha256": {}}
        original = base64.b64decode(JPEG_B64)
        for side in ("left", "right"):
            # JPEG comments preserve valid pixels and dimensions while giving
            # each independently received fixture frame unique byte provenance.
            comment = "{}:{}".format(identifier, side).encode()
            raw = original[:2] + b"\xff\xfe" + struct.pack(">H", len(comment)+2) + comment + original[2:]
            pair["image_sha256"][side] = hashlib.sha256(raw).hexdigest()
            pair[side] = {"status": "ok", "side": side, "mime_type": "image/jpeg", "width": 640, "height": 480,
                          "stamp_sim_s": stamp, "age_seconds": .1, "measurement_age_seconds": .02,
                          "image_frame_id": "uav_stereo_optical", "camera_intrinsics": calibration(stamp, side),
                          "image_quality": copy.deepcopy(QUALITY), "image_base64": base64.b64encode(raw).decode()}
        validate_pair(pair, pair["now_sim_s"])
        self.pairs[identifier] = copy.deepcopy(pair)
        self.camera_reads.append(identifier)
        return pair

    def capture_camera(self, side="left"):
        raise AssertionError("Production capture bypassed the stereo transport")

    def localize_stereo(self, pair_id, bboxes):
        if pair_id not in self.pairs:
            raise AssertionError("Localization of an unknown pair")
        pair = self.pairs[pair_id]
        self.localizations.append((pair_id, copy.deepcopy(bboxes)))
        # Geometry is deliberately a fake transport result: this test proves
        # provenance/bookkeeping wiring, not the CV algorithm or target identity.
        metric = {"status": "ok", "method": "stereo_sgbm_robust_surface", "frame": "local_enu",
                  "position_enu_m": [4., 1., 0.], "covariance_enu_m2": [[.01, 0., 0.], [0., .01, 0.], [0., 0., .01]],
                  "sigma_enu_m": [.1, .1, .1], "image_stamp_sim_s": pair["stamp_sim_s"],
                  "body_pose_stamp_sim_s": pair["body_pose_enu"]["stamp_sim_s"],
                  "image_sha256": copy.deepcopy(pair["image_sha256"]),
                  "coordinate_epoch": search_tools.coordinate_epoch(pair["external_vision"]),
                  "capture_pair_id": pair_id, "historical_observation": True, "usable_as_flight_waypoint": False,
                  "uncertainty_statistics_calibrated": False}
        return {"status": "ok", "pair_id": pair_id, "targets": [copy.deepcopy(metric) for _ in bboxes],
                "historical_observation": True, "report_delay_wall_s": .01}


def flight_call(steps, identifier):
    return tool_call("execute_mission", {"steps": steps, "description": "offline graph test"}, identifier)


def next_view():
    return tool_call("search_observation_status", {"view": "next"}, "next-view")


def goto_next(messages):
    point = tool_payload(messages, "search_observation_status")["next_view"]["position_enu_m"]
    return flight_call([{"action": "goto", "x": point[0], "y": point[1], "altitude": point[2]}], "goto-view")


def capture():
    return tool_call("capture_scene", {"query": QUERY, "side": "left"}, "capture-view")


def report(messages, invalid=False):
    capture_id = tool_payload(messages, "capture_scene")["capture_id"]
    return tool_call("report_scene", {"capture_id": capture_id, "summary": "本地脚本观察报告",
        "targets": [{"label": QUERY, "confidence": .8, "bbox": [.6, .4, .4, .6] if invalid else [.4, .6, .6, .8],
                     "surroundings": ["仅测试夹具"], "position_basis": "image_only"}],
        "limitations": ["离线脚本与合成相机夹具，不代表真实目标识别"]}, "report-view")


def report_without_target(messages):
    capture_id = tool_payload(messages, "capture_scene")["capture_id"]
    return tool_call("report_scene", {"capture_id": capture_id, "summary": "本次图像未确认蓝色桶",
        "targets": [], "limitations": ["仅离线图路径夹具，不证明目标不存在"]}, "report-view")


def prefix():
    return [tool_call("get_drone_state", {"view": "all"}, "state"),
            flight_call([{"action": "takeoff", "altitude": 3.5}], "takeoff"),
            tool_call("plan_region_search", {"query": QUERY, "bounds": [3., 0., 5., 2.],
                       "spacing": 1., "max_observations": 4}, "plan"),
            next_view(), goto_next, capture()]


def land():
    return tool_call("land_drone", {"reason": "user-authorized final safety landing"}, "land")


class ProductionSearchGraphTests(unittest.TestCase):
    def test_synthetic_stereo_fixture_is_actual_640x480_with_same_stamp_projection_and_hashes(self):
        fake = LocalStereoBridge()
        pair = fake.capture_stereo()
        for side in ("left", "right"):
            raw = base64.b64decode(pair[side]["image_base64"])
            marker = raw.index(b"\xff\xc0")
            height, width = struct.unpack(">HH", raw[marker+5:marker+9])
            self.assertEqual((width, height), (640, 480))
            self.assertEqual(hashlib.sha256(raw).hexdigest(), pair["image_sha256"][side])
            self.assertEqual(pair[side]["stamp_sim_s"], pair[side]["camera_intrinsics"]["stamp_sim_s"])
        self.assertEqual(pair["left"]["stamp_sim_s"], pair["right"]["stamp_sim_s"])
        self.assertEqual(pair["right"]["camera_intrinsics"]["P"][3], -38.4)
        self.assertGreater(pair["left"]["image_quality"]["laplacian_variance"], 20.)

    def application(self, script, reject_saved_report=False):
        fake = LocalStereoBridge()
        model = LocalScriptModel(script=list(script))
        holder, journal = {}, []
        real_builder = search_tools.build_search_tools

        def search_builder(client, manager=None):
            tools, manager = real_builder(client, manager)
            holder["manager"] = manager
            consume = manager.consume_report

            def checked_consume(owner, frame, result):
                # The production web audit must have persisted this exact base
                # report before its real manager can count the observation.
                command = client.application.current_command
                saved = [item for item in command["perceptions"]
                         if item.get("capture_id") == frame["capture_id"] and item.get("status") == "ok"]
                if not saved or ("saved", frame["capture_id"]) not in journal:
                    raise AssertionError("Search bookkeeping preceded successful base-report save")
                journal.append(("consume", frame["capture_id"]))
                return consume(owner, frame, result)
            manager.consume_report = checked_consume
            return tools, manager

        def factory(client, config):
            original_record = client.record
            def record(name, result):
                if name == "inspect_scene" and result.get("status") == "ok" and reject_saved_report:
                    result = {"status": "error", "error": "local persistence rejection",
                              "capture_id": result["capture_id"], "targets": [], "limitations": []}
                returned = original_record(name, result)
                if name == "inspect_scene" and returned.get("status") == "ok":
                    journal.append(("saved", returned["capture_id"]))
                return returned
            client.record = record
            return single_vlm.build_single_vlm_agent(client, config, agent.flight_tools(client),
                                                    "Use only the explicitly requested bounded region.", model=model)

        with mock.patch.object(search_tools, "build_search_tools", side_effect=search_builder), \
                mock.patch.object(single_vlm, "ChatOpenAI", side_effect=AssertionError("External model forbidden")):
            app = WebApplication(fake, CONFIG, factory, vision_config=CONFIG, agent_architecture="single_vlm")
        return app, fake, model, holder, journal

    def run_local(self, script, reject_saved_report=False):
        app, fake, model, holder, journal = self.application(script, reject_saved_report)
        # Also enforce the no-network and no-optional-file-evidence boundary
        # during the real production graph invocation.
        with mock.patch("socket.create_connection", side_effect=AssertionError("Network forbidden")), \
                mock.patch.dict(os.environ, {"UAV_CAPTURE_EVIDENCE_DIR": ""}), \
                contextlib.redirect_stdout(io.StringIO()):
            accepted = app.submit(PROMPT, mode="task", min_observations=1)
            command = completed(app, accepted["id"])
        return command, app, fake, model, holder["manager"], journal

    def assert_private(self, value):
        serialized = encoded(value)
        for forbidden in (CONFIG["api_key"], "image_base64", "data:image/jpeg;base64,"):
            self.assertNotIn(forbidden, serialized)

    def test_complete_two_view_graph_runs_real_manager_and_public_web_search_audit(self):
        script = prefix() + [report, next_view(), goto_next, capture(), report,
            tool_call("search_observation_status", {"view": "summary"}, "summary"),
            flight_call([{"action": "return_home"}], "return"), land(), AIMessage("观察报告已保存，已返航降落。")]
        command, app, fake, model, manager, journal = self.run_local(script)
        self.assertEqual(command["status"], "succeeded", command.get("error"))
        self.assertEqual([s["action"] for request in fake.flight_requests for s in request["steps"]],
                         ["takeoff", "goto", "goto", "return_home", "land"])
        self.assertEqual(len(command["perceptions"]), 2)  # No duplicate base-report save.
        self.assertEqual(command["search_plan"]["plan"]["bounds_enu_m"], [3., 0., 5., 2.])
        permission = command["search_plan"]["plan"]["navigation_permission"]
        self.assertEqual(permission["navigation_map_bounds_enu_m"], [-2.4, -7.4, 15.4, 7.4])
        self.assertEqual(permission["navigation_goal_boundary_margin_m"], 1.2)
        self.assertEqual(len(command["search_plan"]["plan"]["views"]), 4)
        self.assertTrue(all(v["position_enu_m"][0] < 3. for v in command["search_plan"]["plan"]["views"]))
        events = command["search_events"]
        self.assertIn("plan_region_search", [event["tool"] for event in events])
        progress = [event["result"] for event in events if event["tool"] == "search_progress"]
        self.assertEqual([item["search"]["observed_views"] for item in progress], [1, 2])
        self.assertEqual(progress[-1]["search"]["status"], "target_found")
        self.assertFalse(progress[-1]["search"]["occlusion_free_coverage_proven"])
        self.assertEqual(progress[-1]["search"]["targets"][0]["distinct_viewpoint_count"], 2)
        self.assertTrue(progress[-1]["search"]["targets"][0]["requested_region_membership"]["confirmed_inside"])
        self.assertFalse(progress[-1]["search"]["targets"][0]["generated_flight_goal"])
        ids = [report["capture_id"] for report in command["perceptions"]]
        self.assertEqual(journal, [("saved", ids[0]), ("consume", ids[0]), ("saved", ids[1]), ("consume", ids[1])])
        self.assertEqual(len(fake.camera_reads), 3)  # One actual calibration read + two captures.
        self.assertEqual(len(fake.localizations), 2)
        for report_value, progress_value in zip(command["perceptions"], progress):
            metric = report_value["targets"][0]["stereo_location"]
            observation = progress_value["search"]["observations"][-1]
            self.assertEqual(metric["coordinate_epoch"], observation["coordinate_epoch"])
            self.assertEqual(metric["image_sha256"], observation["image_sha256"])
            self.assertEqual(report_value["camera"]["image_sha256"], metric["image_sha256"]["left"])
            self.assertTrue(metric["historical_observation"])
            self.assertFalse(metric["usable_as_flight_waypoint"])
        self.assertEqual(sum(bool(image_urls(messages)) for messages in model.calls), 2)
        self.assertFalse(fake.state()["armed"])
        self.assertEqual(fake.state()["landed_state"], 1)
        self.assertEqual(len(manager.sessions), 1)
        self.assert_private(command)
        self.assert_private(app.state())
        self.assert_private(app.agent.get_state({"configurable": {"thread_id": app.agent_thread_id}}).values)

    def test_invalid_or_unsaved_report_never_counts_view_and_old_goto_is_blocked_but_land_allowed(self):
        for failure in ("bbox", "save"):
            with self.subTest(failure=failure):
                script = prefix() + [lambda messages: report(messages, invalid=failure == "bbox"),
                    # Deliberately try the old known first view after failure.
                    goto_next,
                    land(), AIMessage("报告失败，已停止移动并完成授权降落。")]
                command, app, fake, model, manager, journal = self.run_local(script, reject_saved_report=failure == "save")
                self.assertEqual(command["status"], "failed")
                self.assertEqual([s["action"] for request in fake.flight_requests for s in request["steps"]],
                                 ["takeoff", "goto", "land"])
                self.assertEqual(journal, [])
                session = next(iter(manager.sessions.values()))
                summary = session["search"].summary()
                self.assertEqual(summary["status"], "observation_failed")
                self.assertEqual(summary["observed_views"], 0)
                self.assertEqual(summary["targets"], [])
                self.assertFalse(any(event["tool"] == "search_progress" for event in command.get("search_events", [])))
                self.assertTrue(any(item["tool"] == "execute_mission" for item in command["tool_errors"]))
                self.assertFalse(fake.state()["armed"])
                self.assertEqual(fake.state()["landed_state"], 1)
                self.assert_private(command)

    def test_four_view_budget_without_confirmed_target_completes_sampling_without_inventing_found(self):
        script = prefix() + [report_without_target]
        for _ in range(3):
            script.extend([next_view(), goto_next, capture(), report_without_target])
        script.extend([tool_call("search_observation_status", {"view": "summary"}, "summary"),
                       flight_call([{"action": "return_home"}], "return"), land(),
                       AIMessage("四点观察预算完成，未确认蓝色桶；已返航降落，不能据此声称目标不存在。")])
        command, app, fake, model, manager, journal = self.run_local(script)
        self.assertEqual(command["status"], "succeeded", command.get("error"))
        progress = [event["result"]["search"] for event in command["search_events"] if event["tool"] == "search_progress"]
        self.assertEqual([item["observed_views"] for item in progress], [1, 2, 3, 4])
        self.assertEqual(progress[-1]["status"], "sampling_complete")
        self.assertEqual(progress[-1]["targets"], [])
        self.assertFalse(progress[-1]["occlusion_free_coverage_proven"])
        self.assertEqual(len(command["perceptions"]), 4)
        self.assertEqual([item[0] for item in journal], ["saved", "consume"]*4)
        planned = command["search_plan"]["plan"]
        from coverage_search import potential_visible_samples
        for view in planned["views"]:
            self.assertIn(view["id"], potential_visible_samples(planned, view["planned_pose_enu"]))
        self.assertEqual([step["action"] for request in fake.flight_requests for step in request["steps"]],
                         ["takeoff", "goto", "goto", "goto", "goto", "return_home", "land"])
        self.assert_private(command)

    def test_actual_ekf_counter_reset_after_report_blocks_next_old_coordinate_point_and_allows_land(self):
        holder = {}
        def reset_then_goto(messages):
            fake = holder["fake"]
            fake.telemetry["external_vision"]["ekf"]["local_position_reset_counters"]["xy"] += 1
            return goto_next(messages)
        script = prefix() + [report, next_view(), reset_then_goto, land(), AIMessage("坐标已重置，停止搜索并完成降落。")]
        app, fake, model, search_holder, journal = self.application(script)
        holder["fake"] = fake
        with mock.patch("socket.create_connection", side_effect=AssertionError("Network forbidden")), \
                mock.patch.dict(os.environ, {"UAV_CAPTURE_EVIDENCE_DIR": ""}), \
                contextlib.redirect_stdout(io.StringIO()):
            command = completed(app, app.submit(PROMPT, mode="task", min_observations=1)["id"])
        self.assertEqual(command["status"], "failed")
        self.assertEqual([s["action"] for request in fake.flight_requests for s in request["steps"]],
                         ["takeoff", "goto", "land"])
        manager = search_holder["manager"]
        session = next(iter(manager.sessions.values()))
        self.assertEqual(session["search"].summary()["observed_views"], 1)
        self.assertEqual(session["search"].status, "observation_failed")
        self.assertEqual(session["coordinate_epoch"]["vio_epoch"], fake.telemetry["external_vision"]["adapter"]["epoch"])
        self.assertNotEqual(session["coordinate_epoch"], search_tools.coordinate_epoch(fake.state()))
        self.assertEqual(len(journal), 2)
        self.assertFalse(fake.state()["armed"])
        self.assertEqual(fake.state()["landed_state"], 1)
        self.assertTrue(any("epoch" in item["error"] for item in command["tool_errors"]))
        self.assert_private(command)


if __name__ == "__main__":
    unittest.main()
