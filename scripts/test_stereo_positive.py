#!/usr/bin/env python3
"""One bounded VLM-selected-object stereo test; request-free unless --execute.

Ground truth never enters the prompt, flight plan or metric acceptance. Passing
means a real model-selected box produced a quality-accepted historical surface
measurement, not independently verified object identity or localization accuracy.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import test_stereo_search as bounded
from scripts.evaluate_scene_boxes import jpeg_dimensions
from scripts.test_realistic_scene import collect_perceptions, verify_return_home
from stereo_localization import (calibration_from_camera_info, configured_limits, covariance_matrix, finite,
                                  optical_to_enu, triangulate_pixel, vector)

MODEL = "qwen3-vl-plus"
COMMON = ("用左目采集一次新的机载图像，让同一个视觉模型从实际画面自行选择一个清晰可见的静态实体，"
          "用report_scene保存一份报告：给出可见类别、完整目标的归一化bbox和周围实际可见物体；不要用手写框、场景真值或猜测坐标。"
          "系统按这个模型目标框计算双目位置，说明其为可见表面点及质量限制。看不到实体或双目质量不足时如实报告，不能编造位置，不能重拍重试。"
          "报告保存后return_home返回本次记录的起飞点，随后降落并等待落地上锁。不要根据测得的目标坐标继续飞行，不搜索其他区域。"
          "每次execute_mission必须提供steps和description。")
PRESETS = {
    "hover": {"point": None, "prompt": "先读取实际无人机状态，起飞至ENU高度3.5米，在本次起飞点上方保持位置，不进行水平移动。"+COMMON},
    "single_view": {"point": [1., 0., 3.5], "prompt": "先读取实际无人机状态，起飞至ENU高度3.5米，用真实激光雷达导航至唯一许可观察点(x=1,y=0,z=3.5)，到点后保持位置。不得自行添加中间航点，没有安全路线时停止并完成降落。"+COMMON},
}


def check_live(command, preset):
    """Stop on an accepted out-of-scope action; never authorize from image XYZ."""
    approved = PRESETS[preset]["point"]
    counts = {}
    for mission in bounded.task_missions(command).values():
        for step in mission["steps"]:
            action = step.get("action")
            counts[action] = counts.get(action,0)+1
            bounded.require(action in ("takeoff", "goto", "hold", "return_home", "land"), "Unexpected positive-test flight action")
            if action == "takeoff":
                bounded.require(abs(finite(step.get("altitude"), "takeoff altitude")-3.5) <= 1e-6,
                                "Positive test changed the fixed flight altitude")
            elif action == "goto":
                bounded.require(approved is not None and bounded.near_point(
                    [step.get("x"), step.get("y"), step.get("altitude")], approved, .02),
                    "Positive test used an unapproved or measured-target waypoint")
            elif action == "hold":
                bounded.require(0 < finite(step.get("duration"), "hold duration") <= 2., "Positive test exceeded its bounded hold")
    bounded.require(all(count <= 1 for count in counts.values()), "Positive test repeated a flight stage without authorization")
    bounded.require(not command.get("search_plan"), "Positive one-view test unexpectedly started a region search")


def paired_archive_contract(metadata, identifier):
    """Positive runs require the new exact-stamp pair, legacy mono may omit it."""
    pair = metadata.get("stereo_evidence") or {}
    snapshot = metadata["snapshot"]
    bounded.require(pair.get("schema_version") == 1 and pair.get("synchronized") is True
                    and pair.get("model_input_side") == "left" and pair.get("ground_truth_included") is False,
                    "Positive localization requires an actual archived left/right JPEG pair")
    stamp = finite(snapshot["stamp_sim_s"],"archived image stamp")
    hashes = snapshot["stereo_image_sha256"]
    bounded.require(pair.get("pair_id") == snapshot["stereo_pair_id"] and pair.get("stamp_sim_s") == stamp
                    and pair.get("image_sha256") == hashes and pair.get("coordinate_epoch") == snapshot["coordinate_epoch"]
                    and pair.get("body_pose_enu") == snapshot["body_pose_enu"], "Archived pair pose/epoch/hash provenance differs")
    for side in ('left','right'):
        member=pair.get(side) or {};info=member.get("camera_intrinsics") or {}
        bounded.require(member.get("side") == side and member.get("mime_type") == "image/jpeg"
                        and (member.get("width"),member.get("height")) == (640,480)
                        and member.get("stamp_sim_s") == stamp and info.get("stamp_sim_s") == stamp
                        and member.get("image_sha256") == hashes.get(side)
                        and member.get("jpeg_filename") == identifier+(".jpg" if side=='left' else ".right.jpg"),
                        "Archived stereo member has mismatched source stamp, file, hash or CameraInfo")
    bounded.require(pair['left']['camera_intrinsics'] == snapshot['camera_intrinsics'],
                    "Archived left CameraInfo differs from the actual model-input snapshot")
    calibration=calibration_from_camera_info(pair['left']['camera_intrinsics'],pair['right']['camera_intrinsics'],.12)
    bounded.require(abs(finite(pair['baseline_m'],'archive baseline')-calibration['baseline_m']) < 1e-9,
                    'Archived actual baseline differs from the known camera calibration')
    return pair


def archive_captures(command, owner, output):
    owner.check()
    directory = Path(owner.manager["run_directory"]) / "captures"
    reports = {}; collect_perceptions(command, reports)
    destination = output / "captures"; destination.mkdir(exist_ok=False)
    retained = {}
    for identifier, report in reports.items():
        bounded.require(isinstance(identifier, str) and re.fullmatch(r"[a-f0-9]{32}", identifier), "Unsafe actual capture identifier")
        raw = (directory / (identifier+".jpg")).read_bytes()
        metadata_bytes = (directory / (identifier+".json")).read_bytes()
        metadata = json.loads(metadata_bytes)
        bounded.require(jpeg_dimensions(raw) == (640,480)
                        and (metadata["camera"].get("width"),metadata["camera"].get("height")) == (640,480),
                        "Actual model-input JPEG does not match the calibrated image dimensions")
        bounded.require(metadata.get("capture_id") == identifier and metadata["camera"]["image_sha256"] ==
                        hashlib.sha256(raw).hexdigest(), "Archived image is not the actual model-input JPEG")
        bounded.require("image_base64" not in metadata_bytes.decode(), "Image bytes unexpectedly leaked into metadata")
        pair = paired_archive_contract(metadata,identifier)
        right_raw = (directory / (identifier+'.right.jpg')).read_bytes()
        hashes = {"left":hashlib.sha256(raw).hexdigest(),"right":hashlib.sha256(right_raw).hexdigest()}
        bounded.require(0 < len(raw) <= 2*1024*1024 and 0 < len(right_raw) <= 2*1024*1024
                        and jpeg_dimensions(right_raw) == (640,480) and hashes == pair['image_sha256'],
                        "Archived actual right JPEG bytes do not match the exact source pair")
        for suffix, data in ((".jpg", raw), (".right.jpg",right_raw), (".json", metadata_bytes)):
            with (destination / (identifier+suffix)).open("xb") as handle: handle.write(data)
        metadata['verified_archive_image_sha256']=hashes
        retained[identifier] = metadata
    owner.check()
    return retained


def audit_positive(command, initial, final, samples, missions, model, preset, metadata):
    bounded.require(command.get("status") == "succeeded", command.get("error") or "Natural-language task failed")
    bounded.require(final.get("armed") is False and final.get("landed_state") == 1 and final.get("ready") is True,
                    "Positive test did not finish landed/disarmed")
    check_live(command, preset)
    calls = command.get("model_calls") or []
    bounded.require(calls and all(c.get("status") == "ok" and c.get("model") == model for c in calls),
                    "A real VLM call failed or changed model")
    sequence = [t.get("name") for c in calls for t in c.get("called_tools", [])]
    bounded.require("get_drone_state" in sequence and sequence.count("capture_scene") == sequence.count("report_scene") == 1,
                    "Expected state read and exactly one actual capture/report")
    reports = {}; collect_perceptions(command, reports)
    bounded.require(len(reports) == 1 and set(reports) == set(metadata), "One owned historical image report is required")
    report = next(iter(reports.values())); capture_id = report["capture_id"]
    frame = metadata[capture_id]; snapshot = frame["snapshot"]; stamp = finite(report["captured_at_sim_s"], "image stamp")
    pair = paired_archive_contract(frame,capture_id)
    bounded.require(frame.get('verified_archive_image_sha256') == pair['image_sha256'],
                    "Positive localization lacks independently checked archived stereo JPEG bytes")
    bounded.require((frame["camera"].get("width"),frame["camera"].get("height")) == (640,480),
                    "Archived model image dimensions do not match stereo calibration")
    bounded.require(report.get("status") == "ok" and report.get("architecture") == "single_vlm" and report.get("model") == model,
                    "Structured perception failed or used another architecture")
    bounded.require(any(capture_id in c.get("capture_ids", []) and "image" in c.get("input_modalities", [])
                        and any(t.get("name") == "report_scene" for t in c.get("called_tools", [])) for c in calls),
                    "No real model-input image informed this structured report")
    bounded.require(abs(stamp-finite(snapshot["stamp_sim_s"], "archived stamp")) <= 1e-6 and
                    report["camera"]["image_sha256"] == frame["camera"]["image_sha256"], "Historical image provenance differs")
    body = snapshot["body_pose_enu"]; observed = vector(body["position_m"], 3, "actual observer")
    home = vector(initial["position_enu_m"], 3, "initial home")
    approved = PRESETS[preset]["point"] or [home[0], home[1], 3.5]
    bounded.require(bounded.near_point(observed, approved, .65), "Model image came from an unapproved vehicle position")
    flown = [m for m in missions.values() if m.get("id") != (initial.get("mission") or {}).get("id")]
    bounded.require(flown and all(m.get("status") == "succeeded" for m in flown), "A flight stage failed")
    events = [e for m in flown for e in m.get("events", [])]
    bounded.require(sum(e.get("action") == "takeoff" for e in events) == 1 and sum(e.get("action") == "land" for e in events) == 1,
                    "Expected one completed takeoff and landing")
    bounded.require(any(e.get("action") == "takeoff" and e.get("finished_at", math.inf) <= report["created_at"]+.1 for e in events),
                    "Image preceded completed takeoff")
    if PRESETS[preset]["point"]:
        bounded.require(any(e.get("action") == "goto" and e.get("finished_at", math.inf) <= report["created_at"]+.1
                            and bounded.near_point(e.get("position_enu_m"), approved, .65) for e in events), "Image preceded successful approved goto")
    nearest = min(samples, key=lambda s: abs(s["wall_time"]-report["created_at"])) if samples else {}
    bounded.require(nearest and abs(nearest["wall_time"]-report["created_at"]) <= 1.5
                    and nearest["state"].get("armed") is True
                    and bounded.near_point(nearest["state"].get("position_enu_m"), observed, .85), "Missing aligned airborne telemetry")
    limits, results = configured_limits(), []
    unavailable = []
    hashes = snapshot["stereo_image_sha256"]
    epoch = snapshot.get("coordinate_epoch")
    bounded.require(isinstance(epoch,dict) and type(epoch.get("vio_epoch")) is int and epoch["vio_epoch"] >= 0
                    and isinstance(epoch.get("ekf_counters"),dict) and set(epoch["ekf_counters"]) == {"xy","z","heading"}
                    and all(type(v) is int and 0 <= v <= 255 for v in epoch["ekf_counters"].values()),
                    "Actual fused image lacks full coordinate epoch evidence")
    bounded.require(abs(finite(body["stamp_sim_s"],"pose stamp")-stamp) <= .05,
                    "Actual image and historical body pose are not time aligned")
    bounded.require(all(re.fullmatch(r"[a-f0-9]{64}", hashes.get(side, "")) for side in ("left", "right"))
                    and hashes["left"] == frame["camera"]["image_sha256"], "Actual stereo source hashes are missing")
    for index, target in enumerate(report.get("targets") or []):
        metric = target.get("stereo_location") or {}
        if metric.get("status") == "unavailable":
            bounded.require(metric.get("reason") and "position_enu_m" not in metric, "Unavailable stereo has invented coordinates")
            unavailable.append({"index": index, "label": target.get("label"), "reason": metric["reason"]});continue
        bounded.require(metric.get("status") == "ok" and metric.get("method") == "stereo_sgbm_robust_surface"
                        and metric.get("historical_observation") is True and metric.get("usable_as_flight_waypoint") is False
                        and metric.get("uncertainty_statistics_calibrated") is False and metric.get("frame") == "local_enu"
                        and metric.get("image_sha256") == hashes and metric.get("capture_pair_id") == snapshot["stereo_pair_id"]
                        and metric.get("coordinate_epoch") == snapshot["coordinate_epoch"]
                        and abs(finite(metric["image_stamp_sim_s"], "metric stamp")-stamp) <= 1e-6,
                        "Positive stereo lacks same-frame historical provenance")
        box = vector(target["bbox"], 4, "model-selected bbox")
        bounded.require(0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1, "Invalid actual VLM bbox")
        quality = metric["quality"]
        for count in ("roi_pixels", "raw_valid_pixels", "lr_valid_pixels", "robust_inlier_pixels"):
            bounded.require(type(quality.get(count)) is int and quality[count] > 0, "Missing actual stereo pixel counts")
        bounded.require(quality["robust_inlier_pixels"] <= quality["lr_valid_pixels"] <= quality["raw_valid_pixels"] <= quality["roi_pixels"]
                        and abs(quality["valid_ratio"]-quality["lr_valid_pixels"]/quality["roi_pixels"]) < 1e-7
                        and abs(quality["lr_consistent_ratio"]-quality["lr_valid_pixels"]/quality["raw_valid_pixels"]) < 1e-7,
                        "Stereo quality counts and ratios disagree")
        bounded.require(quality["robust_inlier_pixels"] >= limits["min_valid_pixels"]
                        and quality["robust_inlier_pixels"]/quality["roi_pixels"] >= limits["min_valid_ratio"]
                        and finite(quality["valid_ratio"], "valid ratio") >= limits["min_valid_ratio"]
                        and finite(quality["lr_consistent_ratio"], "LR ratio") >= limits["min_lr_consistent_ratio"]
                        and finite(quality["texture_laplacian_variance"], "texture") >= limits["min_texture_variance"],
                        "Positive metric lacks accepted real stereo quality")
        calibration = metric["calibration"]
        k = vector(snapshot["camera_intrinsics"]["K"], 9, "same-frame K")
        bounded.require(abs(finite(calibration["baseline_m"], "baseline")-.12) < 1e-9
                        and all(abs(calibration[key]-value) < 1e-9 for key,value in (("fx",k[0]),("fy",k[4]),("cx",k[2]),("cy",k[5]))),
                        "Metric calibration differs from actual camera")
        uv = vector(quality["representative_pixel_uv"], 2, "representative pixel")
        bounded.require(box[0]*640 <= uv[0] <= box[2]*640 and box[1]*480 <= uv[1] <= box[3]*480,
                        "Stereo representative pixel is outside the model-selected box")
        optical, optical_cov = triangulate_pixel(uv[0], uv[1], quality["median_disparity_px"], calibration,
                                                 sigma_disparity_px=quality["sigma_disparity_px"])
        spread = 1.4826*finite(quality["disparity_mad_px"],"disparity MAD")
        depth_sigma = math.sqrt(optical_cov[2][2])
        bounded.require(0 <= spread/quality["median_disparity_px"] <= limits["max_relative_disparity_spread"]
                        and abs(quality["sigma_disparity_px"]-max(.5,spread)) < 1e-7
                        and abs(finite(quality["depth_sigma_m"],"depth sigma")-depth_sigma) < 1e-7
                        and depth_sigma <= limits["max_depth_sigma_m"]
                        and depth_sigma/optical[2] <= limits["max_relative_depth_sigma"],
                        "Positive stereo exceeds its depth/spread uncertainty gate")
        enu, covariance, _assumed = optical_to_enu(optical, optical_cov, body)
        bounded.require(limits["min_depth_m"] <= optical[2] <= limits["max_depth_m"]
                        and bounded.near_point(metric["position_camera_optical_m"], optical, 1e-6)
                        and bounded.near_point(metric["position_enu_m"], enu, 1e-6), "Metric geometry does not agree with stereo and pose")
        actual_cov = covariance_matrix(metric["covariance_enu_m2"], "ENU covariance")
        sigma = vector(metric["sigma_enu_m"], 3, "ENU sigma")
        bounded.require(all(abs(actual_cov[i][j]-covariance[i][j]) <= 1e-7 for i in range(3) for j in range(3))
                        and all(abs(sigma[i]**2-actual_cov[i][i]) <= 1e-7 for i in range(3)), "Metric covariance differs from actual transform")
        results.append({"target_index": index, "label": target.get("label"), "actual_vlm_bbox": box,
                        "surface_position_enu_m": enu, "covariance_enu_m2": actual_cov, "quality": copy.deepcopy(quality)})
    bounded.require(results, "No model-selected target produced a quality-accepted stereo surface point; positive localization NOT passed")
    returned = verify_return_home(flown, list(reports.values()), samples, home)
    bounded.require(math.hypot(final["position_enu_m"][0]-home[0], final["position_enu_m"][1]-home[1]) < .6, "Landing was not near home")
    return {"passed": True, "positive_stereo_surface_contract_passed": True, "capture_id": capture_id,
            "metric_results": results, "unavailable_targets": unavailable, "returned_via": returned,
            "paired_jpeg_archive_checked":True,"archived_stereo_image_sha256":copy.deepcopy(pair['image_sha256']),
            "object_identity_ground_truth_verified": False, "metric_accuracy_ground_truth_verified": False,
            "uncertainty_statistics_calibrated": False, "target_used_as_flight_goal": False,
            "limitations": ["A model-selected bbox can include background; saved image requires independent review.",
                            "Quality-accepted surface geometry is not object-center accuracy or a ground-truth identification."]}


def execute(base, output, model=MODEL, preset="hover", timeout=600., interval=.5, **kwargs):
    owners = []
    factory = kwargs.pop("ownership_factory", bounded.LocalSimulationOwner)
    def own(*args):
        owner = factory(*args);owners.append(owner);return owner
    def validate(command, initial, final, samples, missions, used_model):
        bounded.require(command.get("status") == "succeeded", command.get("error") or "Natural-language task failed")
        evidence = archive_captures(command, owners[0], output)
        return audit_positive(command, initial, final, samples, missions, used_model, preset, evidence)
    return bounded.execute(base, output, model, timeout, interval, ownership_factory=own,
                           task_prompt=PRESETS[preset]["prompt"], task_validator=validate,
                           live_validator=lambda command: check_live(command,preset), max_reports=1, **kwargs)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preset", choices=tuple(PRESETS), default="hover")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--base-url", default=os.getenv("UAV_WEB_URL","http://127.0.0.1:8080"))
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--timeout", type=float, default=600.)
    parser.add_argument("--px4-directory", type=Path, default=Path(os.getenv("PX4_DIR") or "/home/dut/PX4-Autopilot"))
    args=parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 1200:parser.error("timeout must be 1..1200 wall seconds")
    if not args.execute:
        print(json.dumps({"preset":args.preset, "prompt":PRESETS[args.preset]["prompt"], "approved_observation_point":PRESETS[args.preset]["point"],
                          "max_reports":1, "actual_requests":False, "actual_model_call":False, "actual_flight":False,
                          "ground_truth_in_agent":False, "metric_required_for_positive_pass":True},ensure_ascii=False,indent=2));return 0
    if args.output is None:parser.error("--execute requires a new exclusive --output directory")
    result=execute(args.base_url.rstrip('/'),args.output,args.model,args.preset,args.timeout,px4_directory=args.px4_directory)
    print(json.dumps({"output":str(args.output),"passed":result["validation_passed"],"error":result.get("error")},ensure_ascii=False))
    return 0 if result["validation_passed"] else 1


if __name__=="__main__":
    sys.exit(main())
