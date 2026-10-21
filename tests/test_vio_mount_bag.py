"""Offline fixtures only: no bag services, ROS master, models or flight."""
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import zlib

from scripts import analyze_vio_mount_bag as bag


def stamp(ns):
    return NS(secs=ns//1000000000,nsecs=ns%1000000000)


def pose(position=(0.,0.,0.), q=(0.,0.,0.,1.)):
    return NS(position=NS(x=position[0],y=position[1],z=position[2]),
              orientation=NS(x=q[0],y=q[1],z=q[2],w=q[3]))


def configuration():
    camera={"position_m":[.12,.06,.18],"orientation_xyzw":bag.rpy_quaternion([0.,.35,0.])}
    imu={"position_m":[0.,0.,.02],"orientation_xyzw":[0.,0.,0.,1.]}
    return {"camera_link_expected":camera,"imu_link_expected":imu,
            "rig_center_offset_camera_link_m":[0.,-.06,0.],
            "rig_center_expected":bag.camera_center(camera,[0.,-.06,0.])}


def sdf_fixture(direct):
    inertial='<inertial><pose>.001 0 .002 0 0 0</pose><mass>1.51</mass><inertia><ixx>.02913</ixx><iyy>.03013</iyy><izz>.05793</izz><ixy>0</ixy><ixz>-.0002</ixz><iyz>0</iyz></inertia></inertial>'
    sensor='<sensor name="vio_camera" type="multicamera">'+('<pose>.12 .06 .18 0 .35 0</pose>' if direct else '')+'<camera name="left"/><camera name="right"><pose>0 -.12 0 0 0 0</pose></camera></sensor>'
    base='<link name="base_link">'+inertial+(sensor if direct else '')+'</link>'
    camera='' if direct else '<link name="vio_camera_link"><pose>.12 .06 .18 0 .35 0</pose>'+sensor+'</link><joint name="vio_camera_joint" type="fixed"><parent>base_link</parent><child>vio_camera_link</child></joint>'
    return '<sdf version="1.5"><model name="iris_vio">'+base+camera+'<link name="/imu_link"><pose>0 0 .02 0 0 0</pose></link></model></sdf>'


def load_fixture(directory,direct):
    path=Path(directory)/('direct.sdf' if direct else 'legacy.sdf')
    path.write_text(sdf_fixture(direct))
    return bag.sdf_configuration(path)


def direct_links():
    value=links()
    value.name.pop(1);value.pose.pop(1)
    return value


def links():
    config=configuration();c=config["camera_link_expected"];i=config["imu_link_expected"]
    return NS(name=["iris::base_link","iris::vio_camera_link","iris::/imu_link"],
              pose=[pose(),pose(c["position_m"],c["orientation_xyzw"]),pose(i["position_m"],i["orientation_xyzw"])])


def image(ns,encoding="rgb8",data=b"\x01\x02\x03\x04\x05\x06",step=6):
    return NS(header=NS(stamp=stamp(ns),seq=3,frame_id="uav_stereo_optical"),
              width=2,height=1,step=step,encoding=encoding,is_bigendian=0,data=data)


def messages(with_clock=True):
    ns=10000000000
    values=[(bag.TOPICS["links"],links(),stamp(ns-20000000)),
            (bag.TOPICS["models"],NS(name=["iris"],pose=[pose()]),stamp(ns-20000000))]
    if with_clock:values.append((bag.TOPICS["clock"],NS(clock=stamp(ns)),stamp(ns)))
    values += [(bag.TOPICS["left"],image(ns),stamp(ns)),(bag.TOPICS["right"],image(ns),stamp(ns+1)),
               (bag.TOPICS["links"],links(),stamp(ns+20000000)),
               (bag.TOPICS["models"],NS(name=["iris"],pose=[pose()]),stamp(ns+20000000))]
    return values


class VioMountBagTests(unittest.TestCase):
    def test_actual_integer_nanoseconds_not_reconstructed_float(self):
        ns=9007199254740993
        self.assertEqual(bag.stamp_ns(stamp(ns)),ns)
        self.assertEqual(bag.image_value(image(ns),ns)[0]["source_stamp_ns"],ns)
        for value in (NS(secs=True,nsecs=0),NS(secs=1,nsecs=1000000000),NS(secs=-1,nsecs=0),
                      NS(secs=1.0,nsecs=0),NS(to_sec=lambda:1.)):
            with self.assertRaises(ValueError):bag.stamp_ns(value)

    def test_window_precision_does_not_round_source_stamps(self):
        self.assertEqual(bag.seconds_ns("9007199.254740993"),9007199254740993)
        for value in ("nan","-1","0.0000000001"):
            with self.assertRaises(ValueError):bag.seconds_ns(value)

    def test_relative_pose_cancels_world_translation_and_yaw(self):
        q=bag.rpy_quaternion([0,0,math.pi/2]);mount=configuration()["camera_link_expected"]
        base={"position_m":[3.,-2.,4.],"orientation_xyzw":q}
        rotated=bag.rotate(q,mount["position_m"])
        child={"position_m":[v+r for v,r in zip(base["position_m"],rotated)],
               "orientation_xyzw":bag.qmul(q,mount["orientation_xyzw"])}
        error=bag.pose_error(bag.relative_pose(base,child),mount)
        self.assertLess(error["translation_error_m"],1e-12)
        self.assertLess(error["rotation_error_rad"],1e-12)

    def test_rig_center_offset_rotates_in_camera_link_frame(self):
        actual={"position_m":[1.,2.,3.],"orientation_xyzw":bag.rpy_quaternion([0,0,math.pi/2])}
        center=bag.camera_center(actual,[0,-.06,0])
        self.assertAlmostEqual(center["position_m"][0],1.06)
        self.assertAlmostEqual(center["position_m"][1],2.)

    def test_actual_sdf_link_origin_distinct_from_rig_center(self):
        config=bag.sdf_configuration(bag.ROOT/"vio/models/iris_vio/iris_vio.sdf")
        self.assertEqual(config["camera_sensor_expected"]["position_m"],[.12,.06,.18])
        self.assertEqual(config["rig_center_offset_camera_link_m"],[0.,-.06,0.])
        for actual,expected in zip(config["rig_center_expected"]["position_m"],[.12,0.,.18]):self.assertAlmostEqual(actual,expected)
        self.assertEqual(config["imu_link_expected"]["position_m"],[0.,0.,.02])
        self.assertEqual(len(config["model_sdf_sha256"]),64)

    def test_same_message_relative_mount_and_measured_perturbation(self):
        value=links();value.pose[1].position.x+=.025
        value.pose[1].orientation=pose(q=bag.qmul(bag.rpy_quaternion([.03,0,0]),
                                                configuration()["camera_link_expected"]["orientation_xyzw"])).orientation
        row=bag.link_row(value,10000000000,configuration())
        self.assertIsNone(row["source_stamp_ns"])
        self.assertAlmostEqual(row["camera_error"]["translation_error_m"],.025)
        self.assertAlmostEqual(row["camera_error"]["rotation_error_rad"],.03)

    def test_missing_ambiguous_link_and_invalid_geometry_rejected(self):
        row=bag.link_row(NS(name=["iris::base_link"],pose=[pose()]),10,configuration())
        self.assertEqual(row["camera_measurement"]["status"],"unavailable")
        self.assertEqual(row["imu_measurement"]["status"],"unavailable")
        self.assertIsNone(row["camera_error"])
        with self.assertRaises(ValueError):bag.link_row(NS(name=["iris::base_link","iris::base_link"],pose=[pose(),pose()]),10,configuration())
        for q in ([0,0,0,0],[0,True,0,1],[0,0,float("nan"),1]):
            with self.assertRaises(ValueError):bag.qnorm(q)
        with self.assertRaises(ValueError):bag.relative_pose({"position_m":[0,0],"orientation_xyzw":[0,0,0,1]},configuration()["camera_link_expected"])

    def test_actual_link_pose_nonunit_poison_rejected_not_normalized_to_identity(self):
        value=links();value.pose[1].orientation.w*=2
        value.pose[1].orientation.y*=2
        row=bag.link_row(value,10000000000,configuration())
        self.assertIsNone(row["camera_error"])
        self.assertEqual(row["camera_measurement"]["status"],"unavailable")
        self.assertIn("not near unit",row["camera_measurement"]["reason"])
        with self.assertRaisesRegex(ValueError,"not near unit"):
            bag.pose_value(pose(q=(0,0,0,2)))

    def test_actual_model_pose_poison_preserves_failed_analysis_evidence(self):
        values=messages()
        for topic,message,_ in values:
            if topic==bag.TOPICS["models"]:message.pose[0].orientation.w=2
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)/"new"
            result=bag.analysis(values,out,configuration(),9970000000,10030000000)
            self.assertFalse(result["analysis_completed"])
            self.assertIn("not near unit",result["errors"][0]["message"])
            self.assertTrue((out/"summary.json").is_file())

    def test_actual_link_and_model_roundoff_norm_accepted_with_original_evidence(self):
        q=bag.rpy_quaternion([.02,.35,.05]);raw=[v*(1+1e-6) for v in q]
        parsed=bag.pose_value(pose(q=raw))
        self.assertEqual(parsed["actual_orientation_raw_xyzw"],raw)
        self.assertAlmostEqual(parsed["actual_orientation_raw_norm"],1+1e-6)
        self.assertAlmostEqual(math.hypot(*parsed["orientation_xyzw"]),1.)
        values=messages()
        for topic,message,_ in values:
            if topic==bag.TOPICS["models"]:message.pose[0]=pose(q=raw)
            if topic==bag.TOPICS["links"]:
                message.pose[0]=pose(q=tuple(v*1.000001 for v in (0,0,0,1)))
        with tempfile.TemporaryDirectory() as directory:
            result=bag.analysis(values,Path(directory)/"new",configuration(),9970000000,10030000000)
            self.assertTrue(result["analysis_completed"])
            self.assertTrue(result["relative_mount_pose_available"])

    def test_receipt_bracket_never_claims_render_source(self):
        rows=[bag.link_row(links(),9980000000,configuration()),bag.link_row(links(),10020000000,configuration())]
        result=bag.receipt_bracket(rows,10000000000)
        self.assertEqual(result["gap_ns"],40000000)
        self.assertFalse(result["exact_render_time_pose_proven"])
        self.assertIsNone(result["before"]["source_stamp_ns"])

    def test_receipt_bracket_rejects_fake_source_gap_duplicates_order_and_extrapolation(self):
        a=bag.link_row(links(),9980000000,configuration());b=bag.link_row(links(),10020000000,configuration())
        forged=copy.deepcopy(a);forged["source_stamp_ns"]=forged["recorder_receipt_ns"]
        far=copy.deepcopy(b);far["recorder_receipt_ns"]=10090000000
        for values in ([a],[b,a],[a,a,b],[a,far],[forged,b]):
            with self.assertRaises(ValueError):bag.receipt_bracket(values,10000000000)

    def test_png_lossless_with_bgr_and_raw_padding_kept_in_source_hash(self):
        original=b"\x03\x02\x01\x06\x05\x04\xaa\xbb"
        meta,raw=bag.image_value(image(10,"bgr8",original,8),11)
        encoded=bag.png_bytes(meta,raw);offset=8;payload=b""
        while offset<len(encoded):
            size=struct.unpack(">I",encoded[offset:offset+4])[0];kind=encoded[offset+4:offset+8]
            if kind==b"IDAT":payload+=encoded[offset+8:offset+8+size]
            offset+=12+size
        self.assertEqual(zlib.decompress(payload),b"\x00\x01\x02\x03\x04\x05\x06")
        self.assertEqual(raw,original)
        self.assertEqual(meta["raw_data_sha256"],hashlib.sha256(original).hexdigest())
        with self.assertRaises(ValueError):bag.image_value(image(10,data=b"short"),11)

    def test_stream_pair_exact_ns_archive_and_mount_maximum(self):
        values=messages();values[-2][1].pose[1].position.z+=.04
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)/"new"
            result=bag.analysis(iter(values),out,configuration(),9970000000,10030000000,sample_every=1)
            self.assertTrue(result["analysis_completed"])
            self.assertEqual(result["pairs"],1);self.assertEqual(result["saved_pairs"],1)
            pair=json.loads((out/"pairs.jsonl").read_text())
            self.assertEqual(pair["source_stamp_ns"],10000000000)
            self.assertEqual(pair["link_receipt_bracket"]["status"],"approximate_receipt_bracket")
            self.assertEqual((out/"images"/pair["left"]["raw_filename"]).read_bytes(),b"\x01\x02\x03\x04\x05\x06")
            self.assertAlmostEqual(result["maximum_mount_errors"]["camera_translation_m"]["value"],.04)
            self.assertEqual(result["maximum_mount_errors"]["camera_translation_m"]["recorder_receipt_ns"],10020000000)
            self.assertFalse(result["ground_truth_exposed_to_agent"])
            self.assertIsNone(result["mount_rigidity_or_render_time_accuracy_passed"])
            with self.assertRaises(FileExistsError):bag.analysis([],out,configuration(),9970000000,10030000000)

    def test_missing_topic_link_pair_and_clock_explicit_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            values=messages(False)
            result=bag.analysis(iter(values),root/"no_clock",configuration(),9970000000,10030000000)
            self.assertEqual(result["availability"]["clock"]["status"],"unavailable")
            pair=json.loads((root/"no_clock/pairs.jsonl").read_text())
            self.assertEqual(pair["link_receipt_bracket"]["status"],"unavailable")
            missing=[(bag.TOPICS["left"],image(10000000000),stamp(10000000000)),
                     (bag.TOPICS["links"],NS(name=[],pose=[]),stamp(10000000000))]
            result=bag.analysis(missing,root/"missing",configuration(),9970000000,10030000000)
            self.assertFalse(result["relative_mount_pose_available"])
            self.assertFalse(result["exact_pairs_available"])
            self.assertEqual(result["unpaired_images"][0]["reason"],"no_exact_source_partner")
            self.assertEqual(json.loads((root/"missing/mount_timeline.jsonl").read_text())["status"],"unavailable")

    def test_duplicate_completed_source_pair_fails_preserving_first_evidence(self):
        values=messages();values.extend([(bag.TOPICS["left"],image(10000000000),stamp(10025000000))])
        with tempfile.TemporaryDirectory() as directory:
            result=bag.analysis(values,Path(directory)/"new",configuration(),9970000000,10030000000)
            self.assertFalse(result["analysis_completed"]);self.assertEqual(result["pairs"],1)
            self.assertIn("Duplicate/nonmonotonic",result["errors"][0]["message"])

    def test_auto_window_only_uses_actual_native_lost_marker(self):
        value=lambda r:(bag.TOPICS["native"],NS(data=json.dumps(r)),stamp(1))
        values=[value({"stamp":10.,"tracking_ok":False,"reason":"stationary_initialization_waiting"}),
                value({"stamp":125.552,"tracking_ok":False,"reason":"native_limits_or_visual_tracking_lost"})]
        self.assertEqual(bag.failure_window(values),(121552000000,126552000000))
        with self.assertRaises(ValueError):bag.failure_window(values[:1])

    def test_default_is_plan_no_bag_or_configuration_read(self):
        with mock.patch.object(bag,"sdf_configuration",side_effect=AssertionError("Unexpected file read")),contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(bag.main([]),0)
        plan=json.loads(stdout.getvalue());self.assertTrue(plan["default_no_bag_read"])

    def test_legacy_sdf_explicit_fixture_still_measures_same_message_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,False)
            self.assertEqual(config["camera_mount_topology"],"physics_camera_link_fixed_joint")
            row=bag.link_row(links(),10000000000,config)
            self.assertEqual(row["camera_measurement"]["status"],"measured_same_LinkStates_message")
            self.assertAlmostEqual(row["camera_error"]["translation_error_m"],0.)
            self.assertNotIn("nominal_camera_world",row)

    def test_direct_parent_has_no_measured_camera_zero_and_retains_payload_source(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,True)
            self.assertEqual(config["camera_mount_topology"],"sensor_direct_parent_base_link")
            self.assertIsNone(config["camera_link_expected"])
            self.assertEqual(config["camera_sensor_expected"]["position_m"],[.12,.06,.18])
            self.assertEqual(config["base_inertial_configuration"]["mass_kg"],1.51)
            self.assertEqual(config["base_inertial_configuration"]["pose"]["position_m"],[.001,0.,.002])
            self.assertEqual(config["base_inertial_configuration"]["inertia_kg_m2"]["ixz"],-.0002)
            self.assertEqual(config["model_sdf_sha256"],hashlib.sha256((Path(directory)/"direct.sdf").read_bytes()).hexdigest())
            row=bag.link_row(direct_links(),10000000000,config)
            self.assertEqual(row["camera_measurement"]["status"],"unavailable_by_direct_parent")
            self.assertIsNone(row["camera_error"]);self.assertIsNone(row["camera_relative_body"])
            self.assertIsNone(row["rig_center_error"])
            self.assertEqual(row["nominal_camera_world"]["status"],"configured_kinematics_only")
            self.assertFalse(row["nominal_camera_world"]["measured_sensor_pose"])
            self.assertFalse(row["nominal_camera_world"]["exact_render_time_pose_proven"])
            self.assertEqual(row["imu_measurement"]["status"],"measured_same_LinkStates_message")

    def test_direct_nominal_world_composes_actual_base_without_claiming_render_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,True);value=direct_links()
            q=bag.rpy_quaternion([0,0,math.pi/2]);value.pose[0]=pose([3,-2,4],q)
            imu=bag.compose_pose({"position_m":[3,-2,4],"orientation_xyzw":q},config["imu_link_expected"])
            value.pose[1]=pose(imu["position_m"],imu["orientation_xyzw"])
            row=bag.link_row(value,10000000000,config)
            p=row["nominal_camera_world"]["pose_world"]["position_m"]
            for a,b in zip(p,[2.94,-1.88,4.18]):self.assertAlmostEqual(a,b)
            self.assertAlmostEqual(row["imu_error"]["translation_error_m"],0.)
            self.assertIsNone(row["camera_error"])

    def test_direct_analysis_images_keep_exact_evidence_imu_is_measured_camera_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,True);values=messages()
            for topic,value,_ in values:
                if topic==bag.TOPICS["links"]:
                    value.name.pop(1);value.pose.pop(1);value.pose[1].position.x+=.007
            out=Path(directory)/"analysis"
            result=bag.analysis(values,out,config,9970000000,10030000000,sample_every=1)
            self.assertTrue(result["analysis_completed"])
            self.assertEqual(result["camera_relative_measurement"]["status"],"unavailable_by_direct_parent")
            self.assertFalse(result["relative_mount_pose_available"]);self.assertTrue(result["imu_relative_pose_available"])
            self.assertIsNone(result["maximum_mount_errors"]["camera_translation_m"])
            self.assertIsNone(result["maximum_mount_errors"]["camera_rotation_rad"])
            self.assertAlmostEqual(result["maximum_mount_errors"]["imu_translation_m"]["value"],.007)
            pair=json.loads((out/"pairs.jsonl").read_text())
            self.assertEqual(pair["source_stamp_ns"],10000000000)
            self.assertEqual(pair["left"]["raw_data_sha256"],hashlib.sha256(image(10000000000).data).hexdigest())
            row=pair["link_receipt_bracket"]["before"]
            self.assertIsNone(row["camera_error"])
            self.assertFalse(pair["link_receipt_bracket"]["exact_render_time_pose_proven"])

    def test_direct_missing_imu_and_poisoned_base_not_fabricated(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,True)
            value=NS(name=["iris::base_link"],pose=[pose()])
            row=bag.link_row(value,10,config)
            self.assertEqual(row["imu_measurement"]["status"],"unavailable")
            self.assertIsNone(row["imu_error"]);self.assertIsNone(row["camera_error"])
            value.pose[0]=pose(q=(0,0,0,2))
            with self.assertRaisesRegex(ValueError,"not near unit"):bag.link_row(value,10,config)

    def test_direct_configuration_cannot_mislabel_old_actual_camera_link(self):
        with tempfile.TemporaryDirectory() as directory:
            config=load_fixture(directory,True)
            with self.assertRaisesRegex(ValueError,"contradict direct-parent"):
                bag.link_row(links(),10000000000,config)

    def test_ambiguous_sdf_topology_or_sensor_pose_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"broken.sdf"
            for text in (sdf_fixture(True).replace('</model>','<link name="vio_camera_link"/></model>'),
                         sdf_fixture(True).replace('.12 .06 .18','nan .06 .18'),
                         sdf_fixture(False).replace('type="fixed"','type="revolute"')):
                path.write_text(text)
                with self.assertRaises(ValueError):bag.sdf_configuration(path)


if __name__=="__main__":unittest.main()
