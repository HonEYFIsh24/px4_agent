"""Explicit MAVLink 2 ODOMETRY encoding; no ROS or network side effects.

The adapter's gravity-aligned local FLU axes have an arbitrary fixed heading.
LOCAL_FRD preserves that heading: x is unchanged, y/right and z/down change
sign. This is NOT a geographic ENU-to-NED rotation. BODY_FLU similarly changes
to BODY_FRD. Both pose quaternion and covariance follow those transformations.
"""
import math
import struct

from vio.ev_adapter import conjugate, multiply, unit_quaternion


FRD_SIGNS = (1, -1, -1)


def flu_to_frd(vector):
    if len(vector) != 3 or not all(math.isfinite(x) for x in vector):
        raise ValueError("vector must contain three finite values")
    return tuple(FRD_SIGNS[i] * vector[i] for i in range(3))


def body_frd_to_local_frd_quaternion(q_xyzw):
    q = unit_quaternion(q_xyzw)
    half_turn_x = (1, 0, 0, 0)
    converted = multiply(multiply(half_turn_x, q), conjugate(half_turn_x))
    return (converted[3], converted[0], converted[1], converted[2])  # MAVLink wxyz


def covariance_flu_to_frd(covariance):
    if len(covariance) != 36 or not all(math.isfinite(x) for x in covariance):
        raise ValueError("covariance must contain 36 finite values")
    signs = FRD_SIGNS + FRD_SIGNS
    if any(covariance[i * 6 + i] <= 0 for i in range(6)):
        raise ValueError("all covariance diagonal entries must be positive")
    if any(abs(covariance[i * 6 + j] - covariance[j * 6 + i]) > 1e-9
           for i in range(6) for j in range(6)):
        raise ValueError("covariance must be symmetric")
    return tuple(covariance[i * 6 + j] * signs[i] * signs[j]
                 for i in range(6) for j in range(6))


def covariance_upper_triangle(covariance):
    if len(covariance) != 36:
        raise ValueError("covariance must contain 36 values")
    return tuple(covariance[i * 6 + j] for i in range(6) for j in range(i, 6))


def odometry_fields(measurement):
    if not math.isfinite(measurement.stamp_sim_s) or measurement.stamp_sim_s <= 0:
        raise ValueError("measurement timestamp must be positive and finite")
    if type(measurement.epoch) is not int or measurement.epoch < 0:
        raise ValueError("epoch must be a nonnegative integer")
    if type(measurement.quality) is not int or not 1 <= measurement.quality <= 100:
        raise ValueError("only healthy measurements can be encoded")
    pose = flu_to_frd(measurement.position_local_m)
    velocity = flu_to_frd(measurement.linear_velocity_body_flu_m_s)
    omega = flu_to_frd(measurement.angular_velocity_body_flu_rad_s)
    return {"time_usec": round(measurement.stamp_sim_s * 1_000_000),
            "frame_id": 20, "child_frame_id": 12,  # LOCAL_FRD, BODY_FRD
            "x": pose[0], "y": pose[1], "z": pose[2],
            "q": body_frd_to_local_frd_quaternion(measurement.orientation_body_to_local_xyzw),
            "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
            "rollspeed": omega[0], "pitchspeed": omega[1], "yawspeed": omega[2],
            "pose_covariance": covariance_upper_triangle(covariance_flu_to_frd(measurement.pose_covariance)),
            "velocity_covariance": covariance_upper_triangle(covariance_flu_to_frd(measurement.twist_covariance_body)),
            "reset_counter": measurement.epoch % 256, "estimator_type": 3,  # visual-inertial
            "quality": measurement.quality}


class Mavlink2OdometryEncoder:
    """Lazy pymavlink dependency. The returned packet has a real v2 sequence/CRC."""
    def __init__(self, source_system=1, source_component=197):
        if not all(type(x) is int and 1 <= x <= 255 for x in (source_system, source_component)):
            raise ValueError("MAVLink source identifiers must be integers from 1 to 255")
        try:
            from pymavlink.dialects.v20 import common
        except ImportError as error:
            raise RuntimeError("MAVLink fusion requires pymavlink for the ROS Python interpreter") from error
        self.common = common
        self.mav = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)

    def encode(self, measurement):
        message = self.common.MAVLink_odometry_message(**odometry_fields(measurement))
        packet = message.pack(self.mav, force_mavlink1=False)
        self.mav.seq = (self.mav.seq + 1) % 256
        if packet[0] != 253 or packet[2] != 0:
            raise RuntimeError("expected an unsigned MAVLink 2 packet")
        return packet


def packet_ros_fields(packet):
    """Extract MAVROS Mavlink fields without its version/global-state helper bug."""
    if not isinstance(packet, (bytes, bytearray)) or len(packet) < 12 or packet[0] != 253:
        raise ValueError("expected MAVLink 2 packet")
    payload_length = packet[1]
    if packet[2] != 0 or len(packet) != payload_length + 12:
        raise ValueError("only complete unsigned MAVLink 2 packets are supported")
    payload = bytes(packet[10:10 + payload_length])
    padded = payload + b"\x00" * ((-len(payload)) % 8)
    return {"framing_status": 1, "magic": 253, "len": payload_length,
            "incompat_flags": packet[2], "compat_flags": packet[3],
            "seq": packet[4], "sysid": packet[5], "compid": packet[6],
            "msgid": int.from_bytes(packet[7:10], "little"),
            "checksum": int.from_bytes(packet[10 + payload_length:12 + payload_length], "little"),
            "payload64": list(struct.unpack("<" + "Q" * (len(padded) // 8), padded)),
            "signature": []}
