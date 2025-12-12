#!/usr/bin/env python3
import sys
import math
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def yaw_from_quat(qx, qy, qz, qw):
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def unwrap_angles(angles):
    # unwrap radians to avoid jumps at +-pi
    return np.unwrap(np.array(angles, dtype=np.float64))

def main():
    if len(sys.argv) < 2:
        print("Usage: compute_imu_cov.py <bag_dir> [topic]")
        sys.exit(1)

    bag_dir = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) >= 3 else "/imu"

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                         output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    if topic not in type_map:
        print(f"Topic {topic} not found in bag. Available topics:")
        for k in type_map.keys():
            print(" -", k)
        sys.exit(2)

    msg_type = get_message(type_map[topic])

    ts = []
    wz = []
    yaw = []

    while reader.has_next():
        (tpc, data, t) = reader.read_next()
        if tpc != topic:
            continue
        msg = deserialize_message(data, msg_type)

        # time in seconds
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ts.append(stamp)

        wz.append(msg.angular_velocity.z)
        yaw.append(yaw_from_quat(msg.orientation.x, msg.orientation.y,
                                 msg.orientation.z, msg.orientation.w))

    if len(ts) < 20:
        print(f"Not enough samples on {topic}: {len(ts)}")
        sys.exit(3)

    ts = np.array(ts, dtype=np.float64)
    wz = np.array(wz, dtype=np.float64)
    yaw_u = unwrap_angles(yaw)

    # --- gyro z variance ---
    gyro_z_var = float(np.var(wz, ddof=1))
    gyro_z_std = float(np.std(wz, ddof=1))

    # --- yaw variance: detrend linear drift ---
    t0 = ts[0]
    x = ts - t0
    # fit yaw = a*x + b
    a, b = np.polyfit(x, yaw_u, 1)
    yaw_fit = a * x + b
    yaw_res = yaw_u - yaw_fit

    yaw_var_detrended = float(np.var(yaw_res, ddof=1))
    yaw_std_detrended = float(np.std(yaw_res, ddof=1))

    yaw_var_total = float(np.var(yaw_u, ddof=1))
    yaw_std_total = float(np.std(yaw_u, ddof=1))

    duration = float(ts[-1] - ts[0])

    print(f"Bag: {bag_dir}")
    print(f"Topic: {topic}")
    print(f"Samples: {len(ts)}, duration: {duration:.2f} s")
    print("")
    print("=== Recommended covariance candidates ===")
    print(f"gyro_z_var (rad/s)^2      = {gyro_z_var:.6g}   (std={gyro_z_std:.6g} rad/s)")
    print(f"yaw_var_detrended rad^2   = {yaw_var_detrended:.6g}   (std={yaw_std_detrended:.6g} rad)")
    print(f"yaw_var_total rad^2       = {yaw_var_total:.6g}   (std={yaw_std_total:.6g} rad)")
    print("")
    print("Notes:")
    print("- Use gyro_z_var for angular_velocity_covariance[8].")
    print("- For yaw_var: if yaw drifts, choose larger (closer to yaw_var_total).")
    print("- If yaw is stable, yaw_var_detrended is a good starting point.")

if __name__ == "__main__":
    main()
