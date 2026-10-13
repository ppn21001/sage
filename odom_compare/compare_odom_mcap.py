#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOM_TOPIC = "/terrascout1/fast_lio/odometry" # Odometry from FAST-LIO2 state estimation
GT_TOPIC = "/terrascout1/odom_gt" # Ground truth odometry from Gazebo simulation


def quat_to_rpy(x, y, z, w):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=float)


def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def read_odom(bag_path, topic_names):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="mcap",
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    missing = [t for t in topic_names if t not in type_map]
    if missing:
        raise RuntimeError(
            f"Missing topic(s) in bag: {missing}\n"
            f"Available topics: {sorted(type_map)}"
        )

    msg_types = {
        topic: get_message(type_map[topic])
        for topic in topic_names
    }

    data = {topic: [] for topic in topic_names}
    frames = {topic: set() for topic in topic_names}

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in data:
            continue

        msg = deserialize_message(serialized, msg_types[topic])

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rpy = quat_to_rpy(q.x, q.y, q.z, q.w)

        data[topic].append(
            (
                timestamp_ns * 1e-9,
                np.array([p.x, p.y, p.z], dtype=float),
                rpy,
            )
        )
        frames[topic].add((msg.header.frame_id, msg.child_frame_id))

    return data, frames


def nearest_pairs(odom, gt, max_dt):
    odom_t = np.array([x[0] for x in odom])
    gt_t = np.array([x[0] for x in gt])

    pairs = []

    for i, t in enumerate(odom_t):
        j = np.searchsorted(gt_t, t)

        candidates = []
        if j < len(gt_t):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)

        if not candidates:
            continue

        best_j = min(candidates, key=lambda k: abs(gt_t[k] - t))
        dt = abs(gt_t[best_j] - t)

        if dt <= max_dt:
            pairs.append((i, best_j, dt))

    return pairs


def stats(name, values, unit):
    values = np.asarray(values)
    abs_values = np.abs(values)
    rmse = math.sqrt(float(np.mean(values ** 2)))
    mae = float(np.mean(abs_values))
    maximum = float(np.max(abs_values))
    mean = float(np.mean(values))

    print(
        f"{name:8s}  mean={mean: .5f} {unit:3s}  "
        f"MAE={mae:.5f} {unit:3s}  "
        f"RMSE={rmse:.5f} {unit:3s}  "
        f"max={maximum:.5f} {unit}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Compare SAGE /odom against /odom_gt from an MCAP rosbag."
    )
    parser.add_argument(
        "bag",
        help="Path to .mcap file, e.g. odom_compare_0.mcap",
    )
    parser.add_argument(
        "--max-dt",
        type=float,
        default=0.03,
        help="Maximum timestamp difference for pairing messages in seconds (default: 0.03)",
    )
    parser.add_argument(
        "--out-dir",
        default="odom_error_results",
        help="Directory for plots and CSV output",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    data, frames = read_odom(bag_path, [ODOM_TOPIC, GT_TOPIC])

    odom = data[ODOM_TOPIC]
    gt = data[GT_TOPIC]

    print(f"\nRead {len(odom)} odom messages")
    print(f"Read {len(gt)} ground-truth messages")
    print(f"{ODOM_TOPIC} frames: {frames[ODOM_TOPIC]}")
    print(f"{GT_TOPIC} frames: {frames[GT_TOPIC]}")

    pairs = nearest_pairs(odom, gt, args.max_dt)
    if not pairs:
        raise RuntimeError(
            "No timestamp pairs found. Try increasing --max-dt."
        )

    print(f"Matched {len(pairs)} pairs (max Δt = {args.max_dt:.3f} s)")

    times = []
    pos_err = []
    rpy_err = []
    dt_err = []

    t0 = odom[pairs[0][0]][0]

    for i, j, dt in pairs:
        to, po, ro = odom[i]
        _, pg, rg = gt[j]

        times.append(to - t0)
        pos_err.append(po - pg)
        rpy_err.append(wrap_angle(ro - rg))
        dt_err.append(dt)

    times = np.asarray(times)
    pos_err = np.asarray(pos_err)
    rpy_err = np.asarray(rpy_err)
    dt_err = np.asarray(dt_err)

    print("\nPosition error statistics")
    stats("x", pos_err[:, 0], "m")
    stats("y", pos_err[:, 1], "m")
    stats("z", pos_err[:, 2], "m")

    horizontal = np.linalg.norm(pos_err[:, :2], axis=1)
    total_3d = np.linalg.norm(pos_err, axis=1)

    stats("xy norm", horizontal, "m")
    stats("3D norm", total_3d, "m")

    rpy_deg = np.degrees(rpy_err)

    print("\nOrientation error statistics")
    stats("roll", rpy_deg[:, 0], "deg")
    stats("pitch", rpy_deg[:, 1], "deg")
    stats("yaw", rpy_deg[:, 2], "deg")

    print(
        f"\nTimestamp pairing: mean Δt={np.mean(dt_err)*1000:.2f} ms, "
        f"max Δt={np.max(dt_err)*1000:.2f} ms"
    )

    csv = np.column_stack(
        (
            times,
            pos_err[:, 0],
            pos_err[:, 1],
            pos_err[:, 2],
            horizontal,
            total_3d,
            rpy_deg[:, 0],
            rpy_deg[:, 1],
            rpy_deg[:, 2],
            dt_err,
        )
    )

    csv_path = out_dir / "odom_error.csv"
    np.savetxt(
        csv_path,
        csv,
        delimiter=",",
        header=(
            "time_s,dx_m,dy_m,dz_m,xy_error_m,error_3d_m,"
            "droll_deg,dpitch_deg,dyaw_deg,pair_dt_s"
        ),
        comments="",
    )

    plt.figure(figsize=(10, 5))
    plt.plot(times, pos_err[:, 0], label="x")
    plt.plot(times, pos_err[:, 1], label="y")
    plt.plot(times, pos_err[:, 2], label="z")
    plt.xlabel("Time [s]")
    plt.ylabel("Position error [m]")
    plt.title("SAGE odom vs ground truth: position error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    pos_plot = out_dir / "position_error.png"
    plt.savefig(pos_plot, dpi=160)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(times, rpy_deg[:, 0], label="roll")
    plt.plot(times, rpy_deg[:, 1], label="pitch")
    plt.plot(times, rpy_deg[:, 2], label="yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Orientation error [deg]")
    plt.title("SAGE odom vs ground truth: orientation error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    ori_plot = out_dir / "orientation_error.png"
    plt.savefig(ori_plot, dpi=160)
    plt.close()

    print(f"\nSaved:")
    print(f"  {csv_path}")
    print(f"  {pos_plot}")
    print(f"  {ori_plot}")


if __name__ == "__main__":
    main()
