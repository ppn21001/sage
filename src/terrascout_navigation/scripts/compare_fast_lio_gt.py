#!/usr/bin/env python3
"""Evaluate FAST-LIO2 odometry against Gazebo ground truth from a ROS 2 MCAP bag.

Both trajectories are converted to IMU poses, then the FAST-LIO2 world frame
(camera_init) is aligned to Gazebo's world frame (odom) at the first matched pose.
Reported errors therefore describe drift/relative accuracy after initialization,
not absolute global localization accuracy.
"""

import argparse
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")  # Save plots even on headless Docker hosts.
    import matplotlib.pyplot as plt
except ImportError:
    plt = None  # Still generate CSV and statistics without optional plotting dependency.
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp

LIO_TOPIC = "/terrascout1/odom"
GT_TOPIC = "/terrascout1/odom_gt"


def read_odometry_bag(bag_path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in type_map]
    if missing:
        raise RuntimeError(
            f"Missing topics {missing}. Available: {sorted(type_map)}"
        )
    expected_type = "nav_msgs/msg/Odometry"
    for topic in topics:
        if type_map[topic] != expected_type:
            raise RuntimeError(f"{topic}: expected {expected_type}, got {type_map[topic]}")
    classes = {topic: get_message(type_map[topic]) for topic in topics}
    data = {topic: [] for topic in topics}
    frames = {topic: set() for topic in topics}
    stamp_fallbacks = {topic: 0 for topic in topics}

    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic not in data:
            continue
        msg = deserialize_message(serialized, classes[topic])
        ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if ns == 0:
            ns = received_ns
            stamp_fallbacks[topic] += 1
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = np.array([p.x, p.y, p.z], dtype=float)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)):
            continue
        norm = np.linalg.norm(quat)
        if norm < 1e-8:
            continue
        data[topic].append((ns * 1e-9, xyz, quat / norm))
        frames[topic].add((msg.header.frame_id, msg.child_frame_id))

    for topic in topics:
        # Sorted, unique stamp samples for interpolation.
        data[topic].sort(key=lambda row: row[0])
        unique = {}
        for row in data[topic]:
            unique[row[0]] = row
        data[topic] = list(unique.values())
        print(f"{topic}: {len(data[topic])} samples, frames={frames[topic]}")
        if stamp_fallbacks[topic]:
            print(
                f"  WARNING: {stamp_fallbacks[topic]} messages had zero header stamps; "
                "used bag receipt times for those messages"
            )
        if not data[topic]:
            raise RuntimeError(f"No usable odometry samples for {topic}")
    return data, frames


def as_arrays(rows):
    return (
        np.array([r[0] for r in rows], dtype=float),
        np.stack([r[1] for r in rows]),
        np.stack([r[2] for r in rows]),
    )


def matched_and_interpolated(lio_rows, gt_rows, max_dt):
    lio_t, lio_p, lio_q = as_arrays(lio_rows)
    gt_t, gt_p, gt_q = as_arrays(gt_rows)
    if len(gt_t) < 2:
        raise RuntimeError("At least two ground-truth samples are required")
    inside = (lio_t >= gt_t[0]) & (lio_t <= gt_t[-1])
    li = np.flatnonzero(inside)
    ti = lio_t[li]

    upper = np.searchsorted(gt_t, ti)
    upper = np.clip(upper, 1, len(gt_t) - 1)
    lower = upper - 1
    distance = np.minimum(np.abs(ti - gt_t[lower]), np.abs(gt_t[upper] - ti))
    valid = distance <= max_dt
    li = li[valid]
    ti = ti[valid]
    distance = distance[valid]
    if len(li) < 2:
        raise RuntimeError(
            f"Only {len(li)} matched messages within {max_dt * 1000:.1f} ms. "
            "Check that both streams use simulation time and overlap in the bag."
        )
    interpolated_p = np.column_stack(
        [np.interp(ti, gt_t, gt_p[:, k]) for k in range(3)]
    )
    interpolated_r = Slerp(gt_t, Rotation.from_quat(gt_q))(ti)
    print(
        f"Matched {len(li)}/{len(lio_t)} FAST-LIO2 samples; "
        f"nearest GT timestamp mean={np.mean(distance)*1000:.1f} ms, "
        f"max={np.max(distance)*1000:.1f} ms"
    )
    return ti, lio_p[li], Rotation.from_quat(lio_q[li]), interpolated_p, interpolated_r, distance


def stats(name, values, unit):
    values = np.asarray(values)
    print(
        f"{name:14s} mean={np.mean(values): .5f} {unit:3s} "
        f"MAE={np.mean(np.abs(values)):.5f} {unit:3s} "
        f"RMSE={np.sqrt(np.mean(values ** 2)):.5f} {unit:3s} "
        f"max={np.max(np.abs(values)):.5f} {unit}"
    )


def save_plot(path, t, series, ylabel, title):
    if plt is None:
        return
    plt.figure(figsize=(10, 5))
    for name, values in series.items():
        plt.plot(t, values, label=name)
    plt.xlabel("Time since first matched measurement [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def save_xy_trajectory(path, gt_xyz, lio_xyz):
    if plt is None:
        return
    plt.figure(figsize=(7, 7))
    plt.plot(gt_xyz[:, 0], gt_xyz[:, 1], label="Gazebo ground truth")
    plt.plot(lio_xyz[:, 0], lio_xyz[:, 1], label="FAST-LIO2 (aligned)")
    plt.scatter([gt_xyz[0, 0]], [gt_xyz[0, 1]], marker="o", label="Start")
    plt.xlabel("World X [m]")
    plt.ylabel("World Y [m]")
    plt.title("FAST-LIO2 vs ground truth: XY trajectory")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory (with metadata.yaml and .mcap files)")
    parser.add_argument("--max-dt", type=float, default=0.03, help="Max distance to nearest GT stamp, seconds")
    parser.add_argument("--imu-height", type=float, default=0.48, help="IMU height above base_footprint, m (SAGE simulation: 0.33 + 0.15)")
    parser.add_argument("--out-dir", type=Path, default=Path("fast_lio_error_results"))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    data, frames = read_odometry_bag(args.bag, [LIO_TOPIC, GT_TOPIC])
    expected_lio = ("camera_init", "body")
    expected_gt = ("odom", "terrascout1/base_footprint")
    if frames[LIO_TOPIC] != {expected_lio}:
        print(f"WARNING: FAST-LIO2 frames differ from expected {expected_lio}. Confirm the tracked frame is the IMU.")
    if frames[GT_TOPIC] != {expected_gt}:
        raise RuntimeError(
            f"Ground-truth frames {frames[GT_TOPIC]} differ from expected {expected_gt}. "
            "The IMU offset below assumes GT tracks base_footprint."
        )

    t, lio_p, lio_r, gt_base_p, gt_base_r, time_deltas = matched_and_interpolated(
        data[LIO_TOPIC], data[GT_TOPIC], args.max_dt
    )

    # Convert GT base_footprint pose to IMU pose (same rotation in SAGE URDF).
    gt_imu_p = gt_base_p + gt_base_r.apply(np.array([0.0, 0.0, args.imu_height]))
    gt_imu_r = gt_base_r

    # R_odom_camera_init and its translation, determined from first matched pose.
    # This is initial-pose SE(3) alignment, NOT a best-fit alignment over the run.
    align_r = gt_imu_r[0] * lio_r[0].inv()
    align_t = gt_imu_p[0] - align_r.apply(lio_p[0])
    lio_p_aligned = align_r.apply(lio_p) + align_t
    lio_r_aligned = align_r * lio_r

    err_p = lio_p_aligned - gt_imu_p
    err_r = gt_imu_r.inv() * lio_r_aligned
    err_rpy_deg = err_r.as_euler("xyz", degrees=True)
    angle_deg = np.rad2deg(err_r.magnitude())
    xy_norm = np.linalg.norm(err_p[:, :2], axis=1)
    xyz_norm = np.linalg.norm(err_p, axis=1)
    seconds = t - t[0]

    print(f"\nInitial-pose alignment, GT base_footprint -> IMU offset: [0, 0, {args.imu_height:.3f}] m")
    print("These statistics represent relative trajectory error/drift from the aligned start pose.")
    print("\nPosition error:")
    for k, name in enumerate(("x", "y", "z")):
        stats(name, err_p[:, k], "m")
    stats("horizontal", xy_norm, "m")
    stats("3D", xyz_norm, "m")
    print("\nOrientation error (relative rotation):")
    for k, name in enumerate(("roll", "pitch", "yaw")):
        stats(name, err_rpy_deg[:, k], "deg")
    stats("rotation angle", angle_deg, "deg")

    csv_path = args.out_dir / "fast_lio_error.csv"
    columns = np.column_stack([
        seconds,
        gt_imu_p, lio_p_aligned, err_p,
        xy_norm, xyz_norm, err_rpy_deg, angle_deg, time_deltas,
    ])
    np.savetxt(
        csv_path, columns, delimiter=",", comments="",
        header=(
            "time_s,gt_imu_x_m,gt_imu_y_m,gt_imu_z_m,"
            "lio_aligned_x_m,lio_aligned_y_m,lio_aligned_z_m,"
            "error_x_m,error_y_m,error_z_m,error_xy_m,error_3d_m,"
            "error_roll_deg,error_pitch_deg,error_yaw_deg,"
            "error_rotation_deg,nearest_gt_stamp_dt_s"
        ),
    )

    save_plot(
        args.out_dir / "position_error.png", seconds,
        {"x": err_p[:, 0], "y": err_p[:, 1], "z": err_p[:, 2], "XY norm": xy_norm},
        "Position error [m]", "FAST-LIO2 vs GT: initial-aligned IMU position error",
    )
    save_plot(
        args.out_dir / "orientation_error.png", seconds,
        {"roll": err_rpy_deg[:, 0], "pitch": err_rpy_deg[:, 1], "yaw": err_rpy_deg[:, 2]},
        "Relative rotation error [deg]", "FAST-LIO2 vs GT: orientation error",
    )
    save_xy_trajectory(args.out_dir / "trajectory_xy.png", gt_imu_p, lio_p_aligned)
    print(f"\nSaved results in {args.out_dir.resolve()}")
    print(f"  {csv_path.name}")
    if plt is None:
        print("  Plots skipped: matplotlib unavailable. Add python3-matplotlib to the image for plots.")
    else:
        print("  position_error.png, orientation_error.png, trajectory_xy.png")


if __name__ == "__main__":
    main()
