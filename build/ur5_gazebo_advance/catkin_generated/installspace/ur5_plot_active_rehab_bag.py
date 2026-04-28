#!/usr/bin/env python3
"""从 rosbag 提取主动康复演示数据并保存图表。"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


AXIS_INDEX = {"x": 1, "y": 2, "z": 3}
AXES = ("x", "y", "z")


def collect_pose_series(bag, topic):
    times = []
    xs = []
    ys = []
    zs = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 else t.to_sec()
        times.append(ts)
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
    return times, xs, ys, zs


def collect_wrench_series(bag, topic):
    times = []
    fx = []
    fy = []
    fz = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0.0 else t.to_sec()
        times.append(ts)
        fx.append(msg.wrench.force.x)
        fy.append(msg.wrench.force.y)
        fz.append(msg.wrench.force.z)
    return times, fx, fy, fz


def normalize_time(*series):
    valid_times = [times[0] for times, *_ in series if times]
    if not valid_times:
        return series
    t0 = min(valid_times)
    normalized = []
    for entry in series:
        times = entry[0]
        normalized.append(([t - t0 for t in times], *entry[1:]))
    return normalized


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def series_matrix(series):
    return np.column_stack([series[1], series[2], series[3]])


def vector_norm_series(series):
    _, xs, ys, zs = series
    return [
        float(np.linalg.norm([xs[i], ys[i], zs[i]]))
        for i in range(len(xs))
    ]


def build_active_segments(cmd, active_threshold):
    times = cmd[0]
    norm = vector_norm_series(cmd)
    segments = []
    start_t = None
    end_t = None

    for idx, magnitude in enumerate(norm):
        if magnitude > active_threshold:
            if start_t is None:
                start_t = times[idx]
            end_t = times[idx]
        elif start_t is not None:
            segments.append((start_t, end_t))
            start_t = None
            end_t = None

    if start_t is not None:
        segments.append((start_t, end_t))

    return segments


def add_active_segment_spans(axes, segments, color="tab:orange", alpha=0.12):
    if not segments:
        return

    if not isinstance(axes, (list, tuple, np.ndarray)):
        axes = [axes]

    label_used = False
    for ax in axes:
        for start_t, end_t in segments:
            ax.axvspan(
                start_t,
                end_t,
                color=color,
                alpha=alpha,
                linewidth=0.0,
                label="command active" if not label_used else None,
            )
            label_used = True


def maybe_warn_missing_series(series, topic_name):
    if series[0]:
        return
    print("WARNING: no samples found on {}".format(topic_name))


def interpolate_series(series, target_times):
    source_times = np.array(series[0], dtype=float)
    if source_times.size == 0:
        return tuple([[]] * 4)

    values = []
    for idx in (1, 2, 3):
        source_values = np.array(series[idx], dtype=float)
        values.append(
            np.interp(
                np.array(target_times, dtype=float),
                source_times,
                source_values,
                left=source_values[0],
                right=source_values[-1],
            )
        )

    return (list(target_times), values[0].tolist(), values[1].tolist(), values[2].tolist())


def compute_error_norm_series(source_series, target_series):
    if not source_series[0] or not target_series[0]:
        return [], []

    target_interp = interpolate_series(target_series, source_series[0])
    source_xyz = series_matrix(source_series)
    target_xyz = series_matrix(target_interp)
    error_norm = np.linalg.norm(source_xyz - target_xyz, axis=1)
    return list(source_series[0]), error_norm.tolist()


def save_mode2_tracking_error_plots(cmd, corrected, actual, reference, nominal, trajectory, output_dir):
    corrected_at_pose = interpolate_series(corrected, actual[0])
    corrected_norm = np.linalg.norm(series_matrix(corrected_at_pose), axis=1)
    active_segments = build_active_segments(cmd, active_threshold=1e-4)

    actual_vs_reference_t, actual_vs_reference = compute_error_norm_series(actual, reference)
    actual_vs_nominal_t, actual_vs_nominal = compute_error_norm_series(actual, nominal)
    actual_vs_trajectory_t, actual_vs_trajectory = compute_error_norm_series(actual, trajectory)
    reference_vs_nominal_t, reference_vs_nominal = compute_error_norm_series(reference, nominal)

    mean_actual_vs_nominal = float(np.mean(actual_vs_nominal)) if actual_vs_nominal else float("nan")
    max_actual_vs_nominal = float(np.max(actual_vs_nominal)) if actual_vs_nominal else float("nan")
    mean_actual_vs_reference = float(np.mean(actual_vs_reference)) if actual_vs_reference else float("nan")
    max_reference_vs_nominal = float(np.max(reference_vs_nominal)) if reference_vs_nominal else float("nan")

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(actual[0], corrected_norm, label="|corrected_force|", linewidth=1.8)
    add_active_segment_spans([axes[0]], active_segments)
    axes[0].set_ylabel("Force (N)")
    axes[0].set_title("Mode2: Interaction Force Magnitude")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(
        actual_vs_nominal_t,
        np.array(actual_vs_nominal) * 1000.0,
        label="|actual - nominal|",
        linewidth=1.8,
    )
    axes[1].plot(
        actual_vs_reference_t,
        np.array(actual_vs_reference) * 1000.0,
        label="|actual - compliant_reference|",
        linewidth=1.6,
    )
    axes[1].plot(
        actual_vs_trajectory_t,
        np.array(actual_vs_trajectory) * 1000.0,
        label="|actual - trajectory|",
        linewidth=1.6,
    )
    axes[1].plot(
        reference_vs_nominal_t,
        np.array(reference_vs_nominal) * 1000.0,
        label="|reference - nominal|",
        linewidth=1.6,
        )
    if actual_vs_nominal:
        axes[1].axhline(
            mean_actual_vs_nominal * 1000.0,
            color="tab:red",
            linestyle="--",
            linewidth=1.5,
            label="mean |actual - nominal| = {:.2f} mm".format(mean_actual_vs_nominal * 1000.0),
        )
    add_active_segment_spans([axes[1]], active_segments)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Error / Offset (mm)")
    axes[1].set_title("Mode2: Tracking Error, Compliance Deviation, and Recovery")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode2_tracking_error.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(output_dir, "mode2_tracking_error_summary.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["metric", "value_mm"])
        writer.writerow(["mean_actual_vs_nominal", "" if np.isnan(mean_actual_vs_nominal) else "{:.6f}".format(mean_actual_vs_nominal * 1000.0)])
        writer.writerow(["max_actual_vs_nominal", "" if np.isnan(max_actual_vs_nominal) else "{:.6f}".format(max_actual_vs_nominal * 1000.0)])
        writer.writerow(["mean_actual_vs_reference", "" if np.isnan(mean_actual_vs_reference) else "{:.6f}".format(mean_actual_vs_reference * 1000.0)])
        writer.writerow(["max_reference_vs_nominal", "" if np.isnan(max_reference_vs_nominal) else "{:.6f}".format(max_reference_vs_nominal * 1000.0)])


def save_mode2_offset_plots(cmd, actual, reference, nominal, trajectory, output_dir):
    actual_at_t = actual[0]
    reference_at_actual = interpolate_series(reference, actual_at_t)
    nominal_at_actual = interpolate_series(nominal, actual_at_t)
    trajectory_at_actual = interpolate_series(trajectory, actual_at_t)
    active_segments = build_active_segments(cmd, active_threshold=1e-4)

    actual_xyz = series_matrix(actual)
    reference_xyz = series_matrix(reference_at_actual)
    nominal_xyz = series_matrix(nominal_at_actual)
    trajectory_xyz = series_matrix(trajectory_at_actual)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    for idx, axis in enumerate(AXES):
        ax = axes[idx]
        ax.plot(
            actual_at_t,
            (actual_xyz[:, idx] - nominal_xyz[:, idx]) * 1000.0,
            label="actual - nominal",
            linewidth=1.8,
        )
        ax.plot(
            actual_at_t,
            (reference_xyz[:, idx] - nominal_xyz[:, idx]) * 1000.0,
            label="reference - nominal",
            linewidth=1.6,
        )
        ax.plot(
            actual_at_t,
            (actual_xyz[:, idx] - trajectory_xyz[:, idx]) * 1000.0,
            label="actual - trajectory",
            linewidth=1.4,
        )
        ax.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
        ax.set_ylabel("{} offset (mm)".format(axis.upper()))
        ax.set_title("Mode2: {}-Axis Deviation and Recovery".format(axis.upper()))
        ax.grid(True, alpha=0.3)

    add_active_segment_spans(axes, active_segments)
    for ax in axes:
        ax.legend()
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode2_position_offset_xyz.png"), dpi=200)
    plt.close(fig)


def collect_mode1_sensitivity_samples(cmd, corrected, actual, active_threshold, force_threshold):
    segments = build_active_segments(cmd, active_threshold)
    pose_times = np.array(actual[0], dtype=float)
    pose_xyz = np.column_stack([actual[1], actual[2], actual[3]])

    samples = {axis: {"force": [], "disp_cm": []} for axis in AXES}
    if pose_times.size == 0 or not segments:
        return samples

    corrected_at_pose = interpolate_series(corrected, actual[0])
    corrected_xyz = np.column_stack([corrected_at_pose[1], corrected_at_pose[2], corrected_at_pose[3]])

    for start_t, end_t in segments:
        active_mask = (pose_times >= start_t) & (pose_times <= end_t)
        if not np.any(active_mask):
            continue

        segment_pose = pose_xyz[active_mask]
        baseline = segment_pose[0]
        displacement_cm = np.abs(segment_pose - baseline) * 100.0
        segment_force = np.abs(corrected_xyz[active_mask])

        for axis_idx, axis in enumerate(AXES):
            valid = segment_force[:, axis_idx] > force_threshold
            if not np.any(valid):
                continue
            samples[axis]["force"].extend(segment_force[valid, axis_idx].tolist())
            samples[axis]["disp_cm"].extend(displacement_cm[valid, axis_idx].tolist())

    return samples


def fit_sensitivity_cm_per_n(forces, displacements_cm):
    if len(forces) < 2:
        return None

    x = np.array(forces, dtype=float)
    y = np.array(displacements_cm, dtype=float)
    denom = float(np.dot(x, x))
    if denom <= 1e-12:
        return None

    slope = float(np.dot(x, y) / denom)
    y_fit = slope * x
    residual = float(np.sum((y - y_fit) ** 2))
    total = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - residual / total if total > 1e-12 else 1.0
    return {
        "slope_cm_per_n": slope,
        "r2": r2,
        "sample_count": len(forces),
        "max_disp_cm": float(np.max(y)),
        "mean_force_n": float(np.mean(x)),
    }


def save_mode1_sensitivity_timeseries(cmd, corrected, actual, output_dir):
    pose_times = np.array(actual[0], dtype=float)
    if pose_times.size == 0:
        return

    corrected_at_pose = interpolate_series(corrected, actual[0])
    corrected_xyz = np.column_stack([corrected_at_pose[1], corrected_at_pose[2], corrected_at_pose[3]])
    force_norm = np.linalg.norm(corrected_xyz, axis=1)
    active_segments = build_active_segments(cmd, active_threshold=1e-4)
    pose_xyz = np.column_stack([actual[1], actual[2], actual[3]])

    displacement_cm = np.full(pose_times.shape, np.nan, dtype=float)
    sensitivity_cm_per_n = np.full(pose_times.shape, np.nan, dtype=float)

    for start_t, end_t in active_segments:
        active_mask = (pose_times >= start_t) & (pose_times <= end_t)
        if not np.any(active_mask):
            continue

        segment_pose = pose_xyz[active_mask]
        baseline = segment_pose[0]
        segment_disp_cm = np.linalg.norm(segment_pose - baseline, axis=1) * 100.0
        segment_force = force_norm[active_mask]

        displacement_cm[active_mask] = segment_disp_cm
        valid = segment_force > 0.02
        segment_ratio = np.full(segment_force.shape, np.nan, dtype=float)
        segment_ratio[valid] = segment_disp_cm[valid] / segment_force[valid]
        sensitivity_cm_per_n[active_mask] = segment_ratio

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(pose_times, force_norm, label="|corrected_force|", linewidth=1.8)
    axes[0].plot(pose_times, displacement_cm, label="|displacement|", linewidth=1.8)
    axes[0].set_ylabel("N / cm")
    axes[0].set_title("Mode1: Resultant Force and Resultant Displacement")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(pose_times, sensitivity_cm_per_n, label="|displacement| / |force|", linewidth=1.8)
    axes[1].axhline(0.5, color="tab:red", linestyle="--", linewidth=1.5, label="requirement: 0.5 cm/N")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Sensitivity (cm/N)")
    axes[1].set_title("Mode1: Time-Series Sensitivity During Active Force Segments")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode1_force_displacement_ratio_timeseries.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(output_dir, "mode1_force_displacement_ratio_timeseries.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_s", "force_norm_n", "displacement_norm_cm", "sensitivity_cm_per_n"])
        for idx in range(len(pose_times)):
            writer.writerow(
                [
                    "{:.6f}".format(pose_times[idx]),
                    "{:.6f}".format(force_norm[idx]),
                    "" if np.isnan(displacement_cm[idx]) else "{:.6f}".format(displacement_cm[idx]),
                    "" if np.isnan(sensitivity_cm_per_n[idx]) else "{:.6f}".format(sensitivity_cm_per_n[idx]),
                ]
            )


def save_mode1_sensitivity_plots(cmd, corrected, actual, output_dir):
    samples = collect_mode1_sensitivity_samples(
        cmd,
        corrected,
        actual,
        active_threshold=1e-4,
        force_threshold=0.02,
    )

    summary = {}
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    for idx, axis in enumerate(AXES):
        ax = axes[idx]
        forces = samples[axis]["force"]
        displacements_cm = samples[axis]["disp_cm"]
        metrics = fit_sensitivity_cm_per_n(forces, displacements_cm)
        summary[axis] = metrics

        if metrics is None:
            ax.set_title("Axis {}: insufficient samples".format(axis.upper()))
            ax.set_xlabel("|Force| (N)")
            ax.set_ylabel("|Displacement| (cm)")
            ax.grid(True, alpha=0.3)
            continue

        ax.scatter(forces, displacements_cm, s=10, alpha=0.35, label="samples")
        fit_x = np.linspace(0.0, max(forces), 100)
        fit_y = metrics["slope_cm_per_n"] * fit_x
        ax.plot(
            fit_x,
            fit_y,
            color="tab:red",
            linewidth=2.0,
            label="fit: {:.3f} cm/N".format(metrics["slope_cm_per_n"]),
        )
        ax.set_title("Axis {}".format(axis.upper()))
        ax.set_xlabel("|Force| (N)")
        ax.set_ylabel("|Displacement| (cm)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.text(
            0.03,
            0.97,
            "sensitivity={:.3f} cm/N\nR^2={:.3f}\nsamples={}".format(
                metrics["slope_cm_per_n"],
                metrics["r2"],
                metrics["sample_count"],
            ),
            transform=ax.transAxes,
            va="top",
            ha="left",
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
        )

    fig.suptitle("Mode1: Force-Displacement Sensitivity by Axis", fontsize=16)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode1_force_displacement_sensitivity.png"), dpi=200)
    plt.close(fig)

    valid_axes = [axis for axis in AXES if summary[axis] is not None]
    if valid_axes:
        fig, ax = plt.subplots(figsize=(8, 5))
        bar_values = [summary[axis]["slope_cm_per_n"] for axis in valid_axes]
        bars = ax.bar(valid_axes, bar_values, color=["#2a6f97", "#f4a261", "#2b9348"][: len(valid_axes)])
        ax.axhline(0.5, color="tab:red", linestyle="--", linewidth=1.5, label="requirement: 0.5 cm/N")
        ax.set_ylabel("Sensitivity (cm/N)")
        ax.set_title("Mode1: Sensitivity Summary")
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend()
        for bar, value in zip(bars, bar_values):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height(),
                "{:.3f}".format(value),
                ha="center",
                va="bottom",
            )
        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, "mode1_sensitivity_summary.png"), dpi=200)
        plt.close(fig)

    with open(os.path.join(output_dir, "mode1_sensitivity_summary.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["axis", "sensitivity_cm_per_n", "r2", "sample_count", "max_displacement_cm", "mean_force_n"])
        for axis in AXES:
            metrics = summary[axis]
            if metrics is None:
                writer.writerow([axis, "", "", 0, "", ""])
                continue
            writer.writerow(
                [
                    axis,
                    "{:.6f}".format(metrics["slope_cm_per_n"]),
                    "{:.6f}".format(metrics["r2"]),
                    metrics["sample_count"],
                    "{:.6f}".format(metrics["max_disp_cm"]),
                    "{:.6f}".format(metrics["mean_force_n"]),
                ]
            )


def save_force_position_plot(cmd, actual, reference, nominal, axis, output_path, title_prefix):
    idx = AXIS_INDEX[axis]

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(cmd[0], cmd[idx], label="command_force_{}".format(axis), linewidth=1.8)
    axes[0].set_ylabel("Force {} (N)".format(axis.upper()))
    axes[0].set_title("{}: Force Pulse on {} Axis".format(title_prefix, axis.upper()))
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(actual[0], actual[idx], label="end_effector_{}".format(axis), linewidth=1.8)
    axes[1].plot(reference[0], reference[idx], label="reference_{}".format(axis), linewidth=1.5)
    axes[1].plot(nominal[0], nominal[idx], label="nominal_{}".format(axis), linewidth=1.5)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Position {} (m)".format(axis.upper()))
    axes[1].set_title("{}: Compliance Response on {} Axis".format(title_prefix, axis.upper()))
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def save_mode2_command_force_plot(cmd, corrected, output_dir):
    corrected_at_cmd = interpolate_series(corrected, cmd[0]) if cmd[0] else corrected
    active_segments = build_active_segments(cmd, active_threshold=1e-4)

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    for idx, axis in enumerate(AXES):
        series_idx = AXIS_INDEX[axis]
        axes[idx].plot(cmd[0], cmd[series_idx], label="command_force_{}".format(axis), linewidth=1.8)
        axes[idx].plot(
            corrected_at_cmd[0],
            corrected_at_cmd[series_idx],
            label="corrected_force_{}".format(axis),
            linewidth=1.5,
        )
        axes[idx].set_ylabel("F{} (N)".format(axis.upper()))
        axes[idx].set_title("Mode2: Command and Corrected Force on {} Axis".format(axis.upper()))
        axes[idx].grid(True, alpha=0.3)

    command_norm = vector_norm_series(cmd) if cmd[0] else []
    corrected_norm = vector_norm_series(corrected_at_cmd) if corrected_at_cmd[0] else []
    axes[3].plot(cmd[0], command_norm, label="|command_force|", linewidth=1.8)
    axes[3].plot(corrected_at_cmd[0], corrected_norm, label="|corrected_force|", linewidth=1.5)
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("|F| (N)")
    axes[3].set_title("Mode2: Resultant Force Magnitude")
    axes[3].grid(True, alpha=0.3)

    add_active_segment_spans(axes, active_segments)
    for ax in axes:
        ax.legend()

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode2_command_force_xyz.png"), dpi=200)
    plt.close(fig)


def write_mode2_data_summary(cmd, corrected, actual, reference, nominal, trajectory, output_dir):
    active_segments = build_active_segments(cmd, active_threshold=1e-4)
    command_norm = vector_norm_series(cmd) if cmd[0] else []
    corrected_norm = vector_norm_series(corrected) if corrected[0] else []

    rows = [
        ("command_samples", len(cmd[0])),
        ("corrected_samples", len(corrected[0])),
        ("actual_pose_samples", len(actual[0])),
        ("reference_pose_samples", len(reference[0])),
        ("nominal_pose_samples", len(nominal[0])),
        ("trajectory_pose_samples", len(trajectory[0])),
        ("command_active_segments", len(active_segments)),
        ("max_command_force_n", "" if not command_norm else "{:.6f}".format(float(np.max(command_norm)))),
        ("max_corrected_force_n", "" if not corrected_norm else "{:.6f}".format(float(np.max(corrected_norm)))),
    ]

    with open(os.path.join(output_dir, "mode2_data_summary.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["metric", "value"])
        writer.writerows(rows)

    if not cmd[0]:
        return

    actual_vs_nominal_t, actual_vs_nominal = compute_error_norm_series(actual, nominal)
    if not actual_vs_nominal_t:
        return

    error_t = np.array(actual_vs_nominal_t, dtype=float)
    error_mm = np.array(actual_vs_nominal, dtype=float) * 1000.0

    with open(os.path.join(output_dir, "mode2_pulse_response_summary.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["segment", "start_s", "end_s", "peak_actual_vs_nominal_mm", "settled_actual_vs_nominal_mm"])
        for segment_idx, (start_t, end_t) in enumerate(active_segments, start=1):
            active_mask = (error_t >= start_t) & (error_t <= end_t)
            settle_mask = (error_t > end_t) & (error_t <= end_t + 2.0)
            peak_error = float(np.max(error_mm[active_mask])) if np.any(active_mask) else float("nan")
            settled_error = float(np.min(error_mm[settle_mask])) if np.any(settle_mask) else float("nan")
            writer.writerow(
                [
                    segment_idx,
                    "{:.6f}".format(start_t),
                    "{:.6f}".format(end_t),
                    "" if np.isnan(peak_error) else "{:.6f}".format(peak_error),
                    "" if np.isnan(settled_error) else "{:.6f}".format(settled_error),
                ]
            )


def save_mode1_plots(bag_path, output_dir):
    with rosbag.Bag(bag_path, "r") as bag:
        cmd = collect_wrench_series(bag, "/ur5/ft_sensor/command_wrench")
        corrected = collect_wrench_series(bag, "/ur5/ft_sensor/wrench_corrected")
        actual = collect_pose_series(bag, "/ur5/end_effector_pose")
        reference = collect_pose_series(bag, "/ur5/active_rehab/reference_pose")
        nominal = collect_pose_series(bag, "/ur5/active_rehab/nominal_reference_pose")

    cmd, corrected, actual, reference, nominal = normalize_time(cmd, corrected, actual, reference, nominal)

    for axis in ("x", "y", "z"):
        save_force_position_plot(
            cmd,
            actual,
            reference,
            nominal,
            axis,
            os.path.join(output_dir, "mode1_force_position_{}.png".format(axis)),
            "Mode1",
        )

    save_mode1_sensitivity_timeseries(cmd, corrected, actual, output_dir)
    save_mode1_sensitivity_plots(cmd, corrected, actual, output_dir)


def save_mode2_plots(bag_path, output_dir):
    with rosbag.Bag(bag_path, "r") as bag:
        cmd = collect_wrench_series(bag, "/ur5/ft_sensor/command_wrench")
        corrected = collect_wrench_series(bag, "/ur5/ft_sensor/wrench_corrected")
        actual = collect_pose_series(bag, "/ur5/end_effector_pose")
        reference = collect_pose_series(bag, "/ur5/active_rehab/reference_pose")
        nominal = collect_pose_series(bag, "/ur5/active_rehab/nominal_reference_pose")
        trajectory = collect_pose_series(bag, "/ur5/active_rehab/trajectory_reference")

    cmd, corrected, actual, reference, nominal, trajectory = normalize_time(
        cmd, corrected, actual, reference, nominal, trajectory
    )

    corrected_at_cmd = interpolate_series(corrected, cmd[0]) if cmd[0] else corrected
    active_segments = build_active_segments(cmd, active_threshold=1e-4)

    maybe_warn_missing_series(cmd, "/ur5/ft_sensor/command_wrench")
    maybe_warn_missing_series(corrected, "/ur5/ft_sensor/wrench_corrected")
    maybe_warn_missing_series(actual, "/ur5/end_effector_pose")
    maybe_warn_missing_series(reference, "/ur5/active_rehab/reference_pose")
    maybe_warn_missing_series(nominal, "/ur5/active_rehab/nominal_reference_pose")
    maybe_warn_missing_series(trajectory, "/ur5/active_rehab/trajectory_reference")

    write_mode2_data_summary(cmd, corrected, actual, reference, nominal, trajectory, output_dir)
    save_mode2_command_force_plot(cmd, corrected, output_dir)

    for axis in ("x", "y", "z"):
        idx = AXIS_INDEX[axis]
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        axes[0].plot(cmd[0], cmd[idx], label="command_force_{}".format(axis), linewidth=1.8)
        axes[0].plot(
            corrected_at_cmd[0],
            corrected_at_cmd[idx],
            label="corrected_force_{}".format(axis),
            linewidth=1.6,
        )
        axes[0].set_ylabel("Force {} (N)".format(axis.upper()))
        axes[0].set_title("Mode2: Force Excitation and Measured Interaction on {} Axis".format(axis.upper()))
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(actual[0], actual[idx], label="end_effector_{}".format(axis), linewidth=1.8)
        axes[1].plot(reference[0], reference[idx], label="reference_{}".format(axis), linewidth=1.5)
        axes[1].plot(nominal[0], nominal[idx], label="nominal_{}".format(axis), linewidth=1.5)
        axes[1].plot(trajectory[0], trajectory[idx], label="trajectory_{}".format(axis), linewidth=1.5)
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Position {} (m)".format(axis.upper()))
        axes[1].set_title("Mode2: End-Effector Deviation, Compliance, and Recovery on {} Axis".format(axis.upper()))
        axes[1].grid(True, alpha=0.3)

        add_active_segment_spans(axes, active_segments)
        axes[0].legend()
        axes[1].legend()

        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, "mode2_force_position_{}.png".format(axis)), dpi=200)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(trajectory[2], trajectory[3], label="trajectory_yz", linewidth=1.5)
    ax.plot(reference[2], reference[3], label="reference_yz", linewidth=1.5)
    ax.plot(actual[2], actual[3], label="end_effector_yz", linewidth=1.8)
    ax.set_xlabel("Y (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Mode2: YZ Trajectory View")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "mode2_yz_trajectory.png"), dpi=200)
    plt.close(fig)

    save_mode2_tracking_error_plots(cmd, corrected, actual, reference, nominal, trajectory, output_dir)
    save_mode2_offset_plots(cmd, actual, reference, nominal, trajectory, output_dir)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--mode", choices=["mode1", "mode2"], required=True)
    parser.add_argument("--output-dir", required=True, help="Directory for saved plots")
    args = parser.parse_args()

    ensure_dir(args.output_dir)
    if args.mode == "mode1":
        save_mode1_plots(args.bag, args.output_dir)
    else:
        save_mode2_plots(args.bag, args.output_dir)


if __name__ == "__main__":
    main()
