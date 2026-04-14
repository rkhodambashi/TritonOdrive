import csv
import datetime
import threading
import time
import tkinter as tk
from collections import deque
from math import acos, atan, cos, pi, sin, sqrt
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import matplotlib
from skyfield.api import EarthSatellite, Topos, load, utc
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

matplotlib.use("TkAgg")

EPS = 1e-10
TRACK_COMMAND_INTERVAL_SEC = 0.05
TRAJECTORY_HORIZON_SEC = 2.0
TRAJECTORY_POINT_SPACING_SEC = 0.05
TRAJECTORY_REBUILD_MARGIN_SEC = 0.25
PREPOINT_LEAD_TIME_SEC = 30.0
MAX_ANGLE_STEP_DEG = 0.05
MIN_TRAJECTORY_SPACING_SEC = 0.01
TRACK_FEEDFORWARD_LEAD_SEC = 0.05
TRACK_CAPTURE_ERROR_DEG = 0.5
TRACK_FILTER_BANDWIDTH_HZ = 20.0
TRACK_USE_FILTER = False
TRACK_USE_LEAD = False
TRACK_USE_PASSTHROUGH = True
TRACK_USE_TIME_SPLIT = False
TRACK_USE_VELOCITY_MODE = True
TRACK_MAX_DEGREE = 91.0
TRACK_MIN_DEGREE = -91.0
TRACK_VEL_FEEDFORWARD_SCALE_X = 1.0
TRACK_VEL_FEEDFORWARD_SCALE_Y = 1.0
TRACK_SPI_POSITION_CORRECTION_GAIN_X = 8.0
TRACK_SPI_POSITION_CORRECTION_GAIN_Y = 8.0
TRACK_SPI_INTEGRAL_GAIN_X = 2.0
TRACK_SPI_INTEGRAL_GAIN_Y = 2.0
TRACK_SPI_INTEGRAL_ACTIVE_ERROR_DEG = 0.5
TRACK_SPI_INTEGRAL_MAX_STATE_X = 10.0
TRACK_SPI_INTEGRAL_MAX_STATE_Y = 10.0
TRACK_SPI_INTEGRAL_MAX_CORRECTION_X = 10.0
TRACK_SPI_INTEGRAL_MAX_CORRECTION_Y = 10.0
TRACK_SPI_DERIVATIVE_GAIN_X = 0.3
TRACK_SPI_DERIVATIVE_GAIN_Y = 0.0
TRACK_SPI_DERIVATIVE_FILTER_ALPHA_X = 0.8
TRACK_SPI_DERIVATIVE_FILTER_ALPHA_Y = 0.6
TRACK_SPI_DERIVATIVE_MAX_CORRECTION_X = 0.5
TRACK_SPI_DERIVATIVE_MAX_CORRECTION_Y = 0.15
TRACK_VELOCITY_POSITION_GAIN_X = 10.0
TRACK_VELOCITY_POSITION_GAIN_Y = 10.0
TRACK_VELOCITY_INTEGRAL_GAIN_X = 1.5
TRACK_VELOCITY_INTEGRAL_GAIN_Y = 1.5
TRACK_VELOCITY_INTEGRAL_MAX_STATE_X = 10.0
TRACK_VELOCITY_INTEGRAL_MAX_STATE_Y = 10.0
TRACK_VELOCITY_ERROR_GAIN_X = 0.4
TRACK_VELOCITY_ERROR_GAIN_Y = 0.4
TRACK_SPI_VELOCITY_FILTER_ALPHA_X = 0.7
TRACK_SPI_VELOCITY_FILTER_ALPHA_Y = 0.7
TRACK_VELOCITY_COMMAND_MAX_X = 100.0
TRACK_VELOCITY_COMMAND_MAX_Y = 100.0
TRACK_VELOCITY_CONTROLLER_MODE = "full"
TRACK_VELOCITY_DEBUG_SCALE = 1.0


def get_local_timezone():
    return datetime.datetime.now().astimezone().tzinfo


def format_local_datetime(dt_utc, include_seconds=False):
    if dt_utc is None:
        return "-"
    local_dt = dt_utc.astimezone(get_local_timezone())
    if include_seconds:
        return local_dt.strftime("%Y-%m-%d %I:%M:%S %p")
    return local_dt.strftime("%Y-%m-%d %I:%M %p")


def Kvector(AZ, EL):
    azRad = AZ * pi / 180.0
    elRad = EL * pi / 180.0
    x = cos(elRad) * cos(azRad)
    y = -cos(elRad) * sin(azRad)
    z = sin(elRad)
    x = 0 if abs(x) < EPS else x
    y = 0 if abs(y) < EPS else y
    z = 0 if abs(z) < EPS else z
    return [x, y, z]


def Xvector(AZ, EL):
    k = Kvector(AZ, EL)
    xvec = [0, k[1], k[2]]
    norm = sqrt(xvec[1] ** 2 + xvec[2] ** 2)
    if norm != 0:
        xvec[1] /= norm
        xvec[2] /= norm
    xvec[1] = 0 if abs(xvec[1]) < EPS else xvec[1]
    xvec[2] = 0 if abs(xvec[2]) < EPS else xvec[2]
    return xvec


def Xangle(AZ, EL):
    xvec = Xvector(AZ, EL)
    if xvec[2] == 0:
        if xvec[1] > 0:
            angleRad = pi / 2
        elif xvec[1] < 0:
            angleRad = -pi / 2
        else:
            angleRad = 0
    else:
        angleRad = atan(xvec[1] / xvec[2])
        if xvec[2] < 0:
            angleRad += pi
    return angleRad * 180.0 / pi


def Yangle(AZ, EL):
    kvec = Kvector(AZ, EL)
    xvec = Xvector(AZ, EL)
    dotProd = kvec[0] * xvec[0] + kvec[1] * xvec[1] + kvec[2] * xvec[2]
    dotProd = max(min(dotProd, 1), -1)
    angleRad = acos(dotProd)
    if kvec[0] < 0:
        angleRad = -angleRad
    return angleRad * 180.0 / pi


def update_odrive_axes(x_angle_deg, y_angle_deg, control):
    x_angle_deg = max(min(x_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE)
    y_angle_deg = max(min(y_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE)
    try:
        control.command_absolute_pair(x_deg=x_angle_deg, y_deg=y_angle_deg)
    except Exception as e:
        print(f"ODrive move error: {e}")


def update_odrive_axes_with_velocity(x_angle_deg, y_angle_deg, x_vel_deg_s, y_vel_deg_s, control):
    x_angle_deg = max(min(x_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE)
    y_angle_deg = max(min(y_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE)
    try:
        control.command_absolute_pair_with_velocity(
            x_deg=x_angle_deg,
            y_deg=y_angle_deg,
            x_vel_deg_s=x_vel_deg_s,
            y_vel_deg_s=y_vel_deg_s,
        )
    except Exception as e:
        print(f"ODrive move error: {e}")


def update_odrive_axes_velocity_only(x_vel_deg_s, y_vel_deg_s, control):
    try:
        control.command_velocity_pair(
            x_vel_deg_s=max(min(x_vel_deg_s, TRACK_VELOCITY_COMMAND_MAX_X), -TRACK_VELOCITY_COMMAND_MAX_X),
            y_vel_deg_s=max(min(y_vel_deg_s, TRACK_VELOCITY_COMMAND_MAX_Y), -TRACK_VELOCITY_COMMAND_MAX_Y),
        )
    except Exception as e:
        print(f"ODrive velocity command error: {e}")


def clamp_tracking_angles(x_angle_deg, y_angle_deg):
    return (
        max(min(x_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE),
        max(min(y_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE),
    )


def shorten_display_path(path_text, max_len=36):
    if not path_text:
        return ""
    name = Path(path_text).name
    if len(name) <= max_len:
        return name
    return f"{name[:max_len-3]}..."


def limit_command_step(previous_command, target_command, max_angle_step_deg):
    if previous_command is None or max_angle_step_deg <= 0:
        return target_command

    dx = target_command[0] - previous_command[0]
    dy = target_command[1] - previous_command[1]
    max_delta = max(abs(dx), abs(dy))
    if max_delta <= max_angle_step_deg:
        return target_command

    scale = max_angle_step_deg / max_delta
    return (
        previous_command[0] + dx * scale,
        previous_command[1] + dy * scale,
    )


def command_step_exceeds(previous_command, target_command, max_angle_step_deg):
    if previous_command is None or max_angle_step_deg <= 0:
        return False
    return max(
        abs(target_command[0] - previous_command[0]),
        abs(target_command[1] - previous_command[1]),
    ) > max_angle_step_deg


def build_pointing_sample(ts, observer, sat, sample_time_utc, offset_sec):
    difference = sat - observer
    t = ts.from_datetime(sample_time_utc)
    topocentric = difference.at(t)
    el, az, distance = topocentric.altaz()

    az_deg = az.degrees
    el_deg = el.degrees

    return {
        "offset_sec": offset_sec,
        "az_deg": az_deg,
        "el_deg": el_deg,
        "x_angle": Xangle(az_deg, el_deg),
        "y_angle": Yangle(az_deg, el_deg),
        "visible": el_deg >= 0,
    }


def needs_refinement(left_point, right_point, max_angle_step_deg):
    return (
        abs(right_point["x_angle"] - left_point["x_angle"]) > max_angle_step_deg
        or abs(right_point["y_angle"] - left_point["y_angle"]) > max_angle_step_deg
    )


def refine_segment(ts, observer, sat, start_time_utc, left_point, right_point, max_angle_step_deg, min_spacing_sec):
    segment_span = right_point["offset_sec"] - left_point["offset_sec"]
    if segment_span <= min_spacing_sec:
        return [left_point]

    if not needs_refinement(left_point, right_point, max_angle_step_deg):
        return [left_point]

    midpoint_offset = left_point["offset_sec"] + segment_span / 2.0
    midpoint_time = start_time_utc + datetime.timedelta(seconds=midpoint_offset)
    midpoint = build_pointing_sample(ts, observer, sat, midpoint_time, midpoint_offset)

    left_samples = refine_segment(
        ts,
        observer,
        sat,
        start_time_utc,
        left_point,
        midpoint,
        max_angle_step_deg,
        min_spacing_sec,
    )
    right_samples = refine_segment(
        ts,
        observer,
        sat,
        start_time_utc,
        midpoint,
        right_point,
        max_angle_step_deg,
        min_spacing_sec,
    )

    return left_samples + right_samples


def build_tracking_trajectory(ts, observer, sat, start_time_utc, horizon_sec, spacing_sec, max_angle_step_deg, min_spacing_sec):
    coarse_points = []

    steps = max(2, int(horizon_sec / spacing_sec) + 1)
    for index in range(steps):
        offset_sec = index * spacing_sec
        sample_time = start_time_utc + datetime.timedelta(seconds=offset_sec)
        coarse_points.append(build_pointing_sample(ts, observer, sat, sample_time, offset_sec))

    refined_points = []
    for left_point, right_point in zip(coarse_points, coarse_points[1:]):
        refined_points.extend(
            refine_segment(
                ts,
                observer,
                sat,
                start_time_utc,
                left_point,
                right_point,
                max_angle_step_deg,
                min_spacing_sec,
            )
        )

    refined_points.append(coarse_points[-1])
    return refined_points


def sample_tracking_trajectory(points, elapsed_sec):
    if not points:
        return None

    if elapsed_sec <= points[0]["offset_sec"]:
        return points[0]

    if elapsed_sec >= points[-1]["offset_sec"]:
        return points[-1]

    for left, right in zip(points, points[1:]):
        if left["offset_sec"] <= elapsed_sec <= right["offset_sec"]:
            span = right["offset_sec"] - left["offset_sec"]
            alpha = 0.0 if span <= 0 else (elapsed_sec - left["offset_sec"]) / span

            return {
                "az_deg": left["az_deg"] + (right["az_deg"] - left["az_deg"]) * alpha,
                "el_deg": left["el_deg"] + (right["el_deg"] - left["el_deg"]) * alpha,
                "x_angle": left["x_angle"] + (right["x_angle"] - left["x_angle"]) * alpha,
                "y_angle": left["y_angle"] + (right["y_angle"] - left["y_angle"]) * alpha,
                "visible": left["visible"] or right["visible"],
            }

    return points[-1]


def sample_tracking_state(points, elapsed_sec, derivative_dt_sec):
    current = sample_tracking_trajectory(points, elapsed_sec)
    if current is None:
        return None

    if not points:
        current["x_vel"] = 0.0
        current["y_vel"] = 0.0
        current["x_acc"] = 0.0
        current["y_acc"] = 0.0
        return current

    max_offset = points[-1]["offset_sec"]
    dt = max(1e-4, min(derivative_dt_sec, max_offset if max_offset > 0 else derivative_dt_sec))
    prev_t = max(0.0, elapsed_sec - dt)
    next_t = min(max_offset, elapsed_sec + dt)
    actual_dt = max(1e-4, next_t - prev_t)

    prev_sample = sample_tracking_trajectory(points, prev_t)
    next_sample = sample_tracking_trajectory(points, next_t)

    x_vel = (next_sample["x_angle"] - prev_sample["x_angle"]) / actual_dt
    y_vel = (next_sample["y_angle"] - prev_sample["y_angle"]) / actual_dt

    center_dt = max(1e-4, actual_dt / 2.0)
    x_acc = (next_sample["x_angle"] - 2.0 * current["x_angle"] + prev_sample["x_angle"]) / (center_dt * center_dt)
    y_acc = (next_sample["y_angle"] - 2.0 * current["y_angle"] + prev_sample["y_angle"]) / (center_dt * center_dt)

    current["x_vel"] = x_vel
    current["y_vel"] = y_vel
    current["x_acc"] = x_acc
    current["y_acc"] = y_acc
    return current


def sample_tracking_state_continuous(
    ts,
    observer,
    sat,
    trajectory_start_utc,
    trajectory_origin_elapsed_sec,
    trajectory_end_elapsed_sec,
    absolute_elapsed_sec,
    derivative_dt_sec,
):
    clamped_absolute_elapsed_sec = min(max(trajectory_origin_elapsed_sec, absolute_elapsed_sec), trajectory_end_elapsed_sec)
    local_elapsed_sec = clamped_absolute_elapsed_sec - trajectory_origin_elapsed_sec
    sample_time_utc = trajectory_start_utc + datetime.timedelta(seconds=local_elapsed_sec)
    current = build_pointing_sample(ts, observer, sat, sample_time_utc, local_elapsed_sec)

    dt = max(1e-4, derivative_dt_sec)
    prev_elapsed_sec = max(trajectory_origin_elapsed_sec, clamped_absolute_elapsed_sec - dt)
    next_elapsed_sec = min(trajectory_end_elapsed_sec, clamped_absolute_elapsed_sec + dt)
    actual_dt = max(1e-4, next_elapsed_sec - prev_elapsed_sec)

    prev_local_elapsed_sec = prev_elapsed_sec - trajectory_origin_elapsed_sec
    next_local_elapsed_sec = next_elapsed_sec - trajectory_origin_elapsed_sec
    prev_sample = build_pointing_sample(
        ts,
        observer,
        sat,
        trajectory_start_utc + datetime.timedelta(seconds=prev_local_elapsed_sec),
        prev_local_elapsed_sec,
    )
    next_sample = build_pointing_sample(
        ts,
        observer,
        sat,
        trajectory_start_utc + datetime.timedelta(seconds=next_local_elapsed_sec),
        next_local_elapsed_sec,
    )

    current["x_vel"] = (next_sample["x_angle"] - prev_sample["x_angle"]) / actual_dt
    current["y_vel"] = (next_sample["y_angle"] - prev_sample["y_angle"]) / actual_dt

    center_dt = max(1e-4, actual_dt / 2.0)
    current["x_acc"] = (next_sample["x_angle"] - 2.0 * current["x_angle"] + prev_sample["x_angle"]) / (center_dt * center_dt)
    current["y_acc"] = (next_sample["y_angle"] - 2.0 * current["y_angle"] + prev_sample["y_angle"]) / (center_dt * center_dt)
    return current


def sample_tracking_state_utc(ts, observer, sat, sample_time_utc, derivative_dt_sec):
    current = build_pointing_sample(ts, observer, sat, sample_time_utc, 0.0)

    dt = max(1e-4, derivative_dt_sec)
    prev_sample = build_pointing_sample(
        ts,
        observer,
        sat,
        sample_time_utc - datetime.timedelta(seconds=dt),
        -dt,
    )
    next_sample = build_pointing_sample(
        ts,
        observer,
        sat,
        sample_time_utc + datetime.timedelta(seconds=dt),
        dt,
    )

    actual_dt = max(1e-4, 2.0 * dt)
    current["x_vel"] = (next_sample["x_angle"] - prev_sample["x_angle"]) / actual_dt
    current["y_vel"] = (next_sample["y_angle"] - prev_sample["y_angle"]) / actual_dt

    center_dt = max(1e-4, dt)
    current["x_acc"] = (next_sample["x_angle"] - 2.0 * current["x_angle"] + prev_sample["x_angle"]) / (center_dt * center_dt)
    current["y_acc"] = (next_sample["y_angle"] - 2.0 * current["y_angle"] + prev_sample["y_angle"]) / (center_dt * center_dt)
    return current


def load_replay_reference_csv(file_path):
    samples = []
    with open(file_path, newline="", encoding="utf-8") as replay_file:
        reader = csv.DictReader(replay_file)
        for row in reader:
            try:
                samples.append(
                    {
                        "utc_time": datetime.datetime.fromisoformat(row["utc_time"]),
                        "elapsed_sec": float(row["elapsed_sec"]),
                        "visible": bool(int(row["sample_visible"])),
                        "tracking_started": bool(int(row.get("tracking_started", "0"))),
                        "az_deg": float(row["az_deg"]),
                        "el_deg": float(row["el_deg"]),
                        "raw_x_angle": float(row["raw_x_angle_deg"]),
                        "raw_y_angle": float(row["raw_y_angle_deg"]),
                        "x_angle": float(row["cmd_x_angle_deg"]),
                        "y_angle": float(row["cmd_y_angle_deg"]),
                        "x_vel": float(row["cmd_x_vel_ff_deg_per_s"]),
                        "y_vel": float(row["cmd_y_vel_ff_deg_per_s"]),
                        "x_acc": float(row["x_acc_deg_per_s2"]),
                        "y_acc": float(row["y_acc_deg_per_s2"]),
                        "status": row.get("status", ""),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue

    samples.sort(key=lambda sample: sample["elapsed_sec"])
    # Trim any obviously invalid terminal jump that was captured in the source log.
    while len(samples) >= 2:
        previous = samples[-2]
        current = samples[-1]
        if (
            abs(current["x_angle"] - previous["x_angle"]) > 30.0
            or abs(current["y_angle"] - previous["y_angle"]) > 30.0
        ):
            samples.pop()
            continue
        break
    return samples


def interpolate_replay_sample_utc(samples, elapsed_sec):
    if not samples:
        return None

    if elapsed_sec <= samples[0]["elapsed_sec"]:
        return samples[0]["utc_time"]

    if elapsed_sec >= samples[-1]["elapsed_sec"]:
        return None

    left = samples[0]
    right = samples[-1]
    for candidate_left, candidate_right in zip(samples, samples[1:]):
        if candidate_left["elapsed_sec"] <= elapsed_sec <= candidate_right["elapsed_sec"]:
            left = candidate_left
            right = candidate_right
            break

    span = max(1e-9, right["elapsed_sec"] - left["elapsed_sec"])
    alpha = max(0.0, min(1.0, (elapsed_sec - left["elapsed_sec"]) / span))
    dt = right["utc_time"] - left["utc_time"]
    return left["utc_time"] + datetime.timedelta(seconds=dt.total_seconds() * alpha)


def sample_replay_tracking_state(samples, elapsed_sec):
    if not samples:
        return None

    if elapsed_sec <= samples[0]["elapsed_sec"]:
        return dict(samples[0])

    if elapsed_sec >= samples[-1]["elapsed_sec"]:
        return None

    left = samples[0]
    right = samples[-1]
    for candidate_left, candidate_right in zip(samples, samples[1:]):
        if candidate_left["elapsed_sec"] <= elapsed_sec <= candidate_right["elapsed_sec"]:
            left = candidate_left
            right = candidate_right
            break

    span = max(1e-9, right["elapsed_sec"] - left["elapsed_sec"])
    alpha = max(0.0, min(1.0, (elapsed_sec - left["elapsed_sec"]) / span))

    def lerp(field):
        return left[field] + (right[field] - left[field]) * alpha

    return {
        "elapsed_sec": elapsed_sec,
        "visible": left["visible"] if alpha < 0.5 else right["visible"],
        "tracking_started": left["tracking_started"] if alpha < 0.5 else right["tracking_started"],
        "az_deg": lerp("az_deg"),
        "el_deg": lerp("el_deg"),
        "raw_x_angle": lerp("raw_x_angle"),
        "raw_y_angle": lerp("raw_y_angle"),
        "x_angle": lerp("x_angle"),
        "y_angle": lerp("y_angle"),
        "x_vel": lerp("x_vel"),
        "y_vel": lerp("y_vel"),
        "x_acc": lerp("x_acc"),
        "y_acc": lerp("y_acc"),
        "status": left["status"] if alpha < 0.5 else right["status"],
    }


def sample_replay_tracking_state_utc(ts, observer, sat, replay_samples, elapsed_sec, derivative_dt_sec):
    sample_time_utc = interpolate_replay_sample_utc(replay_samples, elapsed_sec)
    if sample_time_utc is None:
        return None

    state = sample_tracking_state_utc(ts, observer, sat, sample_time_utc, derivative_dt_sec)
    state["elapsed_sec"] = elapsed_sec
    return state


def sample_limited_tracking_target_continuous(
    ts,
    observer,
    sat,
    trajectory_start_utc,
    trajectory_origin_elapsed_sec,
    trajectory_end_elapsed_sec,
    previous_target_elapsed_sec,
    target_elapsed_sec,
    derivative_dt_sec,
    previous_command=None,
    max_angle_step_deg=0.0,
):
    target_state = sample_tracking_state_continuous(
        ts,
        observer,
        sat,
        trajectory_start_utc,
        trajectory_origin_elapsed_sec,
        trajectory_end_elapsed_sec,
        target_elapsed_sec,
        derivative_dt_sec,
    )
    if target_state is None:
        return None, target_elapsed_sec

    search_start_elapsed_sec = max(
        trajectory_origin_elapsed_sec,
        previous_target_elapsed_sec if previous_target_elapsed_sec is not None else trajectory_origin_elapsed_sec,
    )

    if previous_command is None or max_angle_step_deg <= 0 or target_elapsed_sec <= search_start_elapsed_sec:
        return target_state, target_elapsed_sec

    target_command = (target_state["x_angle"], target_state["y_angle"])
    if not command_step_exceeds(previous_command, target_command, max_angle_step_deg):
        return target_state, target_elapsed_sec

    low_elapsed = search_start_elapsed_sec
    high_elapsed = target_elapsed_sec
    best_elapsed = search_start_elapsed_sec
    best_state = sample_tracking_state_continuous(
        ts,
        observer,
        sat,
        trajectory_start_utc,
        trajectory_origin_elapsed_sec,
        trajectory_end_elapsed_sec,
        search_start_elapsed_sec,
        derivative_dt_sec,
    )

    for _ in range(24):
        mid_elapsed = (low_elapsed + high_elapsed) / 2.0
        mid_state = sample_tracking_state_continuous(
            ts,
            observer,
            sat,
            trajectory_start_utc,
            trajectory_origin_elapsed_sec,
            trajectory_end_elapsed_sec,
            mid_elapsed,
            derivative_dt_sec,
        )
        mid_command = (mid_state["x_angle"], mid_state["y_angle"])
        if command_step_exceeds(previous_command, mid_command, max_angle_step_deg):
            high_elapsed = mid_elapsed
        else:
            low_elapsed = mid_elapsed
            best_elapsed = mid_elapsed
            best_state = mid_state

    return best_state, best_elapsed


def get_next_rise_time(ts, observer, sat, search_hours=24):
    t0 = ts.now()
    t1 = ts.from_datetime(datetime.datetime.utcnow().replace(tzinfo=utc) + datetime.timedelta(hours=search_hours))

    try:
        times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)
    except Exception:
        return None

    for ti, event in zip(times, events):
        if event == 0:
            return ti.utc_datetime().replace(tzinfo=utc)

    return None


def get_pointing_sample(ts, observer, sat, sample_time_utc):
    t = ts.from_datetime(sample_time_utc)
    difference = sat - observer
    topocentric = difference.at(t)
    el, az, distance = topocentric.altaz()

    az_deg = az.degrees
    el_deg = el.degrees

    return {
        "az_deg": az_deg,
        "el_deg": el_deg,
        "x_angle": Xangle(az_deg, el_deg),
        "y_angle": Yangle(az_deg, el_deg),
        "visible": el_deg >= 0,
    }


class SatelliteTrackingWindow(tk.Toplevel):
    def __init__(
        self,
        parent,
        odrvs=None,
        control=None,
        observer_lat=33.67,
        observer_lon=-112.09,
        preposition_gains=None,
        tracking_gains=None,
    ):
        super().__init__(parent)

        self.title("Satellite Tracking")
        self.geometry("1080x980")

        self.odrvs = odrvs or {}
        self.control = control
        self.preposition_gains = preposition_gains
        self.tracking_gains = tracking_gains

        self.observer_lat = observer_lat
        self.observer_lon = observer_lon
        self.track_command_interval_sec = TRACK_COMMAND_INTERVAL_SEC
        self.trajectory_horizon_sec = TRAJECTORY_HORIZON_SEC
        self.trajectory_point_spacing_sec = TRAJECTORY_POINT_SPACING_SEC
        self.max_angle_step_deg = MAX_ANGLE_STEP_DEG
        self.feedforward_lead_sec = TRACK_FEEDFORWARD_LEAD_SEC
        self.track_filter_bandwidth_hz = TRACK_FILTER_BANDWIDTH_HZ
        self.track_capture_error_deg = TRACK_CAPTURE_ERROR_DEG
        self.trajectory_rebuild_margin_sec = TRAJECTORY_REBUILD_MARGIN_SEC
        self.prepoint_lead_time_sec = PREPOINT_LEAD_TIME_SEC
        self.min_trajectory_spacing_sec = MIN_TRAJECTORY_SPACING_SEC
        self.velocity_position_gain_x = TRACK_VELOCITY_POSITION_GAIN_X
        self.velocity_position_gain_y = TRACK_VELOCITY_POSITION_GAIN_Y
        self.velocity_integral_gain_x = TRACK_VELOCITY_INTEGRAL_GAIN_X
        self.velocity_integral_gain_y = TRACK_VELOCITY_INTEGRAL_GAIN_Y
        self.velocity_error_gain_x = TRACK_VELOCITY_ERROR_GAIN_X
        self.velocity_error_gain_y = TRACK_VELOCITY_ERROR_GAIN_Y
        self.spi_velocity_filter_alpha_x = TRACK_SPI_VELOCITY_FILTER_ALPHA_X
        self.spi_velocity_filter_alpha_y = TRACK_SPI_VELOCITY_FILTER_ALPHA_Y
        self.velocity_command_max_x = TRACK_VELOCITY_COMMAND_MAX_X
        self.velocity_command_max_y = TRACK_VELOCITY_COMMAND_MAX_Y
        self.velocity_feedforward_scale_x = TRACK_VEL_FEEDFORWARD_SCALE_X
        self.velocity_feedforward_scale_y = TRACK_VEL_FEEDFORWARD_SCALE_Y
        self.velocity_controller_mode = TRACK_VELOCITY_CONTROLLER_MODE
        self.velocity_debug_scale = TRACK_VELOCITY_DEBUG_SCALE

        self.running = False
        self.tracking_thread = None
        self.current_sat_index = None
        self.error_time = deque()
        self.x_error_history = deque()
        self.y_error_history = deque()

        self.satellites = []
        self.display_map = []
        self.satellite_labels = []
        self.reference_mode_var = tk.StringVar(value="live")
        self.replay_path = None
        self.replay_samples = []

        top_frame = ttk.Frame(self)
        top_frame.pack(fill="x", padx=10, pady=8)

        location_frame = ttk.LabelFrame(top_frame, text="Location")
        location_frame.pack(side="left", fill="y", anchor="n")

        self.location_label = ttk.Label(location_frame, text="Location: not set")
        self.location_label.grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 2))

        ttk.Label(location_frame, text="Latitude (deg)").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.lat_entry = ttk.Entry(location_frame, width=14)
        self.lat_entry.grid(row=1, column=1, padx=5, pady=2)
        self.lat_entry.insert(0, str(self.observer_lat))

        ttk.Label(location_frame, text="Longitude (deg)").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.lon_entry = ttk.Entry(location_frame, width=14)
        self.lon_entry.grid(row=2, column=1, padx=5, pady=2)
        self.lon_entry.insert(0, str(self.observer_lon))

        ttk.Button(location_frame, text="Apply Location", command=self.apply_location).grid(
            row=3, column=0, columnspan=2, pady=6
        )

        settings_container = ttk.Frame(top_frame)
        settings_container.pack(side="right", anchor="n")

        velocity_frame = ttk.LabelFrame(settings_container, text="Velocity Mode Settings")

        ttk.Label(velocity_frame, text="Kp Pos X").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.vel_pos_gain_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_pos_gain_x_entry.grid(row=0, column=1, padx=5, pady=2)
        self.vel_pos_gain_x_entry.insert(0, f"{self.velocity_position_gain_x:g}")

        ttk.Label(velocity_frame, text="Kp Pos Y").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.vel_pos_gain_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_pos_gain_y_entry.grid(row=1, column=1, padx=5, pady=2)
        self.vel_pos_gain_y_entry.insert(0, f"{self.velocity_position_gain_y:g}")

        ttk.Label(velocity_frame, text="Ki X").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.vel_int_gain_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_int_gain_x_entry.grid(row=2, column=1, padx=5, pady=2)
        self.vel_int_gain_x_entry.insert(0, f"{self.velocity_integral_gain_x:g}")

        ttk.Label(velocity_frame, text="Ki Y").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        self.vel_int_gain_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_int_gain_y_entry.grid(row=3, column=1, padx=5, pady=2)
        self.vel_int_gain_y_entry.insert(0, f"{self.velocity_integral_gain_y:g}")

        ttk.Label(velocity_frame, text="Kd/Vel X").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        self.vel_err_gain_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_err_gain_x_entry.grid(row=4, column=1, padx=5, pady=2)
        self.vel_err_gain_x_entry.insert(0, f"{self.velocity_error_gain_x:g}")

        ttk.Label(velocity_frame, text="Kd/Vel Y").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        self.vel_err_gain_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_err_gain_y_entry.grid(row=5, column=1, padx=5, pady=2)
        self.vel_err_gain_y_entry.insert(0, f"{self.velocity_error_gain_y:g}")

        ttk.Label(velocity_frame, text="Vel Alpha X").grid(row=6, column=0, sticky="w", padx=5, pady=2)
        self.vel_alpha_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_alpha_x_entry.grid(row=6, column=1, padx=5, pady=2)
        self.vel_alpha_x_entry.insert(0, f"{self.spi_velocity_filter_alpha_x:g}")

        ttk.Label(velocity_frame, text="Vel Alpha Y").grid(row=7, column=0, sticky="w", padx=5, pady=2)
        self.vel_alpha_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_alpha_y_entry.grid(row=7, column=1, padx=5, pady=2)
        self.vel_alpha_y_entry.insert(0, f"{self.spi_velocity_filter_alpha_y:g}")

        ttk.Label(velocity_frame, text="Vel Max X").grid(row=8, column=0, sticky="w", padx=5, pady=2)
        self.vel_cmd_max_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_cmd_max_x_entry.grid(row=8, column=1, padx=5, pady=2)
        self.vel_cmd_max_x_entry.insert(0, f"{self.velocity_command_max_x:g}")

        ttk.Label(velocity_frame, text="Vel Max Y").grid(row=9, column=0, sticky="w", padx=5, pady=2)
        self.vel_cmd_max_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_cmd_max_y_entry.grid(row=9, column=1, padx=5, pady=2)
        self.vel_cmd_max_y_entry.insert(0, f"{self.velocity_command_max_y:g}")

        ttk.Label(velocity_frame, text="FF Scale X").grid(row=10, column=0, sticky="w", padx=5, pady=2)
        self.vel_ff_scale_x_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_ff_scale_x_entry.grid(row=10, column=1, padx=5, pady=2)
        self.vel_ff_scale_x_entry.insert(0, f"{self.velocity_feedforward_scale_x:g}")

        ttk.Label(velocity_frame, text="FF Scale Y").grid(row=11, column=0, sticky="w", padx=5, pady=2)
        self.vel_ff_scale_y_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_ff_scale_y_entry.grid(row=11, column=1, padx=5, pady=2)
        self.vel_ff_scale_y_entry.insert(0, f"{self.velocity_feedforward_scale_y:g}")

        ttk.Label(velocity_frame, text="Ctrl Mode").grid(row=12, column=0, sticky="w", padx=5, pady=2)
        self.vel_controller_mode_var = tk.StringVar(value=self.velocity_controller_mode)
        ttk.Combobox(
            velocity_frame,
            textvariable=self.vel_controller_mode_var,
            values=("full", "velocity_error_only", "feedforward_only"),
            state="readonly",
            width=18,
        ).grid(row=12, column=1, padx=5, pady=2)

        ttk.Label(velocity_frame, text="Vel Debug Scale").grid(row=13, column=0, sticky="w", padx=5, pady=2)
        self.vel_debug_scale_entry = ttk.Entry(velocity_frame, width=10)
        self.vel_debug_scale_entry.grid(row=13, column=1, padx=5, pady=2)
        self.vel_debug_scale_entry.insert(0, f"{self.velocity_debug_scale:g}")

        ttk.Button(velocity_frame, text="Apply Vel Gains", command=self.apply_tracking_settings).grid(
            row=14, column=0, columnspan=2, pady=6
        )

        tle_frame = ttk.LabelFrame(top_frame, text="TLE And Satellite")
        tle_frame.pack(side="left", fill="x", expand=True, padx=10, anchor="n")

        ttk.Label(tle_frame, text="Select TLE File:").grid(row=0, column=0, sticky="w", padx=5, pady=(5, 2))
        ttk.Button(tle_frame, text="Browse", command=self.load_tle).grid(row=0, column=1, sticky="w", padx=5, pady=(5, 2))

        self.tle_file_label = ttk.Label(tle_frame, text="No file selected", width=32)
        self.tle_file_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        ttk.Label(tle_frame, text="Select Satellite:").grid(row=2, column=0, sticky="w", padx=5, pady=(8, 2))
        self.sat_combobox = ttk.Combobox(tle_frame, state="readonly", width=34)
        self.sat_combobox.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=2)
        ttk.Button(tle_frame, text="Refresh Pass Times", command=self.refresh_satellite_display).grid(
            row=2, column=1, sticky="e", padx=5, pady=(8, 2)
        )

        ttk.Label(tle_frame, text="Reference Source:").grid(row=4, column=0, sticky="w", padx=5, pady=(8, 2))
        self.reference_mode_combo = ttk.Combobox(
            tle_frame,
            state="readonly",
            width=18,
            values=("live", "replay", "replay_utc"),
            textvariable=self.reference_mode_var,
        )
        self.reference_mode_combo.grid(row=4, column=1, sticky="w", padx=5, pady=(8, 2))

        ttk.Button(tle_frame, text="Replay File", command=self.load_replay_file).grid(
            row=5, column=0, sticky="w", padx=5, pady=(2, 2)
        )
        self.replay_file_label = ttk.Label(tle_frame, text="No replay file selected", width=32)
        self.replay_file_label.grid(row=5, column=1, sticky="ew", padx=5, pady=(2, 2))

        button_row = ttk.Frame(tle_frame)
        button_row.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=8)
        ttk.Button(button_row, text="Start Tracking", command=self.start_tracking_thread).pack(side="left", padx=(0, 8))
        ttk.Button(button_row, text="Stop Tracking / Stow", command=self.stop_tracking).pack(side="left")

        tle_frame.columnconfigure(0, weight=1)

        settings_frame = ttk.LabelFrame(settings_container, text="Tracking Settings")
        settings_frame.pack(side="left", anchor="n")

        ttk.Label(settings_frame, text="Cmd Interval (s)").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.cmd_interval_entry = ttk.Entry(settings_frame, width=10)
        self.cmd_interval_entry.grid(row=0, column=1, padx=5, pady=2)
        self.cmd_interval_entry.insert(0, f"{self.track_command_interval_sec:g}")

        ttk.Label(settings_frame, text="Horizon (s)").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.horizon_entry = ttk.Entry(settings_frame, width=10)
        self.horizon_entry.grid(row=1, column=1, padx=5, pady=2)
        self.horizon_entry.insert(0, f"{self.trajectory_horizon_sec:g}")

        ttk.Label(settings_frame, text="Point Spacing (s)").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.point_spacing_entry = ttk.Entry(settings_frame, width=10)
        self.point_spacing_entry.grid(row=2, column=1, padx=5, pady=2)
        self.point_spacing_entry.insert(0, f"{self.trajectory_point_spacing_sec:g}")

        ttk.Label(settings_frame, text="Max Angle Step").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        self.max_angle_step_entry = ttk.Entry(settings_frame, width=10)
        self.max_angle_step_entry.grid(row=3, column=1, padx=5, pady=2)
        self.max_angle_step_entry.insert(0, f"{self.max_angle_step_deg:g}")

        ttk.Label(settings_frame, text="Min Spacing").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        self.min_spacing_entry = ttk.Entry(settings_frame, width=10)
        self.min_spacing_entry.grid(row=4, column=1, padx=5, pady=2)
        self.min_spacing_entry.insert(0, f"{self.min_trajectory_spacing_sec:g}")

        ttk.Label(settings_frame, text="Lead (s)").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        self.feedforward_lead_entry = ttk.Entry(settings_frame, width=10)
        self.feedforward_lead_entry.grid(row=5, column=1, padx=5, pady=2)
        self.feedforward_lead_entry.insert(0, f"{self.feedforward_lead_sec:g}")

        ttk.Label(settings_frame, text="Filter BW").grid(row=0, column=2, sticky="w", padx=(18, 5), pady=2)
        self.filter_bandwidth_entry = ttk.Entry(settings_frame, width=10)
        self.filter_bandwidth_entry.grid(row=0, column=3, padx=5, pady=2)
        self.filter_bandwidth_entry.insert(0, f"{self.track_filter_bandwidth_hz:g}")

        ttk.Label(settings_frame, text="Capture Err").grid(row=1, column=2, sticky="w", padx=(18, 5), pady=2)
        self.capture_error_entry = ttk.Entry(settings_frame, width=10)
        self.capture_error_entry.grid(row=1, column=3, padx=5, pady=2)
        self.capture_error_entry.insert(0, f"{self.track_capture_error_deg:g}")

        ttk.Label(settings_frame, text="Rebuild Margin").grid(row=2, column=2, sticky="w", padx=(18, 5), pady=2)
        self.rebuild_margin_entry = ttk.Entry(settings_frame, width=10)
        self.rebuild_margin_entry.grid(row=2, column=3, padx=5, pady=2)
        self.rebuild_margin_entry.insert(0, f"{self.trajectory_rebuild_margin_sec:g}")

        ttk.Label(settings_frame, text="Prepoint Lead").grid(row=3, column=2, sticky="w", padx=(18, 5), pady=2)
        self.prepoint_lead_entry = ttk.Entry(settings_frame, width=10)
        self.prepoint_lead_entry.grid(row=3, column=3, padx=5, pady=2)
        self.prepoint_lead_entry.insert(0, f"{self.prepoint_lead_time_sec:g}")

        ttk.Button(settings_frame, text="Apply", command=self.apply_tracking_settings).grid(
            row=6, column=0, columnspan=4, pady=6
        )

        settings_frame.columnconfigure(0, weight=0)
        settings_frame.columnconfigure(1, weight=0)
        settings_frame.columnconfigure(2, weight=0)
        settings_frame.columnconfigure(3, weight=0)

        velocity_frame.pack(side="left", anchor="n", padx=(10, 0))

        output_frame = ttk.Frame(self)
        output_frame.pack(fill="x", padx=10, pady=(6, 10))

        self.output_text = tk.Text(output_frame, height=8, width=100, wrap="none", font=("Consolas", 10))
        self.output_text.pack(side="left", fill="x", expand=True)
        self.output_text.configure(state="disabled")

        output_scrollbar = ttk.Scrollbar(output_frame, orient="vertical", command=self.output_text.yview)
        output_scrollbar.pack(side="right", fill="y")
        self.output_text.configure(yscrollcommand=output_scrollbar.set)

        self.error_fig = Figure(figsize=(7.8, 5.2), dpi=100)
        self.error_ax_x = self.error_fig.add_subplot(211)
        self.error_ax_y = self.error_fig.add_subplot(212, sharex=self.error_ax_x)
        self.error_ax_x.set_title("Trajectory Error")
        self.error_ax_x.set_ylabel("X Traj Error (deg)")
        self.error_ax_y.set_xlabel("Time (s)")
        self.error_ax_y.set_ylabel("Y Traj Error (deg)")
        self.x_error_line, = self.error_ax_x.plot([], [], label="X Traj Error")
        self.y_error_line, = self.error_ax_y.plot([], [], label="Y Traj Error", color="tab:orange")
        self.error_ax_x.axhline(0.03, color="tab:red", linestyle="--", linewidth=1.0, alpha=0.8)
        self.error_ax_x.axhline(-0.03, color="tab:red", linestyle="--", linewidth=1.0, alpha=0.8)
        self.error_ax_y.axhline(0.03, color="tab:red", linestyle="--", linewidth=1.0, alpha=0.8)
        self.error_ax_y.axhline(-0.03, color="tab:red", linestyle="--", linewidth=1.0, alpha=0.8)
        self.error_ax_x.set_ylim(-0.1, 0.1)
        self.error_ax_y.set_ylim(-0.1, 0.1)
        self.error_ax_x.legend(loc="upper right")
        self.error_ax_y.legend(loc="upper right")
        self.error_fig.tight_layout()

        self.error_canvas = FigureCanvasTkAgg(self.error_fig, master=self)
        self.error_canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def reset_error_plot(self):
        self.error_time.clear()
        self.x_error_history.clear()
        self.y_error_history.clear()
        self.x_error_line.set_data([], [])
        self.y_error_line.set_data([], [])
        self.error_ax_x.set_xlim(0, 1)
        self.error_ax_y.set_xlim(0, 1)
        self.error_ax_x.set_ylim(-0.1, 0.1)
        self.error_ax_y.set_ylim(-0.1, 0.1)
        self.error_canvas.draw_idle()

    def set_output_text(self, text):
        prior_yview = self.output_text.yview()
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        self.output_text.insert("1.0", text)
        self.output_text.configure(state="disabled")
        if prior_yview:
            self.output_text.yview_moveto(prior_yview[0])

    def format_output_columns(self, lines, columns=3, gutter=4):
        if not lines:
            return ""

        row_count = (len(lines) + columns - 1) // columns
        padded = list(lines) + [""] * (row_count * columns - len(lines))
        column_widths = []
        for col in range(columns):
            column_items = padded[col * row_count : (col + 1) * row_count]
            column_widths.append(max((len(item) for item in column_items), default=0))

        rows = []
        for row in range(row_count):
            row_parts = []
            for col in range(columns):
                item = padded[col * row_count + row]
                if col < columns - 1:
                    row_parts.append(item.ljust(column_widths[col] + gutter))
                else:
                    row_parts.append(item)
            rows.append("".join(row_parts).rstrip())
        return "\n".join(rows)

    def update_error_plot(self, elapsed_sec, x_error, y_error):
        self.error_time.append(elapsed_sec)
        self.x_error_history.append(x_error)
        self.y_error_history.append(y_error)
        self.x_error_line.set_data(list(self.error_time), list(self.x_error_history))
        self.y_error_line.set_data(list(self.error_time), list(self.y_error_history))
        xmax = max(1.0, elapsed_sec)
        self.error_ax_x.set_xlim(0, xmax)
        self.error_ax_y.set_xlim(0, xmax)
        self.error_ax_x.set_ylim(-0.1, 0.1)
        self.error_ax_y.set_ylim(-0.1, 0.1)
        self.error_canvas.draw_idle()

    def apply_tracking_settings(self):
        try:
            cmd_interval = float(self.cmd_interval_entry.get())
            horizon = float(self.horizon_entry.get())
            point_spacing = float(self.point_spacing_entry.get())
            max_angle_step = float(self.max_angle_step_entry.get())
            min_spacing = float(self.min_spacing_entry.get())
            feedforward_lead = float(self.feedforward_lead_entry.get())
            filter_bandwidth = float(self.filter_bandwidth_entry.get())
            capture_error = float(self.capture_error_entry.get())
            rebuild_margin = float(self.rebuild_margin_entry.get())
            prepoint_lead = float(self.prepoint_lead_entry.get())
            velocity_position_gain_x = float(self.vel_pos_gain_x_entry.get())
            velocity_position_gain_y = float(self.vel_pos_gain_y_entry.get())
            velocity_integral_gain_x = float(self.vel_int_gain_x_entry.get())
            velocity_integral_gain_y = float(self.vel_int_gain_y_entry.get())
            velocity_error_gain_x = float(self.vel_err_gain_x_entry.get())
            velocity_error_gain_y = float(self.vel_err_gain_y_entry.get())
            spi_velocity_filter_alpha_x = float(self.vel_alpha_x_entry.get())
            spi_velocity_filter_alpha_y = float(self.vel_alpha_y_entry.get())
            velocity_command_max_x = float(self.vel_cmd_max_x_entry.get())
            velocity_command_max_y = float(self.vel_cmd_max_y_entry.get())
            velocity_feedforward_scale_x = float(self.vel_ff_scale_x_entry.get())
            velocity_feedforward_scale_y = float(self.vel_ff_scale_y_entry.get())
            velocity_controller_mode = self.vel_controller_mode_var.get().strip()
            velocity_debug_scale = float(self.vel_debug_scale_entry.get())

            if (
                cmd_interval <= 0
                or horizon <= 0
                or point_spacing <= 0
                or max_angle_step <= 0
                or min_spacing <= 0
                or feedforward_lead < 0
                or filter_bandwidth <= 0
                or capture_error <= 0
                or rebuild_margin <= 0
                or prepoint_lead <= 0
                or velocity_command_max_x <= 0
                or velocity_command_max_y <= 0
                or velocity_debug_scale <= 0
            ):
                raise ValueError
            if point_spacing > horizon:
                raise ValueError
            if rebuild_margin >= horizon:
                raise ValueError
            if min_spacing > point_spacing:
                raise ValueError
            if not (0.0 <= spi_velocity_filter_alpha_x < 1.0):
                raise ValueError
            if not (0.0 <= spi_velocity_filter_alpha_y < 1.0):
                raise ValueError
            if velocity_controller_mode not in ("full", "velocity_error_only", "feedforward_only"):
                raise ValueError

            self.track_command_interval_sec = cmd_interval
            self.trajectory_horizon_sec = horizon
            self.trajectory_point_spacing_sec = point_spacing
            self.max_angle_step_deg = max_angle_step
            self.min_trajectory_spacing_sec = min_spacing
            self.feedforward_lead_sec = feedforward_lead
            self.track_filter_bandwidth_hz = filter_bandwidth
            self.track_capture_error_deg = capture_error
            self.trajectory_rebuild_margin_sec = rebuild_margin
            self.prepoint_lead_time_sec = prepoint_lead
            self.velocity_position_gain_x = velocity_position_gain_x
            self.velocity_position_gain_y = velocity_position_gain_y
            self.velocity_integral_gain_x = velocity_integral_gain_x
            self.velocity_integral_gain_y = velocity_integral_gain_y
            self.velocity_error_gain_x = velocity_error_gain_x
            self.velocity_error_gain_y = velocity_error_gain_y
            self.spi_velocity_filter_alpha_x = spi_velocity_filter_alpha_x
            self.spi_velocity_filter_alpha_y = spi_velocity_filter_alpha_y
            self.velocity_command_max_x = velocity_command_max_x
            self.velocity_command_max_y = velocity_command_max_y
            self.velocity_feedforward_scale_x = velocity_feedforward_scale_x
            self.velocity_feedforward_scale_y = velocity_feedforward_scale_y
            self.velocity_controller_mode = velocity_controller_mode
            self.velocity_debug_scale = velocity_debug_scale
        except Exception:
            messagebox.showerror(
                "Error",
                (
                    "Tracking settings must be positive numbers.\n"
                    "Lead can be zero or positive.\n"
                    "Point spacing cannot exceed horizon.\n"
                    "Rebuild margin must be smaller than horizon.\n"
                    "Min spacing cannot exceed point spacing.\n"
                    "Velocity filter alpha must be in [0, 1).\n"
                    "Velocity command max and debug scale must be positive."
                ),
            )

    def load_tle(self):
        file_path = filedialog.askopenfilename(filetypes=[("TLE Files", "*.txt"), ("All Files", "*.*")])
        if not file_path:
            return

        self.tle_file_label.config(text=shorten_display_path(file_path))

        with open(file_path, "r") as f:
            lines = f.readlines()

        self.satellites = []

        for i in range(0, len(lines) - 2, 3):
            sat = EarthSatellite(lines[i + 1].strip(), lines[i + 2].strip(), lines[i].strip())
            self.satellites.append(sat)

        if not self.satellites:
            messagebox.showerror("Error", "No satellites found")
            return

        self.refresh_satellite_display()

        messagebox.showinfo("TLE Loaded", f"{len(self.satellites)} satellites loaded")

    def load_replay_file(self):
        default_dir = Path(__file__).resolve().parent / "tracking_replays"
        file_path = filedialog.askopenfilename(
            initialdir=str(default_dir),
            filetypes=[("Replay CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        if not file_path:
            return

        try:
            replay_samples = load_replay_reference_csv(file_path)
            if not replay_samples:
                raise ValueError("Replay file does not contain any usable samples.")
        except Exception as exc:
            messagebox.showerror("Replay Load Error", str(exc))
            return

        self.replay_path = file_path
        self.replay_samples = replay_samples
        self.replay_file_label.config(text=shorten_display_path(file_path))
        self.reference_mode_var.set("replay")
        messagebox.showinfo("Replay Loaded", f"Loaded {len(replay_samples)} replay samples.")

    def refresh_satellite_display(self):
        if not self.satellites:
            return

        previous_sat_index = None
        current_selection = self.sat_combobox.current()
        if current_selection >= 0 and current_selection < len(self.display_map):
            previous_sat_index = self.display_map[current_selection]

        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)

        display_data = []
        self.display_map = []

        now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)

        for idx, sat in enumerate(self.satellites):
            t0 = ts.now()
            t1 = ts.from_datetime(now_utc + datetime.timedelta(hours=24))

            try:
                times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)

                next_rise = None
                for ti, event in zip(times, events):
                    if event == 0:
                        next_rise = ti.utc_datetime().replace(tzinfo=utc)
                        break

                short_name = sat.name[:24].rstrip()
                if next_rise:
                    dt = next_rise - now_utc
                    minutes = max(0, int(dt.total_seconds() / 60))
                    label = f"{short_name} [{sat.model.satnum}]  {minutes}m"
                    sort_key = dt.total_seconds()
                else:
                    label = f"{short_name} [{sat.model.satnum}]  no pass"
                    sort_key = 999999999

            except Exception:
                short_name = sat.name[:24].rstrip()
                label = f"{short_name} [{sat.model.satnum}]"
                sort_key = 999999999

            display_data.append((sort_key, label, idx))

        display_data.sort(key=lambda x: x[0])

        display_list = []
        new_selection = None
        for pos, item in enumerate(display_data):
            display_list.append(item[1])
            self.display_map.append(item[2])
            if previous_sat_index is not None and item[2] == previous_sat_index:
                new_selection = pos

        self.satellite_labels = display_list
        self.sat_combobox["values"] = display_list

        if display_list:
            if new_selection is not None:
                self.sat_combobox.current(new_selection)
            elif current_selection >= 0 and current_selection < len(display_list):
                self.sat_combobox.current(current_selection)
            else:
                self.sat_combobox.current(0)

    def start_tracking_thread(self):
        reference_mode = self.reference_mode_var.get()
        replay_mode = reference_mode in ("replay", "replay_utc")
        replay_utc_mode = reference_mode == "replay_utc"
        sat_index = None
        sat_label = "Replay"

        if replay_mode:
            if not self.replay_samples:
                messagebox.showerror("Error", "Load a replay CSV first")
                return
            if replay_utc_mode:
                if not self.satellites:
                    messagebox.showerror("Error", "Load TLE first for replay_utc mode")
                    return
                sel = self.sat_combobox.current()
                if sel < 0:
                    messagebox.showerror("Error", "Select satellite for replay_utc mode")
                    return
                sat_index = self.display_map[sel]
                sat_label = f"{self.satellites[sat_index].name} [UTC Replay]"
            else:
                replay_path = Path(self.replay_path) if self.replay_path else None
                sat_label = replay_path.stem.replace("_reference", "") if replay_path else "Replay"
        else:
            if not self.satellites:
                messagebox.showerror("Error", "Load TLE first")
                return

            sel = self.sat_combobox.current()

            if sel < 0:
                messagebox.showerror("Error", "Select satellite")
                return

            sat_index = self.display_map[sel]
            sat_label = self.satellites[sat_index].name

        if self.running:
            if sat_index == self.current_sat_index:
                return

            result = messagebox.askyesno(
                "Switch Satellite",
                "Tracking already in progress.\n\nDo you want to switch satellites?",
            )

            if not result:
                return

            self.running = False
            time.sleep(0.6)

        self.current_sat_index = sat_index
        self.running = True
        self.reset_error_plot()
        self.set_output_text(
            self.format_output_columns(
                [
                    f"Satellite: {sat_label}",
                    "Status: Starting tracking...",
                    "Rise ETA: calculating...",
                    "Rise Local: calculating...",
                    "AZ: -",
                    "EL: -",
                    "Raw X: -",
                    "Raw Y: -",
                    "X Angle: -",
                    "Y Angle: -",
                    "X Actual: -",
                    "Y Actual: -",
                    "X Cmd Error: -",
                    "Y Cmd Error: -",
                    "X Traj Error: -",
                    "Y Traj Error: -",
                ]
            )
        )

        self.tracking_thread = threading.Thread(
            target=self.track_satellite_loop,
            args=(sat_index,),
            daemon=True,
        )
        self.tracking_thread.start()

    def stop_tracking(self):
        self.running = False
        self.current_sat_index = None

        tracking_thread = self.tracking_thread
        if tracking_thread and tracking_thread.is_alive() and tracking_thread is not threading.current_thread():
            tracking_thread.join(timeout=max(0.2, self.track_command_interval_sec * 2.0))

        if self.control:
            try:
                self.control.exit_tracking_mode_all()
            except Exception:
                pass
            if self.preposition_gains:
                try:
                    self.control.set_gains_all(*self.preposition_gains)
                except Exception:
                    pass
            try:
                self.control.move_absolute_pair(x_deg=0, y_deg=0)
            except Exception:
                pass
        self.tracking_thread = None

    def track_satellite_loop(self, sat_index):
        reference_mode = self.reference_mode_var.get()
        replay_mode = reference_mode in ("replay", "replay_utc")
        replay_utc_mode = reference_mode == "replay_utc"
        ts = load.timescale() if (not replay_mode or replay_utc_mode) else None
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon) if (not replay_mode or replay_utc_mode) else None
        sat = self.satellites[sat_index] if (not replay_mode or replay_utc_mode) else None
        replay_samples = list(self.replay_samples) if replay_mode else None
        sat_name = sat.name if sat is not None else (Path(self.replay_path).stem.replace("_reference", "") if self.replay_path else "Replay")
        self.set_output_text(
            self.format_output_columns(
                [
                    f"Satellite: {sat_name}",
                    "Status: Computing next pass...",
                    "Rise ETA: calculating...",
                    "Rise Local: calculating...",
                    "AZ: -",
                    "EL: -",
                    "Raw X: -",
                    "Raw Y: -",
                    "X Angle: -",
                    "Y Angle: -",
                    "X Actual: -",
                    "Y Actual: -",
                    "X Cmd Error: -",
                    "Y Cmd Error: -",
                    "X Traj Error: -",
                    "Y Traj Error: -",
                ]
            )
        )
        log_dir = Path(__file__).resolve().parent / "tracking_logs"
        log_dir.mkdir(exist_ok=True)
        replay_dir = Path(__file__).resolve().parent / "tracking_replays"
        replay_dir.mkdir(exist_ok=True)
        safe_sat_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in sat_name).strip("_") or "satellite"
        run_timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = log_dir / f"{safe_sat_name}_{run_timestamp}.csv"
        replay_path = replay_dir / f"{safe_sat_name}_{run_timestamp}_reference.csv"
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        replay_file = open(replay_path, "w", newline="", encoding="utf-8")
        replay_writer = csv.writer(replay_file)
        log_writer.writerow(
            [
                "utc_time",
                "loop_start_monotonic_sec",
                "elapsed_sec",
                "trajectory_elapsed_sec",
                "sample_visible",
                "tracking_started",
                "rebuilt_trajectory",
                "target_elapsed_sec",
                "ideal_target_elapsed_sec",
                "az_deg",
                "el_deg",
                "raw_x_angle_deg",
                "raw_y_angle_deg",
                "cmd_x_angle_deg",
                "cmd_y_angle_deg",
                "cmd_x_p_term_deg",
                "cmd_y_p_term_deg",
                "cmd_x_i_term_deg",
                "cmd_y_i_term_deg",
                "cmd_x_d_term_deg",
                "cmd_y_d_term_deg",
                "y_integral_unwind_applied",
                "y_integral_state",
                "cmd_x_correction_deg",
                "cmd_y_correction_deg",
                "cmd_x_sent_deg",
                "cmd_y_sent_deg",
                "raw_dx_deg",
                "raw_dy_deg",
                "cmd_dx_deg",
                "cmd_dy_deg",
                "x_vel_deg_per_s",
                "y_vel_deg_per_s",
                "x_acc_deg_per_s2",
                "y_acc_deg_per_s2",
                "cmd_x_vel_ff_deg_per_s",
                "cmd_y_vel_ff_deg_per_s",
                "cmd_x_vel_sent_deg_per_s",
                "cmd_y_vel_sent_deg_per_s",
                "x_actual_vel_raw_deg_per_s",
                "y_actual_vel_raw_deg_per_s",
                "x_actual_vel_deg_per_s",
                "y_actual_vel_deg_per_s",
                "x_vel_error_deg_per_s",
                "y_vel_error_deg_per_s",
                "velocity_mode_active",
                "x_control_mode",
                "x_input_mode",
                "y_control_mode",
                "y_input_mode",
                "x_controller_vel_limit",
                "y_controller_vel_limit",
                "x_traj_vel_limit",
                "x_traj_accel_limit",
                "x_traj_decel_limit",
                "y_traj_vel_limit",
                "y_traj_accel_limit",
                "y_traj_decel_limit",
                "x_actual_deg",
                "y_actual_deg",
                "x_cmd_error_deg",
                "y_cmd_error_deg",
                "x_traj_error_deg",
                "y_traj_error_deg",
                "capture_error_deg",
                "sgp4_live_eval_ms",
                "sgp4_ideal_eval_ms",
                "sgp4_command_eval_ms",
                "spi_x_read_ms",
                "spi_y_read_ms",
                "spi_total_ms",
                "control_snapshot_ms",
                "command_send_ms",
                "loop_total_ms",
                "observed_loop_period_sec",
                "target_lookahead_sec",
                "used_meas_to_command_delay_sec",
                "meas_x_elapsed_sec",
                "meas_y_elapsed_sec",
                "meas_mid_elapsed_sec",
                "command_send_elapsed_sec",
                "status",
            ]
        )
        replay_writer.writerow(
            [
                "utc_time",
                "elapsed_sec",
                "sample_visible",
                "tracking_started",
                "target_elapsed_sec",
                "ideal_target_elapsed_sec",
                "az_deg",
                "el_deg",
                "raw_x_angle_deg",
                "raw_y_angle_deg",
                "cmd_x_angle_deg",
                "cmd_y_angle_deg",
                "cmd_x_p_term_deg",
                "cmd_y_p_term_deg",
                "cmd_x_i_term_deg",
                "cmd_y_i_term_deg",
                "cmd_x_d_term_deg",
                "cmd_y_d_term_deg",
                "y_integral_unwind_applied",
                "y_integral_state",
                "x_vel_deg_per_s",
                "y_vel_deg_per_s",
                "x_acc_deg_per_s2",
                "y_acc_deg_per_s2",
                "cmd_x_vel_ff_deg_per_s",
                "cmd_y_vel_ff_deg_per_s",
                "cmd_x_vel_sent_deg_per_s",
                "cmd_y_vel_sent_deg_per_s",
                "x_actual_vel_raw_deg_per_s",
                "y_actual_vel_raw_deg_per_s",
                "x_actual_vel_deg_per_s",
                "y_actual_vel_deg_per_s",
                "x_vel_error_deg_per_s",
                "y_vel_error_deg_per_s",
                "velocity_mode_active",
                "status",
            ]
        )
        was_visible = False
        tracking_started = False
        tracking_phase = "PREPOSITIONING"
        settled_cycles = 0
        last_tracking_command = None
        last_display_target_elapsed = None
        last_raw_angles = None
        last_display_command = None
        last_loop_start_monotonic = None
        prev_meas_to_command_delay_sec = self.track_command_interval_sec
        next_rise_utc = None if replay_mode else get_next_rise_time(ts, observer, sat)
        prepointed = False
        tracking_start_monotonic = time.monotonic()
        replay_end_elapsed_sec = replay_samples[-1]["elapsed_sec"] if replay_mode and replay_samples else None
        replay_ended = False
        x_error_integral = 0.0
        y_error_integral = 0.0
        x_velocity_error_integral = 0.0
        y_velocity_error_integral = 0.0
        prev_y_ref_vel_for_unwind = None
        prev_x_traj_error_for_d = None
        prev_y_traj_error_for_d = None
        x_error_derivative_filtered = 0.0
        y_error_derivative_filtered = 0.0
        prev_x_actual_for_vel = None
        prev_y_actual_for_vel = None
        prev_x_actual_elapsed_sec = None
        prev_y_actual_elapsed_sec = None
        x_actual_vel_raw = 0.0
        y_actual_vel_raw = 0.0
        x_actual_vel_filtered = 0.0
        y_actual_vel_filtered = 0.0

        def stow_axes():
            nonlocal x_error_integral, y_error_integral, prev_y_ref_vel_for_unwind
            nonlocal x_velocity_error_integral, y_velocity_error_integral
            nonlocal prev_x_traj_error_for_d, prev_y_traj_error_for_d
            nonlocal x_error_derivative_filtered, y_error_derivative_filtered
            nonlocal prev_x_actual_for_vel, prev_y_actual_for_vel
            nonlocal prev_x_actual_elapsed_sec, prev_y_actual_elapsed_sec
            nonlocal x_actual_vel_raw, y_actual_vel_raw
            nonlocal x_actual_vel_filtered, y_actual_vel_filtered
            if not self.control:
                return
            try:
                self.control.exit_tracking_mode_all()
            except Exception:
                pass
            try:
                self.control.exit_velocity_mode_all()
            except Exception:
                pass
            if self.preposition_gains:
                try:
                    self.control.set_gains_all(*self.preposition_gains)
                except Exception:
                    pass
            try:
                self.control.move_absolute_pair(x_deg=0, y_deg=0)
            except Exception:
                pass
            x_error_integral = 0.0
            y_error_integral = 0.0
            x_velocity_error_integral = 0.0
            y_velocity_error_integral = 0.0
            prev_y_ref_vel_for_unwind = None
            prev_x_traj_error_for_d = None
            prev_y_traj_error_for_d = None
            x_error_derivative_filtered = 0.0
            y_error_derivative_filtered = 0.0
            prev_x_actual_for_vel = None
            prev_y_actual_for_vel = None
            prev_x_actual_elapsed_sec = None
            prev_y_actual_elapsed_sec = None
            x_actual_vel_raw = 0.0
            y_actual_vel_raw = 0.0
            x_actual_vel_filtered = 0.0
            y_actual_vel_filtered = 0.0

        def sample_reference(sample_elapsed_sec, sample_time_utc, derivative_dt_sec):
            if replay_utc_mode:
                return sample_replay_tracking_state_utc(ts, observer, sat, replay_samples, sample_elapsed_sec, derivative_dt_sec)
            if replay_mode:
                return sample_replay_tracking_state(replay_samples, sample_elapsed_sec)
            return sample_tracking_state_utc(ts, observer, sat, sample_time_utc, derivative_dt_sec)

        if self.control and self.preposition_gains:
            try:
                self.control.set_gains_all(*self.preposition_gains)
            except Exception:
                pass

        try:
            while self.running and sat_index == self.current_sat_index:
                loop_start_monotonic = time.monotonic()
                observed_loop_period_sec = (
                    self.track_command_interval_sec
                    if last_loop_start_monotonic is None
                    else loop_start_monotonic - last_loop_start_monotonic
                )
                last_loop_start_monotonic = loop_start_monotonic
                now_monotonic = loop_start_monotonic
                now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
                continuous_elapsed_sec = now_monotonic - tracking_start_monotonic
                rebuilt_trajectory = False
                sgp4_live_eval_ms = 0.0
                sgp4_ideal_eval_ms = 0.0
                sgp4_command_eval_ms = 0.0
                spi_x_read_ms = 0.0
                spi_y_read_ms = 0.0
                spi_total_ms = 0.0
                control_snapshot_ms = 0.0
                command_send_ms = 0.0
                meas_x_elapsed_sec = None
                meas_y_elapsed_sec = None
                meas_mid_elapsed_sec = None
                command_send_elapsed_sec = None
                used_meas_to_command_delay_sec = prev_meas_to_command_delay_sec
                target_lookahead_sec = (
                    self.feedforward_lead_sec
                    if TRACK_USE_LEAD
                    else max(observed_loop_period_sec, used_meas_to_command_delay_sec)
                )

                derivative_dt = min(self.track_command_interval_sec, self.trajectory_point_spacing_sec)
                elapsed_sec = continuous_elapsed_sec
                if replay_mode and replay_end_elapsed_sec is not None and continuous_elapsed_sec > replay_end_elapsed_sec:
                    stow_axes()
                    replay_ended = True
                    self.running = False
                    break
                sgp4_live_start = time.monotonic()
                live_sample = sample_reference(continuous_elapsed_sec, now_utc, derivative_dt)
                sgp4_live_eval_ms = (time.monotonic() - sgp4_live_start) * 1000.0
                if live_sample is None:
                    if replay_mode:
                        stow_axes()
                        replay_ended = True
                        self.running = False
                        break
                    time.sleep(self.track_command_interval_sec)
                    continue
                sample = live_sample

                prepoint_status = "Tracking pass"
                rise_eta_text = "Rise ETA: -"
                rise_local_text = "Rise Local: -"
                pickup_window_active = False
                prepoint_target_sample = None
                if replay_mode:
                    rise_eta_text = "Rise ETA: replay"
                    rise_local_text = "Rise Local: replay"
                    prepoint_status = "Replay UTC reference" if replay_utc_mode else "Replay reference"
                elif next_rise_utc and not was_visible:
                    seconds_to_rise = (next_rise_utc - now_utc).total_seconds()
                    if seconds_to_rise > 0:
                        rise_eta_text = f"Rise ETA: {seconds_to_rise:.1f} s"
                        rise_local_text = f"Rise Local: {format_local_datetime(next_rise_utc, include_seconds=True)}"
                        prepoint_status = f"Waiting for rise in {seconds_to_rise:.1f} s"
                        if seconds_to_rise <= self.prepoint_lead_time_sec:
                            rise_sample = get_pointing_sample(ts, observer, sat, next_rise_utc)
                            prepoint_target_sample = {
                                "az_deg": rise_sample["az_deg"],
                                "el_deg": rise_sample["el_deg"],
                                "x_angle": rise_sample["x_angle"],
                                "y_angle": rise_sample["y_angle"],
                                "visible": False,
                            }
                            pickup_window_active = True
                            prepointed = True
                            prepoint_status = f"Prepointing rise in {seconds_to_rise:.1f} s"

                az_deg = sample["az_deg"]
                el_deg = sample["el_deg"]
                raw_x_angle = sample.get("raw_x_angle", sample["x_angle"])
                raw_y_angle = sample.get("raw_y_angle", sample["y_angle"])
                x_angle = raw_x_angle
                y_angle = raw_y_angle
                x_vel = sample.get("x_vel", 0.0)
                y_vel = sample.get("y_vel", 0.0)
                x_acc = sample.get("x_acc", 0.0)
                y_acc = sample.get("y_acc", 0.0)

                command_sample = prepoint_target_sample if prepoint_target_sample is not None and tracking_phase != "TRACKING" else sample
                target_elapsed_sec = continuous_elapsed_sec
                ideal_target_elapsed_sec = continuous_elapsed_sec
                if sample["visible"]:
                    ideal_target_elapsed_sec = continuous_elapsed_sec + target_lookahead_sec
                    target_elapsed_sec = ideal_target_elapsed_sec
                    sgp4_command_start = time.monotonic()
                    command_sample = sample_reference(
                        continuous_elapsed_sec + target_lookahead_sec,
                        now_utc + datetime.timedelta(seconds=target_lookahead_sec),
                        derivative_dt,
                    )
                    sgp4_command_eval_ms = (time.monotonic() - sgp4_command_start) * 1000.0

                if command_sample is None:
                    if replay_mode:
                        stow_axes()
                        replay_ended = True
                        self.running = False
                        break
                    time.sleep(self.track_command_interval_sec)
                    continue

                x_angle = command_sample["x_angle"]
                y_angle = command_sample["y_angle"]
                cmd_x_vel_ff = self.velocity_feedforward_scale_x * command_sample.get("x_vel", x_vel)
                cmd_y_vel_ff = self.velocity_feedforward_scale_y * command_sample.get("y_vel", y_vel)

                x_angle, y_angle = clamp_tracking_angles(x_angle, y_angle)
                cmd_x_angle = x_angle
                cmd_y_angle = y_angle
                cmd_x_p_term = 0.0
                cmd_y_p_term = 0.0
                cmd_x_i_term = 0.0
                cmd_y_i_term = 0.0
                cmd_x_d_term = 0.0
                cmd_y_d_term = 0.0
                y_integral_unwind_applied = 0
                cmd_x_correction = 0.0
                cmd_y_correction = 0.0
                cmd_x_sent = cmd_x_angle
                cmd_y_sent = cmd_y_angle
                cmd_x_vel_sent = 0.0
                cmd_y_vel_sent = 0.0
                raw_dx = 0.0 if last_raw_angles is None else raw_x_angle - last_raw_angles[0]
                raw_dy = 0.0 if last_raw_angles is None else raw_y_angle - last_raw_angles[1]
                cmd_dx = 0.0 if last_display_command is None else cmd_x_angle - last_display_command[0]
                cmd_dy = 0.0 if last_display_command is None else cmd_y_angle - last_display_command[1]
                x_actual = None
                y_actual = None
                x_actual_vel = 0.0
                y_actual_vel = 0.0
                x_vel_error = 0.0
                y_vel_error = 0.0
                x_error = 0.0
                y_error = 0.0
                x_traj_error = 0.0
                y_traj_error = 0.0
                capture_error = None
                velocity_mode_active = int(bool(TRACK_USE_VELOCITY_MODE))
                x_control_mode = None
                x_input_mode = None
                y_control_mode = None
                y_input_mode = None
                x_controller_vel_limit = None
                y_controller_vel_limit = None
                x_traj_vel_limit = None
                x_traj_accel_limit = None
                x_traj_decel_limit = None
                y_traj_vel_limit = None
                y_traj_accel_limit = None
                y_traj_decel_limit = None
                if self.control:
                    try:
                        control_snapshot_start = time.monotonic()
                        x_axis0 = self.control.get_odrive("x").axis0
                        y_axis0 = self.control.get_odrive("y").axis0
                        x_control_mode = int(x_axis0.controller.config.control_mode)
                        x_input_mode = int(x_axis0.controller.config.input_mode)
                        y_control_mode = int(y_axis0.controller.config.control_mode)
                        y_input_mode = int(y_axis0.controller.config.input_mode)
                        x_controller_vel_limit = float(x_axis0.controller.config.vel_limit)
                        y_controller_vel_limit = float(y_axis0.controller.config.vel_limit)
                        x_traj_vel_limit = float(x_axis0.trap_traj.config.vel_limit)
                        x_traj_accel_limit = float(x_axis0.trap_traj.config.accel_limit)
                        x_traj_decel_limit = float(x_axis0.trap_traj.config.decel_limit)
                        y_traj_vel_limit = float(y_axis0.trap_traj.config.vel_limit)
                        y_traj_accel_limit = float(y_axis0.trap_traj.config.accel_limit)
                        y_traj_decel_limit = float(y_axis0.trap_traj.config.decel_limit)
                        control_snapshot_ms = (time.monotonic() - control_snapshot_start) * 1000.0
                        spi_window_start = time.monotonic()
                        spi_x_start = time.monotonic()
                        x_actual = self.control.get_spi_position("x")
                        spi_x_done = time.monotonic()
                        spi_x_read_ms = (spi_x_done - spi_x_start) * 1000.0
                        spi_y_start = time.monotonic()
                        y_actual = self.control.get_spi_position("y")
                        spi_y_done = time.monotonic()
                        spi_y_read_ms = (spi_y_done - spi_y_start) * 1000.0
                        spi_total_ms = (spi_y_done - spi_window_start) * 1000.0
                        meas_x_elapsed_sec = ((spi_x_start + spi_x_done) * 0.5) - tracking_start_monotonic
                        meas_y_elapsed_sec = ((spi_y_start + spi_y_done) * 0.5) - tracking_start_monotonic
                        meas_mid_elapsed_sec = ((spi_window_start + spi_y_done) * 0.5) - tracking_start_monotonic
                        x_error = cmd_x_angle - x_actual
                        y_error = cmd_y_angle - y_actual
                        sgp4_ideal_start = time.monotonic()
                        x_ideal_sample = sample_reference(
                            meas_x_elapsed_sec,
                            now_utc + datetime.timedelta(seconds=meas_x_elapsed_sec - continuous_elapsed_sec),
                            derivative_dt,
                        )
                        y_ideal_sample = sample_reference(
                            meas_y_elapsed_sec,
                            now_utc + datetime.timedelta(seconds=meas_y_elapsed_sec - continuous_elapsed_sec),
                            derivative_dt,
                        )
                        sgp4_ideal_eval_ms = (time.monotonic() - sgp4_ideal_start) * 1000.0
                        if x_ideal_sample is not None:
                            ideal_x_angle, _ = clamp_tracking_angles(
                                x_ideal_sample["x_angle"],
                                x_ideal_sample["y_angle"],
                            )
                            x_traj_error = ideal_x_angle - x_actual
                        if y_ideal_sample is not None:
                            _, ideal_y_angle = clamp_tracking_angles(
                                y_ideal_sample["x_angle"],
                                y_ideal_sample["y_angle"],
                            )
                            y_traj_error = ideal_y_angle - y_actual
                        if (
                            prev_x_actual_for_vel is not None
                            and prev_x_actual_elapsed_sec is not None
                            and meas_x_elapsed_sec is not None
                            and meas_x_elapsed_sec > prev_x_actual_elapsed_sec
                        ):
                            x_actual_vel_raw = (
                                x_actual - prev_x_actual_for_vel
                            ) / (meas_x_elapsed_sec - prev_x_actual_elapsed_sec)
                            x_actual_vel_filtered = (
                                self.spi_velocity_filter_alpha_x * x_actual_vel_filtered
                                + (1.0 - self.spi_velocity_filter_alpha_x) * x_actual_vel_raw
                            )
                        else:
                            x_actual_vel_raw = 0.0
                            x_actual_vel_filtered = 0.0
                        if (
                            prev_y_actual_for_vel is not None
                            and prev_y_actual_elapsed_sec is not None
                            and meas_y_elapsed_sec is not None
                            and meas_y_elapsed_sec > prev_y_actual_elapsed_sec
                        ):
                            y_actual_vel_raw = (
                                y_actual - prev_y_actual_for_vel
                            ) / (meas_y_elapsed_sec - prev_y_actual_elapsed_sec)
                            y_actual_vel_filtered = (
                                self.spi_velocity_filter_alpha_y * y_actual_vel_filtered
                                + (1.0 - self.spi_velocity_filter_alpha_y) * y_actual_vel_raw
                            )
                        else:
                            y_actual_vel_raw = 0.0
                            y_actual_vel_filtered = 0.0
                        x_actual_vel = x_actual_vel_filtered
                        y_actual_vel = y_actual_vel_filtered
                        prev_x_actual_for_vel = x_actual
                        prev_y_actual_for_vel = y_actual
                        prev_x_actual_elapsed_sec = meas_x_elapsed_sec
                        prev_y_actual_elapsed_sec = meas_y_elapsed_sec
                        capture_error = max(abs(x_error), abs(y_error))
                    except Exception:
                        x_actual = None
                        y_actual = None

                track_ready = capture_error is not None and capture_error <= self.track_capture_error_deg
                track_gate_open = sample["visible"] or pickup_window_active

                if tracking_phase != "TRACKING" and track_gate_open and track_ready:
                    settled_cycles += 1
                elif tracking_phase != "TRACKING":
                    settled_cycles = 0

                enter_tracking_now = tracking_phase != "TRACKING" and track_gate_open and settled_cycles >= 3
                should_track_now = track_gate_open and (tracking_phase == "TRACKING" or enter_tracking_now)
                use_velocity_tracking_mode = bool(TRACK_USE_VELOCITY_MODE)

                if enter_tracking_now and self.control:
                    x_error_integral = 0.0
                    y_error_integral = 0.0
                    x_velocity_error_integral = 0.0
                    y_velocity_error_integral = 0.0
                    prev_y_ref_vel_for_unwind = None
                    prev_x_traj_error_for_d = None
                    prev_y_traj_error_for_d = None
                    x_error_derivative_filtered = 0.0
                    y_error_derivative_filtered = 0.0
                    prev_x_actual_for_vel = None
                    prev_y_actual_for_vel = None
                    prev_x_actual_elapsed_sec = None
                    prev_y_actual_elapsed_sec = None
                    x_actual_vel_raw = 0.0
                    y_actual_vel_raw = 0.0
                    x_actual_vel_filtered = 0.0
                    y_actual_vel_filtered = 0.0
                    try:
                        if use_velocity_tracking_mode:
                            self.control.enter_velocity_mode_all(input_filter_bandwidth=self.track_filter_bandwidth_hz)
                        elif TRACK_USE_PASSTHROUGH:
                            self.control.enter_tracking_mode_all(input_filter_bandwidth=self.track_filter_bandwidth_hz)
                    except Exception:
                        pass

                if should_track_now:
                    ideal_target_elapsed_sec = continuous_elapsed_sec + target_lookahead_sec
                    target_elapsed_sec = ideal_target_elapsed_sec
                    tracking_command_sample = sample_reference(
                        continuous_elapsed_sec + target_lookahead_sec,
                        now_utc + datetime.timedelta(seconds=target_lookahead_sec),
                        derivative_dt,
                    )
                    if tracking_command_sample is not None:
                        command_sample = tracking_command_sample
                        x_angle = command_sample["x_angle"]
                        y_angle = command_sample["y_angle"]
                        x_angle, y_angle = clamp_tracking_angles(x_angle, y_angle)
                        cmd_x_angle = x_angle
                        cmd_y_angle = y_angle
                        cmd_dx = 0.0 if last_display_command is None else cmd_x_angle - last_display_command[0]
                        cmd_dy = 0.0 if last_display_command is None else cmd_y_angle - last_display_command[1]
                        cmd_x_vel_ff = self.velocity_feedforward_scale_x * command_sample.get("x_vel", cmd_x_vel_ff)
                        cmd_y_vel_ff = self.velocity_feedforward_scale_y * command_sample.get("y_vel", cmd_y_vel_ff)
                        if x_actual is not None and y_actual is not None:
                            x_error = cmd_x_angle - x_actual
                            y_error = cmd_y_angle - y_actual
                            capture_error = max(abs(x_error), abs(y_error))
                        if not sample["visible"]:
                            prepoint_status = "Below-horizon pickup"

                if should_track_now and x_actual is not None and y_actual is not None:
                    if use_velocity_tracking_mode:
                        x_velocity_error_integral += x_traj_error * observed_loop_period_sec
                        y_velocity_error_integral += y_traj_error * observed_loop_period_sec
                        x_velocity_error_integral = max(
                            -TRACK_VELOCITY_INTEGRAL_MAX_STATE_X,
                            min(TRACK_VELOCITY_INTEGRAL_MAX_STATE_X, x_velocity_error_integral),
                        )
                        y_velocity_error_integral = max(
                            -TRACK_VELOCITY_INTEGRAL_MAX_STATE_Y,
                            min(TRACK_VELOCITY_INTEGRAL_MAX_STATE_Y, y_velocity_error_integral),
                        )
                        x_vel_error = cmd_x_vel_ff - x_actual_vel
                        y_vel_error = cmd_y_vel_ff - y_actual_vel
                        if self.velocity_controller_mode == "feedforward_only":
                            cmd_x_vel_sent = cmd_x_vel_ff
                            cmd_y_vel_sent = cmd_y_vel_ff
                        elif self.velocity_controller_mode == "velocity_error_only":
                            cmd_x_vel_sent = cmd_x_vel_ff + self.velocity_error_gain_x * x_vel_error
                            cmd_y_vel_sent = cmd_y_vel_ff + self.velocity_error_gain_y * y_vel_error
                        else:
                            cmd_x_vel_sent = (
                                cmd_x_vel_ff
                                + self.velocity_position_gain_x * x_traj_error
                                + self.velocity_integral_gain_x * x_velocity_error_integral
                                + self.velocity_error_gain_x * x_vel_error
                            )
                            cmd_y_vel_sent = (
                                cmd_y_vel_ff
                                + self.velocity_position_gain_y * y_traj_error
                                + self.velocity_integral_gain_y * y_velocity_error_integral
                                + self.velocity_error_gain_y * y_vel_error
                            )
                        cmd_x_vel_sent *= self.velocity_debug_scale
                        cmd_y_vel_sent *= self.velocity_debug_scale
                        cmd_x_vel_sent = max(min(cmd_x_vel_sent, self.velocity_command_max_x), -self.velocity_command_max_x)
                        cmd_y_vel_sent = max(min(cmd_y_vel_sent, self.velocity_command_max_y), -self.velocity_command_max_y)
                    else:
                        cmd_x_p_term = TRACK_SPI_POSITION_CORRECTION_GAIN_X * x_traj_error
                        cmd_y_p_term = TRACK_SPI_POSITION_CORRECTION_GAIN_Y * y_traj_error
                        if abs(x_traj_error) <= TRACK_SPI_INTEGRAL_ACTIVE_ERROR_DEG:
                            x_error_integral += x_traj_error * observed_loop_period_sec
                            x_error_integral = max(
                                -TRACK_SPI_INTEGRAL_MAX_STATE_X,
                                min(TRACK_SPI_INTEGRAL_MAX_STATE_X, x_error_integral),
                            )
                        cmd_x_i_term = TRACK_SPI_INTEGRAL_GAIN_X * x_error_integral
                        cmd_x_i_term = max(
                            -TRACK_SPI_INTEGRAL_MAX_CORRECTION_X,
                            min(TRACK_SPI_INTEGRAL_MAX_CORRECTION_X, cmd_x_i_term),
                        )
                        if abs(y_traj_error) <= TRACK_SPI_INTEGRAL_ACTIVE_ERROR_DEG:
                            y_error_integral += y_traj_error * observed_loop_period_sec
                            y_error_integral = max(
                                -TRACK_SPI_INTEGRAL_MAX_STATE_Y,
                                min(TRACK_SPI_INTEGRAL_MAX_STATE_Y, y_error_integral),
                            )
                        cmd_y_i_term = TRACK_SPI_INTEGRAL_GAIN_Y * y_error_integral
                        cmd_y_i_term = max(
                            -TRACK_SPI_INTEGRAL_MAX_CORRECTION_Y,
                            min(TRACK_SPI_INTEGRAL_MAX_CORRECTION_Y, cmd_y_i_term),
                        )
                        if prev_x_traj_error_for_d is None or observed_loop_period_sec <= 1e-6:
                            x_error_derivative_filtered = 0.0
                        else:
                            x_error_derivative_raw = (x_traj_error - prev_x_traj_error_for_d) / observed_loop_period_sec
                            x_error_derivative_filtered = (
                                TRACK_SPI_DERIVATIVE_FILTER_ALPHA_X * x_error_derivative_filtered
                                + (1.0 - TRACK_SPI_DERIVATIVE_FILTER_ALPHA_X) * x_error_derivative_raw
                            )
                        cmd_x_d_term = TRACK_SPI_DERIVATIVE_GAIN_X * x_error_derivative_filtered
                        cmd_x_d_term = max(
                            -TRACK_SPI_DERIVATIVE_MAX_CORRECTION_X,
                            min(TRACK_SPI_DERIVATIVE_MAX_CORRECTION_X, cmd_x_d_term),
                        )
                        if prev_y_traj_error_for_d is None or observed_loop_period_sec <= 1e-6:
                            y_error_derivative_filtered = 0.0
                        else:
                            y_error_derivative_raw = (y_traj_error - prev_y_traj_error_for_d) / observed_loop_period_sec
                            y_error_derivative_filtered = (
                                TRACK_SPI_DERIVATIVE_FILTER_ALPHA_Y * y_error_derivative_filtered
                                + (1.0 - TRACK_SPI_DERIVATIVE_FILTER_ALPHA_Y) * y_error_derivative_raw
                            )
                        cmd_y_d_term = TRACK_SPI_DERIVATIVE_GAIN_Y * y_error_derivative_filtered
                        cmd_y_d_term = max(
                            -TRACK_SPI_DERIVATIVE_MAX_CORRECTION_Y,
                            min(TRACK_SPI_DERIVATIVE_MAX_CORRECTION_Y, cmd_y_d_term),
                        )
                        prev_x_traj_error_for_d = x_traj_error
                        prev_y_traj_error_for_d = y_traj_error
                        prev_y_ref_vel_for_unwind = cmd_y_vel_ff
                        cmd_x_correction = cmd_x_p_term + cmd_x_i_term + cmd_x_d_term
                        cmd_y_correction = cmd_y_p_term + cmd_y_i_term + cmd_y_d_term
                        cmd_x_sent, cmd_y_sent = clamp_tracking_angles(
                            cmd_x_angle + cmd_x_correction,
                            cmd_y_angle + cmd_y_correction,
                        )
                else:
                    cmd_x_p_term = 0.0
                    cmd_y_p_term = 0.0
                    cmd_x_i_term = 0.0
                    cmd_y_i_term = 0.0
                    cmd_x_d_term = 0.0
                    cmd_y_d_term = 0.0
                    if tracking_phase != "TRACKING":
                        x_error_integral = 0.0
                        y_error_integral = 0.0
                        x_velocity_error_integral = 0.0
                        y_velocity_error_integral = 0.0
                        prev_y_ref_vel_for_unwind = None
                        prev_x_traj_error_for_d = None
                        prev_y_traj_error_for_d = None
                        x_error_derivative_filtered = 0.0
                        y_error_derivative_filtered = 0.0
                        prev_x_actual_for_vel = None
                        prev_y_actual_for_vel = None
                        prev_x_actual_elapsed_sec = None
                        prev_y_actual_elapsed_sec = None
                        x_actual_vel_filtered = 0.0
                        y_actual_vel_filtered = 0.0

                displayed_tracking_started = int(bool(tracking_phase == "TRACKING" or enter_tracking_now))

                self.set_output_text(
                    self.format_output_columns(
                        [
                            f"Satellite: {sat_name}",
                            f"AZ: {az_deg:.3f}",
                            f"EL: {el_deg:.3f}",
                            f"Raw X: {raw_x_angle:.3f}",
                            f"Raw Y: {raw_y_angle:.3f}",
                            f"X Angle: {cmd_x_angle:.3f}",
                            f"Y Angle: {cmd_y_angle:.3f}",
                            f"dRaw X: {raw_dx:.3f}",
                            f"dRaw Y: {raw_dy:.3f}",
                            f"dCmd X: {cmd_dx:.3f}",
                            f"dCmd Y: {cmd_dy:.3f}",
                            f"X Vel: {x_vel:.3f}",
                            f"Y Vel: {y_vel:.3f}",
                            f"X Act Vel: {x_actual_vel:.3f}",
                            f"Y Act Vel: {y_actual_vel:.3f}",
                            f"X Cmd Vel: {cmd_x_vel_sent:.3f}",
                            f"Y Cmd Vel: {cmd_y_vel_sent:.3f}",
                            f"X Acc: {x_acc:.3f}",
                            f"Y Acc: {y_acc:.3f}",
                            f"Mode: {'VEL' if use_velocity_tracking_mode else 'POS'}",
                            f"Rebuilt: {'YES' if rebuilt_trajectory else 'no'}",
                            rise_eta_text,
                            rise_local_text,
                            f"X Actual: {x_actual:.3f}" if x_actual is not None else "X Actual: -",
                            f"Y Actual: {y_actual:.3f}" if y_actual is not None else "Y Actual: -",
                            f"X Cmd Error: {x_error:.3f}",
                            f"Y Cmd Error: {y_error:.3f}",
                            f"X Traj Error: {x_traj_error:.3f}",
                            f"Y Traj Error: {y_traj_error:.3f}",
                            f"X Corr: {cmd_x_correction:.3f}",
                            f"Y Corr: {cmd_y_correction:.3f}",
                            f"Y I: {cmd_y_i_term:.3f}",
                            f"Y D: {cmd_y_d_term:.3f}",
                            f"Y Unwind: {'YES' if y_integral_unwind_applied else 'no'}",
                            prepoint_status,
                        ]
                    )
                )
                last_raw_angles = (raw_x_angle, raw_y_angle)
                last_display_command = (cmd_x_angle, cmd_y_angle)
                last_display_target_elapsed = target_elapsed_sec

                if sample["visible"]:
                    was_visible = True
                    prepointed = False

                if should_track_now or sample["visible"]:
                    if not self.running or sat_index != self.current_sat_index:
                        break

                    if self.control:
                        if should_track_now:
                            if enter_tracking_now and self.tracking_gains:
                                try:
                                    self.control.set_gains_all(*self.tracking_gains)
                                except Exception:
                                    pass
                            command_send_start = time.monotonic()
                            if use_velocity_tracking_mode:
                                update_odrive_axes_velocity_only(
                                    cmd_x_vel_sent,
                                    cmd_y_vel_sent,
                                    self.control,
                                )
                            else:
                                update_odrive_axes_with_velocity(
                                    cmd_x_sent,
                                    cmd_y_sent,
                                    cmd_x_vel_ff,
                                    cmd_y_vel_ff,
                                    self.control,
                                )
                            command_send_done = time.monotonic()
                            command_send_ms = (command_send_done - command_send_start) * 1000.0
                            command_send_elapsed_sec = command_send_done - tracking_start_monotonic
                            last_tracking_command = (
                                (cmd_x_vel_sent, cmd_y_vel_sent)
                                if use_velocity_tracking_mode
                                else (cmd_x_sent, cmd_y_sent)
                            )
                            if enter_tracking_now:
                                tracking_phase = "TRACKING"
                                tracking_started = True
                            self.update_error_plot(continuous_elapsed_sec, x_traj_error, y_traj_error)
                        else:
                            command_send_start = time.monotonic()
                            update_odrive_axes(x_angle, y_angle, self.control)
                            command_send_done = time.monotonic()
                            command_send_ms = (command_send_done - command_send_start) * 1000.0
                            command_send_elapsed_sec = command_send_done - tracking_start_monotonic
                            last_tracking_command = None
                    if not tracking_started and sample["visible"]:
                        prepoint_status = "Catching up to track"
                else:
                    if not self.running or sat_index != self.current_sat_index:
                        break

                    if prepointed and self.control:
                        update_odrive_axes(x_angle, y_angle, self.control)
                    last_tracking_command = None
                    last_display_command = None
                    last_display_target_elapsed = None

                    if was_visible:
                        stow_axes()
                        self.running = False
                        break

                    tracking_started = False
                    tracking_phase = "PREPOSITIONING"
                    settled_cycles = 0
                    last_tracking_command = None
                    last_display_command = None
                    last_display_target_elapsed = None

                loop_total_ms = (time.monotonic() - loop_start_monotonic) * 1000.0
                if meas_mid_elapsed_sec is not None and command_send_elapsed_sec is not None:
                    prev_meas_to_command_delay_sec = command_send_elapsed_sec - meas_mid_elapsed_sec
                if replay_ended:
                    break

                log_writer.writerow(
                    [
                        now_utc.isoformat(),
                        f"{loop_start_monotonic:.6f}",
                        f"{continuous_elapsed_sec:.6f}",
                        f"{elapsed_sec:.6f}",
                        int(bool(sample["visible"])),
                        displayed_tracking_started,
                        int(bool(rebuilt_trajectory)),
                        f"{target_elapsed_sec:.6f}",
                        f"{ideal_target_elapsed_sec:.6f}",
                        f"{az_deg:.6f}",
                        f"{el_deg:.6f}",
                        f"{raw_x_angle:.6f}",
                        f"{raw_y_angle:.6f}",
                        f"{cmd_x_angle:.6f}",
                        f"{cmd_y_angle:.6f}",
                        f"{cmd_x_p_term:.6f}",
                        f"{cmd_y_p_term:.6f}",
                        f"{cmd_x_i_term:.6f}",
                        f"{cmd_y_i_term:.6f}",
                        f"{cmd_x_d_term:.6f}",
                        f"{cmd_y_d_term:.6f}",
                        str(y_integral_unwind_applied),
                        f"{y_error_integral:.6f}",
                        f"{cmd_x_correction:.6f}",
                        f"{cmd_y_correction:.6f}",
                        f"{cmd_x_sent:.6f}",
                        f"{cmd_y_sent:.6f}",
                        f"{raw_dx:.6f}",
                        f"{raw_dy:.6f}",
                        f"{cmd_dx:.6f}",
                        f"{cmd_dy:.6f}",
                        f"{x_vel:.6f}",
                        f"{y_vel:.6f}",
                        f"{x_acc:.6f}",
                        f"{y_acc:.6f}",
                        f"{cmd_x_vel_ff:.6f}",
                        f"{cmd_y_vel_ff:.6f}",
                        f"{cmd_x_vel_sent:.6f}",
                        f"{cmd_y_vel_sent:.6f}",
                        f"{x_actual_vel_raw:.6f}",
                        f"{y_actual_vel_raw:.6f}",
                        f"{x_actual_vel:.6f}",
                        f"{y_actual_vel:.6f}",
                        f"{x_vel_error:.6f}",
                        f"{y_vel_error:.6f}",
                        str(int(use_velocity_tracking_mode)),
                        "" if x_control_mode is None else str(x_control_mode),
                        "" if x_input_mode is None else str(x_input_mode),
                        "" if y_control_mode is None else str(y_control_mode),
                        "" if y_input_mode is None else str(y_input_mode),
                        "" if x_controller_vel_limit is None else f"{x_controller_vel_limit:.6f}",
                        "" if y_controller_vel_limit is None else f"{y_controller_vel_limit:.6f}",
                        "" if x_traj_vel_limit is None else f"{x_traj_vel_limit:.6f}",
                        "" if x_traj_accel_limit is None else f"{x_traj_accel_limit:.6f}",
                        "" if x_traj_decel_limit is None else f"{x_traj_decel_limit:.6f}",
                        "" if y_traj_vel_limit is None else f"{y_traj_vel_limit:.6f}",
                        "" if y_traj_accel_limit is None else f"{y_traj_accel_limit:.6f}",
                        "" if y_traj_decel_limit is None else f"{y_traj_decel_limit:.6f}",
                        "" if x_actual is None else f"{x_actual:.6f}",
                        "" if y_actual is None else f"{y_actual:.6f}",
                        f"{x_error:.6f}",
                        f"{y_error:.6f}",
                        f"{x_traj_error:.6f}",
                        f"{y_traj_error:.6f}",
                        "" if capture_error is None else f"{capture_error:.6f}",
                        f"{sgp4_live_eval_ms:.6f}",
                        f"{sgp4_ideal_eval_ms:.6f}",
                        f"{sgp4_command_eval_ms:.6f}",
                        f"{spi_x_read_ms:.6f}",
                        f"{spi_y_read_ms:.6f}",
                        f"{spi_total_ms:.6f}",
                        f"{control_snapshot_ms:.6f}",
                        f"{command_send_ms:.6f}",
                        f"{loop_total_ms:.6f}",
                        f"{observed_loop_period_sec:.6f}",
                        f"{target_lookahead_sec:.6f}",
                        f"{used_meas_to_command_delay_sec:.6f}",
                        "" if meas_x_elapsed_sec is None else f"{meas_x_elapsed_sec:.6f}",
                        "" if meas_y_elapsed_sec is None else f"{meas_y_elapsed_sec:.6f}",
                        "" if meas_mid_elapsed_sec is None else f"{meas_mid_elapsed_sec:.6f}",
                        "" if command_send_elapsed_sec is None else f"{command_send_elapsed_sec:.6f}",
                        prepoint_status,
                    ]
                )
                replay_writer.writerow(
                    [
                        now_utc.isoformat(),
                        f"{continuous_elapsed_sec:.6f}",
                        int(bool(sample["visible"])),
                        displayed_tracking_started,
                        f"{target_elapsed_sec:.6f}",
                        f"{ideal_target_elapsed_sec:.6f}",
                        f"{az_deg:.6f}",
                        f"{el_deg:.6f}",
                        f"{raw_x_angle:.6f}",
                        f"{raw_y_angle:.6f}",
                        f"{cmd_x_angle:.6f}",
                        f"{cmd_y_angle:.6f}",
                        f"{cmd_x_p_term:.6f}",
                        f"{cmd_y_p_term:.6f}",
                        f"{cmd_x_i_term:.6f}",
                        f"{cmd_y_i_term:.6f}",
                        f"{cmd_x_d_term:.6f}",
                        f"{cmd_y_d_term:.6f}",
                        str(y_integral_unwind_applied),
                        f"{y_error_integral:.6f}",
                        f"{x_vel:.6f}",
                        f"{y_vel:.6f}",
                        f"{x_acc:.6f}",
                        f"{y_acc:.6f}",
                        f"{cmd_x_vel_ff:.6f}",
                        f"{cmd_y_vel_ff:.6f}",
                        f"{cmd_x_vel_sent:.6f}",
                        f"{cmd_y_vel_sent:.6f}",
                        f"{x_actual_vel_raw:.6f}",
                        f"{y_actual_vel_raw:.6f}",
                        f"{x_actual_vel:.6f}",
                        f"{y_actual_vel:.6f}",
                        f"{x_vel_error:.6f}",
                        f"{y_vel_error:.6f}",
                        str(int(use_velocity_tracking_mode)),
                        prepoint_status,
                    ]
                )
                log_file.flush()
                replay_file.flush()
                time.sleep(self.track_command_interval_sec)
        finally:
            try:
                log_file.close()
            except Exception:
                pass
            try:
                replay_file.close()
            except Exception:
                pass
            if self.control:
                try:
                    self.control.exit_tracking_mode_all()
                except Exception:
                    pass
                try:
                    self.control.exit_velocity_mode_all()
                except Exception:
                    pass
                if self.preposition_gains:
                    try:
                        self.control.set_gains_all(*self.preposition_gains)
                    except Exception:
                        pass

    def apply_location(self):
        try:
            self.observer_lat = float(self.lat_entry.get())
            self.observer_lon = float(self.lon_entry.get())

            self.location_label.config(text=f"Location: {self.observer_lat:.4f}, {self.observer_lon:.4f}")

            messagebox.showinfo(
                "Location Set",
                f"Observer location set to:\nLat {self.observer_lat}\nLon {self.observer_lon}",
            )
        except Exception:
            messagebox.showerror("Error", "Invalid latitude or longitude")


if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()

    SatelliteTrackingWindow(root)

    root.mainloop()
