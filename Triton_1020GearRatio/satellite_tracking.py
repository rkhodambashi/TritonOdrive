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
TRACK_MAX_DEGREE = 91.0
TRACK_MIN_DEGREE = -91.0


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


def clamp_tracking_angles(x_angle_deg, y_angle_deg):
    return (
        max(min(x_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE),
        max(min(y_angle_deg, TRACK_MAX_DEGREE), TRACK_MIN_DEGREE),
    )


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

        self.running = False
        self.tracking_thread = None
        self.current_sat_index = None
        self.error_time = deque(maxlen=1200)
        self.x_error_history = deque(maxlen=1200)
        self.y_error_history = deque(maxlen=1200)

        self.satellites = []
        self.display_map = []
        self.satellite_labels = []

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

        tle_frame = ttk.LabelFrame(top_frame, text="TLE And Satellite")
        tle_frame.pack(side="left", fill="x", expand=True, padx=10, anchor="n")

        ttk.Label(tle_frame, text="Select TLE File:").grid(row=0, column=0, sticky="w", padx=5, pady=(5, 2))
        ttk.Button(tle_frame, text="Browse", command=self.load_tle).grid(row=0, column=1, sticky="w", padx=5, pady=(5, 2))

        self.tle_file_label = ttk.Label(tle_frame, text="No file selected")
        self.tle_file_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        ttk.Label(tle_frame, text="Select Satellite:").grid(row=2, column=0, sticky="w", padx=5, pady=(8, 2))
        self.sat_combobox = ttk.Combobox(tle_frame, state="readonly", width=48)
        self.sat_combobox.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=2)
        ttk.Button(tle_frame, text="Refresh Pass Times", command=self.refresh_satellite_display).grid(
            row=2, column=1, sticky="e", padx=5, pady=(8, 2)
        )

        button_row = ttk.Frame(tle_frame)
        button_row.grid(row=4, column=0, columnspan=2, sticky="w", padx=5, pady=8)
        ttk.Button(button_row, text="Start Tracking", command=self.start_tracking_thread).pack(side="left", padx=(0, 8))
        ttk.Button(button_row, text="Stop Tracking / Stow", command=self.stop_tracking).pack(side="left")

        tle_frame.columnconfigure(0, weight=1)

        settings_frame = ttk.LabelFrame(top_frame, text="Tracking Settings")
        settings_frame.pack(side="right", anchor="n")

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
        self.error_ax_x.relim()
        self.error_ax_x.autoscale_view()
        self.error_ax_y.relim()
        self.error_ax_y.autoscale_view()
        self.error_canvas.draw_idle()

    def set_output_text(self, text):
        self.output_text.configure(state="normal")
        self.output_text.delete("1.0", tk.END)
        self.output_text.insert("1.0", text)
        self.output_text.configure(state="disabled")

    def format_output_columns(self, lines, columns=2, gutter=4):
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
        self.error_ax_x.relim()
        self.error_ax_x.autoscale_view()
        self.error_ax_y.relim()
        self.error_ax_y.autoscale_view()
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
            ):
                raise ValueError
            if point_spacing > horizon:
                raise ValueError
            if rebuild_margin >= horizon:
                raise ValueError
            if min_spacing > point_spacing:
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
        except Exception:
            messagebox.showerror(
                "Error",
                (
                    "Tracking settings must be positive numbers.\n"
                    "Lead can be zero or positive.\n"
                    "Point spacing cannot exceed horizon.\n"
                    "Rebuild margin must be smaller than horizon.\n"
                    "Min spacing cannot exceed point spacing."
                ),
            )

    def load_tle(self):
        file_path = filedialog.askopenfilename(filetypes=[("TLE Files", "*.txt"), ("All Files", "*.*")])
        if not file_path:
            return

        self.tle_file_label.config(text=file_path)

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

                if next_rise:
                    dt = next_rise - now_utc
                    minutes = max(0, int(dt.total_seconds() / 60))
                    local_rise_text = format_local_datetime(next_rise)
                    label = f"{sat.name} [{sat.model.satnum}] - {local_rise_text} local (in {minutes} min)"
                    sort_key = dt.total_seconds()
                else:
                    label = f"{sat.name} [{sat.model.satnum}] - No pass soon"
                    sort_key = 999999999

            except Exception:
                label = f"{sat.name} [{sat.model.satnum}]"
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
        if not self.satellites:
            messagebox.showerror("Error", "Load TLE first")
            return

        sel = self.sat_combobox.current()

        if sel < 0:
            messagebox.showerror("Error", "Select satellite")
            return

        sat_index = self.display_map[sel]

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
        sat = self.satellites[sat_index]
        self.set_output_text(
            self.format_output_columns(
                [
                    f"Satellite: {sat.name}",
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
        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)
        sat = self.satellites[sat_index]
        self.set_output_text(
            self.format_output_columns(
                [
                    f"Satellite: {sat.name}",
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
        safe_sat_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in sat.name).strip("_") or "satellite"
        log_path = log_dir / f"{safe_sat_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            [
                "utc_time",
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
        next_rise_utc = get_next_rise_time(ts, observer, sat)
        prepointed = False
        tracking_start_monotonic = time.monotonic()

        if self.control and self.preposition_gains:
            try:
                self.control.set_gains_all(*self.preposition_gains)
            except Exception:
                pass

        try:
            while self.running and sat_index == self.current_sat_index:
                now_monotonic = time.monotonic()
                now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
                continuous_elapsed_sec = now_monotonic - tracking_start_monotonic
                rebuilt_trajectory = False

                derivative_dt = min(self.track_command_interval_sec, self.trajectory_point_spacing_sec)
                elapsed_sec = continuous_elapsed_sec
                live_sample = sample_tracking_state_utc(ts, observer, sat, now_utc, derivative_dt)
                if live_sample is None:
                    time.sleep(self.track_command_interval_sec)
                    continue
                sample = live_sample

                prepoint_status = "Tracking pass"
                rise_eta_text = "Rise ETA: -"
                rise_local_text = "Rise Local: -"
                pickup_window_active = False
                prepoint_target_sample = None
                if next_rise_utc and not was_visible:
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
                raw_x_angle = sample["x_angle"]
                raw_y_angle = sample["y_angle"]
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
                    target_lookahead_sec = self.feedforward_lead_sec if TRACK_USE_LEAD else self.track_command_interval_sec
                    ideal_target_elapsed_sec = continuous_elapsed_sec + target_lookahead_sec
                    target_elapsed_sec = ideal_target_elapsed_sec
                    if tracking_started and TRACK_USE_TIME_SPLIT:
                        command_sample = sample_tracking_state_utc(
                            ts,
                            observer,
                            sat,
                            now_utc + datetime.timedelta(seconds=target_lookahead_sec),
                            derivative_dt,
                        )
                    else:
                        command_sample = sample_tracking_state_utc(
                            ts,
                            observer,
                            sat,
                            now_utc + datetime.timedelta(seconds=target_lookahead_sec),
                            derivative_dt,
                        )

                if command_sample is None:
                    time.sleep(self.track_command_interval_sec)
                    continue

                x_angle = command_sample["x_angle"]
                y_angle = command_sample["y_angle"]
                cmd_x_vel_ff = command_sample.get("x_vel", x_vel)
                cmd_y_vel_ff = command_sample.get("y_vel", y_vel)

                x_angle, y_angle = clamp_tracking_angles(x_angle, y_angle)
                cmd_x_angle = x_angle
                cmd_y_angle = y_angle
                raw_dx = 0.0 if last_raw_angles is None else raw_x_angle - last_raw_angles[0]
                raw_dy = 0.0 if last_raw_angles is None else raw_y_angle - last_raw_angles[1]
                cmd_dx = 0.0 if last_display_command is None else cmd_x_angle - last_display_command[0]
                cmd_dy = 0.0 if last_display_command is None else cmd_y_angle - last_display_command[1]
                x_actual = None
                y_actual = None
                x_error = 0.0
                y_error = 0.0
                x_traj_error = 0.0
                y_traj_error = 0.0
                capture_error = None
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
                        ideal_sample = sample_tracking_state_utc(ts, observer, sat, now_utc, derivative_dt)
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
                        x_actual = self.control.get_spi_position("x")
                        y_actual = self.control.get_spi_position("y")
                        x_error = cmd_x_angle - x_actual
                        y_error = cmd_y_angle - y_actual
                        if ideal_sample is not None:
                            ideal_x_angle, ideal_y_angle = clamp_tracking_angles(
                                ideal_sample["x_angle"],
                                ideal_sample["y_angle"],
                            )
                            x_traj_error = ideal_x_angle - x_actual
                            y_traj_error = ideal_y_angle - y_actual
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

                if TRACK_USE_PASSTHROUGH and enter_tracking_now and self.control:
                    try:
                        self.control.enter_tracking_mode_all(input_filter_bandwidth=self.track_filter_bandwidth_hz)
                    except Exception:
                        pass

                if should_track_now:
                    target_lookahead_sec = self.feedforward_lead_sec if TRACK_USE_LEAD else self.track_command_interval_sec
                    ideal_target_elapsed_sec = continuous_elapsed_sec + target_lookahead_sec
                    target_elapsed_sec = ideal_target_elapsed_sec
                    tracking_command_sample = sample_tracking_state_utc(
                        ts,
                        observer,
                        sat,
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
                        cmd_x_vel_ff = command_sample.get("x_vel", cmd_x_vel_ff)
                        cmd_y_vel_ff = command_sample.get("y_vel", cmd_y_vel_ff)
                        if x_actual is not None and y_actual is not None:
                            x_error = cmd_x_angle - x_actual
                            y_error = cmd_y_angle - y_actual
                            capture_error = max(abs(x_error), abs(y_error))
                        if not sample["visible"]:
                            prepoint_status = "Below-horizon pickup"

                displayed_tracking_started = int(bool(tracking_phase == "TRACKING" or enter_tracking_now))

                self.set_output_text(
                    self.format_output_columns(
                        [
                            f"Satellite: {sat.name}",
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
                            f"X Acc: {x_acc:.3f}",
                            f"Y Acc: {y_acc:.3f}",
                            f"Rebuilt: {'YES' if rebuilt_trajectory else 'no'}",
                            rise_eta_text,
                            rise_local_text,
                            f"X Actual: {x_actual:.3f}" if x_actual is not None else "X Actual: -",
                            f"Y Actual: {y_actual:.3f}" if y_actual is not None else "Y Actual: -",
                            f"X Cmd Error: {x_error:.3f}",
                            f"Y Cmd Error: {y_error:.3f}",
                            f"X Traj Error: {x_traj_error:.3f}",
                            f"Y Traj Error: {y_traj_error:.3f}",
                            prepoint_status,
                        ]
                    )
                )
                log_writer.writerow(
                    [
                        now_utc.isoformat(),
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
                        prepoint_status,
                    ]
                )
                log_file.flush()
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
                            update_odrive_axes_with_velocity(
                                cmd_x_angle,
                                cmd_y_angle,
                                cmd_x_vel_ff,
                                cmd_y_vel_ff,
                                self.control,
                            )
                            last_tracking_command = (cmd_x_angle, cmd_y_angle)
                            if enter_tracking_now:
                                tracking_phase = "TRACKING"
                                tracking_started = True
                            self.update_error_plot(continuous_elapsed_sec, x_traj_error, y_traj_error)
                        else:
                            update_odrive_axes(x_angle, y_angle, self.control)
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

                        self.running = False
                        break

                    tracking_started = False
                    tracking_phase = "PREPOSITIONING"
                    settled_cycles = 0
                    last_tracking_command = None
                    last_display_command = None
                    last_display_target_elapsed = None

                time.sleep(self.track_command_interval_sec)
        finally:
            try:
                log_file.close()
            except Exception:
                pass
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
