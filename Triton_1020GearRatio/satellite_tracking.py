import csv
import datetime
import threading
import time
import traceback
import tkinter as tk
from collections import deque
from math import acos, atan, cos, pi, sin, sqrt
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import matplotlib
from odrive import enums as odrive_enums
from skyfield.api import EarthSatellite, Topos, load, utc
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    from endat_serial_reader import (
        DEFAULT_BAUD as ENDAT_DEFAULT_BAUD,
        DEFAULT_COUNTS_PER_REV as ENDAT_DEFAULT_COUNTS_PER_REV,
        EndatSerialReader,
    )
except Exception:
    ENDAT_DEFAULT_BAUD = 115200
    ENDAT_DEFAULT_COUNTS_PER_REV = 1 << 25
    EndatSerialReader = None

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
TRACK_X_POSITION_GAIN_VEL_LOW_DEG_S = 2.0
TRACK_X_POSITION_GAIN_VEL_HIGH_DEG_S = 3.0
TRACK_X_POSITION_GAIN_SCALE_LOW = 1.0
TRACK_X_POSITION_GAIN_SCALE_HIGH = 0.5
TRACK_Y_POSITION_GAIN_VEL_LOW_DEG_S = 0.2
TRACK_Y_POSITION_GAIN_VEL_HIGH_DEG_S = 2.0
TRACK_Y_POSITION_GAIN_SCALE_LOW = 1.0
TRACK_Y_POSITION_GAIN_SCALE_HIGH = 1.0
TRACK_VELOCITY_INTEGRAL_GAIN_X = 0.0
TRACK_VELOCITY_INTEGRAL_GAIN_Y = 0.0
TRACK_VELOCITY_INTEGRAL_MAX_STATE_X = 10.0
TRACK_VELOCITY_INTEGRAL_MAX_STATE_Y = 10.0
TRACK_VELOCITY_POSITION_DERIVATIVE_GAIN_X = 0.0
TRACK_VELOCITY_POSITION_DERIVATIVE_GAIN_Y = 0.0
TRACK_VELOCITY_POSITION_DERIVATIVE_FILTER_ALPHA_X = 0.0
TRACK_VELOCITY_POSITION_DERIVATIVE_FILTER_ALPHA_Y = 0.0
TRACK_VELOCITY_ERROR_GAIN_X = 0.0
TRACK_VELOCITY_ERROR_GAIN_Y = 0.0
TRACK_X_DAMPING_GAIN_LOW = 1.0
TRACK_X_DAMPING_GAIN_MID = 1.0
TRACK_X_DAMPING_GAIN_HIGH = 1.0
TRACK_X_DAMPING_ANGLE_LOW = 20.0
TRACK_X_DAMPING_ANGLE_HIGH = 40.0
TRACK_Y_DAMPING_GAIN_LOW = 1.0
TRACK_Y_DAMPING_GAIN_MID = 1.0
TRACK_Y_DAMPING_GAIN_HIGH = 1.0
TRACK_Y_DAMPING_ANGLE_LOW = 20.0
TRACK_Y_DAMPING_ANGLE_HIGH = 40.0
TRACK_SPI_ENCODER_BITS = 23
TRACK_SPI_POSITION_FILTER_ALPHA_X = 0.0
TRACK_SPI_POSITION_FILTER_ALPHA_Y = 0.0
TRACK_SPI_POSITION_SKIP_BITS_X = 0
TRACK_SPI_POSITION_SKIP_BITS_Y = 0
TRACK_SPI_VELOCITY_FILTER_ALPHA_X = 0.0
TRACK_SPI_VELOCITY_FILTER_ALPHA_Y = 0.0
TRACK_BENEDICT_BORDNER_ENABLE_X = 1.0
TRACK_BENEDICT_BORDNER_ENABLE_Y = 1.0
TRACK_BENEDICT_BORDNER_ALPHA_X = 0.35
TRACK_BENEDICT_BORDNER_ALPHA_Y = 0.35
TRACK_BENEDICT_BORDNER_BETA_SCALE_X = 1.0
TRACK_BENEDICT_BORDNER_BETA_SCALE_Y = 1.0
TRACK_VELOCITY_COMMAND_MAX_X = 100.0
TRACK_VELOCITY_COMMAND_MAX_Y = 100.0
TRACK_VELOCITY_RAMP_RATE_DEG_S2 = 5.0
TRACK_COMMAND_VELOCITY_FILTER_ALPHA_X = 0.0
TRACK_COMMAND_VELOCITY_FILTER_ALPHA_Y = 0.0
TRACK_VELOCITY_CORRECTION_SLEW_RATE_X = 10.0
TRACK_VELOCITY_CORRECTION_SLEW_RATE_Y = 0.0
TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_RATE = 4.0
TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_START_DEG = 3.0
TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_STOP_DEG = 10.0
TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_RATE = 0.0
TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_START_DEG = 0.0
TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_STOP_DEG = 10.0
TRACK_VELOCITY_CONTROLLER_MODE = "full"
TRACK_VELOCITY_DEBUG_SCALE = 1.0
TRACK_X_MOTOR_VEL_GAIN_TOWARD_LOW = 0.3
TRACK_X_MOTOR_VEL_GAIN_TOWARD_MID = 1.0
TRACK_X_MOTOR_VEL_GAIN_TOWARD_HIGH = 1.5
TRACK_X_MOTOR_VEL_GAIN_AWAY_LOW = 0.1
TRACK_X_MOTOR_VEL_GAIN_AWAY_MID = 0.3
TRACK_X_MOTOR_VEL_GAIN_AWAY_HIGH = 1.0
TRACK_X_MOTOR_VEL_GAIN_TOWARD_ANGLE_LOW = 55.0
TRACK_X_MOTOR_VEL_GAIN_TOWARD_ANGLE_HIGH = 85.0
TRACK_X_MOTOR_VEL_GAIN_AWAY_ANGLE_LOW = 55.0
TRACK_X_MOTOR_VEL_GAIN_AWAY_ANGLE_HIGH = 85.0
TRACK_X_MOTOR_VEL_GAIN_RAMP_IN_SEC = 4.0
TRACK_X_VEL_FF_ATTENUATE_START_DEG = 1.0
TRACK_X_VEL_FF_ATTENUATE_STOP_DEG = 10.0
TRACK_X_VEL_FF_ATTENUATE_SCALE = 1.0
TRACK_Y_MOTOR_VEL_GAIN_TOWARD_LOW = 1.0
TRACK_Y_MOTOR_VEL_GAIN_TOWARD_MID = 1.0
TRACK_Y_MOTOR_VEL_GAIN_TOWARD_HIGH = 1.0
TRACK_Y_MOTOR_VEL_GAIN_AWAY_LOW = 1.0
TRACK_Y_MOTOR_VEL_GAIN_AWAY_MID = 1.0
TRACK_Y_MOTOR_VEL_GAIN_AWAY_HIGH = 1.0
TRACK_Y_MOTOR_VEL_GAIN_TOWARD_ANGLE_LOW = 55.0
TRACK_Y_MOTOR_VEL_GAIN_TOWARD_ANGLE_HIGH = 85.0
TRACK_Y_MOTOR_VEL_GAIN_AWAY_ANGLE_LOW = 55.0
TRACK_Y_MOTOR_VEL_GAIN_AWAY_ANGLE_HIGH = 85.0
TRACK_Y_MOTOR_VEL_GAIN_RAMP_IN_SEC = 4.0
TRACK_Y_VEL_FF_ATTENUATE_START_DEG = 100.0
TRACK_Y_VEL_FF_ATTENUATE_STOP_DEG = 101.0
TRACK_Y_VEL_FF_ATTENUATE_SCALE = 1.0
TRACK_ENDAT_X_ENABLE = False
TRACK_ENDAT_X_USE_FOR_CONTROL = False
TRACK_ENDAT_X_PORT = "COM6"
TRACK_ENDAT_X_BAUD = ENDAT_DEFAULT_BAUD
TRACK_ENDAT_X_COUNTS_PER_REV = ENDAT_DEFAULT_COUNTS_PER_REV
TRACK_ENDAT_X_ZERO_COUNT = 0
TRACK_ENDAT_X_SIGN = 1.0

ENCODER_ID_NAMES = {
    value: name
    for name, value in vars(odrive_enums).items()
    if name.startswith("ENCODER_ID_") and isinstance(value, int)
}


def get_local_timezone():
    return datetime.datetime.now().astimezone().tzinfo


def encoder_id_to_name(encoder_id):
    if encoder_id is None:
        return None
    return ENCODER_ID_NAMES.get(int(encoder_id), f"ENCODER_ID_UNKNOWN_{int(encoder_id)}")


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


def attenuate_x_velocity_feedforward(
    x_angle_deg,
    x_vel_ff_deg_s,
    attenuate_start_deg=TRACK_X_VEL_FF_ATTENUATE_START_DEG,
    attenuate_stop_deg=TRACK_X_VEL_FF_ATTENUATE_STOP_DEG,
    attenuate_scale=TRACK_X_VEL_FF_ATTENUATE_SCALE,
):
    abs_x_angle_deg = abs(x_angle_deg)
    if abs_x_angle_deg <= attenuate_start_deg:
        return attenuate_scale * x_vel_ff_deg_s
    if abs_x_angle_deg >= attenuate_stop_deg:
        return x_vel_ff_deg_s

    span_deg = attenuate_stop_deg - attenuate_start_deg
    alpha = (abs_x_angle_deg - attenuate_start_deg) / span_deg
    weight = blend_value(attenuate_scale, 1.0, alpha)
    return weight * x_vel_ff_deg_s


def scheduled_x_motor_vel_gain(x_angle_deg, low_gain, mid_gain, high_gain, low_angle_deg, high_angle_deg):
    abs_x_angle_deg = abs(x_angle_deg)
    if abs_x_angle_deg < low_angle_deg:
        return low_gain
    if abs_x_angle_deg < high_angle_deg:
        return mid_gain
    return high_gain


def scheduled_velocity_scale(abs_velocity_deg_s, low_scale, high_scale, low_velocity_deg_s, high_velocity_deg_s):
    if abs_velocity_deg_s <= low_velocity_deg_s:
        return low_scale
    if abs_velocity_deg_s >= high_velocity_deg_s:
        return high_scale

    span_deg_s = high_velocity_deg_s - low_velocity_deg_s
    alpha = (abs_velocity_deg_s - low_velocity_deg_s) / span_deg_s
    return blend_value(low_scale, high_scale, alpha)


def scheduled_zero_slew_rate(axis_angle_deg, far_slew_rate, zero_slew_rate, zero_start_deg, zero_stop_deg):
    if far_slew_rate <= 0.0:
        return 0.0

    abs_axis_angle_deg = abs(axis_angle_deg)
    if abs_axis_angle_deg <= zero_start_deg:
        return zero_slew_rate
    if abs_axis_angle_deg >= zero_stop_deg:
        return far_slew_rate

    span_deg = zero_stop_deg - zero_start_deg
    alpha = (abs_axis_angle_deg - zero_start_deg) / span_deg
    return blend_value(zero_slew_rate, far_slew_rate, alpha)


def quantize_spi_position_deg(position_deg, skip_bits):
    skip_bits = int(skip_bits)
    if skip_bits <= 0:
        return position_deg

    effective_bits = TRACK_SPI_ENCODER_BITS - skip_bits
    step_deg = 360.0 / float(1 << effective_bits)
    return round(position_deg / step_deg) * step_deg


def wrap_degrees_signed(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def endat_angle_to_axis_deg(angle_deg, sign):
    return float(sign) * wrap_degrees_signed(angle_deg)


def benedict_bordner_beta(alpha, beta_scale):
    if alpha <= 0.0:
        return 0.0
    return beta_scale * (alpha * alpha) / (2.0 - alpha)


class BenedictBordnerState:
    def __init__(self):
        self.position_deg = None
        self.velocity_deg_s = 0.0
        self.predicted_position_deg = None
        self.residual_deg = 0.0

    def reset(self):
        self.position_deg = None
        self.velocity_deg_s = 0.0
        self.predicted_position_deg = None
        self.residual_deg = 0.0

    def update(self, measured_position_deg, dt_sec, alpha, beta):
        if self.position_deg is None or dt_sec <= 0.0:
            self.position_deg = measured_position_deg
            self.velocity_deg_s = 0.0
            self.predicted_position_deg = measured_position_deg
            self.residual_deg = 0.0
            return self.position_deg, self.velocity_deg_s, self.predicted_position_deg, self.residual_deg

        self.predicted_position_deg = self.position_deg + self.velocity_deg_s * dt_sec
        self.residual_deg = measured_position_deg - self.predicted_position_deg
        self.position_deg = self.predicted_position_deg + alpha * self.residual_deg
        self.velocity_deg_s = self.velocity_deg_s + beta * self.residual_deg / dt_sec
        return self.position_deg, self.velocity_deg_s, self.predicted_position_deg, self.residual_deg


def moving_toward_vertical(x_angle_deg, x_velocity_deg_s):
    if abs(x_velocity_deg_s) <= 1e-9:
        return False
    return x_angle_deg * x_velocity_deg_s < 0.0


def blend_value(start_value, end_value, alpha):
    alpha = max(0.0, min(1.0, alpha))
    return start_value + alpha * (end_value - start_value)


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
        tracking_gains_y=None,
    ):
        super().__init__(parent)

        self.title("Satellite Tracking")
        self.geometry("1080x980")

        self.odrvs = odrvs or {}
        self.control = control
        self.preposition_gains = preposition_gains
        self.tracking_gains = tracking_gains
        self.tracking_gains_y = tracking_gains_y or tracking_gains

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
        self.x_position_gain_vel_low_deg_s = TRACK_X_POSITION_GAIN_VEL_LOW_DEG_S
        self.x_position_gain_vel_high_deg_s = TRACK_X_POSITION_GAIN_VEL_HIGH_DEG_S
        self.x_position_gain_scale_low = TRACK_X_POSITION_GAIN_SCALE_LOW
        self.x_position_gain_scale_high = TRACK_X_POSITION_GAIN_SCALE_HIGH
        self.y_position_gain_vel_low_deg_s = TRACK_Y_POSITION_GAIN_VEL_LOW_DEG_S
        self.y_position_gain_vel_high_deg_s = TRACK_Y_POSITION_GAIN_VEL_HIGH_DEG_S
        self.y_position_gain_scale_low = TRACK_Y_POSITION_GAIN_SCALE_LOW
        self.y_position_gain_scale_high = TRACK_Y_POSITION_GAIN_SCALE_HIGH
        self.velocity_integral_gain_x = TRACK_VELOCITY_INTEGRAL_GAIN_X
        self.velocity_integral_gain_y = TRACK_VELOCITY_INTEGRAL_GAIN_Y
        self.velocity_position_derivative_gain_x = TRACK_VELOCITY_POSITION_DERIVATIVE_GAIN_X
        self.velocity_position_derivative_gain_y = TRACK_VELOCITY_POSITION_DERIVATIVE_GAIN_Y
        self.velocity_position_derivative_filter_alpha_x = TRACK_VELOCITY_POSITION_DERIVATIVE_FILTER_ALPHA_X
        self.velocity_position_derivative_filter_alpha_y = TRACK_VELOCITY_POSITION_DERIVATIVE_FILTER_ALPHA_Y
        self.velocity_error_gain_x = TRACK_VELOCITY_ERROR_GAIN_X
        self.velocity_error_gain_y = TRACK_VELOCITY_ERROR_GAIN_Y
        self.x_damping_gain_low = TRACK_X_DAMPING_GAIN_LOW
        self.x_damping_gain_mid = TRACK_X_DAMPING_GAIN_MID
        self.x_damping_gain_high = TRACK_X_DAMPING_GAIN_HIGH
        self.x_damping_angle_low = TRACK_X_DAMPING_ANGLE_LOW
        self.x_damping_angle_high = TRACK_X_DAMPING_ANGLE_HIGH
        self.y_damping_gain_low = TRACK_Y_DAMPING_GAIN_LOW
        self.y_damping_gain_mid = TRACK_Y_DAMPING_GAIN_MID
        self.y_damping_gain_high = TRACK_Y_DAMPING_GAIN_HIGH
        self.y_damping_angle_low = TRACK_Y_DAMPING_ANGLE_LOW
        self.y_damping_angle_high = TRACK_Y_DAMPING_ANGLE_HIGH
        self.spi_position_filter_alpha_x = TRACK_SPI_POSITION_FILTER_ALPHA_X
        self.spi_position_filter_alpha_y = TRACK_SPI_POSITION_FILTER_ALPHA_Y
        self.spi_position_skip_bits_x = TRACK_SPI_POSITION_SKIP_BITS_X
        self.spi_position_skip_bits_y = TRACK_SPI_POSITION_SKIP_BITS_Y
        self.spi_velocity_filter_alpha_x = TRACK_SPI_VELOCITY_FILTER_ALPHA_X
        self.spi_velocity_filter_alpha_y = TRACK_SPI_VELOCITY_FILTER_ALPHA_Y
        self.benedict_bordner_enable_x = TRACK_BENEDICT_BORDNER_ENABLE_X
        self.benedict_bordner_enable_y = TRACK_BENEDICT_BORDNER_ENABLE_Y
        self.benedict_bordner_alpha_x = TRACK_BENEDICT_BORDNER_ALPHA_X
        self.benedict_bordner_alpha_y = TRACK_BENEDICT_BORDNER_ALPHA_Y
        self.benedict_bordner_beta_scale_x = TRACK_BENEDICT_BORDNER_BETA_SCALE_X
        self.benedict_bordner_beta_scale_y = TRACK_BENEDICT_BORDNER_BETA_SCALE_Y
        self.velocity_command_max_x = TRACK_VELOCITY_COMMAND_MAX_X
        self.velocity_command_max_y = TRACK_VELOCITY_COMMAND_MAX_Y
        self.velocity_ramp_rate_deg_s2 = TRACK_VELOCITY_RAMP_RATE_DEG_S2
        self.command_velocity_filter_alpha_x = TRACK_COMMAND_VELOCITY_FILTER_ALPHA_X
        self.command_velocity_filter_alpha_y = TRACK_COMMAND_VELOCITY_FILTER_ALPHA_Y
        self.velocity_correction_slew_rate_x = TRACK_VELOCITY_CORRECTION_SLEW_RATE_X
        self.velocity_correction_slew_rate_y = TRACK_VELOCITY_CORRECTION_SLEW_RATE_Y
        self.x_velocity_correction_slew_zero_rate = TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_RATE
        self.x_velocity_correction_slew_zero_start_deg = TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_START_DEG
        self.x_velocity_correction_slew_zero_stop_deg = TRACK_X_VELOCITY_CORRECTION_SLEW_ZERO_STOP_DEG
        self.y_velocity_correction_slew_zero_rate = TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_RATE
        self.y_velocity_correction_slew_zero_start_deg = TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_START_DEG
        self.y_velocity_correction_slew_zero_stop_deg = TRACK_Y_VELOCITY_CORRECTION_SLEW_ZERO_STOP_DEG
        self.velocity_feedforward_scale_x = TRACK_VEL_FEEDFORWARD_SCALE_X
        self.velocity_feedforward_scale_y = TRACK_VEL_FEEDFORWARD_SCALE_Y
        self.x_vel_ff_attenuate_start_deg = TRACK_X_VEL_FF_ATTENUATE_START_DEG
        self.x_vel_ff_attenuate_stop_deg = TRACK_X_VEL_FF_ATTENUATE_STOP_DEG
        self.x_vel_ff_attenuate_scale = TRACK_X_VEL_FF_ATTENUATE_SCALE
        self.y_vel_ff_attenuate_start_deg = TRACK_Y_VEL_FF_ATTENUATE_START_DEG
        self.y_vel_ff_attenuate_stop_deg = TRACK_Y_VEL_FF_ATTENUATE_STOP_DEG
        self.y_vel_ff_attenuate_scale = TRACK_Y_VEL_FF_ATTENUATE_SCALE
        self.endat_x_enable = TRACK_ENDAT_X_ENABLE
        self.endat_x_use_for_control = TRACK_ENDAT_X_USE_FOR_CONTROL
        self.endat_x_port = TRACK_ENDAT_X_PORT
        self.endat_x_baud = TRACK_ENDAT_X_BAUD
        self.endat_x_counts_per_rev = TRACK_ENDAT_X_COUNTS_PER_REV
        self.endat_x_zero_count = TRACK_ENDAT_X_ZERO_COUNT
        self.endat_x_sign = TRACK_ENDAT_X_SIGN
        self.velocity_controller_mode = TRACK_VELOCITY_CONTROLLER_MODE
        self.velocity_debug_scale = TRACK_VELOCITY_DEBUG_SCALE
        y_motor_vel_gain_default = (
            self.tracking_gains_y[1] if self.tracking_gains_y else TRACK_Y_MOTOR_VEL_GAIN_TOWARD_MID
        )
        self.x_motor_vel_gain_toward_low = TRACK_X_MOTOR_VEL_GAIN_TOWARD_LOW
        self.x_motor_vel_gain_toward_mid = TRACK_X_MOTOR_VEL_GAIN_TOWARD_MID
        self.x_motor_vel_gain_toward_high = TRACK_X_MOTOR_VEL_GAIN_TOWARD_HIGH
        self.x_motor_vel_gain_away_low = TRACK_X_MOTOR_VEL_GAIN_AWAY_LOW
        self.x_motor_vel_gain_away_mid = TRACK_X_MOTOR_VEL_GAIN_AWAY_MID
        self.x_motor_vel_gain_away_high = TRACK_X_MOTOR_VEL_GAIN_AWAY_HIGH
        self.x_motor_vel_gain_toward_angle_low = TRACK_X_MOTOR_VEL_GAIN_TOWARD_ANGLE_LOW
        self.x_motor_vel_gain_toward_angle_high = TRACK_X_MOTOR_VEL_GAIN_TOWARD_ANGLE_HIGH
        self.x_motor_vel_gain_away_angle_low = TRACK_X_MOTOR_VEL_GAIN_AWAY_ANGLE_LOW
        self.x_motor_vel_gain_away_angle_high = TRACK_X_MOTOR_VEL_GAIN_AWAY_ANGLE_HIGH
        self.x_motor_vel_gain_ramp_in_sec = TRACK_X_MOTOR_VEL_GAIN_RAMP_IN_SEC
        self.y_motor_vel_gain_toward_low = y_motor_vel_gain_default
        self.y_motor_vel_gain_toward_mid = y_motor_vel_gain_default
        self.y_motor_vel_gain_toward_high = y_motor_vel_gain_default
        self.y_motor_vel_gain_away_low = y_motor_vel_gain_default
        self.y_motor_vel_gain_away_mid = y_motor_vel_gain_default
        self.y_motor_vel_gain_away_high = y_motor_vel_gain_default
        self.y_motor_vel_gain_toward_angle_low = TRACK_Y_MOTOR_VEL_GAIN_TOWARD_ANGLE_LOW
        self.y_motor_vel_gain_toward_angle_high = TRACK_Y_MOTOR_VEL_GAIN_TOWARD_ANGLE_HIGH
        self.y_motor_vel_gain_away_angle_low = TRACK_Y_MOTOR_VEL_GAIN_AWAY_ANGLE_LOW
        self.y_motor_vel_gain_away_angle_high = TRACK_Y_MOTOR_VEL_GAIN_AWAY_ANGLE_HIGH
        self.y_motor_vel_gain_ramp_in_sec = TRACK_Y_MOTOR_VEL_GAIN_RAMP_IN_SEC
        self.velocity_settings_window = None
        self.velocity_setting_entries = {}

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
        self.endat_x_enable_var = tk.IntVar(value=int(bool(self.endat_x_enable)))
        self.endat_x_use_for_control_var = tk.IntVar(value=int(bool(self.endat_x_use_for_control)))
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

        center_container = ttk.Frame(top_frame)
        center_container.pack(side="left", fill="x", expand=False, padx=10, anchor="n")

        settings_container = ttk.Frame(top_frame)
        settings_container.pack(side="right", fill="x", expand=True, anchor="n")

        tle_frame = ttk.LabelFrame(center_container, text="TLE And Satellite")
        tle_frame.pack(fill="x", expand=True, anchor="n")

        ttk.Label(tle_frame, text="Select TLE File:").grid(row=0, column=0, sticky="w", padx=5, pady=(5, 2))
        ttk.Button(tle_frame, text="Browse", command=self.load_tle).grid(row=0, column=1, sticky="w", padx=5, pady=(5, 2))

        self.tle_file_label = ttk.Label(tle_frame, text="No file selected", width=24)
        self.tle_file_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=2)

        ttk.Label(tle_frame, text="Select Satellite:").grid(row=2, column=0, sticky="w", padx=5, pady=(8, 2))
        self.sat_combobox = ttk.Combobox(tle_frame, state="readonly", width=24)
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
        self.replay_file_label = ttk.Label(tle_frame, text="No replay file selected", width=24)
        self.replay_file_label.grid(row=5, column=1, sticky="ew", padx=5, pady=(2, 2))

        button_row = ttk.Frame(tle_frame)
        button_row.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=8)
        ttk.Button(button_row, text="Start Tracking", command=self.start_tracking_thread).pack(side="left", padx=(0, 8))
        ttk.Button(button_row, text="Stop Tracking / Stow", command=self.stop_tracking).pack(side="left")

        tle_frame.columnconfigure(0, weight=0)
        tle_frame.columnconfigure(1, weight=1)

        self.vel_controller_mode_var = tk.StringVar(value=self.velocity_controller_mode)
        ttk.Button(
            top_frame,
            text="Velocity Mode Settings",
            command=self.open_velocity_settings_window,
        ).pack(side="left", anchor="n", padx=(0, 10))

        settings_frame = ttk.LabelFrame(settings_container, text="Tracking Settings")
        settings_frame.pack(side="left", anchor="n", fill="x", expand=True)

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

        ttk.Checkbutton(settings_frame, text="EnDat X", variable=self.endat_x_enable_var).grid(
            row=4, column=2, sticky="w", padx=(18, 5), pady=2
        )
        ttk.Checkbutton(settings_frame, text="Use X Ctrl", variable=self.endat_x_use_for_control_var).grid(
            row=4, column=3, sticky="w", padx=5, pady=2
        )

        ttk.Label(settings_frame, text="EnDat Port").grid(row=5, column=2, sticky="w", padx=(18, 5), pady=2)
        self.endat_x_port_entry = ttk.Entry(settings_frame, width=10)
        self.endat_x_port_entry.grid(row=5, column=3, padx=5, pady=2)
        self.endat_x_port_entry.insert(0, self.endat_x_port)

        ttk.Label(settings_frame, text="EnDat Zero").grid(row=6, column=2, sticky="w", padx=(18, 5), pady=2)
        self.endat_x_zero_entry = ttk.Entry(settings_frame, width=10)
        self.endat_x_zero_entry.grid(row=6, column=3, padx=5, pady=2)
        self.endat_x_zero_entry.insert(0, f"{self.endat_x_zero_count:g}")

        ttk.Label(settings_frame, text="EnDat Sign").grid(row=7, column=2, sticky="w", padx=(18, 5), pady=2)
        self.endat_x_sign_entry = ttk.Entry(settings_frame, width=10)
        self.endat_x_sign_entry.grid(row=7, column=3, padx=5, pady=2)
        self.endat_x_sign_entry.insert(0, f"{self.endat_x_sign:g}")

        ttk.Button(settings_frame, text="Apply", command=self.apply_tracking_settings).grid(
            row=8, column=0, columnspan=4, pady=6
        )

        settings_frame.columnconfigure(0, weight=0)
        settings_frame.columnconfigure(1, weight=1)
        settings_frame.columnconfigure(2, weight=0)
        settings_frame.columnconfigure(3, weight=1)

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

    def velocity_setting_groups(self):
        return [
            (
                "General",
                [
                    ("Vel Ramp", "velocity_ramp_rate_deg_s2"),
                    ("Vel Debug Scale", "velocity_debug_scale"),
                ],
            ),
            (
                "X Axis",
                [
                    ("Kp Pos X", "velocity_position_gain_x"),
                    ("Kp X Vel Low", "x_position_gain_vel_low_deg_s"),
                    ("Kp X Vel High", "x_position_gain_vel_high_deg_s"),
                    ("Kp X Scale at Low Vel", "x_position_gain_scale_low"),
                    ("Kp X Scale at High Vel", "x_position_gain_scale_high"),
                    ("Ki X", "velocity_integral_gain_x"),
                    ("Kd Pos X", "velocity_position_derivative_gain_x"),
                    ("Kd Pos Alpha X", "velocity_position_derivative_filter_alpha_x"),
                    ("Kd/Vel X", "velocity_error_gain_x"),
                    ("X Damp Low", "x_damping_gain_low"),
                    ("X Damp Mid", "x_damping_gain_mid"),
                    ("X Damp High", "x_damping_gain_high"),
                    ("X Damp Ang Low", "x_damping_angle_low"),
                    ("X Damp Ang High", "x_damping_angle_high"),
                    ("Pos Alpha X", "spi_position_filter_alpha_x"),
                    ("SPI Skip X", "spi_position_skip_bits_x"),
                    ("Vel Alpha X", "spi_velocity_filter_alpha_x"),
                    ("BB Enable X", "benedict_bordner_enable_x"),
                    ("BB Alpha X", "benedict_bordner_alpha_x"),
                    ("BB Beta Scale X", "benedict_bordner_beta_scale_x"),
                    ("Vel Max X", "velocity_command_max_x"),
                    ("Cmd Vel Alpha X", "command_velocity_filter_alpha_x"),
                    ("Correction Slew X", "velocity_correction_slew_rate_x"),
                    ("Corr Slew Zero X", "x_velocity_correction_slew_zero_rate"),
                    ("Corr Slew Zero Start X", "x_velocity_correction_slew_zero_start_deg"),
                    ("Corr Slew Zero Stop X", "x_velocity_correction_slew_zero_stop_deg"),
                    ("FF Scale X", "velocity_feedforward_scale_x"),
                    ("X FF Zero Start", "x_vel_ff_attenuate_start_deg"),
                    ("X FF Zero Stop", "x_vel_ff_attenuate_stop_deg"),
                    ("X FF Zero Scale", "x_vel_ff_attenuate_scale"),
                    ("X Toward Gain Low", "x_motor_vel_gain_toward_low"),
                    ("X Toward Gain Mid", "x_motor_vel_gain_toward_mid"),
                    ("X Toward Gain High", "x_motor_vel_gain_toward_high"),
                    ("X Toward Ang Low", "x_motor_vel_gain_toward_angle_low"),
                    ("X Toward Ang High", "x_motor_vel_gain_toward_angle_high"),
                    ("X Away Gain Low", "x_motor_vel_gain_away_low"),
                    ("X Away Gain Mid", "x_motor_vel_gain_away_mid"),
                    ("X Away Gain High", "x_motor_vel_gain_away_high"),
                    ("X Away Ang Low", "x_motor_vel_gain_away_angle_low"),
                    ("X Away Ang High", "x_motor_vel_gain_away_angle_high"),
                    ("X Gain Ramp (s)", "x_motor_vel_gain_ramp_in_sec"),
                ],
            ),
            (
                "Y Axis",
                [
                    ("Kp Pos Y", "velocity_position_gain_y"),
                    ("Kp Y Vel Low", "y_position_gain_vel_low_deg_s"),
                    ("Kp Y Vel High", "y_position_gain_vel_high_deg_s"),
                    ("Kp Y Scale at Low Vel", "y_position_gain_scale_low"),
                    ("Kp Y Scale at High Vel", "y_position_gain_scale_high"),
                    ("Ki Y", "velocity_integral_gain_y"),
                    ("Kd Pos Y", "velocity_position_derivative_gain_y"),
                    ("Kd Pos Alpha Y", "velocity_position_derivative_filter_alpha_y"),
                    ("Kd/Vel Y", "velocity_error_gain_y"),
                    ("Y Damp Low", "y_damping_gain_low"),
                    ("Y Damp Mid", "y_damping_gain_mid"),
                    ("Y Damp High", "y_damping_gain_high"),
                    ("Y Damp Ang Low", "y_damping_angle_low"),
                    ("Y Damp Ang High", "y_damping_angle_high"),
                    ("Pos Alpha Y", "spi_position_filter_alpha_y"),
                    ("SPI Skip Y", "spi_position_skip_bits_y"),
                    ("Vel Alpha Y", "spi_velocity_filter_alpha_y"),
                    ("BB Enable Y", "benedict_bordner_enable_y"),
                    ("BB Alpha Y", "benedict_bordner_alpha_y"),
                    ("BB Beta Scale Y", "benedict_bordner_beta_scale_y"),
                    ("Vel Max Y", "velocity_command_max_y"),
                    ("Cmd Vel Alpha Y", "command_velocity_filter_alpha_y"),
                    ("Correction Slew Y", "velocity_correction_slew_rate_y"),
                    ("Corr Slew Zero Y", "y_velocity_correction_slew_zero_rate"),
                    ("Corr Slew Zero Start Y", "y_velocity_correction_slew_zero_start_deg"),
                    ("Corr Slew Zero Stop Y", "y_velocity_correction_slew_zero_stop_deg"),
                    ("FF Scale Y", "velocity_feedforward_scale_y"),
                    ("Y FF Zero Start", "y_vel_ff_attenuate_start_deg"),
                    ("Y FF Zero Stop", "y_vel_ff_attenuate_stop_deg"),
                    ("Y FF Zero Scale", "y_vel_ff_attenuate_scale"),
                    ("Y Toward Gain Low", "y_motor_vel_gain_toward_low"),
                    ("Y Toward Gain Mid", "y_motor_vel_gain_toward_mid"),
                    ("Y Toward Gain High", "y_motor_vel_gain_toward_high"),
                    ("Y Toward Ang Low", "y_motor_vel_gain_toward_angle_low"),
                    ("Y Toward Ang High", "y_motor_vel_gain_toward_angle_high"),
                    ("Y Away Gain Low", "y_motor_vel_gain_away_low"),
                    ("Y Away Gain Mid", "y_motor_vel_gain_away_mid"),
                    ("Y Away Gain High", "y_motor_vel_gain_away_high"),
                    ("Y Away Ang Low", "y_motor_vel_gain_away_angle_low"),
                    ("Y Away Ang High", "y_motor_vel_gain_away_angle_high"),
                    ("Y Gain Ramp (s)", "y_motor_vel_gain_ramp_in_sec"),
                ],
            ),
        ]

    def open_velocity_settings_window(self):
        if self.velocity_settings_window is not None and self.velocity_settings_window.winfo_exists():
            self.velocity_settings_window.lift()
            self.velocity_settings_window.focus_force()
            return

        window = tk.Toplevel(self)
        window.title("Velocity Mode Settings")
        window.geometry("1180x720")
        self.velocity_settings_window = window
        self.velocity_setting_entries = {}

        def on_close():
            self.velocity_settings_window = None
            self.velocity_setting_entries = {}
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", on_close)

        top_row = ttk.Frame(window)
        top_row.pack(fill="x", padx=10, pady=(10, 4))
        ttk.Label(top_row, text="Ctrl Mode").pack(side="left", padx=(0, 6))
        ttk.Combobox(
            top_row,
            textvariable=self.vel_controller_mode_var,
            values=("full", "velocity_error_only", "feedforward_only"),
            state="readonly",
            width=18,
        ).pack(side="left")

        notebook = ttk.Notebook(window)
        notebook.pack(fill="both", expand=True, padx=10, pady=6)

        for group_name, fields in self.velocity_setting_groups():
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=group_name)
            rows_per_col = 16
            for index, (label_text, attr_name) in enumerate(fields):
                block = index // rows_per_col
                row = index % rows_per_col
                label_col = block * 2
                entry_col = label_col + 1
                ttk.Label(frame, text=label_text).grid(
                    row=row,
                    column=label_col,
                    sticky="w",
                    padx=(8 if block == 0 else 24, 6),
                    pady=3,
                )
                entry = ttk.Entry(frame, width=12)
                entry.grid(row=row, column=entry_col, sticky="w", padx=6, pady=3)
                entry.insert(0, f"{getattr(self, attr_name):g}")
                self.velocity_setting_entries[attr_name] = entry
            for column in range(((len(fields) + rows_per_col - 1) // rows_per_col) * 2):
                frame.columnconfigure(column, weight=1 if column % 2 else 0)

        button_row = ttk.Frame(window)
        button_row.pack(fill="x", padx=10, pady=(4, 10))
        ttk.Button(button_row, text="Apply", command=self.apply_velocity_settings).pack(side="left", padx=(0, 8))
        ttk.Button(button_row, text="Close", command=on_close).pack(side="left")

    def apply_velocity_settings(self):
        try:
            values = {}
            for _, fields in self.velocity_setting_groups():
                for _, attr_name in fields:
                    entry = self.velocity_setting_entries.get(attr_name)
                    if entry is None:
                        continue
                    if attr_name in ("spi_position_skip_bits_x", "spi_position_skip_bits_y"):
                        values[attr_name] = int(float(entry.get()))
                    else:
                        values[attr_name] = float(entry.get())

            velocity_controller_mode = self.vel_controller_mode_var.get().strip()
            if velocity_controller_mode not in ("full", "velocity_error_only", "feedforward_only"):
                raise ValueError
            if values["velocity_command_max_x"] <= 0 or values["velocity_command_max_y"] <= 0:
                raise ValueError
            if values["velocity_ramp_rate_deg_s2"] < 0 or values["velocity_debug_scale"] <= 0:
                raise ValueError
            for attr_name in (
                "spi_velocity_filter_alpha_x",
                "spi_velocity_filter_alpha_y",
                "command_velocity_filter_alpha_x",
                "command_velocity_filter_alpha_y",
                "spi_position_filter_alpha_x",
                "spi_position_filter_alpha_y",
                "velocity_position_derivative_filter_alpha_x",
                "velocity_position_derivative_filter_alpha_y",
            ):
                if not (0.0 <= values[attr_name] < 1.0):
                    raise ValueError
            for attr_name in ("benedict_bordner_enable_x", "benedict_bordner_enable_y"):
                if values[attr_name] not in (0.0, 1.0):
                    raise ValueError
            for attr_name in ("benedict_bordner_alpha_x", "benedict_bordner_alpha_y"):
                if not (0.0 < values[attr_name] < 1.0):
                    raise ValueError
            for attr_name in ("benedict_bordner_beta_scale_x", "benedict_bordner_beta_scale_y"):
                if values[attr_name] < 0.0:
                    raise ValueError
            for attr_name in ("spi_position_skip_bits_x", "spi_position_skip_bits_y"):
                if values[attr_name] < 0 or values[attr_name] >= TRACK_SPI_ENCODER_BITS:
                    raise ValueError
            for prefix in ("x", "y"):
                if values[f"{prefix}_position_gain_vel_low_deg_s"] >= values[f"{prefix}_position_gain_vel_high_deg_s"]:
                    raise ValueError
                if values[f"{prefix}_vel_ff_attenuate_start_deg"] >= values[f"{prefix}_vel_ff_attenuate_stop_deg"]:
                    raise ValueError
                if (
                    values[f"{prefix}_velocity_correction_slew_zero_start_deg"]
                    >= values[f"{prefix}_velocity_correction_slew_zero_stop_deg"]
                ):
                    raise ValueError
                if values[f"{prefix}_damping_angle_low"] >= values[f"{prefix}_damping_angle_high"]:
                    raise ValueError
                if values[f"{prefix}_motor_vel_gain_toward_angle_low"] >= values[f"{prefix}_motor_vel_gain_toward_angle_high"]:
                    raise ValueError
                if values[f"{prefix}_motor_vel_gain_away_angle_low"] >= values[f"{prefix}_motor_vel_gain_away_angle_high"]:
                    raise ValueError
                for attr_name in (
                    f"{prefix}_position_gain_vel_low_deg_s",
                    f"{prefix}_position_gain_vel_high_deg_s",
                    f"{prefix}_position_gain_scale_low",
                    f"{prefix}_position_gain_scale_high",
                    f"velocity_correction_slew_rate_{prefix}",
                    f"{prefix}_velocity_correction_slew_zero_rate",
                    f"{prefix}_velocity_correction_slew_zero_start_deg",
                    f"{prefix}_velocity_correction_slew_zero_stop_deg",
                    f"{prefix}_damping_gain_low",
                    f"{prefix}_damping_gain_mid",
                    f"{prefix}_damping_gain_high",
                    f"{prefix}_damping_angle_low",
                    f"{prefix}_damping_angle_high",
                    f"{prefix}_vel_ff_attenuate_start_deg",
                    f"{prefix}_vel_ff_attenuate_stop_deg",
                    f"{prefix}_vel_ff_attenuate_scale",
                    f"{prefix}_motor_vel_gain_toward_angle_low",
                    f"{prefix}_motor_vel_gain_toward_angle_high",
                    f"{prefix}_motor_vel_gain_away_angle_low",
                    f"{prefix}_motor_vel_gain_away_angle_high",
                    f"{prefix}_motor_vel_gain_ramp_in_sec",
                ):
                    if values[attr_name] < 0:
                        raise ValueError
                for attr_name in (
                    f"{prefix}_motor_vel_gain_toward_low",
                    f"{prefix}_motor_vel_gain_toward_mid",
                    f"{prefix}_motor_vel_gain_toward_high",
                    f"{prefix}_motor_vel_gain_away_low",
                    f"{prefix}_motor_vel_gain_away_mid",
                    f"{prefix}_motor_vel_gain_away_high",
                ):
                    if values[attr_name] <= 0:
                        raise ValueError

            for attr_name, value in values.items():
                setattr(self, attr_name, value)
            self.velocity_controller_mode = velocity_controller_mode
            return True
        except Exception:
            messagebox.showerror(
                "Error",
                (
                    "Velocity settings contain an invalid value.\n"
                    "Alpha values must be in [0, 1).\n"
                    "BB enable must be 0 or 1, BB alpha must be in (0, 1), and BB beta scale must be non-negative.\n"
                    "Skipped bits must be valid encoder bits.\n"
                    "Low thresholds must be smaller than high thresholds.\n"
                    "Motor gain values must be positive."
                ),
            )
            return False

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
            endat_x_enable = bool(self.endat_x_enable_var.get())
            endat_x_use_for_control = bool(self.endat_x_use_for_control_var.get())
            endat_x_port = self.endat_x_port_entry.get().strip()
            endat_x_zero_count = int(float(self.endat_x_zero_entry.get()))
            endat_x_sign = float(self.endat_x_sign_entry.get())

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
                or (endat_x_enable and not endat_x_port)
                or endat_x_zero_count < 0
                or endat_x_sign == 0
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
            self.endat_x_enable = endat_x_enable
            self.endat_x_use_for_control = endat_x_use_for_control
            self.endat_x_port = endat_x_port
            self.endat_x_zero_count = endat_x_zero_count
            self.endat_x_sign = endat_x_sign
            return True
        except Exception:
            messagebox.showerror(
                "Error",
                (
                    "Tracking settings must be positive numbers.\n"
                    "Lead can be zero or positive.\n"
                    "Point spacing cannot exceed horizon.\n"
                    "Rebuild margin must be smaller than horizon.\n"
                    "Min spacing cannot exceed point spacing.\n"
                    "EnDat port cannot be blank when enabled; zero count must be non-negative; sign cannot be zero."
                ),
            )
            return False

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
        if not self.apply_tracking_settings():
            return

        if self.control:
            try:
                safe_min_deg = getattr(self.control, "TRACKING_MIN_DEGREE", -91.0)
                safe_max_deg = getattr(self.control, "TRACKING_MAX_DEGREE", 91.0)
                for axis in ("x", "y"):
                    if hasattr(self.control, "is_safety_tripped") and self.control.is_safety_tripped(axis):
                        axis_pos = self.control.get_spi_position(axis)
                        if safe_min_deg <= axis_pos <= safe_max_deg:
                            self.control.clear_safety_trip(axis)
                        else:
                            reason = self.control.get_safety_trip_reason(axis) if hasattr(self.control, "get_safety_trip_reason") else None
                            messagebox.showerror(
                                "Safety Trip Active",
                                reason or (
                                    f"{axis.upper()} axis is outside safe SPI range "
                                    f"[{safe_min_deg:.1f}, {safe_max_deg:.1f}] deg."
                                ),
                            )
                            return
            except Exception as exc:
                messagebox.showerror("Safety Check Failed", str(exc))
                return

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
            try:
                self.control.move_absolute_pair(x_deg=0, y_deg=0)
            except Exception:
                pass

        tracking_thread = self.tracking_thread
        if tracking_thread and tracking_thread.is_alive() and tracking_thread is not threading.current_thread():
            tracking_thread.join(timeout=max(0.2, self.track_command_interval_sec * 2.0))
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
                "x_pos_error_derivative_deg_per_s",
                "y_pos_error_derivative_deg_per_s",
                "x_pos_d_term_deg_per_s",
                "y_pos_d_term_deg_per_s",
                "y_integral_unwind_applied",
                "y_integral_state",
                "cmd_x_correction_deg",
                "cmd_y_correction_deg",
                "cmd_x_sent_deg",
                "cmd_y_sent_deg",
                "meas_cmd_x_angle_deg",
                "meas_cmd_y_angle_deg",
                "meas_cmd_x_sent_deg",
                "meas_cmd_y_sent_deg",
                "raw_dx_deg",
                "raw_dy_deg",
                "cmd_dx_deg",
                "cmd_dy_deg",
                "x_vel_deg_per_s",
                "y_vel_deg_per_s",
                "x_acc_deg_per_s2",
                "y_acc_deg_per_s2",
                "cmd_x_vel_ff_raw_deg_per_s",
                "cmd_x_vel_ff_deg_per_s",
                "x_vel_ff_attenuate_start_deg",
                "x_vel_ff_attenuate_stop_deg",
                "x_vel_ff_attenuate_scale",
                "cmd_y_vel_ff_raw_deg_per_s",
                "cmd_y_vel_ff_deg_per_s",
                "y_vel_ff_attenuate_start_deg",
                "y_vel_ff_attenuate_stop_deg",
                "y_vel_ff_attenuate_scale",
                "x_vel_ref_now_deg_per_s",
                "y_vel_ref_now_deg_per_s",
                "velocity_controller_mode",
                "x_velocity_p_term_used_deg_per_s",
                "y_velocity_p_term_used_deg_per_s",
                "x_velocity_i_term_used_deg_per_s",
                "y_velocity_i_term_used_deg_per_s",
                "x_velocity_i_state_deg_s",
                "y_velocity_i_state_deg_s",
                "x_velocity_pos_d_term_used_deg_per_s",
                "y_velocity_pos_d_term_used_deg_per_s",
                "x_velocity_vel_d_term_used_deg_per_s",
                "y_velocity_vel_d_term_used_deg_per_s",
                "x_velocity_feedback_sum_raw_deg_per_s",
                "y_velocity_feedback_sum_raw_deg_per_s",
                "x_velocity_feedback_sum_slewed_deg_per_s",
                "y_velocity_feedback_sum_slewed_deg_per_s",
                "x_velocity_feedback_slew_delta_deg_per_s",
                "y_velocity_feedback_slew_delta_deg_per_s",
                "x_velocity_feedback_slew_rate_limit_deg_per_s2",
                "y_velocity_feedback_slew_rate_limit_deg_per_s2",
                "x_velocity_feedback_slew_far_rate_deg_per_s2",
                "y_velocity_feedback_slew_far_rate_deg_per_s2",
                "x_velocity_feedback_slew_zero_rate_deg_per_s2",
                "y_velocity_feedback_slew_zero_rate_deg_per_s2",
                "x_velocity_feedback_slew_zero_start_deg",
                "y_velocity_feedback_slew_zero_start_deg",
                "x_velocity_feedback_slew_zero_stop_deg",
                "y_velocity_feedback_slew_zero_stop_deg",
                "x_velocity_cmd_sum_before_debug_deg_per_s",
                "y_velocity_cmd_sum_before_debug_deg_per_s",
                "x_velocity_cmd_after_debug_before_filter_deg_per_s",
                "y_velocity_cmd_after_debug_before_filter_deg_per_s",
                "x_velocity_cmd_after_filter_before_limit_deg_per_s",
                "y_velocity_cmd_after_filter_before_limit_deg_per_s",
                "x_velocity_integral_gain",
                "y_velocity_integral_gain",
                "x_velocity_position_derivative_gain",
                "y_velocity_position_derivative_gain",
                "x_velocity_position_derivative_filter_alpha",
                "y_velocity_position_derivative_filter_alpha",
                "x_velocity_error_gain",
                "y_velocity_error_gain",
                "x_command_velocity_filter_alpha",
                "y_command_velocity_filter_alpha",
                "velocity_debug_scale",
                "cmd_x_vel_sent_deg_per_s",
                "cmd_y_vel_sent_deg_per_s",
                "x_actual_vel_raw_deg_per_s",
                "y_actual_vel_raw_deg_per_s",
                "x_actual_vel_deg_per_s",
                "y_actual_vel_deg_per_s",
                "x_spi_sample_deg",
                "y_spi_sample_deg",
                "x_measurement_source",
                "x_endat_raw_count",
                "x_endat_abs_deg",
                "x_endat_axis_deg",
                "x_endat_age_sec",
                "x_endat_error1",
                "x_endat_error2",
                "x_endat_timeout_step",
                "x_endat_read_ms",
                "x_benedict_bordner_enabled",
                "y_benedict_bordner_enabled",
                "x_benedict_bordner_alpha",
                "y_benedict_bordner_alpha",
                "x_benedict_bordner_beta",
                "y_benedict_bordner_beta",
                "x_benedict_bordner_beta_scale",
                "y_benedict_bordner_beta_scale",
                "x_benedict_bordner_predicted_deg",
                "y_benedict_bordner_predicted_deg",
                "x_benedict_bordner_residual_deg",
                "y_benedict_bordner_residual_deg",
                "x_position_gain_active",
                "x_damping_gain_active",
                "y_position_gain_active",
                "y_damping_gain_active",
                "x_vel_error_deg_per_s",
                "y_vel_error_deg_per_s",
                "x_gain_toward_vertical",
                "y_gain_toward_vertical",
                "x_motor_vel_gain_active",
                "y_motor_vel_gain_active",
                "velocity_mode_active",
                "x_control_mode",
                "x_input_mode",
                "y_control_mode",
                "y_input_mode",
                "x_load_encoder_id",
                "x_load_encoder_name",
                "x_commutation_encoder_id",
                "x_commutation_encoder_name",
                "x_use_commutation_vel",
                "x_use_load_encoder_for_commutation_vel",
                "x_pos_vel_mapper_scale",
                "y_load_encoder_id",
                "y_load_encoder_name",
                "y_commutation_encoder_id",
                "y_commutation_encoder_name",
                "y_use_commutation_vel",
                "y_use_load_encoder_for_commutation_vel",
                "y_pos_vel_mapper_scale",
                "x_controller_vel_limit",
                "y_controller_vel_limit",
                "x_traj_vel_limit",
                "x_traj_accel_limit",
                "x_traj_decel_limit",
                "y_traj_vel_limit",
                "y_traj_accel_limit",
                "y_traj_decel_limit",
                "x_actual_raw_deg",
                "y_actual_raw_deg",
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
                "x_pos_error_derivative_deg_per_s",
                "y_pos_error_derivative_deg_per_s",
                "x_pos_d_term_deg_per_s",
                "y_pos_d_term_deg_per_s",
                "y_integral_unwind_applied",
                "y_integral_state",
                "x_vel_deg_per_s",
                "y_vel_deg_per_s",
                "x_acc_deg_per_s2",
                "y_acc_deg_per_s2",
                "cmd_x_vel_ff_raw_deg_per_s",
                "cmd_x_vel_ff_deg_per_s",
                "x_vel_ff_attenuate_start_deg",
                "x_vel_ff_attenuate_stop_deg",
                "x_vel_ff_attenuate_scale",
                "cmd_y_vel_ff_raw_deg_per_s",
                "cmd_y_vel_ff_deg_per_s",
                "y_vel_ff_attenuate_start_deg",
                "y_vel_ff_attenuate_stop_deg",
                "y_vel_ff_attenuate_scale",
                "x_vel_ref_now_deg_per_s",
                "y_vel_ref_now_deg_per_s",
                "velocity_controller_mode",
                "x_velocity_p_term_used_deg_per_s",
                "y_velocity_p_term_used_deg_per_s",
                "x_velocity_i_term_used_deg_per_s",
                "y_velocity_i_term_used_deg_per_s",
                "x_velocity_i_state_deg_s",
                "y_velocity_i_state_deg_s",
                "x_velocity_pos_d_term_used_deg_per_s",
                "y_velocity_pos_d_term_used_deg_per_s",
                "x_velocity_vel_d_term_used_deg_per_s",
                "y_velocity_vel_d_term_used_deg_per_s",
                "x_velocity_feedback_sum_raw_deg_per_s",
                "y_velocity_feedback_sum_raw_deg_per_s",
                "x_velocity_feedback_sum_slewed_deg_per_s",
                "y_velocity_feedback_sum_slewed_deg_per_s",
                "x_velocity_feedback_slew_delta_deg_per_s",
                "y_velocity_feedback_slew_delta_deg_per_s",
                "x_velocity_feedback_slew_rate_limit_deg_per_s2",
                "y_velocity_feedback_slew_rate_limit_deg_per_s2",
                "x_velocity_feedback_slew_far_rate_deg_per_s2",
                "y_velocity_feedback_slew_far_rate_deg_per_s2",
                "x_velocity_feedback_slew_zero_rate_deg_per_s2",
                "y_velocity_feedback_slew_zero_rate_deg_per_s2",
                "x_velocity_feedback_slew_zero_start_deg",
                "y_velocity_feedback_slew_zero_start_deg",
                "x_velocity_feedback_slew_zero_stop_deg",
                "y_velocity_feedback_slew_zero_stop_deg",
                "x_velocity_cmd_sum_before_debug_deg_per_s",
                "y_velocity_cmd_sum_before_debug_deg_per_s",
                "x_velocity_cmd_after_debug_before_filter_deg_per_s",
                "y_velocity_cmd_after_debug_before_filter_deg_per_s",
                "x_velocity_cmd_after_filter_before_limit_deg_per_s",
                "y_velocity_cmd_after_filter_before_limit_deg_per_s",
                "x_velocity_integral_gain",
                "y_velocity_integral_gain",
                "x_velocity_position_derivative_gain",
                "y_velocity_position_derivative_gain",
                "x_velocity_position_derivative_filter_alpha",
                "y_velocity_position_derivative_filter_alpha",
                "x_velocity_error_gain",
                "y_velocity_error_gain",
                "x_command_velocity_filter_alpha",
                "y_command_velocity_filter_alpha",
                "velocity_debug_scale",
                "cmd_x_vel_sent_deg_per_s",
                "cmd_y_vel_sent_deg_per_s",
                "x_actual_vel_raw_deg_per_s",
                "y_actual_vel_raw_deg_per_s",
                "x_actual_vel_deg_per_s",
                "y_actual_vel_deg_per_s",
                "x_spi_sample_deg",
                "y_spi_sample_deg",
                "x_benedict_bordner_enabled",
                "y_benedict_bordner_enabled",
                "x_benedict_bordner_alpha",
                "y_benedict_bordner_alpha",
                "x_benedict_bordner_beta",
                "y_benedict_bordner_beta",
                "x_benedict_bordner_beta_scale",
                "y_benedict_bordner_beta_scale",
                "x_benedict_bordner_predicted_deg",
                "y_benedict_bordner_predicted_deg",
                "x_benedict_bordner_residual_deg",
                "y_benedict_bordner_residual_deg",
                "x_position_gain_active",
                "x_damping_gain_active",
                "y_position_gain_active",
                "y_damping_gain_active",
                "x_vel_error_deg_per_s",
                "y_vel_error_deg_per_s",
                "x_gain_toward_vertical",
                "y_gain_toward_vertical",
                "x_motor_vel_gain_active",
                "y_motor_vel_gain_active",
                "velocity_mode_active",
                "x_load_encoder_id",
                "x_load_encoder_name",
                "x_commutation_encoder_id",
                "x_commutation_encoder_name",
                "x_use_commutation_vel",
                "x_use_load_encoder_for_commutation_vel",
                "x_pos_vel_mapper_scale",
                "y_load_encoder_id",
                "y_load_encoder_name",
                "y_commutation_encoder_id",
                "y_commutation_encoder_name",
                "y_use_commutation_vel",
                "y_use_load_encoder_for_commutation_vel",
                "y_pos_vel_mapper_scale",
                "status",
            ]
        )
        was_visible = False
        tracking_started = False
        tracking_phase = "PREPOSITIONING"
        settled_cycles = 0
        last_tracking_command = None
        last_sent_raw_command = None
        last_sent_command = None
        last_display_target_elapsed = None
        last_raw_angles = None
        last_display_command = None
        last_loop_start_monotonic = None
        prev_meas_to_command_delay_sec = self.track_command_interval_sec
        next_rise_utc = None if replay_mode else get_next_rise_time(ts, observer, sat)
        prepointed = False
        tracking_start_perf = time.perf_counter()
        tracking_start_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
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
        x_velocity_position_error_derivative_filtered = 0.0
        y_velocity_position_error_derivative_filtered = 0.0
        prev_x_actual_for_vel = None
        prev_y_actual_for_vel = None
        prev_x_actual_elapsed_sec = None
        prev_y_actual_elapsed_sec = None
        x_actual_vel_raw = 0.0
        y_actual_vel_raw = 0.0
        x_actual_vel_filtered = 0.0
        y_actual_vel_filtered = 0.0
        x_position_filtered = None
        y_position_filtered = None
        x_bb_state = BenedictBordnerState()
        y_bb_state = BenedictBordnerState()
        x_bb_predicted = None
        y_bb_predicted = None
        x_bb_residual = 0.0
        y_bb_residual = 0.0
        x_bb_beta = benedict_bordner_beta(
            self.benedict_bordner_alpha_x,
            self.benedict_bordner_beta_scale_x,
        )
        y_bb_beta = benedict_bordner_beta(
            self.benedict_bordner_alpha_y,
            self.benedict_bordner_beta_scale_y,
        )
        x_command_vel_filtered = None
        y_command_vel_filtered = None
        x_velocity_feedback_slewed = None
        y_velocity_feedback_slewed = None
        x_motor_vel_gain_active = self.x_motor_vel_gain_toward_mid
        y_motor_vel_gain_active = self.y_motor_vel_gain_toward_mid
        x_position_gain_active = self.velocity_position_gain_x
        y_position_gain_active = self.velocity_position_gain_y
        x_damping_gain_active = self.x_damping_gain_mid
        y_damping_gain_active = self.y_damping_gain_mid
        x_gain_toward_vertical = 0
        y_gain_toward_vertical = 0
        last_applied_x_motor_vel_gain = None
        last_applied_y_motor_vel_gain = None

        def perf_to_utc(perf_value):
            return tracking_start_utc + datetime.timedelta(seconds=perf_value - tracking_start_perf)

        def stow_axes():
            nonlocal x_error_integral, y_error_integral, prev_y_ref_vel_for_unwind
            nonlocal x_velocity_error_integral, y_velocity_error_integral
            nonlocal prev_x_traj_error_for_d, prev_y_traj_error_for_d
            nonlocal x_error_derivative_filtered, y_error_derivative_filtered
            nonlocal x_velocity_position_error_derivative_filtered, y_velocity_position_error_derivative_filtered
            nonlocal prev_x_actual_for_vel, prev_y_actual_for_vel
            nonlocal prev_x_actual_elapsed_sec, prev_y_actual_elapsed_sec
            nonlocal x_actual_vel_raw, y_actual_vel_raw
            nonlocal x_actual_vel_filtered, y_actual_vel_filtered
            nonlocal x_position_filtered, y_position_filtered
            nonlocal x_bb_predicted, y_bb_predicted, x_bb_residual, y_bb_residual
            nonlocal x_command_vel_filtered, y_command_vel_filtered
            nonlocal x_velocity_feedback_slewed, y_velocity_feedback_slewed
            nonlocal last_applied_x_motor_vel_gain, last_applied_y_motor_vel_gain
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
            x_velocity_position_error_derivative_filtered = 0.0
            y_velocity_position_error_derivative_filtered = 0.0
            prev_x_actual_for_vel = None
            prev_y_actual_for_vel = None
            prev_x_actual_elapsed_sec = None
            prev_y_actual_elapsed_sec = None
            x_actual_vel_raw = 0.0
            y_actual_vel_raw = 0.0
            x_actual_vel_filtered = 0.0
            y_actual_vel_filtered = 0.0
            x_position_filtered = None
            y_position_filtered = None
            x_bb_state.reset()
            y_bb_state.reset()
            x_bb_predicted = None
            y_bb_predicted = None
            x_bb_residual = 0.0
            y_bb_residual = 0.0
            x_command_vel_filtered = None
            y_command_vel_filtered = None
            x_velocity_feedback_slewed = None
            y_velocity_feedback_slewed = None
            last_applied_x_motor_vel_gain = None
            last_applied_y_motor_vel_gain = None

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

        loop_exception = None
        loop_traceback = None
        endat_x_reader = None
        try:
            if self.endat_x_enable:
                if EndatSerialReader is None:
                    raise RuntimeError("EnDat reader is unavailable. Install pyserial and ensure endat_serial_reader.py is present.")
                endat_x_reader = EndatSerialReader(
                    self.endat_x_port,
                    baud=self.endat_x_baud,
                    counts_per_rev=self.endat_x_counts_per_rev,
                    zero_count=self.endat_x_zero_count,
                    timeout_s=0.0,
                )
                endat_x_reader.open()

            while self.running and sat_index == self.current_sat_index:
                loop_start_monotonic = time.perf_counter()
                observed_loop_period_sec = (
                    self.track_command_interval_sec
                    if last_loop_start_monotonic is None
                    else loop_start_monotonic - last_loop_start_monotonic
                )
                last_loop_start_monotonic = loop_start_monotonic
                now_monotonic = loop_start_monotonic
                now_utc = perf_to_utc(now_monotonic)
                continuous_elapsed_sec = now_monotonic - tracking_start_perf
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
                sgp4_live_start = time.perf_counter()
                live_sample = sample_reference(continuous_elapsed_sec, now_utc, derivative_dt)
                sgp4_live_eval_ms = (time.perf_counter() - sgp4_live_start) * 1000.0
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
                    sgp4_command_start = time.perf_counter()
                    command_sample = sample_reference(
                        continuous_elapsed_sec + target_lookahead_sec,
                        now_utc + datetime.timedelta(seconds=target_lookahead_sec),
                        derivative_dt,
                    )
                    sgp4_command_eval_ms = (time.perf_counter() - sgp4_command_start) * 1000.0

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
                cmd_x_vel_ff_raw = self.velocity_feedforward_scale_x * command_sample.get("x_vel", x_vel)
                cmd_x_vel_ff = cmd_x_vel_ff_raw
                cmd_y_vel_ff_raw = self.velocity_feedforward_scale_y * command_sample.get("y_vel", y_vel)
                cmd_y_vel_ff = cmd_y_vel_ff_raw

                x_angle, y_angle = clamp_tracking_angles(x_angle, y_angle)
                cmd_x_vel_ff = attenuate_x_velocity_feedforward(
                    x_angle,
                    cmd_x_vel_ff,
                    self.x_vel_ff_attenuate_start_deg,
                    self.x_vel_ff_attenuate_stop_deg,
                    self.x_vel_ff_attenuate_scale,
                )
                cmd_y_vel_ff = attenuate_x_velocity_feedforward(
                    y_angle,
                    cmd_y_vel_ff,
                    self.y_vel_ff_attenuate_start_deg,
                    self.y_vel_ff_attenuate_stop_deg,
                    self.y_vel_ff_attenuate_scale,
                )
                x_gain_toward_vertical = int(moving_toward_vertical(x_angle, cmd_x_vel_ff))
                if x_gain_toward_vertical:
                    x_gain_low = self.x_motor_vel_gain_toward_low
                    x_gain_mid = self.x_motor_vel_gain_toward_mid
                    x_gain_high = self.x_motor_vel_gain_toward_high
                    x_gain_angle_low = self.x_motor_vel_gain_toward_angle_low
                    x_gain_angle_high = self.x_motor_vel_gain_toward_angle_high
                else:
                    x_gain_low = self.x_motor_vel_gain_away_low
                    x_gain_mid = self.x_motor_vel_gain_away_mid
                    x_gain_high = self.x_motor_vel_gain_away_high
                    x_gain_angle_low = self.x_motor_vel_gain_away_angle_low
                    x_gain_angle_high = self.x_motor_vel_gain_away_angle_high
                x_motor_vel_gain_target = scheduled_x_motor_vel_gain(
                    x_angle,
                    x_gain_low,
                    x_gain_mid,
                    x_gain_high,
                    x_gain_angle_low,
                    x_gain_angle_high,
                )
                if self.tracking_gains:
                    x_motor_vel_gain_base = self.tracking_gains[1]
                    if last_applied_x_motor_vel_gain is None:
                        x_motor_vel_gain_active = x_motor_vel_gain_base
                    elif self.x_motor_vel_gain_ramp_in_sec > 0.0:
                        ramp_alpha = observed_loop_period_sec / self.x_motor_vel_gain_ramp_in_sec
                        x_motor_vel_gain_active = blend_value(
                            last_applied_x_motor_vel_gain,
                            x_motor_vel_gain_target,
                            ramp_alpha,
                        )
                    else:
                        x_motor_vel_gain_active = x_motor_vel_gain_target
                else:
                    x_motor_vel_gain_active = x_motor_vel_gain_target
                x_damping_gain_active = scheduled_x_motor_vel_gain(
                    x_angle,
                    self.x_damping_gain_low,
                    self.x_damping_gain_mid,
                    self.x_damping_gain_high,
                    self.x_damping_angle_low,
                    self.x_damping_angle_high,
                )
                y_gain_toward_vertical = int(moving_toward_vertical(y_angle, cmd_y_vel_ff))
                if y_gain_toward_vertical:
                    y_gain_low = self.y_motor_vel_gain_toward_low
                    y_gain_mid = self.y_motor_vel_gain_toward_mid
                    y_gain_high = self.y_motor_vel_gain_toward_high
                    y_gain_angle_low = self.y_motor_vel_gain_toward_angle_low
                    y_gain_angle_high = self.y_motor_vel_gain_toward_angle_high
                else:
                    y_gain_low = self.y_motor_vel_gain_away_low
                    y_gain_mid = self.y_motor_vel_gain_away_mid
                    y_gain_high = self.y_motor_vel_gain_away_high
                    y_gain_angle_low = self.y_motor_vel_gain_away_angle_low
                    y_gain_angle_high = self.y_motor_vel_gain_away_angle_high
                y_motor_vel_gain_target = scheduled_x_motor_vel_gain(
                    y_angle,
                    y_gain_low,
                    y_gain_mid,
                    y_gain_high,
                    y_gain_angle_low,
                    y_gain_angle_high,
                )
                if self.tracking_gains_y:
                    y_motor_vel_gain_base = self.tracking_gains_y[1]
                    if last_applied_y_motor_vel_gain is None:
                        y_motor_vel_gain_active = y_motor_vel_gain_base
                    elif self.y_motor_vel_gain_ramp_in_sec > 0.0:
                        ramp_alpha = observed_loop_period_sec / self.y_motor_vel_gain_ramp_in_sec
                        y_motor_vel_gain_active = blend_value(
                            last_applied_y_motor_vel_gain,
                            y_motor_vel_gain_target,
                            ramp_alpha,
                        )
                    else:
                        y_motor_vel_gain_active = y_motor_vel_gain_target
                else:
                    y_motor_vel_gain_active = y_motor_vel_gain_target
                y_damping_gain_active = scheduled_x_motor_vel_gain(
                    y_angle,
                    self.y_damping_gain_low,
                    self.y_damping_gain_mid,
                    self.y_damping_gain_high,
                    self.y_damping_angle_low,
                    self.y_damping_angle_high,
                )
                cmd_x_angle = x_angle
                cmd_y_angle = y_angle
                cmd_x_p_term = 0.0
                cmd_y_p_term = 0.0
                cmd_x_i_term = 0.0
                cmd_y_i_term = 0.0
                cmd_x_d_term = 0.0
                cmd_y_d_term = 0.0
                x_pos_error_derivative = 0.0
                y_pos_error_derivative = 0.0
                x_pos_d_term = 0.0
                y_pos_d_term = 0.0
                x_velocity_p_term_used = 0.0
                y_velocity_p_term_used = 0.0
                x_velocity_i_term_used = 0.0
                y_velocity_i_term_used = 0.0
                x_velocity_i_state = x_velocity_error_integral
                y_velocity_i_state = y_velocity_error_integral
                x_velocity_pos_d_term_used = 0.0
                y_velocity_pos_d_term_used = 0.0
                x_velocity_vel_d_term_used = 0.0
                y_velocity_vel_d_term_used = 0.0
                x_velocity_feedback_sum_raw = 0.0
                y_velocity_feedback_sum_raw = 0.0
                x_velocity_feedback_sum_slewed = 0.0
                y_velocity_feedback_sum_slewed = 0.0
                x_velocity_feedback_slew_delta = 0.0
                y_velocity_feedback_slew_delta = 0.0
                x_velocity_feedback_slew_rate_limit = 0.0
                y_velocity_feedback_slew_rate_limit = 0.0
                x_velocity_cmd_sum_before_debug = 0.0
                y_velocity_cmd_sum_before_debug = 0.0
                x_velocity_cmd_after_debug_before_filter = 0.0
                y_velocity_cmd_after_debug_before_filter = 0.0
                x_velocity_cmd_after_filter_before_limit = 0.0
                y_velocity_cmd_after_filter_before_limit = 0.0
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
                x_actual_raw = None
                y_actual_raw = None
                x_actual_sample = None
                y_actual_sample = None
                x_spi_sample_for_log = None
                x_measurement_source = "spi"
                x_endat_sample = None
                x_endat_raw_count = None
                x_endat_abs_deg = None
                x_endat_axis_deg = None
                x_endat_age_sec = None
                x_endat_error1 = None
                x_endat_error2 = None
                x_endat_timeout_step = None
                x_endat_read_ms = 0.0
                x_bb_active = False
                y_bb_active = False
                x_actual_vel = 0.0
                y_actual_vel = 0.0
                x_vel_ref_now = 0.0
                y_vel_ref_now = 0.0
                x_vel_error = 0.0
                y_vel_error = 0.0
                meas_cmd_x_angle = cmd_x_angle
                meas_cmd_y_angle = cmd_y_angle
                meas_cmd_x_sent = cmd_x_sent
                meas_cmd_y_sent = cmd_y_sent
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
                x_load_encoder_id = None
                x_load_encoder_name = None
                x_commutation_encoder_id = None
                x_commutation_encoder_name = None
                x_use_commutation_vel = None
                x_use_load_encoder_for_commutation_vel = None
                x_pos_vel_mapper_scale = None
                y_load_encoder_id = None
                y_load_encoder_name = None
                y_commutation_encoder_id = None
                y_commutation_encoder_name = None
                y_use_commutation_vel = None
                y_use_load_encoder_for_commutation_vel = None
                y_pos_vel_mapper_scale = None
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
                        control_snapshot_start = time.perf_counter()
                        x_axis0 = self.control.get_odrive("x").axis0
                        y_axis0 = self.control.get_odrive("y").axis0
                        x_control_mode = int(x_axis0.controller.config.control_mode)
                        x_input_mode = int(x_axis0.controller.config.input_mode)
                        y_control_mode = int(y_axis0.controller.config.control_mode)
                        y_input_mode = int(y_axis0.controller.config.input_mode)
                        x_load_encoder_id = int(x_axis0.config.load_encoder)
                        x_load_encoder_name = encoder_id_to_name(x_load_encoder_id)
                        x_commutation_encoder_id = int(x_axis0.config.commutation_encoder)
                        x_commutation_encoder_name = encoder_id_to_name(x_commutation_encoder_id)
                        x_use_commutation_vel = int(bool(x_axis0.controller.config.use_commutation_vel))
                        x_use_load_encoder_for_commutation_vel = int(
                            bool(x_axis0.controller.config.use_load_encoder_for_commutation_vel)
                        )
                        x_pos_vel_mapper_scale = float(x_axis0.pos_vel_mapper.config.scale)
                        y_load_encoder_id = int(y_axis0.config.load_encoder)
                        y_load_encoder_name = encoder_id_to_name(y_load_encoder_id)
                        y_commutation_encoder_id = int(y_axis0.config.commutation_encoder)
                        y_commutation_encoder_name = encoder_id_to_name(y_commutation_encoder_id)
                        y_use_commutation_vel = int(bool(y_axis0.controller.config.use_commutation_vel))
                        y_use_load_encoder_for_commutation_vel = int(
                            bool(y_axis0.controller.config.use_load_encoder_for_commutation_vel)
                        )
                        y_pos_vel_mapper_scale = float(y_axis0.pos_vel_mapper.config.scale)
                        x_controller_vel_limit = float(x_axis0.controller.config.vel_limit)
                        y_controller_vel_limit = float(y_axis0.controller.config.vel_limit)
                        x_traj_vel_limit = float(x_axis0.trap_traj.config.vel_limit)
                        x_traj_accel_limit = float(x_axis0.trap_traj.config.accel_limit)
                        x_traj_decel_limit = float(x_axis0.trap_traj.config.decel_limit)
                        y_traj_vel_limit = float(y_axis0.trap_traj.config.vel_limit)
                        y_traj_accel_limit = float(y_axis0.trap_traj.config.accel_limit)
                        y_traj_decel_limit = float(y_axis0.trap_traj.config.decel_limit)
                        control_snapshot_ms = (time.perf_counter() - control_snapshot_start) * 1000.0
                        spi_window_start = time.perf_counter()
                        spi_x_start = time.perf_counter()
                        x_actual_raw = self.control.get_spi_position("x")
                        x_actual_sample = quantize_spi_position_deg(x_actual_raw, self.spi_position_skip_bits_x)
                        spi_x_done = time.perf_counter()
                        x_spi_sample_for_log = x_actual_sample
                        meas_x_elapsed_sec = spi_x_done - tracking_start_perf
                        if endat_x_reader is not None:
                            endat_x_read_start = time.perf_counter()
                            x_endat_sample = endat_x_reader.read_latest_available()
                            endat_x_read_done = time.perf_counter()
                            x_endat_read_ms = (endat_x_read_done - endat_x_read_start) * 1000.0
                            if x_endat_sample is not None:
                                x_endat_raw_count = x_endat_sample.pos32
                                x_endat_abs_deg = x_endat_sample.angle_deg
                                x_endat_axis_deg = endat_angle_to_axis_deg(
                                    x_endat_abs_deg,
                                    self.endat_x_sign,
                                )
                                x_endat_age_sec = max(0.0, time.monotonic() - x_endat_sample.timestamp_s)
                                x_endat_error1 = x_endat_sample.error1
                                x_endat_error2 = x_endat_sample.error2
                                x_endat_timeout_step = x_endat_sample.timeout_step
                                if self.endat_x_use_for_control:
                                    x_actual_raw = x_endat_axis_deg
                                    x_actual_sample = x_endat_axis_deg
                                    x_measurement_source = "endat"
                                    meas_x_elapsed_sec = endat_x_read_done - tracking_start_perf
                        x_bb_active = self.benedict_bordner_enable_x >= 0.5
                        if x_bb_active:
                            x_bb_beta = benedict_bordner_beta(
                                self.benedict_bordner_alpha_x,
                                self.benedict_bordner_beta_scale_x,
                            )
                            x_bb_dt_sec = 0.0
                            if prev_x_actual_elapsed_sec is not None:
                                x_bb_dt_sec = meas_x_elapsed_sec - prev_x_actual_elapsed_sec
                            x_actual, x_actual_vel_filtered, x_bb_predicted, x_bb_residual = x_bb_state.update(
                                x_actual_sample,
                                x_bb_dt_sec,
                                self.benedict_bordner_alpha_x,
                                x_bb_beta,
                            )
                            x_actual_vel_raw = x_actual_vel_filtered
                            x_position_filtered = x_actual
                        else:
                            x_bb_predicted = None
                            x_bb_residual = 0.0
                            x_bb_state.reset()
                            if x_position_filtered is None or self.spi_position_filter_alpha_x <= 0.0:
                                x_position_filtered = x_actual_sample
                            else:
                                x_position_filtered = (
                                    self.spi_position_filter_alpha_x * x_position_filtered
                                    + (1.0 - self.spi_position_filter_alpha_x) * x_actual_sample
                                )
                            x_actual = x_position_filtered
                        spi_x_read_ms = (spi_x_done - spi_x_start) * 1000.0
                        spi_y_start = time.perf_counter()
                        y_actual_raw = self.control.get_spi_position("y")
                        y_actual_sample = quantize_spi_position_deg(y_actual_raw, self.spi_position_skip_bits_y)
                        spi_y_done = time.perf_counter()
                        meas_y_elapsed_sec = spi_y_done - tracking_start_perf
                        y_bb_active = self.benedict_bordner_enable_y >= 0.5
                        if y_bb_active:
                            y_bb_beta = benedict_bordner_beta(
                                self.benedict_bordner_alpha_y,
                                self.benedict_bordner_beta_scale_y,
                            )
                            y_bb_dt_sec = 0.0
                            if prev_y_actual_elapsed_sec is not None:
                                y_bb_dt_sec = meas_y_elapsed_sec - prev_y_actual_elapsed_sec
                            y_actual, y_actual_vel_filtered, y_bb_predicted, y_bb_residual = y_bb_state.update(
                                y_actual_sample,
                                y_bb_dt_sec,
                                self.benedict_bordner_alpha_y,
                                y_bb_beta,
                            )
                            y_actual_vel_raw = y_actual_vel_filtered
                            y_position_filtered = y_actual
                        else:
                            y_bb_predicted = None
                            y_bb_residual = 0.0
                            y_bb_state.reset()
                            if y_position_filtered is None or self.spi_position_filter_alpha_y <= 0.0:
                                y_position_filtered = y_actual_sample
                            else:
                                y_position_filtered = (
                                    self.spi_position_filter_alpha_y * y_position_filtered
                                    + (1.0 - self.spi_position_filter_alpha_y) * y_actual_sample
                                )
                            y_actual = y_position_filtered
                        spi_y_read_ms = (spi_y_done - spi_y_start) * 1000.0
                        spi_total_ms = (spi_y_done - spi_window_start) * 1000.0
                        meas_mid_elapsed_sec = spi_y_done - tracking_start_perf
                        if last_sent_raw_command is not None:
                            meas_cmd_x_angle, meas_cmd_y_angle = last_sent_raw_command
                        if last_sent_command is not None:
                            meas_cmd_x_sent, meas_cmd_y_sent = last_sent_command
                        x_error = meas_cmd_x_sent - x_actual
                        y_error = meas_cmd_y_sent - y_actual
                        sgp4_ideal_start = time.perf_counter()
                        x_ideal_sample = sample_reference(
                            meas_x_elapsed_sec,
                            perf_to_utc(spi_x_done),
                            derivative_dt,
                        )
                        y_ideal_sample = sample_reference(
                            meas_y_elapsed_sec,
                            perf_to_utc(spi_y_done),
                            derivative_dt,
                        )
                        sgp4_ideal_eval_ms = (time.perf_counter() - sgp4_ideal_start) * 1000.0
                        if x_ideal_sample is not None:
                            ideal_x_angle, _ = clamp_tracking_angles(
                                x_ideal_sample["x_angle"],
                                x_ideal_sample["y_angle"],
                            )
                            x_vel_ref_now = self.velocity_feedforward_scale_x * x_ideal_sample.get("x_vel", 0.0)
                            x_vel_ref_now = attenuate_x_velocity_feedforward(
                                ideal_x_angle,
                                x_vel_ref_now,
                                self.x_vel_ff_attenuate_start_deg,
                                self.x_vel_ff_attenuate_stop_deg,
                                self.x_vel_ff_attenuate_scale,
                            )
                            x_traj_error = ideal_x_angle - x_actual
                        if y_ideal_sample is not None:
                            _, ideal_y_angle = clamp_tracking_angles(
                                y_ideal_sample["x_angle"],
                                y_ideal_sample["y_angle"],
                            )
                            y_vel_ref_now = self.velocity_feedforward_scale_y * y_ideal_sample.get("y_vel", 0.0)
                            y_vel_ref_now = attenuate_x_velocity_feedforward(
                                ideal_y_angle,
                                y_vel_ref_now,
                                self.y_vel_ff_attenuate_start_deg,
                                self.y_vel_ff_attenuate_stop_deg,
                                self.y_vel_ff_attenuate_scale,
                            )
                            y_traj_error = ideal_y_angle - y_actual
                        x_position_gain_active = self.velocity_position_gain_x * scheduled_velocity_scale(
                            abs(x_vel_ref_now),
                            self.x_position_gain_scale_low,
                            self.x_position_gain_scale_high,
                            self.x_position_gain_vel_low_deg_s,
                            self.x_position_gain_vel_high_deg_s,
                        )
                        y_position_gain_active = self.velocity_position_gain_y * scheduled_velocity_scale(
                            abs(y_vel_ref_now),
                            self.y_position_gain_scale_low,
                            self.y_position_gain_scale_high,
                            self.y_position_gain_vel_low_deg_s,
                            self.y_position_gain_vel_high_deg_s,
                        )
                        if x_bb_active:
                            x_actual_vel = x_actual_vel_filtered
                        elif (
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
                            x_actual_vel = x_actual_vel_filtered
                        if y_bb_active:
                            y_actual_vel = y_actual_vel_filtered
                        elif (
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
                            y_actual_vel = y_actual_vel_filtered
                        if not x_bb_active:
                            x_actual_vel = x_actual_vel_filtered
                        if not y_bb_active:
                            y_actual_vel = y_actual_vel_filtered
                        prev_x_actual_for_vel = x_actual
                        prev_y_actual_for_vel = y_actual
                        prev_x_actual_elapsed_sec = meas_x_elapsed_sec
                        prev_y_actual_elapsed_sec = meas_y_elapsed_sec
                        capture_error = max(abs(x_error), abs(y_error))
                    except Exception:
                        x_actual = None
                        y_actual = None
                        x_actual_raw = None
                        y_actual_raw = None
                        x_actual_sample = None
                        y_actual_sample = None
                        x_spi_sample_for_log = None
                        x_measurement_source = "error"
                        x_endat_sample = None
                        x_bb_active = False
                        y_bb_active = False

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
                    x_command_vel_filtered = None
                    y_command_vel_filtered = None
                    try:
                        if use_velocity_tracking_mode:
                            self.control.enter_velocity_mode_all(
                                input_filter_bandwidth=self.track_filter_bandwidth_hz,
                                ramp_rate_deg_s2=self.velocity_ramp_rate_deg_s2,
                            )
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
                        cmd_x_vel_ff_raw = self.velocity_feedforward_scale_x * command_sample.get("x_vel", cmd_x_vel_ff)
                        cmd_x_vel_ff = cmd_x_vel_ff_raw
                        cmd_y_vel_ff_raw = self.velocity_feedforward_scale_y * command_sample.get("y_vel", cmd_y_vel_ff)
                        cmd_y_vel_ff = cmd_y_vel_ff_raw
                        cmd_x_vel_ff = attenuate_x_velocity_feedforward(
                            cmd_x_angle,
                            cmd_x_vel_ff,
                            self.x_vel_ff_attenuate_start_deg,
                            self.x_vel_ff_attenuate_stop_deg,
                            self.x_vel_ff_attenuate_scale,
                        )
                        cmd_y_vel_ff = attenuate_x_velocity_feedforward(
                            cmd_y_angle,
                            cmd_y_vel_ff,
                            self.y_vel_ff_attenuate_start_deg,
                            self.y_vel_ff_attenuate_stop_deg,
                            self.y_vel_ff_attenuate_scale,
                        )
                        if x_actual is not None and y_actual is not None:
                            x_error = cmd_x_angle - x_actual
                            y_error = cmd_y_angle - y_actual
                            capture_error = max(abs(x_error), abs(y_error))
                        if not sample["visible"]:
                            prepoint_status = "Below-horizon pickup"

                if should_track_now and x_actual is not None and y_actual is not None:
                    if use_velocity_tracking_mode:
                        if prev_x_traj_error_for_d is None or observed_loop_period_sec <= 1e-6:
                            x_velocity_position_error_derivative_filtered = 0.0
                        else:
                            x_pos_error_derivative_raw = (
                                x_traj_error - prev_x_traj_error_for_d
                            ) / observed_loop_period_sec
                            x_velocity_position_error_derivative_filtered = (
                                self.velocity_position_derivative_filter_alpha_x
                                * x_velocity_position_error_derivative_filtered
                                + (1.0 - self.velocity_position_derivative_filter_alpha_x)
                                * x_pos_error_derivative_raw
                            )
                        if prev_y_traj_error_for_d is None or observed_loop_period_sec <= 1e-6:
                            y_velocity_position_error_derivative_filtered = 0.0
                        else:
                            y_pos_error_derivative_raw = (
                                y_traj_error - prev_y_traj_error_for_d
                            ) / observed_loop_period_sec
                            y_velocity_position_error_derivative_filtered = (
                                self.velocity_position_derivative_filter_alpha_y
                                * y_velocity_position_error_derivative_filtered
                                + (1.0 - self.velocity_position_derivative_filter_alpha_y)
                                * y_pos_error_derivative_raw
                            )
                        x_pos_error_derivative = x_velocity_position_error_derivative_filtered
                        y_pos_error_derivative = y_velocity_position_error_derivative_filtered
                        x_pos_d_term = self.velocity_position_derivative_gain_x * x_pos_error_derivative
                        y_pos_d_term = self.velocity_position_derivative_gain_y * y_pos_error_derivative
                        prev_x_traj_error_for_d = x_traj_error
                        prev_y_traj_error_for_d = y_traj_error
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
                        x_velocity_i_state = x_velocity_error_integral
                        y_velocity_i_state = y_velocity_error_integral
                        x_vel_error = x_vel_ref_now - x_actual_vel
                        y_vel_error = y_vel_ref_now - y_actual_vel
                        x_vel_error_term = self.velocity_error_gain_x * x_damping_gain_active * x_vel_error
                        y_vel_error_term = self.velocity_error_gain_y * y_damping_gain_active * y_vel_error
                        x_velocity_p_term_computed = x_position_gain_active * x_traj_error
                        y_velocity_p_term_computed = y_position_gain_active * y_traj_error
                        x_velocity_i_term_computed = self.velocity_integral_gain_x * x_velocity_error_integral
                        y_velocity_i_term_computed = self.velocity_integral_gain_y * y_velocity_error_integral
                        x_velocity_feedback_slew_rate_limit = scheduled_zero_slew_rate(
                            cmd_x_angle,
                            self.velocity_correction_slew_rate_x,
                            self.x_velocity_correction_slew_zero_rate,
                            self.x_velocity_correction_slew_zero_start_deg,
                            self.x_velocity_correction_slew_zero_stop_deg,
                        )
                        y_velocity_feedback_slew_rate_limit = scheduled_zero_slew_rate(
                            cmd_y_angle,
                            self.velocity_correction_slew_rate_y,
                            self.y_velocity_correction_slew_zero_rate,
                            self.y_velocity_correction_slew_zero_start_deg,
                            self.y_velocity_correction_slew_zero_stop_deg,
                        )
                        if self.velocity_controller_mode == "feedforward_only":
                            x_velocity_feedback_slewed = None
                            y_velocity_feedback_slewed = None
                            cmd_x_vel_sent = cmd_x_vel_ff
                            cmd_y_vel_sent = cmd_y_vel_ff
                        elif self.velocity_controller_mode == "velocity_error_only":
                            x_velocity_vel_d_term_used = x_vel_error_term
                            y_velocity_vel_d_term_used = y_vel_error_term
                            x_velocity_feedback_sum_raw = x_velocity_vel_d_term_used
                            y_velocity_feedback_sum_raw = y_velocity_vel_d_term_used
                            x_velocity_feedback_sum_slewed = x_velocity_feedback_sum_raw
                            y_velocity_feedback_sum_slewed = y_velocity_feedback_sum_raw
                            if x_velocity_feedback_slew_rate_limit > 0.0 and observed_loop_period_sec > 0.0:
                                if x_velocity_feedback_slewed is None:
                                    x_velocity_feedback_slewed = x_velocity_feedback_sum_raw
                                else:
                                    max_delta = x_velocity_feedback_slew_rate_limit * observed_loop_period_sec
                                    raw_delta = x_velocity_feedback_sum_raw - x_velocity_feedback_slewed
                                    x_velocity_feedback_slew_delta = max(-max_delta, min(max_delta, raw_delta))
                                    x_velocity_feedback_slewed += x_velocity_feedback_slew_delta
                                x_velocity_feedback_sum_slewed = x_velocity_feedback_slewed
                            else:
                                x_velocity_feedback_slewed = x_velocity_feedback_sum_raw
                            if y_velocity_feedback_slew_rate_limit > 0.0 and observed_loop_period_sec > 0.0:
                                if y_velocity_feedback_slewed is None:
                                    y_velocity_feedback_slewed = y_velocity_feedback_sum_raw
                                else:
                                    max_delta = y_velocity_feedback_slew_rate_limit * observed_loop_period_sec
                                    raw_delta = y_velocity_feedback_sum_raw - y_velocity_feedback_slewed
                                    y_velocity_feedback_slew_delta = max(-max_delta, min(max_delta, raw_delta))
                                    y_velocity_feedback_slewed += y_velocity_feedback_slew_delta
                                y_velocity_feedback_sum_slewed = y_velocity_feedback_slewed
                            else:
                                y_velocity_feedback_slewed = y_velocity_feedback_sum_raw
                            cmd_x_vel_sent = cmd_x_vel_ff + x_velocity_feedback_sum_slewed
                            cmd_y_vel_sent = cmd_y_vel_ff + y_velocity_feedback_sum_slewed
                        else:
                            x_velocity_p_term_used = x_velocity_p_term_computed
                            y_velocity_p_term_used = y_velocity_p_term_computed
                            x_velocity_i_term_used = x_velocity_i_term_computed
                            y_velocity_i_term_used = y_velocity_i_term_computed
                            x_velocity_pos_d_term_used = x_pos_d_term
                            y_velocity_pos_d_term_used = y_pos_d_term
                            x_velocity_vel_d_term_used = x_vel_error_term
                            y_velocity_vel_d_term_used = y_vel_error_term
                            x_velocity_feedback_sum_raw = (
                                x_velocity_p_term_used
                                + x_velocity_i_term_used
                                + x_velocity_pos_d_term_used
                                + x_velocity_vel_d_term_used
                            )
                            y_velocity_feedback_sum_raw = (
                                y_velocity_p_term_used
                                + y_velocity_i_term_used
                                + y_velocity_pos_d_term_used
                                + y_velocity_vel_d_term_used
                            )
                            x_velocity_feedback_sum_slewed = x_velocity_feedback_sum_raw
                            y_velocity_feedback_sum_slewed = y_velocity_feedback_sum_raw
                            if x_velocity_feedback_slew_rate_limit > 0.0 and observed_loop_period_sec > 0.0:
                                if x_velocity_feedback_slewed is None:
                                    x_velocity_feedback_slewed = x_velocity_feedback_sum_raw
                                else:
                                    max_delta = x_velocity_feedback_slew_rate_limit * observed_loop_period_sec
                                    raw_delta = x_velocity_feedback_sum_raw - x_velocity_feedback_slewed
                                    x_velocity_feedback_slew_delta = max(-max_delta, min(max_delta, raw_delta))
                                    x_velocity_feedback_slewed += x_velocity_feedback_slew_delta
                                x_velocity_feedback_sum_slewed = x_velocity_feedback_slewed
                            else:
                                x_velocity_feedback_slewed = x_velocity_feedback_sum_raw
                            if y_velocity_feedback_slew_rate_limit > 0.0 and observed_loop_period_sec > 0.0:
                                if y_velocity_feedback_slewed is None:
                                    y_velocity_feedback_slewed = y_velocity_feedback_sum_raw
                                else:
                                    max_delta = y_velocity_feedback_slew_rate_limit * observed_loop_period_sec
                                    raw_delta = y_velocity_feedback_sum_raw - y_velocity_feedback_slewed
                                    y_velocity_feedback_slew_delta = max(-max_delta, min(max_delta, raw_delta))
                                    y_velocity_feedback_slewed += y_velocity_feedback_slew_delta
                                y_velocity_feedback_sum_slewed = y_velocity_feedback_slewed
                            else:
                                y_velocity_feedback_slewed = y_velocity_feedback_sum_raw
                            cmd_x_vel_sent = (
                                cmd_x_vel_ff
                                + x_velocity_feedback_sum_slewed
                            )
                            cmd_y_vel_sent = (
                                cmd_y_vel_ff
                                + y_velocity_feedback_sum_slewed
                            )
                        x_velocity_cmd_sum_before_debug = cmd_x_vel_sent
                        y_velocity_cmd_sum_before_debug = cmd_y_vel_sent
                        cmd_x_vel_sent *= self.velocity_debug_scale
                        cmd_y_vel_sent *= self.velocity_debug_scale
                        x_velocity_cmd_after_debug_before_filter = cmd_x_vel_sent
                        y_velocity_cmd_after_debug_before_filter = cmd_y_vel_sent
                        if x_command_vel_filtered is None or y_command_vel_filtered is None:
                            x_command_vel_filtered = cmd_x_vel_sent
                            y_command_vel_filtered = cmd_y_vel_sent
                        else:
                            x_command_vel_filtered = (
                                self.command_velocity_filter_alpha_x * x_command_vel_filtered
                                + (1.0 - self.command_velocity_filter_alpha_x) * cmd_x_vel_sent
                            )
                            y_command_vel_filtered = (
                                self.command_velocity_filter_alpha_y * y_command_vel_filtered
                                + (1.0 - self.command_velocity_filter_alpha_y) * cmd_y_vel_sent
                            )
                        cmd_x_vel_sent = x_command_vel_filtered
                        cmd_y_vel_sent = y_command_vel_filtered
                        x_velocity_cmd_after_filter_before_limit = cmd_x_vel_sent
                        y_velocity_cmd_after_filter_before_limit = cmd_y_vel_sent
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
                        x_velocity_position_error_derivative_filtered = 0.0
                        y_velocity_position_error_derivative_filtered = 0.0
                        prev_x_actual_for_vel = None
                        prev_y_actual_for_vel = None
                        prev_x_actual_elapsed_sec = None
                        prev_y_actual_elapsed_sec = None
                        x_actual_vel_filtered = 0.0
                        y_actual_vel_filtered = 0.0
                        x_command_vel_filtered = None
                        y_command_vel_filtered = None
                        x_velocity_feedback_slewed = None
                        y_velocity_feedback_slewed = None

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
                            f"X Source: {x_measurement_source}",
                            (
                                f"X EnDat: {x_endat_axis_deg:.3f} ({x_endat_age_sec:.3f}s)"
                                if x_endat_axis_deg is not None and x_endat_age_sec is not None
                                else "X EnDat: -"
                            ),
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
                                    self.control.set_gains(
                                        self.tracking_gains[0],
                                        x_motor_vel_gain_active,
                                        self.tracking_gains[2],
                                        axis="x",
                                    )
                                    if self.tracking_gains_y:
                                        self.control.set_gains(
                                            self.tracking_gains_y[0],
                                            y_motor_vel_gain_active,
                                            self.tracking_gains_y[2],
                                            axis="y",
                                        )
                                    else:
                                        self.control.set_gains(
                                            self.tracking_gains[0],
                                            y_motor_vel_gain_active,
                                            self.tracking_gains[2],
                                            axis="y",
                                        )
                                    last_applied_x_motor_vel_gain = x_motor_vel_gain_active
                                    last_applied_y_motor_vel_gain = y_motor_vel_gain_active
                                except Exception:
                                    pass
                            elif (
                                use_velocity_tracking_mode
                                and self.tracking_gains
                                and (
                                    last_applied_x_motor_vel_gain is None
                                    or abs(x_motor_vel_gain_active - last_applied_x_motor_vel_gain) > 1e-9
                                )
                            ):
                                try:
                                    self.control.set_gains(
                                        self.tracking_gains[0],
                                        x_motor_vel_gain_active,
                                        self.tracking_gains[2],
                                        axis="x",
                                    )
                                    last_applied_x_motor_vel_gain = x_motor_vel_gain_active
                                except Exception:
                                    pass
                            if (
                                should_track_now
                                and not enter_tracking_now
                                and use_velocity_tracking_mode
                                and self.tracking_gains_y
                                and (
                                    last_applied_y_motor_vel_gain is None
                                    or abs(y_motor_vel_gain_active - last_applied_y_motor_vel_gain) > 1e-9
                                )
                            ):
                                try:
                                    self.control.set_gains(
                                        self.tracking_gains_y[0],
                                        y_motor_vel_gain_active,
                                        self.tracking_gains_y[2],
                                        axis="y",
                                    )
                                    last_applied_y_motor_vel_gain = y_motor_vel_gain_active
                                except Exception:
                                    pass
                            command_send_start = time.perf_counter()
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
                            command_send_done = time.perf_counter()
                            command_send_ms = (command_send_done - command_send_start) * 1000.0
                            command_send_elapsed_sec = command_send_done - tracking_start_perf
                            last_tracking_command = (
                                (cmd_x_vel_sent, cmd_y_vel_sent)
                                if use_velocity_tracking_mode
                                else (cmd_x_sent, cmd_y_sent)
                            )
                            last_sent_raw_command = (cmd_x_angle, cmd_y_angle)
                            last_sent_command = (
                                (cmd_x_angle, cmd_y_angle)
                                if use_velocity_tracking_mode
                                else (cmd_x_sent, cmd_y_sent)
                            )
                            if enter_tracking_now:
                                tracking_phase = "TRACKING"
                                tracking_started = True
                            self.update_error_plot(continuous_elapsed_sec, x_traj_error, y_traj_error)
                        else:
                            command_send_start = time.perf_counter()
                            update_odrive_axes(x_angle, y_angle, self.control)
                            command_send_done = time.perf_counter()
                            command_send_ms = (command_send_done - command_send_start) * 1000.0
                            command_send_elapsed_sec = command_send_done - tracking_start_perf
                            last_tracking_command = None
                            last_sent_raw_command = (x_angle, y_angle)
                            last_sent_command = (x_angle, y_angle)
                    if not tracking_started and sample["visible"]:
                        prepoint_status = "Catching up to track"
                else:
                    if not self.running or sat_index != self.current_sat_index:
                        break

                    if prepointed and self.control:
                        update_odrive_axes(x_angle, y_angle, self.control)
                    last_tracking_command = None
                    last_sent_raw_command = None
                    last_sent_command = None
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
                    last_sent_raw_command = None
                    last_sent_command = None
                    last_display_command = None
                    last_display_target_elapsed = None

                loop_total_ms = (time.perf_counter() - loop_start_monotonic) * 1000.0
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
                        f"{x_pos_error_derivative:.6f}",
                        f"{y_pos_error_derivative:.6f}",
                        f"{x_pos_d_term:.6f}",
                        f"{y_pos_d_term:.6f}",
                        str(y_integral_unwind_applied),
                        f"{y_error_integral:.6f}",
                        f"{cmd_x_correction:.6f}",
                        f"{cmd_y_correction:.6f}",
                        f"{cmd_x_sent:.6f}",
                        f"{cmd_y_sent:.6f}",
                        f"{meas_cmd_x_angle:.6f}",
                        f"{meas_cmd_y_angle:.6f}",
                        f"{meas_cmd_x_sent:.6f}",
                        f"{meas_cmd_y_sent:.6f}",
                        f"{raw_dx:.6f}",
                        f"{raw_dy:.6f}",
                        f"{cmd_dx:.6f}",
                        f"{cmd_dy:.6f}",
                        f"{x_vel:.6f}",
                        f"{y_vel:.6f}",
                        f"{x_acc:.6f}",
                        f"{y_acc:.6f}",
                        f"{cmd_x_vel_ff_raw:.6f}",
                        f"{cmd_x_vel_ff:.6f}",
                        f"{self.x_vel_ff_attenuate_start_deg:.6f}",
                        f"{self.x_vel_ff_attenuate_stop_deg:.6f}",
                        f"{self.x_vel_ff_attenuate_scale:.6f}",
                        f"{cmd_y_vel_ff_raw:.6f}",
                        f"{cmd_y_vel_ff:.6f}",
                        f"{self.y_vel_ff_attenuate_start_deg:.6f}",
                        f"{self.y_vel_ff_attenuate_stop_deg:.6f}",
                        f"{self.y_vel_ff_attenuate_scale:.6f}",
                        f"{x_vel_ref_now:.6f}",
                        f"{y_vel_ref_now:.6f}",
                        self.velocity_controller_mode,
                        f"{x_velocity_p_term_used:.6f}",
                        f"{y_velocity_p_term_used:.6f}",
                        f"{x_velocity_i_term_used:.6f}",
                        f"{y_velocity_i_term_used:.6f}",
                        f"{x_velocity_i_state:.6f}",
                        f"{y_velocity_i_state:.6f}",
                        f"{x_velocity_pos_d_term_used:.6f}",
                        f"{y_velocity_pos_d_term_used:.6f}",
                        f"{x_velocity_vel_d_term_used:.6f}",
                        f"{y_velocity_vel_d_term_used:.6f}",
                        f"{x_velocity_feedback_sum_raw:.6f}",
                        f"{y_velocity_feedback_sum_raw:.6f}",
                        f"{x_velocity_feedback_sum_slewed:.6f}",
                        f"{y_velocity_feedback_sum_slewed:.6f}",
                        f"{x_velocity_feedback_slew_delta:.6f}",
                        f"{y_velocity_feedback_slew_delta:.6f}",
                        f"{x_velocity_feedback_slew_rate_limit:.6f}",
                        f"{y_velocity_feedback_slew_rate_limit:.6f}",
                        f"{self.velocity_correction_slew_rate_x:.6f}",
                        f"{self.velocity_correction_slew_rate_y:.6f}",
                        f"{self.x_velocity_correction_slew_zero_rate:.6f}",
                        f"{self.y_velocity_correction_slew_zero_rate:.6f}",
                        f"{self.x_velocity_correction_slew_zero_start_deg:.6f}",
                        f"{self.y_velocity_correction_slew_zero_start_deg:.6f}",
                        f"{self.x_velocity_correction_slew_zero_stop_deg:.6f}",
                        f"{self.y_velocity_correction_slew_zero_stop_deg:.6f}",
                        f"{x_velocity_cmd_sum_before_debug:.6f}",
                        f"{y_velocity_cmd_sum_before_debug:.6f}",
                        f"{x_velocity_cmd_after_debug_before_filter:.6f}",
                        f"{y_velocity_cmd_after_debug_before_filter:.6f}",
                        f"{x_velocity_cmd_after_filter_before_limit:.6f}",
                        f"{y_velocity_cmd_after_filter_before_limit:.6f}",
                        f"{self.velocity_integral_gain_x:.6f}",
                        f"{self.velocity_integral_gain_y:.6f}",
                        f"{self.velocity_position_derivative_gain_x:.6f}",
                        f"{self.velocity_position_derivative_gain_y:.6f}",
                        f"{self.velocity_position_derivative_filter_alpha_x:.6f}",
                        f"{self.velocity_position_derivative_filter_alpha_y:.6f}",
                        f"{self.velocity_error_gain_x:.6f}",
                        f"{self.velocity_error_gain_y:.6f}",
                        f"{self.command_velocity_filter_alpha_x:.6f}",
                        f"{self.command_velocity_filter_alpha_y:.6f}",
                        f"{self.velocity_debug_scale:.6f}",
                        f"{cmd_x_vel_sent:.6f}",
                        f"{cmd_y_vel_sent:.6f}",
                        f"{x_actual_vel_raw:.6f}",
                        f"{y_actual_vel_raw:.6f}",
                        f"{x_actual_vel:.6f}",
                        f"{y_actual_vel:.6f}",
                        "" if x_spi_sample_for_log is None else f"{x_spi_sample_for_log:.6f}",
                        "" if y_actual_sample is None else f"{y_actual_sample:.6f}",
                        x_measurement_source,
                        "" if x_endat_raw_count is None else str(x_endat_raw_count),
                        "" if x_endat_abs_deg is None else f"{x_endat_abs_deg:.6f}",
                        "" if x_endat_axis_deg is None else f"{x_endat_axis_deg:.6f}",
                        "" if x_endat_age_sec is None else f"{x_endat_age_sec:.6f}",
                        "" if x_endat_error1 is None else str(x_endat_error1),
                        "" if x_endat_error2 is None else str(x_endat_error2),
                        "" if x_endat_timeout_step is None else str(x_endat_timeout_step),
                        f"{x_endat_read_ms:.6f}",
                        str(int(x_bb_active)),
                        str(int(y_bb_active)),
                        f"{self.benedict_bordner_alpha_x:.6f}",
                        f"{self.benedict_bordner_alpha_y:.6f}",
                        f"{x_bb_beta:.6f}",
                        f"{y_bb_beta:.6f}",
                        f"{self.benedict_bordner_beta_scale_x:.6f}",
                        f"{self.benedict_bordner_beta_scale_y:.6f}",
                        "" if x_bb_predicted is None else f"{x_bb_predicted:.6f}",
                        "" if y_bb_predicted is None else f"{y_bb_predicted:.6f}",
                        f"{x_bb_residual:.6f}",
                        f"{y_bb_residual:.6f}",
                        f"{x_position_gain_active:.6f}",
                        f"{x_damping_gain_active:.6f}",
                        f"{y_position_gain_active:.6f}",
                        f"{y_damping_gain_active:.6f}",
                        f"{x_vel_error:.6f}",
                        f"{y_vel_error:.6f}",
                        str(x_gain_toward_vertical),
                        str(y_gain_toward_vertical),
                        f"{x_motor_vel_gain_active:.6f}",
                        f"{y_motor_vel_gain_active:.6f}",
                        str(int(use_velocity_tracking_mode)),
                        "" if x_control_mode is None else str(x_control_mode),
                        "" if x_input_mode is None else str(x_input_mode),
                        "" if y_control_mode is None else str(y_control_mode),
                        "" if y_input_mode is None else str(y_input_mode),
                        "" if x_load_encoder_id is None else str(x_load_encoder_id),
                        "" if x_load_encoder_name is None else x_load_encoder_name,
                        "" if x_commutation_encoder_id is None else str(x_commutation_encoder_id),
                        "" if x_commutation_encoder_name is None else x_commutation_encoder_name,
                        "" if x_use_commutation_vel is None else str(x_use_commutation_vel),
                        "" if x_use_load_encoder_for_commutation_vel is None else str(x_use_load_encoder_for_commutation_vel),
                        "" if x_pos_vel_mapper_scale is None else f"{x_pos_vel_mapper_scale:.6f}",
                        "" if y_load_encoder_id is None else str(y_load_encoder_id),
                        "" if y_load_encoder_name is None else y_load_encoder_name,
                        "" if y_commutation_encoder_id is None else str(y_commutation_encoder_id),
                        "" if y_commutation_encoder_name is None else y_commutation_encoder_name,
                        "" if y_use_commutation_vel is None else str(y_use_commutation_vel),
                        "" if y_use_load_encoder_for_commutation_vel is None else str(y_use_load_encoder_for_commutation_vel),
                        "" if y_pos_vel_mapper_scale is None else f"{y_pos_vel_mapper_scale:.6f}",
                        "" if x_controller_vel_limit is None else f"{x_controller_vel_limit:.6f}",
                        "" if y_controller_vel_limit is None else f"{y_controller_vel_limit:.6f}",
                        "" if x_traj_vel_limit is None else f"{x_traj_vel_limit:.6f}",
                        "" if x_traj_accel_limit is None else f"{x_traj_accel_limit:.6f}",
                        "" if x_traj_decel_limit is None else f"{x_traj_decel_limit:.6f}",
                        "" if y_traj_vel_limit is None else f"{y_traj_vel_limit:.6f}",
                        "" if y_traj_accel_limit is None else f"{y_traj_accel_limit:.6f}",
                        "" if y_traj_decel_limit is None else f"{y_traj_decel_limit:.6f}",
                        "" if x_actual_raw is None else f"{x_actual_raw:.6f}",
                        "" if y_actual_raw is None else f"{y_actual_raw:.6f}",
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
                        f"{x_pos_error_derivative:.6f}",
                        f"{y_pos_error_derivative:.6f}",
                        f"{x_pos_d_term:.6f}",
                        f"{y_pos_d_term:.6f}",
                        str(y_integral_unwind_applied),
                        f"{y_error_integral:.6f}",
                        f"{x_vel:.6f}",
                        f"{y_vel:.6f}",
                        f"{x_acc:.6f}",
                        f"{y_acc:.6f}",
                        f"{cmd_x_vel_ff_raw:.6f}",
                        f"{cmd_x_vel_ff:.6f}",
                        f"{self.x_vel_ff_attenuate_start_deg:.6f}",
                        f"{self.x_vel_ff_attenuate_stop_deg:.6f}",
                        f"{self.x_vel_ff_attenuate_scale:.6f}",
                        f"{cmd_y_vel_ff_raw:.6f}",
                        f"{cmd_y_vel_ff:.6f}",
                        f"{self.y_vel_ff_attenuate_start_deg:.6f}",
                        f"{self.y_vel_ff_attenuate_stop_deg:.6f}",
                        f"{self.y_vel_ff_attenuate_scale:.6f}",
                        f"{x_vel_ref_now:.6f}",
                        f"{y_vel_ref_now:.6f}",
                        self.velocity_controller_mode,
                        f"{x_velocity_p_term_used:.6f}",
                        f"{y_velocity_p_term_used:.6f}",
                        f"{x_velocity_i_term_used:.6f}",
                        f"{y_velocity_i_term_used:.6f}",
                        f"{x_velocity_i_state:.6f}",
                        f"{y_velocity_i_state:.6f}",
                        f"{x_velocity_pos_d_term_used:.6f}",
                        f"{y_velocity_pos_d_term_used:.6f}",
                        f"{x_velocity_vel_d_term_used:.6f}",
                        f"{y_velocity_vel_d_term_used:.6f}",
                        f"{x_velocity_feedback_sum_raw:.6f}",
                        f"{y_velocity_feedback_sum_raw:.6f}",
                        f"{x_velocity_feedback_sum_slewed:.6f}",
                        f"{y_velocity_feedback_sum_slewed:.6f}",
                        f"{x_velocity_feedback_slew_delta:.6f}",
                        f"{y_velocity_feedback_slew_delta:.6f}",
                        f"{x_velocity_feedback_slew_rate_limit:.6f}",
                        f"{y_velocity_feedback_slew_rate_limit:.6f}",
                        f"{self.velocity_correction_slew_rate_x:.6f}",
                        f"{self.velocity_correction_slew_rate_y:.6f}",
                        f"{self.x_velocity_correction_slew_zero_rate:.6f}",
                        f"{self.y_velocity_correction_slew_zero_rate:.6f}",
                        f"{self.x_velocity_correction_slew_zero_start_deg:.6f}",
                        f"{self.y_velocity_correction_slew_zero_start_deg:.6f}",
                        f"{self.x_velocity_correction_slew_zero_stop_deg:.6f}",
                        f"{self.y_velocity_correction_slew_zero_stop_deg:.6f}",
                        f"{x_velocity_cmd_sum_before_debug:.6f}",
                        f"{y_velocity_cmd_sum_before_debug:.6f}",
                        f"{x_velocity_cmd_after_debug_before_filter:.6f}",
                        f"{y_velocity_cmd_after_debug_before_filter:.6f}",
                        f"{x_velocity_cmd_after_filter_before_limit:.6f}",
                        f"{y_velocity_cmd_after_filter_before_limit:.6f}",
                        f"{self.velocity_integral_gain_x:.6f}",
                        f"{self.velocity_integral_gain_y:.6f}",
                        f"{self.velocity_position_derivative_gain_x:.6f}",
                        f"{self.velocity_position_derivative_gain_y:.6f}",
                        f"{self.velocity_position_derivative_filter_alpha_x:.6f}",
                        f"{self.velocity_position_derivative_filter_alpha_y:.6f}",
                        f"{self.velocity_error_gain_x:.6f}",
                        f"{self.velocity_error_gain_y:.6f}",
                        f"{self.command_velocity_filter_alpha_x:.6f}",
                        f"{self.command_velocity_filter_alpha_y:.6f}",
                        f"{self.velocity_debug_scale:.6f}",
                        f"{cmd_x_vel_sent:.6f}",
                        f"{cmd_y_vel_sent:.6f}",
                        f"{x_actual_vel_raw:.6f}",
                        f"{y_actual_vel_raw:.6f}",
                        f"{x_actual_vel:.6f}",
                        f"{y_actual_vel:.6f}",
                        "" if x_actual_sample is None else f"{x_actual_sample:.6f}",
                        "" if y_actual_sample is None else f"{y_actual_sample:.6f}",
                        str(int(x_bb_active)),
                        str(int(y_bb_active)),
                        f"{self.benedict_bordner_alpha_x:.6f}",
                        f"{self.benedict_bordner_alpha_y:.6f}",
                        f"{x_bb_beta:.6f}",
                        f"{y_bb_beta:.6f}",
                        f"{self.benedict_bordner_beta_scale_x:.6f}",
                        f"{self.benedict_bordner_beta_scale_y:.6f}",
                        "" if x_bb_predicted is None else f"{x_bb_predicted:.6f}",
                        "" if y_bb_predicted is None else f"{y_bb_predicted:.6f}",
                        f"{x_bb_residual:.6f}",
                        f"{y_bb_residual:.6f}",
                        f"{x_position_gain_active:.6f}",
                        f"{x_damping_gain_active:.6f}",
                        f"{y_position_gain_active:.6f}",
                        f"{y_damping_gain_active:.6f}",
                        f"{x_vel_error:.6f}",
                        f"{y_vel_error:.6f}",
                        str(x_gain_toward_vertical),
                        str(y_gain_toward_vertical),
                        f"{x_motor_vel_gain_active:.6f}",
                        f"{y_motor_vel_gain_active:.6f}",
                        str(int(use_velocity_tracking_mode)),
                        "" if x_load_encoder_id is None else str(x_load_encoder_id),
                        "" if x_load_encoder_name is None else x_load_encoder_name,
                        "" if x_commutation_encoder_id is None else str(x_commutation_encoder_id),
                        "" if x_commutation_encoder_name is None else x_commutation_encoder_name,
                        "" if x_use_commutation_vel is None else str(x_use_commutation_vel),
                        "" if x_use_load_encoder_for_commutation_vel is None else str(x_use_load_encoder_for_commutation_vel),
                        "" if x_pos_vel_mapper_scale is None else f"{x_pos_vel_mapper_scale:.6f}",
                        "" if y_load_encoder_id is None else str(y_load_encoder_id),
                        "" if y_load_encoder_name is None else y_load_encoder_name,
                        "" if y_commutation_encoder_id is None else str(y_commutation_encoder_id),
                        "" if y_commutation_encoder_name is None else y_commutation_encoder_name,
                        "" if y_use_commutation_vel is None else str(y_use_commutation_vel),
                        "" if y_use_load_encoder_for_commutation_vel is None else str(y_use_load_encoder_for_commutation_vel),
                        "" if y_pos_vel_mapper_scale is None else f"{y_pos_vel_mapper_scale:.6f}",
                        prepoint_status,
                    ]
                )
                log_file.flush()
                replay_file.flush()
                time.sleep(self.track_command_interval_sec)
        except Exception as exc:
            loop_exception = exc
            loop_traceback = traceback.format_exc()
        finally:
            if endat_x_reader is not None:
                try:
                    endat_x_reader.close()
                except Exception:
                    pass
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
            if loop_exception is not None:
                self.running = False
                self.current_sat_index = None
                error_text = (
                    f"Tracking failed: {loop_exception}\n\n"
                    f"{loop_traceback or ''}"
                )
                try:
                    self.after(0, lambda text=error_text: self.set_output_text(text))
                    self.after(0, lambda msg=str(loop_exception): messagebox.showerror("Tracking Error", msg))
                except Exception:
                    print(error_text)

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
