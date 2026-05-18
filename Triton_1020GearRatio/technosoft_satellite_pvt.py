"""Satellite trajectory helpers for Technosoft PVT streaming.

This keeps the SGP4/TLE math separate from the drive code. The output is a
plain list of time, position, and velocity samples so the same shape can later
feed either PVT, velocity mode, or an offline preview.
"""

from __future__ import annotations

import datetime
from math import acos, atan, cos, pi, sin, sqrt
from pathlib import Path

from skyfield.api import EarthSatellite, Topos, load, utc


EPS = 1e-10


def k_vector(az_deg: float, el_deg: float) -> list[float]:
    az_rad = az_deg * pi / 180.0
    el_rad = el_deg * pi / 180.0
    x = cos(el_rad) * cos(az_rad)
    y = -cos(el_rad) * sin(az_rad)
    z = sin(el_rad)
    return [0 if abs(value) < EPS else value for value in (x, y, z)]


def x_vector(az_deg: float, el_deg: float) -> list[float]:
    k = k_vector(az_deg, el_deg)
    xvec = [0, k[1], k[2]]
    norm = sqrt(xvec[1] ** 2 + xvec[2] ** 2)
    if norm != 0:
        xvec[1] /= norm
        xvec[2] /= norm
    xvec[1] = 0 if abs(xvec[1]) < EPS else xvec[1]
    xvec[2] = 0 if abs(xvec[2]) < EPS else xvec[2]
    return xvec


def x_angle(az_deg: float, el_deg: float) -> float:
    xvec = x_vector(az_deg, el_deg)
    if xvec[2] == 0:
        if xvec[1] > 0:
            angle_rad = pi / 2
        elif xvec[1] < 0:
            angle_rad = -pi / 2
        else:
            angle_rad = 0
    else:
        angle_rad = atan(xvec[1] / xvec[2])
        if xvec[2] < 0:
            angle_rad += pi
    return angle_rad * 180.0 / pi


def y_angle(az_deg: float, el_deg: float) -> float:
    kvec = k_vector(az_deg, el_deg)
    xvec = x_vector(az_deg, el_deg)
    dot_prod = kvec[0] * xvec[0] + kvec[1] * xvec[1] + kvec[2] * xvec[2]
    dot_prod = max(min(dot_prod, 1), -1)
    angle_rad = acos(dot_prod)
    if kvec[0] < 0:
        angle_rad = -angle_rad
    return angle_rad * 180.0 / pi


def load_tle_file(path: str | Path) -> list[EarthSatellite]:
    tle_path = Path(path)
    with tle_path.open("r", encoding="utf-8") as tle_file:
        lines = [line.strip() for line in tle_file.readlines() if line.strip()]

    satellites = []
    index = 0
    while index < len(lines) - 1:
        if lines[index].startswith("1 ") and lines[index + 1].startswith("2 "):
            satellites.append(EarthSatellite(lines[index], lines[index + 1], f"SAT-{len(satellites) + 1}"))
            index += 2
        elif index < len(lines) - 2 and lines[index + 1].startswith("1 ") and lines[index + 2].startswith("2 "):
            satellites.append(EarthSatellite(lines[index + 1], lines[index + 2], lines[index]))
            index += 3
        else:
            index += 1
    return satellites


def format_local_datetime(dt_utc: datetime.datetime | None) -> str:
    if dt_utc is None:
        return "-"
    return dt_utc.astimezone(datetime.datetime.now().astimezone().tzinfo).strftime("%Y-%m-%d %I:%M:%S %p")


def next_rise_utc(
    sat: EarthSatellite,
    lat_deg: float,
    lon_deg: float,
    search_hours: float = 24.0,
    altitude_deg: float = 0.0,
) -> datetime.datetime | None:
    ts = load.timescale()
    observer = Topos(latitude_degrees=lat_deg, longitude_degrees=lon_deg)
    now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
    t0 = ts.now()
    t1 = ts.from_datetime(now_utc + datetime.timedelta(hours=search_hours))
    try:
        times, events = sat.find_events(observer, t0, t1, altitude_degrees=altitude_deg)
    except Exception:
        return None
    for ti, event in zip(times, events):
        if event == 0:
            return ti.utc_datetime().replace(tzinfo=utc)
    return None


def find_pickup_start_utc(
    sat: EarthSatellite,
    lat_deg: float,
    lon_deg: float,
    pickup_el_deg: float,
    max_abs_angle_deg: float,
    search_hours: float = 24.0,
    scan_after_sec: float = 1800.0,
    scan_step_sec: float = 0.5,
) -> datetime.datetime | None:
    """Find the next pickup time that is above the requested EL and inside axis limits."""
    candidate_utc = next_rise_utc(sat, lat_deg, lon_deg, search_hours=search_hours, altitude_deg=pickup_el_deg)
    if candidate_utc is None:
        return None

    ts = load.timescale()
    observer = Topos(latitude_degrees=lat_deg, longitude_degrees=lon_deg)
    steps = max(1, int(scan_after_sec / scan_step_sec))
    for index in range(steps + 1):
        sample_time = candidate_utc + datetime.timedelta(seconds=index * scan_step_sec)
        sample = pointing_sample(ts, observer, sat, sample_time)
        if (
            float(sample["el_deg"]) >= pickup_el_deg
            and abs(float(sample["x_angle_deg"])) <= max_abs_angle_deg
            and abs(float(sample["y_angle_deg"])) <= max_abs_angle_deg
        ):
            return sample_time
    return None


def find_track_end_utc(
    sat: EarthSatellite,
    lat_deg: float,
    lon_deg: float,
    start_time_utc: datetime.datetime,
    pickup_el_deg: float,
    max_abs_angle_deg: float,
    scan_after_sec: float = 1800.0,
    scan_step_sec: float = 0.5,
) -> datetime.datetime | None:
    """Find where the valid satellite track ends after the pickup/start time."""
    ts = load.timescale()
    observer = Topos(latitude_degrees=lat_deg, longitude_degrees=lon_deg)
    steps = max(1, int(scan_after_sec / scan_step_sec))
    last_valid_time = None
    seen_valid = False

    for index in range(steps + 1):
        sample_time = start_time_utc + datetime.timedelta(seconds=index * scan_step_sec)
        sample = pointing_sample(ts, observer, sat, sample_time)
        valid = (
            float(sample["el_deg"]) >= pickup_el_deg
            and abs(float(sample["x_angle_deg"])) <= max_abs_angle_deg
            and abs(float(sample["y_angle_deg"])) <= max_abs_angle_deg
        )
        if valid:
            last_valid_time = sample_time
            seen_valid = True
        elif seen_valid:
            return last_valid_time

    return last_valid_time


def satellite_display_labels(satellites: list[EarthSatellite], lat_deg: float, lon_deg: float) -> tuple[list[str], list[int]]:
    now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
    rows = []
    for index, sat in enumerate(satellites):
        rise = next_rise_utc(sat, lat_deg, lon_deg)
        short_name = sat.name[:24].rstrip()
        if rise is None:
            rows.append((999999999.0, f"{short_name} [{sat.model.satnum}]  no pass", index))
        else:
            seconds = max(0.0, (rise - now_utc).total_seconds())
            rows.append((seconds, f"{short_name} [{sat.model.satnum}]  {int(seconds / 60)}m", index))
    rows.sort(key=lambda item: item[0])
    return [row[1] for row in rows], [row[2] for row in rows]


def pointing_sample(ts, observer, sat: EarthSatellite, sample_time_utc: datetime.datetime) -> dict[str, float | bool]:
    topocentric = (sat - observer).at(ts.from_datetime(sample_time_utc))
    el, az, _distance = topocentric.altaz()
    az_deg = az.degrees
    el_deg = el.degrees
    return {
        "az_deg": az_deg,
        "el_deg": el_deg,
        "x_angle_deg": x_angle(az_deg, el_deg),
        "y_angle_deg": y_angle(az_deg, el_deg),
        "visible": el_deg >= 0.0,
    }


def build_satellite_pvt_points(
    sat: EarthSatellite,
    lat_deg: float,
    lon_deg: float,
    start_time_utc: datetime.datetime,
    duration_s: float,
    point_count: int,
    derivative_dt_s: float,
) -> list[dict[str, float | bool | str]]:
    if duration_s <= 0:
        raise ValueError("Satellite PVT duration must be positive")
    if point_count < 4:
        raise ValueError("Satellite PVT point count must be at least 4")

    ts = load.timescale()
    observer = Topos(latitude_degrees=lat_deg, longitude_degrees=lon_deg)
    segment_s = duration_s / point_count
    derivative_dt_s = max(1e-4, min(float(derivative_dt_s), segment_s))

    samples = []
    for index in range(point_count + 1):
        offset_s = index * segment_s
        sample_time = start_time_utc + datetime.timedelta(seconds=offset_s)
        current = pointing_sample(ts, observer, sat, sample_time)
        prev_sample = pointing_sample(ts, observer, sat, sample_time - datetime.timedelta(seconds=derivative_dt_s))
        next_sample = pointing_sample(ts, observer, sat, sample_time + datetime.timedelta(seconds=derivative_dt_s))
        velocity_dt = max(1e-4, 2.0 * derivative_dt_s)
        samples.append(
            {
                "utc_time": sample_time.isoformat(),
                "offset_s": offset_s,
                "az_deg": float(current["az_deg"]),
                "el_deg": float(current["el_deg"]),
                "x_angle_deg": float(current["x_angle_deg"]),
                "y_angle_deg": float(current["y_angle_deg"]),
                "x_vel_deg_s": (float(next_sample["x_angle_deg"]) - float(prev_sample["x_angle_deg"])) / velocity_dt,
                "y_vel_deg_s": (float(next_sample["y_angle_deg"]) - float(prev_sample["y_angle_deg"])) / velocity_dt,
                "visible": bool(current["visible"]),
            }
        )
    return samples
