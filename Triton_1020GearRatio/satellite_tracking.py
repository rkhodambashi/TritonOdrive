import datetime
import threading
import time
import tkinter as tk
from math import acos, atan, cos, pi, sin, sqrt
from tkinter import filedialog, messagebox, ttk

from skyfield.api import EarthSatellite, Topos, load, utc

EPS = 1e-10
TRACK_COMMAND_INTERVAL_SEC = 0.05
TRAJECTORY_HORIZON_SEC = 2.0
TRAJECTORY_POINT_SPACING_SEC = 0.05
TRAJECTORY_REBUILD_MARGIN_SEC = 0.25
PREPOINT_LEAD_TIME_SEC = 30.0


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
    x_angle_deg = max(min(x_angle_deg, 90), -90)
    y_angle_deg = max(min(y_angle_deg, 90), -90)
    try:
        control.command_absolute_pair(x_deg=x_angle_deg, y_deg=y_angle_deg)
    except Exception as e:
        print(f"ODrive move error: {e}")


def build_tracking_trajectory(ts, observer, sat, start_time_utc, horizon_sec, spacing_sec):
    difference = sat - observer
    points = []

    steps = max(2, int(horizon_sec / spacing_sec) + 1)
    for index in range(steps):
        offset_sec = index * spacing_sec
        sample_time = start_time_utc + datetime.timedelta(seconds=offset_sec)
        t = ts.from_datetime(sample_time)
        topocentric = difference.at(t)
        el, az, distance = topocentric.altaz()

        az_deg = az.degrees
        el_deg = el.degrees

        points.append(
            {
                "offset_sec": offset_sec,
                "az_deg": az_deg,
                "el_deg": el_deg,
                "x_angle": Xangle(az_deg, el_deg),
                "y_angle": Yangle(az_deg, el_deg),
                "visible": el_deg >= 0,
            }
        )

    return points


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
    def __init__(self, parent, odrvs=None, control=None, observer_lat=33.67, observer_lon=-112.09):
        super().__init__(parent)

        self.title("Satellite Tracking")
        self.geometry("650x520")

        self.odrvs = odrvs or {}
        self.control = control

        self.observer_lat = observer_lat
        self.observer_lon = observer_lon

        self.running = False
        self.tracking_thread = None
        self.current_sat_index = None

        self.satellites = []
        self.display_map = []

        self.location_label = ttk.Label(self, text="Location: not set")
        self.location_label.pack()

        ttk.Label(self, text="Observer Latitude (deg):").pack()
        self.lat_entry = ttk.Entry(self, width=20)
        self.lat_entry.pack()
        self.lat_entry.insert(0, str(self.observer_lat))

        ttk.Label(self, text="Observer Longitude (deg):").pack()
        self.lon_entry = ttk.Entry(self, width=20)
        self.lon_entry.pack()
        self.lon_entry.insert(0, str(self.observer_lon))

        ttk.Button(self, text="Apply Location", command=self.apply_location).pack(pady=5)

        ttk.Label(self, text="Select TLE File:").pack(pady=5)
        ttk.Button(self, text="Browse", command=self.load_tle).pack(pady=5)

        self.tle_file_label = ttk.Label(self, text="No file selected")
        self.tle_file_label.pack(pady=5)

        ttk.Label(self, text="Select Satellite:").pack(pady=5)
        self.sat_combobox = ttk.Combobox(self, state="readonly", width=60)
        self.sat_combobox.pack(pady=5)

        ttk.Button(self, text="Start Tracking", command=self.start_tracking_thread).pack(pady=8)
        ttk.Button(self, text="Stop Tracking / Stow", command=self.stop_tracking).pack(pady=5)

        self.output_var = tk.StringVar(value="")
        self.output_label = ttk.Label(self, textvariable=self.output_var, justify="left", anchor="nw")
        self.output_label.pack(fill="both", expand=True, padx=10, pady=10)

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

        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)

        display_data = []
        self.display_map = []

        for idx, sat in enumerate(self.satellites):
            t0 = ts.now()
            t1 = ts.from_datetime(datetime.datetime.utcnow().replace(tzinfo=utc) + datetime.timedelta(hours=24))

            try:
                times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)

                next_rise = None
                for ti, event in zip(times, events):
                    if event == 0:
                        next_rise = ti.utc_datetime().replace(tzinfo=utc)
                        break

                if next_rise:
                    dt = next_rise - datetime.datetime.utcnow().replace(tzinfo=utc)
                    minutes = int(dt.total_seconds() / 60)
                    label = f"{sat.name} - Rise in {minutes} min"
                    sort_key = minutes
                else:
                    label = f"{sat.name} - No pass soon"
                    sort_key = 99999

            except Exception:
                label = f"{sat.name}"
                sort_key = 99999

            display_data.append((sort_key, label, idx))

        display_data.sort(key=lambda x: x[0])

        display_list = []
        for item in display_data:
            display_list.append(item[1])
            self.display_map.append(item[2])

        self.sat_combobox["values"] = display_list

        if display_list:
            self.sat_combobox.current(0)

        messagebox.showinfo("TLE Loaded", f"{len(self.satellites)} satellites loaded")

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
                self.control.move_absolute_pair(x_deg=0, y_deg=0)
            except Exception:
                pass

    def track_satellite_loop(self, sat_index):
        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)
        sat = self.satellites[sat_index]
        was_visible = False
        trajectory = []
        trajectory_start_monotonic = 0.0
        trajectory_start_utc = None
        next_rise_utc = get_next_rise_time(ts, observer, sat)
        prepointed = False

        while self.running and sat_index == self.current_sat_index:
            now_monotonic = time.monotonic()
            now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)

            if (
                not trajectory
                or trajectory_start_utc is None
                or now_monotonic - trajectory_start_monotonic >= TRAJECTORY_HORIZON_SEC - TRAJECTORY_REBUILD_MARGIN_SEC
            ):
                trajectory_start_monotonic = now_monotonic
                trajectory_start_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
                trajectory = build_tracking_trajectory(
                    ts,
                    observer,
                    sat,
                    trajectory_start_utc,
                    TRAJECTORY_HORIZON_SEC,
                    TRAJECTORY_POINT_SPACING_SEC,
                )

            sample = sample_tracking_trajectory(trajectory, now_monotonic - trajectory_start_monotonic)
            if sample is None:
                time.sleep(TRACK_COMMAND_INTERVAL_SEC)
                continue

            prepoint_status = "Tracking pass"
            if next_rise_utc and not was_visible:
                seconds_to_rise = (next_rise_utc - now_utc).total_seconds()
                if seconds_to_rise > 0:
                    prepoint_status = f"Waiting for rise in {seconds_to_rise:.1f} s"
                    if seconds_to_rise <= PREPOINT_LEAD_TIME_SEC:
                        rise_sample = get_pointing_sample(ts, observer, sat, next_rise_utc)
                        sample = {
                            "az_deg": rise_sample["az_deg"],
                            "el_deg": rise_sample["el_deg"],
                            "x_angle": rise_sample["x_angle"],
                            "y_angle": rise_sample["y_angle"],
                            "visible": False,
                        }
                        prepointed = True
                        prepoint_status = f"Prepointing rise in {seconds_to_rise:.1f} s"

            az_deg = sample["az_deg"]
            el_deg = sample["el_deg"]
            x_angle = sample["x_angle"]
            y_angle = sample["y_angle"]

            self.output_var.set(
                "\n".join(
                    [
                        f"Satellite: {sat.name}",
                        f"AZ: {az_deg:.3f}",
                        f"EL: {el_deg:.3f}",
                        f"X Angle: {x_angle:.3f}",
                        f"Y Angle: {y_angle:.3f}",
                        f"Cmd Interval: {TRACK_COMMAND_INTERVAL_SEC:.3f} s",
                        f"Trajectory Horizon: {TRAJECTORY_HORIZON_SEC:.1f} s",
                        prepoint_status,
                    ]
                )
            )

            if sample["visible"]:
                was_visible = True
                prepointed = False

                if self.control:
                    update_odrive_axes(x_angle, y_angle, self.control)
            else:
                if prepointed and self.control:
                    update_odrive_axes(x_angle, y_angle, self.control)

                if was_visible:
                    if self.control:
                        try:
                            self.control.move_absolute_pair(x_deg=0, y_deg=0)
                        except Exception:
                            pass

                    self.running = False
                    break

            time.sleep(TRACK_COMMAND_INTERVAL_SEC)

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
