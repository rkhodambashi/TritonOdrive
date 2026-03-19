import datetime
import threading
import time
import tkinter as tk
from collections import deque
from math import ceil
from math import acos, atan, cos, pi, sin, sqrt
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


def clamp_tracking_angles(x_angle_deg, y_angle_deg):
    return (
        max(min(x_angle_deg, 90), -90),
        max(min(y_angle_deg, 90), -90),
    )


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
        self.geometry("820x840")

        self.odrvs = odrvs or {}
        self.control = control

        self.observer_lat = observer_lat
        self.observer_lon = observer_lon
        self.track_command_interval_sec = TRACK_COMMAND_INTERVAL_SEC
        self.trajectory_horizon_sec = TRAJECTORY_HORIZON_SEC
        self.trajectory_point_spacing_sec = TRAJECTORY_POINT_SPACING_SEC
        self.max_angle_step_deg = MAX_ANGLE_STEP_DEG
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

        ttk.Label(settings_frame, text="Rebuild Margin").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        self.rebuild_margin_entry = ttk.Entry(settings_frame, width=10)
        self.rebuild_margin_entry.grid(row=4, column=1, padx=5, pady=2)
        self.rebuild_margin_entry.insert(0, f"{self.trajectory_rebuild_margin_sec:g}")

        ttk.Label(settings_frame, text="Prepoint Lead").grid(row=5, column=0, sticky="w", padx=5, pady=2)
        self.prepoint_lead_entry = ttk.Entry(settings_frame, width=10)
        self.prepoint_lead_entry.grid(row=5, column=1, padx=5, pady=2)
        self.prepoint_lead_entry.insert(0, f"{self.prepoint_lead_time_sec:g}")

        ttk.Label(settings_frame, text="Min Spacing").grid(row=6, column=0, sticky="w", padx=5, pady=2)
        self.min_spacing_entry = ttk.Entry(settings_frame, width=10)
        self.min_spacing_entry.grid(row=6, column=1, padx=5, pady=2)
        self.min_spacing_entry.insert(0, f"{self.min_trajectory_spacing_sec:g}")

        ttk.Button(settings_frame, text="Apply", command=self.apply_tracking_settings).grid(
            row=7, column=0, columnspan=2, pady=6
        )

        output_frame = ttk.Frame(self)
        output_frame.pack(fill="x", padx=10, pady=10)

        self.output_text = tk.Text(output_frame, height=14, wrap="none")
        self.output_text.pack(side="left", fill="x", expand=True)
        self.output_text.configure(state="disabled")

        output_scrollbar = ttk.Scrollbar(output_frame, orient="vertical", command=self.output_text.yview)
        output_scrollbar.pack(side="right", fill="y")
        self.output_text.configure(yscrollcommand=output_scrollbar.set)

        self.error_fig = Figure(figsize=(7.8, 5.2), dpi=100)
        self.error_ax_x = self.error_fig.add_subplot(211)
        self.error_ax_y = self.error_fig.add_subplot(212, sharex=self.error_ax_x)
        self.error_ax_x.set_title("Tracking Error")
        self.error_ax_x.set_ylabel("X Error (deg)")
        self.error_ax_y.set_xlabel("Time (s)")
        self.error_ax_y.set_ylabel("Y Error (deg)")
        self.x_error_line, = self.error_ax_x.plot([], [], label="X Error")
        self.y_error_line, = self.error_ax_y.plot([], [], label="Y Error", color="tab:orange")
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

    def stream_tracking_command(self, previous_command, target_command, total_interval_sec):
        if not self.control:
            return target_command, 0.0

        if previous_command is None:
            update_odrive_axes(target_command[0], target_command[1], self.control)
            return target_command, 0.0

        dx = target_command[0] - previous_command[0]
        dy = target_command[1] - previous_command[1]
        max_delta = max(abs(dx), abs(dy))
        if self.max_angle_step_deg <= 0:
            steps = 1
        else:
            steps = max(1, int(ceil(max_delta / self.max_angle_step_deg)))

        if steps == 1:
            update_odrive_axes(target_command[0], target_command[1], self.control)
            return target_command, 0.0

        step_interval = total_interval_sec / steps
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            x_cmd = previous_command[0] + dx * alpha
            y_cmd = previous_command[1] + dy * alpha
            update_odrive_axes(x_cmd, y_cmd, self.control)
            if step_index < steps:
                time.sleep(step_interval)

        return target_command, total_interval_sec * (steps - 1) / steps

    def apply_tracking_settings(self):
        try:
            cmd_interval = float(self.cmd_interval_entry.get())
            horizon = float(self.horizon_entry.get())
            point_spacing = float(self.point_spacing_entry.get())
            max_angle_step = float(self.max_angle_step_entry.get())
            rebuild_margin = float(self.rebuild_margin_entry.get())
            prepoint_lead = float(self.prepoint_lead_entry.get())
            min_spacing = float(self.min_spacing_entry.get())

            if (
                cmd_interval <= 0
                or horizon <= 0
                or point_spacing <= 0
                or max_angle_step <= 0
                or rebuild_margin <= 0
                or prepoint_lead <= 0
                or min_spacing <= 0
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
            self.trajectory_rebuild_margin_sec = rebuild_margin
            self.prepoint_lead_time_sec = prepoint_lead
            self.min_trajectory_spacing_sec = min_spacing
        except Exception:
            messagebox.showerror(
                "Error",
                (
                    "Tracking settings must be positive numbers.\n"
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
        self.reset_error_plot()

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
                self.control.move_absolute_pair(x_deg=0, y_deg=0)
            except Exception:
                pass

    def track_satellite_loop(self, sat_index):
        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)
        sat = self.satellites[sat_index]
        was_visible = False
        tracking_started = False
        last_tracking_command = None
        trajectory = []
        trajectory_start_monotonic = 0.0
        trajectory_start_utc = None
        next_rise_utc = get_next_rise_time(ts, observer, sat)
        prepointed = False
        tracking_start_monotonic = time.monotonic()

        try:
            while self.running and sat_index == self.current_sat_index:
                now_monotonic = time.monotonic()
                now_utc = datetime.datetime.utcnow().replace(tzinfo=utc)

                if (
                    not trajectory
                    or trajectory_start_utc is None
                    or now_monotonic - trajectory_start_monotonic >= self.trajectory_horizon_sec - self.trajectory_rebuild_margin_sec
                ):
                    trajectory_start_monotonic = now_monotonic
                    trajectory_start_utc = datetime.datetime.utcnow().replace(tzinfo=utc)
                    trajectory = build_tracking_trajectory(
                        ts,
                        observer,
                        sat,
                        trajectory_start_utc,
                        self.trajectory_horizon_sec,
                        self.trajectory_point_spacing_sec,
                        self.max_angle_step_deg,
                        self.min_trajectory_spacing_sec,
                    )

                sample = sample_tracking_trajectory(trajectory, now_monotonic - trajectory_start_monotonic)
                if sample is None:
                    time.sleep(self.track_command_interval_sec)
                    continue

                prepoint_status = "Tracking pass"
                rise_eta_text = "Rise ETA: -"
                if next_rise_utc and not was_visible:
                    seconds_to_rise = (next_rise_utc - now_utc).total_seconds()
                    if seconds_to_rise > 0:
                        rise_eta_text = f"Rise ETA: {seconds_to_rise:.1f} s"
                        prepoint_status = f"Waiting for rise in {seconds_to_rise:.1f} s"
                        if seconds_to_rise <= self.prepoint_lead_time_sec:
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
                x_angle, y_angle = clamp_tracking_angles(sample["x_angle"], sample["y_angle"])
                x_actual = None
                y_actual = None
                x_error = 0.0
                y_error = 0.0

                if self.control:
                    try:
                        x_actual = self.control.get_spi_position("x")
                        y_actual = self.control.get_spi_position("y")
                        x_error = x_angle - x_actual
                        y_error = y_angle - y_actual
                    except Exception:
                        x_actual = None
                        y_actual = None

                self.set_output_text(
                    "\n".join(
                        [
                            f"Satellite: {sat.name}",
                            f"AZ: {az_deg:.3f}",
                            f"EL: {el_deg:.3f}",
                            f"X Angle: {x_angle:.3f}",
                            f"Y Angle: {y_angle:.3f}",
                            rise_eta_text,
                            f"X Actual: {x_actual:.3f}" if x_actual is not None else "X Actual: -",
                            f"Y Actual: {y_actual:.3f}" if y_actual is not None else "Y Actual: -",
                            f"X Error: {x_error:.3f}",
                            f"Y Error: {y_error:.3f}",
                            prepoint_status,
                        ]
                    )
                )

                if sample["visible"]:
                    was_visible = True
                    prepointed = False

                    if self.control and not tracking_started:
                        try:
                            filter_bandwidth = max(1.0, 1.0 / self.track_command_interval_sec)
                            self.control.enter_tracking_mode_all(input_filter_bandwidth=filter_bandwidth)
                        except Exception:
                            pass

                    if self.control and tracking_started:
                        self.update_error_plot(now_monotonic - tracking_start_monotonic, x_error, y_error)

                    consumed_sleep = 0.0
                    if self.control:
                        last_tracking_command, consumed_sleep = self.stream_tracking_command(
                            last_tracking_command,
                            (x_angle, y_angle),
                            self.track_command_interval_sec,
                        )
                    tracking_started = True
                else:
                    if prepointed and self.control:
                        update_odrive_axes(x_angle, y_angle, self.control)

                    if was_visible:
                        if self.control:
                            try:
                                self.control.exit_tracking_mode_all()
                            except Exception:
                                pass
                            try:
                                self.control.move_absolute_pair(x_deg=0, y_deg=0)
                            except Exception:
                                pass

                        self.running = False
                        break

                    tracking_started = False
                    last_tracking_command = None
                    consumed_sleep = 0.0

                remaining_sleep = self.track_command_interval_sec - consumed_sleep
                if remaining_sleep > 0:
                    time.sleep(remaining_sleep)
        finally:
            if self.control:
                try:
                    self.control.exit_tracking_mode_all()
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
