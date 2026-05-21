"""Minimal GUI for direct Technosoft iPOS8020 bring-up."""

from __future__ import annotations

import csv
import time
import tkinter as tk
from datetime import datetime, timedelta, timezone
from pathlib import Path
from tkinter import filedialog, ttk

import matplotlib

matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from technosoft_minimal_controller import (
    CHANNEL_PEAK_SYS_PCAN_USB,
    DEFAULT_BAUDRATE,
    DEFAULT_CHANNEL_NAME,
    DEFAULT_HOST_ID,
    DEFAULT_SETUP_PATH,
    GAIN_NAMES,
    AxisConfig,
    TechnosoftEx04,
)
from technosoft_satellite_pvt import (
    build_satellite_pvt_points,
    find_pickup_start_utc,
    find_track_end_utc,
    format_local_datetime,
    load_tle_file,
    next_rise_utc,
    satellite_display_labels,
)


controller = TechnosoftEx04()
connected_axes: set[str] = set()
refresh_after_id = None
satellite_pvt_window = None
satellites = []
satellite_display_map: list[int] = []
satellite_tle_path: Path | None = None
DEFAULT_SATELLITE_TLE_LOAD_LIMIT = 100


def pvt_log_dir() -> Path:
    path = Path(__file__).resolve().parent / "tracking_logs" / "technosoft_pvt_logs"
    path.mkdir(parents=True, exist_ok=True)
    return path


def log(message: str) -> None:
    stamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{stamp}] {message}"
    print(line, flush=True)
    log_text.insert("end", line + "\n")
    log_text.see("end")


def run_worker(label: str, func) -> None:
    try:
        result = func()
        log(f"{label} OK" + (f": {result}" if result is not None else ""))
        refresh_once()
    except Exception as exc:
        log(f"{label} FAILED: {exc}")


def get_axis_config(axis: str) -> AxisConfig:
    node_id = int(axis_vars[axis]["node_id"].get())
    setup_path = Path(axis_vars[axis]["setup"].get().strip())
    sign = float(axis_vars[axis]["sign"].get())
    return AxisConfig(axis=axis, node_id=node_id, setup_path=setup_path, output_sign=sign)


def connect_axis(axis: str) -> None:
    def action():
        controller.open_channel(
            channel_name=channel_var.get().strip() or DEFAULT_CHANNEL_NAME,
            channel_type=int(channel_type_var.get()),
            host_id=int(host_id_var.get()),
            baudrate=int(baudrate_var.get()),
        )
        state = controller.setup_axis_only(get_axis_config(axis))
        controller.set_motion_params_deg(
            axis,
            float(speed_deg_s_var.get()),
            float(accel_deg_s2_var.get()),
            float(decel_deg_s2_var.get()),
        )
        connected_axes.add(axis)
        return (
            f"node={state.config.node_id}, setup={state.config.setup_path}, "
            f"load_scale={state.load_scale:g}, motor_scale={state.motor_scale:g}, "
            f"time_scale_ms={state.time_scale_ms:g}"
        )

    run_worker(f"Connect {axis.upper()}", action)


def vendor_init_axis(axis: str) -> None:
    def action():
        controller.open_channel(
            channel_name=channel_var.get().strip() or DEFAULT_CHANNEL_NAME,
            channel_type=int(channel_type_var.get()),
            host_id=int(host_id_var.get()),
            baudrate=int(baudrate_var.get()),
        )
        state = controller.vendor_init_axis(get_axis_config(axis))
        controller.set_motion_params_deg(
            axis,
            float(speed_deg_s_var.get()),
            float(accel_deg_s2_var.get()),
            float(decel_deg_s2_var.get()),
        )
        connected_axes.add(axis)
        return (
            f"Ex04 init complete: node={state.config.node_id}, "
            f"load_scale={state.load_scale:g}, motor_scale={state.motor_scale:g}"
        )

    run_worker(f"Vendor Init {axis.upper()}", action)


def close_channel() -> None:
    def action():
        controller.close()
        connected_axes.clear()

    run_worker("Close channel", action)


def refresh_once() -> None:
    for axis in ("x", "y"):
        if axis not in connected_axes:
            pos_vars[axis].set("not connected")
            status_vars[axis].set("")
            continue
        try:
            pos = controller.read_positions(axis)
            status = controller.read_status(axis)
            pos_vars[axis].set(
                f"APOS {pos['APOS']} ({pos['APOS_deg']:.6f} deg) | "
                f"CPOS {pos['CPOS']} ({pos['CPOS_deg']:.6f} deg) | "
                f"TPOS {pos['TPOS']} ({pos['TPOS_deg']:.6f} deg)"
            )
            status_vars[axis].set(" ".join(f"{name}=0x{value:04X}" for name, value in status.items()))
        except Exception as exc:
            status_vars[axis].set(f"refresh failed: {exc}")


def refresh_loop() -> None:
    global refresh_after_id
    refresh_once()
    refresh_after_id = root.after(200, refresh_loop)


def set_motion_params() -> None:
    def action():
        for axis in connected_axes:
            controller.set_motion_params_deg(
                axis,
                float(speed_deg_s_var.get()),
                float(accel_deg_s2_var.get()),
                float(decel_deg_s2_var.get()),
            )
        if abs(float(accel_deg_s2_var.get()) - float(decel_deg_s2_var.get())) > 1e-12:
            return "Ex04 trapezoidal moves use the Accel deg/s2 value for both acceleration and deceleration"

    run_worker("Set motion params", action)


def axis_command(axis: str, label: str, func) -> None:
    run_worker(f"{axis.upper()} {label}", func)


def read_gains(axis: str) -> None:
    def action():
        gains = controller.read_gains(axis)
        for name, value in gains.items():
            gain_vars[axis][name].set(str(value))
        return gains

    axis_command(axis, "read gains", action)


def apply_gains(axis: str) -> None:
    def action():
        gains = {name: int(float(gain_vars[axis][name].get())) for name in GAIN_NAMES}
        return controller.write_gains(axis, gains)

    axis_command(axis, "apply gains", action)


def restore_pointing_move_settings(axis: str) -> dict[str, object]:
    """Apply the GUI's pointing/profile settings before a position move."""
    controller.set_motion_params_deg(
        axis,
        float(speed_deg_s_var.get()),
        float(accel_deg_s2_var.get()),
        float(decel_deg_s2_var.get()),
    )
    gains = {}
    for name in GAIN_NAMES:
        text = gain_vars[axis][name].get().strip()
        if text:
            gains[name] = int(float(text))
    applied_gains = controller.write_gains(axis, gains) if gains else {}
    return {"profile": "pointing", "gains": applied_gains}


def home_axis_with_pointing_settings(axis: str):
    """Use this for every Home 0 request so homing setup stays centralized."""
    restore_pointing_move_settings(axis)
    return controller.home_absolute_deg_checked(axis, 0.0)


def move_relative(axis: str, delta_deg: float) -> None:
    axis_command(axis, f"move relative {delta_deg:+g} deg", lambda: controller.move_relative_deg(axis, delta_deg))


def move_absolute(axis: str) -> None:
    target = float(axis_vars[axis]["abs_target"].get())
    axis_command(axis, f"move absolute {target:g} deg", lambda: controller.move_absolute_deg(axis, target))


def go_home(axis: str) -> None:
    axis_command(axis, "home / vertical 0", lambda: home_axis_with_pointing_settings(axis))


def set_vertical_zero(axis: str) -> None:
    def action():
        result = controller.current_vertical_zero_hardcode(axis)
        return f"vertical zero now: {result['code_line']}"

    axis_command(axis, "set vertical zero", action)


def run_streaming_pvt(axis: str) -> None:
    def action():
        return controller.run_streaming_pvt_line_deg(
            axis,
            float(axis_vars[axis]["pvt_target"].get()),
            float(axis_vars[axis]["pvt_duration"].get()),
            int(float(axis_vars[axis]["pvt_stream_points"].get())),
            axis_vars[axis]["pvt_profile"].get(),
        )

    axis_command(axis, "streaming PVT", action)


def run_sine_pvt(axis: str) -> None:
    def action():
        return controller.run_streaming_pvt_sine_deg(
            axis,
            float(axis_vars[axis]["pvt_sine_center"].get()),
            float(axis_vars[axis]["pvt_sine_amplitude"].get()),
            float(axis_vars[axis]["pvt_sine_period"].get()),
            float(axis_vars[axis]["pvt_sine_duration"].get()),
            int(float(axis_vars[axis]["pvt_sine_points"].get())),
        )

    axis_command(axis, "sine PVT", action)


def run_positive_horizon_slow_pvt(axis: str) -> None:
    def action():
        start_deg = 90.0
        target_deg = 85.0
        duration_s = 40.0
        sample_dt_s = 0.5
        point_count = int(round(duration_s / sample_dt_s))
        current_tpos_deg = float(controller.read_positions(axis)["TPOS_deg"])
        if abs(current_tpos_deg - start_deg) > 2.0:
            raise RuntimeError(
                f"Move {axis.upper()} near +90 deg first. Current TPOS is {current_tpos_deg:.3f} deg; "
                "this diagnostic intentionally does not preposition the axis."
            )
        velocity_deg_s = (target_deg - start_deg) / duration_s
        points = [
            (start_deg + (target_deg - start_deg) * index / point_count, velocity_deg_s)
            for index in range(1, point_count + 1)
        ]
        metadata = {
            "trajectory": "synthetic_positive_horizon_slow",
            "start_deg": start_deg,
            "target_deg": target_deg,
            "duration_s": duration_s,
            "sample_dt_s": sample_dt_s,
            "diagnostic_kpp": 500,
            "reason": "mimic failed TLE +90-to-+85 slow segment without TLE math",
        }
        old_gains = controller.read_gains(axis)
        try:
            controller.write_gains(axis, {"KPP": 500})
            result = controller.run_streaming_pvt_axis_deg(axis, points, duration_s, metadata=metadata)
            result["restored_KPP"] = old_gains.get("KPP", "")
            return result
        finally:
            controller.write_gains(axis, old_gains)

    axis_command(axis, "+90 to +85 slow diagnostic PVT", action)


def open_pvt_window(axis: str) -> None:
    window = tk.Toplevel(root)
    window.title(f"{axis.upper()} Axis PVT Trajectories")
    window.geometry("560x540")
    window.transient(root)

    frame = ttk.Frame(window, padding=12)
    frame.pack(fill="both", expand=True)
    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(1, weight=1)

    line_frame = ttk.LabelFrame(frame, text="Line PVT", padding=10)
    line_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
    ttk.Label(line_frame, text="Target deg").grid(row=0, column=0, sticky="e")
    ttk.Entry(line_frame, textvariable=axis_vars[axis]["pvt_target"], width=12).grid(row=0, column=1, sticky="w", padx=6)
    ttk.Label(line_frame, text="Duration s").grid(row=1, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(line_frame, textvariable=axis_vars[axis]["pvt_duration"], width=12).grid(row=1, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(line_frame, text="Point count").grid(row=2, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(line_frame, textvariable=axis_vars[axis]["pvt_stream_points"], width=12).grid(row=2, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(line_frame, text="Profile").grid(row=3, column=0, sticky="e", pady=(6, 0))
    ttk.Combobox(
        line_frame,
        textvariable=axis_vars[axis]["pvt_profile"],
        values=("smoothstep", "linear"),
        state="readonly",
        width=12,
    ).grid(row=3, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Button(line_frame, text="Run Line PVT", command=lambda a=axis: run_streaming_pvt(a)).grid(
        row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0)
    )

    sine_frame = ttk.LabelFrame(frame, text="Sine PVT", padding=10)
    sine_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
    ttk.Label(sine_frame, text="Center deg").grid(row=0, column=0, sticky="e")
    ttk.Entry(sine_frame, textvariable=axis_vars[axis]["pvt_sine_center"], width=12).grid(row=0, column=1, sticky="w", padx=6)
    ttk.Label(sine_frame, text="Amplitude deg").grid(row=1, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(sine_frame, textvariable=axis_vars[axis]["pvt_sine_amplitude"], width=12).grid(row=1, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(sine_frame, text="Period s").grid(row=2, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(sine_frame, textvariable=axis_vars[axis]["pvt_sine_period"], width=12).grid(row=2, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(sine_frame, text="Duration s").grid(row=3, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(sine_frame, textvariable=axis_vars[axis]["pvt_sine_duration"], width=12).grid(row=3, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(sine_frame, text="Point count").grid(row=4, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(sine_frame, textvariable=axis_vars[axis]["pvt_sine_points"], width=12).grid(row=4, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Button(sine_frame, text="Run Sine PVT", command=lambda a=axis: run_sine_pvt(a)).grid(
        row=5, column=0, columnspan=2, sticky="ew", pady=(12, 0)
    )

    diag_frame = ttk.LabelFrame(frame, text="Diagnostics", padding=10)
    diag_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
    ttk.Button(
        diag_frame,
        text="Run +90 -> +85 deg / 40 s Test",
        command=lambda a=axis: run_positive_horizon_slow_pvt(a),
    ).grid(row=0, column=0, sticky="ew")
    ttk.Label(
        diag_frame,
        text=(
            "Feeds the same arbitrary absolute PVT path used by satellite tracking: "
            "80 points, 0.5 s/point, constant -0.125 deg/s. It refuses to run unless TPOS is near +90 deg."
        ),
        wraplength=500,
    ).grid(row=1, column=0, sticky="w", pady=(8, 0))

    ttk.Label(
        frame,
        text=(
            "Use 5 points/sec or less, e.g. 4 s -> 20 points, 10 s -> 50 points, "
            "20 s -> 100 points. Sine starts from the current position if it is inside center +/- amplitude."
        ),
        wraplength=500,
    ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(14, 0))


def write_satellite_reference_log(sat_name: str, samples: list[dict[str, float | bool | str]]) -> Path:
    safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in sat_name).strip("_") or "satellite"
    path = pvt_log_dir() / f"technosoft_satellite_reference_{safe_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "utc_time",
                "offset_s",
                "visible",
                "az_deg",
                "el_deg",
                "x_angle_deg",
                "y_angle_deg",
                "x_vel_deg_s",
                "y_vel_deg_s",
            ],
        )
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample)
    return path


def open_satellite_pvt_window() -> None:
    global satellite_pvt_window, satellites, satellite_display_map, satellite_tle_path
    if satellite_pvt_window is not None and satellite_pvt_window.winfo_exists():
        satellite_pvt_window.lift()
        satellite_pvt_window.focus_force()
        return

    satellite_pvt_window = tk.Toplevel(root)
    satellite_pvt_window.title("Satellite PVT")
    satellite_pvt_window.geometry("760x650")
    satellite_pvt_window.transient(root)

    def on_close() -> None:
        global satellite_pvt_window
        satellite_pvt_window = None
        window.destroy()

    window = satellite_pvt_window
    window.protocol("WM_DELETE_WINDOW", on_close)

    lat_var = tk.StringVar(value="33.67")
    lon_var = tk.StringVar(value="-112.09")
    sample_period_var = tk.StringVar(value="0.2")
    lead_in_points_var = tk.StringVar(value="2")
    max_pass_scan_var = tk.StringVar(value="1800")
    pickup_el_var = tk.StringVar(value="0")
    pickup_offset_var = tk.StringVar(value="0")
    derivative_dt_var = tk.StringVar(value="0.2")
    max_start_error_var = tk.StringVar(value="2")
    max_abs_angle_var = tk.StringVar(value="91")
    pvt_vcorr_enable_var = tk.StringVar(value="0")
    pvt_vcorr_kp_var = tk.StringVar(value="0")
    pvt_vcorr_max_var = tk.StringVar(value="0.2")
    pvt_vcorr_alpha_var = tk.StringVar(value="0")
    satellite_axes_var = tk.StringVar(value="x")
    tle_load_limit_var = tk.StringVar(value=str(DEFAULT_SATELLITE_TLE_LOAD_LIMIT))
    satellite_search_var = tk.StringVar(value="")
    tle_label_var = tk.StringVar(value="No TLE selected")
    summary_var = tk.StringVar(value="Load a TLE, choose a satellite, preview, then run.")
    countdown_var = tk.StringVar(value="Countdown: -")
    stop_track_requested = {"value": False}
    satellite_pvt_active = {"value": False}

    class LiveErrorPlot(ttk.LabelFrame):
        def __init__(self, parent):
            super().__init__(parent, text="Live APOS - Target Error")
            self.error_time: list[float] = []
            self.x_error_history: list[float] = []
            self.y_error_history: list[float] = []

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
            self.error_canvas.get_tk_widget().pack(fill="both", expand=True)

        def reset(self) -> None:
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

        def add_sample(self, axis: str, t_s: float, error_deg: float) -> None:
            elapsed_sec = float(t_s)
            error_deg = float(error_deg)
            if axis.lower() == "x":
                self.error_time.append(elapsed_sec)
                self.x_error_history.append(error_deg)
                self.y_error_history.append(float("nan"))
            elif axis.lower() == "y":
                self.error_time.append(elapsed_sec)
                self.x_error_history.append(float("nan"))
                self.y_error_history.append(error_deg)
            else:
                return
            self.x_error_line.set_data(list(self.error_time), list(self.x_error_history))
            self.y_error_line.set_data(list(self.error_time), list(self.y_error_history))
            xmax = max(1.0, elapsed_sec)
            self.error_ax_x.set_xlim(0, xmax)
            self.error_ax_y.set_xlim(0, xmax)
            self.error_ax_x.set_ylim(-0.1, 0.1)
            self.error_ax_y.set_ylim(-0.1, 0.1)
            self.error_canvas.draw_idle()

    def set_countdown(message: str) -> None:
        countdown_var.set(message)
        window.update()

    def set_summary(message: str) -> None:
        summary_var.set(message)
        window.update_idletasks()

    frame = ttk.Frame(window, padding=12)
    frame.pack(fill="both", expand=True)
    frame.columnconfigure(1, weight=1)
    frame.columnconfigure(3, weight=1)
    frame.rowconfigure(8, weight=1)
    error_plot = LiveErrorPlot(frame)

    ttk.Label(frame, text="Latitude").grid(row=0, column=0, sticky="e", padx=5, pady=3)
    ttk.Entry(frame, textvariable=lat_var, width=12).grid(row=0, column=1, sticky="w", padx=5, pady=3)
    ttk.Label(frame, text="Longitude").grid(row=0, column=2, sticky="e", padx=5, pady=3)
    ttk.Entry(frame, textvariable=lon_var, width=12).grid(row=0, column=3, sticky="w", padx=5, pady=3)

    sat_combo = ttk.Combobox(frame, state="readonly", width=42)

    def refresh_satellite_list() -> None:
        global satellite_display_map
        if not satellites:
            return
        search = satellite_search_var.get().strip().lower()
        labels, display_map = satellite_display_labels(satellites, float(lat_var.get()), float(lon_var.get()))
        filtered = [
            (label, sat_index)
            for label, sat_index in zip(labels, display_map)
            if not search or search in satellites[sat_index].name.lower() or search in str(satellites[sat_index].model.satnum)
        ]
        satellite_display_map = [sat_index for _label, sat_index in filtered]
        labels = [label for label, _sat_index in filtered]
        sat_combo["values"] = labels
        if labels:
            sat_combo.current(0)
        else:
            sat_combo.set("")

    def browse_tle() -> None:
        global satellites, satellite_tle_path
        file_path = filedialog.askopenfilename(filetypes=[("TLE Files", "*.txt"), ("All Files", "*.*")])
        if not file_path:
            return
        satellite_tle_path = Path(file_path)
        load_limit = max(1, int(float(tle_load_limit_var.get())))
        satellites = load_tle_file(satellite_tle_path)[:load_limit]
        tle_label_var.set(satellite_tle_path.name)
        refresh_satellite_list()
        set_summary(
            f"Loaded first {len(satellites)} satellites from {satellite_tle_path.name} "
            f"(load limit {load_limit})"
        )

    ttk.Button(frame, text="Browse TLE", command=browse_tle).grid(row=1, column=0, sticky="ew", padx=5, pady=(10, 3))
    ttk.Label(frame, textvariable=tle_label_var).grid(row=1, column=1, sticky="w", padx=5, pady=(10, 3))
    ttk.Label(frame, text="Load count").grid(row=1, column=2, sticky="e", padx=5, pady=(10, 3))
    ttk.Entry(frame, textvariable=tle_load_limit_var, width=10).grid(row=1, column=3, sticky="w", padx=5, pady=(10, 3))
    ttk.Label(frame, text="Search").grid(row=2, column=0, sticky="e", padx=5, pady=3)
    search_entry = ttk.Entry(frame, textvariable=satellite_search_var, width=18)
    search_entry.grid(row=2, column=1, sticky="w", padx=5, pady=3)
    search_entry.bind("<Return>", lambda _event: refresh_satellite_list())
    ttk.Button(frame, text="Apply Search", command=refresh_satellite_list).grid(row=2, column=2, sticky="ew", padx=5, pady=3)
    ttk.Button(frame, text="Refresh Passes", command=refresh_satellite_list).grid(row=2, column=3, sticky="ew", padx=5, pady=3)
    ttk.Label(frame, text="Satellite").grid(row=3, column=0, sticky="e", padx=5, pady=3)
    sat_combo.grid(row=3, column=1, columnspan=3, sticky="ew", padx=5, pady=3)

    settings = ttk.LabelFrame(frame, text="PVT Segment", padding=8)
    settings.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(12, 6))
    for col in range(6):
        settings.columnconfigure(col, weight=1 if col % 2 else 0)

    entries = [
        ("Axes", satellite_axes_var),
        ("Pickup EL deg", pickup_el_var),
        ("Pickup offset s", pickup_offset_var),
        ("Lead-in pts", lead_in_points_var),
        ("Sample dt s", sample_period_var),
        ("Max pass scan s", max_pass_scan_var),
        ("Derivative dt s", derivative_dt_var),
        ("Max start err deg", max_start_error_var),
        ("Max abs angle deg", max_abs_angle_var),
        ("V Corr Enable", pvt_vcorr_enable_var),
        ("V Corr Kp 1/s", pvt_vcorr_kp_var),
        ("V Corr Max deg/s", pvt_vcorr_max_var),
        ("V Corr Alpha", pvt_vcorr_alpha_var),
    ]
    for index, (label_text, variable) in enumerate(entries):
        row = index // 3
        col = (index % 3) * 2
        ttk.Label(settings, text=label_text).grid(row=row, column=col, sticky="e", padx=5, pady=3)
        if label_text == "Axes":
            ttk.Combobox(
                settings,
                textvariable=variable,
                values=("x", "y", "both"),
                state="readonly",
                width=10,
            ).grid(row=row, column=col + 1, sticky="w", padx=5, pady=3)
        else:
            ttk.Entry(settings, textvariable=variable, width=12).grid(row=row, column=col + 1, sticky="w", padx=5, pady=3)

    def selected_satellite():
        selection = sat_combo.current()
        if selection < 0 or selection >= len(satellite_display_map):
            raise RuntimeError("Select a satellite first")
        return satellites[satellite_display_map[selection]]

    def selected_axis_tuple() -> tuple[str, ...]:
        selected_axes = satellite_axes_var.get()
        return ("x", "y") if selected_axes == "both" else (selected_axes,)

    def terminate_selected_axes(home: bool = True) -> None:
        for axis in selected_axis_tuple():
            if axis not in connected_axes:
                continue
            if home:
                # A failed/aborted PVT run can leave the drive in a state where
                # a light stop is not enough. Match the manual recovery sequence
                # that has been reliable: power off, vendor init, then home.
                try:
                    controller.stop(axis)
                except Exception as exc:
                    log(f"{axis.upper()} stop before recovery ignored: {exc}")
                try:
                    controller.power(axis, False)
                except Exception as exc:
                    log(f"{axis.upper()} power off before recovery ignored: {exc}")
                time.sleep(0.2)
                controller.vendor_init_axis(get_axis_config(axis))
                controller.set_motion_params_deg(
                    axis,
                    float(speed_deg_s_var.get()),
                    float(accel_deg_s2_var.get()),
                    float(decel_deg_s2_var.get()),
                )
                connected_axes.add(axis)
                home_axis_with_pointing_settings(axis)
            else:
                controller.stop(axis)

    def home_selected_axes_after_success() -> None:
        try:
            terminate_selected_axes(home=True)
        except Exception as exc:
            log(f"Home after successful PVT warning: {exc}")

    def build_current_plan():
        sat = selected_satellite()
        sample_period_s = max(0.2, float(sample_period_var.get()))
        lead_in_points = max(0, int(float(lead_in_points_var.get())))
        max_pass_scan_s = float(max_pass_scan_var.get())
        pickup_el_deg = float(pickup_el_var.get())
        pickup_offset_s = float(pickup_offset_var.get())
        derivative_dt_s = float(derivative_dt_var.get())
        max_abs_angle = float(max_abs_angle_var.get())
        pickup_utc = find_pickup_start_utc(
            sat,
            float(lat_var.get()),
            float(lon_var.get()),
            pickup_el_deg,
            max_abs_angle,
        )
        if pickup_utc is None:
            raise RuntimeError(
                f"No pickup point found above {pickup_el_deg:g} deg EL within +/-{max_abs_angle:g} deg axis limits."
            )
        pickup_start_utc = pickup_utc + timedelta(seconds=pickup_offset_s)
        end_utc = find_track_end_utc(
            sat,
            float(lat_var.get()),
            float(lon_var.get()),
            pickup_start_utc,
            pickup_el_deg,
            max_abs_angle,
            scan_after_sec=max_pass_scan_s,
            scan_step_sec=sample_period_s,
        )
        if end_utc is None:
            raise RuntimeError("Could not find a valid track end after pickup/start time.")
        for actual_lead_in_points in range(lead_in_points, -1, -1):
            start_utc = pickup_start_utc - timedelta(seconds=actual_lead_in_points * sample_period_s)
            duration_s = (end_utc - start_utc).total_seconds()
            if duration_s <= sample_period_s:
                continue
            point_count = max(4, int(round(duration_s / sample_period_s)))
            duration_s = point_count * sample_period_s
            samples = build_satellite_pvt_points(
                sat,
                float(lat_var.get()),
                float(lon_var.get()),
                start_utc,
                duration_s,
                point_count,
                derivative_dt_s,
            )
            max_x = max(abs(float(sample["x_angle_deg"])) for sample in samples)
            max_y = max(abs(float(sample["y_angle_deg"])) for sample in samples)
            if max(max_x, max_y) <= max_abs_angle:
                return (
                    sat,
                    samples,
                    duration_s,
                    point_count,
                    pickup_utc,
                    pickup_start_utc,
                    start_utc,
                    end_utc,
                    sample_period_s,
                    actual_lead_in_points,
                )
        raise RuntimeError(
            f"Trajectory exceeds max angle even with no lead-in. "
            f"Check pickup elevation, pickup offset, and angle limit {max_abs_angle:.2f} deg."
        )

    def summarize_plan(check_start: bool) -> tuple[str, list[dict[str, float | bool | str]], float, int, datetime]:
        (
            sat,
            samples,
            duration_s,
            point_count,
            pickup_utc,
            pickup_start_utc,
            start_utc,
            end_utc,
            sample_period_s,
            lead_in_points,
        ) = build_current_plan()
        first = samples[0]
        last = samples[-1]
        max_abs_angle = float(max_abs_angle_var.get())
        max_x = max(abs(float(sample["x_angle_deg"])) for sample in samples)
        max_y = max(abs(float(sample["y_angle_deg"])) for sample in samples)
        if max(max_x, max_y) > max_abs_angle:
            raise RuntimeError(
                f"Trajectory exceeds max angle: max |X|={max_x:.2f}, max |Y|={max_y:.2f}, limit={max_abs_angle:.2f}"
            )
        start_error_text = ""
        if check_start:
            selected_axes = satellite_axes_var.get()
            required_axes = ("x", "y") if selected_axes == "both" else (selected_axes,)
            missing_axes = [axis for axis in required_axes if axis not in connected_axes]
            if missing_axes:
                raise RuntimeError(f"Connect {', '.join(axis.upper() for axis in missing_axes)} before running satellite PVT")
            max_start_error = float(max_start_error_var.get())
            start_errors = []
            max_start_error_seen = 0.0
            if "x" in required_axes:
                x_pos = controller.read_positions("x")
                x_start_error = float(first["x_angle_deg"]) - float(x_pos["APOS_deg"])
                start_errors.append(f"X={x_start_error:.3f} deg")
                max_start_error_seen = max(max_start_error_seen, abs(x_start_error))
            if "y" in required_axes:
                y_pos = controller.read_positions("y")
                y_start_error = float(first["y_angle_deg"]) - float(y_pos["APOS_deg"])
                start_errors.append(f"Y={y_start_error:.3f} deg")
                max_start_error_seen = max(max_start_error_seen, abs(y_start_error))
            start_error_text = ", start errors " + " ".join(start_errors)
            if max_start_error_seen > max_start_error:
                raise RuntimeError(
                    f"Start point is too far from current APOS{start_error_text}. "
                    f"Limit is {max_start_error:.3f} deg. Preposition closer first."
                )
        rise = next_rise_utc(sat, float(lat_var.get()), float(lon_var.get()))
        wait_s = (start_utc - datetime.now(timezone.utc)).total_seconds()
        summary = (
            f"{sat.name}: {duration_s:g}s/{point_count} points at {sample_period_s:g}s, lead-in {lead_in_points} pts, "
            f"first X={float(first['x_angle_deg']):.3f} Y={float(first['y_angle_deg']):.3f}, "
            f"last X={float(last['x_angle_deg']):.3f} Y={float(last['y_angle_deg']):.3f}, "
            f"visible {sum(1 for sample in samples if sample['visible'])}/{len(samples)}, "
            f"next rise {format_local_datetime(rise)}, pickup {format_local_datetime(pickup_utc)}, "
            f"first visible {format_local_datetime(pickup_start_utc)}, "
            f"PVT start {format_local_datetime(start_utc)}, end {format_local_datetime(end_utc)} "
            f"({wait_s:.1f}s){start_error_text}"
        )
        return summary, samples, duration_s, point_count, start_utc

    def preview_satellite_pvt() -> None:
        try:
            summary, samples, _duration_s, _point_count, _start_utc = summarize_plan(check_start=False)
            reference_path = write_satellite_reference_log(selected_satellite().name, samples)
            set_summary(f"{summary}; reference log: {reference_path.name}")
            set_countdown(f"Countdown: start at {format_local_datetime(_start_utc)}")
            log(f"Satellite PVT preview OK: {summary}; reference={reference_path}")
        except Exception as exc:
            set_summary(f"Preview failed: {exc}")
            set_countdown("Countdown: -")
            log(f"Satellite PVT preview FAILED: {exc}")

    def run_satellite_pvt() -> None:
        def action():
            stop_track_requested["value"] = False
            satellite_pvt_active["value"] = True
            error_plot.reset()

            def record_live_error(sample: dict[str, float | str]) -> None:
                window.after(
                    0,
                    lambda axis=str(sample["axis"]),
                    t_s=float(sample["t_s"]),
                    error=float(sample["APOS_minus_target_deg"]): error_plot.add_sample(axis, t_s, error),
                )

            try:
                summary, samples, duration_s, _point_count, start_utc = summarize_plan(check_start=False)
                first = samples[0]
                selected_axes = satellite_axes_var.get()
                required_axes = selected_axis_tuple()
                missing_axes = [axis for axis in required_axes if axis not in connected_axes]
                if missing_axes:
                    raise RuntimeError(f"Connect {', '.join(axis.upper() for axis in missing_axes)} before running satellite PVT")
                if selected_axes in ("x", "both"):
                    controller.move_absolute_deg("x", float(first["x_angle_deg"]))
                if selected_axes in ("y", "both"):
                    controller.move_absolute_deg("y", float(first["y_angle_deg"]))
                set_summary(f"Preposition target sent; waiting for PVT start: {summary}")
                wait_s = (start_utc - datetime.now(timezone.utc)).total_seconds()
                if wait_s > 0:
                    log(f"Satellite PVT waiting {wait_s:.2f}s for pickup/start time")
                    while wait_s > 0:
                        if stop_track_requested["value"]:
                            raise RuntimeError("Satellite PVT stopped before stream start")
                        set_countdown(
                            f"Countdown: {wait_s:.1f} s to PVT start at {format_local_datetime(start_utc)}"
                        )
                        time.sleep(min(0.2, wait_s))
                        wait_s = (start_utc - datetime.now(timezone.utc)).total_seconds()
                    set_countdown("Countdown: starting PVT stream now")
                else:
                    set_countdown("Countdown: pickup time reached; starting PVT stream now")
                if stop_track_requested["value"]:
                    raise RuntimeError("Satellite PVT stopped before stream start")
                reference_path = write_satellite_reference_log(selected_satellite().name, samples)
                x_points = [(float(sample["x_angle_deg"]), float(sample["x_vel_deg_s"])) for sample in samples[1:]]
                y_points = [(float(sample["y_angle_deg"]), float(sample["y_vel_deg_s"])) for sample in samples[1:]]
                metadata = {
                    "trajectory": "satellite",
                    "satellite": selected_satellite().name,
                    "axes": satellite_axes_var.get(),
                    "reference_log": str(reference_path),
                }
                pvt_velocity_correction = {
                    "enable": int(float(pvt_vcorr_enable_var.get())),
                    "kp": float(pvt_vcorr_kp_var.get()),
                    "max_deg_s": float(pvt_vcorr_max_var.get()),
                    "alpha": float(pvt_vcorr_alpha_var.get()),
                }
                if satellite_axes_var.get() == "both":
                    result = controller.run_streaming_pvt_xy_deg(
                        x_points,
                        y_points,
                        duration_s,
                        metadata=metadata,
                    )
                elif satellite_axes_var.get() == "x":
                    result = controller.run_streaming_pvt_axis_deg(
                        "x",
                        x_points,
                        duration_s,
                        metadata=metadata,
                        stop_requested=lambda: stop_track_requested["value"],
                        ui_pump=window.update,
                        sample_callback=record_live_error,
                        pvt_velocity_correction=pvt_velocity_correction,
                    )
                elif satellite_axes_var.get() == "y":
                    result = controller.run_streaming_pvt_axis_deg(
                        "y",
                        y_points,
                        duration_s,
                        metadata=metadata,
                        stop_requested=lambda: stop_track_requested["value"],
                        ui_pump=window.update,
                        sample_callback=record_live_error,
                        pvt_velocity_correction=pvt_velocity_correction,
                    )
                else:
                    raise RuntimeError("Select axes as x, y, or both")
                set_summary(f"{summary}; run log: {Path(str(result['log_path'])).name}")
                set_countdown("Countdown: PVT stream complete; sending selected axis home")
                home_selected_axes_after_success()
                set_countdown("Countdown: PVT stream complete; selected axis sent home")
                return result
            except Exception:
                set_countdown("Countdown: PVT failed/stopped; sending selected axis home")
                terminate_selected_axes(home=True)
                raise
            finally:
                satellite_pvt_active["value"] = False

        run_worker("Satellite PVT", action)

    def stop_satellite_pvt() -> None:
        stop_track_requested["value"] = True
        if satellite_pvt_active["value"]:
            set_countdown("Countdown: stop requested; selected axis will go home")
            return

        def action():
            terminate_selected_axes(home=True)
            set_countdown("Countdown: stopped; selected axis sent home")

        run_worker("Satellite stop/home", action)

    button_row = ttk.Frame(frame)
    button_row.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(8, 4))
    ttk.Button(button_row, text="Preview / Write Reference", command=preview_satellite_pvt).pack(side="left", padx=(0, 8))
    ttk.Button(button_row, text="Run Satellite PVT", command=run_satellite_pvt).pack(side="left")
    ttk.Button(button_row, text="Stop Track / Home", command=stop_satellite_pvt).pack(side="left", padx=(8, 0))

    ttk.Label(frame, textvariable=summary_var, wraplength=650).grid(
        row=6, column=0, columnspan=4, sticky="w", padx=5, pady=(10, 0)
    )
    ttk.Label(frame, textvariable=countdown_var, wraplength=650).grid(
        row=7, column=0, columnspan=4, sticky="w", padx=5, pady=(6, 0)
    )
    error_plot.grid(row=8, column=0, columnspan=4, sticky="nsew", padx=5, pady=(8, 0))
    ttk.Label(
        frame,
        text=(
            "Safety rule: Run refuses to start unless current X/Y APOS is within Max start err deg "
            "of the selected-axis pickup point after the automatic preposition move. "
            "Satellite runs use Sample dt s to generate the full valid pass, not a fixed debug point count."
        ),
        wraplength=650,
    ).grid(row=9, column=0, columnspan=4, sticky="w", padx=5, pady=(10, 0))

    if satellites:
        tle_label_var.set(satellite_tle_path.name if satellite_tle_path else "TLE already loaded")
        refresh_satellite_list()


root = tk.Tk()
root.title("Technosoft Ex04 BasicMove Python Port")
root.geometry("1180x760")

style = ttk.Style(root)
style.configure("Axis.TLabelframe", background="#cfcfcf")
style.configure("Axis.TLabelframe.Label", background="#cfcfcf")
style.configure("AxisInner.TLabelframe", background="#d7eeee")
style.configure("AxisInner.TLabelframe.Label", background="#d7eeee")

main = ttk.Frame(root, padding=8)
main.pack(fill="both", expand=True)
main.columnconfigure(1, weight=1)
main.columnconfigure(3, weight=1)

channel_var = tk.StringVar(value=DEFAULT_CHANNEL_NAME)
channel_type_var = tk.StringVar(value=str(CHANNEL_PEAK_SYS_PCAN_USB))
host_id_var = tk.StringVar(value=str(DEFAULT_HOST_ID))
baudrate_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
speed_deg_s_var = tk.StringVar(value="1.0")
accel_deg_s2_var = tk.StringVar(value="2.0")
decel_deg_s2_var = tk.StringVar(value="2.0")

comm_frame = ttk.LabelFrame(main, text="Communication", padding=8)
comm_frame.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 6))
ttk.Label(comm_frame, text="Channel").grid(row=0, column=0, sticky="w")
ttk.Entry(comm_frame, textvariable=channel_var, width=10).grid(row=0, column=1, sticky="w", padx=(2, 10))
ttk.Label(comm_frame, text="Type").grid(row=0, column=2, sticky="e")
ttk.Entry(comm_frame, textvariable=channel_type_var, width=10).grid(row=0, column=3, sticky="w", padx=(2, 10))
ttk.Label(comm_frame, text="Host").grid(row=0, column=4, sticky="e")
ttk.Entry(comm_frame, textvariable=host_id_var, width=10).grid(row=0, column=5, sticky="w", padx=(2, 10))
ttk.Label(comm_frame, text="Baud").grid(row=0, column=6, sticky="e")
ttk.Entry(comm_frame, textvariable=baudrate_var, width=12).grid(row=0, column=7, sticky="w", padx=(2, 10))
ttk.Button(comm_frame, text="Close Channel", command=close_channel).grid(row=0, column=8, sticky="ew", padx=4)

motion_param_frame = ttk.LabelFrame(main, text="Position Move Profile", padding=8)
motion_param_frame.grid(row=0, column=4, columnspan=4, sticky="ew", pady=(0, 6), padx=(8, 0))
ttk.Label(motion_param_frame, text="Speed deg/s").grid(row=0, column=0, sticky="w")
ttk.Entry(motion_param_frame, textvariable=speed_deg_s_var, width=10).grid(row=0, column=1, sticky="w", padx=(2, 10))
ttk.Label(motion_param_frame, text="Accel deg/s2").grid(row=0, column=2, sticky="e")
ttk.Entry(motion_param_frame, textvariable=accel_deg_s2_var, width=10).grid(row=0, column=3, sticky="w", padx=(2, 10))
ttk.Label(motion_param_frame, text="Decel deg/s2").grid(row=0, column=4, sticky="e")
ttk.Entry(motion_param_frame, textvariable=decel_deg_s2_var, width=10).grid(row=0, column=5, sticky="w", padx=(2, 10))
ttk.Button(motion_param_frame, text="Apply Profile", command=set_motion_params).grid(row=0, column=6, sticky="ew", padx=4)

axis_vars: dict[str, dict[str, tk.StringVar]] = {}
pos_vars: dict[str, tk.StringVar] = {}
status_vars: dict[str, tk.StringVar] = {}
gain_vars: dict[str, dict[str, tk.StringVar]] = {}


def build_axis_frame(axis: str, row: int, default_node_id: int) -> None:
    frame = ttk.LabelFrame(main, text=f"{axis.upper()} Axis", padding=8, style="Axis.TLabelframe")
    frame.grid(row=row, column=0, columnspan=8, sticky="nsew", pady=8)
    frame.columnconfigure(1, weight=1)
    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(1, weight=1)

    axis_vars[axis] = {
        "node_id": tk.StringVar(value=str(default_node_id)),
        "setup": tk.StringVar(value=str(DEFAULT_SETUP_PATH)),
        "sign": tk.StringVar(value="1"),
        "abs_target": tk.StringVar(value="0"),
        "pvt_target": tk.StringVar(value="2"),
        "pvt_duration": tk.StringVar(value="4"),
        "pvt_stream_points": tk.StringVar(value="20"),
        "pvt_profile": tk.StringVar(value="smoothstep"),
        "pvt_sine_center": tk.StringVar(value="10"),
        "pvt_sine_amplitude": tk.StringVar(value="5"),
        "pvt_sine_period": tk.StringVar(value="10"),
        "pvt_sine_duration": tk.StringVar(value="20"),
        "pvt_sine_points": tk.StringVar(value="100"),
    }
    pos_vars[axis] = tk.StringVar(value="not connected")
    status_vars[axis] = tk.StringVar(value="")
    gain_vars[axis] = {name: tk.StringVar(value="100" if name == "KPP" else "") for name in GAIN_NAMES}

    startup_frame = ttk.LabelFrame(frame, text="Startup", padding=6, style="AxisInner.TLabelframe")
    startup_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
    startup_frame.columnconfigure(1, weight=1)
    ttk.Label(startup_frame, text="Node ID").grid(row=0, column=0, sticky="w")
    ttk.Entry(startup_frame, textvariable=axis_vars[axis]["node_id"], width=8).grid(row=0, column=1, sticky="w", padx=(4, 12))
    ttk.Label(startup_frame, text="Output sign").grid(row=0, column=2, sticky="e")
    ttk.Entry(startup_frame, textvariable=axis_vars[axis]["sign"], width=8).grid(row=0, column=3, sticky="w", padx=(4, 12))
    ttk.Button(startup_frame, text="Vendor Init", command=lambda a=axis: vendor_init_axis(a)).grid(row=0, column=4, padx=4)
    ttk.Label(startup_frame, text=".t.zip").grid(row=1, column=0, sticky="w", pady=(6, 0))
    ttk.Entry(startup_frame, textvariable=axis_vars[axis]["setup"], width=80).grid(row=1, column=1, columnspan=4, sticky="ew", pady=(6, 0))

    feedback_frame = ttk.LabelFrame(frame, text="Live Feedback", padding=6, style="AxisInner.TLabelframe")
    feedback_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 6))
    feedback_frame.columnconfigure(1, weight=1)
    ttk.Label(feedback_frame, text="Position").grid(row=0, column=0, sticky="w")
    ttk.Label(feedback_frame, textvariable=pos_vars[axis]).grid(row=0, column=1, sticky="w")
    ttk.Label(feedback_frame, text="Status").grid(row=1, column=0, sticky="w", pady=(6, 0))
    ttk.Label(feedback_frame, textvariable=status_vars[axis]).grid(row=1, column=1, sticky="w", pady=(6, 0))

    motion_frame = ttk.LabelFrame(frame, text="Position Moves", padding=6, style="AxisInner.TLabelframe")
    motion_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=6)
    ttk.Button(motion_frame, text="-1 deg", command=lambda a=axis: move_relative(a, -1.0)).grid(row=0, column=0, padx=3, pady=3)
    ttk.Button(motion_frame, text="-0.2 deg", command=lambda a=axis: move_relative(a, -0.2)).grid(row=0, column=1, padx=3, pady=3)
    ttk.Button(motion_frame, text="+0.2 deg", command=lambda a=axis: move_relative(a, 0.2)).grid(row=0, column=2, padx=3, pady=3)
    ttk.Button(motion_frame, text="+1 deg", command=lambda a=axis: move_relative(a, 1.0)).grid(row=0, column=3, padx=3, pady=3)
    ttk.Button(motion_frame, text="Home 0", command=lambda a=axis: go_home(a)).grid(row=0, column=4, padx=8, pady=3)
    ttk.Label(motion_frame, text="Abs deg").grid(row=1, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(motion_frame, textvariable=axis_vars[axis]["abs_target"], width=10).grid(row=1, column=1, sticky="w", pady=(6, 0))
    ttk.Button(motion_frame, text="Move Abs", command=lambda a=axis: move_absolute(a)).grid(row=1, column=2, padx=3, pady=(6, 0))

    gains_frame = ttk.LabelFrame(frame, text="Controller Gains", padding=6, style="AxisInner.TLabelframe")
    gains_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=6)
    for col, name in enumerate(GAIN_NAMES):
        ttk.Label(gains_frame, text=name).grid(row=0, column=col, sticky="w")
        ttk.Entry(gains_frame, textvariable=gain_vars[axis][name], width=10).grid(row=1, column=col, sticky="w", padx=(0, 6))
    ttk.Button(gains_frame, text="Read Gains", command=lambda a=axis: read_gains(a)).grid(row=1, column=5, padx=4)
    ttk.Button(gains_frame, text="Apply Gains", command=lambda a=axis: apply_gains(a)).grid(row=1, column=6, padx=4)

    safety_frame = ttk.LabelFrame(frame, text="Safety / Utilities", padding=6, style="AxisInner.TLabelframe")
    safety_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
    safety_frame.columnconfigure(0, weight=1)
    safety_frame.columnconfigure(1, weight=1)

    safety_left = ttk.Frame(safety_frame)
    safety_left.grid(row=0, column=0, sticky="w")
    ttk.Button(safety_left, text="Stop", command=lambda a=axis: axis_command(a, "Stop", lambda: controller.stop(a))).grid(row=0, column=0, padx=3, pady=3)
    ttk.Button(safety_left, text="Power Off", command=lambda a=axis: axis_command(a, "Power Off", lambda: controller.power(a, False))).grid(row=0, column=1, padx=3, pady=3)
    ttk.Button(safety_left, text="Reset Fault", command=lambda a=axis: axis_command(a, "Reset Fault", lambda: controller.reset_fault(a))).grid(row=0, column=2, padx=3, pady=3)
    ttk.Button(safety_left, text="Wait Motion Done", command=lambda a=axis: axis_command(a, "Wait Motion Done", lambda: controller.wait_motion_complete(a))).grid(row=0, column=3, padx=3, pady=3)
    ttk.Button(safety_left, text="Set Vertical Zero", command=lambda a=axis: set_vertical_zero(a)).grid(row=0, column=4, padx=3, pady=3)

    safety_right = ttk.Frame(safety_frame)
    safety_right.grid(row=0, column=1, sticky="e")
    ttk.Button(safety_right, text="PVT", command=lambda a=axis: open_pvt_window(a)).grid(row=0, column=0, padx=3, pady=3)


build_axis_frame("x", 2, 1)
build_axis_frame("y", 3, 2)

refresh_frame = ttk.Frame(main)
refresh_frame.grid(row=4, column=0, columnspan=8, sticky="ew")
ttk.Button(refresh_frame, text="Refresh", command=refresh_once).pack(side="left", padx=4)
ttk.Button(refresh_frame, text="Satellite PVT", command=open_satellite_pvt_window).pack(side="left", padx=4)

log_text = tk.Text(main, height=10, wrap="none")
log_text.grid(row=5, column=0, columnspan=8, sticky="nsew", pady=(8, 0))
main.rowconfigure(5, weight=1)

refresh_loop()
root.mainloop()
