"""Minimal GUI for direct Technosoft iPOS8020 bring-up."""

from __future__ import annotations

import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import ttk

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


controller = TechnosoftEx04()
connected_axes: set[str] = set()
refresh_after_id = None


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


def move_relative(axis: str, delta_deg: float) -> None:
    axis_command(axis, f"move relative {delta_deg:+g} deg", lambda: controller.move_relative_deg(axis, delta_deg))


def move_absolute(axis: str) -> None:
    target = float(axis_vars[axis]["abs_target"].get())
    axis_command(axis, f"move absolute {target:g} deg", lambda: controller.move_absolute_deg(axis, target))


def go_home(axis: str) -> None:
    axis_command(axis, "home / vertical 0", lambda: controller.move_absolute_deg(axis, 0.0))


def set_vertical_zero(axis: str) -> None:
    def action():
        result = controller.current_vertical_zero_hardcode(axis)
        return f"vertical zero now: {result['code_line']}"

    axis_command(axis, "set vertical zero", action)


def run_pvt(axis: str) -> None:
    def action():
        return controller.run_pvt_line_deg(
            axis,
            float(axis_vars[axis]["pvt_target"].get()),
            float(axis_vars[axis]["pvt_duration"].get()),
            int(float(axis_vars[axis]["pvt_points"].get())),
        )

    axis_command(axis, "PVT line", action)


def open_pvt_window(axis: str) -> None:
    window = tk.Toplevel(root)
    window.title(f"{axis.upper()} Axis PVT")
    window.geometry("420x220")
    window.transient(root)

    frame = ttk.Frame(window, padding=12)
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, text=f"{axis.upper()} axis absolute PVT line move").grid(row=0, column=0, columnspan=2, sticky="w")
    ttk.Label(frame, text="Target deg").grid(row=1, column=0, sticky="e", pady=(12, 0))
    ttk.Entry(frame, textvariable=axis_vars[axis]["pvt_target"], width=12).grid(row=1, column=1, sticky="w", padx=6, pady=(12, 0))
    ttk.Label(frame, text="Duration s").grid(row=2, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(frame, textvariable=axis_vars[axis]["pvt_duration"], width=12).grid(row=2, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Label(frame, text="Point count").grid(row=3, column=0, sticky="e", pady=(6, 0))
    ttk.Entry(frame, textvariable=axis_vars[axis]["pvt_points"], width=12).grid(row=3, column=1, sticky="w", padx=6, pady=(6, 0))
    ttk.Button(frame, text="Run PVT", command=lambda a=axis: run_pvt(a)).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(14, 0))
    ttk.Label(
        frame,
        text="First test suggestion: target 2 deg, duration 4 s, point count 8.",
        wraplength=360,
    ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(10, 0))


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
        "pvt_points": tk.StringVar(value="8"),
    }
    pos_vars[axis] = tk.StringVar(value="not connected")
    status_vars[axis] = tk.StringVar(value="")
    gain_vars[axis] = {name: tk.StringVar(value="") for name in GAIN_NAMES}

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

log_text = tk.Text(main, height=10, wrap="none")
log_text.grid(row=5, column=0, columnspan=8, sticky="nsew", pady=(8, 0))
main.rowconfigure(5, weight=1)

refresh_loop()
root.mainloop()
