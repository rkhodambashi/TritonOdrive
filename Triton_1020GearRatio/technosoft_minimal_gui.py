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


def connect_both() -> None:
    connect_axis("x")
    connect_axis("y")


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


root = tk.Tk()
root.title("Technosoft Ex04 BasicMove Python Port")
root.geometry("1180x760")

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

ttk.Label(main, text="Channel").grid(row=0, column=0, sticky="w")
ttk.Entry(main, textvariable=channel_var, width=10).grid(row=0, column=1, sticky="w")
ttk.Label(main, text="Type").grid(row=0, column=2, sticky="e")
ttk.Entry(main, textvariable=channel_type_var, width=10).grid(row=0, column=3, sticky="w")
ttk.Label(main, text="Host").grid(row=0, column=4, sticky="e")
ttk.Entry(main, textvariable=host_id_var, width=10).grid(row=0, column=5, sticky="w")
ttk.Label(main, text="Baud").grid(row=0, column=6, sticky="e")
ttk.Entry(main, textvariable=baudrate_var, width=12).grid(row=0, column=7, sticky="w")

ttk.Label(main, text="Speed deg/s").grid(row=1, column=0, sticky="w")
ttk.Entry(main, textvariable=speed_deg_s_var, width=10).grid(row=1, column=1, sticky="w")
ttk.Label(main, text="Accel deg/s2").grid(row=1, column=2, sticky="e")
ttk.Entry(main, textvariable=accel_deg_s2_var, width=10).grid(row=1, column=3, sticky="w")
ttk.Label(main, text="Decel deg/s2").grid(row=1, column=4, sticky="e")
ttk.Entry(main, textvariable=decel_deg_s2_var, width=10).grid(row=1, column=5, sticky="w")
ttk.Button(main, text="Set Motion Params", command=set_motion_params).grid(row=1, column=6, sticky="ew", padx=4)
ttk.Button(main, text="Connect Both", command=connect_both).grid(row=1, column=7, sticky="ew", padx=4)
ttk.Button(main, text="Close Channel", command=close_channel).grid(row=1, column=8, sticky="ew", padx=4)

axis_vars: dict[str, dict[str, tk.StringVar]] = {}
pos_vars: dict[str, tk.StringVar] = {}
status_vars: dict[str, tk.StringVar] = {}
gain_vars: dict[str, dict[str, tk.StringVar]] = {}


def build_axis_frame(axis: str, row: int, default_node_id: int) -> None:
    frame = ttk.LabelFrame(main, text=f"{axis.upper()} Axis", padding=8)
    frame.grid(row=row, column=0, columnspan=8, sticky="nsew", pady=8)
    frame.columnconfigure(1, weight=1)
    frame.columnconfigure(3, weight=1)

    axis_vars[axis] = {
        "node_id": tk.StringVar(value=str(default_node_id)),
        "setup": tk.StringVar(value=str(DEFAULT_SETUP_PATH)),
        "sign": tk.StringVar(value="1"),
        "abs_target": tk.StringVar(value="0"),
    }
    pos_vars[axis] = tk.StringVar(value="not connected")
    status_vars[axis] = tk.StringVar(value="")
    gain_vars[axis] = {name: tk.StringVar(value="") for name in GAIN_NAMES}

    ttk.Label(frame, text="Node ID").grid(row=0, column=0, sticky="w")
    ttk.Entry(frame, textvariable=axis_vars[axis]["node_id"], width=8).grid(row=0, column=1, sticky="w")
    ttk.Label(frame, text="Output sign").grid(row=0, column=2, sticky="e")
    ttk.Entry(frame, textvariable=axis_vars[axis]["sign"], width=8).grid(row=0, column=3, sticky="w")
    ttk.Button(frame, text="Setup Only", command=lambda a=axis: connect_axis(a)).grid(row=0, column=4, padx=4)
    ttk.Button(frame, text="Vendor Init", command=lambda a=axis: vendor_init_axis(a)).grid(row=0, column=5, padx=4)

    ttk.Label(frame, text=".t.zip").grid(row=1, column=0, sticky="w")
    ttk.Entry(frame, textvariable=axis_vars[axis]["setup"], width=100).grid(row=1, column=1, columnspan=6, sticky="ew")

    ttk.Label(frame, text="Position").grid(row=2, column=0, sticky="w")
    ttk.Label(frame, textvariable=pos_vars[axis]).grid(row=2, column=1, columnspan=6, sticky="w")
    ttk.Label(frame, text="Status").grid(row=3, column=0, sticky="w")
    ttk.Label(frame, textvariable=status_vars[axis]).grid(row=3, column=1, columnspan=6, sticky="w")

    for col, name in enumerate(GAIN_NAMES):
        ttk.Label(frame, text=name).grid(row=4, column=col, sticky="w")
        ttk.Entry(frame, textvariable=gain_vars[axis][name], width=10).grid(row=5, column=col, sticky="w")
    ttk.Button(frame, text="Read Gains", command=lambda a=axis: read_gains(a)).grid(row=5, column=5, padx=4)
    ttk.Button(frame, text="Apply Gains", command=lambda a=axis: apply_gains(a)).grid(row=5, column=6, padx=4)

    ttk.Button(frame, text="DriveInit", command=lambda a=axis: axis_command(a, "DriveInit", lambda: controller.drive_initialisation(a))).grid(row=6, column=0, padx=3, pady=4)
    ttk.Button(frame, text="Power On", command=lambda a=axis: axis_command(a, "Power On", lambda: controller.power(a, True))).grid(row=6, column=1, padx=3, pady=4)
    ttk.Button(frame, text="Power Off", command=lambda a=axis: axis_command(a, "Power Off", lambda: controller.power(a, False))).grid(row=6, column=2, padx=3, pady=4)
    ttk.Button(frame, text="Stop", command=lambda a=axis: axis_command(a, "Stop", lambda: controller.stop(a))).grid(row=6, column=3, padx=3, pady=4)
    ttk.Button(frame, text="Reset Fault", command=lambda a=axis: axis_command(a, "Reset Fault", lambda: controller.reset_fault(a))).grid(row=6, column=4, padx=3, pady=4)
    ttk.Button(frame, text="Wait Motion Done", command=lambda a=axis: axis_command(a, "Wait Motion Done", lambda: controller.wait_motion_complete(a))).grid(row=6, column=5, columnspan=2, padx=3, pady=4)
    ttk.Button(frame, text="Set Vertical Zero", command=lambda a=axis: set_vertical_zero(a)).grid(row=6, column=7, padx=3, pady=4)

    ttk.Button(frame, text="-1 deg", command=lambda a=axis: move_relative(a, -1.0)).grid(row=7, column=0, padx=3, pady=4)
    ttk.Button(frame, text="-0.2 deg", command=lambda a=axis: move_relative(a, -0.2)).grid(row=7, column=1, padx=3, pady=4)
    ttk.Button(frame, text="+0.2 deg", command=lambda a=axis: move_relative(a, 0.2)).grid(row=7, column=2, padx=3, pady=4)
    ttk.Button(frame, text="+1 deg", command=lambda a=axis: move_relative(a, 1.0)).grid(row=7, column=3, padx=3, pady=4)
    ttk.Button(frame, text="Home 0", command=lambda a=axis: go_home(a)).grid(row=7, column=4, padx=3, pady=4)
    ttk.Label(frame, text="Abs deg").grid(row=7, column=5, sticky="e")
    ttk.Entry(frame, textvariable=axis_vars[axis]["abs_target"], width=10).grid(row=7, column=6, sticky="w")
    ttk.Button(frame, text="Move Abs", command=lambda a=axis: move_absolute(a)).grid(row=7, column=7, padx=3, pady=4)


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
