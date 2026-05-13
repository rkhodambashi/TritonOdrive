"""Minimal Technosoft iPOS8020 BX-CAN GUI.

This is intentionally smaller than odrive_gui.py.  It is for safe bring-up:
connect, read positions, small moves, stop/disarm, recovery, and PVT smoke tests.
"""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk
from datetime import datetime
from pathlib import Path

import Technosoft8020_ACL60250_Motors_PositionInput as control


position_loop_after_id = None
axis_devices = {"x": None, "y": None}
DIAGNOSTIC_LOG_DIR = Path(__file__).resolve().parent / "technosoft_diagnostics"


def _safe_call(func, fallback=None):
    try:
        return func()
    except Exception:
        return fallback


def connect_drives():
    try:
        if use_sim_var.get():
            control.use_simulated_backend()
        else:
            control.set_axis_node_ids(
                x_node_id=int(x_node_id_var.get()),
                y_node_id=int(y_node_id_var.get()),
            )
            control.use_tml_backend(
                channel_name=channel_name_var.get().strip() or "1",
                channel_type=int(channel_type_var.get()),
                host_id=int(host_id_var.get()),
                baudrate=int(baudrate_var.get()),
                x_setup_path=x_setup_var.get().strip() or None,
                y_setup_path=y_setup_var.get().strip() or None,
                auto_power_on=bool(auto_power_var.get()),
                run_drive_initialisation=bool(run_init_var.get()),
                command_uses_load_scaling=bool(command_uses_load_scaling_var.get()),
            )
        devices = control.connect_all()
        axis_devices.update(devices)
        status_var.set(
            "Connected Technosoft iPOS8020 axes\n"
            f"X: {control.get_serial_number('x')}\n"
            f"Y: {control.get_serial_number('y')}"
        )
        start_position_loop()
    except Exception as exc:
        status_var.set(f"Connection failed: {exc}")


def start_position_loop():
    global position_loop_after_id
    if position_loop_after_id is None:
        update_position_loop()


def update_position_loop():
    global position_loop_after_id
    for axis, pos_var, vel_var, state_var in (
        ("x", x_pos_var, x_vel_var, x_state_var),
        ("y", y_pos_var, y_vel_var, y_state_var),
    ):
        pos = _safe_call(lambda a=axis: control.get_current_position(a))
        motor_raw = _safe_call(lambda a=axis: control.get_motor_raw(a))
        command_raw = _safe_call(lambda a=axis: control.get_command_raw(a))
        target_raw = _safe_call(lambda a=axis: control.get_target_raw(a))
        vel = _safe_call(lambda a=axis: control.get_motor_velocity(a))
        faults = _safe_call(lambda a=axis: control.get_active_errors(a))
        fault_registers = _safe_call(lambda a=axis: control.get_fault_registers(a))
        status_registers = _safe_call(lambda a=axis: control.get_status_registers(a))
        mode = _safe_call(lambda a=axis: control.get_control_mode(a))
        input_mode = _safe_call(lambda a=axis: control.get_input_mode(a))
        buffer_level = _safe_call(lambda a=axis: control.pvt_buffer_level(a))

        if motor_raw is not None or command_raw is not None or target_raw is not None:
            pos_text = f"{pos:.6f} deg" if pos is not None else "-"
            motor_text = f"APOS {motor_raw:.0f}" if motor_raw is not None else "APOS -"
            command_text = f"CPOS {command_raw:.0f}" if command_raw is not None else "CPOS -"
            target_text = f"TPOS {target_raw:.0f}" if target_raw is not None else "TPOS -"
            pos_var.set(f"{pos_text} | {motor_text} | {command_text} | {target_text}")
        else:
            pos_var.set(f"{pos:.6f} deg" if pos is not None else "-")
        vel_var.set(f"{vel:.6f} deg/s" if vel is not None else "-")
        if fault_registers:
            fault_text = " ".join(f"{name}=0x{value:04X}" for name, value in fault_registers.items())
        else:
            fault_text = f"faults={faults if faults is not None else '-'}"
        if status_registers:
            status_text = " ".join(f"{name}=0x{value:04X}" for name, value in status_registers.items())
        else:
            status_text = ""
        state_var.set(
            f"{fault_text} {status_text} "
            f"mode={mode if mode is not None else '-'} "
            f"input={input_mode if input_mode is not None else '-'} "
            f"pvt={buffer_level if buffer_level is not None else '-'}"
        )

    last_update_var.set(f"Last refresh: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
    position_loop_after_id = root.after(100, update_position_loop)


def refresh_now():
    global position_loop_after_id
    if position_loop_after_id is not None:
        root.after_cancel(position_loop_after_id)
        position_loop_after_id = None
    update_position_loop()


def run_axis_command(label, func, kwargs, axis):
    def worker():
        try:
            func(**kwargs)
            root.after(0, lambda: status_var.set(f"{label} complete"))
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{label} failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def move_relative(axis, delta):
    run_axis_command(
        f"Move {axis.upper()} {delta:+g} deg",
        control.move_relative,
        {"axis": axis, "delta_deg": delta},
        axis,
    )


def move_absolute(axis, entry):
    try:
        target = float(entry.get())
    except Exception:
        status_var.set("Absolute target is invalid")
        return
    run_axis_command(
        f"Move {axis.upper()} to {target:g} deg",
        control.move_absolute,
        {"axis": axis, "target_output_deg": target},
        axis,
    )


def home_axis(axis):
    run_axis_command(
        f"Home {axis.upper()}",
        control.go_home,
        {"axis": axis},
        axis,
    )


def recover_axis(axis):
    run_axis_command(
        f"Recover {axis.upper()}",
        control.recover_axis_to_safe_range,
        {"axis": axis},
        axis,
    )


def reset_drive(axis):
    def worker():
        try:
            control.reset_drive(axis)
            root.after(
                0,
                lambda: status_var.set(
                    f"{axis.upper()} drive reset command sent. Wait for the drive to reboot, then reconnect."
                ),
            )
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{axis.upper()} drive reset failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def stop_axis(axis):
    try:
        control.stop_axis(axis)
        status_var.set(f"{axis.upper()} stopped")
    except Exception as exc:
        status_var.set(f"{axis.upper()} stop failed: {exc}")


def power_axis(axis, enable):
    label = f"{axis.upper()} power {'on' if enable else 'off'}"
    run_axis_command(
        label,
        control.power_axis,
        {"axis": axis, "enable": enable},
        axis,
    )


def align_reference(axis):
    def worker():
        try:
            result = control.align_reference_to_actual_report(axis)
            message = (
                f"Align {axis.upper()} reference complete "
                f"before [{_format_registers(result['before'])}] "
                f"after [{_format_registers(result['after'])}]"
            )
            root.after(0, lambda msg=message: status_var.set(msg))
            root.after(0, refresh_now)
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"Align {axis.upper()} reference failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def set_vertical_zero(axis):
    def worker():
        try:
            drive_zero = control.set_vertical_zero_here(axis)
            hardcode = control.get_vertical_zero_hardcode(axis)
            message = (
                f"{axis.upper()} vertical calibration read here. "
                f"Current displayed position is {drive_zero:.6f} deg. "
                f"To hard-code this home: {hardcode['code_line']}"
            )
            root.after(0, lambda msg=message: status_var.set(msg))
            root.after(0, refresh_now)
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{axis.upper()} set vertical zero failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def read_drive_gains(axis):
    def worker():
        try:
            gains = control.get_drive_gains(axis)
            root.after(0, lambda: status_var.set(f"{axis.upper()} gains: {_format_registers(gains)}"))
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{axis.upper()} read gains failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def scale_drive_gains(axis):
    try:
        scale = float(gain_scale_var.get())
    except Exception:
        status_var.set("Gain scale is invalid")
        return

    def worker():
        try:
            result = control.scale_drive_gains(axis, scale=scale)
            message = (
                f"{axis.upper()} gains scaled by {scale:g} in RAM only "
                f"before [{_format_registers(result['before'])}] "
                f"after [{_format_registers(result['after'])}]"
            )
            root.after(0, lambda msg=message: status_var.set(msg))
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{axis.upper()} scale gains failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def axis_on_command(axis, enable):
    label = f"{axis.upper()} AXIS {'ON' if enable else 'OFF'}"
    run_axis_command(
        label,
        control.axis_on_command,
        {"axis": axis, "enable": enable},
        axis,
    )


def axis_on_update_command(axis, enable):
    label = f"{axis.upper()} AXIS {'ON' if enable else 'OFF'} + update"
    run_axis_command(
        label,
        control.axis_on_update_command,
        {"axis": axis, "enable": enable},
        axis,
    )


def end_initialization(axis):
    run_axis_command(
        f"{axis.upper()} ENDINIT",
        control.end_initialization,
        {"axis": axis},
        axis,
    )


def _format_registers(registers):
    return " ".join(f"{name}=0x{value:04X}" for name, value in registers.items())


def _write_driveinit_report(axis, result):
    DIAGNOSTIC_LOG_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = DIAGNOSTIC_LOG_DIR / f"driveinit_{axis}_{stamp}.txt"
    lines = [
        f"Axis: {axis.upper()}",
        f"Time: {datetime.now().isoformat(timespec='seconds')}",
        f"OK: {result['ok']}",
        f"TML error: {result['error']}",
        f"Before: {_format_registers(result['before'])}",
        f"After:  {_format_registers(result['after'])}",
        "",
        "Decoded after:",
    ]
    for name, items in result["decoded_after"].items():
        if items:
            lines.append(f"{name}:")
            lines.extend(f"  {item}" for item in items)
        else:
            lines.append(f"{name}: no decoded bits")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def drive_initialisation_diagnostic(axis):
    def worker():
        try:
            result = control.drive_initialisation_diagnostic(axis)
            report_path = _write_driveinit_report(axis, result)
            decoded = []
            for name, items in result["decoded_after"].items():
                if items:
                    decoded.append(f"{name}: {', '.join(items)}")
            decoded_text = " | ".join(decoded) if decoded else "no decoded fault bits"
            message = (
                f"{axis.upper()} DriveInitialisation diag "
                f"ok={result['ok']} "
                f"before [{_format_registers(result['before'])}] "
                f"after [{_format_registers(result['after'])}] "
                f"{decoded_text}"
            )
            if result["error"]:
                message += f" | TML error: {result['error']}"
            message += f" | report: {report_path}"
            root.after(0, lambda msg=message: status_var.set(msg))
            root.after(0, refresh_now)
        except Exception as exc:
            error_message = str(exc)
            root.after(0, lambda msg=error_message: status_var.set(f"{axis.upper()} DriveInitialisation diag failed: {msg}"))

    threading.Thread(target=worker, daemon=True).start()


def execute_raw_tml(axis):
    command = raw_tml_var.get().strip()
    if not command:
        status_var.set("Raw TML command is empty")
        return
    run_axis_command(
        f"{axis.upper()} TML {command}",
        control.execute_tml,
        {"axis": axis, "command": command},
        axis,
    )


def pvt_smoke(axis):
    try:
        current = control.get_current_position(axis)
        points = [
            control.PVTPoint(position_deg=current + 0.25, velocity_deg_s=0.5, duration_s=0.25),
            control.PVTPoint(position_deg=current + 0.50, velocity_deg_s=0.0, duration_s=0.25),
        ]
        count = control.queue_pvt_points(points, axis=axis, start=True)
        status_var.set(f"{axis.upper()} queued {count} PVT smoke-test points")
    except Exception as exc:
        status_var.set(f"{axis.upper()} PVT smoke test failed: {exc}")


root = tk.Tk()
root.title("Technosoft iPOS8020 Control GUI")
root.geometry("980x720")

main = ttk.Frame(root)
main.pack(fill="both", expand=True, padx=12, pady=12)

use_sim_var = tk.IntVar(value=0)
auto_power_var = tk.IntVar(value=0)
run_init_var = tk.IntVar(value=1)
command_uses_load_scaling_var = tk.IntVar(value=1)
gain_scale_var = tk.StringVar(value="0.5")
ttk.Checkbutton(main, text="Use simulated backend", variable=use_sim_var).grid(row=0, column=0, sticky="w")
ttk.Button(main, text="Connect iPOS8020", command=connect_drives).grid(row=0, column=1, padx=8, pady=4, sticky="w")
ttk.Button(main, text="Refresh Now", command=refresh_now).grid(row=0, column=2, padx=8, pady=4, sticky="w")

channel_name_var = tk.StringVar(value="1")
channel_type_var = tk.StringVar(value=str(control.CHANNEL_PEAK_SYS_PCAN_USB))
host_id_var = tk.StringVar(value="255")
baudrate_var = tk.StringVar(value="1000000")
x_node_id_var = tk.StringVar(value="1")
y_node_id_var = tk.StringVar(value="1")
x_setup_var = tk.StringVar(value=str(control.DEFAULT_TML_SETUP_PATH))
y_setup_var = tk.StringVar(value=str(control.DEFAULT_TML_SETUP_PATH))

ttk.Label(main, text="Real TML connection").grid(row=1, column=0, sticky="w", pady=(8, 0))
ttk.Label(main, text="CAN device").grid(row=2, column=0, sticky="w")
ttk.Entry(main, textvariable=channel_name_var, width=10).grid(row=2, column=1, sticky="w")
ttk.Label(main, text="Type").grid(row=2, column=2, sticky="e")
ttk.Entry(main, textvariable=channel_type_var, width=8).grid(row=2, column=3, sticky="w")
ttk.Label(main, text="Host").grid(row=3, column=0, sticky="w")
ttk.Entry(main, textvariable=host_id_var, width=10).grid(row=3, column=1, sticky="w")
ttk.Label(main, text="Baud").grid(row=3, column=2, sticky="e")
ttk.Entry(main, textvariable=baudrate_var, width=10).grid(row=3, column=3, sticky="w")
ttk.Checkbutton(main, text="Power on during connect", variable=auto_power_var).grid(row=3, column=4, sticky="w")
ttk.Checkbutton(main, text="Run DriveInitialisation", variable=run_init_var).grid(row=4, column=4, sticky="w")
ttk.Checkbutton(main, text="Use load scaling for commands", variable=command_uses_load_scaling_var).grid(row=2, column=4, sticky="w")
ttk.Label(main, text="X Node").grid(row=4, column=0, sticky="w")
ttk.Entry(main, textvariable=x_node_id_var, width=10).grid(row=4, column=1, sticky="w")
ttk.Label(main, text="Y Node").grid(row=4, column=2, sticky="e")
ttk.Entry(main, textvariable=y_node_id_var, width=10).grid(row=4, column=3, sticky="w")
ttk.Label(main, text="X .t.zip").grid(row=5, column=0, sticky="w")
ttk.Entry(main, textvariable=x_setup_var, width=54).grid(row=5, column=1, columnspan=4, sticky="ew")
ttk.Label(main, text="Y .t.zip").grid(row=6, column=0, sticky="w")
ttk.Entry(main, textvariable=y_setup_var, width=54).grid(row=6, column=1, columnspan=4, sticky="ew")

status_var = tk.StringVar(value="Not connected")
last_update_var = tk.StringVar(value="Last refresh: -")
ttk.Label(main, textvariable=status_var, justify="left").grid(row=7, column=0, columnspan=5, sticky="w", pady=8)
ttk.Label(main, textvariable=last_update_var, justify="left").grid(row=8, column=0, columnspan=5, sticky="w")

ttk.Separator(main).grid(row=9, column=0, columnspan=5, sticky="ew", pady=8)

ttk.Label(main, text="Axis").grid(row=10, column=0, sticky="w")
ttk.Label(main, text="Position").grid(row=10, column=1, sticky="w")
ttk.Label(main, text="Velocity").grid(row=10, column=2, sticky="w")
ttk.Label(main, text="State").grid(row=10, column=3, sticky="w")

x_pos_var = tk.StringVar(value="-")
x_vel_var = tk.StringVar(value="-")
x_state_var = tk.StringVar(value="-")
y_pos_var = tk.StringVar(value="-")
y_vel_var = tk.StringVar(value="-")
y_state_var = tk.StringVar(value="-")

ttk.Label(main, text="X").grid(row=11, column=0, sticky="w")
ttk.Label(main, textvariable=x_pos_var).grid(row=11, column=1, sticky="w")
ttk.Label(main, textvariable=x_vel_var).grid(row=11, column=2, sticky="w")
ttk.Label(main, textvariable=x_state_var).grid(row=11, column=3, sticky="w")

ttk.Label(main, text="Y").grid(row=12, column=0, sticky="w")
ttk.Label(main, textvariable=y_pos_var).grid(row=12, column=1, sticky="w")
ttk.Label(main, textvariable=y_vel_var).grid(row=12, column=2, sticky="w")
ttk.Label(main, textvariable=y_state_var).grid(row=12, column=3, sticky="w")

ttk.Separator(main).grid(row=13, column=0, columnspan=5, sticky="ew", pady=8)

for base_row, axis in ((14, "x"), (21, "y")):
    ttk.Label(main, text=axis.upper()).grid(row=base_row, column=0, sticky="w")
    ttk.Button(main, text="-1", command=lambda a=axis: move_relative(a, -1)).grid(row=base_row, column=1, padx=4, pady=3)
    ttk.Button(main, text="+1", command=lambda a=axis: move_relative(a, 1)).grid(row=base_row, column=2, padx=4, pady=3)
    ttk.Button(main, text="Home", command=lambda a=axis: home_axis(a)).grid(row=base_row, column=3, padx=4, pady=3)
    ttk.Button(main, text="Recover", command=lambda a=axis: recover_axis(a)).grid(row=base_row, column=4, padx=4, pady=3)
    ttk.Button(main, text="-0.02", command=lambda a=axis: move_relative(a, -0.02)).grid(row=base_row + 1, column=1, padx=4, pady=3)
    ttk.Button(main, text="+0.02", command=lambda a=axis: move_relative(a, 0.02)).grid(row=base_row + 1, column=2, padx=4, pady=3)
    ttk.Button(main, text="Stop", command=lambda a=axis: stop_axis(a)).grid(row=base_row + 1, column=3, padx=4, pady=3)
    ttk.Button(main, text="Reset Drive", command=lambda a=axis: reset_drive(a)).grid(row=base_row + 1, column=4, padx=4, pady=3)
    ttk.Button(main, text="-0.2", command=lambda a=axis: move_relative(a, -0.2)).grid(row=base_row + 2, column=1, padx=4, pady=3)
    ttk.Button(main, text="+0.2", command=lambda a=axis: move_relative(a, 0.2)).grid(row=base_row + 2, column=2, padx=4, pady=3)
    ttk.Button(main, text="Power On", command=lambda a=axis: power_axis(a, True)).grid(row=base_row + 2, column=3, padx=4, pady=3)
    ttk.Button(main, text="Power Off", command=lambda a=axis: power_axis(a, False)).grid(row=base_row + 2, column=4, padx=4, pady=3)
    ttk.Button(main, text="ENDINIT", command=lambda a=axis: end_initialization(a)).grid(row=base_row + 3, column=3, padx=4, pady=3)
    ttk.Button(main, text="AXISON", command=lambda a=axis: axis_on_command(a, True)).grid(row=base_row + 3, column=1, padx=4, pady=3)
    ttk.Button(main, text="AXISOFF", command=lambda a=axis: axis_on_command(a, False)).grid(row=base_row + 3, column=2, padx=4, pady=3)
    ttk.Button(main, text="AXISON+UPD", command=lambda a=axis: axis_on_update_command(a, True)).grid(row=base_row + 3, column=4, padx=4, pady=3)
    ttk.Button(main, text="DriveInit Diag", command=lambda a=axis: drive_initialisation_diagnostic(a)).grid(row=base_row + 4, column=1, columnspan=2, padx=4, pady=3, sticky="ew")
    ttk.Button(main, text="Reset Drive", command=lambda a=axis: reset_drive(a)).grid(row=base_row + 4, column=3, columnspan=2, padx=4, pady=3, sticky="ew")
    ttk.Button(main, text="Align Ref", command=lambda a=axis: align_reference(a)).grid(row=base_row + 5, column=1, columnspan=2, padx=4, pady=3, sticky="ew")
    ttk.Button(main, text="Set Vertical Zero", command=lambda a=axis: set_vertical_zero(a)).grid(row=base_row + 5, column=3, columnspan=2, padx=4, pady=3, sticky="ew")
    ttk.Button(main, text="Read Gains", command=lambda a=axis: read_drive_gains(a)).grid(row=base_row + 6, column=1, padx=4, pady=3, sticky="ew")
    ttk.Button(main, text="Scale Gains", command=lambda a=axis: scale_drive_gains(a)).grid(row=base_row + 6, column=2, padx=4, pady=3, sticky="ew")
    ttk.Entry(main, textvariable=gain_scale_var, width=8).grid(row=base_row + 6, column=3, padx=4, pady=3, sticky="w")
    ttk.Label(main, text="gain scale").grid(row=base_row + 6, column=4, padx=4, pady=3, sticky="w")

zero_entry = ttk.Entry(main, width=8)
zero_entry.insert(0, "0")
zero_entry.grid(row=29, column=1, sticky="w", padx=4, pady=8)
ttk.Label(main, text="Absolute target for Home buttons").grid(row=29, column=2, columnspan=2, sticky="w")

raw_tml_var = tk.StringVar(value="AXISON;")
ttk.Entry(main, textvariable=raw_tml_var, width=18).grid(row=30, column=1, sticky="w", padx=4, pady=8)
ttk.Button(main, text="Run X TML", command=lambda: execute_raw_tml("x")).grid(row=30, column=2, padx=4, pady=8)
ttk.Label(main, text="Raw TML command").grid(row=30, column=3, columnspan=2, sticky="w")

for col in range(5):
    main.columnconfigure(col, weight=1)

root.mainloop()
