# odrive_gui.py
import csv
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib

import OdrivePro_ACL60250_Motors_PositionInput as control
from satellite_tracking import SatelliteTrackingWindow, Xangle, Yangle

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

axis_devices = {"x": None, "y": None}

logging_active = False
position_loop_after_id = None
ENDAT_DEFAULT_BAUD = control.ENDAT_DEFAULT_BAUD
ENDAT_DEFAULT_COUNTS_PER_REV = control.ENDAT_DEFAULT_COUNTS_PER_REV
velocity_ramp_stop_event = threading.Event()
velocity_ramp_thread = None
SAFE_VELOCITY_RAMP_MAX_DEG_S = 6.0
VELOCITY_RAMP_LIMIT_MARGIN_DEG = 1.0

log_time = []
log_x_cmd_deg = []
log_x_spi_deg = []
log_y_cmd_deg = []
log_y_spi_deg = []


def _safe_call(func, fallback=None):
    try:
        return func()
    except Exception:
        return fallback


def _set_entry(entry, value):
    entry.delete(0, tk.END)
    entry.insert(0, value)


def connect_endat_x():
    port = endat_port_entry.get().strip()
    if not port:
        status_label.config(text="EnDat port is blank")
        return

    try:
        control.connect_endat(
            "x",
            port=port,
            baud=ENDAT_DEFAULT_BAUD,
            counts_per_rev=ENDAT_DEFAULT_COUNTS_PER_REV,
            home_raw=float(endat_zero_entry.get()),
            sign=float(endat_sign_entry.get()),
        )
        status_label.config(text=f"Connected shared EnDat X on {port}")
    except Exception as exc:
        status_label.config(text=f"EnDat X connection failed: {exc}")


def disconnect_endat_x():
    try:
        control.disconnect_endat("x")
    except Exception:
        pass
    x_endat_raw_var.set("-")
    x_endat_axis_var.set("-")
    status_label.config(text="Disconnected EnDat X")


def read_endat_x_sample():
    if not control.is_endat_connected("x"):
        return None
    try:
        control.configure_endat(
            "x",
            counts_per_rev=ENDAT_DEFAULT_COUNTS_PER_REV,
            home_raw=float(endat_zero_entry.get()),
            sign=float(endat_sign_entry.get()),
        )
        return control.read_endat_sample("x")
    except Exception:
        return None


def wrap_degrees_signed(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def get_endat_x_zero_count():
    value = float(endat_zero_entry.get())
    if abs(value) > 1.0:
        return int(value) % ENDAT_DEFAULT_COUNTS_PER_REV
    return int(round((value % 1.0) * ENDAT_DEFAULT_COUNTS_PER_REV))


def get_endat_x_home_raw_default():
    return float(control.AXIS_CONFIG["x"].get("endat_home_raw", 0.0))


def get_endat_x_sign():
    return float(endat_sign_entry.get())


def endat_x_sample_to_axis_deg(sample):
    return control.get_endat_axis_deg("x", sample)


def read_endat_x_measurement():
    sample = read_endat_x_sample()
    if sample is None:
        return None, None
    axis_deg = endat_x_sample_to_axis_deg(sample)
    return sample.pos32 / ENDAT_DEFAULT_COUNTS_PER_REV, axis_deg


def set_endat_x_zero_from_current():
    sample = read_endat_x_sample()
    if sample is None:
        status_label.config(text="Cannot set EnDat X zero: no sample")
        return
    home_raw = sample.pos32 / ENDAT_DEFAULT_COUNTS_PER_REV
    _set_entry(endat_zero_entry, f"{home_raw:.9f}")
    try:
        control.configure_endat("x", home_raw=home_raw, sign=float(endat_sign_entry.get()))
        control.update_external_position("x", 0.0, source="endat", timestamp=sample.timestamp_s)
        control.set_position_feedback_source("x", "external")
        control.align_motor_home_to_current_position("x")
    except Exception:
        pass
    status_label.config(text=f"EnDat X home raw set to {home_raw:.9f}; X feedback position is now 0 deg")


def refresh_config_entries():
    try:
        x_odrive = control.get_odrive("x")
        _set_entry(pos_gain_entry, f"{x_odrive.axis0.controller.config.pos_gain:g}")
        _set_entry(vel_gain_entry, f"{x_odrive.axis0.controller.config.vel_gain:g}")
        _set_entry(vel_i_entry, f"{x_odrive.axis0.controller.config.vel_integrator_gain:g}")
        _set_entry(traj_vel_entry, f"{x_odrive.axis0.trap_traj.config.vel_limit:g}")
        _set_entry(traj_acc_entry, f"{x_odrive.axis0.trap_traj.config.accel_limit:g}")
        _set_entry(traj_dec_entry, f"{x_odrive.axis0.trap_traj.config.decel_limit:g}")
    except Exception:
        pass

    try:
        y_odrive = control.get_odrive("y")
        _set_entry(pos_gain_entry_y, f"{y_odrive.axis0.controller.config.pos_gain:g}")
        _set_entry(vel_gain_entry_y, f"{y_odrive.axis0.controller.config.vel_gain:g}")
        _set_entry(vel_i_entry_y, f"{y_odrive.axis0.controller.config.vel_integrator_gain:g}")
        _set_entry(traj_vel_entry_y, f"{y_odrive.axis0.trap_traj.config.vel_limit:g}")
        _set_entry(traj_acc_entry_y, f"{y_odrive.axis0.trap_traj.config.accel_limit:g}")
        _set_entry(traj_dec_entry_y, f"{y_odrive.axis0.trap_traj.config.decel_limit:g}")
    except Exception:
        pass


def connect_odrive():
    status_label.config(text="Connecting X/Y ODrives...")
    root.update()

    try:
        control.set_force_spi_load_encoder_on_connect(bool(force_spi_load_encoder_var.get()))
        devices = control.connect_all()
        axis_devices.update(devices)
        apply_positioning_defaults()
        refresh_config_entries()
        load_policy = "forced SPI load encoder" if force_spi_load_encoder_var.get() else "ODrive load encoder unchanged"
        status_label.config(
            text=(
                f"Connected & Initialized; {load_policy}; positioning gains/trajectory applied\n"
                f"X: {control.get_serial_number('x')}\n"
                f"Y: {control.get_serial_number('y')}"
            )
        )
        start_position_loop()
    except TimeoutError as e:
        status_label.config(text=f"Initialization timeout: {e}")
    except Exception as e:
        status_label.config(text=f"Connection Failed: {e}")


def start_position_loop():
    global position_loop_after_id
    if position_loop_after_id is None:
        update_position_loop()


def update_position_loop():
    global position_loop_after_id
    x_position = _safe_call(lambda: control.get_current_position("x"))
    x_spi_raw = _safe_call(lambda: control.get_spi_raw("x"))
    x_motor_raw = _safe_call(lambda: control.get_motor_raw("x"))
    x_state = _safe_call(lambda: control.get_axis_state("x"))
    x_active_errors = _safe_call(lambda: control.get_active_errors("x"))
    x_disarm = _safe_call(lambda: control.get_disarm_reason("x"))
    x_procedure_result = _safe_call(lambda: control.get_procedure_result("x"))
    x_input_pos = _safe_call(lambda: control.get_input_pos("x"))
    x_velocity = _safe_call(lambda: control.get_motor_velocity("x"))
    x_control_mode = _safe_call(lambda: control.get_control_mode("x"))
    x_input_mode = _safe_call(lambda: control.get_input_mode("x"))
    x_load_encoder = _safe_call(lambda: control.get_load_encoder("x"))
    x_feedback_source = _safe_call(lambda: control.get_position_feedback_source("x"))
    x_endat_turns, x_endat_axis_deg = _safe_call(read_endat_x_measurement, (None, None))

    if x_position is not None:
        x_position_var.set(f"{x_position:.3f} deg")
    else:
        x_position_var.set("stale/-")
    if x_spi_raw is not None:
        x_spi_raw_var.set(f"{x_spi_raw:.6f}")
    if x_motor_raw is not None:
        x_motor_raw_var.set(f"{x_motor_raw:.6f}")
    if x_endat_turns is not None:
        x_endat_raw_var.set(f"{x_endat_turns:.6f}")
    if x_endat_axis_deg is not None:
        x_endat_axis_var.set(f"{x_endat_axis_deg:.3f} deg")

    x_fault_var.set(
        f"State: {x_state if x_state is not None else '-'}   "
        f"Disarm: 0x{x_disarm:X}" if x_disarm is not None else f"State: {x_state if x_state is not None else '-'}   Disarm: -"
    )
    if x_active_errors is not None:
        x_fault_var.set(f"{x_fault_var.get()}   Active: 0x{x_active_errors:X}")
    else:
        x_fault_var.set(f"{x_fault_var.get()}   Active: -")
    x_fault_var.set(f"{x_fault_var.get()}   Proc: {x_procedure_result if x_procedure_result is not None else '-'}")

    x_diag_var.set(
        f"Cmd: {f'{x_input_pos:.6f}' if x_input_pos is not None else '-'}   "
        f"Est: {f'{x_motor_raw:.6f}' if x_motor_raw is not None else '-'}   "
        f"Vel: {f'{x_velocity:.6f}' if x_velocity is not None else '-'}   "
        f"Angle: {f'{x_position:.3f}' if x_position is not None else '-'}   "
        f"Ctrl/In/Load: "
        f"{x_control_mode if x_control_mode is not None else '-'}/"
        f"{x_input_mode if x_input_mode is not None else '-'}/"
        f"{x_load_encoder if x_load_encoder is not None else '-'}   "
        f"Feedback: {x_feedback_source if x_feedback_source is not None else '-'}"
    )

    y_position = _safe_call(lambda: control.get_current_position("y"))
    y_spi_raw = _safe_call(lambda: control.get_spi_raw("y"))
    y_motor_raw = _safe_call(lambda: control.get_motor_raw("y"))
    y_state = _safe_call(lambda: control.get_axis_state("y"))
    y_active_errors = _safe_call(lambda: control.get_active_errors("y"))
    y_disarm = _safe_call(lambda: control.get_disarm_reason("y"))
    y_procedure_result = _safe_call(lambda: control.get_procedure_result("y"))
    y_input_pos = _safe_call(lambda: control.get_input_pos("y"))
    y_velocity = _safe_call(lambda: control.get_motor_velocity("y"))
    y_control_mode = _safe_call(lambda: control.get_control_mode("y"))
    y_input_mode = _safe_call(lambda: control.get_input_mode("y"))
    y_load_encoder = _safe_call(lambda: control.get_load_encoder("y"))
    y_feedback_source = _safe_call(lambda: control.get_position_feedback_source("y"))

    if y_position is not None:
        y_position_var.set(f"{y_position:.3f} deg")
    else:
        y_position_var.set("stale/-")
    if y_spi_raw is not None:
        y_spi_raw_var.set(f"{y_spi_raw:.6f}")
    if y_motor_raw is not None:
        y_motor_raw_var.set(f"{y_motor_raw:.6f}")

    y_fault_var.set(
        f"State: {y_state if y_state is not None else '-'}   "
        f"Disarm: 0x{y_disarm:X}" if y_disarm is not None else f"State: {y_state if y_state is not None else '-'}   Disarm: -"
    )
    if y_active_errors is not None:
        y_fault_var.set(f"{y_fault_var.get()}   Active: 0x{y_active_errors:X}")
    else:
        y_fault_var.set(f"{y_fault_var.get()}   Active: -")
    y_fault_var.set(f"{y_fault_var.get()}   Proc: {y_procedure_result if y_procedure_result is not None else '-'}")

    y_diag_var.set(
        f"Cmd: {f'{y_input_pos:.6f}' if y_input_pos is not None else '-'}   "
        f"Est: {f'{y_motor_raw:.6f}' if y_motor_raw is not None else '-'}   "
        f"Vel: {f'{y_velocity:.6f}' if y_velocity is not None else '-'}   "
        f"Angle: {f'{y_position:.3f}' if y_position is not None else '-'}   "
        f"Ctrl/In/Load: "
        f"{y_control_mode if y_control_mode is not None else '-'}/"
        f"{y_input_mode if y_input_mode is not None else '-'}/"
        f"{y_load_encoder if y_load_encoder is not None else '-'}   "
        f"Feedback: {y_feedback_source if y_feedback_source is not None else '-'}"
    )

    position_loop_after_id = root.after(100, update_position_loop)


def axis_fault_summary(axis):
    active_errors = _safe_call(lambda: control.get_active_errors(axis))
    disarm_reason = _safe_call(lambda: control.get_disarm_reason(axis))
    axis_state = _safe_call(lambda: control.get_axis_state(axis))
    procedure_result = _safe_call(lambda: control.get_procedure_result(axis))
    selected_pos = _safe_call(lambda: control.get_current_position(axis))
    feedback_source = _safe_call(lambda: control.get_position_feedback_source(axis))
    load_encoder = _safe_call(lambda: control.get_load_encoder(axis))
    return (
        f"{axis.upper()} state={axis_state if axis_state is not None else '-'} "
        f"active=0x{active_errors:X}" if active_errors is not None else
        f"{axis.upper()} state={axis_state if axis_state is not None else '-'} active=-"
    ) + (
        f" disarm=0x{disarm_reason:X}" if disarm_reason is not None else " disarm=-"
    ) + f" proc={procedure_result if procedure_result is not None else '-'}" + (
        f" pos={selected_pos:.3f}" if selected_pos is not None else " pos=-"
    ) + f" feedback={feedback_source if feedback_source is not None else '-'}" + (
        f" load={load_encoder}" if load_encoder is not None else " load=-"
    )


def run_axis_command_async(label, func, kwargs, axis):
    def runner():
        try:
            func(**kwargs)
            root.after(0, lambda: status_label.config(text=f"{label} complete"))
        except Exception as exc:
            details = axis_fault_summary(axis)
            error_text = str(exc)
            root.after(0, lambda: status_label.config(text=f"{label} failed: {error_text}\n{details}"))

    threading.Thread(target=runner, daemon=True).start()


def disarm_axis(axis):
    try:
        control.disarm_axis(axis)
        status_label.config(text=f"{axis.upper()} axis disarmed")
    except Exception as exc:
        status_label.config(text=f"{axis.upper()} disarm failed: {exc}")


def recover_axis(axis):
    apply_positioning_defaults()
    run_axis_command_async(
        f"Recover {axis.upper()} inward",
        control.recover_axis_to_safe_range,
        {"axis": axis},
        axis,
    )


def go_home(axis):
    apply_positioning_defaults()
    run_axis_command_async(f"Go Home {axis.upper()}", control.go_home, {"axis": axis}, axis)


def move_relative(axis, delta):
    apply_positioning_defaults()
    run_axis_command_async(
        f"Move {axis.upper()} {delta:+g} deg",
        control.move_relative,
        {"delta_deg": delta, "axis": axis},
        axis,
    )


def move_absolute(axis, entry):
    try:
        deg = float(entry.get())
        apply_positioning_defaults()
        run_axis_command_async(
            f"Move {axis.upper()} to {deg:g} deg",
            control.move_absolute,
            {"target_output_deg": deg, "axis": axis},
            axis,
        )
    except Exception:
        pass


def start_velocity_ramp_test():
    global velocity_ramp_thread
    if velocity_ramp_thread is not None and velocity_ramp_thread.is_alive():
        return

    try:
        axis = velocity_axis_var.get()
        vel_1 = float(vel_start_entry.get())
        vel_2 = float(vel_target_entry.get())
        hold_v1 = float(vel_hold_v1_entry.get())
        hold_after = float(vel_hold_after_entry.get())
        native_ramp_rate = float(vel_native_ramp_entry.get())
        if axis not in ("x", "y"):
            raise ValueError
        if hold_v1 < 0 or hold_after < 0 or native_ramp_rate < 0:
            raise ValueError
    except Exception:
        status_label.config(text="Velocity ramp inputs invalid")
        return

    vel_1 = max(min(vel_1, SAFE_VELOCITY_RAMP_MAX_DEG_S), -SAFE_VELOCITY_RAMP_MAX_DEG_S)
    vel_2 = max(min(vel_2, SAFE_VELOCITY_RAMP_MAX_DEG_S), -SAFE_VELOCITY_RAMP_MAX_DEG_S)

    def axis_within_safe_travel():
        pos = _safe_call(lambda: control.get_current_position(axis), None)
        if pos is None:
            return True
        if pos >= control.TRACKING_MAX_DEGREE - VELOCITY_RAMP_LIMIT_MARGIN_DEG and max(vel_1, vel_2) > 0:
            return False
        if pos <= control.TRACKING_MIN_DEGREE + VELOCITY_RAMP_LIMIT_MARGIN_DEG and min(vel_1, vel_2) < 0:
            return False
        return True

    def command_with_limit_check(commanded_vel):
        if not axis_within_safe_travel():
            raise RuntimeError(
                f"{axis.upper()} reached travel safety boundary near "
                f"{control.TRACKING_MIN_DEGREE}/{control.TRACKING_MAX_DEGREE} deg"
            )
        control.command_velocity(commanded_vel, axis=axis)

    def worker():
        step_dt = 0.02
        velocity_ramp_stop_event.clear()
        try:
            control.enter_velocity_mode(
                axis=axis,
                ramp_rate_deg_s2=native_ramp_rate if native_ramp_rate > 0.0 else None,
            )
            command_with_limit_check(vel_1)
            hold_v1_start = time.monotonic()
            while not velocity_ramp_stop_event.is_set() and time.monotonic() - hold_v1_start < hold_v1:
                command_with_limit_check(vel_1)
                time.sleep(step_dt)

            if not velocity_ramp_stop_event.is_set():
                command_with_limit_check(vel_2)

            final_start = time.monotonic()
            while not velocity_ramp_stop_event.is_set() and time.monotonic() - final_start < hold_after:
                command_with_limit_check(vel_2)
                time.sleep(step_dt)
        except Exception as exc:
            status_label.config(text=f"Velocity ramp failed: {exc}")
        finally:
            try:
                control.command_velocity(0.0, axis=axis)
            except Exception:
                pass
            try:
                control.exit_velocity_mode(axis=axis)
            except Exception:
                pass
            try:
                control.go_home(axis=axis)
            except Exception:
                pass
            if not velocity_ramp_stop_event.is_set():
                status_label.config(
                    text=(
                        f"Velocity step complete ({axis.upper()}): "
                        f"{vel_1:.2f} -> {vel_2:.2f} deg/s"
                    )
                )

    status_label.config(
        text=(
            f"Velocity step test running ({axis.upper()}): "
            f"{vel_1:.2f} -> {vel_2:.2f} deg/s "
            f"(clamped to +/-{SAFE_VELOCITY_RAMP_MAX_DEG_S:.1f}, "
            f"native ramp {'off' if native_ramp_rate <= 0 else f'{native_ramp_rate:.2f} deg/s^2'})"
        )
    )
    velocity_ramp_thread = threading.Thread(target=worker, daemon=True)
    velocity_ramp_thread.start()


def stop_velocity_ramp_test():
    velocity_ramp_stop_event.set()
    status_label.config(text="Velocity ramp stopping...")


def apply_gains():
    try:
        p = float(pos_gain_entry.get())
        v = float(vel_gain_entry.get())
        vi = float(vel_i_entry.get())
        control.set_gains(p, v, vi, axis="x")
    except Exception:
        pass


def apply_gains_small():
    try:
        p = float(pos_gain_entry_small.get())
        v = float(vel_gain_entry_small.get())
        vi = float(vel_i_entry_small.get())
        control.set_gains(p, v, vi, axis="x")
    except Exception:
        pass


def apply_gains_y():
    try:
        p = float(pos_gain_entry_y.get())
        v = float(vel_gain_entry_y.get())
        vi = float(vel_i_entry_y.get())
        control.set_gains(p, v, vi, axis="y")
    except Exception:
        pass


def apply_gains_small_y():
    try:
        p = float(pos_gain_entry_small_y.get())
        v = float(vel_gain_entry_small_y.get())
        vi = float(vel_i_entry_small_y.get())
        control.set_gains(p, v, vi, axis="y")
    except Exception:
        pass


def get_preposition_gains():
    return (
        float(pos_gain_entry.get()),
        float(vel_gain_entry.get()),
        float(vel_i_entry.get()),
    )


def get_tracking_gains():
    return (
        float(pos_gain_entry_small.get()),
        float(vel_gain_entry_small.get()),
        float(vel_i_entry_small.get()),
    )


def get_tracking_gains_y():
    return (
        float(pos_gain_entry_small_y.get()),
        float(vel_gain_entry_small_y.get()),
        float(vel_i_entry_small_y.get()),
    )


def apply_traj():
    try:
        vel = float(traj_vel_entry.get())
        acc = float(traj_acc_entry.get())
        dec = float(traj_dec_entry.get())
        control.set_traj_params(vel, acc, dec, axis="x")
    except Exception:
        pass


def apply_traj_y():
    try:
        vel = float(traj_vel_entry_y.get())
        acc = float(traj_acc_entry_y.get())
        dec = float(traj_dec_entry_y.get())
        control.set_traj_params(vel, acc, dec, axis="y")
    except Exception:
        pass


def apply_positioning_defaults():
    apply_gains()
    apply_gains_y()
    apply_traj()
    apply_traj_y()


def open_satellite_tracking():
    SatelliteTrackingWindow(
        root,
        odrvs=axis_devices,
        control=control,
        observer_lat=33.677,
        observer_lon=-112.093,
        preposition_gains=get_preposition_gains(),
        tracking_gains=get_tracking_gains(),
        tracking_gains_y=get_tracking_gains_y(),
    )


def open_manual_pointing():
    window = tk.Toplevel(root)
    window.title("Manual Pointing")
    window.geometry("350x300")

    ttk.Label(window, text="Enter Azimuth (deg)").pack()
    az_entry = ttk.Entry(window)
    az_entry.pack()

    ttk.Label(window, text="Enter Elevation (deg)").pack()
    el_entry = ttk.Entry(window)
    el_entry.pack()

    ttk.Label(window, text="Enter X Angle (deg)").pack()
    x_entry = ttk.Entry(window)
    x_entry.pack()

    ttk.Label(window, text="Enter Y Angle (deg)").pack()
    y_entry = ttk.Entry(window)
    y_entry.pack()

    def point_from_azel():
        try:
            az = float(az_entry.get())
            el = float(el_entry.get())
            x = Xangle(az, el)
            y = Yangle(az, el)
            threading.Thread(
                target=control.move_absolute_pair,
                kwargs={"x_deg": x, "y_deg": y},
                daemon=True,
            ).start()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def point_from_xy():
        try:
            x = float(x_entry.get())
            y = float(y_entry.get())
            threading.Thread(
                target=control.move_absolute_pair,
                kwargs={"x_deg": x, "y_deg": y},
                daemon=True,
            ).start()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    ttk.Button(window, text="Point using AZ/EL", command=point_from_azel).pack(pady=10)
    ttk.Button(window, text="Point using X/Y", command=point_from_xy).pack(pady=10)


class LivePlotFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Live Position Plot")

        self.interval_var = tk.DoubleVar(value=0.1)

        control_frame = ttk.Frame(self)
        control_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(control_frame, text="Logging Interval (sec):").pack(side="left")
        ttk.Entry(control_frame, textvariable=self.interval_var, width=6).pack(side="left", padx=5)

        ttk.Button(control_frame, text="Start Logging", command=self.start_logging).pack(side="left", padx=5)
        ttk.Button(control_frame, text="Stop Logging", command=self.stop_logging).pack(side="left", padx=5)
        ttk.Button(control_frame, text="Save CSV", command=self.save_csv).pack(side="left", padx=5)

        self.fig = Figure(figsize=(5.8, 5.2), dpi=100)
        self.ax_x = self.fig.add_subplot(211)
        self.ax_y = self.fig.add_subplot(212)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def start_logging(self):
        global logging_active

        if logging_active:
            return

        log_time.clear()
        log_x_cmd_deg.clear()
        log_x_spi_deg.clear()
        log_y_cmd_deg.clear()
        log_y_spi_deg.clear()
        self.start_time = time.time()

        logging_active = True
        self.logging_loop()

    def stop_logging(self):
        global logging_active
        logging_active = False

    def logging_loop(self):
        if not logging_active:
            return

        now = time.time() - self.start_time

        try:
            log_time.append(now)
            log_x_cmd_deg.append(control.get_current_position("x"))
            log_x_spi_deg.append(control.get_spi_position("x"))
            log_y_cmd_deg.append(control.get_current_position("y"))
            log_y_spi_deg.append(control.get_spi_position("y"))
            self.update_plot()
        except Exception:
            pass

        interval_ms = int(self.interval_var.get() * 1000)
        self.after(interval_ms, self.logging_loop)

    def update_plot(self):
        self.ax_x.clear()
        self.ax_y.clear()

        self.ax_x.plot(log_time, log_x_cmd_deg)
        self.ax_x.plot(log_time, log_x_spi_deg)
        self.ax_x.set_title("X Axis: Commanded vs SPI Output")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("Degrees")
        self.ax_x.legend(["Commanded", "SPI"])

        self.ax_y.plot(log_time, log_y_cmd_deg)
        self.ax_y.plot(log_time, log_y_spi_deg)
        self.ax_y.set_title("Y Axis: Commanded vs SPI Output")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Degrees")
        self.ax_y.legend(["Commanded", "SPI"])

        self.fig.tight_layout()
        self.canvas.draw()

    def save_csv(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".csv")
        if not file_path:
            return

        with open(file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "Time (s)",
                    "X Commanded (deg)",
                    "X SPI (deg)",
                    "X Error (deg)",
                    "Y Commanded (deg)",
                    "Y SPI (deg)",
                    "Y Error (deg)",
                ]
            )
            for i in range(len(log_time)):
                writer.writerow(
                    [
                        log_time[i],
                        log_x_cmd_deg[i],
                        log_x_spi_deg[i],
                        log_x_cmd_deg[i] - log_x_spi_deg[i],
                        log_y_cmd_deg[i],
                        log_y_spi_deg[i],
                        log_y_cmd_deg[i] - log_y_spi_deg[i],
                    ]
                )


root = tk.Tk()
root.title("ODrive Position Control GUI")
root.geometry("1280x760")

main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True)

left_panel = ttk.Frame(main_frame, width=560)
left_panel.pack(side="left", fill="y", padx=(10, 4), pady=10)
left_panel.pack_propagate(False)

left_canvas = tk.Canvas(left_panel, highlightthickness=0)
left_scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=left_canvas.yview)
left_canvas.configure(yscrollcommand=left_scrollbar.set)

left_scrollbar.pack(side="right", fill="y")
left_canvas.pack(side="left", fill="both", expand=True)

left_frame = ttk.Frame(left_canvas)
left_canvas_window = left_canvas.create_window((0, 0), window=left_frame, anchor="nw")


def _update_left_scrollregion(event):
    left_canvas.configure(scrollregion=left_canvas.bbox("all"))


def _resize_left_canvas_window(event):
    left_canvas.itemconfigure(left_canvas_window, width=event.width)


def _bind_mousewheel(_event):
    left_canvas.bind_all("<MouseWheel>", _on_mousewheel)


def _unbind_mousewheel(_event):
    left_canvas.unbind_all("<MouseWheel>")


def _on_mousewheel(event):
    left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")


left_frame.bind("<Configure>", _update_left_scrollregion)
left_canvas.bind("<Configure>", _resize_left_canvas_window)
left_canvas.bind("<Enter>", _bind_mousewheel)
left_canvas.bind("<Leave>", _unbind_mousewheel)

right_frame = ttk.Frame(main_frame)
right_frame.pack(side="right", fill="both", expand=True, padx=(4, 10), pady=10)

ttk.Button(left_frame, text="Connect ODrive", command=connect_odrive).pack(pady=5)
force_spi_load_encoder_var = tk.IntVar(value=int(control.FORCE_SPI_LOAD_ENCODER_ON_CONNECT))
ttk.Checkbutton(
    left_frame,
    text="Force ODrive load encoder = SPI",
    variable=force_spi_load_encoder_var,
).pack(anchor="w", padx=5)

status_label = ttk.Label(left_frame, text="Not Connected")
status_label.pack()

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Current Position").pack()

x_status_frame = ttk.Frame(left_frame)
x_status_frame.pack(fill="x", pady=2)
ttk.Label(x_status_frame, text="X Axis").grid(row=0, column=0, sticky="w")
x_position_var = tk.StringVar(value="0.000 deg")
ttk.Label(x_status_frame, textvariable=x_position_var, font=("Arial", 14)).grid(row=0, column=1, sticky="w", padx=8)
x_fault_var = tk.StringVar(value="State: -   Disarm: -   Active: -   Proc: -")
ttk.Label(x_status_frame, textvariable=x_fault_var).grid(row=1, column=0, columnspan=2, sticky="w")
x_diag_var = tk.StringVar(value="Cmd: -   Est: -   Vel: -   Angle: -")
ttk.Label(x_status_frame, textvariable=x_diag_var).grid(row=2, column=0, columnspan=2, sticky="w")

y_status_frame = ttk.Frame(left_frame)
y_status_frame.pack(fill="x", pady=2)
ttk.Label(y_status_frame, text="Y Axis").grid(row=0, column=0, sticky="w")
y_position_var = tk.StringVar(value="0.000 deg")
ttk.Label(y_status_frame, textvariable=y_position_var, font=("Arial", 14)).grid(row=0, column=1, sticky="w", padx=8)
y_fault_var = tk.StringVar(value="State: -   Disarm: -   Active: -   Proc: -")
ttk.Label(y_status_frame, textvariable=y_fault_var).grid(row=1, column=0, columnspan=2, sticky="w")
y_diag_var = tk.StringVar(value="Cmd: -   Est: -   Vel: -   Angle: -")
ttk.Label(y_status_frame, textvariable=y_diag_var).grid(row=2, column=0, columnspan=2, sticky="w")

raw_frame = ttk.Frame(left_frame)
raw_frame.pack(fill="x", pady=5)

ttk.Label(raw_frame, text="Axis").grid(row=0, column=0, padx=5)
ttk.Label(raw_frame, text="ODrive Est (turns)").grid(row=0, column=1, padx=5)
ttk.Label(raw_frame, text="SPI Encoder (turns)").grid(row=0, column=2, padx=5)
ttk.Label(raw_frame, text="EnDat Encoder (turns)").grid(row=0, column=3, padx=5)

ttk.Label(raw_frame, text="X").grid(row=1, column=0)
x_motor_raw_var = tk.StringVar(value="0")
x_spi_raw_var = tk.StringVar(value="0")
x_endat_raw_var = tk.StringVar(value="-")
x_endat_axis_var = tk.StringVar(value="-")
ttk.Label(raw_frame, textvariable=x_motor_raw_var).grid(row=1, column=1)
ttk.Label(raw_frame, textvariable=x_spi_raw_var).grid(row=1, column=2)
ttk.Label(raw_frame, textvariable=x_endat_raw_var).grid(row=1, column=3)

ttk.Label(raw_frame, text="Y").grid(row=2, column=0)
y_motor_raw_var = tk.StringVar(value="0")
y_spi_raw_var = tk.StringVar(value="0")
ttk.Label(raw_frame, textvariable=y_motor_raw_var).grid(row=2, column=1)
ttk.Label(raw_frame, textvariable=y_spi_raw_var).grid(row=2, column=2)
ttk.Label(raw_frame, text="-").grid(row=2, column=3)

endat_frame = ttk.Frame(left_frame)
endat_frame.pack(fill="x", pady=3)
ttk.Label(endat_frame, text="EnDat X Port").grid(row=0, column=0, padx=5, pady=2)
endat_port_entry = ttk.Entry(endat_frame, width=8)
endat_port_entry.insert(0, "COM6")
endat_port_entry.grid(row=0, column=1, padx=5, pady=2)
ttk.Button(endat_frame, text="Connect EnDat X", command=connect_endat_x).grid(
    row=0, column=2, columnspan=2, sticky="ew", padx=5, pady=2
)
ttk.Button(endat_frame, text="Disconnect", command=disconnect_endat_x).grid(
    row=0, column=4, sticky="ew", padx=5, pady=2
)
ttk.Label(endat_frame, text="Axis").grid(row=1, column=0, padx=5, pady=2)
ttk.Label(endat_frame, textvariable=x_endat_axis_var).grid(row=1, column=1, sticky="w", padx=5, pady=2)
ttk.Label(endat_frame, text="Home Raw").grid(row=1, column=2, padx=5, pady=2)
endat_zero_entry = ttk.Entry(endat_frame, width=10)
endat_zero_entry.insert(0, f"{get_endat_x_home_raw_default():.9f}")
endat_zero_entry.grid(row=1, column=3, padx=5, pady=2)
ttk.Button(endat_frame, text="Set Zero", command=set_endat_x_zero_from_current).grid(
    row=1, column=4, sticky="ew", padx=5, pady=2
)
ttk.Label(endat_frame, text="Sign").grid(row=2, column=0, padx=5, pady=2)
endat_sign_entry = ttk.Entry(endat_frame, width=6)
endat_sign_entry.insert(0, "1")
endat_sign_entry.grid(row=2, column=1, padx=5, pady=2)
ttk.Label(endat_frame, text="Display only here; tracking can use it as X feedback").grid(
    row=2, column=2, columnspan=3, sticky="w", padx=5, pady=2
)

ttk.Separator(left_frame).pack(fill="x", pady=10)

home_frame = ttk.Frame(left_frame)
home_frame.pack(fill="x")
ttk.Button(home_frame, text="Go Home X", command=lambda: go_home("x")).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(home_frame, text="Go Home Y", command=lambda: go_home("y")).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(home_frame, text="Disarm X", command=lambda: disarm_axis("x")).grid(row=0, column=2, padx=5, pady=5)
ttk.Button(home_frame, text="Disarm Y", command=lambda: disarm_axis("y")).grid(row=0, column=3, padx=5, pady=5)
ttk.Button(home_frame, text="Recover X", command=lambda: recover_axis("x")).grid(row=1, column=0, padx=5, pady=5)
ttk.Button(home_frame, text="Recover Y", command=lambda: recover_axis("y")).grid(row=1, column=1, padx=5, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Relative Move").pack(pady=5)

rel_frame = ttk.Frame(left_frame)
rel_frame.pack()

ttk.Label(rel_frame, text="X").grid(row=0, column=0, padx=5)
ttk.Button(rel_frame, text="-10 deg", command=lambda: move_relative("x", -10)).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(rel_frame, text="-1 deg", command=lambda: move_relative("x", -1)).grid(row=0, column=2, padx=5, pady=5)
ttk.Button(rel_frame, text="+1 deg", command=lambda: move_relative("x", 1)).grid(row=0, column=3, padx=5, pady=5)
ttk.Button(rel_frame, text="+10 deg", command=lambda: move_relative("x", 10)).grid(row=0, column=4, padx=5, pady=5)

ttk.Label(rel_frame, text="Y").grid(row=1, column=0, padx=5)
ttk.Button(rel_frame, text="-10 deg", command=lambda: move_relative("y", -10)).grid(row=1, column=1, padx=5, pady=5)
ttk.Button(rel_frame, text="-1 deg", command=lambda: move_relative("y", -1)).grid(row=1, column=2, padx=5, pady=5)
ttk.Button(rel_frame, text="+1 deg", command=lambda: move_relative("y", 1)).grid(row=1, column=3, padx=5, pady=5)
ttk.Button(rel_frame, text="+10 deg", command=lambda: move_relative("y", 10)).grid(row=1, column=4, padx=5, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Absolute Move (deg)").pack()

abs_frame = ttk.Frame(left_frame)
abs_frame.pack(fill="x")

ttk.Label(abs_frame, text="X").grid(row=0, column=0, padx=5, pady=4)
x_abs_entry = ttk.Entry(abs_frame, width=12)
x_abs_entry.grid(row=0, column=1, padx=5, pady=4)
ttk.Button(abs_frame, text="Move Absolute", command=lambda: move_absolute("x", x_abs_entry)).grid(row=0, column=2, padx=5, pady=4)

ttk.Label(abs_frame, text="Y").grid(row=1, column=0, padx=5, pady=4)
y_abs_entry = ttk.Entry(abs_frame, width=12)
y_abs_entry.grid(row=1, column=1, padx=5, pady=4)
ttk.Button(abs_frame, text="Move Absolute", command=lambda: move_absolute("y", y_abs_entry)).grid(row=1, column=2, padx=5, pady=4)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Controller Gains").pack(pady=5)

gains_frame = ttk.Frame(left_frame)
gains_frame.pack()

ttk.Label(gains_frame, text="X Axis").grid(row=0, column=0, columnspan=4, pady=(0, 4))
ttk.Label(gains_frame, text="Preposition / Pointing").grid(row=0, column=0, columnspan=2, sticky="w", padx=(0, 10))
ttk.Label(gains_frame, text="Tracking").grid(row=0, column=2, columnspan=2, sticky="w", padx=10)

ttk.Label(gains_frame, text="Position Gain").grid(row=1, column=0)
pos_gain_entry = ttk.Entry(gains_frame, width=8)
pos_gain_entry.insert(0, f"{control.DEFAULT_POS_GAIN:g}")
pos_gain_entry.grid(row=1, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=2, column=0)
vel_gain_entry = ttk.Entry(gains_frame, width=8)
vel_gain_entry.insert(0, f"{control.DEFAULT_VEL_GAIN:g}")
vel_gain_entry.grid(row=2, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=3, column=0)
vel_i_entry = ttk.Entry(gains_frame, width=8)
vel_i_entry.insert(0, f"{control.DEFAULT_VEL_INTEGRATOR_GAIN:g}")
vel_i_entry.grid(row=3, column=1, padx=5)

ttk.Button(gains_frame, text="Apply Preposition Gains", command=apply_gains).grid(row=4, column=0, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Position Gain").grid(row=1, column=2, padx=10)
pos_gain_entry_small = ttk.Entry(gains_frame, width=8)
pos_gain_entry_small.insert(0, "0.05")
pos_gain_entry_small.grid(row=1, column=3)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=2, column=2)
vel_gain_entry_small = ttk.Entry(gains_frame, width=8)
vel_gain_entry_small.insert(0, "0.2")
vel_gain_entry_small.grid(row=2, column=3)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=3, column=2)
vel_i_entry_small = ttk.Entry(gains_frame, width=8)
vel_i_entry_small.insert(0, "0.1")
vel_i_entry_small.grid(row=3, column=3)

ttk.Button(gains_frame, text="Apply Tracking Gains", command=apply_gains_small).grid(row=4, column=2, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Y Axis").grid(row=5, column=0, columnspan=4, pady=(10, 4))
ttk.Label(gains_frame, text="Preposition / Pointing").grid(row=5, column=0, columnspan=2, sticky="w", padx=(0, 10))
ttk.Label(gains_frame, text="Tracking").grid(row=5, column=2, columnspan=2, sticky="w", padx=10)

ttk.Label(gains_frame, text="Position Gain").grid(row=6, column=0)
pos_gain_entry_y = ttk.Entry(gains_frame, width=8)
pos_gain_entry_y.insert(0, f"{control.DEFAULT_POS_GAIN:g}")
pos_gain_entry_y.grid(row=6, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=7, column=0)
vel_gain_entry_y = ttk.Entry(gains_frame, width=8)
vel_gain_entry_y.insert(0, f"{control.DEFAULT_VEL_GAIN:g}")
vel_gain_entry_y.grid(row=7, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=8, column=0)
vel_i_entry_y = ttk.Entry(gains_frame, width=8)
vel_i_entry_y.insert(0, f"{control.DEFAULT_VEL_INTEGRATOR_GAIN:g}")
vel_i_entry_y.grid(row=8, column=1, padx=5)

ttk.Button(gains_frame, text="Apply Preposition Gains", command=apply_gains_y).grid(row=9, column=0, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Position Gain").grid(row=6, column=2, padx=10)
pos_gain_entry_small_y = ttk.Entry(gains_frame, width=8)
pos_gain_entry_small_y.insert(0, "1")
pos_gain_entry_small_y.grid(row=6, column=3)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=7, column=2)
vel_gain_entry_small_y = ttk.Entry(gains_frame, width=8)
vel_gain_entry_small_y.insert(0, "1")
vel_gain_entry_small_y.grid(row=7, column=3)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=8, column=2)
vel_i_entry_small_y = ttk.Entry(gains_frame, width=8)
vel_i_entry_small_y.insert(0, "1")
vel_i_entry_small_y.grid(row=8, column=3)

ttk.Button(gains_frame, text="Apply Tracking Gains", command=apply_gains_small_y).grid(row=9, column=2, columnspan=2, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Trajectory Parameters").pack(pady=5)

traj_frame = ttk.Frame(left_frame)
traj_frame.pack()

ttk.Label(traj_frame, text="X Axis").grid(row=0, column=0, columnspan=2, pady=(0, 4))

ttk.Label(traj_frame, text="Velocity Limit").grid(row=1, column=0, sticky="w")
traj_vel_entry = ttk.Entry(traj_frame, width=10)
traj_vel_entry.insert(0, f"{control.DEFAULT_TRAJ_VEL_LIMIT:g}")
traj_vel_entry.grid(row=1, column=1, padx=5, pady=2)

ttk.Label(traj_frame, text="Acceleration Limit").grid(row=2, column=0, sticky="w")
traj_acc_entry = ttk.Entry(traj_frame, width=10)
traj_acc_entry.insert(0, f"{control.DEFAULT_TRAJ_ACCEL_LIMIT:g}")
traj_acc_entry.grid(row=2, column=1, padx=5, pady=2)

ttk.Label(traj_frame, text="Deceleration Limit").grid(row=3, column=0, sticky="w")
traj_dec_entry = ttk.Entry(traj_frame, width=10)
traj_dec_entry.insert(0, f"{control.DEFAULT_TRAJ_DECEL_LIMIT:g}")
traj_dec_entry.grid(row=3, column=1, padx=5, pady=2)

ttk.Button(traj_frame, text="Apply Trajectory", command=apply_traj).grid(row=4, column=0, columnspan=2, pady=5)

ttk.Label(traj_frame, text="Y Axis").grid(row=5, column=0, columnspan=2, pady=(10, 4))

ttk.Label(traj_frame, text="Velocity Limit").grid(row=6, column=0, sticky="w")
traj_vel_entry_y = ttk.Entry(traj_frame, width=10)
traj_vel_entry_y.insert(0, f"{control.DEFAULT_TRAJ_VEL_LIMIT:g}")
traj_vel_entry_y.grid(row=6, column=1, padx=5, pady=2)

ttk.Label(traj_frame, text="Acceleration Limit").grid(row=7, column=0, sticky="w")
traj_acc_entry_y = ttk.Entry(traj_frame, width=10)
traj_acc_entry_y.insert(0, f"{control.DEFAULT_TRAJ_ACCEL_LIMIT:g}")
traj_acc_entry_y.grid(row=7, column=1, padx=5, pady=2)

ttk.Label(traj_frame, text="Deceleration Limit").grid(row=8, column=0, sticky="w")
traj_dec_entry_y = ttk.Entry(traj_frame, width=10)
traj_dec_entry_y.insert(0, f"{control.DEFAULT_TRAJ_DECEL_LIMIT:g}")
traj_dec_entry_y.grid(row=8, column=1, padx=5, pady=2)

ttk.Button(traj_frame, text="Apply Trajectory", command=apply_traj_y).grid(row=9, column=0, columnspan=2, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Tracking GUI").pack(pady=5)
ttk.Button(left_frame, text="Satellite Tracking", command=open_satellite_tracking).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Manual Pointing").pack(pady=5)

az_frame = ttk.Frame(left_frame)
az_frame.pack(pady=2, fill="x")
ttk.Label(az_frame, text="Azimuth (deg)").grid(row=0, column=0)
manual_az_entry = ttk.Entry(az_frame, width=10)
manual_az_entry.grid(row=0, column=1, padx=2)
ttk.Label(az_frame, text="X ->").grid(row=0, column=2)
manual_x_display = ttk.Label(az_frame, text="0.0", width=8)
manual_x_display.grid(row=0, column=3)

el_frame = ttk.Frame(left_frame)
el_frame.pack(pady=2, fill="x")
ttk.Label(el_frame, text="Elevation (deg)").grid(row=0, column=0)
manual_el_entry = ttk.Entry(el_frame, width=10)
manual_el_entry.grid(row=0, column=1, padx=2)
ttk.Label(el_frame, text="Y ->").grid(row=0, column=2)
manual_y_display = ttk.Label(el_frame, text="0.0", width=8)
manual_y_display.grid(row=0, column=3)


def update_xy_display(*args):
    try:
        az = float(manual_az_entry.get())
        el = float(manual_el_entry.get())
        manual_x_display.config(text=f"{Xangle(az, el):.2f}")
        manual_y_display.config(text=f"{Yangle(az, el):.2f}")
    except Exception:
        manual_x_display.config(text="0.0")
        manual_y_display.config(text="0.0")


manual_az_entry.bind("<KeyRelease>", update_xy_display)
manual_el_entry.bind("<KeyRelease>", update_xy_display)


def apply_azel():
    try:
        x = float(manual_x_display.cget("text"))
        y = float(manual_y_display.cget("text"))
        threading.Thread(
            target=control.move_absolute_pair,
            kwargs={"x_deg": x, "y_deg": y},
            daemon=True,
        ).start()
    except Exception as e:
        messagebox.showerror("Error", str(e))


ttk.Button(left_frame, text="Point using AZ/EL", command=apply_azel).pack(pady=5)
ttk.Button(left_frame, text="Open Manual Pointing Window", command=open_manual_pointing).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Velocity Step Test").pack(pady=5)

vel_ramp_frame = ttk.Frame(left_frame)
vel_ramp_frame.pack(fill="x")

ttk.Label(vel_ramp_frame, text="Axis").grid(row=0, column=0, padx=4, pady=2, sticky="w")
velocity_axis_var = tk.StringVar(value="x")
ttk.Combobox(vel_ramp_frame, textvariable=velocity_axis_var, values=("x", "y"), state="readonly", width=6).grid(row=0, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="Vel 1").grid(row=1, column=0, padx=4, pady=2, sticky="w")
vel_start_entry = ttk.Entry(vel_ramp_frame, width=8)
vel_start_entry.insert(0, "0.2")
vel_start_entry.grid(row=1, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="Vel 2").grid(row=2, column=0, padx=4, pady=2, sticky="w")
vel_target_entry = ttk.Entry(vel_ramp_frame, width=8)
vel_target_entry.insert(0, "0.8")
vel_target_entry.grid(row=2, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="Hold V1 (s)").grid(row=3, column=0, padx=4, pady=2, sticky="w")
vel_hold_v1_entry = ttk.Entry(vel_ramp_frame, width=8)
vel_hold_v1_entry.insert(0, "2")
vel_hold_v1_entry.grid(row=3, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="Hold V2 (s)").grid(row=4, column=0, padx=4, pady=2, sticky="w")
vel_hold_after_entry = ttk.Entry(vel_ramp_frame, width=8)
vel_hold_after_entry.insert(0, "2")
vel_hold_after_entry.grid(row=4, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="Native Ramp").grid(row=5, column=0, padx=4, pady=2, sticky="w")
vel_native_ramp_entry = ttk.Entry(vel_ramp_frame, width=8)
vel_native_ramp_entry.insert(0, "0")
vel_native_ramp_entry.grid(row=5, column=1, padx=4, pady=2)

ttk.Label(vel_ramp_frame, text="deg/s^2 (0 = off)").grid(row=5, column=2, padx=4, pady=2, sticky="w")

ttk.Label(
    vel_ramp_frame,
    text=(
        f"Safety: +/-{SAFE_VELOCITY_RAMP_MAX_DEG_S:.1f} deg/s, "
        f"stop near +/-{control.TRACKING_MAX_DEGREE:.0f} deg"
    ),
).grid(row=7, column=0, columnspan=2, padx=4, pady=(4, 2), sticky="w")

vel_ramp_button_row = ttk.Frame(left_frame)
vel_ramp_button_row.pack(pady=(4, 0))
ttk.Button(vel_ramp_button_row, text="Start Vel Step", command=start_velocity_ramp_test).pack(side="left", padx=4)
ttk.Button(vel_ramp_button_row, text="Stop Vel Step", command=stop_velocity_ramp_test).pack(side="left", padx=4)

plot_frame = LivePlotFrame(right_frame)
plot_frame.pack(fill="both", expand=True)

root.mainloop()
