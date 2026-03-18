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
        devices = control.connect_all()
        axis_devices.update(devices)
        status_label.config(
            text=(
                "Connected & Initialized\n"
                f"X: {control.get_serial_number('x')}\n"
                f"Y: {control.get_serial_number('y')}"
            )
        )
        refresh_config_entries()
        update_position_loop()
    except Exception as e:
        status_label.config(text=f"Connection Failed: {e}")


def update_position_loop():
    x_position = _safe_call(lambda: control.get_current_position("x"))
    x_spi_raw = _safe_call(lambda: control.get_spi_raw("x"))
    x_motor_raw = _safe_call(lambda: control.get_motor_raw("x"))
    x_state = _safe_call(lambda: control.get_axis_state("x"))
    x_active_errors = _safe_call(lambda: control.get_active_errors("x"))
    x_disarm = _safe_call(lambda: control.get_disarm_reason("x"))
    x_procedure_result = _safe_call(lambda: control.get_procedure_result("x"))
    x_input_pos = _safe_call(lambda: control.get_input_pos("x"))
    x_velocity = _safe_call(lambda: control.get_motor_velocity("x"))

    if x_position is not None:
        x_position_var.set(f"{x_position:.3f} deg")
    if x_spi_raw is not None:
        x_spi_raw_var.set(f"{x_spi_raw:.6f}")
    if x_motor_raw is not None:
        x_motor_raw_var.set(f"{x_motor_raw:.6f}")

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
        f"Motor: {f'{x_motor_raw:.6f}' if x_motor_raw is not None else '-'}   "
        f"Vel: {f'{x_velocity:.6f}' if x_velocity is not None else '-'}   "
        f"Angle: {f'{x_position:.3f}' if x_position is not None else '-'}"
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

    if y_position is not None:
        y_position_var.set(f"{y_position:.3f} deg")
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
        f"Motor: {f'{y_motor_raw:.6f}' if y_motor_raw is not None else '-'}   "
        f"Vel: {f'{y_velocity:.6f}' if y_velocity is not None else '-'}   "
        f"Angle: {f'{y_position:.3f}' if y_position is not None else '-'}"
    )

    root.after(100, update_position_loop)


def go_home(axis):
    threading.Thread(target=control.go_home, kwargs={"axis": axis}, daemon=True).start()


def move_relative(axis, delta):
    threading.Thread(
        target=control.move_relative,
        kwargs={"delta_deg": delta, "axis": axis},
        daemon=True,
    ).start()


def move_absolute(axis, entry):
    try:
        deg = float(entry.get())
        threading.Thread(
            target=control.move_absolute,
            kwargs={"target_output_deg": deg, "axis": axis},
            daemon=True,
        ).start()
    except Exception:
        pass


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


def open_satellite_tracking():
    SatelliteTrackingWindow(root, odrvs=axis_devices, control=control, observer_lat=33.677, observer_lon=-112.093)


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

        self.fig = Figure(figsize=(8, 6), dpi=100)
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
root.geometry("1100x760")

main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True)

left_panel = ttk.Frame(main_frame, width=390)
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
x_diag_var = tk.StringVar(value="Cmd: -   Motor: -   Vel: -   Angle: -")
ttk.Label(x_status_frame, textvariable=x_diag_var).grid(row=2, column=0, columnspan=2, sticky="w")

y_status_frame = ttk.Frame(left_frame)
y_status_frame.pack(fill="x", pady=2)
ttk.Label(y_status_frame, text="Y Axis").grid(row=0, column=0, sticky="w")
y_position_var = tk.StringVar(value="0.000 deg")
ttk.Label(y_status_frame, textvariable=y_position_var, font=("Arial", 14)).grid(row=0, column=1, sticky="w", padx=8)
y_fault_var = tk.StringVar(value="State: -   Disarm: -   Active: -   Proc: -")
ttk.Label(y_status_frame, textvariable=y_fault_var).grid(row=1, column=0, columnspan=2, sticky="w")
y_diag_var = tk.StringVar(value="Cmd: -   Motor: -   Vel: -   Angle: -")
ttk.Label(y_status_frame, textvariable=y_diag_var).grid(row=2, column=0, columnspan=2, sticky="w")

raw_frame = ttk.Frame(left_frame)
raw_frame.pack(fill="x", pady=5)

ttk.Label(raw_frame, text="Axis").grid(row=0, column=0, padx=5)
ttk.Label(raw_frame, text="Motor Encoder (turns)").grid(row=0, column=1, padx=5)
ttk.Label(raw_frame, text="SPI Encoder (turns)").grid(row=0, column=2, padx=5)

ttk.Label(raw_frame, text="X").grid(row=1, column=0)
x_motor_raw_var = tk.StringVar(value="0")
x_spi_raw_var = tk.StringVar(value="0")
ttk.Label(raw_frame, textvariable=x_motor_raw_var).grid(row=1, column=1)
ttk.Label(raw_frame, textvariable=x_spi_raw_var).grid(row=1, column=2)

ttk.Label(raw_frame, text="Y").grid(row=2, column=0)
y_motor_raw_var = tk.StringVar(value="0")
y_spi_raw_var = tk.StringVar(value="0")
ttk.Label(raw_frame, textvariable=y_motor_raw_var).grid(row=2, column=1)
ttk.Label(raw_frame, textvariable=y_spi_raw_var).grid(row=2, column=2)

ttk.Separator(left_frame).pack(fill="x", pady=10)

home_frame = ttk.Frame(left_frame)
home_frame.pack(fill="x")
ttk.Button(home_frame, text="Go Home X", command=lambda: go_home("x")).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(home_frame, text="Go Home Y", command=lambda: go_home("y")).grid(row=0, column=1, padx=5, pady=5)

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

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains).grid(row=4, column=0, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Position Gain").grid(row=1, column=2, padx=10)
pos_gain_entry_small = ttk.Entry(gains_frame, width=8)
pos_gain_entry_small.insert(0, "100")
pos_gain_entry_small.grid(row=1, column=3)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=2, column=2)
vel_gain_entry_small = ttk.Entry(gains_frame, width=8)
vel_gain_entry_small.insert(0, "0.2")
vel_gain_entry_small.grid(row=2, column=3)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=3, column=2)
vel_i_entry_small = ttk.Entry(gains_frame, width=8)
vel_i_entry_small.insert(0, "5")
vel_i_entry_small.grid(row=3, column=3)

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains_small).grid(row=4, column=2, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Y Axis").grid(row=5, column=0, columnspan=4, pady=(10, 4))

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

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains_y).grid(row=9, column=0, columnspan=2, pady=5)

ttk.Label(gains_frame, text="Position Gain").grid(row=6, column=2, padx=10)
pos_gain_entry_small_y = ttk.Entry(gains_frame, width=8)
pos_gain_entry_small_y.insert(0, "100")
pos_gain_entry_small_y.grid(row=6, column=3)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=7, column=2)
vel_gain_entry_small_y = ttk.Entry(gains_frame, width=8)
vel_gain_entry_small_y.insert(0, "0.2")
vel_gain_entry_small_y.grid(row=7, column=3)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=8, column=2)
vel_i_entry_small_y = ttk.Entry(gains_frame, width=8)
vel_i_entry_small_y.insert(0, "5")
vel_i_entry_small_y.grid(row=8, column=3)

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains_small_y).grid(row=9, column=2, columnspan=2, pady=5)

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

plot_frame = LivePlotFrame(right_frame)
plot_frame.pack(fill="both", expand=True)

root.mainloop()
