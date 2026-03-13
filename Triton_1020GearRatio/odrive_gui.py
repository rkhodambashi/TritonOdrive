#odrive_gui.py
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import odrive
import time

import OdrivePro_ACL60250_Motors_PositionInput as control

import csv
from tkinter import filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from satellite_tracking import SatelliteTrackingWindow

from satellite_tracking import Xangle, Yangle

odrv0 = None

logging_active = False
log_thread = None

log_time = []
log_cmd_deg = []
log_spi_deg = []


# ---------------- CONNECT ----------------
def connect_odrive():
    global odrv0
    status_label.config(text="Connecting...")
    root.update()

    try:
        odrv0 = odrive.find_any()
        control.initialize(odrv0)
        status_label.config(text="Connected & Initialized")
        update_position_loop()
    except Exception as e:
        status_label.config(text=f"Connection Failed: {e}")


# ---------------- POSITION DISPLAY ----------------
def update_position_loop():
    try:
        pos = control.get_current_position()
        position_var.set(f"{pos:.3f} °")

        if odrv0 is not None:
            spi_raw_var.set(f"{odrv0.spi_encoder0.raw:.6f}")
            motor_raw_var.set(f"{odrv0.axis0.pos_vel_mapper.pos_rel:.6f}")

    except:
        pass

    root.after(100, update_position_loop)


# ---------------- MOVEMENT ----------------
def go_home():
    threading.Thread(target=control.go_home).start()


def move_relative(delta):
    threading.Thread(target=control.move_relative, args=(delta,)).start()


def move_absolute():
    try:
        deg = float(abs_entry.get())
        threading.Thread(target=control.move_absolute, args=(deg,)).start()
    except:
        pass


# ---------------- GAINS ----------------
def apply_gains():
    try:
        p = float(pos_gain_entry.get())
        v = float(vel_gain_entry.get())
        vi = float(vel_i_entry.get())
        control.set_gains(p, v, vi)
    except:
        pass


def apply_gains_small():
    try:
        p = float(pos_gain_entry_small.get())
        v = float(vel_gain_entry_small.get())
        vi = float(vel_i_entry_small.get())
        control.set_gains(p, v, vi)
    except:
        pass


# ---------------- TRAJECTORY ----------------
def apply_traj():
    try:
        vel = float(traj_vel_entry.get())
        acc = float(traj_acc_entry.get())
        dec = float(traj_dec_entry.get())
        control.set_traj_params(vel, acc, dec)
    except:
        pass

def open_satellite_tracking():
    #import satellite_tracking
    # pass the currently connected odrive object
    SatelliteTrackingWindow(root, odrv0=odrv0, control=control, observer_lat=0.0, observer_lon=0.0)

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

            control.move_absolute(x)

        except Exception as e:
            messagebox.showerror("Error", str(e))

    def point_from_xy():
        try:
            x = float(x_entry.get())
            y = float(y_entry.get())

            control.move_absolute(x)

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
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Commanded vs SPI Output")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Degrees")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def start_logging(self):
        global logging_active

        if logging_active:
            return

        log_time.clear()
        log_cmd_deg.clear()
        log_spi_deg.clear()
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
            cmd_deg = control.get_current_position()
            spi_deg = control.raw_to_output_deg(odrv0.spi_encoder0.raw, control.spi_home_offset)

            log_time.append(now)
            log_cmd_deg.append(cmd_deg)
            log_spi_deg.append(spi_deg)

            self.update_plot()
        except:
            pass

        interval_ms = int(self.interval_var.get() * 1000)
        self.after(interval_ms, self.logging_loop)

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(log_time, log_cmd_deg)
        self.ax.plot(log_time, log_spi_deg)
        self.ax.set_title("Commanded vs SPI Output")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Degrees")
        self.ax.legend(["Commanded (Trajectory)", "SPI Output"])
        self.canvas.draw()

    def save_csv(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".csv")
        if not file_path:
            return

        with open(file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time (s)", "Commanded (deg)", "SPI (deg)", "Error (deg)"])
            for i in range(len(log_time)):
                error_deg = log_cmd_deg[i] - log_spi_deg[i]
                writer.writerow([log_time[i], log_cmd_deg[i], log_spi_deg[i], error_deg])


# ---------------- GUI SETUP ----------------
root = tk.Tk()
root.title("ODrive Position Control GUI")
root.geometry("1000x650")

main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True)

left_frame = ttk.Frame(main_frame, width=300)
left_frame.pack(side="left", fill="y", padx=10, pady=10)

right_frame = ttk.Frame(main_frame)
right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

ttk.Button(left_frame, text="Connect ODrive", command=connect_odrive).pack(pady=5)

status_label = ttk.Label(left_frame, text="Not Connected")
status_label.pack()

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Current Position:").pack()
position_var = tk.StringVar(value="0.000 °")
ttk.Label(left_frame, textvariable=position_var, font=("Arial", 16)).pack(pady=5)

# --- RAW ENCODER VALUES ---
ttk.Label(left_frame, text="Motor Encoder (turns)").pack()
motor_raw_var = tk.StringVar(value="0")
ttk.Label(left_frame, textvariable=motor_raw_var).pack()

ttk.Label(left_frame, text="SPI Encoder (turns)").pack()
spi_raw_var = tk.StringVar(value="0")
ttk.Label(left_frame, textvariable=spi_raw_var).pack()

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Button(left_frame, text="Go Home", command=go_home).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Relative Move").pack(pady=5)

rel_frame = ttk.Frame(left_frame)
rel_frame.pack()

ttk.Button(rel_frame, text="-10°", command=lambda: move_relative(-10)).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(rel_frame, text="-1°", command=lambda: move_relative(-1)).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(rel_frame, text="+1°", command=lambda: move_relative(1)).grid(row=0, column=2, padx=5, pady=5)
ttk.Button(rel_frame, text="+10°", command=lambda: move_relative(10)).grid(row=0, column=3, padx=5, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

ttk.Label(left_frame, text="Absolute Move (°)").pack()
abs_entry = ttk.Entry(left_frame)
abs_entry.pack(pady=5)
ttk.Button(left_frame, text="Move Absolute", command=move_absolute).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

# ---------------- GAINS SECTION ----------------
ttk.Label(left_frame, text="Controller Gains").pack(pady=5)

gains_frame = ttk.Frame(left_frame)
gains_frame.pack()

# First gain set
ttk.Label(gains_frame, text="Position Gain").grid(row=0, column=0)
pos_gain_entry = ttk.Entry(gains_frame, width=8)
pos_gain_entry.insert(0, "50")
pos_gain_entry.grid(row=0, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=1, column=0)
vel_gain_entry = ttk.Entry(gains_frame, width=8)
vel_gain_entry.insert(0, "0.15")
vel_gain_entry.grid(row=1, column=1, padx=5)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=2, column=0)
vel_i_entry = ttk.Entry(gains_frame, width=8)
vel_i_entry.insert(0, "1")
vel_i_entry.grid(row=2, column=1, padx=5)

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains).grid(row=3, column=0, columnspan=2, pady=5)

# Second gain set
ttk.Label(gains_frame, text="Position Gain").grid(row=0, column=2, padx=10)
pos_gain_entry_small = ttk.Entry(gains_frame, width=8)
pos_gain_entry_small.insert(0, "100")
pos_gain_entry_small.grid(row=0, column=3)

ttk.Label(gains_frame, text="Velocity Gain").grid(row=1, column=2)
vel_gain_entry_small = ttk.Entry(gains_frame, width=8)
vel_gain_entry_small.insert(0, "0.2")
vel_gain_entry_small.grid(row=1, column=3)

ttk.Label(gains_frame, text="Velocity Integrator Gain").grid(row=2, column=2)
vel_i_entry_small = ttk.Entry(gains_frame, width=8)
vel_i_entry_small.insert(0, "5")
vel_i_entry_small.grid(row=2, column=3)

ttk.Button(gains_frame, text="Apply Gains", command=apply_gains_small).grid(row=3, column=2, columnspan=2, pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

# ---------------- TRAJECTORY SECTION ----------------
ttk.Label(left_frame, text="Trajectory Parameters").pack(pady=5)

ttk.Label(left_frame, text="Velocity Limit").pack()
traj_vel_entry = ttk.Entry(left_frame)
traj_vel_entry.insert(0, "50")
traj_vel_entry.pack()

ttk.Label(left_frame, text="Acceleration Limit").pack()
traj_acc_entry = ttk.Entry(left_frame)
traj_acc_entry.insert(0, "100")
traj_acc_entry.pack()

ttk.Label(left_frame, text="Deceleration Limit").pack()
traj_dec_entry = ttk.Entry(left_frame)
traj_dec_entry.insert(0, "100")
traj_dec_entry.pack()

ttk.Button(left_frame, text="Apply Trajectory", command=apply_traj).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)

# ---------------- TRACKING SECTION ----------------
ttk.Label(left_frame, text="Tracking GUI").pack(pady=5)
#ttk.Button(left_frame, text="Satellite Tracking", command=lambda: SatelliteTrackingWindow(root)).pack(pady=10)
ttk.Button(left_frame, text="Satellite Tracking", command=open_satellite_tracking).pack(pady=5)

ttk.Separator(left_frame).pack(fill="x", pady=10)
# ---------------- MANUAL POINTING SECTION ----------------
ttk.Label(left_frame, text="Manual Pointing").pack(pady=5)

# AZ/EL inputs with X/Y display
az_frame = ttk.Frame(left_frame)
az_frame.pack(pady=2, fill="x")
ttk.Label(az_frame, text="Azimuth (°)").grid(row=0, column=0)
manual_az_entry = ttk.Entry(az_frame, width=10)
manual_az_entry.grid(row=0, column=1, padx=2)
ttk.Label(az_frame, text="X →").grid(row=0, column=2)
manual_x_display = ttk.Label(az_frame, text="0.0", width=8)
manual_x_display.grid(row=0, column=3)

el_frame = ttk.Frame(left_frame)
el_frame.pack(pady=2, fill="x")
ttk.Label(el_frame, text="Elevation (°)").grid(row=0, column=0)
manual_el_entry = ttk.Entry(el_frame, width=10)
manual_el_entry.grid(row=0, column=1, padx=2)
ttk.Label(el_frame, text="Y →").grid(row=0, column=2)
manual_y_display = ttk.Label(el_frame, text="0.0", width=8)
manual_y_display.grid(row=0, column=3)

def update_xy_display(*args):
    try:
        az = float(manual_az_entry.get())
        el = float(manual_el_entry.get())
        x = Xangle(az, el)
        y = Yangle(az, el)
        manual_x_display.config(text=f"{x:.2f}")
        manual_y_display.config(text=f"{y:.2f}")
    except:
        manual_x_display.config(text="0.0")
        manual_y_display.config(text="0.0")

manual_az_entry.bind("<KeyRelease>", update_xy_display)
manual_el_entry.bind("<KeyRelease>", update_xy_display)

def apply_azel():
    try:
        x = float(manual_x_display.cget("text"))
        y = float(manual_y_display.cget("text"))
        control.move_absolute(x)
        # optionally move Y if second axis exists
    except Exception as e:
        messagebox.showerror("Error", str(e))

ttk.Button(left_frame, text="Point using AZ/EL", command=apply_azel).pack(pady=5)

plot_frame = LivePlotFrame(right_frame)
plot_frame.pack(fill="both", expand=True)

root.mainloop()
