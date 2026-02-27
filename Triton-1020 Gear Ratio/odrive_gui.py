import tkinter as tk
from tkinter import ttk
import threading
import odrive
import time

import OdrivePro_ACL60250_Motors_PositionInput as control

odrv0 = None


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


# ---------------- TRAJECTORY ----------------
def apply_traj():
    try:
        vel = float(traj_vel_entry.get())
        acc = float(traj_acc_entry.get())
        dec = float(traj_dec_entry.get())
        control.set_traj_params(vel, acc, dec)
    except:
        pass


# ---------------- GUI SETUP ----------------
root = tk.Tk()
root.title("ODrive Position Control GUI")
root.geometry("500x650")

main = ttk.Frame(root, padding=10)
main.pack(fill="both", expand=True)

# CONNECT
ttk.Button(main, text="Connect ODrive", command=connect_odrive).pack(pady=5)

status_label = ttk.Label(main, text="Not Connected")
status_label.pack()

ttk.Separator(main).pack(fill="x", pady=10)

# POSITION DISPLAY
ttk.Label(main, text="Current Position:").pack()
position_var = tk.StringVar(value="0.000 °")
ttk.Label(main, textvariable=position_var, font=("Arial", 16)).pack(pady=5)

ttk.Separator(main).pack(fill="x", pady=10)

# HOME BUTTON
ttk.Button(main, text="Go Home", command=go_home).pack(pady=5)

ttk.Separator(main).pack(fill="x", pady=10)

# RELATIVE MOVES
ttk.Label(main, text="Relative Move").pack(pady=5)

rel_frame = ttk.Frame(main)
rel_frame.pack()

ttk.Button(rel_frame, text="-10°", command=lambda: move_relative(-10)).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(rel_frame, text="-1°", command=lambda: move_relative(-1)).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(rel_frame, text="+1°", command=lambda: move_relative(1)).grid(row=0, column=2, padx=5, pady=5)
ttk.Button(rel_frame, text="+10°", command=lambda: move_relative(10)).grid(row=0, column=3, padx=5, pady=5)

ttk.Separator(main).pack(fill="x", pady=10)

# ABSOLUTE MOVE
ttk.Label(main, text="Absolute Move (°)").pack()
abs_entry = ttk.Entry(main)
abs_entry.pack(pady=5)
ttk.Button(main, text="Move Absolute", command=move_absolute).pack(pady=5)

ttk.Separator(main).pack(fill="x", pady=10)

# GAINS SECTION
ttk.Label(main, text="Controller Gains").pack(pady=5)

ttk.Label(main, text="Position Gain").pack()
pos_gain_entry = ttk.Entry(main)
pos_gain_entry.insert(0, "50")
pos_gain_entry.pack()

ttk.Label(main, text="Velocity Gain").pack()
vel_gain_entry = ttk.Entry(main)
vel_gain_entry.insert(0, "0.15")
vel_gain_entry.pack()

ttk.Label(main, text="Velocity Integrator Gain").pack()
vel_i_entry = ttk.Entry(main)
vel_i_entry.insert(0, "0.1")
vel_i_entry.pack()

ttk.Button(main, text="Apply Gains", command=apply_gains).pack(pady=5)

ttk.Separator(main).pack(fill="x", pady=10)

# TRAJECTORY SECTION
ttk.Label(main, text="Trajectory Parameters").pack(pady=5)

ttk.Label(main, text="Velocity Limit").pack()
traj_vel_entry = ttk.Entry(main)
traj_vel_entry.insert(0, "500")
traj_vel_entry.pack()

ttk.Label(main, text="Acceleration Limit").pack()
traj_acc_entry = ttk.Entry(main)
traj_acc_entry.insert(0, "1000")
traj_acc_entry.pack()

ttk.Label(main, text="Deceleration Limit").pack()
traj_dec_entry = ttk.Entry(main)
traj_dec_entry.insert(0, "1000")
traj_dec_entry.pack()

ttk.Button(main, text="Apply Trajectory", command=apply_traj).pack(pady=5)

root.mainloop()
