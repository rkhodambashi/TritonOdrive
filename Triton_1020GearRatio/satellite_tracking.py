# satellite_tracking.py
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from math import sin, cos, atan, sqrt, pi, acos
from skyfield.api import load, EarthSatellite, Topos, utc
import datetime
import threading
import time


EPS = 1e-10

# --- Conversion functions (from your VBA) ---
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
    norm = sqrt(xvec[1]**2 + xvec[2]**2)
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
            angleRad = pi/2
        elif xvec[1] < 0:
            angleRad = -pi/2
        else:
            angleRad = 0
    else:
        angleRad = atan(xvec[1]/xvec[2])
        if xvec[2] < 0:
            angleRad += pi
    return angleRad * 180.0 / pi

def Yangle(AZ, EL):
    kvec = Kvector(AZ, EL)
    xvec = Xvector(AZ, EL)
    dotProd = kvec[0]*xvec[0] + kvec[1]*xvec[1] + kvec[2]*xvec[2]
    dotProd = max(min(dotProd,1),-1)
    angleRad = acos(dotProd)
    if kvec[0] < 0:
        angleRad = -angleRad
    return angleRad * 180.0 / pi

# --- Update ODrive X Axis ---
# def update_odrive_x(odrv0, x_angle_deg, control):
#     try:
#         target_turns = control.MOTOR_HOME + x_angle_deg / 360.0 * control.GEAR_RATIO
#         odrv0.axis0.controller.input_pos = target_turns
#         odrv0.axis0.requested_state = 8  # ensure closed loop
#     except Exception as e:
#         print(f"Error updating ODrive: {e}")
# --- Update ODrive X Axis safely using your control module ---
def update_odrive_x(odrv0, x_angle_deg, control, el_deg=None, pre_move_seconds=60):
    """
    Move the positioner to x_angle_deg, using your control module.

    Parameters:
        odrv0: the ODrive object (unused here but kept for signature)
        x_angle_deg: target angle from calculation
        control: your OdrivePro_ACL60250_Motors_PositionInput module
        el_deg: current elevation (optional, to handle pre-move before horizon)
        pre_move_seconds: how many seconds before rise to start moving
    """
    # Clamp the angle to ±90° travel limit
    x_angle_deg = max(min(x_angle_deg, 90), -90)

    # If elevation is provided and below horizon, check if we should pre-move
    if el_deg is not None and el_deg < 0:
        # If the satellite is approaching horizon soon, start moving
        # This logic assumes you calculate next rise time elsewhere
        # Here, we just skip moving if far below horizon
        if el_deg < -30:  # ignore very low satellites to prevent swinging down
            return

    try:
        # Use your control module to move; it applies gear ratio and home offset
        control.move_absolute(x_angle_deg)
    except Exception as e:
        print(f"Error updating ODrive via control module: {e}")


# --- Satellite Tracking Window ---
class SatelliteTrackingWindow(tk.Toplevel):
    def __init__(self, parent, odrv0=None, control=None, observer_lat=0.0, observer_lon=0.0):
        super().__init__(parent)
        self.title("Satellite Tracking")
        self.geometry("650x500")

        self.location_label = ttk.Label(self, text="Location: not set")
        self.location_label.pack()

        self.odrv0 = odrv0
        self.control = control
        self.observer_lat = observer_lat
        self.observer_lon = observer_lon

        ttk.Label(self, text="Select TLE File:").pack(pady=5)
        ttk.Button(self, text="Browse", command=self.load_tle).pack(pady=5)
        self.tle_file_label = ttk.Label(self, text="No file selected")
        self.tle_file_label.pack(pady=5)

        ttk.Label(self, text="Select Satellite:").pack(pady=5)
        self.sat_combobox = ttk.Combobox(self, state="readonly", width=60)
        self.sat_combobox.pack(pady=5)

        ttk.Label(self, text="Observer Latitude (deg):").pack()
        self.lat_entry = ttk.Entry(self, width=20)
        self.lat_entry.pack()
        self.lat_entry.insert(0, str(self.observer_lat))

        ttk.Label(self, text="Observer Longitude (deg):").pack()
        self.lon_entry = ttk.Entry(self, width=20)
        self.lon_entry.pack()
        self.lon_entry.insert(0, str(self.observer_lon))

        ttk.Button(self, text="Start Tracking", command=self.start_tracking_thread).pack(pady=10)

        ttk.Button(self, text="Apply Location", command=self.apply_location).pack(pady=5)

        self.output_text = tk.Text(self, height=20, width=80)
        self.output_text.pack(pady=10)

        self.satellites = []  # list of EarthSatellite
        self.running = False
        self.tracking_thread = None

    def load_tle(self):
        from skyfield.api import utc  # ensure UTC-aware datetime

        file_path = filedialog.askopenfilename(filetypes=[("TLE Files", "*.txt"), ("All Files", "*.*")])
        if not file_path:
            return
        self.tle_file_label.config(text=file_path)

        with open(file_path, "r") as f:
            lines = f.readlines()

        # Each satellite is 3 lines (name + 2 TLE)
        self.satellites = []
        for i in range(0, len(lines)-2, 3):
            sat = EarthSatellite(lines[i+1].strip(), lines[i+2].strip(), lines[i].strip())
            self.satellites.append(sat)

        if not self.satellites:
            messagebox.showerror("Error", "No valid satellites found in TLE file")
            return

        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)
        now = ts.now()
        display_data = []

        for sat in self.satellites:
            difference = sat - observer
            topocentric = difference.at(now)
            alt, az, distance = topocentric.altaz()
            alt_deg = alt.degrees

            # compute rise events in next 24 hours
            t0 = ts.now()
            t1 = ts.from_datetime(datetime.datetime.utcnow().replace(tzinfo=utc) + datetime.timedelta(hours=24))

            try:
                times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)
                next_rise_minutes = None

                for ti, event in zip(times, events):
                    if event == 0:  # rise
                        # compute difference in minutes, all timezone-aware
                        dt = ti.utc_datetime().replace(tzinfo=utc) - datetime.datetime.utcnow().replace(tzinfo=utc)
                        next_rise_minutes = int(dt.total_seconds() / 60)
                        break

                if alt_deg >= 0:
                    status = f"VISIBLE now ({alt_deg:.1f}°)"
                    sort_key = 0
                elif next_rise_minutes is not None:
                    status = f"Rise in {next_rise_minutes} min ({alt_deg:.1f}°)"
                    sort_key = next_rise_minutes
                else:
                    status = f"No pass soon ({alt_deg:.1f}°)"
                    sort_key = 99999

            except Exception as e:
                # fallback if find_events fails
                status = f"Alt now: {alt_deg:.1f}°"
                sort_key = 99999

            display_data.append((sort_key, f"{sat.name} — {status}"))

        # sort satellites by soonest rise
        display_data.sort(key=lambda x: x[0])
        display_list = [item[1] for item in display_data]
        self.sat_combobox["values"] = display_list
        if display_list:
            self.sat_combobox.current(0)

        messagebox.showinfo("TLE Loaded", f"{len(self.satellites)} satellites loaded")

    def start_tracking_thread(self):
        if not self.satellites:
            messagebox.showerror("Error", "Load a TLE file first")
            return
        selected_index = self.sat_combobox.current()
        if selected_index < 0:
            messagebox.showerror("Error", "Select a satellite first")
            return
        self.running = True
        self.tracking_thread = threading.Thread(target=self.track_satellite_loop, args=(selected_index,), daemon=True)
        self.tracking_thread.start()

    def track_satellite_loop(self, sat_index):
        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)
        sat = self.satellites[sat_index]

        while self.running:
            t = ts.now()
            difference = sat - observer
            topocentric = difference.at(t)
            el, az, distance = topocentric.altaz()

            az_deg = az.degrees
            el_deg = el.degrees

            # compute next rise event
            t0 = ts.now()
            t1 = ts.from_datetime(datetime.datetime.now(utc) + datetime.timedelta(hours=2))

            times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)

            next_rise = None
            for ti, event in zip(times, events):
                if event == 0:
                    next_rise = ti.utc_datetime().replace(tzinfo=utc)
                    break

            preposition = False

            if next_rise:
                seconds_to_rise = (next_rise - datetime.datetime.now(utc)).total_seconds()

                if 0 < seconds_to_rise < 60:
                    preposition = True

            x_angle = Xangle(az_deg, el_deg)
            y_angle = Yangle(az_deg, el_deg)

            # Update GUI text even if below horizon
            self.output_text.delete("1.0", tk.END)
            self.output_text.insert(tk.END, f"Satellite: {sat.name}\n")
            self.output_text.insert(tk.END, f"AZ: {az_deg:.3f}, EL: {el_deg:.3f}\n")
            self.output_text.insert(tk.END, f"X Angle: {x_angle:.3f}, Y Angle: {y_angle:.3f}\n")
            self.output_text.see(tk.END)
            self.output_text.update()  # <-- force GUI refresh

            # Move to horizon if not visible
            # if el_deg < 0:
            #     if self.odrv0 and self.control:
            #         update_odrive_x(self.odrv0, x_angle, self.control)
            #     # wait 1s before checking again
            #     time.sleep(1)
            #     continue
            if el_deg < 0:

                # move to horizon position ~45s before rise
                if preposition:
                    if self.odrv0 and self.control:
                        # update_odrive_x(self.odrv0, x_angle, self.control)
                        update_odrive_x(
                            self.odrv0,
                            x_angle,
                            self.control,
                            el_deg=el_deg,
                            pre_move_seconds=30  # start moving 30 seconds before rise
                        )

                time.sleep(1)
                continue

            # # Update GUI text
            # self.output_text.delete("1.0", tk.END)
            # self.output_text.insert(tk.END, f"Satellite: {sat.name}\n")
            # self.output_text.insert(tk.END, f"AZ: {az_deg:.3f}, EL: {el_deg:.3f}\n")
            # self.output_text.insert(tk.END, f"X Angle: {x_angle:.3f}, Y Angle: {y_angle:.3f}\n")
            # self.output_text.see(tk.END)

            # Move ODrive X-axis
            if self.odrv0 and self.control:
                # update_odrive_x(self.odrv0, x_angle, self.control)
                update_odrive_x(
                    self.odrv0,
                    x_angle,
                    self.control,
                    el_deg=el_deg
                )

            time.sleep(0.5)  # update every 0.5s

    def apply_location(self):
        try:
            self.observer_lat = float(self.lat_entry.get())
            self.observer_lon = float(self.lon_entry.get())

            self.location_label.config(
            text=f"Location: {self.observer_lat:.4f}, {self.observer_lon:.4f}"
            )
            messagebox.showinfo(
                "Location Set",
                f"Observer location set to:\nLat {self.observer_lat}\nLon {self.observer_lon}"
            )

        except:
            messagebox.showerror("Error", "Invalid latitude or longitude")

if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()
    SatelliteTrackingWindow(root)
    root.mainloop()