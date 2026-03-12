# satellite_tracking.py
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from math import sin, cos, atan, sqrt, pi, acos
from skyfield.api import load, EarthSatellite, Topos, utc
import datetime
import threading
import time

EPS = 1e-10

# --- Conversion functions ---
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


# --- ODrive control wrapper ---
def update_odrive_x(odrv0, x_angle_deg, control):
    x_angle_deg = max(min(x_angle_deg, 90), -90)
    try:
        control.move_absolute(x_angle_deg)
    except Exception as e:
        print(f"ODrive move error: {e}")


class SatelliteTrackingWindow(tk.Toplevel):

    def __init__(self, parent, odrv0=None, control=None, observer_lat=33.67, observer_lon=-112.09): #therse are default lat/lon for Phoenix, AZ
        super().__init__(parent)

        self.title("Satellite Tracking")
        self.geometry("650x520")

        self.odrv0 = odrv0
        self.control = control

        self.observer_lat = observer_lat
        self.observer_lon = observer_lon

        self.running = False
        self.tracking_thread = None
        self.current_sat_index = None

        self.satellites = []
        self.display_map = []

        self.location_label = ttk.Label(self, text="Location: not set")
        self.location_label.pack()

        ttk.Label(self, text="Observer Latitude (deg):").pack()
        self.lat_entry = ttk.Entry(self, width=20)
        self.lat_entry.pack()
        self.lat_entry.insert(0, str(self.observer_lat))

        ttk.Label(self, text="Observer Longitude (deg):").pack()
        self.lon_entry = ttk.Entry(self, width=20)
        self.lon_entry.pack()
        self.lon_entry.insert(0, str(self.observer_lon))

        ttk.Button(self, text="Apply Location", command=self.apply_location).pack(pady=5)

        ttk.Label(self, text="Select TLE File:").pack(pady=5)
        ttk.Button(self, text="Browse", command=self.load_tle).pack(pady=5)

        self.tle_file_label = ttk.Label(self, text="No file selected")
        self.tle_file_label.pack(pady=5)

        ttk.Label(self, text="Select Satellite:").pack(pady=5)
        self.sat_combobox = ttk.Combobox(self, state="readonly", width=60)
        self.sat_combobox.pack(pady=5)

        ttk.Button(self, text="Start Tracking", command=self.start_tracking_thread).pack(pady=8)
        ttk.Button(self, text="Stop Tracking / Stow", command=self.stop_tracking).pack(pady=5)

        self.output_text = tk.Text(self, height=18, width=80)
        self.output_text.pack(pady=10)


    def load_tle(self):

        file_path = filedialog.askopenfilename(filetypes=[("TLE Files", "*.txt"), ("All Files", "*.*")])
        if not file_path:
            return

        self.tle_file_label.config(text=file_path)

        with open(file_path, "r") as f:
            lines = f.readlines()

        self.satellites = []

        for i in range(0, len(lines)-2, 3):
            sat = EarthSatellite(lines[i+1].strip(), lines[i+2].strip(), lines[i].strip())
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
                    minutes = int(dt.total_seconds()/60)
                    label = f"{sat.name} — Rise in {minutes} min"
                    sort_key = minutes
                else:
                    label = f"{sat.name} — No pass soon"
                    sort_key = 99999

            except:
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

        # If already tracking something
        if self.running:

            if sat_index == self.current_sat_index:
                return

            result = messagebox.askyesno(
                "Switch Satellite",
                "Tracking already in progress.\n\nDo you want to switch satellites?"
            )

            if not result:
                return

            # stop current tracking thread
            self.running = False
            time.sleep(0.6)

        self.current_sat_index = sat_index
        self.running = True

        self.tracking_thread = threading.Thread(
            target=self.track_satellite_loop,
            args=(sat_index,),
            daemon=True
        )

        self.tracking_thread.start()


    def stop_tracking(self):

        self.running = False
        self.current_sat_index = None

        if self.control:
            try:
                self.control.move_absolute(0)  # STOW
            except:
                pass


    def track_satellite_loop(self, sat_index):

        ts = load.timescale()
        observer = Topos(latitude_degrees=self.observer_lat, longitude_degrees=self.observer_lon)

        sat = self.satellites[sat_index]

        was_visible = False

        while self.running and sat_index == self.current_sat_index:

            t = ts.now()

            difference = sat - observer
            topocentric = difference.at(t)

            el, az, distance = topocentric.altaz()

            az_deg = az.degrees
            el_deg = el.degrees

            x_angle = Xangle(az_deg, el_deg)
            y_angle = Yangle(az_deg, el_deg)

            self.output_text.delete("1.0", tk.END)

            self.output_text.insert(tk.END, f"Satellite: {sat.name}\n")
            self.output_text.insert(tk.END, f"AZ: {az_deg:.3f}\n")
            self.output_text.insert(tk.END, f"EL: {el_deg:.3f}\n")
            self.output_text.insert(tk.END, f"X Angle: {x_angle:.3f}\n")
            self.output_text.insert(tk.END, f"Y Angle: {y_angle:.3f}\n")

            self.output_text.update()

            if el_deg >= 0:

                was_visible = True

                if self.odrv0 and self.control:
                    update_odrive_x(self.odrv0, x_angle, self.control)

            else:

                if was_visible:
                    # satellite finished pass -> STOW
                    if self.control:
                        try:
                            self.control.move_absolute(0)
                        except:
                            pass

                    self.running = False
                    break

            time.sleep(0.5)


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