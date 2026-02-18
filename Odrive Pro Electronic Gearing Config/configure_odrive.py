import odrive
from odrive.enums import *
import time

print("Finding ODrive...")
odrv0 = odrive.find_any()
print(f"Connected! Serial Number: {hex(odrv0.serial_number)}")

# 1. ERASE PREVIOUS CONFIG (Optional but recommended for a clean start)
# odrv0.erase_configuration() 

# 2. MOTOR PARAMETERS (Found under axis0 > config > motor in Inspector)
odrv0.axis0.config.motor.pole_pairs = 7
odrv0.axis0.config.motor.torque_constant = 8.27 / 150  # 8.27 / <kV_rating>
odrv0.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL

# 3. CONTROLLER LIMITS (Found under axis0 > controller > config in Inspector)
odrv0.axis0.controller.config.vel_limit = 10.0  # turns/s
odrv0.axis0.motor.config.current_limit = 20.0  # Amps

# 4. TUNING PARAMETERS (Found under axis0 > controller > config in Inspector)
odrv0.axis0.controller.config.pos_gain = 20.0
odrv0.axis0.controller.config.vel_gain = 0.16
odrv0.axis0.controller.config.vel_integrator_gain = 0.32

# 5. SAVE AND REBOOT
print("Saving configuration...")
try:
    odrv0.save_configuration()
except Exception as e:
    # ODrive often disconnects during save/reboot, which is normal
    print("Rebooting ODrive...")

time.sleep(2)
print("Configuration complete.")
