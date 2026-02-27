import time

# ------------------ CONFIGURATION ------------------
GEAR_RATIO = 1020.0           # Motor turns per 1 output turn
POSITION_TOL = 0.001           # Motor turns tolerance for "settled"
VELOCITY_TOL = 0.001           # Motor turns/sec tolerance for "settled"

MAX_DEGREE = 85               # Mechanical soft limit
MIN_DEGREE = -85

SPI_HOME_RAW = 0.3442915678024292  # SPI encoder raw value at vertical (installation)
GO_TO_HOME_ON_STARTUP = True        # True = move to home on power-on

# ------------------ UTILITY FUNCTIONS ------------------
def raw_to_output_deg(raw, home_offset):
    """
    Convert SPI raw reading to degrees relative to home (vertical)
    """
    slope = -438.0  # calibrated scaling
    return slope * (raw - home_offset)

# ------------------ MAIN CONTROL ------------------
try:
    odrv0.clear_errors()

    # ---- Set controller gains ----
    odrv0.axis0.controller.config.pos_gain = 50.0
    odrv0.axis0.controller.config.vel_gain = 0.15
    odrv0.axis0.controller.config.vel_integrator_gain = 0.1

    # ---- Enter Closed Loop ----
    odrv0.axis0.requested_state = 8
    time.sleep(0.5)
    print("State:", odrv0.axis0.current_state)
    print("Axis error:", hex(odrv0.axis0.disarm_reason))

    # ---- Seed motor position relative to current SPI encoder ----
    output_abs_turns = odrv0.spi_encoder0.raw
    odrv0.axis0.pos_vel_mapper.set_abs_pos(output_abs_turns * GEAR_RATIO)
    startup_motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel

    # ---- Move to home position at startup (if enabled) ----
    if GO_TO_HOME_ON_STARTUP:
        # Calculate motor turns corresponding to 0° (vertical)
        target_motor_turns = startup_motor_pos + (0.0 / 360.0) * GEAR_RATIO
        odrv0.axis0.controller.input_pos = target_motor_turns

        # Wait until motion completes
        while True:
            motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
            motor_vel = odrv0.axis0.pos_vel_mapper.vel
            pos_error = abs(target_motor_turns - motor_pos)
            vel_abs = abs(motor_vel)
            if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
                time.sleep(0.1)
                break
            time.sleep(0.01)
        print("Startup move complete: Axis at home (vertical)")

    # ---- Store SPI offset at home ----
    spi_home_offset = odrv0.spi_encoder0.raw

    print("\n--- OUTPUT POSITION CONTROL ACTIVE ---")
    print(f"Allowed range: {MIN_DEGREE}° to {MAX_DEGREE}°")
    print("Enter desired output position in degrees (q to quit)")

    # ------------------ USER INPUT LOOP ------------------
    while True:
        user_input = input("\nTarget output degrees: ")
        if user_input.lower() == 'q':
            break

        try:
            target_output_deg = float(user_input)
        except ValueError:
            print("Invalid input. Enter a number in degrees.")
            continue

        # Enforce mechanical limits
        if target_output_deg > MAX_DEGREE:
            print(f"Exceeds max limit {MAX_DEGREE}°, clamping")
            target_output_deg = MAX_DEGREE
        elif target_output_deg < MIN_DEGREE:
            print(f"Exceeds min limit {MIN_DEGREE}°, clamping")
            target_output_deg = MIN_DEGREE  

        # Convert output degrees to output turns
        target_output_turns = target_output_deg / 360.0

        # Convert output turns to motor turns relative to startup
        target_motor_turns = startup_motor_pos + target_output_turns * GEAR_RATIO

        # Command motor
        odrv0.axis0.controller.input_pos = target_motor_turns

        # Wait until motion completes
        while True:
            motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
            motor_vel = odrv0.axis0.pos_vel_mapper.vel
            pos_error = abs(target_motor_turns - motor_pos)
            vel_abs = abs(motor_vel)
            if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
                time.sleep(0.1)  # settle
                break
            time.sleep(0.01)

        # Final readings
        motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
        motor_based_output = (motor_pos - startup_motor_pos) * 360 / GEAR_RATIO
        output_encoder = raw_to_output_deg(odrv0.spi_encoder0.raw, spi_home_offset)

        print("\nMove Complete:")
        print(f"Motor-based:      {motor_based_output:.3f}°")
        print(f"Output Encoder:   {output_encoder:.3f}°")
        print(f"Raw:              {odrv0.spi_encoder0.raw}")

except KeyboardInterrupt:
    print("\n[STOP] Disarming...")

finally:
    odrv0.axis0.requested_state = 1

