import time

GEAR_RATIO = 2040.0           # motor turns per 1 output turn
POSITION_TOL = 0.01           # motor turns tolerance for "settled"
VELOCITY_TOL = 0.01           # motor turns/sec tolerance for "settled"

# Mechanical limits in output turns
MAX_DEGREE = 85
MIN_DEGREE = -85
MAX_OUTPUT_TURNS = MAX_DEGREE / 360.0
MIN_OUTPUT_TURNS = MIN_DEGREE / 360.0

try:
    odrv0.clear_errors()

    # ---- Calibration settings ----
    odrv0.axis0.config.calibration_lockin.current = 15.0
    odrv0.axis0.config.calibration_lockin.vel = 2.0
    

    # ---- Always calibrate on startup ----
    print("Starting Encoder Offset Calibration...")
    odrv0.axis0.requested_state = 7  # ENCODER_OFFSET_CALIBRATION
    while odrv0.axis0.current_state != 1:
        time.sleep(0.1)
    print("Calibration Complete.")
    time.sleep(2.0)

    # ---- Gains ----
    odrv0.axis0.controller.config.pos_gain = 120.0
    odrv0.axis0.controller.config.vel_gain = 0.05
    odrv0.axis0.controller.config.vel_integrator_gain = 0.1

    # ---- Enter Closed Loop ----
    odrv0.axis0.requested_state = 8
    time.sleep(0.5)

    # ---- Use absolute encoder to seed motor zero ----
    output_abs_turns = odrv0.spi_encoder0.raw  # 1-turn units
    odrv0.axis0.pos_vel_mapper.set_abs_pos(output_abs_turns * GEAR_RATIO)

    startup_motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel

    print("\n--- OUTPUT POSITION CONTROL ACTIVE ---")
    print(f"Allowed range: {MIN_OUTPUT_TURNS:.3f} to {MAX_OUTPUT_TURNS:.3f} turns")
    print("Enter desired output position in turns (q to quit)")

    while True:
        user_input = input("\nTarget output turns: ")
        if user_input.lower() == 'q':
            break

        target_output_turns = float(user_input)

        # Enforce mechanical limits
        if target_output_turns > MAX_OUTPUT_TURNS:
            print(f"Exceeds max limit {MAX_OUTPUT_TURNS:.3f}, clamping")
            target_output_turns = MAX_OUTPUT_TURNS
        elif target_output_turns < MIN_OUTPUT_TURNS:
            print(f"Exceeds min limit {MIN_OUTPUT_TURNS:.3f}, clamping")
            target_output_turns = MIN_OUTPUT_TURNS

        # Convert output turns to motor turns, relative to startup
        target_motor_turns = startup_motor_pos + target_output_turns * GEAR_RATIO

        # Command motor
        odrv0.axis0.controller.input_pos = target_motor_turns

        # ---- Wait until motion completes ----
        while True:
            motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
            motor_vel = odrv0.axis0.pos_vel_mapper.vel
            pos_error = abs(target_motor_turns - motor_pos)
            vel_abs = abs(motor_vel)

            if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
                time.sleep(0.1)  # settle
                break
            time.sleep(0.01)

        # ---- Final readings ----
        motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
        motor_based_output = (motor_pos - startup_motor_pos) / GEAR_RATIO
        output_encoder = odrv0.spi_encoder0.raw

        print("\nMove Complete:")
        print(f"Motor/2040:      {motor_based_output:.6f} turns")
        print(f"Output Encoder:  {output_encoder:.6f} turns")

except KeyboardInterrupt:
    print("\n[STOP] Disarming...")

finally:
    odrv0.axis0.requested_state = 1
