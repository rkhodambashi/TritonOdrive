import time
import math

# --- Configuration ---
GEAR_RATIO = 2040.0
SWEEP_DEGREES = 5.0
# Calculate motor turns required for the output to move 5 degrees
MOTOR_SWEEP_TURNS = (SWEEP_DEGREES / 360.0) * GEAR_RATIO # Result: 28.333
OSCILLATION_SPEED = 0.1 # Hz (One full back-and-forth every 2 seconds)

try:
    odrv0.clear_errors()

    # 1. CALIBRATION CHECK
    if not odrv0.axis0.is_homed:
        print("Calibrating...")
        odrv0.axis0.requested_state = 7 
        while odrv0.axis0.current_state != 1: time.sleep(0.1)

    # 2. INITIAL SYNC
    # Use raw for 1-turn hand sync (1 hand turn = 1 motor turn)
    start_hand = odrv0.spi_encoder0.raw
    hand_offset = 0
    last_hand_raw = start_hand

    # Seed motor to match hand
    odrv0.axis0.pos_vel_mapper.set_abs_pos(start_hand)
    
    # SOFT ARMING: Start with low gains to protect the gearbox
    original_pos_gain = 40.0 # Adjust as needed for stiffness
    odrv0.axis0.controller.config.pos_gain = 5.0 
    odrv0.axis0.requested_state = 8
    time.sleep(1.0)
    odrv0.axis0.controller.config.pos_gain = original_pos_gain

    print("--- 5° OSCILLATION + HAND FOLLOWING ACTIVE ---")
    start_time = time.time()

    while True:
        # A. Hand Tracking Logic (1:1 with motor shaft)
        current_raw = odrv0.spi_encoder0.raw
        if (current_raw - last_hand_raw) < -0.5: hand_offset += 1
        elif (current_raw - last_hand_raw) > 0.5: hand_offset -= 1
        last_hand_raw = current_raw
        
        hand_pos = hand_offset + current_raw

        # B. Oscillation Logic (Sine wave for smoothness)
        elapsed = time.time() - start_time
        # sin(t) gives -1 to 1; multiply by sweep turns
        oscillation = math.sin(elapsed * 2 * math.pi * OSCILLATION_SPEED) * MOTOR_SWEEP_TURNS

        # C. Combined Command
        # Motor follows hand but adds the 5-degree load sweep on top
        odrv0.axis0.controller.input_pos = hand_pos + oscillation

        # D. Feedback
        motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
        load_degrees = (motor_pos / GEAR_RATIO) * 360.0
        print(f"Load Angle: {load_degrees:.2f}° | Motor Turns: {motor_pos:.2f}", end='\r')

        error = (hand_pos + oscillation) - motor_pos
        print(f"Target: {hand_pos + oscillation:.2f} | Actual: {motor_pos:.2f} | Error: {error:.2f}", end='\r')

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    odrv0.axis0.requested_state = 1
