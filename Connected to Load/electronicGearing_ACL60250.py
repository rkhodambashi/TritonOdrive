import time

# Constants
try:
    odrv0.clear_errors()
    # 1. CALIBRATION CHECK
    # Incremental encoders MUST be calibrated or indexed after power-up
    if not odrv0.axis0.is_homed:
        print("Motor not calibrated. Starting Encoder Offset Calibration...")
        odrv0.axis0.requested_state = 7 # ENCODER_OFFSET_CALIBRATION
        while odrv0.axis0.current_state != 1: # Wait for IDLE
            time.sleep(0.1)
        print("Calibration Complete. Waiting 2 seconds before Arming...")
        time.sleep(2.0) 

    # 3. INITIAL SYNC
    # Read hand position. Since it's BiSS-C, 'raw' is fine for 1-turn sync.
    last_hand_raw = odrv0.spi_encoder0.raw
    hand_offset = 0
    
    # Seed the motor's starting position to match the hand
    odrv0.axis0.pos_vel_mapper.set_abs_pos(last_hand_raw)
    print(f"Sync Complete: Starting at {last_hand_raw:.4f}")

    # 4. Arm the motor
    odrv0.axis0.requested_state = 8  # CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    print("--- HIGH-STRENGTH FOLLOWING ACTIVE ---")

    while True:
        # Read hand (BiSS-C)
        current_raw = odrv0.spi_encoder0.raw
        
        # Manual Multiturn Logic
        if (current_raw - last_hand_raw) < -0.5:
            hand_offset += 1
        elif (current_raw - last_hand_raw) > 0.5:
            hand_offset -= 1
        last_hand_raw = current_raw
        
        total_hand_pos = hand_offset + current_raw

        # Command the motor
        odrv0.axis0.controller.input_pos = total_hand_pos

        # Read actual motor position (from Incremental Encoder)
        motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel

        print(f"Hand: {total_hand_pos:.4f} | Motor: {motor_pos:.4f} | Diff: {total_hand_pos-motor_pos:.4f}", end='\r')
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n[STOP] Disarming...")
finally:
    odrv0.axis0.requested_state = 1
