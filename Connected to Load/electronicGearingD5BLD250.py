 import time
 # Constants
 RESOLUTION = 2**23  # For 23-bit BiSS-C
 try:
     # 1. Clear Errors
     odrv0.clear_errors()

     # 2. SYNC MOTOR TO HAND BEFORE ARMING (Corrected for v0.6.11)
     # We read the current hand position
     current_hand = odrv0.spi_encoder0.raw

     # We force the motor's internal position estimate to match the hand
     # This ensures (Hand - Motor) = 0 before the motor starts moving
     odrv0.axis0.pos_vel_mapper.set_abs_pos(current_hand)
     print(f"Sync Complete: Motor set to {current_hand:.4f}")

     # 3. Arm the motor (Light turns Green)
     odrv0.axis0.requested_state = 8  # CLOSED_LOOP_CONTROL
     time.sleep(0.5)

     print("--- FOLLOWING ACTIVE ---")

     while True:
         # Read hand position (0.0 to 1.0)
         hand_turns = odrv0.spi_encoder0.raw

         # Command the motor
         # With circular_modulus=1.0, this handles the 0.9 -> 0.0 transition perfectly
         odrv0.axis0.controller.input_pos = hand_turns

         # Read actual motor position
         motor_turns = odrv0.axis0.pos_vel_mapper.pos_rel

         # Display live data
         print(f"Hand: {hand_turns:.4f} | Motor: {motor_turns:.4f} | Diff: {hand_turns-motor_turns:.4f}", end='\r')

         time.sleep(0.005)

 except KeyboardInterrupt:
     print("\n[STOP] Disarming...")
 finally:
     odrv0.axis0.requested_state = 1
     print("System Idle.")