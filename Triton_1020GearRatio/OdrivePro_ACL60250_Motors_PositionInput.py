import time
import odrive

# ------------------ CONFIGURATION ------------------
GEAR_RATIO = 1020.0           # Motor turns per 1 output turn
POSITION_TOL = 0.07           # Motor turns tolerance for "settled"
VELOCITY_TOL = 0.001          # Motor turns/sec tolerance for "settled"

MAX_DEGREE = 85               # Mechanical soft limit
MIN_DEGREE = -85

SPI_HOME_RAW = 0.34885120391845703  # SPI encoder raw value at vertical (installation)
GO_TO_HOME_ON_STARTUP = True        # True = move to home on power-on

odrv0 = odrive.find_any(serial_number="3665337E3432")
# ------------------ UTILITY FUNCTIONS ------------------
def raw_to_output_deg(raw, home_offset):
    """
    Convert SPI raw reading to degrees relative to home (vertical)
    """
    slope = -438.0  # calibrated scaling
    return slope * (raw - home_offset)



# ------------------ CONTROL FUNCTIONS FOR GUI ------------------

odrv0 = None
MOTOR_HOME = None
spi_home_offset = None
startup_motor_pos = None


def initialize(odrive_instance):
    global odrv0, MOTOR_HOME, spi_home_offset, startup_motor_pos

    odrv0 = odrive_instance
    odrv0.clear_errors()

    # Gains
    odrv0.axis0.controller.config.pos_gain = 50.0
    odrv0.axis0.controller.config.vel_gain = 0.15
    odrv0.axis0.controller.config.vel_integrator_gain = 0.1

    odrv0.axis0.requested_state = 8
    time.sleep(0.5)

    # Seed absolute motor position
    output_abs_turns = odrv0.spi_encoder0.raw
    odrv0.axis0.pos_vel_mapper.set_abs_pos(output_abs_turns * GEAR_RATIO)
    startup_motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel

    # Compute fixed MOTOR_HOME
    current_motor_pos = startup_motor_pos
    MOTOR_HOME = current_motor_pos + (
        (0.0 - raw_to_output_deg(odrv0.spi_encoder0.raw, SPI_HOME_RAW))
        / 360.0 * GEAR_RATIO
    )

    # Clamp
    max_motor_turns = startup_motor_pos + MAX_DEGREE / 360.0 * GEAR_RATIO
    min_motor_turns = startup_motor_pos + MIN_DEGREE / 360.0 * GEAR_RATIO

    if MOTOR_HOME > max_motor_turns:
        MOTOR_HOME = max_motor_turns
    elif MOTOR_HOME < min_motor_turns:
        MOTOR_HOME = min_motor_turns

    spi_home_offset = odrv0.spi_encoder0.raw

    if GO_TO_HOME_ON_STARTUP:
        go_home()


def wait_until_settled(target_motor_turns):
    while True:
        motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
        motor_vel = odrv0.axis0.pos_vel_mapper.vel
        pos_error = abs(target_motor_turns - motor_pos)
        vel_abs = abs(motor_vel)

        if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
            break
        time.sleep(0.001)


def go_home():
    odrv0.axis0.requested_state = 8
    odrv0.axis0.controller.input_pos = MOTOR_HOME
    wait_until_settled(MOTOR_HOME)


def move_absolute(target_output_deg):
    if target_output_deg > MAX_DEGREE:
        target_output_deg = MAX_DEGREE
    elif target_output_deg < MIN_DEGREE:
        target_output_deg = MIN_DEGREE

    target_output_turns = target_output_deg / 360.0
    target_motor_turns = MOTOR_HOME + target_output_turns * GEAR_RATIO

    odrv0.axis0.requested_state = 8
    odrv0.axis0.controller.input_pos = target_motor_turns

    wait_until_settled(target_motor_turns)


def move_relative(delta_deg):
    current_deg = get_current_position()
    move_absolute(current_deg + delta_deg)


def get_current_position():
    motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
    return (motor_pos - MOTOR_HOME) * 360.0 / GEAR_RATIO


def set_gains(pos_gain, vel_gain, vel_i):
    odrv0.axis0.controller.config.pos_gain = pos_gain
    odrv0.axis0.controller.config.vel_gain = vel_gain
    odrv0.axis0.controller.config.vel_integrator_gain = vel_i


def set_traj_params(vel, acc, dec):
    odrv0.axis0.trap_traj.config.vel_limit = vel
    odrv0.axis0.trap_traj.config.accel_limit = acc
    odrv0.axis0.trap_traj.config.decel_limit = dec

# ------------------ MAIN CONTROL ------------------
if __name__ == "__main__":
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
        current_motor_pos = startup_motor_pos

        # ---- Calculate fixed motor home position ----
        # This corresponds to 0° vertical using SPI_HOME_RAW
        MOTOR_HOME = current_motor_pos + (0.0 - raw_to_output_deg(odrv0.spi_encoder0.raw, SPI_HOME_RAW)) / 360.0 * GEAR_RATIO

        # Clamp MOTOR_HOME to stay within mechanical limits
        max_motor_turns = startup_motor_pos + MAX_DEGREE / 360.0 * GEAR_RATIO
        min_motor_turns = startup_motor_pos + MIN_DEGREE / 360.0 * GEAR_RATIO
        if MOTOR_HOME > max_motor_turns:
            print(f"[WARNING] Calculated home above MAX_DEGREE, clamping to {MAX_DEGREE}°")
            MOTOR_HOME = max_motor_turns
        elif MOTOR_HOME < min_motor_turns:
            print(f"[WARNING] Calculated home below MIN_DEGREE, clamping to {MIN_DEGREE}°")
            MOTOR_HOME = min_motor_turns

        # ---- Move to home position at startup (if enabled) ----
        if GO_TO_HOME_ON_STARTUP:
            odrv0.axis0.controller.input_pos = MOTOR_HOME
            while True:
                motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
                motor_vel = odrv0.axis0.pos_vel_mapper.vel
                pos_error = abs(MOTOR_HOME - motor_pos)
                if pos_error < POSITION_TOL:
                    time.sleep(0.001)
                    break
                time.sleep(0.001)
            print("Startup move complete: Axis at home (vertical)")
            #odrv0.axis0.requested_state = 1  # Disarm after reaching home
            #time.sleep(0.1)
            #odrv0.axis0.requested_state = 8  # Re-enable for user control

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

            # Convert output turns to motor turns relative to home
            target_motor_turns = MOTOR_HOME + target_output_turns * GEAR_RATIO

            # Command motor
            odrv0.axis0.requested_state = 8  # Ensure closed-loop
            odrv0.axis0.controller.input_pos = target_motor_turns

            # Wait until motion completes
            while True:
                motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
                motor_vel = odrv0.axis0.pos_vel_mapper.vel
                pos_error = abs(target_motor_turns - motor_pos)
                vel_abs = abs(motor_vel)
                if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
                    time.sleep(0.001)
                    break
                time.sleep(0.001)

            # Final readings
            motor_pos = odrv0.axis0.pos_vel_mapper.pos_rel
            motor_based_output = (motor_pos - MOTOR_HOME) * 360 / GEAR_RATIO
            output_encoder = raw_to_output_deg(odrv0.spi_encoder0.raw, spi_home_offset)

            print("\nMove Complete:")
            print(f"Motor-based:      {motor_based_output:.3f}°")
            print(f"Output Encoder:   {output_encoder:.3f}°")
            print(f"Raw:              {odrv0.spi_encoder0.raw}")

            #odrv0.axis0.requested_state = 1  # Disarm after each move

    except KeyboardInterrupt:
        print("\n[STOP] Disarming...")

    finally:
        odrv0.axis0.requested_state = 1
