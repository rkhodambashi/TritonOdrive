import threading
import time

import odrive

# ------------------ CONFIGURATION ------------------
GEAR_RATIO = 1240.0
POSITION_TOL = 0.005
VELOCITY_TOL = 0.001

MAX_DEGREE = 90
MIN_DEGREE = -90
TRACKING_MAX_DEGREE = 91
TRACKING_MIN_DEGREE = -91

X_SPI_HOME_RAW = 0.376205 #0.367891 #0.4863780736923218
Y_SPI_HOME_RAW = 0.173497 #0.4863780736923218
GO_TO_HOME_ON_STARTUP = True

DEFAULT_POS_GAIN = 50.0
DEFAULT_VEL_GAIN = 0.15
DEFAULT_VEL_INTEGRATOR_GAIN = 1

DEFAULT_TRAJ_VEL_LIMIT = 50.0
DEFAULT_TRAJ_ACCEL_LIMIT = 100.0
DEFAULT_TRAJ_DECEL_LIMIT = 100.0

DEFAULT_SPINOUT_MECHANICAL_POWER_THRESHOLD = -10.0
DEFAULT_SPINOUT_ELECTRICAL_POWER_THRESHOLD = 10.0

AXIS_CONFIG = {
    "x": {
        "serial_number": "3665337E3432",
        "spi_home_raw": X_SPI_HOME_RAW,
        "home_sign": -1.0,
        "output_sign": 1.0,
        "spi_sign": -1.0,
        "spinout_mechanical_power_threshold": DEFAULT_SPINOUT_MECHANICAL_POWER_THRESHOLD,
        "spinout_electrical_power_threshold": DEFAULT_SPINOUT_ELECTRICAL_POWER_THRESHOLD,
    },
    "y": {
        "serial_number": "367F337A3432",
        "spi_home_raw": Y_SPI_HOME_RAW,
        "home_sign": -1.0,
        "output_sign": 1.0,
        "spi_sign": -1.0,
        "spinout_mechanical_power_threshold": DEFAULT_SPINOUT_MECHANICAL_POWER_THRESHOLD,
        "spinout_electrical_power_threshold": DEFAULT_SPINOUT_ELECTRICAL_POWER_THRESHOLD,
    },
}

AXIS_STATE = {
    axis: {
        "odrive": None,
        "motor_home": None,
        "spi_home_offset": None,
        "startup_motor_pos": None,
        "tracking_prev_input_mode": None,
        "tracking_prev_control_mode": None,
        "tracking_prev_input_filter_bandwidth": None,
        "tracking_mode_active": False,
    }
    for axis in AXIS_CONFIG
}

# Backwards-compatible X-axis globals used elsewhere in the project.
odrv0 = None
odrv1 = None
MOTOR_HOME = None
MOTOR_HOME_Y = None
spi_home_offset = None
spi_home_offset_y = None
startup_motor_pos = None
startup_motor_pos_y = None


def raw_to_output_deg(raw, home_offset):
    return wrapped_raw_delta(raw, home_offset) * 360


def wrapped_raw_delta(raw, reference_raw):
    delta = raw - reference_raw
    while delta >= 0.5:
        delta -= 1.0
    while delta < -0.5:
        delta += 1.0
    return delta


def _sync_legacy_globals():
    global odrv0, odrv1, MOTOR_HOME, MOTOR_HOME_Y
    global spi_home_offset, spi_home_offset_y, startup_motor_pos, startup_motor_pos_y

    x_state = AXIS_STATE["x"]
    y_state = AXIS_STATE["y"]

    odrv0 = x_state["odrive"]
    odrv1 = y_state["odrive"]
    MOTOR_HOME = x_state["motor_home"]
    MOTOR_HOME_Y = y_state["motor_home"]
    spi_home_offset = x_state["spi_home_offset"]
    spi_home_offset_y = y_state["spi_home_offset"]
    startup_motor_pos = x_state["startup_motor_pos"]
    startup_motor_pos_y = y_state["startup_motor_pos"]


def _validate_axis(axis):
    if axis not in AXIS_CONFIG:
        raise ValueError(f"Unknown axis '{axis}'. Expected one of: {', '.join(AXIS_CONFIG)}")


def _get_state(axis="x"):
    _validate_axis(axis)
    state = AXIS_STATE[axis]
    if state["odrive"] is None:
        raise RuntimeError(f"{axis.upper()} axis ODrive is not initialized")
    return state


def get_serial_number(axis="x"):
    _validate_axis(axis)
    return AXIS_CONFIG[axis]["serial_number"]


def get_odrive(axis="x"):
    return _get_state(axis)["odrive"]


def get_spi_home_offset(axis="x"):
    return _get_state(axis)["spi_home_offset"]


def get_spi_raw(axis="x"):
    return _get_state(axis)["odrive"].spi_encoder0.raw


def get_motor_raw(axis="x"):
    return _get_state(axis)["odrive"].axis0.pos_vel_mapper.pos_rel


def get_motor_velocity(axis="x"):
    return _get_state(axis)["odrive"].axis0.pos_vel_mapper.vel


def get_input_pos(axis="x"):
    return _get_state(axis)["odrive"].axis0.controller.input_pos


def get_axis_state(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.current_state)


def get_disarm_reason(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.disarm_reason)


def get_active_errors(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.active_errors)


def get_procedure_result(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.procedure_result)


def initialize(odrive_instance, axis="x"):
    _validate_axis(axis)

    state = AXIS_STATE[axis]
    spi_home_raw = AXIS_CONFIG[axis]["spi_home_raw"]
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    home_sign = AXIS_CONFIG[axis]["home_sign"]
    spinout_mech_threshold = AXIS_CONFIG[axis]["spinout_mechanical_power_threshold"]
    spinout_elec_threshold = AXIS_CONFIG[axis]["spinout_electrical_power_threshold"]

    state["odrive"] = odrive_instance

    odrive_instance.clear_errors()
    odrive_instance.axis0.controller.config.pos_gain = DEFAULT_POS_GAIN
    odrive_instance.axis0.controller.config.vel_gain = DEFAULT_VEL_GAIN
    odrive_instance.axis0.controller.config.vel_integrator_gain = DEFAULT_VEL_INTEGRATOR_GAIN
    odrive_instance.axis0.controller.config.spinout_mechanical_power_threshold = spinout_mech_threshold
    odrive_instance.axis0.controller.config.spinout_electrical_power_threshold = spinout_elec_threshold
    odrive_instance.axis0.trap_traj.config.vel_limit = DEFAULT_TRAJ_VEL_LIMIT
    odrive_instance.axis0.trap_traj.config.accel_limit = DEFAULT_TRAJ_ACCEL_LIMIT
    odrive_instance.axis0.trap_traj.config.decel_limit = DEFAULT_TRAJ_DECEL_LIMIT

    odrive_instance.axis0.requested_state = 8
    time.sleep(0.5)

    output_abs_turns = odrive_instance.spi_encoder0.raw
    odrive_instance.axis0.pos_vel_mapper.set_abs_pos(output_abs_turns * GEAR_RATIO)
    state["startup_motor_pos"] = odrive_instance.axis0.pos_vel_mapper.pos_rel

    current_motor_pos = state["startup_motor_pos"]
    motor_home = current_motor_pos - home_sign * wrapped_raw_delta(odrive_instance.spi_encoder0.raw, spi_home_raw) * GEAR_RATIO

    max_motor_turns = state["startup_motor_pos"] + MAX_DEGREE / 360.0 * GEAR_RATIO
    min_motor_turns = state["startup_motor_pos"] + MIN_DEGREE / 360.0 * GEAR_RATIO

    if motor_home > max_motor_turns:
        motor_home = max_motor_turns
    elif motor_home < min_motor_turns:
        motor_home = min_motor_turns

    state["motor_home"] = motor_home
    state["spi_home_offset"] = odrive_instance.spi_encoder0.raw

    _sync_legacy_globals()

    if GO_TO_HOME_ON_STARTUP:
        go_home(axis=axis)


def connect(axis="x"):
    serial_number = get_serial_number(axis)
    odrive_instance = odrive.find_any(serial_number=serial_number)
    initialize(odrive_instance, axis=axis)
    return odrive_instance


def connect_all():
    devices = {}
    for axis in AXIS_CONFIG:
        devices[axis] = connect(axis)
    return devices


def wait_until_settled(target_motor_turns, axis="x"):
    state = _get_state(axis)
    while True:
        motor_pos = state["odrive"].axis0.pos_vel_mapper.pos_rel
        motor_vel = state["odrive"].axis0.pos_vel_mapper.vel
        pos_error = abs(target_motor_turns - motor_pos)
        vel_abs = abs(motor_vel)

        if pos_error < POSITION_TOL and vel_abs < VELOCITY_TOL:
            break
        time.sleep(0.001)


def go_home(axis="x"):
    state = _get_state(axis)
    state["odrive"].axis0.requested_state = 8
    state["odrive"].axis0.controller.input_pos = state["motor_home"]
    wait_until_settled(state["motor_home"], axis=axis)


def move_absolute(target_output_deg, axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]

    if target_output_deg > MAX_DEGREE:
        target_output_deg = MAX_DEGREE
    elif target_output_deg < MIN_DEGREE:
        target_output_deg = MIN_DEGREE

    target_output_turns = target_output_deg / 360.0
    target_motor_turns = state["motor_home"] + output_sign * target_output_turns * GEAR_RATIO

    state["odrive"].axis0.requested_state = 8
    state["odrive"].axis0.controller.input_pos = target_motor_turns
    wait_until_settled(target_motor_turns, axis=axis)


def command_absolute(target_output_deg, axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]

    if target_output_deg > MAX_DEGREE:
        target_output_deg = MAX_DEGREE
    elif target_output_deg < MIN_DEGREE:
        target_output_deg = MIN_DEGREE

    target_output_turns = target_output_deg / 360.0
    target_motor_turns = state["motor_home"] + output_sign * target_output_turns * GEAR_RATIO

    state["odrive"].axis0.requested_state = 8
    state["odrive"].axis0.controller.input_pos = target_motor_turns


def command_absolute_with_velocity(target_output_deg, target_output_vel_deg_s=0.0, axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]

    if target_output_deg > TRACKING_MAX_DEGREE:
        target_output_deg = TRACKING_MAX_DEGREE
    elif target_output_deg < TRACKING_MIN_DEGREE:
        target_output_deg = TRACKING_MIN_DEGREE

    target_output_turns = target_output_deg / 360.0
    target_motor_turns = state["motor_home"] + output_sign * target_output_turns * GEAR_RATIO
    target_motor_vel_turns_s = output_sign * target_output_vel_deg_s * GEAR_RATIO / 360.0

    state["odrive"].axis0.requested_state = 8
    state["odrive"].axis0.controller.input_pos = target_motor_turns
    state["odrive"].axis0.controller.input_vel = target_motor_vel_turns_s


def move_relative(delta_deg, axis="x"):
    current_deg = get_current_position(axis=axis)
    move_absolute(current_deg + delta_deg, axis=axis)


def get_current_position(axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    motor_pos = state["odrive"].axis0.pos_vel_mapper.pos_rel
    return (motor_pos - state["motor_home"]) * 360.0 / (GEAR_RATIO * output_sign)


def get_spi_position(axis="x"):
    _validate_axis(axis)
    state = _get_state(axis)
    spi_sign = AXIS_CONFIG[axis]["spi_sign"]
    spi_home_raw = AXIS_CONFIG[axis]["spi_home_raw"]
    return raw_to_output_deg(state["odrive"].spi_encoder0.raw, spi_home_raw) * spi_sign


def set_gains(pos_gain, vel_gain, vel_i, axis="x"):
    state = _get_state(axis)
    state["odrive"].axis0.controller.config.pos_gain = pos_gain
    state["odrive"].axis0.controller.config.vel_gain = vel_gain
    state["odrive"].axis0.controller.config.vel_integrator_gain = vel_i


def set_traj_params(vel, acc, dec, axis="x"):
    state = _get_state(axis)
    state["odrive"].axis0.trap_traj.config.vel_limit = vel
    state["odrive"].axis0.trap_traj.config.accel_limit = acc
    state["odrive"].axis0.trap_traj.config.decel_limit = dec


def enter_tracking_mode(axis="x", input_filter_bandwidth=None):
    state = _get_state(axis)
    if state["tracking_mode_active"]:
        return

    axis0 = state["odrive"].axis0
    current_motor_pos = axis0.pos_vel_mapper.pos_rel
    state["tracking_prev_control_mode"] = axis0.controller.config.control_mode
    state["tracking_prev_input_mode"] = axis0.controller.config.input_mode
    state["tracking_prev_input_filter_bandwidth"] = axis0.controller.config.input_filter_bandwidth
    axis0.controller.config.control_mode = 3
    axis0.controller.config.input_mode = 1
    if input_filter_bandwidth is not None:
        axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
    axis0.controller.input_pos = current_motor_pos
    axis0.requested_state = 8
    state["tracking_mode_active"] = True


def exit_tracking_mode(axis="x"):
    state = _get_state(axis)
    if not state["tracking_mode_active"]:
        return

    axis0 = state["odrive"].axis0
    current_motor_pos = axis0.pos_vel_mapper.pos_rel
    if state["tracking_prev_control_mode"] is not None:
        axis0.controller.config.control_mode = state["tracking_prev_control_mode"]
    if state["tracking_prev_input_mode"] is not None:
        axis0.controller.config.input_mode = state["tracking_prev_input_mode"]
    if state["tracking_prev_input_filter_bandwidth"] is not None:
        axis0.controller.config.input_filter_bandwidth = state["tracking_prev_input_filter_bandwidth"]
    axis0.controller.input_pos = current_motor_pos
    axis0.requested_state = 8
    state["tracking_prev_control_mode"] = None
    state["tracking_prev_input_mode"] = None
    state["tracking_prev_input_filter_bandwidth"] = None
    state["tracking_mode_active"] = False


def _run_parallel(calls):
    threads = []
    for func, kwargs in calls:
        thread = threading.Thread(target=func, kwargs=kwargs)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()


def go_home_all():
    _run_parallel([(go_home, {"axis": axis}) for axis in AXIS_CONFIG])


def move_absolute_pair(x_deg=None, y_deg=None):
    calls = []
    if x_deg is not None:
        calls.append((move_absolute, {"target_output_deg": x_deg, "axis": "x"}))
    if y_deg is not None:
        calls.append((move_absolute, {"target_output_deg": y_deg, "axis": "y"}))
    _run_parallel(calls)


def command_absolute_pair(x_deg=None, y_deg=None):
    if x_deg is not None:
        command_absolute(x_deg, axis="x")
    if y_deg is not None:
        command_absolute(y_deg, axis="y")


def command_absolute_pair_with_velocity(x_deg=None, y_deg=None, x_vel_deg_s=0.0, y_vel_deg_s=0.0):
    if x_deg is not None:
        command_absolute_with_velocity(x_deg, target_output_vel_deg_s=x_vel_deg_s, axis="x")
    if y_deg is not None:
        command_absolute_with_velocity(y_deg, target_output_vel_deg_s=y_vel_deg_s, axis="y")


def move_relative_pair(x_delta=None, y_delta=None):
    calls = []
    if x_delta is not None:
        calls.append((move_relative, {"delta_deg": x_delta, "axis": "x"}))
    if y_delta is not None:
        calls.append((move_relative, {"delta_deg": y_delta, "axis": "y"}))
    _run_parallel(calls)


def set_gains_all(pos_gain, vel_gain, vel_i):
    for axis in AXIS_CONFIG:
        set_gains(pos_gain, vel_gain, vel_i, axis=axis)


def set_traj_params_all(vel, acc, dec):
    for axis in AXIS_CONFIG:
        set_traj_params(vel, acc, dec, axis=axis)


def enter_tracking_mode_all(input_filter_bandwidth=None):
    for axis in AXIS_CONFIG:
        enter_tracking_mode(axis=axis, input_filter_bandwidth=input_filter_bandwidth)


def exit_tracking_mode_all():
    for axis in AXIS_CONFIG:
        exit_tracking_mode(axis=axis)


def disarm_all():
    for state in AXIS_STATE.values():
        if state["odrive"] is not None:
            state["odrive"].axis0.requested_state = 1


if __name__ == "__main__":
    try:
        connect_all()
        print("Connected axes:")
        for axis in AXIS_CONFIG:
            print(f"  {axis.upper()}: {get_serial_number(axis)}")
            print(f"     Position: {get_current_position(axis):.3f} deg")
            print(f"     SPI:      {get_spi_position(axis):.3f} deg")

        while True:
            user_input = input("\nTarget output degrees for X axis (q to quit): ")
            if user_input.lower() == "q":
                break

            try:
                target_output_deg = float(user_input)
            except ValueError:
                print("Invalid input. Enter a number in degrees.")
                continue

            move_absolute(target_output_deg, axis="x")
            print(f"X motor-based output: {get_current_position('x'):.3f} deg")
            print(f"X SPI output:         {get_spi_position('x'):.3f} deg")
            print(f"X raw:                {get_spi_raw('x')}")

    except KeyboardInterrupt:
        print("\n[STOP] Disarming...")
    finally:
        disarm_all()
