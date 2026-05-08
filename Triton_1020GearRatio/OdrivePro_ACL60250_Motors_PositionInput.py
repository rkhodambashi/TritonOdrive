import threading
import time

import odrive
from odrive import enums as odrive_enums

try:
    from endat_serial_reader import (
        DEFAULT_BAUD as ENDAT_DEFAULT_BAUD,
        DEFAULT_COUNTS_PER_REV as ENDAT_DEFAULT_COUNTS_PER_REV,
        EndatSerialReader,
    )
except Exception:
    ENDAT_DEFAULT_BAUD = 115200
    ENDAT_DEFAULT_COUNTS_PER_REV = 1 << 25
    EndatSerialReader = None

# ------------------ CONFIGURATION ------------------
GEAR_RATIO = 1240.0  # Used as the pos/vel mapper scale when SPI is the load encoder.
POSITION_TOL = 0.005
VELOCITY_TOL = 0.001
SPI_POSITION_TOL_DEG = 0.05
SETTLE_TIMEOUT_SEC = 20.0

MAX_DEGREE = 90
MIN_DEGREE = -90
MANUAL_EDGE_GUARD_DEG = 0.25
RECOVERY_TARGET_MARGIN_DEG = 5.0
TRACKING_MAX_DEGREE = 91
TRACKING_MIN_DEGREE = -91
POSITION_SAFETY_MAX_DEGREE = 92
POSITION_SAFETY_MIN_DEGREE = -92
POSITION_SAFETY_POLL_INTERVAL_SEC = 0.02
EXTERNAL_POSITION_MAX_AGE_SEC = 0.25

X_SPI_HOME_RAW = 0.345814 #0.376205 #0.367891 #0.4863780736923218
Y_SPI_HOME_RAW = 0.037560 #0.176057 #0.173497 #0.4863780736923218
X_ENDAT_HOME_RAW = 0.594747
Y_ENDAT_HOME_RAW = 0.0
GO_TO_HOME_ON_STARTUP = False

DEFAULT_POS_GAIN = 40.0
DEFAULT_VEL_GAIN = 0.15
DEFAULT_VEL_INTEGRATOR_GAIN = 1

DEFAULT_TRAJ_VEL_LIMIT = 50.0
DEFAULT_TRAJ_ACCEL_LIMIT = 100.0
DEFAULT_TRAJ_DECEL_LIMIT = 100.0

DEFAULT_SPINOUT_MECHANICAL_POWER_THRESHOLD = -10.0
DEFAULT_SPINOUT_ELECTRICAL_POWER_THRESHOLD = 10.0
SPI_LOAD_ENCODER_ID = odrive_enums.ENCODER_ID_SPI_ENCODER0
MANUAL_LOAD_ENCODER_ID = odrive_enums.ENCODER_ID_INC_ENCODER0
MANUAL_POS_VEL_MAPPER_SCALE = 1.0
FORCE_SPI_LOAD_ENCODER_ON_CONNECT = False

AXIS_CONFIG = {
    "x": {
        "serial_number": "3665337E3432",
        "spi_home_raw": X_SPI_HOME_RAW,
        "endat_home_raw": X_ENDAT_HOME_RAW,
        "home_sign": -1.0,
        "output_sign": 1.0,
        "spi_sign": -1.0,
        "spinout_mechanical_power_threshold": DEFAULT_SPINOUT_MECHANICAL_POWER_THRESHOLD,
        "spinout_electrical_power_threshold": DEFAULT_SPINOUT_ELECTRICAL_POWER_THRESHOLD,
    },
    "y": {
        "serial_number": "367F337A3432",
        "spi_home_raw": Y_SPI_HOME_RAW,
        "endat_home_raw": Y_ENDAT_HOME_RAW,
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
        "velocity_prev_input_mode": None,
        "velocity_prev_control_mode": None,
        "velocity_prev_input_filter_bandwidth": None,
        "velocity_prev_vel_ramp_rate": None,
        "velocity_mode_active": False,
        "safety_tripped": False,
        "safety_trip_reason": None,
        "recovery_active": False,
        "feedback_source": "spi",
        "external_position_deg": None,
        "external_position_source": None,
        "external_position_timestamp": None,
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
_safety_monitor_thread = None
_safety_monitor_stop_event = threading.Event()
ENDAT_STATE = {
    axis: {
        "reader": None,
        "port": None,
        "baud": ENDAT_DEFAULT_BAUD,
        "counts_per_rev": ENDAT_DEFAULT_COUNTS_PER_REV,
        "home_raw": AXIS_CONFIG[axis]["endat_home_raw"],
        "sign": 1.0,
        "latest_sample": None,
        "lock": threading.RLock(),
    }
    for axis in AXIS_CONFIG
}


def raw_to_output_deg(raw, home_offset):
    return wrapped_raw_delta(raw, home_offset) * 360


def wrapped_raw_delta(raw, reference_raw):
    delta = raw - reference_raw
    while delta >= 0.5:
        delta -= 1.0
    while delta < -0.5:
        delta += 1.0
    return delta


def wrap_degrees_signed(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def endat_home_raw_to_zero_count(home_raw, counts_per_rev):
    value = float(home_raw)
    if abs(value) > 1.0:
        return int(value) % int(counts_per_rev)
    return int(round((value % 1.0) * int(counts_per_rev)))


def endat_zero_count_to_home_raw(zero_count, counts_per_rev):
    return (int(zero_count) % int(counts_per_rev)) / float(counts_per_rev)


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


def _ensure_safe_to_command(axis="x"):
    state = _get_state(axis)
    if state["safety_tripped"]:
        try:
            position_deg = get_current_position(axis)
            if POSITION_SAFETY_MIN_DEGREE <= position_deg <= POSITION_SAFETY_MAX_DEGREE:
                clear_safety_trip(axis)
                return state
        except Exception:
            pass
        reason = state["safety_trip_reason"] or f"{axis.upper()} axis safety trip is active"
        raise RuntimeError(reason)
    return state


def clear_safety_trip(axis="x"):
    state = _get_state(axis)
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None


def clear_safety_trip_all():
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["odrive"] is not None:
            clear_safety_trip(axis=axis)


def get_safety_trip_reason(axis="x"):
    state = _get_state(axis)
    return state["safety_trip_reason"]


def is_safety_tripped(axis="x"):
    state = _get_state(axis)
    return bool(state["safety_tripped"])


def get_serial_number(axis="x"):
    _validate_axis(axis)
    return AXIS_CONFIG[axis]["serial_number"]


def get_odrive(axis="x"):
    return _get_state(axis)["odrive"]


def get_spi_home_offset(axis="x"):
    return _get_state(axis)["spi_home_offset"]


def get_spi_raw(axis="x"):
    return _get_state(axis)["odrive"].spi_encoder0.raw


def get_position_feedback_source(axis="x"):
    return _get_state(axis)["feedback_source"]


def set_position_feedback_source(axis="x", source="spi"):
    _validate_axis(axis)
    if source not in ("spi", "external"):
        raise ValueError("Position feedback source must be 'spi' or 'external'")
    AXIS_STATE[axis]["feedback_source"] = source


def update_external_position(axis="x", position_deg=None, source="external", timestamp=None):
    _validate_axis(axis)
    if position_deg is None:
        raise ValueError("External position cannot be None")
    state = AXIS_STATE[axis]
    state["external_position_deg"] = float(position_deg)
    state["external_position_source"] = source
    state["external_position_timestamp"] = time.monotonic() if timestamp is None else float(timestamp)


def clear_external_position(axis="x"):
    _validate_axis(axis)
    state = AXIS_STATE[axis]
    state["external_position_deg"] = None
    state["external_position_source"] = None
    state["external_position_timestamp"] = None


def configure_endat(axis="x", counts_per_rev=None, home_raw=None, sign=None):
    _validate_axis(axis)
    state = ENDAT_STATE[axis]
    with state["lock"]:
        if counts_per_rev is not None:
            counts_per_rev = int(counts_per_rev)
            if counts_per_rev <= 0:
                raise ValueError("EnDat counts_per_rev must be positive")
            state["counts_per_rev"] = counts_per_rev
        if home_raw is not None:
            state["home_raw"] = float(home_raw)
        if sign is not None:
            sign = float(sign)
            if sign == 0.0:
                raise ValueError("EnDat sign cannot be zero")
            state["sign"] = sign

        reader = state["reader"]
        if reader is not None:
            reader.counts_per_rev = state["counts_per_rev"]
            reader.zero_count = endat_home_raw_to_zero_count(
                state["home_raw"],
                state["counts_per_rev"],
            )


def connect_endat(axis="x", port="COM6", baud=ENDAT_DEFAULT_BAUD, counts_per_rev=ENDAT_DEFAULT_COUNTS_PER_REV, home_raw=None, sign=1.0):
    _validate_axis(axis)
    if EndatSerialReader is None:
        raise RuntimeError("EnDat reader is unavailable. Install pyserial and ensure endat_serial_reader.py is present.")
    if not port:
        raise ValueError("EnDat port cannot be blank")

    state = ENDAT_STATE[axis]
    with state["lock"]:
        configure_endat(axis, counts_per_rev=counts_per_rev, home_raw=home_raw, sign=sign)
        baud = int(baud)
        reader = state["reader"]
        if reader is not None and state["port"] == port and state["baud"] == baud:
            set_position_feedback_source(axis, "external")
            return reader

        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass

        reader = EndatSerialReader(
            port,
            baud=baud,
            counts_per_rev=state["counts_per_rev"],
            zero_count=endat_home_raw_to_zero_count(state["home_raw"], state["counts_per_rev"]),
            timeout_s=0.0,
        )
        reader.open()
        state["reader"] = reader
        state["port"] = port
        state["baud"] = baud
        state["latest_sample"] = None
        set_position_feedback_source(axis, "external")
        return reader


def disconnect_endat(axis="x"):
    _validate_axis(axis)
    state = ENDAT_STATE[axis]
    with state["lock"]:
        reader = state["reader"]
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass
        state["reader"] = None
        state["latest_sample"] = None
    clear_external_position(axis)
    set_position_feedback_source(axis, "spi")


def is_endat_connected(axis="x"):
    _validate_axis(axis)
    return ENDAT_STATE[axis]["reader"] is not None


def get_endat_axis_deg(axis, sample):
    state = ENDAT_STATE[axis]
    return state["sign"] * wrap_degrees_signed(sample.angle_deg)


def read_endat_sample(axis="x"):
    _validate_axis(axis)
    state = ENDAT_STATE[axis]
    with state["lock"]:
        reader = state["reader"]
        if reader is None:
            return None
        sample = reader.read_latest_available()
        if sample is not None:
            state["latest_sample"] = sample
            update_external_position(
                axis,
                get_endat_axis_deg(axis, sample),
                source="endat",
                timestamp=sample.timestamp_s,
            )
            set_position_feedback_source(axis, "external")
        return state["latest_sample"]


def align_motor_home_to_current_position(axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    current_motor_pos = state["odrive"].axis0.pos_vel_mapper.pos_rel
    current_output_deg = get_current_position(axis)
    state["motor_home"] = current_motor_pos - output_sign * current_output_deg * GEAR_RATIO / 360.0
    _sync_legacy_globals()
    return state["motor_home"]


def get_external_position(axis="x"):
    state = _get_state(axis)
    position_deg = state["external_position_deg"]
    timestamp = state["external_position_timestamp"]
    if position_deg is None or timestamp is None:
        raise RuntimeError(f"{axis.upper()} external position feedback is not available")
    age_sec = time.monotonic() - timestamp
    if age_sec > EXTERNAL_POSITION_MAX_AGE_SEC:
        raise RuntimeError(
            f"{axis.upper()} external position feedback is stale "
            f"({age_sec:.3f}s > {EXTERNAL_POSITION_MAX_AGE_SEC:.3f}s)"
        )
    return position_deg


def get_motor_raw(axis="x"):
    return _get_state(axis)["odrive"].axis0.pos_vel_mapper.pos_rel


def get_motor_velocity(axis="x"):
    return _get_state(axis)["odrive"].axis0.pos_vel_mapper.vel


def get_input_pos(axis="x"):
    return _get_state(axis)["odrive"].axis0.controller.input_pos


def get_control_mode(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.controller.config.control_mode)


def get_input_mode(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.controller.config.input_mode)


def get_load_encoder(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.config.load_encoder)


def get_commutation_encoder(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.config.commutation_encoder)


def get_pos_vel_mapper_scale(axis="x"):
    return float(_get_state(axis)["odrive"].axis0.pos_vel_mapper.config.scale)


def set_force_spi_load_encoder_on_connect(enabled):
    global FORCE_SPI_LOAD_ENCODER_ON_CONNECT
    FORCE_SPI_LOAD_ENCODER_ON_CONNECT = bool(enabled)


def get_axis_state(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.current_state)


def get_disarm_reason(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.disarm_reason)


def get_active_errors(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.active_errors)


def get_procedure_result(axis="x"):
    return int(_get_state(axis)["odrive"].axis0.procedure_result)


def _trip_safety(axis, position_deg):
    state = _get_state(axis)
    if state["safety_tripped"]:
        return

    axis0 = state["odrive"].axis0
    current_motor_pos = axis0.pos_vel_mapper.pos_rel
    reason = (
        f"{axis.upper()} axis position safety trip: {position_deg:.3f} deg outside "
        f"[{POSITION_SAFETY_MIN_DEGREE:.1f}, {POSITION_SAFETY_MAX_DEGREE:.1f}]"
    )

    try:
        axis0.controller.config.control_mode = 3
        axis0.controller.config.input_mode = 1
    except Exception:
        pass
    try:
        axis0.controller.input_vel = 0.0
    except Exception:
        pass
    try:
        axis0.controller.input_pos = current_motor_pos
    except Exception:
        pass
    try:
        axis0.requested_state = 8
    except Exception:
        pass

    state["tracking_mode_active"] = False
    state["velocity_mode_active"] = False
    state["safety_tripped"] = True
    state["safety_trip_reason"] = reason
    print(reason)


def _position_safety_monitor_loop():
    while not _safety_monitor_stop_event.is_set():
        for axis in AXIS_CONFIG:
            state = AXIS_STATE[axis]
            if state["odrive"] is None:
                continue
            if state["recovery_active"]:
                continue
            try:
                position_deg = get_current_position(axis)
            except Exception:
                continue
            if position_deg < POSITION_SAFETY_MIN_DEGREE or position_deg > POSITION_SAFETY_MAX_DEGREE:
                _trip_safety(axis, position_deg)
        _safety_monitor_stop_event.wait(POSITION_SAFETY_POLL_INTERVAL_SEC)


def _ensure_position_safety_monitor_running():
    global _safety_monitor_thread
    if _safety_monitor_thread is not None and _safety_monitor_thread.is_alive():
        return
    _safety_monitor_stop_event.clear()
    _safety_monitor_thread = threading.Thread(target=_position_safety_monitor_loop, daemon=True)
    _safety_monitor_thread.start()


def _configure_axis_runtime_encoder_setup(odrive_instance):
    if not FORCE_SPI_LOAD_ENCODER_ON_CONNECT:
        return

    axis0 = odrive_instance.axis0

    # Keep commutation and mapper settings as-is; only switch the load encoder.
    if int(axis0.config.load_encoder) != SPI_LOAD_ENCODER_ID:
        axis0.config.load_encoder = SPI_LOAD_ENCODER_ID


def _configure_axis_for_manual_position_control(axis):
    state = _get_state(axis)
    axis0 = state["odrive"].axis0

    # Manual/preposition moves are closed by ODrive using the motor incremental encoder.
    if int(axis0.config.load_encoder) != MANUAL_LOAD_ENCODER_ID:
        axis0.requested_state = odrive_enums.AXIS_STATE_IDLE
        time.sleep(0.05)
        axis0.config.load_encoder = MANUAL_LOAD_ENCODER_ID

    axis0.pos_vel_mapper.config.scale = MANUAL_POS_VEL_MAPPER_SCALE
    axis0.pos_vel_mapper.config.use_index_gpio = False
    axis0.pos_vel_mapper.config.index_offset_valid = False
    axis0.pos_vel_mapper.config.offset_valid = False
    axis0.pos_vel_mapper.config.approx_init_pos_valid = False

    load_encoder = int(axis0.config.load_encoder)
    if load_encoder != MANUAL_LOAD_ENCODER_ID:
        raise RuntimeError(
            f"{axis.upper()} ODrive load encoder is {load_encoder}, expected motor incremental "
            f"encoder {MANUAL_LOAD_ENCODER_ID} before manual/velocity command"
        )


def initialize(odrive_instance, axis="x"):
    _validate_axis(axis)

    state = AXIS_STATE[axis]
    spi_home_raw = AXIS_CONFIG[axis]["spi_home_raw"]
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    spinout_mech_threshold = AXIS_CONFIG[axis]["spinout_mechanical_power_threshold"]
    spinout_elec_threshold = AXIS_CONFIG[axis]["spinout_electrical_power_threshold"]

    state["odrive"] = odrive_instance

    odrive_instance.clear_errors()
    _configure_axis_runtime_encoder_setup(odrive_instance)
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

    if state["feedback_source"] == "spi":
        output_abs_turns = odrive_instance.spi_encoder0.raw
        odrive_instance.axis0.pos_vel_mapper.set_abs_pos(output_abs_turns * GEAR_RATIO)
    state["startup_motor_pos"] = odrive_instance.axis0.pos_vel_mapper.pos_rel

    current_motor_pos = state["startup_motor_pos"]
    try:
        if state["feedback_source"] == "external":
            current_feedback_deg = get_external_position(axis)
        else:
            current_feedback_deg = get_spi_position(axis)
    except Exception:
        current_feedback_deg = raw_to_output_deg(odrive_instance.spi_encoder0.raw, spi_home_raw) * AXIS_CONFIG[axis]["spi_sign"]
    motor_home = current_motor_pos - output_sign * current_feedback_deg * GEAR_RATIO / 360.0

    max_motor_turns = state["startup_motor_pos"] + MAX_DEGREE / 360.0 * GEAR_RATIO
    min_motor_turns = state["startup_motor_pos"] + MIN_DEGREE / 360.0 * GEAR_RATIO

    if motor_home > max_motor_turns:
        motor_home = max_motor_turns
    elif motor_home < min_motor_turns:
        motor_home = min_motor_turns

    state["motor_home"] = motor_home
    state["spi_home_offset"] = odrive_instance.spi_encoder0.raw
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None
    if state["feedback_source"] not in ("spi", "external"):
        state["feedback_source"] = "spi"

    _sync_legacy_globals()
    _ensure_position_safety_monitor_running()

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


def wait_until_settled(target_motor_turns, axis="x", target_output_deg=None):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    start_time = time.monotonic()
    while True:
        if time.monotonic() - start_time > SETTLE_TIMEOUT_SEC:
            raise TimeoutError(f"{axis.upper()} axis failed to settle within {SETTLE_TIMEOUT_SEC:.1f} s")

        active_errors = state["odrive"].axis0.active_errors
        disarm_reason = state["odrive"].axis0.disarm_reason
        if active_errors or disarm_reason:
            raise RuntimeError(
                f"{axis.upper()} axis stopped during settle "
                f"(active_errors={int(active_errors)}, disarm_reason={int(disarm_reason)})"
            )

        motor_pos = state["odrive"].axis0.pos_vel_mapper.pos_rel
        motor_vel = state["odrive"].axis0.pos_vel_mapper.vel
        pos_error = abs(target_motor_turns - motor_pos)
        vel_abs = abs(motor_vel)
        spi_pos_error = 0.0

        if target_output_deg is not None:
            spi_pos = get_current_position(axis)
            spi_pos_error = abs(target_output_deg - spi_pos)

        if (
            pos_error < POSITION_TOL
            and vel_abs < VELOCITY_TOL
            and (target_output_deg is None or spi_pos_error < SPI_POSITION_TOL_DEG)
        ):
            break
        time.sleep(0.001)


def _enter_position_trap_mode(axis):
    state = _ensure_safe_to_command(axis)
    axis0 = state["odrive"].axis0

    _configure_axis_for_manual_position_control(axis)
    current_motor_pos = axis0.pos_vel_mapper.pos_rel

    axis0.controller.config.control_mode = odrive_enums.CONTROL_MODE_POSITION_CONTROL
    axis0.controller.input_vel = 0.0
    axis0.controller.input_pos = current_motor_pos
    axis0.controller.config.input_mode = odrive_enums.INPUT_MODE_TRAP_TRAJ
    axis0.requested_state = odrive_enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    state["tracking_mode_active"] = False
    state["velocity_mode_active"] = False
    state["tracking_prev_control_mode"] = None
    state["tracking_prev_input_mode"] = None
    state["tracking_prev_input_filter_bandwidth"] = None
    state["velocity_prev_control_mode"] = None
    state["velocity_prev_input_mode"] = None
    state["velocity_prev_input_filter_bandwidth"] = None
    state["velocity_prev_vel_ramp_rate"] = None
    return state


def _align_selected_feedback_before_position_move(axis):
    state = _get_state(axis)
    if state["feedback_source"] == "external":
        align_motor_home_to_current_position(axis)
    return state


def _clamp_manual_target(target_output_deg):
    return max(min(float(target_output_deg), MAX_DEGREE), MIN_DEGREE)


def _command_position_target_from_selected_feedback(axis, target_output_deg):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    _configure_axis_for_manual_position_control(axis)
    align_motor_home_to_current_position(axis)

    axis0 = state["odrive"].axis0
    current_motor_pos = axis0.pos_vel_mapper.pos_rel
    axis0.controller.config.control_mode = odrive_enums.CONTROL_MODE_POSITION_CONTROL
    axis0.controller.input_vel = 0.0
    axis0.controller.input_pos = current_motor_pos
    axis0.controller.config.input_mode = odrive_enums.INPUT_MODE_TRAP_TRAJ
    axis0.requested_state = odrive_enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    target_output_deg = _clamp_manual_target(target_output_deg)
    target_motor_turns = state["motor_home"] + output_sign * target_output_deg * GEAR_RATIO / 360.0
    axis0.controller.input_pos = target_motor_turns
    return target_motor_turns, target_output_deg


def recover_axis_to_safe_range(axis="x"):
    state = _get_state(axis)
    current_deg = get_current_position(axis)

    if current_deg > MAX_DEGREE:
        target_deg = MAX_DEGREE - RECOVERY_TARGET_MARGIN_DEG
    elif current_deg < MIN_DEGREE:
        target_deg = MIN_DEGREE + RECOVERY_TARGET_MARGIN_DEG
    else:
        clear_safety_trip(axis)
        return current_deg

    # This is the only path allowed to bypass a latched safety trip, and it only
    # commands inward toward the manual safe range.
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None
    state["recovery_active"] = True
    try:
        target_motor_turns, target_deg = _command_position_target_from_selected_feedback(axis, target_deg)
        wait_until_settled(target_motor_turns, axis=axis, target_output_deg=target_deg)
        clear_safety_trip(axis)
        return target_deg
    finally:
        state["recovery_active"] = False


def go_home(axis="x"):
    state = _enter_position_trap_mode(axis)
    _align_selected_feedback_before_position_move(axis)
    state["odrive"].axis0.controller.input_pos = state["motor_home"]
    wait_until_settled(state["motor_home"], axis=axis, target_output_deg=0.0)


def move_absolute(target_output_deg, axis="x"):
    state = _enter_position_trap_mode(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    _align_selected_feedback_before_position_move(axis)

    target_output_deg = _clamp_manual_target(target_output_deg)

    target_output_turns = target_output_deg / 360.0
    target_motor_turns = state["motor_home"] + output_sign * target_output_turns * GEAR_RATIO

    state["odrive"].axis0.controller.input_pos = target_motor_turns
    wait_until_settled(target_motor_turns, axis=axis, target_output_deg=target_output_deg)


def command_absolute(target_output_deg, axis="x"):
    state = _ensure_safe_to_command(axis)
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
    state = _ensure_safe_to_command(axis)
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


def command_velocity(target_output_vel_deg_s=0.0, axis="x"):
    state = _ensure_safe_to_command(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    target_motor_vel_turns_s = output_sign * target_output_vel_deg_s * GEAR_RATIO / 360.0

    state["odrive"].axis0.requested_state = 8
    state["odrive"].axis0.controller.input_vel = target_motor_vel_turns_s


def move_relative(delta_deg, axis="x"):
    current_deg = get_current_position(axis=axis)
    if current_deg >= MAX_DEGREE - MANUAL_EDGE_GUARD_DEG and delta_deg > 0:
        raise RuntimeError(
            f"{axis.upper()} relative move blocked: selected position is {current_deg:.3f} deg "
            f"near/outside +{MAX_DEGREE:.1f} deg"
        )
    if current_deg <= MIN_DEGREE + MANUAL_EDGE_GUARD_DEG and delta_deg < 0:
        raise RuntimeError(
            f"{axis.upper()} relative move blocked: selected position is {current_deg:.3f} deg "
            f"near/outside {MIN_DEGREE:.1f} deg"
        )
    target_deg = _clamp_manual_target(current_deg + delta_deg)
    move_absolute(target_deg, axis=axis)


def get_motor_position(axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    motor_pos = state["odrive"].axis0.pos_vel_mapper.pos_rel
    return (motor_pos - state["motor_home"]) * 360.0 / (GEAR_RATIO * output_sign)


def get_current_position(axis="x"):
    state = _get_state(axis)
    if state["feedback_source"] == "external":
        return get_external_position(axis)
    return get_spi_position(axis)


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


def set_velocity_ramp_rate(ramp_rate_deg_s2, axis="x"):
    state = _get_state(axis)
    output_sign = AXIS_CONFIG[axis]["output_sign"]
    ramp_rate_motor_turns_s2 = abs(output_sign * ramp_rate_deg_s2 * GEAR_RATIO / 360.0)
    state["odrive"].axis0.controller.config.vel_ramp_rate = ramp_rate_motor_turns_s2


def enter_tracking_mode(axis="x", input_filter_bandwidth=None):
    state = _get_state(axis)
    if state["tracking_mode_active"]:
        return

    axis0 = state["odrive"].axis0
    _configure_axis_for_manual_position_control(axis)
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


def enter_velocity_mode(axis="x", input_filter_bandwidth=None, ramp_rate_deg_s2=None):
    state = _get_state(axis)
    if state["velocity_mode_active"]:
        return

    axis0 = state["odrive"].axis0
    _configure_axis_for_manual_position_control(axis)
    state["velocity_prev_control_mode"] = axis0.controller.config.control_mode
    state["velocity_prev_input_mode"] = axis0.controller.config.input_mode
    state["velocity_prev_input_filter_bandwidth"] = axis0.controller.config.input_filter_bandwidth
    state["velocity_prev_vel_ramp_rate"] = axis0.controller.config.vel_ramp_rate
    axis0.controller.config.control_mode = 2
    if ramp_rate_deg_s2 is not None and ramp_rate_deg_s2 > 0.0:
        set_velocity_ramp_rate(ramp_rate_deg_s2, axis=axis)
        axis0.controller.config.input_mode = 2
    else:
        axis0.controller.config.input_mode = 1
    if input_filter_bandwidth is not None:
        axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
    axis0.controller.input_vel = 0.0
    axis0.requested_state = 8
    state["velocity_mode_active"] = True


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


def exit_velocity_mode(axis="x"):
    state = _get_state(axis)
    if not state["velocity_mode_active"]:
        return

    axis0 = state["odrive"].axis0
    if state["velocity_prev_control_mode"] is not None:
        axis0.controller.config.control_mode = state["velocity_prev_control_mode"]
    if state["velocity_prev_input_mode"] is not None:
        axis0.controller.config.input_mode = state["velocity_prev_input_mode"]
    if state["velocity_prev_input_filter_bandwidth"] is not None:
        axis0.controller.config.input_filter_bandwidth = state["velocity_prev_input_filter_bandwidth"]
    if state["velocity_prev_vel_ramp_rate"] is not None:
        axis0.controller.config.vel_ramp_rate = state["velocity_prev_vel_ramp_rate"]
    axis0.controller.input_vel = 0.0
    axis0.requested_state = 8
    state["velocity_prev_control_mode"] = None
    state["velocity_prev_input_mode"] = None
    state["velocity_prev_input_filter_bandwidth"] = None
    state["velocity_prev_vel_ramp_rate"] = None
    state["velocity_mode_active"] = False


def _run_parallel(calls):
    threads = []
    exceptions = []

    def run_call(func, kwargs):
        try:
            func(**kwargs)
        except Exception as exc:
            exceptions.append(exc)

    for func, kwargs in calls:
        thread = threading.Thread(target=run_call, args=(func, kwargs))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    if exceptions:
        raise exceptions[0]


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


def command_velocity_pair(x_vel_deg_s=0.0, y_vel_deg_s=0.0):
    command_velocity(x_vel_deg_s, axis="x")
    command_velocity(y_vel_deg_s, axis="y")


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


def enter_velocity_mode_all(input_filter_bandwidth=None, ramp_rate_deg_s2=None):
    for axis in AXIS_CONFIG:
        enter_velocity_mode(
            axis=axis,
            input_filter_bandwidth=input_filter_bandwidth,
            ramp_rate_deg_s2=ramp_rate_deg_s2,
        )


def exit_tracking_mode_all():
    for axis in AXIS_CONFIG:
        exit_tracking_mode(axis=axis)


def exit_velocity_mode_all():
    for axis in AXIS_CONFIG:
        exit_velocity_mode(axis=axis)


def disarm_all():
    for state in AXIS_STATE.values():
        if state["odrive"] is not None:
            state["odrive"].axis0.requested_state = 1


def disarm_axis(axis="x"):
    state = _get_state(axis)
    try:
        state["odrive"].axis0.controller.input_vel = 0.0
    except Exception:
        pass
    state["odrive"].axis0.requested_state = odrive_enums.AXIS_STATE_IDLE


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
            print(f"X ODrive output estimate: {get_current_position('x'):.3f} deg")
            print(f"X SPI output:         {get_spi_position('x'):.3f} deg")
            print(f"X raw:                {get_spi_raw('x')}")

    except KeyboardInterrupt:
        print("\n[STOP] Disarming...")
    finally:
        disarm_all()
