"""Technosoft iPOS8020 BX-CAN adapter for the Triton tracker.

This module mirrors the high-level shape of the ODrive adapter, but keeps the
drive-specific TML calls behind a backend interface.  The first goal is to make
axis units, safety limits, state handling, and future PVT/velocity control paths
explicit before wiring real hardware calls.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import os
from pathlib import Path
import threading
import time
from typing import Iterable, Protocol


# ------------------ TRACKER CONFIGURATION ------------------
GEAR_RATIO = 2040.0
POSITION_TOL_DEG = 0.002
VELOCITY_TOL_DEG_S = 0.01
SETTLE_TIMEOUT_SEC = 20.0

MAX_DEGREE = 90.0
MIN_DEGREE = -90.0
MANUAL_EDGE_GUARD_DEG = 0.25
RECOVERY_TARGET_MARGIN_DEG = 5.0
TRACKING_MAX_DEGREE = 91.0
TRACKING_MIN_DEGREE = -91.0
POSITION_SAFETY_MAX_DEGREE = 92.0
POSITION_SAFETY_MIN_DEGREE = -92.0
POSITION_SAFETY_POLL_INTERVAL_SEC = 0.02
POSITION_MAX_AGE_SEC = 0.25

ENDAT_COUNTS_PER_REV = 1 << 25
X_ENDAT_HOME_RAW = 0.090259016
Y_ENDAT_HOME_RAW = 0.0
X_TECHNOSOFT_HOME_IU = 519745
Y_TECHNOSOFT_HOME_IU = 0

DEFAULT_PROFILE_VEL_DEG_S = 1.0
DEFAULT_PROFILE_ACCEL_DEG_S2 = 2.0
DEFAULT_PROFILE_DECEL_DEG_S2 = 2.0

DEFAULT_PVT_BUFFER_LOW_WATERMARK = 3

TML_LIB_ROOT = Path(r"C:\Program Files (x86)\Technosoft\TML_LIB_x64")
TML_LIB_DIR = TML_LIB_ROOT / "lib-multithread"
TML_LIB_DLL = TML_LIB_DIR / "TML_lib.dll"
DEFAULT_TML_SETUP_PATH = Path(
    r"C:\TML_Setups\ipos8020_dual_loop_master.t.zip"
)

CHANNEL_PEAK_SYS_PCAN_USB = 7
REG_SRL = 3
REG_SRH = 4
REG_MER = 5
REG_DER = 6
REG_DER2 = 7
UPDATE_NONE = -1
UPDATE_IMMEDIATE = 1
ABSOLUTE_POSITION = 0
FROM_REFERENCE = 1
FROM_MEASURE = 0
POWER_ON = 1
POWER_OFF = 0

SRL_BITS = {
    7: "Homing/CALLS warning",
    8: "Homing/CALLS active",
    10: "Motion completed",
    14: "Event occurred",
    15: "Axis ON / power stage enabled",
}

SRH_BITS = {
    0: "ENDINIT executed",
    1: "Over position trigger 1",
    2: "Over position trigger 2",
    3: "Over position trigger 3",
    4: "Over position trigger 4",
    6: "LSP event/interrupt",
    7: "LSN event/interrupt",
    8: "Capture event/interrupt",
    10: "Motor I2t warning",
    11: "Drive I2t warning",
    12: "In Gear",
    14: "In Cam",
    15: "Fault",
}

MER_BITS = {
    0: "CAN error",
    1: "Short-circuit",
    2: "Invalid setup data",
    3: "Control error: position/speed error too large",
    4: "Communication error",
    5: "Feedback error; see DER2",
    6: "Positive limit switch active",
    7: "Negative limit switch active",
    8: "Over current",
    9: "I2t protection",
    10: "Motor over temperature",
    11: "Drive over temperature",
    12: "Over voltage",
    13: "Under voltage",
    14: "Command error; see DER",
    15: "Enable/STO input disabled",
}

DER_BITS = {
    0: "TML stack overflow / nested calls exceeded",
    1: "RET/RETI with no active function/ISR",
    2: "Call to nonexistent homing routine",
    3: "Call to nonexistent function",
    4: "UPD ignored during AXISON",
    5: "Cancelable call while another cancelable function active",
    6: "Positive software limit active",
    7: "Negative software limit active",
    8: "Invalid S-curve profile",
    9: "Update ignored for S-curve",
    10: "Encoder broken wire",
    11: "Start mode failed",
    13: "Self-check / memory checksum error",
    14: "STO or enable circuit hardware error",
    15: "EEPROM locked",
}

DER2_BITS = {
    0: "BiSS data CRC error",
    1: "BiSS data warning bit set",
    2: "BiSS data error bit set",
    3: "BiSS sensor missing / no communication",
    4: "Absolute Encoder Interface error",
    5: "Hall sensor missing",
    6: "Position wraparound",
    15: "Output frequency limit",
}


def decode_register_bits(register_name: str, value: int) -> list[str]:
    """Decode Technosoft status/error register bits from the official manuals."""
    bit_maps = {
        "SRL": SRL_BITS,
        "SRH": SRH_BITS,
        "MER": MER_BITS,
        "DER": DER_BITS,
        "DER2": DER2_BITS,
    }
    mapping = bit_maps.get(register_name.upper(), {})
    return [f"bit {bit}: {text}" for bit, text in mapping.items() if int(value) & (1 << bit)]


@dataclass(frozen=True)
class AxisConfig:
    name: str
    node_id: int
    output_sign: float = 1.0
    endat_home_raw: float = 0.0
    endat_sign: float = 1.0
    endat_counts_per_rev: int = ENDAT_COUNTS_PER_REV


AXIS_CONFIG = {
    "x": AxisConfig("x", node_id=1, output_sign=1.0, endat_home_raw=X_ENDAT_HOME_RAW),
    "y": AxisConfig("y", node_id=2, output_sign=1.0, endat_home_raw=Y_ENDAT_HOME_RAW),
}

TECHNOSOFT_HOME_IU = {
    "x": X_TECHNOSOFT_HOME_IU,
    "y": Y_TECHNOSOFT_HOME_IU,
}


def set_axis_node_ids(x_node_id: int | None = None, y_node_id: int | None = None) -> None:
    """Update CAN node IDs before connecting.

    EasyMotion projects often use a different drive ID than our X=1/Y=2
    convention, so keep this adjustable from the GUI during bring-up.
    """
    updates = {"x": x_node_id, "y": y_node_id}
    for axis, node_id in updates.items():
        if node_id is None:
            continue
        node_id = int(node_id)
        if node_id < 1 or node_id > 255:
            raise ValueError("Technosoft node ID must be between 1 and 255")
        cfg = AXIS_CONFIG[axis]
        AXIS_CONFIG[axis] = AxisConfig(
            name=cfg.name,
            node_id=node_id,
            output_sign=cfg.output_sign,
            endat_home_raw=cfg.endat_home_raw,
            endat_sign=cfg.endat_sign,
            endat_counts_per_rev=cfg.endat_counts_per_rev,
        )


@dataclass(frozen=True)
class PVTPoint:
    """A drive-timed PVT segment endpoint.

    position_deg is axis/load position at the segment end.
    velocity_deg_s is axis/load velocity at the segment end.
    duration_s is segment time from the previous point to this point.
    """

    position_deg: float
    velocity_deg_s: float
    duration_s: float


class TechnosoftBackend(Protocol):
    def connect_axis(self, axis: str, config: AxisConfig) -> object:
        ...

    def disconnect_axis(self, axis: str) -> None:
        ...

    def reset_faults(self, axis: str) -> None:
        ...

    def reset_drive(self, axis: str) -> None:
        ...

    def align_reference_to_actual(self, axis: str) -> None:
        ...

    def stop_axis(self, axis: str) -> None:
        ...

    def read_status(self, axis: str) -> int:
        ...

    def read_faults(self, axis: str) -> int:
        ...

    def read_load_position_counts(self, axis: str) -> int:
        ...

    def read_motor_position_counts(self, axis: str) -> int:
        ...

    def read_command_position_counts(self, axis: str) -> int:
        ...

    def read_target_position_counts(self, axis: str) -> int:
        ...

    def read_drive_gains(self, axis: str) -> dict[str, int]:
        ...

    def scale_drive_gains(self, axis: str, scale: float) -> dict[str, dict[str, int]]:
        ...

    def read_load_velocity_deg_s(self, axis: str) -> float:
        ...

    def set_profile_limits(self, axis: str, vel_deg_s: float, acc_deg_s2: float, dec_deg_s2: float) -> None:
        ...

    def move_absolute_axis_deg(self, axis: str, position_deg: float) -> None:
        ...

    def move_velocity_axis_deg_s(self, axis: str, velocity_deg_s: float) -> None:
        ...

    def enter_velocity_mode(self, axis: str) -> None:
        ...

    def exit_velocity_mode(self, axis: str) -> None:
        ...

    def pvt_setup(self, axis: str, absolute: bool = True) -> None:
        ...

    def pvt_clear(self, axis: str) -> None:
        ...

    def pvt_send_first_point(self, axis: str, point: PVTPoint) -> None:
        ...

    def pvt_send_point(self, axis: str, point: PVTPoint) -> None:
        ...

    def pvt_start(self, axis: str) -> None:
        ...

    def pvt_buffer_level(self, axis: str) -> int:
        ...


class BackendNotConnected(RuntimeError):
    pass


@dataclass(frozen=True)
class TMLConnectionConfig:
    """Connection and setup data for the Technosoft TML library."""

    dll_path: Path = TML_LIB_DLL
    channel_name: str = "1"
    channel_type: int = CHANNEL_PEAK_SYS_PCAN_USB
    host_id: int = 255
    baudrate: int = 1000000
    x_setup_path: Path | None = DEFAULT_TML_SETUP_PATH
    y_setup_path: Path | None = DEFAULT_TML_SETUP_PATH
    auto_power_on: bool = False
    run_drive_initialisation: bool = False
    command_uses_load_scaling: bool = False


class TMLLibBackend:
    """ctypes binding for Technosoft TML_LIB.

    The real drive API is stateful: one channel and one selected axis are active
    at a time.  All calls are serialized through _lock so the GUI/tracking loop
    cannot accidentally interleave axis selections.
    """

    def __init__(self, config: TMLConnectionConfig | None = None, **overrides) -> None:
        if config is None:
            config = TMLConnectionConfig()
        if overrides:
            config = TMLConnectionConfig(**{**config.__dict__, **overrides})
        self.config = config
        self.dll = None
        self._dll_dir_cookie = None
        self._lock = threading.RLock()
        self._local = threading.local()
        self._channel_fds: set[int] = set()
        self._connected_axes: set[str] = set()
        self._setup_indices: dict[str, int] = {}
        self._position_scales_rev_per_iu: dict[str, dict[str, float]] = {}
        self._time_scale_ms_per_iu: dict[str, float] = {}
        self._profiles: dict[str, tuple[float, float, float]] = {}
        self._pvt_counters: dict[str, int] = {}
        self._pvt_levels: dict[str, int] = {}

    def _load_dll(self) -> None:
        if self.dll is not None:
            return
        dll_path = Path(self.config.dll_path)
        if not dll_path.exists():
            raise FileNotFoundError(f"TML_LIB DLL not found: {dll_path}")
        dll_dir = dll_path.parent
        if hasattr(os, "add_dll_directory"):
            self._dll_dir_cookie = os.add_dll_directory(str(dll_dir))
        self.dll = ctypes.WinDLL(str(dll_path))
        self._bind_signatures()

    def _bind_signatures(self) -> None:
        d = self.dll
        d.TS_Basic_GetLastErrorText.argtypes = [ctypes.c_char_p, ctypes.c_int32]
        d.TS_Basic_GetLastErrorText.restype = None
        d.TS_OpenChannel.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_uint32]
        d.TS_OpenChannel.restype = ctypes.c_int32
        d.TS_SelectChannel.argtypes = [ctypes.c_int32]
        d.TS_SelectChannel.restype = ctypes.c_int32
        d.TS_CloseChannel.argtypes = [ctypes.c_int32]
        d.TS_CloseChannel.restype = None
        d.TS_LoadSetup.argtypes = [ctypes.c_char_p]
        d.TS_LoadSetup.restype = ctypes.c_int32
        d.TS_SetupAxis.argtypes = [ctypes.c_ubyte, ctypes.c_int32]
        d.TS_SetupAxis.restype = ctypes.c_int32
        d.TS_SelectAxis.argtypes = [ctypes.c_ubyte]
        d.TS_SelectAxis.restype = ctypes.c_int32
        d.TS_DriveInitialisation.argtypes = []
        d.TS_DriveInitialisation.restype = ctypes.c_int32
        d.TS_Reset.argtypes = []
        d.TS_Reset.restype = ctypes.c_int32
        d.TS_ResetFault.argtypes = []
        d.TS_ResetFault.restype = ctypes.c_int32
        d.TS_Execute.argtypes = [ctypes.c_char_p]
        d.TS_Execute.restype = ctypes.c_int32
        d.TS_Power.argtypes = [ctypes.c_int32]
        d.TS_Power.restype = ctypes.c_int32
        d.TS_Stop.argtypes = []
        d.TS_Stop.restype = ctypes.c_int32
        d.TS_ReadStatus.argtypes = [ctypes.c_int16, ctypes.POINTER(ctypes.c_uint16)]
        d.TS_ReadStatus.restype = ctypes.c_int32
        d.TS_GetLongVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int32)]
        d.TS_GetLongVariable.restype = ctypes.c_int32
        d.TS_SetIntVariable.argtypes = [ctypes.c_char_p, ctypes.c_int16]
        d.TS_SetIntVariable.restype = ctypes.c_int32
        d.TS_GetIntVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int16)]
        d.TS_GetIntVariable.restype = ctypes.c_int32
        d.TS_GetFixedVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)]
        d.TS_GetFixedVariable.restype = ctypes.c_int32
        d.TS_MoveAbsolute.argtypes = [
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_MoveAbsolute.restype = ctypes.c_int32
        d.TS_MoveVelocity.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_int16, ctypes.c_int16]
        d.TS_MoveVelocity.restype = ctypes.c_int32
        d.TS_PVTSetup.argtypes = [
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_PVTSetup.restype = ctypes.c_int32
        d.TS_SendPVTFirstPoint.argtypes = [
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_uint32,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int32,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_SendPVTFirstPoint.restype = ctypes.c_int32
        d.TS_SendPVTPoint.argtypes = [ctypes.c_int32, ctypes.c_double, ctypes.c_uint32, ctypes.c_int16]
        d.TS_SendPVTPoint.restype = ctypes.c_int32
        d.TS_UpdateImmediate.argtypes = []
        d.TS_UpdateImmediate.restype = ctypes.c_int32
        d.TS_SetTargetPositionToActual.argtypes = []
        d.TS_SetTargetPositionToActual.restype = ctypes.c_int32
        d.TS_GetMotorPositionScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetMotorPositionScalingFactor.restype = ctypes.c_int32
        d.TS_GetLoadPositionScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetLoadPositionScalingFactor.restype = ctypes.c_int32
        d.TS_GetTimeScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetTimeScalingFactor.restype = ctypes.c_int32

    def _last_error(self) -> str:
        if self.dll is None:
            return "TML_LIB is not loaded"
        buffer = ctypes.create_string_buffer(1024)
        self.dll.TS_Basic_GetLastErrorText(buffer, len(buffer))
        text = buffer.value.decode("mbcs", errors="replace").strip()
        return text or "TML_LIB call failed"

    def _check(self, ok: int, action: str) -> None:
        if not ok:
            raise RuntimeError(f"{action} failed: {self._last_error()}")

    def _axis_setup_path(self, axis: str) -> Path:
        configured = self.config.x_setup_path if axis == "x" else self.config.y_setup_path
        env_name = f"TECHNOSOFT_{axis.upper()}_SETUP"
        path = configured or os.environ.get(env_name)
        if not path:
            raise RuntimeError(
                f"Missing setup file for {axis.upper()} iPOS8020. Set {env_name} to the "
                "*.t.zip exported from EasyMotion Studio before using the real backend."
            )
        setup_path = Path(path)
        if not setup_path.exists():
            raise FileNotFoundError(f"{axis.upper()} setup file not found: {setup_path}")
        return setup_path

    def _open_channel_if_needed(self) -> None:
        self._load_dll()
        if getattr(self._local, "channel_fd", None) is not None:
            return
        fd = self.dll.TS_OpenChannel(
            self.config.channel_name.encode("ascii"),
            int(self.config.channel_type),
            int(self.config.host_id),
            int(self.config.baudrate),
        )
        if fd < 0:
            raise RuntimeError(f"TS_OpenChannel failed: {self._last_error()}")
        self._local.channel_fd = int(fd)
        self._channel_fds.add(int(fd))
        self._local.setup_axes = set()

    def _close_thread_channel(self) -> None:
        if self.dll is None:
            return
        fd = getattr(self._local, "channel_fd", None)
        if fd is None:
            return
        self.dll.TS_CloseChannel(int(fd))
        self._channel_fds.discard(int(fd))
        self._local.channel_fd = None

    def _select_axis(self, axis: str) -> None:
        self._open_channel_if_needed()
        cfg = AXIS_CONFIG[axis]
        setup_axes = getattr(self._local, "setup_axes", set())
        if axis not in setup_axes:
            idx_setup = self._setup_indices.get(axis)
            if idx_setup is None:
                setup_path = self._axis_setup_path(axis)
                idx_setup = self.dll.TS_LoadSetup(str(setup_path).encode("mbcs"))
                if idx_setup < 0:
                    raise RuntimeError(f"TS_LoadSetup failed for {axis.upper()}: {self._last_error()}")
                self._setup_indices[axis] = int(idx_setup)
            self._check(self.dll.TS_SetupAxis(int(cfg.node_id), int(idx_setup)), f"TS_SetupAxis({cfg.node_id})")
            setup_axes.add(axis)
            self._local.setup_axes = setup_axes
        self._check(self.dll.TS_SelectAxis(int(cfg.node_id)), f"TS_SelectAxis({cfg.node_id})")

    def _read_double_scaling(self, function_name: str, axis: str) -> float:
        value = ctypes.c_double()
        self._check(getattr(self.dll, function_name)(ctypes.byref(value)), f"{function_name} {axis.upper()}")
        return float(value.value)

    def _command_position_scale(self, axis: str) -> float:
        scales = self._position_scales_rev_per_iu[axis]
        return scales["load" if self.config.command_uses_load_scaling else "motor"]

    def _axis_deg_to_command_rev(self, axis: str, position_deg: float) -> float:
        output_rev = float(position_deg) / 360.0
        if self.config.command_uses_load_scaling:
            return AXIS_CONFIG[axis].output_sign * output_rev
        return AXIS_CONFIG[axis].output_sign * output_rev * GEAR_RATIO

    def _command_rev_to_axis_deg(self, axis: str, command_rev: float) -> float:
        if self.config.command_uses_load_scaling:
            output_rev = AXIS_CONFIG[axis].output_sign * float(command_rev)
        else:
            output_rev = AXIS_CONFIG[axis].output_sign * float(command_rev) / GEAR_RATIO
        return output_rev * 360.0

    def _deg_to_iu(self, axis: str, position_deg: float) -> int:
        scale = self._command_position_scale(axis)
        if scale == 0.0:
            raise ZeroDivisionError(f"{axis.upper()} position scaling factor is zero")
        return int(round(self._axis_deg_to_command_rev(axis, position_deg) * scale))

    def _iu_to_axis_deg(self, axis: str, position_iu: int) -> float:
        scale = self._command_position_scale(axis)
        if scale == 0.0:
            raise ZeroDivisionError(f"{axis.upper()} position scaling factor is zero")
        return self._command_rev_to_axis_deg(axis, float(position_iu) / scale)

    def _load_iu_to_axis_deg(self, axis: str, position_iu: int) -> float:
        scale = self._position_scales_rev_per_iu[axis]["load"]
        if scale == 0.0:
            raise ZeroDivisionError(f"{axis.upper()} load position scaling factor is zero")
        output_rev = AXIS_CONFIG[axis].output_sign * float(position_iu) / scale
        return output_rev * 360.0

    def _time_to_iu(self, axis: str, duration_s: float) -> int:
        scale_ms = self._time_scale_ms_per_iu.get(axis, 1.0)
        if scale_ms <= 0.0:
            scale_ms = 1.0
        return max(1, int(round(float(duration_s) * 1000.0 / scale_ms)))

    def _vel_to_iu_per_sample(self, axis: str, velocity_deg_s: float) -> float:
        scale = self._command_position_scale(axis)
        time_scale_s = self._time_scale_ms_per_iu.get(axis, 1.0) / 1000.0
        rev_s = self._axis_deg_to_command_rev(axis, velocity_deg_s) / 1.0
        return rev_s * scale * time_scale_s

    def _iu_per_sample_to_deg_s(self, axis: str, velocity_iu: float) -> float:
        scale = self._command_position_scale(axis)
        time_scale_s = self._time_scale_ms_per_iu.get(axis, 1.0) / 1000.0
        if time_scale_s <= 0.0 or scale == 0.0:
            return 0.0
        return self._command_rev_to_axis_deg(axis, float(velocity_iu) / (scale * time_scale_s))

    def _acc_to_iu_per_sample2(self, axis: str, accel_deg_s2: float) -> float:
        scale = self._command_position_scale(axis)
        time_scale_s = self._time_scale_ms_per_iu.get(axis, 1.0) / 1000.0
        rev_s2 = abs(self._axis_deg_to_command_rev(axis, accel_deg_s2))
        return rev_s2 * scale * time_scale_s * time_scale_s

    def _read_long_variable(self, name: str) -> int:
        value = ctypes.c_int32()
        self._check(self.dll.TS_GetLongVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetLongVariable({name})")
        return int(value.value)

    def _read_int_variable(self, name: str) -> int:
        value = ctypes.c_int16()
        self._check(self.dll.TS_GetIntVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetIntVariable({name})")
        return int(value.value)

    def _write_int_variable(self, name: str, value: int) -> None:
        value = max(min(int(value), 32767), -32768)
        self._check(
            self.dll.TS_SetIntVariable(name.encode("ascii"), ctypes.c_int16(value)),
            f"TS_SetIntVariable({name})",
        )

    def _read_fixed_variable(self, name: str) -> float:
        value = ctypes.c_double()
        self._check(self.dll.TS_GetFixedVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetFixedVariable({name})")
        return float(value.value)

    def connect_axis(self, axis: str, config: AxisConfig) -> object:
        with self._lock:
            self._open_channel_if_needed()
            setup_path = self._axis_setup_path(axis)
            idx_setup = self.dll.TS_LoadSetup(str(setup_path).encode("mbcs"))
            if idx_setup < 0:
                raise RuntimeError(f"TS_LoadSetup failed for {axis.upper()}: {self._last_error()}")
            self._setup_indices[axis] = int(idx_setup)
            self._check(self.dll.TS_SetupAxis(int(config.node_id), int(idx_setup)), f"TS_SetupAxis({config.node_id})")
            setup_axes = getattr(self._local, "setup_axes", set())
            setup_axes.add(axis)
            self._local.setup_axes = setup_axes
            self._select_axis(axis)
            if self.config.run_drive_initialisation:
                self._check(self.dll.TS_DriveInitialisation(), f"TS_DriveInitialisation {axis.upper()}")
            if self.config.auto_power_on:
                self._check(self.dll.TS_Power(POWER_ON), f"TS_Power ON {axis.upper()}")
            self._position_scales_rev_per_iu[axis] = {
                "motor": self._read_double_scaling("TS_GetMotorPositionScalingFactor", axis),
                "load": self._read_double_scaling("TS_GetLoadPositionScalingFactor", axis),
            }
            self._time_scale_ms_per_iu[axis] = self._read_double_scaling("TS_GetTimeScalingFactor", axis)
            self._profiles[axis] = (
                DEFAULT_PROFILE_VEL_DEG_S,
                DEFAULT_PROFILE_ACCEL_DEG_S2,
                DEFAULT_PROFILE_DECEL_DEG_S2,
            )
            self._pvt_counters[axis] = 0
            self._pvt_levels[axis] = 0
            self._connected_axes.add(axis)
            return {
                "node_id": config.node_id,
                "setup_path": str(setup_path),
                "channel_fd": getattr(self._local, "channel_fd", None),
                "auto_power_on": self.config.auto_power_on,
            }

    def disconnect_axis(self, axis: str) -> None:
        with self._lock:
            if self.dll is None:
                return
            if axis in self._connected_axes:
                try:
                    self._select_axis(axis)
                    self.dll.TS_Stop()
                    if self.config.auto_power_on:
                        self.dll.TS_Power(POWER_OFF)
                finally:
                    self._connected_axes.discard(axis)
            if not self._connected_axes:
                for fd in list(self._channel_fds):
                    self.dll.TS_CloseChannel(int(fd))
                self._channel_fds.clear()
                self._local.channel_fd = None

    def reset_faults(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_ResetFault(), f"TS_ResetFault {axis.upper()}")

    def reset_drive(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_Reset(), f"TS_Reset {axis.upper()}")
            self._connected_axes.discard(axis)
            setup_axes = getattr(self._local, "setup_axes", set())
            setup_axes.discard(axis)
            self._local.setup_axes = setup_axes

    def align_reference_to_actual(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(
                self.dll.TS_SetTargetPositionToActual(),
                f"TS_SetTargetPositionToActual {axis.upper()}",
            )

    def power_axis(self, axis: str, enable: bool = True) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(
                self.dll.TS_Power(POWER_ON if enable else POWER_OFF),
                f"TS_Power {'ON' if enable else 'OFF'} {axis.upper()}",
            )

    def axis_on_command(self, axis: str, enable: bool = True) -> None:
        with self._lock:
            self._select_axis(axis)
            command = b"AXISON;" if enable else b"AXISOFF;"
            self._check(
                self.dll.TS_Execute(command),
                f"{command.decode('ascii').strip(';')} {axis.upper()}",
            )

    def axis_on_update_command(self, axis: str, enable: bool = True) -> None:
        with self._lock:
            self._select_axis(axis)
            command = b"AXISON;" if enable else b"AXISOFF;"
            self._check(
                self.dll.TS_Execute(command),
                f"{command.decode('ascii').strip(';')} {axis.upper()}",
            )
            self._check(self.dll.TS_UpdateImmediate(), f"TS_UpdateImmediate {axis.upper()}")

    def execute_tml(self, axis: str, command: str) -> None:
        with self._lock:
            self._select_axis(axis)
            encoded = command.strip().encode("ascii")
            self._check(self.dll.TS_Execute(encoded), f"TS_Execute({command!r}) {axis.upper()}")

    def end_initialization(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_Execute(b"ENDINIT;"), f"ENDINIT {axis.upper()}")

    def stop_axis(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_Stop(), f"TS_Stop {axis.upper()}")

    def read_status(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            status = ctypes.c_uint16()
            self._check(self.dll.TS_ReadStatus(REG_SRL, ctypes.byref(status)), f"TS_ReadStatus SRL {axis.upper()}")
            return int(status.value)

    def read_status_registers(self, axis: str) -> dict[str, int]:
        with self._lock:
            self._select_axis(axis)
            registers = {}
            for name, register in (("SRL", REG_SRL), ("SRH", REG_SRH)):
                value = ctypes.c_uint16()
                self._check(self.dll.TS_ReadStatus(register, ctypes.byref(value)), f"TS_ReadStatus {name} {axis.upper()}")
                registers[name] = int(value.value)
            return registers

    def read_fault_registers(self, axis: str) -> dict[str, int]:
        with self._lock:
            self._select_axis(axis)
            registers = {}
            for name, register in (("MER", REG_MER), ("DER", REG_DER), ("DER2", REG_DER2)):
                value = ctypes.c_uint16()
                self._check(self.dll.TS_ReadStatus(register, ctypes.byref(value)), f"TS_ReadStatus {name} {axis.upper()}")
                registers[name] = int(value.value)
            return registers

    def read_diagnostic_snapshot(self, axis: str) -> dict[str, int]:
        snapshot = {}
        snapshot.update(self.read_status_registers(axis))
        snapshot.update(self.read_fault_registers(axis))
        return snapshot

    def drive_initialisation_diagnostic(self, axis: str) -> dict:
        with self._lock:
            self._select_axis(axis)
            before = {}
            for name, register in (
                ("SRL", REG_SRL),
                ("SRH", REG_SRH),
                ("MER", REG_MER),
                ("DER", REG_DER),
                ("DER2", REG_DER2),
            ):
                value = ctypes.c_uint16()
                self._check(self.dll.TS_ReadStatus(register, ctypes.byref(value)), f"TS_ReadStatus {name} {axis.upper()}")
                before[name] = int(value.value)

            ok = bool(self.dll.TS_DriveInitialisation())
            error = None if ok else self._last_error()

            # Give the firmware a short moment to latch the post-ENDINIT fault bits.
            time.sleep(0.05)
            after = {}
            for name, register in (
                ("SRL", REG_SRL),
                ("SRH", REG_SRH),
                ("MER", REG_MER),
                ("DER", REG_DER),
                ("DER2", REG_DER2),
            ):
                value = ctypes.c_uint16()
                self._check(self.dll.TS_ReadStatus(register, ctypes.byref(value)), f"TS_ReadStatus {name} {axis.upper()}")
                after[name] = int(value.value)

        return {
            "ok": ok,
            "error": error,
            "before": before,
            "after": after,
            "decoded_after": {
                name: decode_register_bits(name, value)
                for name, value in after.items()
            },
        }

    def read_faults(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            fault = 0
            for shift, register in ((0, REG_MER), (16, REG_DER), (32, REG_DER2)):
                value = ctypes.c_uint16()
                self._check(self.dll.TS_ReadStatus(register, ctypes.byref(value)), f"TS_ReadStatus fault {axis.upper()}")
                fault |= int(value.value) << shift
            return fault

    def read_load_position_counts(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            position_iu = self._read_long_variable("APOS")
            home_iu = int(TECHNOSOFT_HOME_IU[axis])
            position_deg = self._load_iu_to_axis_deg(axis, position_iu - home_iu)
            return axis_deg_to_endat_counts(axis, position_deg)

    def read_motor_position_counts(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            return self._read_long_variable("APOS")

    def read_command_position_counts(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            return self._read_long_variable("CPOS")

    def read_target_position_counts(self, axis: str) -> int:
        with self._lock:
            self._select_axis(axis)
            return self._read_long_variable("TPOS")

    def read_drive_gains(self, axis: str) -> dict[str, int]:
        with self._lock:
            self._select_axis(axis)
            return {
                name: self._read_int_variable(name)
                for name in ("KPP", "KIP", "KDP", "KPS", "KIS")
            }

    def scale_drive_gains(self, axis: str, scale: float) -> dict[str, dict[str, int]]:
        if scale <= 0.0:
            raise ValueError("Gain scale must be positive")
        with self._lock:
            self._select_axis(axis)
            before = self.read_drive_gains(axis)
            after = {}
            for name, old_value in before.items():
                new_value = int(round(old_value * float(scale)))
                after[name] = new_value
                self._write_int_variable(name, new_value)
            return {"before": before, "after": after}

    def read_load_velocity_deg_s(self, axis: str) -> float:
        with self._lock:
            self._select_axis(axis)
            try:
                velocity_iu = self._read_fixed_variable("ASPD")
            except RuntimeError:
                velocity_iu = float(self._read_long_variable("ASPD"))
            return self._iu_per_sample_to_deg_s(axis, velocity_iu)

    def set_profile_limits(self, axis: str, vel_deg_s: float, acc_deg_s2: float, dec_deg_s2: float) -> None:
        self._profiles[axis] = (float(vel_deg_s), float(acc_deg_s2), float(dec_deg_s2))

    def move_absolute_axis_deg(self, axis: str, position_deg: float) -> None:
        with self._lock:
            self._select_axis(axis)
            vel, acc, _dec = self._profiles.get(
                axis,
                (DEFAULT_PROFILE_VEL_DEG_S, DEFAULT_PROFILE_ACCEL_DEG_S2, DEFAULT_PROFILE_DECEL_DEG_S2),
            )
            self._check(
                self.dll.TS_MoveAbsolute(
                    self._deg_to_iu(axis, position_deg),
                    self._vel_to_iu_per_sample(axis, vel),
                    self._acc_to_iu_per_sample2(axis, acc),
                    UPDATE_IMMEDIATE,
                    FROM_MEASURE,
                ),
                f"TS_MoveAbsolute {axis.upper()}",
            )

    def move_velocity_axis_deg_s(self, axis: str, velocity_deg_s: float) -> None:
        with self._lock:
            self._select_axis(axis)
            _vel, acc, _dec = self._profiles.get(
                axis,
                (DEFAULT_PROFILE_VEL_DEG_S, DEFAULT_PROFILE_ACCEL_DEG_S2, DEFAULT_PROFILE_DECEL_DEG_S2),
            )
            self._check(
                self.dll.TS_MoveVelocity(
                    self._vel_to_iu_per_sample(axis, velocity_deg_s),
                    self._acc_to_iu_per_sample2(axis, acc),
                    UPDATE_IMMEDIATE,
                    FROM_REFERENCE,
                ),
                f"TS_MoveVelocity {axis.upper()}",
            )

    def enter_velocity_mode(self, axis: str) -> None:
        # Technosoft jog/velocity mode is entered by issuing TS_MoveVelocity.
        _validate_axis(axis)

    def exit_velocity_mode(self, axis: str) -> None:
        self.stop_axis(axis)

    def pvt_setup(self, axis: str, absolute: bool = True) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_Stop(), f"TS_Stop before PVT {axis.upper()}")
            self._check(
                self.dll.TS_PVTSetup(1, 1, 1, 1 if absolute else 0, 1, 1, DEFAULT_PVT_BUFFER_LOW_WATERMARK),
                f"TS_PVTSetup {axis.upper()}",
            )
            self._pvt_counters[axis] = 0
            self._pvt_levels[axis] = 0

    def pvt_clear(self, axis: str) -> None:
        self.pvt_setup(axis, absolute=True)

    def pvt_send_first_point(self, axis: str, point: PVTPoint) -> None:
        with self._lock:
            self._select_axis(axis)
            counter = 0
            self._check(
                self.dll.TS_SendPVTFirstPoint(
                    self._deg_to_iu(axis, point.position_deg),
                    self._vel_to_iu_per_sample(axis, point.velocity_deg_s),
                    self._time_to_iu(axis, point.duration_s),
                    counter,
                    ABSOLUTE_POSITION,
                    0,
                    UPDATE_NONE,
                    FROM_REFERENCE,
                ),
                f"TS_SendPVTFirstPoint {axis.upper()}",
            )
            self._pvt_counters[axis] = 1
            self._pvt_levels[axis] = 1

    def pvt_send_point(self, axis: str, point: PVTPoint) -> None:
        with self._lock:
            self._select_axis(axis)
            counter = self._pvt_counters.get(axis, 1) & 0x3F
            self._check(
                self.dll.TS_SendPVTPoint(
                    self._deg_to_iu(axis, point.position_deg),
                    self._vel_to_iu_per_sample(axis, point.velocity_deg_s),
                    self._time_to_iu(axis, point.duration_s),
                    counter,
                ),
                f"TS_SendPVTPoint {axis.upper()}",
            )
            self._pvt_counters[axis] = (counter + 1) & 0x3F
            self._pvt_levels[axis] = self._pvt_levels.get(axis, 0) + 1

    def pvt_start(self, axis: str) -> None:
        with self._lock:
            self._select_axis(axis)
            self._check(self.dll.TS_UpdateImmediate(), f"TS_UpdateImmediate PVT start {axis.upper()}")

    def pvt_buffer_level(self, axis: str) -> int:
        return int(self._pvt_levels.get(axis, 0))


class SimulatedTechnosoftBackend:
    """Safe no-hardware backend used for GUI and unit smoke tests."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._axes = {}

    def connect_axis(self, axis: str, config: AxisConfig) -> object:
        with self._lock:
            self._axes.setdefault(
                axis,
                {
                    "position_deg": 0.0,
                    "velocity_deg_s": 0.0,
                    "motor_counts": 0,
                    "faults": 0,
                    "status": 0,
                    "profile": (
                        DEFAULT_PROFILE_VEL_DEG_S,
                        DEFAULT_PROFILE_ACCEL_DEG_S2,
                        DEFAULT_PROFILE_DECEL_DEG_S2,
                    ),
                    "pvt": [],
                },
            )
        return self._axes[axis]

    def disconnect_axis(self, axis: str) -> None:
        with self._lock:
            self._axes.pop(axis, None)

    def _axis(self, axis: str) -> dict:
        with self._lock:
            if axis not in self._axes:
                raise BackendNotConnected(f"{axis.upper()} iPOS8020 is not connected")
            return self._axes[axis]

    def reset_faults(self, axis: str) -> None:
        self._axis(axis)["faults"] = 0

    def reset_drive(self, axis: str) -> None:
        axis_state = self._axis(axis)
        axis_state["faults"] = 0
        axis_state["enabled"] = False
        axis_state["velocity_deg_s"] = 0.0

    def align_reference_to_actual(self, axis: str) -> None:
        axis_state = self._axis(axis)
        axis_state["target_counts"] = self.read_load_position_counts(axis)

    def stop_axis(self, axis: str) -> None:
        self._axis(axis)["velocity_deg_s"] = 0.0

    def read_status(self, axis: str) -> int:
        return int(self._axis(axis)["status"])

    def read_faults(self, axis: str) -> int:
        return int(self._axis(axis)["faults"])

    def read_load_position_counts(self, axis: str) -> int:
        cfg = AXIS_CONFIG[axis]
        position_deg = float(self._axis(axis)["position_deg"])
        raw_turn = (cfg.endat_home_raw + cfg.endat_sign * position_deg / 360.0) % 1.0
        return int(round(raw_turn * cfg.endat_counts_per_rev))

    def read_motor_position_counts(self, axis: str) -> int:
        return int(self._axis(axis)["motor_counts"])

    def read_command_position_counts(self, axis: str) -> int:
        return int(round(float(self._axis(axis)["position_deg"]) / 360.0 * ENDAT_COUNTS_PER_REV))

    def read_target_position_counts(self, axis: str) -> int:
        state = self._axis(axis)
        return int(state.get("target_counts", self.read_command_position_counts(axis)))

    def read_drive_gains(self, axis: str) -> dict[str, int]:
        state = self._axis(axis)
        return dict(state.setdefault("gains", {"KPP": 100, "KIP": 0, "KDP": 0, "KPS": 100, "KIS": 0}))

    def scale_drive_gains(self, axis: str, scale: float) -> dict[str, dict[str, int]]:
        before = self.read_drive_gains(axis)
        after = {name: int(round(value * float(scale))) for name, value in before.items()}
        self._axis(axis)["gains"] = after
        return {"before": before, "after": after}

    def read_load_velocity_deg_s(self, axis: str) -> float:
        return float(self._axis(axis)["velocity_deg_s"])

    def set_profile_limits(self, axis: str, vel_deg_s: float, acc_deg_s2: float, dec_deg_s2: float) -> None:
        self._axis(axis)["profile"] = (float(vel_deg_s), float(acc_deg_s2), float(dec_deg_s2))

    def move_absolute_axis_deg(self, axis: str, position_deg: float) -> None:
        state = self._axis(axis)
        state["position_deg"] = float(position_deg)
        state["velocity_deg_s"] = 0.0

    def move_velocity_axis_deg_s(self, axis: str, velocity_deg_s: float) -> None:
        self._axis(axis)["velocity_deg_s"] = float(velocity_deg_s)

    def enter_velocity_mode(self, axis: str) -> None:
        self._axis(axis)["status"] = 2

    def exit_velocity_mode(self, axis: str) -> None:
        self.stop_axis(axis)
        self._axis(axis)["status"] = 0

    def pvt_setup(self, axis: str, absolute: bool = True) -> None:
        self._axis(axis)["pvt"] = []

    def pvt_clear(self, axis: str) -> None:
        self._axis(axis)["pvt"] = []

    def pvt_send_first_point(self, axis: str, point: PVTPoint) -> None:
        self._axis(axis)["pvt"] = [point]

    def pvt_send_point(self, axis: str, point: PVTPoint) -> None:
        self._axis(axis)["pvt"].append(point)

    def pvt_start(self, axis: str) -> None:
        state = self._axis(axis)
        if state["pvt"]:
            point = state["pvt"][-1]
            state["position_deg"] = point.position_deg
            state["velocity_deg_s"] = point.velocity_deg_s

    def pvt_buffer_level(self, axis: str) -> int:
        return len(self._axis(axis)["pvt"])


_backend: TechnosoftBackend = TMLLibBackend()
_safety_monitor_thread = None
_safety_monitor_stop_event = threading.Event()

AXIS_STATE = {
    axis: {
        "device": None,
        "connected": False,
        "position_deg": None,
        "position_timestamp": None,
        "home_offset_deg": 0.0,
        "velocity_mode_active": False,
        "pvt_mode_active": False,
        "safety_tripped": False,
        "safety_trip_reason": None,
        "recovery_active": False,
    }
    for axis in AXIS_CONFIG
}


def set_backend(backend: TechnosoftBackend) -> None:
    """Inject the real TML backend or the simulator before connecting."""
    global _backend
    _backend = backend


def use_simulated_backend() -> None:
    set_backend(SimulatedTechnosoftBackend())


def use_tml_backend(config: TMLConnectionConfig | None = None, **overrides) -> None:
    set_backend(TMLLibBackend(config=config, **overrides))


def _validate_axis(axis: str) -> None:
    if axis not in AXIS_CONFIG:
        raise ValueError(f"Unknown axis '{axis}'. Expected one of: {', '.join(AXIS_CONFIG)}")


def _get_state(axis: str = "x") -> dict:
    _validate_axis(axis)
    state = AXIS_STATE[axis]
    if not state["connected"]:
        raise RuntimeError(f"{axis.upper()} iPOS8020 is not connected")
    return state


def wrapped_raw_delta(raw_turn: float, reference_turn: float) -> float:
    delta = raw_turn - reference_turn
    while delta >= 0.5:
        delta -= 1.0
    while delta < -0.5:
        delta += 1.0
    return delta


def endat_counts_to_axis_deg(axis: str, counts: int) -> float:
    cfg = AXIS_CONFIG[axis]
    raw_turn = endat_counts_to_raw_turn(axis, counts)
    return cfg.endat_sign * wrapped_raw_delta(raw_turn, cfg.endat_home_raw) * 360.0


def endat_counts_to_raw_turn(axis: str, counts: int) -> float:
    cfg = AXIS_CONFIG[axis]
    return (int(counts) % cfg.endat_counts_per_rev) / float(cfg.endat_counts_per_rev)


def axis_deg_to_endat_counts(axis: str, position_deg: float) -> int:
    cfg = AXIS_CONFIG[axis]
    raw_turn = (cfg.endat_home_raw + cfg.endat_sign * float(position_deg) / 360.0) % 1.0
    return int(round(raw_turn * cfg.endat_counts_per_rev))


def wrap_axis_deg(position_deg: float) -> float:
    position_deg = float(position_deg)
    while position_deg >= 180.0:
        position_deg -= 360.0
    while position_deg < -180.0:
        position_deg += 360.0
    return position_deg


def _read_drive_position_deg(axis: str) -> float:
    counts = _backend.read_load_position_counts(axis)
    return endat_counts_to_axis_deg(axis, counts)


def _drive_to_application_deg(axis: str, drive_position_deg: float) -> float:
    state = AXIS_STATE[axis]
    return wrap_axis_deg(float(drive_position_deg) - float(state["home_offset_deg"]))


def _application_to_drive_deg(axis: str, application_position_deg: float) -> float:
    state = AXIS_STATE[axis]
    return float(application_position_deg) + float(state["home_offset_deg"])


def _refresh_position(axis: str) -> float:
    state = _get_state(axis)
    drive_position_deg = _read_drive_position_deg(axis)
    position_deg = _drive_to_application_deg(axis, drive_position_deg)
    state["position_deg"] = position_deg
    state["position_timestamp"] = time.monotonic()
    return position_deg


def get_current_position(axis: str = "x") -> float:
    return _refresh_position(axis)


def get_motor_position(axis: str = "x") -> float:
    _get_state(axis)
    # Until the real backend exposes motor counts scaling, report load position
    # for GUI compatibility.  The TML binding will replace this with motor units.
    return get_current_position(axis)


def get_motor_raw(axis: str = "x") -> float:
    _get_state(axis)
    return float(_backend.read_motor_position_counts(axis))


def get_command_raw(axis: str = "x") -> float:
    _get_state(axis)
    return float(_backend.read_command_position_counts(axis))


def get_target_raw(axis: str = "x") -> float:
    _get_state(axis)
    return float(_backend.read_target_position_counts(axis))


def get_drive_gains(axis: str = "x") -> dict[str, int]:
    _get_state(axis)
    return _backend.read_drive_gains(axis)


def scale_drive_gains(axis: str = "x", scale: float = 0.5) -> dict[str, dict[str, int]]:
    _get_state(axis)
    return _backend.scale_drive_gains(axis, float(scale))


def get_status_registers(axis: str = "x") -> dict[str, int]:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return {"SRL": int(_backend.read_status(axis)), "SRH": 0}
    return _backend.read_status_registers(axis)


def get_motor_velocity(axis: str = "x") -> float:
    _get_state(axis)
    return float(_backend.read_load_velocity_deg_s(axis))


def get_input_pos(axis: str = "x") -> float:
    return get_current_position(axis)


def get_control_mode(axis: str = "x") -> int:
    state = _get_state(axis)
    if state["pvt_mode_active"]:
        return 30
    if state["velocity_mode_active"]:
        return 2
    return 3


def get_input_mode(axis: str = "x") -> int:
    state = _get_state(axis)
    if state["pvt_mode_active"]:
        return 31
    if state["velocity_mode_active"]:
        return 1
    return 5


def get_load_encoder(axis: str = "x") -> int:
    _get_state(axis)
    return 22  # EnDat 2.2 load feedback placeholder for logs.


def get_axis_state(axis: str = "x") -> int:
    return int(_backend.read_status(axis))


def get_active_errors(axis: str = "x") -> int:
    return int(_backend.read_faults(axis))


def split_fault_registers(faults: int) -> tuple[int, int, int]:
    return int(faults) & 0xFFFF, (int(faults) >> 16) & 0xFFFF, (int(faults) >> 32) & 0xFFFF


def get_fault_registers(axis: str = "x") -> dict[str, int]:
    _get_state(axis)
    if isinstance(_backend, TMLLibBackend):
        return _backend.read_fault_registers(axis)
    mer, der, der2 = split_fault_registers(get_active_errors(axis))
    return {"MER": mer, "DER": der, "DER2": der2}


def get_decoded_fault_registers(axis: str = "x") -> dict[str, list[str]]:
    return {
        name: decode_register_bits(name, value)
        for name, value in get_fault_registers(axis).items()
    }


def drive_initialisation_diagnostic(axis: str = "x") -> dict:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        before = {"SRL": int(_backend.read_status(axis)), "SRH": 0, "MER": 0, "DER": 0, "DER2": 0}
        return {
            "ok": True,
            "error": None,
            "before": before,
            "after": before.copy(),
            "decoded_after": {name: decode_register_bits(name, value) for name, value in before.items()},
        }
    return _backend.drive_initialisation_diagnostic(axis)


def get_disarm_reason(axis: str = "x") -> int:
    return int(_backend.read_faults(axis))


def get_procedure_result(axis: str = "x") -> int:
    return 0


def clear_safety_trip(axis: str = "x") -> None:
    state = _get_state(axis)
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None


def clear_safety_trip_all() -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            clear_safety_trip(axis)


def get_safety_trip_reason(axis: str = "x") -> str | None:
    return _get_state(axis)["safety_trip_reason"]


def is_safety_tripped(axis: str = "x") -> bool:
    return bool(_get_state(axis)["safety_tripped"])


def _ensure_safe_to_command(axis: str = "x") -> dict:
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


def _trip_safety(axis: str, position_deg: float) -> None:
    state = _get_state(axis)
    if state["safety_tripped"]:
        return
    try:
        _backend.stop_axis(axis)
    except Exception:
        pass
    state["velocity_mode_active"] = False
    state["pvt_mode_active"] = False
    state["safety_tripped"] = True
    state["safety_trip_reason"] = (
        f"{axis.upper()} axis position safety trip: {position_deg:.3f} deg outside "
        f"[{POSITION_SAFETY_MIN_DEGREE:.1f}, {POSITION_SAFETY_MAX_DEGREE:.1f}]"
    )


def _position_safety_monitor_loop() -> None:
    while not _safety_monitor_stop_event.is_set():
        for axis, state in AXIS_STATE.items():
            if not state["connected"] or state["recovery_active"]:
                continue
            try:
                position_deg = get_current_position(axis)
            except Exception:
                continue
            if position_deg < POSITION_SAFETY_MIN_DEGREE or position_deg > POSITION_SAFETY_MAX_DEGREE:
                _trip_safety(axis, position_deg)
        _safety_monitor_stop_event.wait(POSITION_SAFETY_POLL_INTERVAL_SEC)


def _ensure_position_safety_monitor_running() -> None:
    global _safety_monitor_thread
    if _safety_monitor_thread is not None and _safety_monitor_thread.is_alive():
        return
    _safety_monitor_stop_event.clear()
    _safety_monitor_thread = threading.Thread(target=_position_safety_monitor_loop, daemon=True)
    _safety_monitor_thread.start()


def connect(axis: str = "x") -> object:
    _validate_axis(axis)
    cfg = AXIS_CONFIG[axis]
    device = _backend.connect_axis(axis, cfg)
    state = AXIS_STATE[axis]
    state["device"] = device
    state["connected"] = True
    state["velocity_mode_active"] = False
    state["pvt_mode_active"] = False
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None
    _backend.reset_faults(axis)
    _backend.set_profile_limits(
        axis,
        DEFAULT_PROFILE_VEL_DEG_S,
        DEFAULT_PROFILE_ACCEL_DEG_S2,
        DEFAULT_PROFILE_DECEL_DEG_S2,
    )
    _refresh_position(axis)
    _ensure_position_safety_monitor_running()
    return device


def connect_all() -> dict[str, object]:
    return {axis: connect(axis) for axis in AXIS_CONFIG}


def disconnect(axis: str = "x") -> None:
    _validate_axis(axis)
    try:
        _backend.stop_axis(axis)
    finally:
        _backend.disconnect_axis(axis)
        AXIS_STATE[axis]["device"] = None
        AXIS_STATE[axis]["connected"] = False


def reset_drive(axis: str = "x") -> None:
    """Reset the active drive so newly downloaded setup data can take effect."""
    _validate_axis(axis)
    _get_state(axis)
    _backend.reset_drive(axis)
    state = AXIS_STATE[axis]
    state["device"] = None
    state["connected"] = False
    state["velocity_mode_active"] = False
    state["pvt_mode_active"] = False
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None


def align_reference_to_actual(axis: str = "x") -> None:
    """Set the drive target/reference position equal to its measured position."""
    _validate_axis(axis)
    _get_state(axis)
    _backend.align_reference_to_actual(axis)


def align_reference_to_actual_report(axis: str = "x") -> dict[str, int]:
    """Align reference and return before/after position-reference variables."""
    _validate_axis(axis)
    _get_state(axis)
    before = {
        "APOS": int(_backend.read_motor_position_counts(axis)),
        "CPOS": int(_backend.read_command_position_counts(axis)),
        "TPOS": int(_backend.read_target_position_counts(axis)),
    }
    _backend.align_reference_to_actual(axis)
    time.sleep(0.02)
    after = {
        "APOS": int(_backend.read_motor_position_counts(axis)),
        "CPOS": int(_backend.read_command_position_counts(axis)),
        "TPOS": int(_backend.read_target_position_counts(axis)),
    }
    return {"before": before, "after": after}


def set_vertical_zero_here(axis: str = "x") -> float:
    """Define the current physical vertical position as application 0 deg."""
    _validate_axis(axis)
    state = _get_state(axis)
    drive_position_deg = _read_drive_position_deg(axis)
    state["home_offset_deg"] = drive_position_deg
    state["position_deg"] = 0.0
    state["position_timestamp"] = time.monotonic()
    return drive_position_deg


def get_vertical_zero_hardcode(axis: str = "x") -> dict[str, float | int | str]:
    """Return the source constant needed to make current measured APOS zero."""
    _validate_axis(axis)
    _get_state(axis)
    position_iu = int(_backend.read_motor_position_counts(axis))
    constant_name = f"{axis.upper()}_TECHNOSOFT_HOME_IU"
    return {
        "position_iu": position_iu,
        "constant_name": constant_name,
        "code_line": f"{constant_name} = {position_iu}",
    }


def get_home_offset_deg(axis: str = "x") -> float:
    _validate_axis(axis)
    return float(AXIS_STATE[axis]["home_offset_deg"])


def get_serial_number(axis: str = "x") -> str:
    _validate_axis(axis)
    return f"iPOS8020-BX-CAN node {AXIS_CONFIG[axis].node_id}"


def _clamp_manual_target(target_output_deg: float) -> float:
    return max(min(float(target_output_deg), MAX_DEGREE), MIN_DEGREE)


def wait_until_settled(target_output_deg: float, axis: str = "x") -> None:
    start_time = time.monotonic()
    while True:
        if time.monotonic() - start_time > SETTLE_TIMEOUT_SEC:
            raise TimeoutError(f"{axis.upper()} axis failed to settle within {SETTLE_TIMEOUT_SEC:.1f} s")
        active_errors = get_active_errors(axis)
        if active_errors:
            raise RuntimeError(f"{axis.upper()} axis stopped during settle (faults=0x{active_errors:X})")
        position_deg = get_current_position(axis)
        velocity_abs = abs(get_motor_velocity(axis))
        if abs(target_output_deg - position_deg) < POSITION_TOL_DEG and velocity_abs < VELOCITY_TOL_DEG_S:
            return
        time.sleep(0.01)


def set_traj_params(vel: float, acc: float, dec: float, axis: str = "x") -> None:
    _get_state(axis)
    _backend.set_profile_limits(axis, float(vel), float(acc), float(dec))


def set_gains(pos_gain: float, vel_gain: float, vel_i: float, axis: str = "x") -> None:
    _get_state(axis)
    # Gain mapping is drive-specific and will be bound once we import the TML setup.


def move_absolute(target_output_deg: float, axis: str = "x") -> None:
    _ensure_safe_to_command(axis)
    target_output_deg = _clamp_manual_target(target_output_deg)
    _backend.move_absolute_axis_deg(axis, _application_to_drive_deg(axis, target_output_deg))
    wait_until_settled(target_output_deg, axis=axis)


def go_home(axis: str = "x") -> None:
    move_absolute(0.0, axis=axis)


def move_relative(delta_deg: float, axis: str = "x") -> None:
    current_deg = get_current_position(axis=axis)
    if current_deg >= MAX_DEGREE - MANUAL_EDGE_GUARD_DEG and delta_deg > 0:
        raise RuntimeError(f"{axis.upper()} relative move blocked near +{MAX_DEGREE:.1f} deg")
    if current_deg <= MIN_DEGREE + MANUAL_EDGE_GUARD_DEG and delta_deg < 0:
        raise RuntimeError(f"{axis.upper()} relative move blocked near {MIN_DEGREE:.1f} deg")
    move_absolute(_clamp_manual_target(current_deg + delta_deg), axis=axis)


def recover_axis_to_safe_range(axis: str = "x") -> float:
    state = _get_state(axis)
    current_deg = get_current_position(axis)
    if current_deg > MAX_DEGREE:
        target_deg = MAX_DEGREE - RECOVERY_TARGET_MARGIN_DEG
    elif current_deg < MIN_DEGREE:
        target_deg = MIN_DEGREE + RECOVERY_TARGET_MARGIN_DEG
    else:
        clear_safety_trip(axis)
        return current_deg

    state["recovery_active"] = True
    state["safety_tripped"] = False
    state["safety_trip_reason"] = None
    try:
        _backend.move_absolute_axis_deg(axis, _application_to_drive_deg(axis, target_deg))
        wait_until_settled(target_deg, axis=axis)
        clear_safety_trip(axis)
        return target_deg
    finally:
        state["recovery_active"] = False


def enter_velocity_mode(axis: str = "x", input_filter_bandwidth=None, ramp_rate_deg_s2=None) -> None:
    _ensure_safe_to_command(axis)
    _backend.enter_velocity_mode(axis)
    AXIS_STATE[axis]["velocity_mode_active"] = True
    AXIS_STATE[axis]["pvt_mode_active"] = False


def command_velocity(target_output_vel_deg_s: float = 0.0, axis: str = "x") -> None:
    _ensure_safe_to_command(axis)
    _backend.move_velocity_axis_deg_s(axis, float(target_output_vel_deg_s))


def exit_velocity_mode(axis: str = "x") -> None:
    if not AXIS_STATE[axis]["connected"]:
        return
    _backend.exit_velocity_mode(axis)
    AXIS_STATE[axis]["velocity_mode_active"] = False


def pvt_setup(axis: str = "x", absolute: bool = True) -> None:
    _ensure_safe_to_command(axis)
    _backend.pvt_setup(axis, absolute=absolute)
    AXIS_STATE[axis]["pvt_mode_active"] = True
    AXIS_STATE[axis]["velocity_mode_active"] = False


def pvt_clear(axis: str = "x") -> None:
    _get_state(axis)
    _backend.pvt_clear(axis)


def pvt_send_first_point(point: PVTPoint, axis: str = "x") -> None:
    _ensure_safe_to_command(axis)
    _validate_pvt_point(axis, point)
    drive_point = PVTPoint(
        position_deg=_application_to_drive_deg(axis, point.position_deg),
        velocity_deg_s=point.velocity_deg_s,
        duration_s=point.duration_s,
    )
    _backend.pvt_send_first_point(axis, drive_point)


def pvt_send_point(point: PVTPoint, axis: str = "x") -> None:
    _ensure_safe_to_command(axis)
    _validate_pvt_point(axis, point)
    drive_point = PVTPoint(
        position_deg=_application_to_drive_deg(axis, point.position_deg),
        velocity_deg_s=point.velocity_deg_s,
        duration_s=point.duration_s,
    )
    _backend.pvt_send_point(axis, drive_point)


def pvt_start(axis: str = "x") -> None:
    _ensure_safe_to_command(axis)
    _backend.pvt_start(axis)


def pvt_buffer_level(axis: str = "x") -> int:
    _get_state(axis)
    return int(_backend.pvt_buffer_level(axis))


def queue_pvt_points(points: Iterable[PVTPoint], axis: str = "x", start: bool = False) -> int:
    points = list(points)
    if not points:
        return 0
    pvt_setup(axis=axis, absolute=True)
    pvt_send_first_point(points[0], axis=axis)
    for point in points[1:]:
        pvt_send_point(point, axis=axis)
    if start:
        pvt_start(axis)
    return len(points)


def _validate_pvt_point(axis: str, point: PVTPoint) -> None:
    if point.duration_s <= 0.0:
        raise ValueError("PVT point duration must be positive")
    if point.position_deg < TRACKING_MIN_DEGREE or point.position_deg > TRACKING_MAX_DEGREE:
        raise ValueError(
            f"{axis.upper()} PVT point {point.position_deg:.3f} deg outside "
            f"[{TRACKING_MIN_DEGREE:.1f}, {TRACKING_MAX_DEGREE:.1f}]"
        )


def stop_axis(axis: str = "x") -> None:
    _get_state(axis)
    _backend.stop_axis(axis)
    AXIS_STATE[axis]["velocity_mode_active"] = False
    AXIS_STATE[axis]["pvt_mode_active"] = False


def power_axis(axis: str = "x", enable: bool = True) -> None:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return
    _backend.power_axis(axis, enable=enable)


def axis_on_command(axis: str = "x", enable: bool = True) -> None:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return
    _backend.axis_on_command(axis, enable=enable)


def axis_on_update_command(axis: str = "x", enable: bool = True) -> None:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return
    _backend.axis_on_update_command(axis, enable=enable)


def execute_tml(axis: str = "x", command: str = "") -> None:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return
    _backend.execute_tml(axis, command)


def end_initialization(axis: str = "x") -> None:
    _get_state(axis)
    if not isinstance(_backend, TMLLibBackend):
        return
    _backend.end_initialization(axis)


def power_all(enable: bool = True) -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            power_axis(axis, enable=enable)


def disarm_axis(axis: str = "x") -> None:
    stop_axis(axis)


def stop_all() -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            stop_axis(axis)


def disarm_all() -> None:
    stop_all()


def _run_parallel(calls):
    threads = []
    errors = []

    def runner(func, kwargs):
        try:
            func(**kwargs)
        except Exception as exc:
            errors.append(exc)

    for func, kwargs in calls:
        thread = threading.Thread(target=runner, args=(func, kwargs), daemon=True)
        thread.start()
        threads.append(thread)
    for thread in threads:
        thread.join()
    if errors:
        raise errors[0]


def move_absolute_pair(x_deg=None, y_deg=None) -> None:
    calls = []
    if x_deg is not None:
        calls.append((move_absolute, {"target_output_deg": x_deg, "axis": "x"}))
    if y_deg is not None:
        calls.append((move_absolute, {"target_output_deg": y_deg, "axis": "y"}))
    _run_parallel(calls)


def command_velocity_pair(x_vel_deg_s: float = 0.0, y_vel_deg_s: float = 0.0) -> None:
    command_velocity(x_vel_deg_s, axis="x")
    command_velocity(y_vel_deg_s, axis="y")


def move_relative_pair(x_delta=None, y_delta=None) -> None:
    calls = []
    if x_delta is not None:
        calls.append((move_relative, {"delta_deg": x_delta, "axis": "x"}))
    if y_delta is not None:
        calls.append((move_relative, {"delta_deg": y_delta, "axis": "y"}))
    _run_parallel(calls)


def set_gains_all(pos_gain: float, vel_gain: float, vel_i: float) -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            set_gains(pos_gain, vel_gain, vel_i, axis=axis)


def set_traj_params_all(vel: float, acc: float, dec: float) -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            set_traj_params(vel, acc, dec, axis=axis)


def enter_velocity_mode_all(input_filter_bandwidth=None, ramp_rate_deg_s2=None) -> None:
    for axis in AXIS_CONFIG:
        enter_velocity_mode(axis=axis, input_filter_bandwidth=input_filter_bandwidth, ramp_rate_deg_s2=ramp_rate_deg_s2)


def exit_velocity_mode_all() -> None:
    for axis in AXIS_CONFIG:
        if AXIS_STATE[axis]["connected"]:
            exit_velocity_mode(axis=axis)
