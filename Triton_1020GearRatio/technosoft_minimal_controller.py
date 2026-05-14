"""Small Python port of Technosoft TML_LIB Ex04_BasicMove.

Keep this file boring on purpose.  The startup and motion sequence follows the
vendor example:

    TS_OpenChannel
    TS_LoadSetup
    TS_SetupAxis
    TS_SelectAxis
    TS_DriveInitialisation
    TS_Power(POWER_ON)
    TS_MoveRelative / TS_MoveAbsolute

Project-specific additions are limited to PCAN settings, setup path, node ID,
and degree-to-load-internal-unit conversion.
"""

from __future__ import annotations

import ctypes
import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


TML_LIB_DLL = Path(r"C:\Program Files (x86)\Technosoft\TML_LIB_x64\lib\TML_lib.dll")
DEFAULT_SETUP_PATH = Path(r"C:\TML_Setups\ipos8020_dual_loop_master.t.zip")

CHANNEL_PEAK_SYS_PCAN_USB = 7
DEFAULT_CHANNEL_NAME = "1"
DEFAULT_HOST_ID = 255
DEFAULT_BAUDRATE = 1_000_000
PVT_MAX_TIME_IU = 511
PVT_MIN_FALLBACK_TIME_MS = 200
PVT_MAX_PRELOAD_POINTS = 8
PVT_STREAM_FEED_MARGIN_POINTS = 7
PVT_TARGET_TOLERANCE_DEG = 0.25
PVT_STATUS_ADDRESS = 0x0863
SRH_SRL_ADDRESS = 0x090E
MER_ADDRESS = 0x08FC
PVT_LOG_DIR = Path(__file__).resolve().parent / "tracking_logs" / "technosoft_pvt_logs"

# Hard-code these after placing each axis at vertical zero and pressing
# "Set Vertical Zero" in the GUI.
X_VERTICAL_ZERO_IU = 519296
Y_VERTICAL_ZERO_IU = 0

POWER_OFF = 0
POWER_ON = 1
UPDATE_NONE = -1
UPDATE_IMMEDIATE = 1
FROM_MEASURE = 0
FROM_REFERENCE = 1
ABSOLUTE_POSITION = 0
NO_ADDITIVE = 0
WAIT_EVENT = 1
NO_STOP = 0

# Selector indexes from Technosoft TML_lib.h. These are not memory addresses.
REG_SRL = 3
REG_SRH = 4
REG_MER = 5
REG_DER = 6
REG_DER2 = 7

GAIN_NAMES = ("KPP", "KIP", "KDP", "KPS", "KIS")

VERTICAL_ZERO_IU = {
    "x": X_VERTICAL_ZERO_IU,
    "y": Y_VERTICAL_ZERO_IU,
}


@dataclass
class AxisConfig:
    axis: str
    node_id: int
    setup_path: Path
    output_sign: float = 1.0


@dataclass
class AxisInfo:
    config: AxisConfig
    setup_index: int
    motor_scale: float
    load_scale: float
    time_scale_ms: float
    speed_iu: float = 30.0
    accel_iu: float = 0.6
    decel_iu: float = 0.6
    speed_deg_s: float = 1.0
    accel_deg_s2: float = 2.0
    decel_deg_s2: float = 2.0


class TechnosoftEx04:
    """Minimal single-process TML_LIB wrapper following the vendor examples."""

    def __init__(self, dll_path: Path = TML_LIB_DLL) -> None:
        self.dll_path = Path(dll_path)
        self.dll = None
        self._dll_cookie = None
        self.channel_fd: int | None = None
        self.axes: dict[str, AxisInfo] = {}
        self._pvt_callback_type = None
        self._pvt_callback = None
        self._pvt_status_by_node: dict[int, dict[str, int]] = {}

    def load_dll(self) -> None:
        if self.dll is not None:
            return
        if not self.dll_path.exists():
            raise FileNotFoundError(f"TML_LIB DLL not found: {self.dll_path}")
        if hasattr(os, "add_dll_directory"):
            self._dll_cookie = os.add_dll_directory(str(self.dll_path.parent))
        self.dll = ctypes.WinDLL(str(self.dll_path))
        self._bind()

    def _bind(self) -> None:
        d = self.dll
        d.TS_Basic_GetLastErrorText.argtypes = [ctypes.c_char_p, ctypes.c_int32]
        d.TS_Basic_GetLastErrorText.restype = None
        d.TS_OpenChannel.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_uint32]
        d.TS_OpenChannel.restype = ctypes.c_int32
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
        d.TS_Power.argtypes = [ctypes.c_int32]
        d.TS_Power.restype = ctypes.c_int32
        d.TS_Stop.argtypes = []
        d.TS_Stop.restype = ctypes.c_int32
        d.TS_ResetFault.argtypes = []
        d.TS_ResetFault.restype = ctypes.c_int32
        d.TS_ReadStatus.argtypes = [ctypes.c_int16, ctypes.POINTER(ctypes.c_uint16)]
        d.TS_ReadStatus.restype = ctypes.c_int32
        d.TS_GetLongVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int32)]
        d.TS_GetLongVariable.restype = ctypes.c_int32
        d.TS_GetIntVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int16)]
        d.TS_GetIntVariable.restype = ctypes.c_int32
        d.TS_SetIntVariable.argtypes = [ctypes.c_char_p, ctypes.c_int16]
        d.TS_SetIntVariable.restype = ctypes.c_int32
        d.TS_GetFixedVariable.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)]
        d.TS_GetFixedVariable.restype = ctypes.c_int32
        d.TS_MoveRelative.argtypes = [
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_MoveRelative.restype = ctypes.c_int32
        d.TS_MoveAbsolute.argtypes = [
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_MoveAbsolute.restype = ctypes.c_int32
        d.TS_UpdateImmediate.argtypes = []
        d.TS_UpdateImmediate.restype = ctypes.c_int32
        d.TS_GOTO.argtypes = [ctypes.c_uint16]
        d.TS_GOTO.restype = ctypes.c_int32
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
            ctypes.c_uint16,
            ctypes.c_int16,
            ctypes.c_int16,
            ctypes.c_int32,
            ctypes.c_int16,
            ctypes.c_int16,
        ]
        d.TS_SendPVTFirstPoint.restype = ctypes.c_int32
        d.TS_SendPVTPoint.argtypes = [
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_uint16,
            ctypes.c_int16,
        ]
        d.TS_SendPVTPoint.restype = ctypes.c_int32
        d.TS_SetEventOnMotionComplete.argtypes = [ctypes.c_int32, ctypes.c_int32]
        d.TS_SetEventOnMotionComplete.restype = ctypes.c_int32
        d.TS_SetTargetPositionToActual.argtypes = []
        d.TS_SetTargetPositionToActual.restype = ctypes.c_int32
        d.TS_GetMotorPositionScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetMotorPositionScalingFactor.restype = ctypes.c_int32
        d.TS_GetLoadPositionScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetLoadPositionScalingFactor.restype = ctypes.c_int32
        d.TS_GetTimeScalingFactor.argtypes = [ctypes.POINTER(ctypes.c_double)]
        d.TS_GetTimeScalingFactor.restype = ctypes.c_int32
        d.TS_RegisterHandlerForUnrequestedDriveMessages.argtypes = [ctypes.c_void_p]
        d.TS_RegisterHandlerForUnrequestedDriveMessages.restype = None
        d.TS_CheckForUnrequestedDriveMessages.argtypes = []
        d.TS_CheckForUnrequestedDriveMessages.restype = ctypes.c_int32
        d.TS_SendDataToHost.argtypes = [ctypes.c_ubyte, ctypes.c_uint32, ctypes.c_uint16]
        d.TS_SendDataToHost.restype = ctypes.c_int32

    def last_error(self) -> str:
        if self.dll is None:
            return "TML_LIB not loaded"
        buffer = ctypes.create_string_buffer(1024)
        self.dll.TS_Basic_GetLastErrorText(buffer, len(buffer))
        return buffer.value.decode("mbcs", errors="replace").strip() or "TML_LIB call failed"

    def check(self, ok: int, action: str) -> None:
        if not ok:
            raise RuntimeError(f"{action} failed: {self.last_error()}")

    def _register_pvt_handler(self) -> None:
        if self._pvt_callback is not None:
            return

        callback_type = ctypes.WINFUNCTYPE(None, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_int32)

        def handle_message(node_id: int, address: int, value: int) -> None:
            status = self._pvt_status_by_node.setdefault(
                int(node_id),
                {
                    "buffer_full": 0,
                    "buffer_empty": 0,
                    "buffer_initialise": 0,
                    "counter_dsp": 0,
                    "raw_messages": 0,
                    "messages": 0,
                    "last_axis": 0,
                    "last_address": 0,
                    "last_value": 0,
                },
            )
            status["raw_messages"] += 1
            status["last_axis"] = int(node_id)
            status["last_address"] = int(address)
            status["last_value"] = int(value)
            if int(address) != PVT_STATUS_ADDRESS:
                return
            status["messages"] += 1
            status["buffer_full"] = 1 if value & (1 << 13) else 0
            if value & (1 << 15):
                status["buffer_empty"] = 1
            if status["buffer_full"]:
                status["counter_dsp"] = value & 0x3F
            if not status["buffer_initialise"] and status["buffer_full"]:
                status["buffer_initialise"] = 1
                status["buffer_empty"] = 0

        self._pvt_callback_type = callback_type
        self._pvt_callback = callback_type(handle_message)
        self.dll.TS_RegisterHandlerForUnrequestedDriveMessages.argtypes = [callback_type]
        self.dll.TS_RegisterHandlerForUnrequestedDriveMessages(self._pvt_callback)

    def _reset_pvt_status(self, node_id: int) -> dict[str, int]:
        status = {
            "buffer_full": 0,
            "buffer_empty": 0,
            "buffer_initialise": 0,
            "counter_dsp": 0,
            "raw_messages": 0,
            "messages": 0,
            "last_axis": 0,
            "last_address": 0,
            "last_value": 0,
        }
        self._pvt_status_by_node[int(node_id)] = status
        return status

    def enable_basic_unrequested_messages(self, axis: str) -> dict[str, int]:
        """Ask the drive to send SRH/SRL and MER unrequested messages.

        This is a diagnostic based on Technosoft Ex02. It lets us verify whether
        callbacks work at all on the current channel before blaming PVT status.
        """
        info = self.select_axis(axis)
        self._register_pvt_handler()
        status = self._pvt_status_by_node.setdefault(info.config.node_id, {})
        status.update({"raw_messages": 0, "last_axis": 0, "last_address": 0, "last_value": 0})
        self.check(
            self.dll.TS_SendDataToHost(DEFAULT_HOST_ID, 0x00000400, 0xFFFF),
            "TS_SendDataToHost",
        )
        return {"node_id": info.config.node_id, "host_id": DEFAULT_HOST_ID}

    def open_channel(
        self,
        channel_name: str = DEFAULT_CHANNEL_NAME,
        channel_type: int = CHANNEL_PEAK_SYS_PCAN_USB,
        host_id: int = DEFAULT_HOST_ID,
        baudrate: int = DEFAULT_BAUDRATE,
    ) -> int:
        """Vendor InitCommunicationChannel(), with project CAN settings."""
        self.load_dll()
        if self.channel_fd is not None:
            return self.channel_fd
        fd = self.dll.TS_OpenChannel(
            str(channel_name).encode("ascii"),
            int(channel_type),
            int(host_id),
            int(baudrate),
        )
        if fd < 0:
            raise RuntimeError(f"TS_OpenChannel failed: {self.last_error()}")
        self.channel_fd = int(fd)
        return self.channel_fd

    def setup_axis_only(self, config: AxisConfig) -> AxisInfo:
        """Vendor InitAxis() up to TS_SelectAxis, without ENDINIT/AXISON."""
        self.open_channel()
        setup_path = Path(config.setup_path)
        if not setup_path.exists():
            raise FileNotFoundError(f"Setup file not found: {setup_path}")
        idx_setup = self.dll.TS_LoadSetup(str(setup_path).encode("mbcs"))
        if idx_setup < 0:
            raise RuntimeError(f"TS_LoadSetup failed: {self.last_error()}")
        self.check(self.dll.TS_SetupAxis(int(config.node_id), int(idx_setup)), f"TS_SetupAxis({config.node_id})")
        self.check(self.dll.TS_SelectAxis(int(config.node_id)), f"TS_SelectAxis({config.node_id})")
        info = AxisInfo(
            config=config,
            setup_index=int(idx_setup),
            motor_scale=self._read_scale("TS_GetMotorPositionScalingFactor"),
            load_scale=self._read_scale("TS_GetLoadPositionScalingFactor"),
            time_scale_ms=self._read_scale("TS_GetTimeScalingFactor"),
        )
        self.axes[config.axis.lower()] = info
        return info

    def vendor_init_axis(self, config: AxisConfig) -> AxisInfo:
        """Vendor InitAxis(): LoadSetup, SetupAxis, SelectAxis, ENDINIT, AXISON.

        Technosoft's C example then polls SRL.15 until the power stage is on.
        On this CAN setup TS_ReadStatus can time out even after TS_Power()
        returns OK, so the GUI keeps that status read as a separate diagnostic
        instead of making initialisation fail solely on the poll.
        """
        info = self.setup_axis_only(config)
        self.check(self.dll.TS_DriveInitialisation(), "TS_DriveInitialisation")
        self.check(self.dll.TS_Power(POWER_ON), "TS_Power(POWER_ON)")
        return info

    def select_axis(self, axis: str) -> AxisInfo:
        info = self.axes[axis.lower()]
        self.check(self.dll.TS_SelectAxis(int(info.config.node_id)), f"TS_SelectAxis({info.config.node_id})")
        return info

    def _read_scale(self, function_name: str) -> float:
        value = ctypes.c_double()
        self.check(getattr(self.dll, function_name)(ctypes.byref(value)), function_name)
        return float(value.value)

    def read_status(self, axis: str) -> dict[str, int]:
        self.select_axis(axis)
        result = {}
        for name, selector in (("SRL", REG_SRL), ("SRH", REG_SRH), ("MER", REG_MER), ("DER", REG_DER), ("DER2", REG_DER2)):
            value = ctypes.c_uint16()
            self.check(self.dll.TS_ReadStatus(selector, ctypes.byref(value)), f"TS_ReadStatus({name})")
            result[name] = int(value.value)
        return result

    def wait_power_on(self, axis: str, max_reads: int = 200) -> None:
        for _ in range(max_reads):
            srl = self.read_status(axis)["SRL"]
            if srl & (1 << 15):
                return
        raise RuntimeError("Power stage did not report AXISON in SRL.15")

    def _read_long(self, name: str) -> int:
        value = ctypes.c_int32()
        self.check(self.dll.TS_GetLongVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetLongVariable({name})")
        return int(value.value)

    def _read_int(self, name: str) -> int:
        value = ctypes.c_int16()
        self.check(self.dll.TS_GetIntVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetIntVariable({name})")
        return int(value.value)

    def _write_int(self, name: str, value: int) -> None:
        clamped = max(min(int(value), 32767), -32768)
        self.check(
            self.dll.TS_SetIntVariable(name.encode("ascii"), ctypes.c_int16(clamped)),
            f"TS_SetIntVariable({name})",
        )

    def read_positions(self, axis: str) -> dict[str, float | int]:
        self.select_axis(axis)
        apos = self._read_long("APOS")
        cpos = self._read_long("CPOS")
        tpos = self._read_long("TPOS")
        zero = int(VERTICAL_ZERO_IU[axis.lower()])
        return {
            "APOS": apos,
            "CPOS": cpos,
            "TPOS": tpos,
            "zero_iu": zero,
            "APOS_deg": self.iu_to_deg(axis, apos - zero),
            "CPOS_deg": self.iu_to_deg(axis, cpos - zero),
            "TPOS_deg": self.iu_to_deg(axis, tpos - zero),
        }

    def read_gains(self, axis: str) -> dict[str, int]:
        self.select_axis(axis)
        return {name: self._read_int(name) for name in GAIN_NAMES}

    def write_gains(self, axis: str, gains: dict[str, int]) -> dict[str, int]:
        self.select_axis(axis)
        for name in GAIN_NAMES:
            if name in gains:
                self._write_int(name, gains[name])
        return self.read_gains(axis)

    def set_motion_params_iu(self, axis: str, speed_iu: float, accel_iu: float, decel_iu: float | None = None) -> None:
        info = self.select_axis(axis)
        info.speed_iu = float(speed_iu)
        info.accel_iu = float(accel_iu)
        info.decel_iu = float(accel_iu if decel_iu is None else decel_iu)

    def set_motion_params_deg(
        self,
        axis: str,
        speed_deg_s: float,
        accel_deg_s2: float,
        decel_deg_s2: float | None = None,
    ) -> None:
        info = self.select_axis(axis)
        info.speed_deg_s = float(speed_deg_s)
        info.accel_deg_s2 = float(accel_deg_s2)
        info.decel_deg_s2 = float(accel_deg_s2 if decel_deg_s2 is None else decel_deg_s2)
        info.speed_iu = self.speed_deg_s_to_iu(axis, info.speed_deg_s)
        info.accel_iu = self.accel_deg_s2_to_iu(axis, info.accel_deg_s2)
        info.decel_iu = self.accel_deg_s2_to_iu(axis, info.decel_deg_s2)

    def deg_to_iu(self, axis: str, deg: float) -> int:
        info = self.axes[axis.lower()]
        return int(round(info.config.output_sign * float(deg) / 360.0 * info.load_scale))

    def absolute_deg_to_iu(self, axis: str, deg: float) -> int:
        return self.deg_to_iu(axis, deg) + int(VERTICAL_ZERO_IU[axis.lower()])

    def iu_to_deg(self, axis: str, iu: int | float) -> float:
        info = self.axes[axis.lower()]
        if info.load_scale == 0:
            return 0.0
        return info.config.output_sign * float(iu) / info.load_scale * 360.0

    def speed_deg_s_to_iu(self, axis: str, speed_deg_s: float) -> float:
        info = self.axes[axis.lower()]
        dt = info.time_scale_ms / 1000.0
        output_rev_s = abs(float(speed_deg_s)) / 360.0
        return output_rev_s * info.load_scale * dt

    def accel_deg_s2_to_iu(self, axis: str, accel_deg_s2: float) -> float:
        info = self.axes[axis.lower()]
        dt = info.time_scale_ms / 1000.0
        output_rev_s2 = abs(float(accel_deg_s2)) / 360.0
        return output_rev_s2 * info.load_scale * dt * dt

    def velocity_deg_s_to_iu(self, axis: str, velocity_deg_s: float) -> float:
        info = self.axes[axis.lower()]
        dt = info.time_scale_ms / 1000.0
        output_rev_s = info.config.output_sign * float(velocity_deg_s) / 360.0
        return output_rev_s * info.load_scale * dt

    def velocity_iu_to_deg_s(self, axis: str, velocity_iu: float) -> float:
        info = self.axes[axis.lower()]
        dt = info.time_scale_ms / 1000.0
        if info.load_scale == 0 or dt == 0 or info.config.output_sign == 0:
            return 0.0
        return float(velocity_iu) / (info.load_scale * dt) * 360.0 / info.config.output_sign

    def _build_pvt_line_points(
        self,
        axis: str,
        start_iu: int,
        target_iu: int,
        duration_s: float,
        point_count: int,
        profile: str,
    ) -> list[tuple[int, float]]:
        profile = profile.lower()
        if profile not in {"linear", "smoothstep"}:
            raise ValueError("PVT profile must be 'linear' or 'smoothstep'")
        delta_iu = target_iu - start_iu
        points = []
        for i in range(1, point_count + 1):
            u = i / point_count
            if profile == "smoothstep":
                pos_scale = 3.0 * u * u - 2.0 * u * u * u
                vel_scale = 6.0 * u * (1.0 - u)
            else:
                pos_scale = u
                vel_scale = 1.0
            pos_iu = int(round(start_iu + delta_iu * pos_scale))
            vel_iu_per_s = delta_iu * vel_scale / duration_s
            velocity_deg_s = self.iu_to_deg(axis, vel_iu_per_s)
            points.append((pos_iu, self.velocity_deg_s_to_iu(axis, velocity_deg_s)))
        return points

    def move_relative_deg(self, axis: str, delta_deg: float) -> None:
        """Vendor Ex04_Trapezoidal TS_MoveRelative using project degree conversion."""
        info = self.select_axis(axis)
        self.check(
            self.dll.TS_MoveRelative(
                self.deg_to_iu(axis, delta_deg),
                info.speed_iu,
                info.accel_iu,
                NO_ADDITIVE,
                UPDATE_IMMEDIATE,
                FROM_REFERENCE,
            ),
            "TS_MoveRelative",
        )

    def move_absolute_deg(self, axis: str, target_deg: float) -> None:
        info = self.select_axis(axis)
        target_iu = self.absolute_deg_to_iu(axis, target_deg)
        self.check(
            self.dll.TS_MoveAbsolute(
                target_iu,
                info.speed_iu,
                info.accel_iu,
                UPDATE_IMMEDIATE,
                FROM_REFERENCE,
            ),
            "TS_MoveAbsolute",
        )

    def run_pvt_line_deg(self, axis: str, target_deg: float, duration_s: float, point_count: int) -> dict[str, float | int | list]:
        """Send a short absolute PVT line move.

        This deliberately preloads a small point set instead of implementing the
        full Ex08 streaming callback. It is meant to prove that the drive accepts
        PVT points before we build the continuous satellite-tracking streamer.
        """
        if duration_s <= 0:
            raise ValueError("PVT duration must be positive")
        if point_count < 2:
            raise ValueError("PVT point count must be at least 2")
        if point_count > PVT_MAX_PRELOAD_POINTS:
            raise ValueError(
                f"This non-streaming PVT proof test can preload at most {PVT_MAX_PRELOAD_POINTS} points. "
                "Longer trajectories need the streaming PVT buffer handshake."
            )

        info = self.select_axis(axis)
        self.check(self.dll.TS_Stop(), "TS_Stop")
        self.check(self.dll.TS_SetTargetPositionToActual(), "TS_SetTargetPositionToActual")
        start_pos = self.read_positions(axis)
        start_iu = int(start_pos["TPOS"])
        start_deg = self.iu_to_deg(axis, start_iu - int(VERTICAL_ZERO_IU[axis.lower()]))
        target_iu = self.absolute_deg_to_iu(axis, target_deg)
        segment_count = point_count
        segment_time_ms = duration_s * 1000.0 / segment_count
        segment_time_iu = int(round(segment_time_ms / info.time_scale_ms))
        if segment_time_iu <= 0 or segment_time_iu > PVT_MAX_TIME_IU:
            min_points = int(duration_s * 1000.0 / (PVT_MAX_TIME_IU * info.time_scale_ms)) + 1
            raise ValueError(
                f"PVT segment time must be 1..{PVT_MAX_TIME_IU}; got {segment_time_iu}. "
                f"Increase Point count to at least {min_points} for {duration_s:g} s."
            )

        velocity_deg_s = (target_deg - start_deg) / duration_s
        velocity_iu = self.velocity_deg_s_to_iu(axis, velocity_deg_s)
        points = [
            int(round(start_iu + (target_iu - start_iu) * i / segment_count))
            for i in range(1, segment_count + 1)
        ]

        self.check(self.dll.TS_GOTO(0x4000), "TS_GOTO(0x4000)")
        self.check(self.dll.TS_PVTSetup(1, 1, 1, 1, 1, 1, 1), "TS_PVTSetup")
        self.check(
            self.dll.TS_SendPVTFirstPoint(
                points[0],
                velocity_iu,
                segment_time_iu,
                0,
                ABSOLUTE_POSITION,
                0,
                UPDATE_NONE,
                FROM_REFERENCE,
            ),
            "TS_SendPVTFirstPoint",
        )
        for counter, point in enumerate(points[1:], start=1):
            self.check(
                self.dll.TS_SendPVTPoint(point, velocity_iu, segment_time_iu, counter & 0x3F),
                "TS_SendPVTPoint",
            )
        self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate")
        trace = []
        t0 = time.monotonic()
        next_sample = t0
        while True:
            now = time.monotonic()
            if now >= next_sample:
                pos = self.read_positions(axis)
                trace.append(
                    {
                        "t_s": round(now - t0, 3),
                        "APOS_deg": round(float(pos["APOS_deg"]), 6),
                        "CPOS_deg": round(float(pos["CPOS_deg"]), 6),
                        "TPOS_deg": round(float(pos["TPOS_deg"]), 6),
                    }
                )
                next_sample += 0.25
            if now - t0 >= duration_s + 0.5:
                break
            time.sleep(0.01)
        end_pos = self.read_positions(axis)
        return {
            "start_deg": start_deg,
            "target_deg": float(target_deg),
            "delta_deg": float(target_deg - start_deg),
            "duration_s": float(duration_s),
            "point_count": int(point_count),
            "segment_time_iu": segment_time_iu,
            "velocity_iu": velocity_iu,
            "start_APOS": int(start_pos["APOS"]),
            "start_TPOS": int(start_pos["TPOS"]),
            "target_iu": int(target_iu),
            "end_APOS_deg": float(end_pos["APOS_deg"]),
            "end_TPOS_deg": float(end_pos["TPOS_deg"]),
            "trace_first": trace[:3],
            "trace_last": trace[-3:],
        }

    def run_streaming_pvt_line_deg(
        self,
        axis: str,
        target_deg: float,
        duration_s: float,
        point_count: int,
        profile: str = "smoothstep",
    ) -> dict[str, float | int | str]:
        """Stream a generated line trajectory using the Ex08 PVT buffer handshake."""
        info, status, start_pos = self._prepare_pvt_stream(axis)
        start_iu = int(start_pos["TPOS"])
        start_deg = self.iu_to_deg(axis, start_iu - int(VERTICAL_ZERO_IU[axis.lower()]))
        target_iu = self.absolute_deg_to_iu(axis, target_deg)
        segment_time_iu, _segment_s = self._pvt_segment_timing(info, duration_s, point_count)
        points = self._build_pvt_line_points(axis, start_iu, target_iu, duration_s, point_count, profile)
        metadata: dict[str, float | int | str] = {
            "trajectory": "line",
            "start_deg": start_deg,
            "target_deg": float(target_deg),
            "delta_deg": float(target_deg - start_deg),
            "duration_s": float(duration_s),
            "point_count": int(point_count),
            "profile": profile,
            "target_iu": int(target_iu),
        }
        return self._stream_pvt_points(
            axis,
            info,
            status,
            points,
            duration_s,
            segment_time_iu,
            final_target_deg=float(target_deg),
            metadata=metadata,
        )

    def run_streaming_pvt_sine_deg(
        self,
        axis: str,
        center_deg: float,
        amplitude_deg: float,
        period_s: float,
        duration_s: float,
        point_count: int,
    ) -> dict[str, float | int | str]:
        """Stream a sine trajectory using absolute PVT points.

        The sine phase is chosen from the current TPOS, so the first generated
        trajectory is continuous with the present axis position instead of
        jumping to a fixed phase. This is the same seam that later satellite
        position/velocity points should feed.
        """
        if amplitude_deg <= 0:
            raise ValueError("Sine amplitude must be positive")
        if period_s <= 0:
            raise ValueError("Sine period must be positive")

        info, status, start_pos = self._prepare_pvt_stream(axis)
        start_iu = int(start_pos["TPOS"])
        start_deg = self.iu_to_deg(axis, start_iu - int(VERTICAL_ZERO_IU[axis.lower()]))
        offset_ratio = (float(start_deg) - float(center_deg)) / float(amplitude_deg)
        if abs(offset_ratio) > 1.0:
            raise ValueError(
                f"Current position {start_deg:.3f} deg is outside sine range "
                f"[{center_deg - amplitude_deg:.3f}, {center_deg + amplitude_deg:.3f}] deg. "
                "Move inside the range first to avoid an initial PVT jump."
            )

        segment_time_iu, segment_s = self._pvt_segment_timing(info, duration_s, point_count)
        phase0 = math.asin(max(-1.0, min(1.0, offset_ratio)))
        omega = 2.0 * math.pi / float(period_s)
        points: list[tuple[int, float]] = []
        for i in range(1, point_count + 1):
            t_s = i * segment_s
            phase = phase0 + omega * t_s
            pos_deg = float(center_deg) + float(amplitude_deg) * math.sin(phase)
            vel_deg_s = float(amplitude_deg) * omega * math.cos(phase)
            points.append((self.absolute_deg_to_iu(axis, pos_deg), self.velocity_deg_s_to_iu(axis, vel_deg_s)))

        final_target_deg = self.iu_to_deg(axis, points[-1][0] - int(VERTICAL_ZERO_IU[axis.lower()]))
        metadata = {
            "trajectory": "sine",
            "start_deg": start_deg,
            "center_deg": float(center_deg),
            "amplitude_deg": float(amplitude_deg),
            "period_s": float(period_s),
            "duration_s": float(duration_s),
            "point_count": int(point_count),
            "profile": "sine",
            "final_target_deg": float(final_target_deg),
        }
        return self._stream_pvt_points(
            axis,
            info,
            status,
            points,
            duration_s,
            segment_time_iu,
            final_target_deg=float(final_target_deg),
            metadata=metadata,
        )

    def _prepare_pvt_stream(self, axis: str) -> tuple[AxisInfo, dict[str, int], dict[str, int | float]]:
        info = self.select_axis(axis)
        self._register_pvt_handler()
        status = self._reset_pvt_status(info.config.node_id)
        self.check(
            self.dll.TS_SendDataToHost(DEFAULT_HOST_ID, 0x00000400, 0xFFFF),
            "TS_SendDataToHost",
        )
        self.check(self.dll.TS_Stop(), "TS_Stop")
        self.check(self.dll.TS_SetTargetPositionToActual(), "TS_SetTargetPositionToActual")
        return info, status, self.read_positions(axis)

    def _pvt_segment_timing(self, info: AxisInfo, duration_s: float, point_count: int) -> tuple[int, float]:
        if duration_s <= 0:
            raise ValueError("PVT duration must be positive")
        if point_count < 4:
            raise ValueError("Streaming PVT point count must be at least 4")
        segment_time_ms = duration_s * 1000.0 / point_count
        segment_time_iu = int(round(segment_time_ms / info.time_scale_ms))
        if segment_time_iu <= 0 or segment_time_iu > PVT_MAX_TIME_IU:
            min_points = int(duration_s * 1000.0 / (PVT_MAX_TIME_IU * info.time_scale_ms)) + 1
            raise ValueError(
                f"PVT segment time must be 1..{PVT_MAX_TIME_IU}; got {segment_time_iu}. "
                f"Increase Point count to at least {min_points} for {duration_s:g} s."
            )
        if segment_time_ms < PVT_MIN_FALLBACK_TIME_MS:
            max_points = max(1, int(duration_s * 1000.0 / PVT_MIN_FALLBACK_TIME_MS))
            raise ValueError(
                f"Fallback streaming is not stable below {PVT_MIN_FALLBACK_TIME_MS:g} ms per point; "
                f"got {segment_time_ms:g} ms. Reduce Point count to {max_points} or less for {duration_s:g} s."
            )
        return segment_time_iu, segment_time_iu * info.time_scale_ms / 1000.0

    def _stream_pvt_points(
        self,
        axis: str,
        info: AxisInfo,
        status: dict[str, int],
        points: list[tuple[int, float]],
        duration_s: float,
        segment_time_iu: int,
        final_target_deg: float,
        metadata: dict[str, float | int | str],
    ) -> dict[str, float | int | str]:
        if not points:
            raise ValueError("PVT point list is empty")
        segment_s = segment_time_iu * info.time_scale_ms / 1000.0
        log_path = self._new_pvt_log_path(axis)
        samples_written = 0
        points_sent = 1
        counter = 0
        stream_index = 1

        self.check(self.dll.TS_GOTO(0x4000), "TS_GOTO(0x4000)")
        self.check(self.dll.TS_PVTSetup(1, 1, 1, 1, 1, 1, 1), "TS_PVTSetup")
        first_pos, first_vel = points[0]
        self.check(
            self.dll.TS_SendPVTFirstPoint(
                first_pos,
                first_vel,
                segment_time_iu,
                counter,
                ABSOLUTE_POSITION,
                0,
                UPDATE_NONE,
                FROM_REFERENCE,
            ),
            "TS_SendPVTFirstPoint",
        )
        initial_preload = min(PVT_MAX_PRELOAD_POINTS, len(points))
        while stream_index < initial_preload:
            counter = (counter + 1) & 0x3F
            pos_iu, vel_iu = points[stream_index]
            self.check(
                self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, counter),
                "TS_SendPVTPoint",
            )
            stream_index += 1
            points_sent += 1
        self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate")

        t0 = time.monotonic()
        next_sample = t0
        sample_period_s = max(0.05, min(0.2, segment_s / 2.0))
        deadline = t0 + duration_s + 3.0
        update_sent = True
        with log_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    "t_s",
                    "target_deg",
                    "target_velocity_deg_s",
                    "APOS_deg",
                    "CPOS_deg",
                    "TPOS_deg",
                    "points_sent",
                    "buffer_full",
                    "buffer_empty",
                    "buffer_initialise",
                    "raw_messages",
                    "messages",
                    "last_address",
                    "last_value",
                ],
            )
            writer.writeheader()
            while True:
                self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")

                now = time.monotonic()
                elapsed = now - t0
                if status["buffer_empty"]:
                    if stream_index >= len(points) and elapsed >= duration_s - segment_s:
                        break
                    raise RuntimeError("PVT buffer empty before all points were streamed")

                if status["messages"]:
                    should_send = not status["buffer_full"]
                else:
                    next_point_needed_at = max(0.0, (points_sent - PVT_STREAM_FEED_MARGIN_POINTS) * segment_s)
                    should_send = elapsed >= next_point_needed_at

                if should_send and stream_index < len(points):
                    counter = (counter + 1) & 0x3F
                    pos_iu, vel_iu = points[stream_index]
                    self.check(
                        self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, counter),
                        "TS_SendPVTPoint",
                    )
                    stream_index += 1
                    points_sent += 1

                if now >= next_sample:
                    pos = self.read_positions(axis)
                    expected_index = min(int(elapsed / segment_s), len(points) - 1)
                    expected_pos_iu, expected_vel_iu = points[expected_index]
                    writer.writerow(
                        {
                            "t_s": f"{elapsed:.4f}",
                            "target_deg": f"{self.iu_to_deg(axis, expected_pos_iu - int(VERTICAL_ZERO_IU[axis.lower()])):.6f}",
                            "target_velocity_deg_s": f"{self.velocity_iu_to_deg_s(axis, expected_vel_iu):.6f}",
                            "APOS_deg": f"{float(pos['APOS_deg']):.6f}",
                            "CPOS_deg": f"{float(pos['CPOS_deg']):.6f}",
                            "TPOS_deg": f"{float(pos['TPOS_deg']):.6f}",
                            "points_sent": points_sent,
                            "buffer_full": status["buffer_full"],
                            "buffer_empty": status["buffer_empty"],
                            "buffer_initialise": status["buffer_initialise"],
                            "raw_messages": status["raw_messages"],
                            "messages": status["messages"],
                            "last_address": f"0x{status['last_address']:04X}",
                            "last_value": status["last_value"],
                        }
                    )
                    samples_written += 1
                    next_sample += sample_period_s

                if points_sent >= len(points) and now - t0 >= duration_s + 0.5:
                    break
                if now > deadline:
                    raise RuntimeError("Streaming PVT timed out")
                time.sleep(0.002)

        end_pos = self.read_positions(axis)
        final_command_error_deg = float(end_pos["TPOS_deg"]) - float(final_target_deg)
        if abs(final_command_error_deg) > PVT_TARGET_TOLERANCE_DEG:
            raise RuntimeError(
                f"Streaming PVT did not reach target reference: TPOS error {final_command_error_deg:.3f} deg. "
                f"Log: {log_path}"
            )
        result = dict(metadata)
        result.update(
            {
                "segment_time_iu": segment_time_iu,
                "points_sent": points_sent,
                "update_sent": int(update_sent),
                "raw_messages": status["raw_messages"],
                "pvt_messages": status["messages"],
                "last_message_address": f"0x{status['last_address']:04X}",
                "end_APOS_deg": float(end_pos["APOS_deg"]),
                "end_TPOS_deg": float(end_pos["TPOS_deg"]),
                "final_TPOS_error_deg": final_command_error_deg,
                "samples_written": samples_written,
                "log_path": str(log_path),
            }
        )
        return result

    def _new_pvt_log_path(self, axis: str) -> Path:
        PVT_LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return PVT_LOG_DIR / f"technosoft_pvt_{axis.lower()}_{stamp}.csv"

    def current_vertical_zero_hardcode(self, axis: str) -> dict[str, int | str]:
        self.select_axis(axis)
        apos = self._read_long("APOS")
        constant_name = f"{axis.upper()}_VERTICAL_ZERO_IU"
        return {
            "axis": axis.lower(),
            "apos": apos,
            "constant_name": constant_name,
            "code_line": f"{constant_name} = {apos}",
        }

    def wait_motion_complete(self, axis: str) -> None:
        self.select_axis(axis)
        self.check(self.dll.TS_SetEventOnMotionComplete(WAIT_EVENT, NO_STOP), "TS_SetEventOnMotionComplete")

    def drive_initialisation(self, axis: str) -> None:
        self.select_axis(axis)
        self.check(self.dll.TS_DriveInitialisation(), "TS_DriveInitialisation")

    def power(self, axis: str, enable: bool) -> None:
        self.select_axis(axis)
        self.check(self.dll.TS_Power(POWER_ON if enable else POWER_OFF), f"TS_Power({enable})")

    def stop(self, axis: str) -> None:
        self.select_axis(axis)
        self.check(self.dll.TS_Stop(), "TS_Stop")

    def reset_fault(self, axis: str) -> None:
        self.select_axis(axis)
        self.check(self.dll.TS_ResetFault(), "TS_ResetFault")

    def close(self) -> None:
        if self.dll is not None:
            if self.channel_fd is not None:
                self.dll.TS_CloseChannel(int(self.channel_fd))
            else:
                self.dll.TS_CloseChannel(-1)
        self.channel_fd = None
        self.axes.clear()
