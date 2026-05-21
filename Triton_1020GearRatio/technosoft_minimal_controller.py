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
PVT_TARGET_TOLERANCE_DEG = 0.25
PVT_ACTUAL_FOLLOWING_ERROR_DEG = 5.0
PVT_ACTUAL_FOLLOWING_GRACE_S = 5.0
PVT_STATUS_ADDRESS = 0x0863
PVT_COUNTER_MASK = 0x3F
PVT_BUFFER_FULL_BIT = 1 << 13
PVT_BUFFER_LOW_BIT = 1 << 14
PVT_BUFFER_EMPTY_BIT = 1 << 15
PVT_WORDS_PER_POINT = 9
PVT_DESIRED_BUFFER_POINTS = 21
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
                    "buffer_low": 0,
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
            status["buffer_full"] = 1 if value & PVT_BUFFER_FULL_BIT else 0
            status["buffer_low"] = 1 if value & PVT_BUFFER_LOW_BIT else 0
            if value & PVT_BUFFER_EMPTY_BIT:
                status["buffer_empty"] = 1
            status["counter_dsp"] = value & PVT_COUNTER_MASK
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
            "buffer_low": 0,
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

    def _update_pvt_status_from_word(self, status: dict[str, int], value: int) -> None:
        status["last_address"] = PVT_STATUS_ADDRESS
        status["last_value"] = int(value)
        status["buffer_full"] = 1 if value & PVT_BUFFER_FULL_BIT else 0
        status["buffer_low"] = 1 if value & PVT_BUFFER_LOW_BIT else 0
        if value & PVT_BUFFER_EMPTY_BIT:
            status["buffer_empty"] = 1
        elif status.get("buffer_low", 0):
            status["buffer_empty"] = 0
        status["counter_dsp"] = value & PVT_COUNTER_MASK
        if not status["buffer_initialise"] and status["buffer_full"]:
            status["buffer_initialise"] = 1
            status["buffer_empty"] = 0

    def _poll_pvt_status(self, status: dict[str, int]) -> None:
        self._update_pvt_status_from_word(status, self._read_int("PVTSTS"))

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

    def _read_fixed(self, name: str) -> float:
        value = ctypes.c_double()
        self.check(self.dll.TS_GetFixedVariable(name.encode("ascii"), ctypes.byref(value)), f"TS_GetFixedVariable({name})")
        return float(value.value)

    def _write_int(self, name: str, value: int) -> None:
        clamped = max(min(int(value), 32767), -32768)
        self.check(
            self.dll.TS_SetIntVariable(name.encode("ascii"), ctypes.c_int16(clamped)),
            f"TS_SetIntVariable({name})",
        )

    def _configure_pvt_buffer(self, axis: str) -> dict[str, int]:
        """Increase PVT FIFO capacity and choose a safer BufferLow threshold."""
        self.select_axis(axis)
        begin_words = self._read_int("PVTBUFBEGIN")
        old_len_words = self._read_int("PVTBUFLEN")
        requested_len_words = PVT_DESIRED_BUFFER_POINTS * PVT_WORDS_PER_POINT
        if old_len_words < requested_len_words:
            self._write_int("PVTBUFLEN", requested_len_words)
        new_len_words = self._read_int("PVTBUFLEN")
        buffer_points = max(1, new_len_words // PVT_WORDS_PER_POINT)
        low_level = max(1, buffer_points // 3)
        preload_points = max(1, buffer_points - low_level)
        return {
            "pvt_buffer_begin_words": begin_words,
            "pvt_buffer_old_len_words": old_len_words,
            "pvt_buffer_len_words": new_len_words,
            "pvt_buffer_points": buffer_points,
            "pvt_low_level": low_level,
            "pvt_preload_points": preload_points,
        }

    def _pvt_setup(self, axis: str) -> dict[str, int]:
        buffer_info = self._configure_pvt_buffer(axis)
        self.check(
            self.dll.TS_PVTSetup(1, 0, 0, 1, 1, 0, buffer_info["pvt_low_level"]),
            f"TS_PVTSetup({axis})",
        )
        return buffer_info

    def read_positions(self, axis: str) -> dict[str, float | int]:
        info = self.select_axis(axis)
        apos = self._read_long("APOS")
        cpos = self._read_long("CPOS")
        tpos = self._read_long("TPOS")
        motor_pos = self._read_long("APOS_MT")
        load_speed = self._read_fixed("ASPD_LD")
        motor_speed = self._read_fixed("ASPD_MT")
        motor_current = self._read_int("Motor_Current")
        current_ref = self._read_int("Current_Reference")
        zero = int(VERTICAL_ZERO_IU[axis.lower()])
        return {
            "APOS": apos,
            "CPOS": cpos,
            "TPOS": tpos,
            "APOS_MT": motor_pos,
            "ASPD_LD": load_speed,
            "ASPD_MT": motor_speed,
            "Motor_Current": motor_current,
            "Current_Reference": current_ref,
            "zero_iu": zero,
            "APOS_deg": self.iu_to_deg(axis, apos - zero),
            "CPOS_deg": self.iu_to_deg(axis, cpos - zero),
            "TPOS_deg": self.iu_to_deg(axis, tpos - zero),
            "APOS_MT_motor_rev": motor_pos / info.motor_scale if info.motor_scale else 0.0,
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

    def home_absolute_deg(self, axis: str, target_deg: float = 0.0) -> None:
        """Start Home from the measured position, not a stale reference path."""
        info = self.select_axis(axis)
        self.check(self.dll.TS_Stop(), "TS_Stop")
        self.check(self.dll.TS_SetTargetPositionToActual(), "TS_SetTargetPositionToActual")
        target_iu = self.absolute_deg_to_iu(axis, target_deg)
        self.check(
            self.dll.TS_MoveAbsolute(
                target_iu,
                info.speed_iu,
                info.accel_iu,
                UPDATE_IMMEDIATE,
                FROM_REFERENCE,
            ),
            "TS_MoveAbsolute(home)",
        )

    def home_absolute_deg_checked(
        self,
        axis: str,
        target_deg: float = 0.0,
        timeout_s: float | None = None,
        tolerance_deg: float = 0.25,
        following_error_deg: float = 5.0,
    ) -> dict[str, float | int]:
        """Home from measured position and verify the axis actually settles."""
        self.home_absolute_deg(axis, target_deg)
        start_pos = self.read_positions(axis)
        info = self.axes[axis.lower()]
        distance_deg = abs(float(target_deg) - float(start_pos["APOS_deg"]))
        if timeout_s is None:
            speed_deg_s = max(0.05, abs(float(info.speed_deg_s)))
            accel_deg_s2 = max(0.05, abs(float(info.accel_deg_s2)))
            timeout_s = distance_deg / speed_deg_s + 2.0 * speed_deg_s / accel_deg_s2 + 10.0
        deadline = time.monotonic() + max(0.1, timeout_s)
        progress_check_at = time.monotonic() + 5.0
        start_apos_deg = float(start_pos["APOS_deg"])
        start_tpos_deg = float(start_pos["TPOS_deg"])
        last_pos = start_pos
        while time.monotonic() < deadline:
            pos = self.read_positions(axis)
            last_pos = pos
            apos_tpos_error = float(pos["APOS_deg"]) - float(pos["TPOS_deg"])
            target_error = float(pos["APOS_deg"]) - float(target_deg)
            if time.monotonic() >= progress_check_at and abs(target_error) > tolerance_deg:
                apos_progress = abs(float(pos["APOS_deg"]) - start_apos_deg)
                tpos_progress = abs(float(pos["TPOS_deg"]) - start_tpos_deg)
                if apos_progress < 0.05 and tpos_progress < 0.05:
                    raise RuntimeError(
                        f"{axis.upper()} home did not start moving: "
                        f"APOS progress={apos_progress:.3f} deg, TPOS progress={tpos_progress:.3f} deg, "
                        f"target={float(target_deg):.3f} deg"
                    )
                progress_check_at = time.monotonic() + 5.0
            if abs(apos_tpos_error) > following_error_deg:
                raise RuntimeError(
                    f"{axis.upper()} actual position is not following home reference: "
                    f"APOS-TPOS={apos_tpos_error:.3f} deg"
                )
            if abs(target_error) <= tolerance_deg and abs(apos_tpos_error) <= tolerance_deg:
                return pos
            time.sleep(0.05)
        apos_tpos_error = float(last_pos["APOS_deg"]) - float(last_pos["TPOS_deg"])
        target_error = float(last_pos["APOS_deg"]) - float(target_deg)
        raise RuntimeError(
            f"{axis.upper()} home did not settle: APOS={float(last_pos['APOS_deg']):.3f} deg, "
            f"TPOS={float(last_pos['TPOS_deg']):.3f} deg, target={float(target_deg):.3f} deg, "
            f"APOS-target={target_error:.3f} deg, APOS-TPOS={apos_tpos_error:.3f} deg, "
            f"timeout={timeout_s:.1f}s"
        )

    def recover_and_home_absolute_deg(self, axis: str, target_deg: float = 0.0) -> None:
        """Recover from PVT/reference mode before commanding Home."""
        info = self.select_axis(axis)
        self.check(self.dll.TS_Stop(), "TS_Stop(recover)")
        self.check(self.dll.TS_Power(POWER_OFF), "TS_Power(OFF,recover)")
        time.sleep(0.2)
        self.check(self.dll.TS_DriveInitialisation(), "TS_DriveInitialisation(recover)")
        self.check(self.dll.TS_Power(POWER_ON), "TS_Power(ON,recover)")
        self.check(self.dll.TS_SetTargetPositionToActual(), "TS_SetTargetPositionToActual(recover)")
        target_iu = self.absolute_deg_to_iu(axis, target_deg)
        self.check(
            self.dll.TS_MoveAbsolute(
                target_iu,
                info.speed_iu,
                info.accel_iu,
                UPDATE_IMMEDIATE,
                FROM_REFERENCE,
            ),
            "TS_MoveAbsolute(recover home)",
        )

    def move_absolute_deg_checked(
        self,
        axis: str,
        target_deg: float,
        timeout_s: float | None = None,
        tolerance_deg: float = 0.25,
        following_error_deg: float = 5.0,
    ) -> dict[str, float | int]:
        start_pos = self.read_positions(axis)
        info = self.axes[axis.lower()]
        distance_deg = abs(float(target_deg) - float(start_pos["APOS_deg"]))
        if timeout_s is None:
            speed_deg_s = max(0.05, abs(float(info.speed_deg_s)))
            accel_deg_s2 = max(0.05, abs(float(info.accel_deg_s2)))
            timeout_s = distance_deg / speed_deg_s + 2.0 * speed_deg_s / accel_deg_s2 + 10.0
        self.move_absolute_deg(axis, target_deg)
        deadline = time.monotonic() + max(0.1, timeout_s)
        progress_check_at = time.monotonic() + 5.0
        start_apos_deg = float(start_pos["APOS_deg"])
        start_tpos_deg = float(start_pos["TPOS_deg"])
        last_pos = start_pos
        while time.monotonic() < deadline:
            pos = self.read_positions(axis)
            last_pos = pos
            apos_tpos_error = float(pos["APOS_deg"]) - float(pos["TPOS_deg"])
            target_error = float(pos["APOS_deg"]) - float(target_deg)
            if time.monotonic() >= progress_check_at and abs(target_error) > tolerance_deg:
                apos_progress = abs(float(pos["APOS_deg"]) - start_apos_deg)
                tpos_progress = abs(float(pos["TPOS_deg"]) - start_tpos_deg)
                if apos_progress < 0.05 and tpos_progress < 0.05:
                    raise RuntimeError(
                        f"{axis.upper()} command did not start moving: "
                        f"APOS progress={apos_progress:.3f} deg, TPOS progress={tpos_progress:.3f} deg, "
                        f"target={float(target_deg):.3f} deg"
                    )
                progress_check_at = time.monotonic() + 5.0
            if abs(apos_tpos_error) > following_error_deg:
                raise RuntimeError(
                    f"{axis.upper()} actual position is not following reference: "
                    f"APOS-TPOS={apos_tpos_error:.3f} deg after absolute move command"
                )
            if abs(target_error) <= tolerance_deg and abs(apos_tpos_error) <= tolerance_deg:
                return pos
            time.sleep(0.05)
        apos_tpos_error = float(last_pos["APOS_deg"]) - float(last_pos["TPOS_deg"])
        target_error = float(last_pos["APOS_deg"]) - float(target_deg)
        raise RuntimeError(
            f"{axis.upper()} move did not settle: APOS={float(last_pos['APOS_deg']):.3f} deg, "
            f"TPOS={float(last_pos['TPOS_deg']):.3f} deg, target={float(target_deg):.3f} deg, "
            f"APOS-target={target_error:.3f} deg, APOS-TPOS={apos_tpos_error:.3f} deg, "
            f"timeout={timeout_s:.1f}s"
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
        self._pvt_setup(axis)
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
                self.dll.TS_SendPVTPoint(point, velocity_iu, segment_time_iu, counter & PVT_COUNTER_MASK),
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

    def run_streaming_pvt_xy_deg(
        self,
        x_points_deg: list[tuple[float, float]],
        y_points_deg: list[tuple[float, float]],
        duration_s: float,
        metadata: dict[str, float | int | str] | None = None,
    ) -> dict[str, float | int | str]:
        """Stream synchronized absolute X/Y PVT points.

        Each point is (position_deg, velocity_deg_s). X and Y must have the
        same point count and duration so both axes consume one PVT segment per
        sample. This is intentionally generic; satellite tracking is just one
        producer of these point lists.
        """
        if len(x_points_deg) != len(y_points_deg):
            raise ValueError("X and Y PVT point lists must have the same length")
        if len(x_points_deg) < 4:
            raise ValueError("XY PVT point count must be at least 4")

        info_x = self.select_axis("x")
        segment_time_iu_x, segment_s_x = self._pvt_segment_timing(info_x, duration_s, len(x_points_deg))
        info_y = self.select_axis("y")
        segment_time_iu_y, segment_s_y = self._pvt_segment_timing(info_y, duration_s, len(y_points_deg))
        if segment_time_iu_x != segment_time_iu_y:
            raise RuntimeError(
                f"X/Y time scales do not match for synchronized PVT: X={segment_time_iu_x}, Y={segment_time_iu_y}"
            )

        self._register_pvt_handler()
        status_x = self._reset_pvt_status(info_x.config.node_id)
        status_y = self._reset_pvt_status(info_y.config.node_id)
        self.check(
            self.dll.TS_SendDataToHost(DEFAULT_HOST_ID, 0x00000400, 0xFFFF),
            "TS_SendDataToHost",
        )

        start_x = self.read_positions("x")
        start_y = self.read_positions("y")
        x_points = [
            (self.absolute_deg_to_iu("x", pos_deg), self.velocity_deg_s_to_iu("x", vel_deg_s))
            for pos_deg, vel_deg_s in x_points_deg
        ]
        y_points = [
            (self.absolute_deg_to_iu("y", pos_deg), self.velocity_deg_s_to_iu("y", vel_deg_s))
            for pos_deg, vel_deg_s in y_points_deg
        ]

        log_path = self._new_pvt_log_path("xy")
        stream_x = self._start_axis_pvt_stream("x", x_points, segment_time_iu_x)
        stream_y = self._start_axis_pvt_stream("y", y_points, segment_time_iu_y)
        self.select_axis("x")
        self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate(x)")
        self.select_axis("y")
        self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate(y)")

        sample_count = self._stream_xy_pvt_points(
            x_points,
            y_points,
            duration_s,
            segment_s_x,
            segment_time_iu_x,
            stream_x,
            stream_y,
            status_x,
            status_y,
            log_path,
        )
        end_x = self.read_positions("x")
        end_y = self.read_positions("y")
        final_x_deg = self.iu_to_deg("x", x_points[-1][0] - int(VERTICAL_ZERO_IU["x"]))
        final_y_deg = self.iu_to_deg("y", y_points[-1][0] - int(VERTICAL_ZERO_IU["y"]))
        result = dict(metadata or {})
        result.update(
            {
                "trajectory": result.get("trajectory", "xy"),
                "duration_s": float(duration_s),
                "point_count": len(x_points),
                "segment_time_iu": segment_time_iu_x,
                "start_x_deg": float(start_x["TPOS_deg"]),
                "start_y_deg": float(start_y["TPOS_deg"]),
                "final_x_target_deg": float(final_x_deg),
                "final_y_target_deg": float(final_y_deg),
                "end_x_APOS_deg": float(end_x["APOS_deg"]),
                "end_y_APOS_deg": float(end_y["APOS_deg"]),
                "end_x_TPOS_deg": float(end_x["TPOS_deg"]),
                "end_y_TPOS_deg": float(end_y["TPOS_deg"]),
                "x_final_TPOS_error_deg": float(end_x["TPOS_deg"]) - float(final_x_deg),
                "y_final_TPOS_error_deg": float(end_y["TPOS_deg"]) - float(final_y_deg),
                "x_raw_messages": status_x["raw_messages"],
                "y_raw_messages": status_y["raw_messages"],
                "x_pvt_messages": status_x["messages"],
                "y_pvt_messages": status_y["messages"],
                "samples_written": sample_count,
                "log_path": str(log_path),
            }
        )
        return result

    def run_streaming_pvt_axis_deg(
        self,
        axis: str,
        points_deg: list[tuple[float, float]],
        duration_s: float,
        metadata: dict[str, float | int | str] | None = None,
        stop_requested=None,
        ui_pump=None,
        sample_callback=None,
    ) -> dict[str, float | int | str]:
        """Stream arbitrary absolute PVT points on one axis.

        Each point is (position_deg, velocity_deg_s). This is used for
        single-axis satellite bring-up before both axes are installed.
        """
        axis = axis.lower()
        if axis not in ("x", "y"):
            raise ValueError("axis must be 'x' or 'y'")
        if len(points_deg) < 4:
            raise ValueError("Axis PVT point count must be at least 4")

        info, status, _start_pos = self._prepare_pvt_stream(axis)
        segment_time_iu, _segment_s = self._pvt_segment_timing(info, duration_s, len(points_deg))
        points = [
            (self.absolute_deg_to_iu(axis, pos_deg), self.velocity_deg_s_to_iu(axis, vel_deg_s))
            for pos_deg, vel_deg_s in points_deg
        ]
        # Technosoft's manual states a PVT sequence must end with zero
        # velocity; otherwise buffer-empty enters quick-stop behavior.
        points[-1] = (points[-1][0], 0.0)
        final_target_deg = self.iu_to_deg(axis, points[-1][0] - int(VERTICAL_ZERO_IU[axis]))
        base_metadata = {
            "trajectory": "axis",
            "axis": axis,
            "duration_s": float(duration_s),
            "point_count": len(points),
            "final_target_deg": float(final_target_deg),
        }
        if metadata:
            base_metadata.update(metadata)
        return self._stream_pvt_points(
            axis,
            info,
            status,
            points,
            duration_s,
            segment_time_iu,
            final_target_deg=float(final_target_deg),
            metadata=base_metadata,
            stop_requested=stop_requested,
            ui_pump=ui_pump,
            sample_callback=sample_callback,
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

    def _start_axis_pvt_stream(
        self,
        axis: str,
        points: list[tuple[int, float]],
        segment_time_iu: int,
    ) -> dict[str, int]:
        self.select_axis(axis)
        self.check(self.dll.TS_Stop(), f"TS_Stop({axis})")
        self.check(self.dll.TS_SetTargetPositionToActual(), f"TS_SetTargetPositionToActual({axis})")
        self.check(self.dll.TS_GOTO(0x4000), f"TS_GOTO(0x4000,{axis})")
        buffer_info = self._pvt_setup(axis)
        first_pos, first_vel = points[0]
        self.check(
            self.dll.TS_SendPVTFirstPoint(
                first_pos,
                first_vel,
                segment_time_iu,
                0,
                ABSOLUTE_POSITION,
                0,
                UPDATE_NONE,
                FROM_REFERENCE,
            ),
            f"TS_SendPVTFirstPoint({axis})",
        )
        stream_index = 1
        points_sent = 1
        counter = 0
        initial_preload = min(buffer_info["pvt_preload_points"], len(points))
        while stream_index < initial_preload:
            counter += 1
            pos_iu, vel_iu = points[stream_index]
            self.check(
                self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, counter),
                f"TS_SendPVTPoint({axis})",
            )
            counter &= PVT_COUNTER_MASK
            stream_index += 1
            points_sent += 1
        result = dict(buffer_info)
        result.update({"counter": counter, "stream_index": stream_index, "points_sent": points_sent})
        return result

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
        stop_requested=None,
        ui_pump=None,
        sample_callback=None,
    ) -> dict[str, float | int | str]:
        if not points:
            raise ValueError("PVT point list is empty")
        segment_s = segment_time_iu * info.time_scale_ms / 1000.0
        log_path = self._new_pvt_log_path(axis)
        samples_written = 0
        self.check(self.dll.TS_GOTO(0x4000), "TS_GOTO(0x4000)")
        buffer_info = self._pvt_setup(axis)
        first_pos, first_vel = points[0]
        counter_pc = 0
        stream_index = 1
        points_sent = 1
        self.check(
            self.dll.TS_SendPVTFirstPoint(
                first_pos,
                first_vel,
                segment_time_iu,
                counter_pc,
                ABSOLUTE_POSITION,
                0,
                UPDATE_NONE,
                FROM_REFERENCE,
            ),
            "TS_SendPVTFirstPoint",
        )
        update_sent = False
        update_reason = "not_started"

        def start_pvt_if_buffer_initialised(reason: str) -> bool:
            nonlocal update_sent, update_reason
            if update_sent:
                return False
            if status["buffer_initialise"] or status["buffer_full"]:
                self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate")
                update_sent = True
                update_reason = reason
                if status["buffer_initialise"] == 1:
                    status["buffer_initialise"] = 2
                return True
            return False

        self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
        start_pvt_if_buffer_initialised("initial_status")
        preload_points = min(buffer_info["pvt_preload_points"], len(points))
        while stream_index < preload_points:
            counter_pc += 1
            pos_iu, vel_iu = points[stream_index]
            self.check(
                self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, counter_pc),
                "TS_SendPVTPoint",
            )
            counter_pc &= PVT_COUNTER_MASK
            stream_index += 1
            points_sent = stream_index
            self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
            start_pvt_if_buffer_initialised("buffer_initialised_during_preload")
        start_wait_deadline = time.monotonic() + 0.5
        while not update_sent and time.monotonic() < start_wait_deadline:
            self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
            start_pvt_if_buffer_initialised("buffer_initialised_after_preload")
            if update_sent:
                break
            time.sleep(0.002)
        if not update_sent:
            self.check(self.dll.TS_UpdateImmediate(), "TS_UpdateImmediate")
            update_sent = True
            update_reason = "fallback_no_buffer_status"
        refill_events = 0
        last_refill_messages = status["messages"]
        last_refill_reason = "initial_preload"
        last_refill_sent = 0
        next_refill_allowed = 0.0

        def refill_buffer(reason: str) -> int:
            nonlocal counter_pc, stream_index, points_sent, next_refill_allowed
            nonlocal refill_events, last_refill_reason, last_refill_sent, last_refill_messages
            points_to_add = max(1, buffer_info["pvt_buffer_points"] - buffer_info["pvt_low_level"])
            target_stream_index = min(len(points), stream_index + points_to_add)
            sent_now = 0
            while stream_index < target_stream_index and not status["buffer_full"]:
                counter_pc += 1
                pos_iu, vel_iu = points[stream_index]
                self.check(
                    self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, counter_pc),
                    "TS_SendPVTPoint",
                )
                counter_pc &= PVT_COUNTER_MASK
                stream_index += 1
                points_sent = stream_index
                sent_now += 1
                self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
            if sent_now:
                refill_events += 1
                last_refill_reason = reason
                last_refill_sent = sent_now
                last_refill_messages = status["messages"]
                status["buffer_empty"] = 0
                # A single BufferLow condition can generate repeated messages while
                # we are filling. Do not respond again until the drive has had time
                # to consume most of the points we just added.
                next_refill_allowed = time.monotonic() + max(segment_s, sent_now * segment_s * 0.75)
            return sent_now

        t0 = time.monotonic()
        next_sample = t0
        next_status_poll = t0
        sample_period_s = max(0.05, min(0.2, segment_s / 2.0))
        status_poll_period_s = max(0.05, min(0.2, segment_s / 2.0))
        deadline = t0 + duration_s + 3.0

        def interpolated_target(elapsed_s: float) -> tuple[float, float, float]:
            """Return target position/velocity at the logger sample timestamp."""
            point_phase = max(0.0, min(float(elapsed_s) / segment_s, float(len(points) - 1)))
            lower_index = int(point_phase)
            upper_index = min(lower_index + 1, len(points) - 1)
            fraction = point_phase - lower_index
            lower_pos_iu, lower_vel_iu = points[lower_index]
            upper_pos_iu, upper_vel_iu = points[upper_index]
            pos_iu = lower_pos_iu + (upper_pos_iu - lower_pos_iu) * fraction
            vel_iu = lower_vel_iu + (upper_vel_iu - lower_vel_iu) * fraction
            target_deg = self.iu_to_deg(axis, pos_iu - int(VERTICAL_ZERO_IU[axis.lower()]))
            target_vel_deg_s = self.velocity_iu_to_deg_s(axis, vel_iu)
            return point_phase, target_deg, target_vel_deg_s

        with log_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    "t_s",
                    "sample_wall_time_ns",
                    "sample_monotonic_ns",
                    "position_read_done_monotonic_ns",
                    "position_read_duration_ms",
                    "target_point_phase",
                    "target_deg",
                    "target_velocity_deg_s",
                    "target_step_index",
                    "target_step_deg",
                    "target_step_velocity_deg_s",
                    "APOS_deg",
                    "CPOS_deg",
                    "TPOS_deg",
                    "APOS_MT_raw",
                    "APOS_MT_motor_rev",
                    "ASPD_LD",
                    "ASPD_MT",
                    "Motor_Current",
                    "Current_Reference",
                    "APOS_minus_target_deg",
                    "TPOS_minus_target_deg",
                    "APOS_minus_TPOS_deg",
                    "points_sent",
                    "stream_index",
                    "counter_pc",
                    "counter_dsp",
                    "pvt_buffer_points",
                    "pvt_low_level",
                    "pvt_preload_points",
                    "refill_events",
                    "last_refill_reason",
                    "last_refill_sent",
                    "buffer_full",
                    "buffer_low",
                    "buffer_empty",
                    "buffer_initialise",
                    "update_sent",
                    "raw_messages",
                    "messages",
                    "last_address",
                    "last_value",
                ],
            )
            writer.writeheader()
            while True:
                self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
                if ui_pump is not None:
                    ui_pump()
                if stop_requested is not None and stop_requested():
                    raise RuntimeError("PVT stopped by user")

                now = time.monotonic()
                elapsed = now - t0
                if now >= next_status_poll:
                    self._poll_pvt_status(status)
                    next_status_poll = now + status_poll_period_s

                refill_allowed = now >= next_refill_allowed
                if status["buffer_empty"] and stream_index < len(points):
                    if elapsed < max(0.5, 2.0 * segment_s):
                        status["buffer_empty"] = 0
                    elif refill_allowed:
                        if refill_buffer("buffer_empty") == 0:
                            raise RuntimeError(
                                f"{axis.upper()} PVT buffer empty before all points were streamed "
                                f"(sent={points_sent}/{len(points)}, stream_index={stream_index}, "
                                f"counter_pc={counter_pc}, counter_dsp={status['counter_dsp']}, "
                                f"last=0x{status['last_value']:04X})"
                            )
                elif status["buffer_low"] and stream_index < len(points) and refill_allowed:
                    refill_buffer("pvt_status_poll")
                    last_refill_messages = status["messages"]
                elif status["messages"] != last_refill_messages and stream_index < len(points) and refill_allowed:
                    refill_buffer("pvt_status_message")
                    last_refill_messages = status["messages"]
                elif stream_index < len(points) and refill_allowed:
                    expected_consumed_index = min(int(elapsed / segment_s), len(points) - 1)
                    estimated_points_buffered = stream_index - expected_consumed_index
                    if estimated_points_buffered <= buffer_info["pvt_low_level"]:
                        refill_buffer("scheduled_low_water")

                if now >= next_sample:
                    sample_monotonic_ns = time.monotonic_ns()
                    sample_wall_time_ns = time.time_ns()
                    pos = self.read_positions(axis)
                    read_done_monotonic_ns = time.monotonic_ns()
                    read_duration_ms = (read_done_monotonic_ns - sample_monotonic_ns) / 1_000_000.0
                    expected_index = min(int(elapsed / segment_s), len(points) - 1)
                    expected_pos_iu, expected_vel_iu = points[expected_index]
                    expected_step_deg = self.iu_to_deg(axis, expected_pos_iu - int(VERTICAL_ZERO_IU[axis.lower()]))
                    expected_step_vel_deg_s = self.velocity_iu_to_deg_s(axis, expected_vel_iu)
                    target_point_phase, expected_deg, expected_vel_deg_s = interpolated_target(elapsed)
                    apos_deg = float(pos["APOS_deg"])
                    tpos_deg = float(pos["TPOS_deg"])
                    actual_error_deg = apos_deg - expected_deg
                    tpos_error_deg = tpos_deg - expected_deg
                    apos_tpos_error_deg = apos_deg - tpos_deg
                    writer.writerow(
                        {
                            "t_s": f"{elapsed:.4f}",
                            "sample_wall_time_ns": sample_wall_time_ns,
                            "sample_monotonic_ns": sample_monotonic_ns,
                            "position_read_done_monotonic_ns": read_done_monotonic_ns,
                            "position_read_duration_ms": f"{read_duration_ms:.3f}",
                            "target_point_phase": f"{target_point_phase:.4f}",
                            "target_deg": f"{expected_deg:.6f}",
                            "target_velocity_deg_s": f"{expected_vel_deg_s:.6f}",
                            "target_step_index": expected_index,
                            "target_step_deg": f"{expected_step_deg:.6f}",
                            "target_step_velocity_deg_s": f"{expected_step_vel_deg_s:.6f}",
                            "APOS_deg": f"{apos_deg:.6f}",
                            "CPOS_deg": f"{float(pos['CPOS_deg']):.6f}",
                            "TPOS_deg": f"{tpos_deg:.6f}",
                            "APOS_MT_raw": pos["APOS_MT"],
                            "APOS_MT_motor_rev": f"{float(pos['APOS_MT_motor_rev']):.6f}",
                            "ASPD_LD": f"{float(pos['ASPD_LD']):.6f}",
                            "ASPD_MT": f"{float(pos['ASPD_MT']):.6f}",
                            "Motor_Current": pos["Motor_Current"],
                            "Current_Reference": pos["Current_Reference"],
                            "APOS_minus_target_deg": f"{actual_error_deg:.6f}",
                            "TPOS_minus_target_deg": f"{tpos_error_deg:.6f}",
                            "APOS_minus_TPOS_deg": f"{apos_tpos_error_deg:.6f}",
                            "points_sent": points_sent,
                            "stream_index": stream_index,
                            "counter_pc": counter_pc,
                            "counter_dsp": status["counter_dsp"],
                            "pvt_buffer_points": buffer_info["pvt_buffer_points"],
                            "pvt_low_level": buffer_info["pvt_low_level"],
                            "pvt_preload_points": buffer_info["pvt_preload_points"],
                            "refill_events": refill_events,
                            "last_refill_reason": last_refill_reason,
                            "last_refill_sent": last_refill_sent,
                            "buffer_full": status["buffer_full"],
                            "buffer_low": status["buffer_low"],
                            "buffer_empty": status["buffer_empty"],
                            "buffer_initialise": status["buffer_initialise"],
                            "update_sent": int(update_sent),
                            "raw_messages": status["raw_messages"],
                            "messages": status["messages"],
                            "last_address": f"0x{status['last_address']:04X}",
                            "last_value": status["last_value"],
                        }
                    )
                    if sample_callback is not None:
                        sample_callback(
                            {
                                "axis": axis,
                                "t_s": elapsed,
                                "target_deg": expected_deg,
                                "APOS_deg": apos_deg,
                                "TPOS_deg": tpos_deg,
                                "APOS_minus_target_deg": actual_error_deg,
                                "TPOS_minus_target_deg": tpos_error_deg,
                                "APOS_minus_TPOS_deg": apos_tpos_error_deg,
                            }
                        )
                    samples_written += 1
                    next_sample += sample_period_s
                    if elapsed > PVT_ACTUAL_FOLLOWING_GRACE_S and abs(apos_tpos_error_deg) > PVT_ACTUAL_FOLLOWING_ERROR_DEG:
                        raise RuntimeError(
                            f"{axis.upper()} actual position is not following PVT reference: "
                            f"APOS-TPOS={apos_tpos_error_deg:.3f} deg at t={elapsed:.1f}s. "
                            f"Log: {log_path}"
                        )

                if points_sent >= len(points) and update_sent and now - t0 >= duration_s + 0.5:
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
                "pvt_buffer_points": buffer_info["pvt_buffer_points"],
                "pvt_low_level": buffer_info["pvt_low_level"],
                "pvt_preload_points": buffer_info["pvt_preload_points"],
                "pvt_buffer_old_len_words": buffer_info["pvt_buffer_old_len_words"],
                "pvt_buffer_len_words": buffer_info["pvt_buffer_len_words"],
                "pvt_start_reason": update_reason,
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

    def _stream_xy_pvt_points(
        self,
        x_points: list[tuple[int, float]],
        y_points: list[tuple[int, float]],
        duration_s: float,
        segment_s: float,
        segment_time_iu: int,
        stream_x: dict[str, int],
        stream_y: dict[str, int],
        status_x: dict[str, int],
        status_y: dict[str, int],
        log_path: Path,
    ) -> int:
        samples_written = 0
        t0 = time.monotonic()
        next_sample = t0
        sample_period_s = max(0.05, min(0.2, segment_s / 2.0))
        deadline = t0 + duration_s + 3.0

        def maybe_send_next(axis: str, points: list[tuple[int, float]], stream: dict[str, int], status: dict[str, int], elapsed: float) -> None:
            if status["buffer_empty"] and stream["stream_index"] < len(points):
                raise RuntimeError(f"{axis.upper()} PVT buffer empty before all points were streamed")

            target_sent_count = min(
                len(points),
                int(elapsed / segment_s) + int(stream.get("pvt_preload_points", PVT_MAX_PRELOAD_POINTS)),
            )
            while (
                stream["stream_index"] < len(points)
                and stream["points_sent"] < target_sent_count
                and not status["buffer_full"]
            ):
                stream["counter"] += 1
                pos_iu, vel_iu = points[stream["stream_index"]]
                self.select_axis(axis)
                self.check(
                    self.dll.TS_SendPVTPoint(pos_iu, vel_iu, segment_time_iu, stream["counter"]),
                    f"TS_SendPVTPoint({axis})",
                )
                stream["counter"] &= PVT_COUNTER_MASK
                stream["stream_index"] += 1
                stream["points_sent"] += 1

        with log_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    "t_s",
                    "x_target_deg",
                    "y_target_deg",
                    "x_target_velocity_deg_s",
                    "y_target_velocity_deg_s",
                    "x_APOS_deg",
                    "y_APOS_deg",
                    "x_TPOS_deg",
                    "y_TPOS_deg",
                    "x_points_sent",
                    "y_points_sent",
                    "x_buffer_full",
                    "y_buffer_full",
                    "x_buffer_empty",
                    "y_buffer_empty",
                    "x_raw_messages",
                    "y_raw_messages",
                    "x_pvt_messages",
                    "y_pvt_messages",
                ],
            )
            writer.writeheader()
            while True:
                self.check(self.dll.TS_CheckForUnrequestedDriveMessages(), "TS_CheckForUnrequestedDriveMessages")
                now = time.monotonic()
                elapsed = now - t0

                if status_x["buffer_empty"] and stream_x["stream_index"] >= len(x_points) and elapsed >= duration_s - segment_s:
                    pass
                else:
                    maybe_send_next("x", x_points, stream_x, status_x, elapsed)
                if status_y["buffer_empty"] and stream_y["stream_index"] >= len(y_points) and elapsed >= duration_s - segment_s:
                    pass
                else:
                    maybe_send_next("y", y_points, stream_y, status_y, elapsed)

                if now >= next_sample:
                    x_pos = self.read_positions("x")
                    y_pos = self.read_positions("y")
                    expected_index = min(int(elapsed / segment_s), len(x_points) - 1)
                    x_expected_pos, x_expected_vel = x_points[expected_index]
                    y_expected_pos, y_expected_vel = y_points[expected_index]
                    writer.writerow(
                        {
                            "t_s": f"{elapsed:.4f}",
                            "x_target_deg": f"{self.iu_to_deg('x', x_expected_pos - int(VERTICAL_ZERO_IU['x'])):.6f}",
                            "y_target_deg": f"{self.iu_to_deg('y', y_expected_pos - int(VERTICAL_ZERO_IU['y'])):.6f}",
                            "x_target_velocity_deg_s": f"{self.velocity_iu_to_deg_s('x', x_expected_vel):.6f}",
                            "y_target_velocity_deg_s": f"{self.velocity_iu_to_deg_s('y', y_expected_vel):.6f}",
                            "x_APOS_deg": f"{float(x_pos['APOS_deg']):.6f}",
                            "y_APOS_deg": f"{float(y_pos['APOS_deg']):.6f}",
                            "x_TPOS_deg": f"{float(x_pos['TPOS_deg']):.6f}",
                            "y_TPOS_deg": f"{float(y_pos['TPOS_deg']):.6f}",
                            "x_points_sent": stream_x["points_sent"],
                            "y_points_sent": stream_y["points_sent"],
                            "x_buffer_full": status_x["buffer_full"],
                            "y_buffer_full": status_y["buffer_full"],
                            "x_buffer_empty": status_x["buffer_empty"],
                            "y_buffer_empty": status_y["buffer_empty"],
                            "x_raw_messages": status_x["raw_messages"],
                            "y_raw_messages": status_y["raw_messages"],
                            "x_pvt_messages": status_x["messages"],
                            "y_pvt_messages": status_y["messages"],
                        }
                    )
                    samples_written += 1
                    next_sample += sample_period_s

                if (
                    stream_x["points_sent"] >= len(x_points)
                    and stream_y["points_sent"] >= len(y_points)
                    and elapsed >= duration_s + 0.5
                ):
                    break
                if now > deadline:
                    raise RuntimeError("XY streaming PVT timed out")
                time.sleep(0.002)
        return samples_written

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
