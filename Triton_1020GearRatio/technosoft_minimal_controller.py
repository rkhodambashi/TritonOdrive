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
import os
import time
from dataclasses import dataclass
from pathlib import Path


TML_LIB_DLL = Path(r"C:\Program Files (x86)\Technosoft\TML_LIB_x64\lib\TML_lib.dll")
DEFAULT_SETUP_PATH = Path(r"C:\TML_Setups\ipos8020_dual_loop_master.t.zip")

CHANNEL_PEAK_SYS_PCAN_USB = 7
DEFAULT_CHANNEL_NAME = "1"
DEFAULT_HOST_ID = 255
DEFAULT_BAUDRATE = 1_000_000
PVT_MAX_TIME_IU = 511
PVT_MAX_PRELOAD_POINTS = 8

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

    def last_error(self) -> str:
        if self.dll is None:
            return "TML_LIB not loaded"
        buffer = ctypes.create_string_buffer(1024)
        self.dll.TS_Basic_GetLastErrorText(buffer, len(buffer))
        return buffer.value.decode("mbcs", errors="replace").strip() or "TML_LIB call failed"

    def check(self, ok: int, action: str) -> None:
        if not ok:
            raise RuntimeError(f"{action} failed: {self.last_error()}")

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
