"""Reusable EnDat serial reader for the F28379D LaunchPad bridge."""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

try:
    import serial
except ImportError:  # pragma: no cover - handled when constructing the reader
    serial = None


DEFAULT_BAUD = 115200
DEFAULT_COUNTS_PER_REV = 1 << 25
COMMON_BAUD_RATES = (
    1200,
    2400,
    4800,
    7200,
    9600,
    14400,
    19200,
    28800,
    38400,
    57600,
    76800,
    115200,
    230400,
    460800,
    921600,
)
CSV_HEADER = [
    "loop",
    "pos_hi",
    "pos_lo",
    "pos32",
    "position_clocks",
    "error1",
    "error2",
    "timeout_step",
    "timeout_loops",
    "angle_deg",
]


@dataclass(frozen=True)
class EndatSample:
    loop: int
    pos_hi: int
    pos_lo: int
    pos32: int
    position_clocks: int
    error1: int
    error2: int
    timeout_step: int
    timeout_loops: int
    angle_deg: float
    timestamp_s: float

    def as_csv_row(self) -> list[str]:
        return [
            str(self.loop),
            str(self.pos_hi),
            str(self.pos_lo),
            str(self.pos32),
            str(self.position_clocks),
            str(self.error1),
            str(self.error2),
            str(self.timeout_step),
            str(self.timeout_loops),
            f"{self.angle_deg:.9f}",
        ]


class EndatSerialReader:
    """Read raw EnDat CSV rows and expose converted angle samples.

    The C2000 firmware currently streams:
    loop,pos_hi,pos_lo,pos32,position_clocks,error1,error2,timeout_step,timeout_loops
    """

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        counts_per_rev: int = DEFAULT_COUNTS_PER_REV,
        zero_count: int = 0,
        timeout_s: float = 1.0,
    ) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required. Install it with: python -m pip install pyserial")
        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        self.port = port
        self.baud = baud
        self.counts_per_rev = counts_per_rev
        self.zero_count = zero_count
        self.timeout_s = timeout_s
        self._serial = None

    def open(self) -> None:
        if self._serial is None:
            self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
            self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "EndatSerialReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def count_to_angle_deg(self, raw_count: int) -> float:
        angle_deg = ((raw_count - self.zero_count) % self.counts_per_rev) * 360.0
        return angle_deg / self.counts_per_rev

    def parse_line(self, line: str) -> EndatSample | None:
        fields = line.strip().split(",")
        if len(fields) < 9 or not fields[0].isdigit():
            return None

        loop = int(fields[0])
        pos_hi = int(fields[1])
        pos_lo = int(fields[2])
        pos32 = int(fields[3])
        position_clocks = int(fields[4])
        error1 = int(fields[5])
        error2 = int(fields[6])
        timeout_step = int(fields[7])
        timeout_loops = int(fields[8])

        return EndatSample(
            loop=loop,
            pos_hi=pos_hi,
            pos_lo=pos_lo,
            pos32=pos32,
            position_clocks=position_clocks,
            error1=error1,
            error2=error2,
            timeout_step=timeout_step,
            timeout_loops=timeout_loops,
            angle_deg=self.count_to_angle_deg(pos32),
            timestamp_s=monotonic(),
        )

    def read_sample(self) -> EndatSample | None:
        if self._serial is None:
            self.open()

        raw = self._serial.readline()
        if not raw:
            return None

        line = raw.decode("ascii", errors="replace").strip()
        return self.parse_line(line)

    def read_latest(self, max_lines: int = 20) -> EndatSample | None:
        latest = None
        for _ in range(max_lines):
            sample = self.read_sample()
            if sample is not None:
                latest = sample
            elif latest is not None:
                break
        return latest

    def read_latest_available(self, max_lines: int = 50) -> EndatSample | None:
        """Drain already-buffered serial rows without waiting for a new row."""
        if self._serial is None:
            self.open()

        latest = None
        for _ in range(max_lines):
            if self._serial.in_waiting <= 0:
                break
            sample = self.read_sample()
            if sample is not None:
                latest = sample
        return latest

    def read_angle_deg(self) -> float | None:
        sample = self.read_sample()
        if sample is None:
            return None
        return sample.angle_deg
