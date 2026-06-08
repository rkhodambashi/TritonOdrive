"""Read EnDat position CSV data from the F28379D LaunchPad serial port.

Usage:
    python read_endat_serial.py COM7
    python read_endat_serial.py COM7 --output endat_capture.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from endat_serial_reader import (
    COMMON_BAUD_RATES,
    CSV_HEADER,
    DEFAULT_BAUD,
    DEFAULT_COUNTS_PER_REV,
    EndatSerialReader,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required. Install it with: python -m pip install pyserial")
    raise SystemExit(1)


def _print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument(
        "--counts-per-rev",
        type=int,
        default=DEFAULT_COUNTS_PER_REV,
        help="Encoder counts per mechanical revolution. ECN425 capture reports 25 bits by default.",
    )
    parser.add_argument(
        "--zero-count",
        type=int,
        default=0,
        help="Raw count that should be treated as 0 degrees.",
    )
    parser.add_argument(
        "--scan-baud",
        action="store_true",
        help="Try common baud rates and print any received bytes",
    )
    parser.add_argument("--output", type=Path, help="Optional CSV file to write")
    parser.add_argument(
        "--rate-stats",
        action="store_true",
        help="Print received sample rate once per second.",
    )
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    args = parser.parse_args()

    if args.list or not args.port:
        _print_ports()
        if not args.port:
            print("\nRun again with the XDS100 serial port, for example: python read_endat_serial.py COM7")
        return 0

    if args.scan_baud:
        for baud in COMMON_BAUD_RATES:
            print(f"\n--- trying {baud} baud for 3 seconds ---")
            try:
                with serial.Serial(args.port, baud, timeout=0.2) as ser:
                    ser.reset_input_buffer()
                    end_time = time.monotonic() + 3.0
                    while time.monotonic() < end_time:
                        raw = ser.read(256)
                        if raw:
                            text = raw.decode("ascii", errors="replace")
                            print(text.encode("ascii", errors="replace").decode("ascii"), end="")
            except serial.SerialException as exc:
                print(f"Could not open {args.port} at {baud}: {exc}")
                return 1
        return 0

    output_file = None
    writer = None
    try:
        if args.output:
            output_file = args.output.open("w", newline="", encoding="utf-8", errors="replace")
            writer = csv.writer(output_file)
            writer.writerow(CSV_HEADER)
            output_file.flush()

        with EndatSerialReader(
            args.port,
            baud=args.baud,
            counts_per_rev=args.counts_per_rev,
            zero_count=args.zero_count,
            timeout_s=1.0,
        ) as reader:
            print(f"Listening on {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
            print(",".join(CSV_HEADER))
            rate_window_start = time.monotonic()
            rate_window_count = 0
            while True:
                sample = reader.read_sample()
                if sample is None:
                    continue

                rate_window_count += 1
                now = time.monotonic()
                if args.rate_stats and now - rate_window_start >= 1.0:
                    rate_hz = rate_window_count / (now - rate_window_start)
                    print(f"# rate_hz={rate_hz:.1f}")
                    rate_window_start = now
                    rate_window_count = 0

                fields = sample.as_csv_row()
                print(",".join(fields))
                if writer:
                    writer.writerow(fields)
                    output_file.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if output_file:
            output_file.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
