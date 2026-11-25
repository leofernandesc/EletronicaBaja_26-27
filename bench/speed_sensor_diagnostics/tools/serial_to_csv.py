#!/usr/bin/env python3
"""Display and record ESP32 speed diagnostics with run identifiers in CSV."""

import argparse
import csv
from datetime import datetime, timezone
import re
import select
import sys

import serial


RUN_FIELDS = re.compile(
    r"\[(T[0-7])\].*?\bboot=([0-9a-fA-F]+)\s+run=(\d+)"
    r"(?:\s+up_ms=(\d+))?"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Mirror the diagnostic UART to the terminal and a CSV file."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--output", required=True, help="CSV output path")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--no-input", action="store_true", help="Do not forward terminal commands to the ESP32"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    with serial.Serial(port=None, baudrate=args.baud, timeout=0.1) as port:
        port.port = args.port
        port.dtr = False
        port.rts = False
        port.open()
        with open(args.output, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(
                ["captured_at_utc", "test", "boot_id", "run_id", "uptime_ms", "serial_line"]
            )
            print(f"Logging {args.port} at {args.baud} baud to {args.output}; Ctrl-C stops.")
            try:
                while True:
                    raw = port.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        match = RUN_FIELDS.search(line)
                        fields = match.groups() if match else ("", "", "", "")
                        print(line, flush=True)
                        writer.writerow(
                            [datetime.now(timezone.utc).isoformat(), *fields, line]
                        )
                        output.flush()
                    if not args.no_input and sys.stdin.isatty():
                        ready, _, _ = select.select([sys.stdin], [], [], 0)
                        if ready:
                            command = sys.stdin.readline()
                            if command:
                                port.write(command.encode("utf-8"))
            except KeyboardInterrupt:
                print("\nStopped; CSV closed.")


if __name__ == "__main__":
    main()
