#!/usr/bin/env python3
"""Minimal standalone Python CLI for the ESP32 Base Station inside oxebots_comms."""

import argparse
import struct
import sys
import time
import serial

from Adapt import (
    BS_MSG_PING,
    BS_MSG_RF_TX,
    DEFAULT_BAUD,
    encode_frame,
    encode_robot_command,
    find_base_station,
    open_serial_no_reset,
)


def main():
    parser = argparse.ArgumentParser(description="Teste rápido da ESP32 Base Station (oxebots_comms)")
    parser.add_argument("-p", "--port", default=None)
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--vx", type=int, default=0)
    parser.add_argument("--vy", type=int, default=0)
    parser.add_argument("--w", type=int, default=0)
    parser.add_argument("--kick", type=int, default=0)
    args = parser.parse_args()

    if args.port:
        ser = open_serial_no_reset(args.port, baud=args.baud)
    else:
        ser = find_base_station(args.baud)
        if ser is None:
            print("ESP32 Base Station não encontrada.", file=sys.stderr)
            sys.exit(1)

    try:
        ser.write(encode_frame(BS_MSG_PING, struct.pack("<I", 1)))
        ser.flush()
        time.sleep(0.05)
        payload = encode_robot_command(args.id, args.vx, args.vy, args.w, args.kick)
        rf_tx = struct.pack("<HBBB31s", 1, args.id, 0, len(payload), payload.ljust(31, b"\x00"))
        ser.write(encode_frame(BS_MSG_RF_TX, rf_tx))
        ser.flush()
        print(f"Enviado comando para robô {args.id} via {ser.port} @ {args.baud} baud.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
