#!/usr/bin/env python3
"""Download logs from ArduPilot using the MAVLink remote logging backend."""

import argparse
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

START = mavlink.MAV_REMOTE_LOG_DATA_BLOCK_START
STOP = mavlink.MAV_REMOTE_LOG_DATA_BLOCK_STOP
ACK = mavlink.MAV_REMOTE_LOG_DATA_BLOCK_ACK


def main():
    parser = argparse.ArgumentParser(description="Receive MAVLink log stream and save to a file")
    parser.add_argument("port", help="Serial port e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--output", default="remote_log.bin", help="Output file path")

    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.port, baud=args.baud)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    target_sys = master.target_system
    target_comp = master.target_component
    print(f"Heartbeat from system {target_sys} component {target_comp}")

    # initiate log streaming
    master.mav.remote_log_block_status_send(target_sys, target_comp, START, ACK)

    path = Path(args.output)
    print(f"Writing log to {path}")
    with path.open("wb") as f:
        try:
            while True:
                msg = master.recv_match(type="REMOTE_LOG_DATA_BLOCK", blocking=True, timeout=10)
                if msg is None:
                    continue
                f.write(bytes(msg.data))
                master.mav.remote_log_block_status_send(target_sys, target_comp, msg.seqno, ACK)
        except KeyboardInterrupt:
            print("Stopping log download")
        finally:
            master.mav.remote_log_block_status_send(target_sys, target_comp, STOP, ACK)


if __name__ == "__main__":
    main()
