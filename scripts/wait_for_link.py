#!/usr/bin/env python3
"""
Blocks until a real (non-loopback) MAVLink heartbeat appears, ignoring any
local SITL instance sharing the same port. Start this before powering up /
joining the drone's WiFi -- it prints a dot every couple seconds while
waiting, then reports the moment a genuine link comes alive.
"""
import sys
import time
from typing import cast

from mav_debug import print_source_addresses, wait_heartbeat_filtered
from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:0.0.0.0:14550"
POLL_TIMEOUT = 2  # seconds per attempt, so we can print progress dots


def main():
    print(f"Waiting for a real (non-loopback) heartbeat on {CONN} ...")
    mav = cast(mavutil.mavfile, mavutil.mavlink_connection(CONN))
    t0 = time.time()
    while True:
        msg = wait_heartbeat_filtered(mav, timeout=POLL_TIMEOUT)
        if msg is not None:
            break
        print(".", end="", flush=True)

    print(f"\nLink is live after {time.time()-t0:.1f}s: "
          f"system {mav.target_system} component {mav.target_component}")
    print_source_addresses(mav)


if __name__ == "__main__":
    main()
