#!/usr/bin/env python3
"""
Checks that an RC transmitter is connected to the flight controller by
reading the RC_RECEIVER bit in MAVLink SYS_STATUS, which PX4 streams by
default -- no message-interval request needed.
"""

import sys
import time
from typing import cast

from mav_debug import (
    log_message,
    print_seen_counts,
    print_source_addresses,
    recv_filtered,
    wait_heartbeat_filtered,
)
from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:0.0.0.0:14550"
TIMEOUT = 5
MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 1 << 16


def main():
    print(f"Connecting to {CONN} ...")
    mav = cast(mavutil.mavfile, mavutil.mavlink_connection(CONN))
    t0 = time.time()
    hb = wait_heartbeat_filtered(mav, timeout=TIMEOUT)
    if hb is None:
        print(f"No HEARTBEAT from a non-loopback source within {TIMEOUT}s "
              "(only local SITL responding?) -- aborting.")
        sys.exit(1)
    print(f"Heartbeat from system {mav.target_system} component {mav.target_component}")

    seen_counts = {}
    msg = None
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        m = recv_filtered(mav, timeout=max(0.0, deadline - time.time()))
        if m is None:
            continue
        t = log_message(m, t0, seen_counts)
        if t == "SYS_STATUS":
            msg = m
            break

    print_seen_counts(seen_counts)
    print_source_addresses(mav)

    if msg is None:
        print("\nNo SYS_STATUS message received -> can't determine RC status.")
        sys.exit(1)

    present_mask = msg.onboard_control_sensors_present
    health_mask = msg.onboard_control_sensors_health
    present = bool(present_mask & MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
    healthy = bool(health_mask & MAV_SYS_STATUS_SENSOR_RC_RECEIVER)

    print(f"\nonboard_control_sensors_present = {hex(present_mask)}")
    print(f"onboard_control_sensors_health  = {hex(health_mask)}")
    print(f"RC_RECEIVER bit = {hex(MAV_SYS_STATUS_SENSOR_RC_RECEIVER)}")
    print(f"RC receiver present={present} healthy={healthy}")
    if present and healthy:
        print("-> RC is connected.")
    else:
        print("-> RC is NOT connected.")
        sys.exit(1)


if __name__ == "__main__":
    main()
