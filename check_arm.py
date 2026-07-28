#!/usr/bin/env python3
"""
Passive arm-readiness check: asks PX4 to run its pre-arm checks and reports
any failures via STATUSTEXT, without actually arming the vehicle.
"""
import sys
import time
from typing import cast
from pymavlink import mavutil

from mav_debug import (
    log_message,
    print_seen_counts,
    print_source_addresses,
    recv_filtered,
    send_command,
    wait_heartbeat_filtered,
)

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:0.0.0.0:14550"
TIMEOUT = 5
WINDOW = 3  # seconds to collect messages after triggering the check
AUTOPILOT_COMPONENT = 1  # ponytail: target this explicitly, heartbeat component was bogus (0)


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

    send_command(
        mav, t0, "MAV_CMD_RUN_PREARM_CHECKS",
        mav.target_system, AUTOPILOT_COMPONENT,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        0, 0, 0, 0, 0, 0, 0,
    )

    print(f"Collecting messages for {WINDOW}s ...")
    seen_counts = {}
    failures = []
    ack_seen = False
    t_end = time.time() + WINDOW
    while time.time() < t_end:
        msg = recv_filtered(mav, timeout=max(0.0, t_end - time.time()))
        if msg is None:
            continue
        t = log_message(msg, t0, seen_counts)
        if t == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS:
            ack_seen = True
        elif t == "STATUSTEXT":
            text = msg.text.strip().lower()
            if "preflight" in text or "prearm" in text or "fail" in text:
                failures.append(msg.text.strip())

    print_seen_counts(seen_counts)
    print_source_addresses(mav)
    print(f"\nCOMMAND_ACK for MAV_CMD_RUN_PREARM_CHECKS seen: {ack_seen}")
    if not ack_seen:
        print("  -> PX4 never acknowledged the request; treat any verdict below as unreliable.")

    if failures:
        print(f"-> NOT ready to arm ({len(failures)} check(s) failed):")
        for f in failures:
            print(f"     {f}")
        sys.exit(1)

    print("-> No pre-arm failures reported. Looks ready to arm.")


if __name__ == "__main__":
    main()
