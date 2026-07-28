#!/usr/bin/env python3
"""
Live diagnostic for a failed arm attempt: shows whether RC stick movement is
reaching the FC, whether the armed bit ever flips, and any STATUSTEXT PX4
sends when it rejects an arm request. Run this, then do your arm gesture
on the transmitter while it's running.
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
DURATION = 20
AUTOPILOT_COMPONENT = 1
RC_CHANNELS_MSG_ID = 65


def armed(base_mode):
    return bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


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
        mav, t0, "MAV_CMD_SET_MESSAGE_INTERVAL",
        mav.target_system, AUTOPILOT_COMPONENT,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        RC_CHANNELS_MSG_ID, 200000, 0, 0, 0, 0, 0,
    )

    print(f"Watching for {DURATION}s -- move the arm stick/switch now.\n")
    seen_counts = {}
    last_armed = None
    last_chans = None
    ack_seen = False
    t_end = time.time() + DURATION
    while time.time() < t_end:
        msg = recv_filtered(mav, timeout=max(0.0, t_end - time.time()))
        if msg is None:
            continue
        t = log_message(msg, t0, seen_counts)

        if t == "HEARTBEAT":
            last_armed = armed(msg.base_mode)

        elif t == "RC_CHANNELS":
            chans = tuple(getattr(msg, f"chan{i}_raw") for i in range(1, 9))
            if chans != last_chans:
                print(f"  ch1-8 raw: {chans}")
                last_chans = chans

        elif t == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            ack_seen = True

    print_seen_counts(seen_counts)
    print_source_addresses(mav)
    print(f"\nCOMMAND_ACK for MAV_CMD_SET_MESSAGE_INTERVAL seen: {ack_seen}")
    print(f"Final armed state seen: {last_armed}")
    print(f"Last RC ch1-8 raw seen: {last_chans}")
    if last_chans is None:
        print("  -> No RC_CHANNELS message arrived (this stream may just not be forwarded "
              "over this link -- it does NOT by itself mean the FC has no RC signal, "
              "check SYS_STATUS RC_RECEIVER health for that).")
    if last_armed is not True:
        print("  -> Armed bit never went True: arm request is being rejected or never sent.")


if __name__ == "__main__":
    main()
