#!/usr/bin/env python3
"""
Fetches all PX4 parameters and prints the ones related to RC input and
arming, so we see the vehicle's actual config instead of guessing param
names from memory.
"""
import sys
import time
from typing import cast

from mav_debug import print_source_addresses, recv_filtered, wait_heartbeat_filtered
from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:0.0.0.0:14550"
TIMEOUT = 5
KEYWORDS = ("RC_IN", "RC_MAP_ARM", "ARM_SW", "MAN_ARM", "COM_ARM", "COM_RC")

# COM_RC_IN_MODE enum values, PX4 v1.14+ (verify against your firmware's param doc if it looks off)
COM_RC_IN_MODE_MEANING = {
    0: "RC Transmitter -- stick/switch arming and control enabled",
    1: "Joystick/No RC Checks -- RC input largely ignored",
    2: "Virtual RC by Joystick",
    3: "Disabled -- RC input off entirely",
    4: "Generic FMU RC by MAVLink",
}


def main():
    print(f"Connecting to {CONN} ...")
    mav = cast(mavutil.mavfile, mavutil.mavlink_connection(CONN))
    hb = wait_heartbeat_filtered(mav, timeout=TIMEOUT)
    if hb is None:
        print(f"No HEARTBEAT from a non-loopback source within {TIMEOUT}s "
              "(only local SITL responding?) -- aborting.")
        sys.exit(1)
    print(f"Heartbeat from system {mav.target_system} component {mav.target_component}")

    mav.mav.param_request_list_send(mav.target_system, 1)

    print("Fetching parameters (this can take a bit) ...")
    seen = {}
    last_progress = time.time()
    while time.time() - last_progress < 3:
        msg = recv_filtered(mav, timeout=1, msg_type="PARAM_VALUE")
        if msg is None:
            continue
        last_progress = time.time()
        seen[msg.param_id] = msg.param_value
        if len(seen) % 50 == 0:
            print(f"  ... {len(seen)} params so far (last: {msg.param_id}={msg.param_value})")

    print_source_addresses(mav)
    print(f"\nFetched {len(seen)} parameters total. RC/arm-related matches:\n")
    for name in sorted(seen):
        if any(k in name for k in KEYWORDS):
            print(f"  {name} = {seen[name]}")

    print()
    if "COM_RC_IN_MODE" in seen:
        val = int(seen["COM_RC_IN_MODE"])
        meaning = COM_RC_IN_MODE_MEANING.get(val, "unknown value, check firmware param docs")
        print(f"COM_RC_IN_MODE = {val} ({meaning})")
        if val != 0:
            print("  -> This is very likely why stick arming does nothing: "
                  "RC input is not in normal 'RC Transmitter' mode.")
    else:
        print("COM_RC_IN_MODE not found in fetched params "
              "(fetch may have been incomplete -- check the count above against "
              "your firmware's expected param count).")


if __name__ == "__main__":
    main()
