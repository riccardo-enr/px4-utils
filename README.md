# px4-utils

Small MAVLink/pymavlink scripts for checking whether an RC transmitter (e.g.
FrSky Taranis) is actually connected and healthy on a PX4 vehicle, and
whether it's ready to arm -- without touching motors.

All scripts default to `udp:0.0.0.0:14550` and take a connection string as
an optional first argument (e.g. a serial radio telemetry link:
`serial:/dev/ttyUSB0:57600`).

## Why

- `SYS_STATUS`'s RC_RECEIVER health bit is a more reliable "is RC connected"
  signal than watching for `RC_CHANNELS`, which isn't always streamed by
  default.
- If a local PX4 SITL instance is running on the same machine, it can share
  UDP port 14550 with a real, WiFi-connected flight controller. Every script
  here reads raw datagrams and discards anything from `127.0.0.1`/`::1`
  *before* parsing, so a background SITL instance never gets mistaken for
  real hardware.

## Scripts

- `check_rc.py` -- checks the RC_RECEIVER bit in `SYS_STATUS`.
- `check_arm.py` -- triggers PX4's pre-arm checks (`MAV_CMD_RUN_PREARM_CHECKS`)
  and reports failures via `STATUSTEXT`, without arming.
- `watch_arm.py` -- live monitor: armed-state changes, RC channel values,
  and status text, while you do an arm gesture on the transmitter.
- `check_rc_params.py` -- fetches and prints RC/arm-related PX4 parameters
  (e.g. `COM_RC_IN_MODE`) with a plain-language interpretation.
- `wait_for_link.py` -- blocks until a real (non-loopback) heartbeat shows
  up; useful for waiting on a drone to power on / join WiFi.
- `diagnose_all.py` -- runs the RC check, arm check, and param dump back to
  back against the same connection.
- `mav_debug.py` -- shared helpers (source-filtered receive, enum decoding,
  message logging) used by all of the above.

## Usage

```bash
uv run check_rc.py udp:0.0.0.0:14550
uv run diagnose_all.py
```

Every script prints which UDP source address(es) it actually accepted vs.
discarded, so it's always visible whether a result came from a real vehicle
or got filtered out.
