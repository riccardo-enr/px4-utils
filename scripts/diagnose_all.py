#!/usr/bin/env python3
"""
Runs the RC-link check, arm-readiness check, and RC-param dump back to back
against the same connection, so you get one combined log to read or paste
instead of running three scripts separately.

watch_arm.py isn't included here since it needs you to physically move the
sticks during a live window -- run that one on its own.
"""
import subprocess
import sys
from pathlib import Path

CONN = sys.argv[1] if len(sys.argv) > 1 else "udp:0.0.0.0:14550"
HERE = Path(__file__).parent

STEPS = ["check_rc.py", "check_arm.py", "check_rc_params.py"]


def main():
    for step in STEPS:
        print(f"\n{'=' * 20} {step} {'=' * 20}")
        subprocess.run([sys.executable, str(HERE / step), CONN])


if __name__ == "__main__":
    main()
