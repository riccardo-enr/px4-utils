#!/usr/bin/env python3
"""
Shared debug helpers for the check_rc / check_arm / watch_arm / check_rc_params
scripts: decode MAVLink enums to readable names and log every message that
comes over the link, not just the ones a given script cares about.
"""
import select
import time

from pymavlink import mavutil

EXCLUDE_HOSTS_DEFAULT = ("127.0.0.1", "::1")


def mav_result_name(result):
    try:
        return mavutil.mavlink.enums["MAV_RESULT"][result].name
    except KeyError:
        return f"UNKNOWN({result})"


def severity_name(sev):
    try:
        return mavutil.mavlink.enums["MAV_SEVERITY"][sev].name
    except KeyError:
        return f"UNKNOWN({sev})"


def send_command(mav, t0, command_name, target_system, target_component, command_id, *params):
    print(f"[{time.time()-t0:6.2f}s] SEND COMMAND_LONG {command_name} "
          f"target=({target_system},{target_component}) params={params}")
    mav.mav.command_long_send(target_system, target_component, command_id, 0, *params)


def recv_filtered(mav, timeout: float = 1, msg_type=None, exclude_hosts=EXCLUDE_HOSTS_DEFAULT):
    """Like mav.recv_match(type=msg_type, blocking=True, timeout=timeout), but
    reads raw UDP datagrams directly and drops any whose source host is in
    exclude_hosts BEFORE parsing -- so a local SITL instance sharing the same
    port never contaminates results from a real, WiFi-connected vehicle.
    Falls back to plain recv_match on non-UDP-server connections (e.g. serial),
    where there's only one possible source anyway."""
    if not hasattr(mav, "port") or not hasattr(mav, "clients"):
        return mav.recv_match(type=msg_type, blocking=True, timeout=timeout)

    queue = getattr(mav, "_filtered_queue", None)
    if queue is None:
        queue = []
        mav._filtered_queue = queue
    accepted = getattr(mav, "_accepted_sources", None)
    if accepted is None:
        accepted = set()
        mav._accepted_sources = accepted
    rejected = getattr(mav, "_rejected_sources", None)
    if rejected is None:
        rejected = set()
        mav._rejected_sources = rejected

    deadline = time.time() + timeout
    while True:
        for i, m in enumerate(queue):
            if msg_type is None or m.get_type() == msg_type:
                return queue.pop(i)
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        r, _, _ = select.select([mav.port], [], [], remaining)
        if not r:
            return None
        try:
            data, addr = mav.port.recvfrom(65535)
        except OSError:
            continue
        if addr[0] in exclude_hosts:
            rejected.add(addr)
            continue
        accepted.add(addr)
        queue.extend(mav.mav.parse_buffer(data) or [])


def wait_heartbeat_filtered(mav, timeout=5, exclude_hosts=EXCLUDE_HOSTS_DEFAULT):
    """wait_heartbeat() that ignores HEARTBEATs from excluded hosts (see
    recv_filtered). Sets mav.target_system/target_component from the
    accepted heartbeat, same as the real wait_heartbeat does."""
    msg = recv_filtered(mav, timeout=timeout, msg_type="HEARTBEAT", exclude_hosts=exclude_hosts)
    if msg is not None:
        mav.target_system = msg.get_srcSystem()
        mav.target_component = msg.get_srcComponent()
    return msg


def log_message(msg, t0, seen_counts=None):
    """Print every message received, decoding the interesting ones. Returns the type string."""
    t = msg.get_type()
    if seen_counts is not None:
        seen_counts[t] = seen_counts.get(t, 0) + 1

    if t == "COMMAND_ACK":
        print(f"[{time.time()-t0:6.2f}s] COMMAND_ACK command={msg.command} "
              f"result={mav_result_name(msg.result)}")
    elif t == "STATUSTEXT":
        print(f"[{time.time()-t0:6.2f}s] STATUSTEXT [{severity_name(msg.severity)}] "
              f"{msg.text.strip()}")
    elif t == "HEARTBEAT":
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[{time.time()-t0:6.2f}s] HEARTBEAT sysid/compid armed={armed} "
              f"base_mode={msg.base_mode} custom_mode={msg.custom_mode} "
              f"system_status={msg.system_status}")
    else:
        print(f"[{time.time()-t0:6.2f}s] {t}: {msg.to_dict()}")

    return t


def print_seen_counts(seen_counts):
    print("\nMessage types seen and counts:")
    for name in sorted(seen_counts):
        print(f"  {name}: {seen_counts[name]}")


def print_source_addresses(mav):
    """Print which UDP source(s) were actually used vs. discarded, when
    recv_filtered/wait_heartbeat_filtered were used to exclude local SITL.
    Falls back to the raw (unfiltered) client set for plain recv_match use."""
    accepted = getattr(mav, "_accepted_sources", None)
    rejected = getattr(mav, "_rejected_sources", None)
    if accepted is not None or rejected is not None:
        print(f"\nAccepted source address(es): {accepted or set()}")
        if rejected:
            print(f"Discarded (excluded) source address(es): {rejected}")
        if not accepted:
            print("  -> No accepted source ever seen -- only excluded hosts "
                  "(e.g. local SITL) sent anything on this port.")
        return

    clients = getattr(mav, "clients", None)
    if not clients:
        print("\n(no UDP source-address info available for this connection type)")
        return
    print(f"\nUDP source address(es) seen: {clients}")
    if len(clients) > 1:
        print("  -> MULTIPLE sources on this port -- results above may be a mix "
              "of devices (e.g. local SITL + a real WiFi-connected FC). Not reliable "
              "as a single-vehicle check.")
    elif any(addr[0] in ("127.0.0.1", "::1") for addr in clients):
        print("  -> Only source is loopback -- this is local SITL, not real hardware.")
