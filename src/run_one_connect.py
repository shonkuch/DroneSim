# Comments in English.
import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict, Any

from pymavlink import mavutil


# -----------------------------
# Data model
# -----------------------------
@dataclass
class ScenarioAction:
    t: float
    type: str
    mode: Optional[str] = None
    altM: Optional[float] = None
    rollDeg: Optional[float] = None
    pitchDeg: Optional[float] = None
    yawDeg: Optional[float] = None
    thrust: Optional[float] = None
    durationSec: Optional[float] = None


# -----------------------------
# Math helpers
# -----------------------------
def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


# -----------------------------
# MAVLink helpers (robust)
# -----------------------------
def now_ms(boot_t0: float) -> int:
    # time_boot_ms should be monotonic-ish since start
    return int((time.time() - boot_t0) * 1000) & 0xFFFFFFFF


def read_statustext_nonblocking(mav: mavutil.mavfile, max_msgs: int = 20) -> List[str]:
    texts = []
    for _ in range(max_msgs):
        msg = mav.recv_match(type="STATUSTEXT", blocking=False)
        if not msg:
            break
        try:
            texts.append(str(msg.text))
        except Exception:
            texts.append(repr(msg))
    return texts


def wait_heartbeat(mav: mavutil.mavfile, timeout_s: float = 60.0) -> None:
    hb = mav.wait_heartbeat(timeout=timeout_s)
    if not hb:
        raise RuntimeError("Heartbeat timeout: no heartbeat received.")


def set_message_interval(mav: mavutil.mavfile, msg_id: int, interval_us: int) -> None:
    # MAV_CMD_SET_MESSAGE_INTERVAL: param1=message id, param2=interval in microseconds
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(msg_id),
        float(interval_us),
        0, 0, 0, 0, 0
    )


def enable_streams(mav: mavutil.mavfile) -> None:
    # 1) Legacy request_data_stream - still useful in SITL.
    try:
        mav.mav.request_data_stream_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # Hz
            1
        )
    except Exception:
        pass

    # 2) Ensure required messages come frequently.
    #    10 Hz = 100_000 us, 20 Hz = 50_000 us.
    try:
        set_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50000)            # 20 Hz
        set_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100000)            # 10 Hz
        set_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000)   # 10 Hz
        set_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000)  # 10 Hz
    except Exception:
        pass


def get_mode_mapping(mav: mavutil.mavfile) -> Dict[str, int]:
    mapping = mav.mode_mapping()
    if not mapping:
        # Fallback for some pymavlink setups
        mapping = getattr(mavutil, "mode_mapping", lambda *_: {})()
    return mapping or {}


def current_mode_name(mav: mavutil.mavfile) -> Optional[str]:
    # Best-effort: decode via mav.mode_mapping()
    hb = mav.recv_match(type="HEARTBEAT", blocking=False)
    if not hb:
        return None
    try:
        mapping = get_mode_mapping(mav)
        inv = {v: k for k, v in mapping.items()}
        # For ArduPilot, custom_mode holds mode id.
        cm = int(getattr(hb, "custom_mode", -1))
        return inv.get(cm)
    except Exception:
        return None


def set_mode(mav: mavutil.mavfile, mode: str, timeout_s: float = 10.0) -> None:
    mode = mode.upper()
    mapping = get_mode_mapping(mav)
    if mode not in mapping:
        raise RuntimeError(f"Mode '{mode}' is not in mode mapping: {sorted(mapping.keys())}")

    # Send
    mav.set_mode(mode)

    # Wait until heartbeat reflects the new mode
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not hb:
            continue
        cm = int(getattr(hb, "custom_mode", -1))
        if cm == mapping[mode]:
            return

    # If failed, print recent STATUSTEXT (often explains why)
    texts = read_statustext_nonblocking(mav, 50)
    hint = "\n".join(texts[-10:]) if texts else "(no STATUSTEXT)"
    raise RuntimeError(f"Failed to switch to mode {mode} within {timeout_s}s.\nSTATUSTEXT:\n{hint}")


def arm(mav: mavutil.mavfile, timeout_s: float = 30.0) -> None:
    mav.arducopter_arm()

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if mav.motors_armed():
            return

        # If there are prearm messages, expose them quickly
        _ = read_statustext_nonblocking(mav, 5)

    texts = read_statustext_nonblocking(mav, 50)
    hint = "\n".join(texts[-10:]) if texts else "(no STATUSTEXT)"
    raise RuntimeError(f"Arming timeout: motors not armed after {timeout_s}s.\nSTATUSTEXT:\n{hint}")


def disarm(mav: mavutil.mavfile, timeout_s: float = 15.0) -> None:
    mav.arducopter_disarm()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not mav.motors_armed():
            return


def takeoff(mav: mavutil.mavfile, alt_m: float, timeout_s: float = 60.0) -> None:
    # ArduPilot expects GUIDED for takeoff
    # MAV_CMD_NAV_TAKEOFF: only altitude used in Copter guided takeoff
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,    # params 1-4 unused
        0, 0,          # lat, lon unused
        float(alt_m)   # altitude
    )

    target_mm = int(alt_m * 1000)
    t0 = time.time()

    last_rel_mm = None
    last_vfr_alt = None

    while time.time() - t0 < timeout_s:
        # Prefer GLOBAL_POSITION_INT.relative_alt (mm above home)
        msg = mav.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD", "STATUSTEXT"], blocking=True, timeout=1)
        if not msg:
            continue

        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            # relative_alt is in millimeters
            last_rel_mm = int(getattr(msg, "relative_alt", 0))
            if last_rel_mm >= int(0.90 * target_mm):
                return

        elif t == "VFR_HUD":
            last_vfr_alt = float(getattr(msg, "alt", 0.0))
            # VFR_HUD alt can be AMSL-ish; still useful as a fallback
            if last_vfr_alt >= alt_m * 0.90:
                return

        elif t == "STATUSTEXT":
            # Keep consuming; useful for debugging
            pass

    texts = read_statustext_nonblocking(mav, 50)
    hint = "\n".join(texts[-10:]) if texts else "(no STATUSTEXT)"
    raise RuntimeError(
        f"Takeoff timeout: altitude not reached after {timeout_s}s.\n"
        f"Last rel_alt_mm={last_rel_mm}, last VFR alt={last_vfr_alt}\n"
        f"STATUSTEXT:\n{hint}"
    )


def land(mav: mavutil.mavfile) -> None:
    set_mode(mav, "LAND", timeout_s=10.0)


def apply_params(mav: mavutil.mavfile, params: Dict[str, Any]) -> None:
    # For MVP: fire param_set; in SITL it's fast enough.
    for k, v in params.items():
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            str(k).encode("utf-8"),
            float(v),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.02)
    time.sleep(0.5)


def send_set_attitude_target(
    mav: mavutil.mavfile,
    boot_t0: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    thrust: float
) -> None:
    # type_mask bits:
    # bit 0-2: ignore body roll/pitch/yaw rates
    # If we want to control attitude quaternion + thrust, ignore body rates.
    type_mask = 0b00000111

    q = euler_to_quaternion(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg)
    )

    mav.mav.set_attitude_target_send(
        now_ms(boot_t0),
        mav.target_system,
        mav.target_component,
        type_mask,
        q,
        0.0, 0.0, 0.0,   # body rates ignored
        float(thrust)
    )


# -----------------------------
# CSV / scenario
# -----------------------------
def ensure_csv_header(path: str) -> None:
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "t,cmd_roll_deg,cmd_pitch_deg,cmd_yaw_deg,cmd_thrust,"
            "roll_rad,pitch_rad,yaw_rad,"
            "rel_alt_m,vfr_alt_m,"
            "servo1,servo2,servo3,servo4\n"
        )


def load_actions(scenario_path: str) -> Tuple[float, float, List[ScenarioAction]]:
    with open(scenario_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    duration = float(data["durationSec"])
    rate_hz = float(data.get("rateHz", 20))
    actions = [ScenarioAction(**a) for a in data["actions"]]
    actions.sort(key=lambda x: x.t)
    return duration, rate_hz, actions


def run_attitude_segment(
    mav: mavutil.mavfile,
    boot_t0: float,
    roll: float,
    pitch: float,
    yaw: float,
    thrust: float,
    duration_s: float,
    rate_hz: float,
    csv_path: str,
    t_run_start: float
) -> None:
    period = 1.0 / max(1.0, rate_hz)

    last_att = None
    last_vfr = None
    last_gpi = None
    last_servo = None

    t0 = time.time()
    with open(csv_path, "a", encoding="utf-8") as f:
        while time.time() - t0 < duration_s:
            send_set_attitude_target(mav, boot_t0, roll, pitch, yaw, thrust)

            # Drain a little to collect the newest telemetry
            drain_until = time.time() + 0.03
            while time.time() < drain_until:
                msg = mav.recv_match(blocking=False)
                if not msg:
                    break
                t = msg.get_type()
                if t == "ATTITUDE":
                    last_att = msg
                elif t == "VFR_HUD":
                    last_vfr = msg
                elif t == "GLOBAL_POSITION_INT":
                    last_gpi = msg
                elif t == "SERVO_OUTPUT_RAW":
                    last_servo = msg

            t_rel = time.time() - t_run_start

            roll_meas = getattr(last_att, "roll", float("nan")) if last_att else float("nan")
            pitch_meas = getattr(last_att, "pitch", float("nan")) if last_att else float("nan")
            yaw_meas = getattr(last_att, "yaw", float("nan")) if last_att else float("nan")

            rel_alt_m = float("nan")
            if last_gpi:
                rel_alt_m = float(getattr(last_gpi, "relative_alt", 0)) / 1000.0

            vfr_alt_m = getattr(last_vfr, "alt", float("nan")) if last_vfr else float("nan")

            s1 = getattr(last_servo, "servo1_raw", float("nan")) if last_servo else float("nan")
            s2 = getattr(last_servo, "servo2_raw", float("nan")) if last_servo else float("nan")
            s3 = getattr(last_servo, "servo3_raw", float("nan")) if last_servo else float("nan")
            s4 = getattr(last_servo, "servo4_raw", float("nan")) if last_servo else float("nan")

            f.write(
                f"{t_rel:.3f},{roll:.3f},{pitch:.3f},{yaw:.3f},{thrust:.3f},"
                f"{roll_meas:.6f},{pitch_meas:.6f},{yaw_meas:.6f},"
                f"{rel_alt_m:.3f},{vfr_alt_m:.3f},"
                f"{s1},{s2},{s3},{s4}\n"
            )
            f.flush()

            time.sleep(max(0.0, period - 0.01))


# -----------------------------
# Main
# -----------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5760")
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "telemetry.csv")
    ensure_csv_header(csv_path)

    # Boot reference for time_boot_ms
    boot_t0 = time.time()

    mav = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    wait_heartbeat(mav, timeout_s=60.0)

    # Ensure we receive altitude/attitude/servos fast enough
    enable_streams(mav)

    # Apply params early
    with open(args.params, "r", encoding="utf-8") as f:
        params = json.load(f)
    apply_params(mav, params)

    _, rate_hz, actions = load_actions(args.scenario)

    t_run_start = time.time()
    i = 0
    while i < len(actions):
        a = actions[i]
        now_rel = time.time() - t_run_start
        if now_rel < a.t:
            time.sleep(0.05)
            continue

        if a.type == "set_mode":
            if not a.mode:
                raise ValueError("set_mode action requires 'mode'")
            set_mode(mav, a.mode, timeout_s=10.0)

        elif a.type == "arm":
            # For attitude control in Copter you usually want GUIDED
            # If scenario did not set it, you can enforce it here (optional).
            # set_mode(mav, "GUIDED", timeout_s=10.0)
            arm(mav, timeout_s=30.0)

        elif a.type == "takeoff":
            if a.altM is None:
                raise ValueError("takeoff action requires 'altM'")
            # Ensure GUIDED before takeoff (common source of silent failures)
            # If scenario already sets GUIDED, this is still fine.
            try:
                set_mode(mav, "GUIDED", timeout_s=10.0)
            except Exception:
                # If already in GUIDED, ignore
                pass
            time.sleep(2.0)
            takeoff(mav, float(a.altM), timeout_s=60.0)

        elif a.type == "set_attitude":
            required = (a.rollDeg, a.pitchDeg, a.yawDeg, a.thrust, a.durationSec)
            if any(x is None for x in required):
                raise ValueError("set_attitude requires rollDeg, pitchDeg, yawDeg, thrust, durationSec")
            run_attitude_segment(
                mav=mav,
                boot_t0=boot_t0,
                roll=float(a.rollDeg),
                pitch=float(a.pitchDeg),
                yaw=float(a.yawDeg),
                thrust=float(a.thrust),
                duration_s=float(a.durationSec),
                rate_hz=float(rate_hz),
                csv_path=csv_path,
                t_run_start=t_run_start
            )

        elif a.type == "land":
            land(mav)

        else:
            raise ValueError(f"Unknown action type: {a.type}")

        i += 1

    time.sleep(2.0)


if __name__ == "__main__":
    main()
