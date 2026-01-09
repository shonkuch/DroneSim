# Comments in English as requested.
import argparse
import json
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

from pymavlink import mavutil


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


def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Tuple[float, float, float, float]:
    # ZYX order
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


def start_sitl(ardupilot_dir: str, out_udp: str) -> subprocess.Popen:
    # Using sim_vehicle.py without MAVProxy for clean automation.
    cmd = [
        "python3", "Tools/autotest/sim_vehicle.py",
        "-v", "ArduCopter",
        "-f", "quad",
        "--no-mavproxy",
        "--out", out_udp,
        "--speedup", "1"
    ]
    return subprocess.Popen(cmd, cwd=ardupilot_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def connect_mav(udp: str) -> mavutil.mavfile:
    mav = mavutil.mavlink_connection(udp, autoreconnect=True)
    mav.wait_heartbeat(timeout=90)
    return mav


def set_mode(mav: mavutil.mavfile, mode: str) -> None:
    mav.set_mode(mode)
    # Wait until mode changes (best-effort)
    t0 = time.time()
    while time.time() - t0 < 20:
        msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not msg:
            continue
        # pymavlink has mav.mode_mapping(), but easiest: just break after some heartbeats
        # (ArduPilot usually switches quickly)
        return


def arm(mav: mavutil.mavfile) -> None:
    mav.arducopter_arm()
    mav.motors_armed_wait(timeout=30)


def takeoff(mav: mavutil.mavfile, alt_m: float) -> None:
    # MAV_CMD_NAV_TAKEOFF in GUIDED
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt_m
    )
    # Wait until reached altitude (VFR_HUD.alt is AGL-ish in SITL; good enough for MVP)
    t0 = time.time()
    while time.time() - t0 < 60:
        msg = mav.recv_match(type="VFR_HUD", blocking=True, timeout=1)
        if msg and getattr(msg, "alt", 0) >= alt_m * 0.85:
            return
    raise RuntimeError("Takeoff timeout: altitude not reached.")


def land(mav: mavutil.mavfile) -> None:
    # Simplest: switch to LAND mode
    set_mode(mav, "LAND")


def apply_params(mav: mavutil.mavfile, params: Dict[str, float]) -> None:
    # Fire-and-forget param_set; for MVP we don't block on ACK for each param.
    for k, v in params.items():
        mav.mav.param_set_send(
            mav.target_system,
            mav.target_component,
            k.encode("utf-8"),
            float(v),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.02)


def send_set_attitude_target(
    mav: mavutil.mavfile,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    thrust: float
) -> None:
    # type_mask: ignore body rates, use attitude quaternion + thrust
    # Bits: 1 ignore body roll rate, 2 ignore body pitch rate, 4 ignore body yaw rate
    type_mask = 0b00000111

    q = euler_to_quaternion(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg)
    )

    mav.mav.set_attitude_target_send(
        int((time.time() * 1000) % 0xFFFFFFFF),
        mav.target_system,
        mav.target_component,
        type_mask,
        q,
        0.0, 0.0, 0.0,
        float(thrust)
    )


def run_attitude_segment(
    mav: mavutil.mavfile,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    thrust: float,
    duration_s: float,
    rate_hz: float,
    csv_path: str,
    t_run_start: float
) -> None:
    period = 1.0 / max(1.0, rate_hz)

    # We will log measured attitude + commanded values in one CSV.
    # Pulling last-known messages non-blocking.
    last_att = None
    last_vfr = None
    last_servo = None

    with open(csv_path, "a", encoding="utf-8") as f:
        t0 = time.time()
        while time.time() - t0 < duration_s:
            send_set_attitude_target(mav, roll_deg, pitch_deg, yaw_deg, thrust)

            # Drain messages quickly (non-blocking) to update last_* snapshots
            drain_until = time.time() + 0.02
            while time.time() < drain_until:
                msg = mav.recv_match(blocking=False)
                if not msg:
                    break
                mtype = msg.get_type()
                if mtype == "ATTITUDE":
                    last_att = msg
                elif mtype == "VFR_HUD":
                    last_vfr = msg
                elif mtype == "SERVO_OUTPUT_RAW":
                    last_servo = msg

            t_rel = time.time() - t_run_start

            # Write row even if some messages are missing
            roll_meas = getattr(last_att, "roll", float("nan")) if last_att else float("nan")
            pitch_meas = getattr(last_att, "pitch", float("nan")) if last_att else float("nan")
            yaw_meas = getattr(last_att, "yaw", float("nan")) if last_att else float("nan")
            alt = getattr(last_vfr, "alt", float("nan")) if last_vfr else float("nan")

            # servo1..servo4 (for saturation proxy)
            s1 = getattr(last_servo, "servo1_raw", float("nan")) if last_servo else float("nan")
            s2 = getattr(last_servo, "servo2_raw", float("nan")) if last_servo else float("nan")
            s3 = getattr(last_servo, "servo3_raw", float("nan")) if last_servo else float("nan")
            s4 = getattr(last_servo, "servo4_raw", float("nan")) if last_servo else float("nan")

            f.write(
                f"{t_rel:.3f},{roll_deg:.3f},{pitch_deg:.3f},{yaw_deg:.3f},{thrust:.3f},"
                f"{roll_meas:.6f},{pitch_meas:.6f},{yaw_meas:.6f},{alt:.3f},"
                f"{s1},{s2},{s3},{s4}\n"
            )
            f.flush()

            time.sleep(max(0.0, period - 0.02))


def ensure_csv_header(csv_path: str) -> None:
    if os.path.exists(csv_path) and os.path.getsize(csv_path) > 0:
        return
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(
            "t,cmd_roll_deg,cmd_pitch_deg,cmd_yaw_deg,cmd_thrust,"
            "roll_rad,pitch_rad,yaw_rad,alt_m,"
            "servo1,servo2,servo3,servo4\n"
        )


def load_actions(scenario_path: str) -> Tuple[float, float, List[ScenarioAction]]:
    data = json.load(open(scenario_path, "r", encoding="utf-8"))
    duration = float(data["durationSec"])
    rate_hz = float(data.get("rateHz", 20))
    actions = []
    for a in data["actions"]:
        actions.append(ScenarioAction(**a))
    actions.sort(key=lambda x: x.t)
    return duration, rate_hz, actions


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ardupilot-dir", required=True, help="Path to ardupilot repo root")
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--udp-out", default="udp:127.0.0.1:14550")
    ap.add_argument("--udp-conn", default="udp:127.0.0.1:14550")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Start SITL
    sitl = start_sitl(args.ardupilot_dir, args.udp_out)
    try:
        mav = connect_mav(args.udp_conn)

        # Apply params
        params = json.load(open(args.params, "r", encoding="utf-8"))
        apply_params(mav, params)

        duration, rate_hz, actions = load_actions(args.scenario)
        csv_path = os.path.join(args.out_dir, "telemetry.csv")
        ensure_csv_header(csv_path)

        t_run_start = time.time()
        idx = 0

        # Execute actions by time
        while idx < len(actions):
            now_rel = time.time() - t_run_start
            a = actions[idx]
            if now_rel < a.t:
                time.sleep(0.05)
                continue

            if a.type == "set_mode":
                set_mode(mav, a.mode)
            elif a.type == "arm":
                arm(mav)
            elif a.type == "takeoff":
                takeoff(mav, float(a.altM))
            elif a.type == "set_attitude":
                run_attitude_segment(
                    mav,
                    float(a.rollDeg),
                    float(a.pitchDeg),
                    float(a.yawDeg),
                    float(a.thrust),
                    float(a.durationSec),
                    rate_hz,
                    csv_path,
                    t_run_start
                )
            elif a.type == "land":
                land(mav)
            else:
                raise ValueError(f"Unknown action type: {a.type}")

            idx += 1

        # Small wait to capture landing telemetry
        time.sleep(2)

    finally:
        # Stop SITL
        try:
            sitl.send_signal(signal.SIGINT)
            sitl.wait(timeout=10)
        except Exception:
            try:
                sitl.kill()
            except Exception:
                pass


if __name__ == "__main__":
    main()
