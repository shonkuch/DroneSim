import argparse
import math
import numpy as np
import pandas as pd

def rad_to_deg(x):
    return x * 180.0 / math.pi

def compute_metrics(df: pd.DataFrame):
    # Convert measured roll to degrees
    roll_meas_deg = rad_to_deg(df["roll_rad"].astype(float).to_numpy())
    cmd_roll = df["cmd_roll_deg"].astype(float).to_numpy()
    t = df["t"].astype(float).to_numpy()

    # Consider only times where command is non-zero (step windows)
    mask = np.isfinite(roll_meas_deg) & (np.abs(cmd_roll) > 1e-3)
    if mask.sum() < 10:
        return {"error": "Not enough commanded segment data"}

    t_seg = t[mask]
    y = roll_meas_deg[mask]
    u = cmd_roll[mask]

    target = float(np.median(u))  # step target

    # Overshoot (relative to target sign)
    if target >= 0:
        overshoot = max(0.0, float(np.max(y) - target))
    else:
        overshoot = max(0.0, float(target - np.min(y)))

    # Settling time: first time after which |y-target| <= band for the rest of segment
    band = max(1.0, abs(target) * 0.05)  # 5% or 1deg
    err = np.abs(y - target)
    settling_time = None
    for i in range(len(err)):
        if np.all(err[i:] <= band):
            settling_time = float(t_seg[i] - t_seg[0])
            break
    if settling_time is None:
        settling_time = float("inf")

    # Oscillation score: RMS of roll rate proxy (derivative)
    dy = np.gradient(y, t_seg)
    oscillation_score = float(np.sqrt(np.mean(dy**2)))

    # Saturation proxy: servo outputs near ends
    # Use servo1..servo4 if present
    sat = 0.0
    if "servo1" in df.columns:
        servos = []
        for c in ["servo1","servo2","servo3","servo4"]:
            servos.append(df[c].astype(float).to_numpy())
        servos = np.vstack(servos)
        # Common PWM min/max in ArduPilot
        sat_mask = (servos < 1100) | (servos > 1900)
        sat = float(np.mean(sat_mask))
    saturation_ratio = sat

    # Tracking error (RMSE)
    rmse = float(np.sqrt(np.mean((y - target)**2)))

    # Simple score
    score = 100.0
    score -= min(30.0, overshoot * 2.0)
    score -= min(30.0, rmse * 2.0)
    score -= min(20.0, oscillation_score * 1.0)
    score -= min(20.0, saturation_ratio * 200.0)
    score = float(max(0.0, min(100.0, score)))

    # Verdict rules
    verdict = "PASS"
    if overshoot > 8.0:
        verdict = "FAIL"
    if oscillation_score > 25.0:
        verdict = "FAIL"
    if saturation_ratio > 0.10:
        verdict = "FAIL"

    return {
        "target_roll_deg": target,
        "overshoot_deg": overshoot,
        "settling_time_s": settling_time,
        "rmse_deg": rmse,
        "oscillation_score": oscillation_score,
        "saturation_ratio": saturation_ratio,
        "score": score,
        "verdict": verdict
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    metrics = compute_metrics(df)
    print(metrics)

if __name__ == "__main__":
    main()
