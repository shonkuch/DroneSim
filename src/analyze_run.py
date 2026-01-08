# Comments in English.
import argparse
import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Dict, Any, Tuple, Optional, List

import numpy as np
import pandas as pd


@dataclass
class StepMetrics:
    t_on: float
    t_off: float
    cmd_step_deg: float

    rise_time_s: Optional[float]
    overshoot_pct: Optional[float]
    settling_time_s: Optional[float]
    steady_state_err_deg: Optional[float]
    rms_err_deg: Optional[float]

    alt_drift_m: Optional[float]
    alt_drift_mps: Optional[float]

    servo_peak: Dict[str, float]
    servo_std: Dict[str, float]


def read_telemetry_csv_robust(path: str) -> pd.DataFrame:
    """
    Robust CSV reader for telemetry.csv.
    Handles the common case where data rows have 1 extra field (e.g. SERVO_OUTPUT_RAW.time_usec).
    """
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        header_line = f.readline().strip("\n\r")
        if not header_line:
            raise RuntimeError("CSV is empty.")
        base_cols = [c.strip() for c in header_line.split(",")]

        rows: List[List[str]] = []
        extra_seen = False

        for line_no, line in enumerate(f, start=2):
            line = line.strip("\n\r")
            if not line:
                continue
            parts = line.split(",")
            # Most common: 1 extra value at the end
            if len(parts) == len(base_cols) + 1:
                extra_seen = True
                rows.append(parts)  # keep extra as-is
            elif len(parts) == len(base_cols):
                rows.append(parts)
            elif len(parts) > len(base_cols) + 1:
                # keep first N+1, drop the rest
                extra_seen = True
                rows.append(parts[: len(base_cols) + 1])
            else:
                # pad missing
                parts = parts + [""] * (len(base_cols) - len(parts))
                rows.append(parts)

    cols = base_cols + (["servo_time_usec"] if extra_seen else [])
    df = pd.DataFrame(rows, columns=cols)

    # Strip BOM from first column if present
    df.columns = [c.lstrip("\ufeff").strip() for c in df.columns]

    return df


def _find_step_window(df: pd.DataFrame, cmd_col: str = "cmd_roll_deg") -> Tuple[float, float, float]:
    cmd = pd.to_numeric(df[cmd_col], errors="coerce").to_numpy(dtype=float)
    t = pd.to_numeric(df["t"], errors="coerce").to_numpy(dtype=float)

    ok = np.isfinite(cmd) & np.isfinite(t)
    cmd = cmd[ok]
    t = t[ok]

    nz = np.where(np.abs(cmd) > 1e-6)[0]
    if len(nz) == 0:
        # Helpful debug
        mn = float(np.nanmin(cmd)) if len(cmd) else float("nan")
        mx = float(np.nanmax(cmd)) if len(cmd) else float("nan")
        raise RuntimeError(f"No non-zero command segment found in {cmd_col}. cmd range=[{mn}, {mx}]")

    i_on = int(nz[0])
    cmd_step = float(cmd[i_on])

    i_off = None
    for i in range(i_on + 1, len(cmd)):
        if abs(cmd[i]) <= 1e-6:
            i_off = i
            break
    if i_off is None:
        raise RuntimeError("No step-off (back to zero) found in cmd_roll_deg.")

    return float(t[i_on]), float(t[i_off]), cmd_step


def _interp_time_at_level(t: np.ndarray, y: np.ndarray, level: float, t_start: float, t_end: float) -> Optional[float]:
    m = (t >= t_start) & (t <= t_end) & np.isfinite(y) & np.isfinite(t)
    tt = t[m]
    yy = y[m]
    if len(tt) < 2:
        return None

    if level >= 0:
        idx = np.where(yy >= level)[0]
    else:
        idx = np.where(yy <= level)[0]
    if len(idx) == 0:
        return None
    k = int(idx[0])
    if k == 0:
        return float(tt[0])

    t0, t1 = float(tt[k - 1]), float(tt[k])
    y0, y1 = float(yy[k - 1]), float(yy[k])
    if y1 == y0:
        return float(t1)
    alpha = (level - y0) / (y1 - y0)
    return float(t0 + alpha * (t1 - t0))


def _settling_time(t: np.ndarray, y: np.ndarray, target: float, band: float, t_on: float, t_off: float) -> Optional[float]:
    m = (t >= t_on) & (t <= t_off) & np.isfinite(y) & np.isfinite(t)
    tt = t[m]
    yy = y[m]
    if len(tt) < 3:
        return None

    err = np.abs(yy - target)
    ok = err <= band
    if not np.any(ok):
        return None

    suffix_all_ok = np.flip(np.cumprod(np.flip(ok).astype(int))).astype(bool)
    idx = np.where(suffix_all_ok)[0]
    if len(idx) == 0:
        return None
    return float(tt[int(idx[0])] - t_on)


def analyze(csv_path: str) -> Dict[str, Any]:
    df = read_telemetry_csv_robust(csv_path)

    # Normalize numerics
    df["t"] = pd.to_numeric(df["t"], errors="coerce")
    df["cmd_roll_deg"] = pd.to_numeric(df["cmd_roll_deg"], errors="coerce")
    df["roll_rad"] = pd.to_numeric(df["roll_rad"], errors="coerce")
    df["alt_m"] = pd.to_numeric(df["alt_m"], errors="coerce")

    df["roll_deg"] = df["roll_rad"] * (180.0 / math.pi)

    for s in ["servo1", "servo2", "servo3", "servo4"]:
        if s in df.columns:
            df[s] = pd.to_numeric(df[s], errors="coerce")

    t_on, t_off, cmd_step = _find_step_window(df, "cmd_roll_deg")

    m_step = (df["t"] >= t_on) & (df["t"] <= t_off)
    step = df.loc[m_step].copy()

    target = float(cmd_step)
    band_2pct = max(0.2, abs(target) * 0.02)

    rise_time = None
    if abs(target) > 1e-6:
        t10 = _interp_time_at_level(df["t"].to_numpy(), df["roll_deg"].to_numpy(), target * 0.10, t_on, t_off)
        t90 = _interp_time_at_level(df["t"].to_numpy(), df["roll_deg"].to_numpy(), target * 0.90, t_on, t_off)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time = float(t90 - t10)

    overshoot = None
    if abs(target) > 1e-6 and len(step) > 0:
        y = step["roll_deg"].to_numpy(dtype=float)
        if target > 0:
            peak = float(np.nanmax(y))
            overshoot = max(0.0, (peak - target) / abs(target) * 100.0)
        else:
            trough = float(np.nanmin(y))
            overshoot = max(0.0, (target - trough) / abs(target) * 100.0)

    settling = _settling_time(df["t"].to_numpy(), df["roll_deg"].to_numpy(), target, band_2pct, t_on, t_off)

    steady_err = None
    rms_err = None
    if len(step) > 10:
        t_ss0 = t_on + 0.8 * (t_off - t_on)
        ss = step[step["t"] >= t_ss0]
        if len(ss) > 3:
            err = (ss["roll_deg"] - target).to_numpy(dtype=float)
            steady_err = float(np.nanmean(err))
        err_all = (step["roll_deg"] - target).to_numpy(dtype=float)
        rms_err = float(np.sqrt(np.nanmean(err_all ** 2)))

    alt_drift = None
    alt_drift_mps = None
    if len(step) > 3:
        a0 = float(step["alt_m"].iloc[0])
        a1 = float(step["alt_m"].iloc[-1])
        alt_drift = a1 - a0
        dt = float(step["t"].iloc[-1] - step["t"].iloc[0])
        if dt > 1e-6:
            alt_drift_mps = alt_drift / dt

    servo_peak = {}
    servo_std = {}
    for s in ["servo1", "servo2", "servo3", "servo4"]:
        if s not in step.columns:
            servo_peak[s] = float("nan")
            servo_std[s] = float("nan")
            continue
        v = step[s].to_numpy(dtype=float)
        servo_peak[s] = float(np.nanmax(v) - np.nanmin(v)) if np.isfinite(v).any() else float("nan")
        servo_std[s] = float(np.nanstd(v)) if np.isfinite(v).any() else float("nan")

    metrics = StepMetrics(
        t_on=t_on,
        t_off=t_off,
        cmd_step_deg=target,
        rise_time_s=rise_time,
        overshoot_pct=overshoot,
        settling_time_s=settling,
        steady_state_err_deg=steady_err,
        rms_err_deg=rms_err,
        alt_drift_m=alt_drift,
        alt_drift_mps=alt_drift_mps,
        servo_peak=servo_peak,
        servo_std=servo_std,
    )

    # Simple score 0..100
    score = 100.0

    def penalize(value, good_max, weight):
        nonlocal score
        if value is None or (isinstance(value, float) and not np.isfinite(value)):
            score -= weight * 0.5
            return
        score -= weight * max(0.0, (value - good_max) / max(good_max, 1e-6))

    penalize(metrics.overshoot_pct, good_max=2.0, weight=20.0)
    penalize(abs(metrics.steady_state_err_deg) if metrics.steady_state_err_deg is not None else None, good_max=0.3, weight=25.0)
    penalize(metrics.rms_err_deg, good_max=0.6, weight=25.0)
    penalize(abs(metrics.alt_drift_mps) if metrics.alt_drift_mps is not None else None, good_max=0.05, weight=20.0)
    penalize(metrics.settling_time_s, good_max=2.0, weight=10.0)

    score = float(np.clip(score, 0.0, 100.0))

    return {
        "csv": os.path.abspath(csv_path),
        "columns": list(df.columns),
        "metrics": asdict(metrics),
        "score_0_100": score,
        "notes": {
            "settling_band_deg": band_2pct,
            "steady_state_window": "last 20% of step window",
            "hint": "If servo_time_usec exists, your telemetry rows had 1 extra field vs header.",
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    result = analyze(args.csv)
    text = json.dumps(result, indent=2)

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)

    print(text)


if __name__ == "__main__":
    main()
