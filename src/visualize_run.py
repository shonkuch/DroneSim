# Comments in English.
import argparse
import json
import math
import os
from typing import Optional, Dict, Any, Tuple

import pandas as pd
import matplotlib.pyplot as plt


def _load_metrics(metrics_path: Optional[str]) -> Optional[Dict[str, Any]]:
    if not metrics_path:
        return None
    if not os.path.exists(metrics_path):
        return None
    with open(metrics_path, "r", encoding="utf-8") as f:
        return json.load(f)


def _get_metrics_block(metrics_json: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not metrics_json:
        return None
    # tolerate both layouts: {"metrics": {...}} or direct {"t_on":...}
    return metrics_json.get("metrics") if isinstance(metrics_json.get("metrics"), dict) else metrics_json


def _ensure_columns(df: pd.DataFrame) -> pd.DataFrame:
    df.columns = [str(c).strip() for c in df.columns]

    # Rename an extra unnamed column if present
    if "servo_time_usec" not in df.columns:
        unnamed = [c for c in df.columns if str(c).lower().startswith("unnamed")]
        if unnamed:
            df = df.rename(columns={unnamed[0]: "servo_time_usec"})

    df["t"] = pd.to_numeric(df["t"], errors="coerce")
    df = df.dropna(subset=["t"]).sort_values("t").reset_index(drop=True)

    # Convert known numeric columns if present
    for c in df.columns:
        if c == "t":
            continue
        df[c] = pd.to_numeric(df[c], errors="ignore")

    # Derive roll_deg if missing and roll_rad exists
    if "roll_deg" not in df.columns and "roll_rad" in df.columns:
        df["roll_deg"] = df["roll_rad"] * (180.0 / math.pi)

    return df


def _apply_step_markers(ax, m: Optional[Dict[str, Any]]):
    if not m:
        return
    t_on = m.get("t_on")
    t_off = m.get("t_off")
    if t_on is not None:
        ax.axvline(float(t_on))
    if t_off is not None:
        ax.axvline(float(t_off))
    if t_on is not None and t_off is not None:
        ax.axvspan(float(t_on), float(t_off), alpha=0.08)


def _slice_window(df: pd.DataFrame, t_on: float, t_off: float) -> pd.DataFrame:
    return df[(df["t"] >= t_on) & (df["t"] <= t_off)].copy()


def plot_attitude(df: pd.DataFrame, m: Optional[Dict[str, Any]], out_path: str):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    if "cmd_roll_deg" in df.columns:
        ax.plot(df["t"], df["cmd_roll_deg"], label="cmd_roll_deg")
    if "roll_deg" in df.columns:
        ax.plot(df["t"], df["roll_deg"], label="roll_deg")

    _apply_step_markers(ax, m)

    ax.set_title("Attitude: command vs measured")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("deg")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_altitude(df: pd.DataFrame, m: Optional[Dict[str, Any]], out_path: str):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    if "alt_m" in df.columns:
        ax.plot(df["t"], df["alt_m"], label="alt_m")
    else:
        ax.text(0.5, 0.5, "alt_m column not found", ha="center", va="center", transform=ax.transAxes)

    _apply_step_markers(ax, m)

    ax.set_title("Altitude over time")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("m")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_servos(df: pd.DataFrame, m: Optional[Dict[str, Any]], out_path: str):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    servo_cols = [c for c in ["servo1", "servo2", "servo3", "servo4"] if c in df.columns]
    if servo_cols:
        for c in servo_cols:
            ax.plot(df["t"], df[c], label=c)
    else:
        ax.text(0.5, 0.5, "servo1..servo4 columns not found", ha="center", va="center", transform=ax.transAxes)

    _apply_step_markers(ax, m)

    ax.set_title("Servo outputs over time")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("raw")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_error(df: pd.DataFrame, m: Optional[Dict[str, Any]], out_path: str):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    if "cmd_roll_deg" not in df.columns or "roll_deg" not in df.columns:
        ax.text(0.5, 0.5, "Need cmd_roll_deg and roll_deg columns", ha="center", va="center", transform=ax.transAxes)
        fig.tight_layout()
        fig.savefig(out_path, dpi=160)
        plt.close(fig)
        return

    df2 = df.copy()
    df2["error_deg"] = df2["cmd_roll_deg"] - df2["roll_deg"]
    ax.plot(df2["t"], df2["error_deg"], label="error_deg = cmd - meas")

    # Settling band (±band around 0)
    band = None
    t_on = t_off = None
    if m:
        band = m.get("settling_band_deg", None)
        t_on = m.get("t_on", None)
        t_off = m.get("t_off", None)

    if band is not None:
        band = float(band)
        ax.axhline(+band, linestyle="--", label=f"+settling_band ({band} deg)")
        ax.axhline(-band, linestyle="--", label=f"-settling_band ({band} deg)")
        ax.axhspan(-band, +band, alpha=0.08)

    # RMS in t_on..t_off window
    rms_txt = "RMS: n/a (no t_on/t_off)"
    if t_on is not None and t_off is not None:
        w = _slice_window(df2, float(t_on), float(t_off))
        if len(w) > 0:
            rms = math.sqrt(float((w["error_deg"] ** 2).mean()))
            rms_txt = f"RMS(t_on..t_off) = {rms:.3f} deg"
        _apply_step_markers(ax, m)

    # Put RMS as a small note on the plot
    ax.text(0.01, 0.99, rms_txt, transform=ax.transAxes, va="top")

    ax.set_title("Roll tracking error")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("deg")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Path to telemetry.csv")
    ap.add_argument("--metrics", default=None, help="Optional path to metrics.json")
    ap.add_argument("--out-dir", required=True, help="Output directory for PNG charts")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    df = pd.read_csv(args.csv)
    df = _ensure_columns(df)

    metrics_json = _load_metrics(args.metrics)
    m = _get_metrics_block(metrics_json)

    plot_attitude(df, m, os.path.join(args.out_dir, "attitude.png"))
    plot_altitude(df, m, os.path.join(args.out_dir, "altitude.png"))
    plot_servos(df, m, os.path.join(args.out_dir, "servos.png"))
    plot_error(df, m, os.path.join(args.out_dir, "error.png"))

    print("OK. Saved:")
    for name in ["attitude.png", "altitude.png", "servos.png", "error.png"]:
        print(os.path.join(args.out_dir, name))


if __name__ == "__main__":
    main()
