#!/usr/bin/env python3
"""Analyse CSVs written by the apps in this repo.

    python analysis/analyze.py data/P01_20260928_141503_tracker.csv
    python analysis/analyze.py data/P01_..._matching_trials.csv
    python analysis/analyze.py data/P01_..._matching.csv        (raw samples of the task)

Sample files (tracker / raw task) -> per-segment movement metrics + plots:
    duration, path length, mean / peak speed, number of speed peaks
    (sub-movements; more = less smooth), SPARC smoothness (closer to 0 = smoother,
    Balasubramanian et al. 2015), and dimensionless jerk (lower = smoother).
    Segments are the "marker" column for the tracker and "trial" for the task.
Trial files (matching) -> absolute / constant / variable error by direction.

Plots and a summary CSV are written next to the input file.
Needs: numpy, pandas, matplotlib  (pip install -r analysis/requirements.txt)
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:  # plots are optional
    plt = None

PHASES = ["WAIT_HOME", "GUIDE_OUT", "HOLD", "GUIDE_BACK", "RELEASE", "MATCH", "RETURN", "FINISH", "DONE"]
MATCH_PHASE = PHASES.index("MATCH")


# ---------------------------------------------------------------------------
# Signal helpers
# ---------------------------------------------------------------------------
def smooth(x, n):
    """Centered moving average (n samples). Cheap low-pass; good enough at 1 kHz."""
    if n <= 1 or len(x) < n:
        return np.asarray(x, dtype=float)
    k = np.ones(n) / n
    pad = n // 2
    xp = np.pad(np.asarray(x, dtype=float), (pad, n - 1 - pad), mode="edge")
    return np.convolve(xp, k, mode="valid")


def sparc(speed, fs, pad_level=4, fc=10.0, amp_th=0.05):
    """Spectral arc length (Balasubramanian 2015). Returns a negative number;
    closer to 0 = smoother."""
    speed = np.asarray(speed, dtype=float)
    if len(speed) < 8 or np.max(speed) <= 0:
        return np.nan
    nfft = int(2 ** (np.ceil(np.log2(len(speed))) + pad_level))
    f = np.arange(0, fs, fs / nfft)
    mf = np.abs(np.fft.fft(speed, nfft))
    mf = mf / np.max(mf)
    sel = f <= fc
    f_sel, mf_sel = f[sel], mf[sel]
    idx = np.nonzero(mf_sel >= amp_th)[0]
    if len(idx) < 2:
        return np.nan
    f_sel = f_sel[idx[0]: idx[-1] + 1]
    mf_sel = mf_sel[idx[0]: idx[-1] + 1]
    df = np.diff(f_sel) / (f_sel[-1] - f_sel[0])
    return -np.sum(np.sqrt(df ** 2 + np.diff(mf_sel) ** 2))


def count_peaks(speed, min_height):
    """Local maxima above min_height (a crude sub-movement count)."""
    s = np.asarray(speed)
    if len(s) < 3:
        return 0
    peaks = (s[1:-1] > s[:-2]) & (s[1:-1] >= s[2:]) & (s[1:-1] > min_height)
    return int(np.sum(peaks))


def movement_metrics(seg, fs):
    t = seg["t_s"].to_numpy()
    p = seg[["x_mm", "y_mm", "z_mm"]].to_numpy()
    n_smooth = max(1, int(round(fs * 0.02)))  # ~20 ms window
    ps = np.column_stack([smooth(p[:, i], n_smooth) for i in range(3)])
    dt = 1.0 / fs
    v = np.gradient(ps, dt, axis=0)
    v = np.column_stack([smooth(v[:, i], n_smooth) for i in range(3)])
    a = np.column_stack([smooth(np.gradient(v[:, i], dt), n_smooth) for i in range(3)])
    j = np.column_stack([np.gradient(a[:, i], dt) for i in range(3)])
    speed = np.linalg.norm(v, axis=1)
    duration = t[-1] - t[0] if len(t) > 1 else 0.0
    path = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    peak = float(speed.max()) if len(speed) else 0.0
    # Dimensionless jerk: sqrt(0.5 * integral(|jerk|^2) * T^5 / L^2)
    djerk = np.nan
    if duration > 0 and path > 1:
        djerk = float(np.sqrt(0.5 * np.sum(np.sum(j ** 2, axis=1)) * dt * duration ** 5 / path ** 2))
    return {
        "samples": len(seg),
        "duration_s": round(duration, 3),
        "path_length_mm": round(path, 2),
        "straight_line_mm": round(float(np.linalg.norm(p[-1] - p[0])), 2),
        "mean_speed_mm_s": round(path / duration, 2) if duration > 0 else np.nan,
        "peak_speed_mm_s": round(peak, 2),
        "speed_peaks": count_peaks(speed, 0.1 * peak) if peak > 0 else 0,
        "sparc": round(sparc(speed, fs), 3),
        "dimensionless_jerk": round(djerk, 1) if np.isfinite(djerk) else np.nan,
    }, t, speed


# ---------------------------------------------------------------------------
# Analyses
# ---------------------------------------------------------------------------
def analyze_samples(path, df, out_prefix):
    df = df.sort_values("t_s").reset_index(drop=True)
    fs = 1.0 / np.median(np.diff(df["t_s"])) if len(df) > 1 else 1000.0
    print(f"{len(df)} samples, {df['t_s'].iloc[-1] - df['t_s'].iloc[0]:.1f} s, ~{fs:.0f} Hz")

    is_task = df["trial"].max() >= 0 and df["phase"].max() >= 0
    if is_task:
        # Only the ACTIVE part of each trial (participant moving on their own).
        seg_col, sub = "trial", df[df["phase"] == MATCH_PHASE]
        print("Task file: metrics are for the MATCH phase (active movement) of each trial.")
    else:
        seg_col, sub = "marker", df
        print("Tracker file: one segment per marker value (press SPACE while recording to split).")

    rows, traces = [], []
    for seg_id, seg in sub.groupby(seg_col):
        if len(seg) < 10:
            continue
        m, t, speed = movement_metrics(seg, fs)
        label = int(seg_id) + (1 if is_task else 0)  # trials are 1-based everywhere users see them
        rows.append({seg_col: label, **m})
        traces.append((label, t - t[0], speed))
    summary = pd.DataFrame(rows)
    if summary.empty:
        print("No segments long enough to analyse.")
        return
    print()
    print(summary.to_string(index=False))
    out_csv = out_prefix.with_name(out_prefix.name + "_metrics.csv")
    summary.to_csv(out_csv, index=False)
    print(f"\nwrote {out_csv}")

    if plt is None:
        return
    fig = plt.figure(figsize=(12, 5))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    for seg_id, seg in sub.groupby(seg_col):
        # Device frame: x right, y up, z toward user. Plot with y as the vertical axis.
        ax3.plot(seg["x_mm"], seg["z_mm"], seg["y_mm"], lw=0.8)
    ax3.set_xlabel("x right (mm)")
    ax3.set_ylabel("z toward user (mm)")
    ax3.set_zlabel("y up (mm)")
    ax3.set_title("Path")
    ax = fig.add_subplot(1, 2, 2)
    for seg_id, t, speed in traces:
        ax.plot(t, speed, lw=0.8, label=f"{seg_col} {seg_id}")
    ax.set_xlabel("time in segment (s)")
    ax.set_ylabel("speed (mm/s)")
    ax.set_title("Speed profile")
    if len(traces) <= 12:
        ax.legend(fontsize=7)
    fig.tight_layout()
    out_png = out_prefix.with_name(out_prefix.name + "_movement.png")
    fig.savefig(out_png, dpi=130)
    print(f"wrote {out_png}")


def analyze_trials(path, df, out_prefix):
    print(f"{len(df)} trials\n")
    g = df.groupby("direction")
    table = pd.DataFrame({
        "n": g.size(),
        "abs_err_mm_mean": g["abs_err_mm"].mean().round(1),
        "abs_err_mm_sd": g["abs_err_mm"].std().round(1),
        # Constant error: mean signed error vector (bias). Variable error: spread around it.
        "const_err_x": g["err_x_mm"].mean().round(1),
        "const_err_y": g["err_y_mm"].mean().round(1),
        "const_err_z": g["err_z_mm"].mean().round(1),
        "variable_err_mm": g.apply(lambda d: np.sqrt(
            ((d[["err_x_mm", "err_y_mm", "err_z_mm"]] - d[["err_x_mm", "err_y_mm", "err_z_mm"]].mean()) ** 2)
            .sum(axis=1).mean()), include_groups=False).round(1),
        "match_time_s": g["match_time_s"].mean().round(2),
    })
    print(table.to_string())
    print(f"\nOverall mean absolute error: {df['abs_err_mm'].mean():.1f} mm "
          f"(sd {df['abs_err_mm'].std():.1f}), mean match time {df['match_time_s'].mean():.2f} s")
    out_csv = out_prefix.with_name(out_prefix.name + "_by_direction.csv")
    table.to_csv(out_csv)
    print(f"\nwrote {out_csv}")

    if plt is None:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    for ax, (a, b, la, lb) in zip(axes, [("x", "y", "x right", "y up"), ("x", "z", "x right", "z toward user")]):
        ax.scatter(df[f"offset_{a}_mm"], df[f"offset_{b}_mm"], marker="x", c="k", label="target")
        rx = df[f"resp_{a}_mm"] - (df[f"target_{a}_mm"] - df[f"offset_{a}_mm"])
        ry = df[f"resp_{b}_mm"] - (df[f"target_{b}_mm"] - df[f"offset_{b}_mm"])
        ax.scatter(rx, ry, s=18, alpha=0.7, label="response")
        for i in range(len(df)):
            ax.plot([df[f"offset_{a}_mm"].iloc[i], rx.iloc[i]], [df[f"offset_{b}_mm"].iloc[i], ry.iloc[i]],
                    lw=0.5, c="gray")
        ax.scatter([0], [0], marker="o", c="r", label="home")
        ax.set_xlabel(f"{la} (mm, relative to home)")
        ax.set_ylabel(f"{lb} (mm)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=8)
    fig.suptitle("Position matching: targets vs responses")
    fig.tight_layout()
    out_png = out_prefix.with_name(out_prefix.name + "_errors.png")
    fig.savefig(out_png, dpi=130)
    print(f"wrote {out_png}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    args = ap.parse_args()
    df = pd.read_csv(args.csv)
    prefix = args.csv.with_suffix("")
    if "abs_err_mm" in df.columns:
        analyze_trials(args.csv, df, prefix)
    elif {"t_s", "x_mm", "y_mm", "z_mm"} <= set(df.columns):
        analyze_samples(args.csv, df, prefix)
    elif {"timestamp", "x_mm"} <= set(df.columns):
        sys.exit("This is a file from the ORIGINAL PositionTracker (one sample every 5 s) - too sparse to analyse "
                 "movement. Record with bin\\position_tracker.exe instead.")
    else:
        sys.exit(f"Don't recognise the columns in {args.csv}")


if __name__ == "__main__":
    main()
