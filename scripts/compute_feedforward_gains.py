#!/usr/bin/env python3
"""
identify_drive_ff_vmax.py – estimate kS, kM, kA for micromouse motors
=====================================================================

1.  **Segment the trace** by power-step changes.
2.  **Fit a single mechanical time-constant Tₘ** shared by all segments using the
    exponential-response model  *v(t) ≈ vₘₐₓ·(1 – e^(−t/Tₘ)).*
3.  **Extract one (vₘₐₓ, u_step) pair per segment.**
4.  **Linear least-squares** on those pairs →  *u_step = kS + kM·vₘₐₓ*.

For purely rotational data the script additionally **flips the sign of the
velocity for every second power step** because the test routine alternated the
motor directions (see drive loop in the question).

The output coefficients therefore have consistent sign conventions – positive
*`v`* means *counter-clockwise* (or *forward*) and negative means the opposite.

Usage
-----

    ./identify_drive_ff_vmax.py path/to/log.csv

Generates  **segment_vmax_summary.csv**  and a diagnostic plot  **vmax_fit.png**.

"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Tuple

import numpy as np
import pandas as pd
from scipy.optimize import minimize_scalar
from scipy.stats import linregress
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm

DEG_TO_RAD = np.pi / 180.0

# ────────────────────────────────────────────────────────────────────────────
#  Loading helpers
# ────────────────────────────────────────────────────────────────────────────

def _detect_file_type(df: pd.DataFrame) -> str:
    """Return "linear" or "angular" depending on present columns."""
    if "vMouse" in df or {"vLeft", "vRight", "vMouse"}.intersection(df.columns):
        return "linear"
    if "vRot" in df:
        return "angular"
    raise ValueError("Unrecognised log format – no velocity column found")


def load_data(path: Path) -> Tuple[pd.DataFrame, str]:
    """Load a CSV and return (normalised-DataFrame, kind)."""
    raw = pd.read_csv(path)
    kind = _detect_file_type(raw)

    if kind == "linear":
        df = raw.rename(
            columns={
                "timestamp": "t",
                "powerPctLeft": "u",
                "vMouse": "v_mmps",
                "vLeft": "vL_mmps",
                "vRight": "vR_mmps",
            }
        )
        df["v"] = df["v_mmps"] / 1_000.0  # mm/s → m/s

    else:  # angular
        df = raw.rename(
            columns={
                "timestamp": "t",
                "powerPctLeft": "u",  # left duty is sufficient, sign handled later
                "vRot": "v_degps",
            }
        )
        df["v"] = df["v_degps"] * DEG_TO_RAD  # °/s → rad/s

    required = {"t", "u", "v"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing columns {missing} after renaming – bad file?")

    return df[["t", "u", "v"]].copy(), kind


# ────────────────────────────────────────────────────────────────────────────
#  Segmentation + direction correction for angular logs
# ────────────────────────────────────────────────────────────────────────────

def split_segments(df: pd.DataFrame) -> pd.DataFrame:
    seg_id = (df["u"].diff().fillna(0).abs() > 1e-3).cumsum()
    out = df.copy()
    out["seg"] = seg_id
    return out


def correct_direction_for_rotation(df: pd.DataFrame) -> pd.DataFrame:
    """Flip the sign of *v* for even-numbered segments (0-based)."""
    out = df.copy()                        # keep the original untouched
    flip_next = False                       # True ⇒ flip this eligible segment

    # `sort=False` keeps the original order of appearance
    for _, seg in out.groupby("seg", sort=False):

        # Skip segments that start with u == 0
        if seg["u"].iloc[0] == 0:
            continue

        # Use the segment's **row index**, not the u-values, to select the rows
        if flip_next:
            out.loc[seg.index, "v"] *= -1.0

        flip_next = not flip_next           # toggle for the next eligible segment

    return out


# ────────────────────────────────────────────────────────────────────────────
#  Global exponential response model  v(t) = v_max·(1 – e^(−t/Tₘ))
# ────────────────────────────────────────────────────────────────────────────

def _g_of_t(t: np.ndarray, Tm: float) -> np.ndarray:
    return 1.0 - np.exp(-t / Tm)

def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    """
    Source: https://stackoverflow.com/questions/18926031/how-to-extract-a-subset-of-a-colormap-as-a-new-colormap-in-matplotlib
    """
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap

def fit_global_Tm(
    df: pd.DataFrame,
    kind,
    *,
    save_csv: str = "global_exp_fit_params.csv",
    plot_overlay: bool = False,
    Tm_bounds=(1e-4, 5.0),
):
    """Return (Tₘ_opt, summary_df) – same algorithm as original code."""

    segments: list[Tuple[np.ndarray, np.ndarray]] = []
    seg_ids: list[int] = []
    seg_id_to_u: dict[int, int] = {}
    for sid, seg in df.groupby("seg"):
        if seg["u"].iloc[0] <= 0:
            continue  # ignore idle / braking
        t = seg["t"].to_numpy() - seg["t"].iloc[0]
        v = seg["v"].to_numpy()
        segments.append((t, v))
        seg_ids.append(sid)
        seg_id_to_u[sid] = seg["u"].iloc[0]

    if not segments:
        raise RuntimeError("No usable segments – check your CSV and u-column")

    def total_sse(Tm: float) -> float:
        if Tm <= 0:
            return np.inf
        sse = 0.0
        for t, v in segments:
            g = _g_of_t(t, Tm)
            v_m = np.dot(v, g) / np.dot(g, g)
            sse += np.sum((v - v_m * g) ** 2)
        return sse

    res = minimize_scalar(total_sse, bounds=Tm_bounds, method="bounded")
    Tm_opt = res.x

    rows = []
    for sid, (t, v) in zip(seg_ids, segments):
        g = _g_of_t(t, Tm_opt)
        v_m = np.dot(v, g) / np.dot(g, g)
        ss_res = np.sum((v - v_m * g) ** 2)
        ss_tot = np.sum((v - v.mean()) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan
        rows.append({"seg": sid, "v_max": v_m, "R2": r2, "T_m": Tm_opt})

    summary = pd.DataFrame(rows)
    summary.to_csv(save_csv, index=False)

    if plot_overlay:
        cmap = cm.get_cmap("YlOrRd")
        cmap = truncate_colormap(cmap, minval=0.2, maxval=1.0)

        norm = plt.Normalize(vmin=0, vmax=100)
        sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array(np.linspace(0, 100, 256))
        
        fig, ax = plt.subplots(figsize=(5, 4))

        for seg_id, (t, v) in zip(seg_ids, segments):
            g = _g_of_t(t, Tm_opt)
            v_m = np.dot(v, g) / np.dot(g, g)
            color = cmap(norm(seg_id_to_u[seg_id]))   # map your real PWM % to [0,1]
            ax.plot(t, v, alpha=0.6, color=color, label=f"$P_{{pwm}}={seg_id}\%$")
            ax.plot(t, v_m * g, alpha=1.0, color="k", linewidth=1.2)

        unit = "m/s" if kind == "linear" else "rad/s"
        ax.set_title(f"Global-$T_m$ exponential fit\n$T_m$ ≈ {Tm_opt:.3f} [s]")
        ax.set_xlabel("Elapsed time [s]")
        ax.set_ylabel(f"Velocity [{unit}]")
        ax.grid(True, linewidth=0.3)

        # Color bar setup
        cbar = fig.colorbar(sm, ax=ax, pad=0.02)
        cbar.set_label("PWM duty cycle [%]")

        plt.tight_layout()
        plt.savefig(f"Tm_fit_{kind}.png", dpi=300)
        plt.show()

    return Tm_opt, summary


# ────────────────────────────────────────────────────────────────────────────
#  kM / kS from (u_step, v_max)
# ────────────────────────────────────────────────────────────────────────────

def fit_km_ks_from_vmax(
    df: pd.DataFrame,
    fit_summary: pd.DataFrame,
    kind: str,
    *,
    plot: bool = True,
):
    rows: list[Tuple[float, float]] = []
    for sid, seg in df.groupby("seg"):
        u_step = seg["u"].iloc[0]
        if u_step <= 0:
            continue
        v_row = fit_summary.loc[fit_summary["seg"] == sid]
        if v_row.empty:
            continue
        v_max = v_row["v_max"].iloc[0]
        rows.append((v_max, u_step))

    if len(rows) < 2:
        raise RuntimeError("Need ≥2 power-step segments to fit kS/kM")

    arr = np.asarray(rows)
    v_vals, u_vals = arr[:, 0], arr[:, 1]
    slope, intercept, *_ = linregress(v_vals, u_vals)

    if plot:
        unit = "m/s" if kind == "linear" else "rad/s"
        plt.figure(figsize=(5, 4))
        plt.scatter(v_vals, u_vals, zorder=3)
        x_line = np.linspace(0, v_vals.max() * 1.05, 100)
        plt.plot(x_line, intercept + slope * x_line)
        plt.xlabel(f"$v_{{max}}$ [{unit}]")
        plt.ylabel("$P_{pwm}$ [%]")
        plt.title(f"Linear fit of power step vs speed\n"
                  rf"$K_m \approx {slope:.3f}$ $\left[\frac{{\%}}{{{unit}}}\right]$, $K_s \approx {intercept:.3f}$ $\left[\%\right]$")
        plt.grid(True, linewidth=0.3, zorder=0)
        plt.tight_layout()
        plt.savefig(f"vmax_fit_{kind}.png", dpi=300)
        plt.show()

    return intercept, slope


# ────────────────────────────────────────────────────────────────────────────
#  Plot helper
# ────────────────────────────────────────────────────────────────────────────

def plot_velocity_curves(df: pd.DataFrame, *, kind: str):
    ylabel = "Velocity [m s⁻¹]" if kind == "linear" else "Angular vel [rad s⁻¹]"
    title = "Velocity curves per power step" if kind == "linear" else "Angular-velocity curves per power step"
    fig, ax = plt.subplots(figsize=(7, 4))
    for _, seg in df.groupby("seg"):
        if seg["u"].iloc[0] == 0:
            continue
        t0 = seg["t"].iloc[0]
        ax.plot(seg["t"] - t0, seg["v"], alpha=0.7, label=f"u = {seg['u'].iloc[0]:.1f}%")
    ax.set(xlabel="Time [s]", ylabel=ylabel, title=title)
    ax.grid(True, linewidth=0.3)
    ax.legend(title="Power step", loc="upper left", fontsize="small")
    fig.tight_layout()
    plt.show()


# ────────────────────────────────────────────────────────────────────────────
#  Main
# ────────────────────────────────────────────────────────────────────────────

def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: identify_drive_ff_vmax.py data.csv", file=sys.stderr)
        sys.exit(1)

    df, kind = load_data(Path(sys.argv[1]))
    df = split_segments(df)

    # Flip direction in rotation logs
    if kind == "angular":
        df = correct_direction_for_rotation(df)

    # Visualise raw curves
    plot_velocity_curves(df, kind=kind)

    # 1) Shared-Tm exponential fit
    T_m, fit_summary = fit_global_Tm(df, plot_overlay=True, kind=kind)

    # 2) kS / kM
    kS, kM = fit_km_ks_from_vmax(df, fit_summary, plot=True, kind=kind)

    fit_summary.to_csv("segment_vmax_summary.csv", index=False)

    unit = "m s⁻¹" if kind == "linear" else "rad s⁻¹"
    print("\nEstimated feed-forward coefficients (v_max method)")
    print(f"  Data type : {kind}")
    print(f"  Tm = {T_m:.5f} [s]")
    print(f"  kM = {kM:.3f} [% / ({unit})]")
    print(f"  kS = {kS:.3f} [%]")

    print("\nDebug files written:")
    print("  • segment_vmax_summary.csv")
    print("  • vmax_fit.png")


if __name__ == "__main__":
    main()
