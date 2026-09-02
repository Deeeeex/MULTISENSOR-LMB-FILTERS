#!/usr/bin/env python3
"""Render the V241 causal-repair timeline as an editable paper figure.

Figure contract
---------------
Core conclusion:
    Causal formation-tree repair changes tracking only after fixed-tree
    failure; it improves E-OSPA and consistency, while the second repair
    exposes a late RMSE trade-off.
Archetype:
    Quantitative grid with one mechanism strip and three aligned outcomes.
Evidence chain:
    (a) executed route messages, (b) E-OSPA, (c) position RMSE,
    (d) inter-formation disagreement.
Backend/export:
    Python/matplotlib only; two-column 7.2-inch SVG/PDF/PNG with editable SVG
    text and a CSV source-data export.
Review risk:
    This is a single opened development seed. Raw traces remain visible under
    the seven-step moving averages; no uncertainty band or significance claim
    is implied.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat


# Mandatory editable-text settings from the selected Python figure workflow.
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = [
    "Arial",
    "DejaVu Sans",
    "Liberation Sans",
]
plt.rcParams["svg.fonttype"] = "none"

mpl.rcParams.update(
    {
        "pdf.fonttype": 42,
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "axes.linewidth": 0.8,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        "legend.frameon": False,
        "savefig.facecolor": "white",
    }
)


BASELINE = "#606060"
CAUSAL = "#0F4D92"
GAIN = "#2E7D32"
LOSS = "#B64342"
EVENT = "#7A5C00"
PHASE_ONE = "#EEF3F9"
PHASE_TWO = "#FFF5E6"


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_input = repo_root / (
        "RUN/GA/dynamic_topology/evidence/tracking_aligned_v241/"
        "formation_braid_routing_comparison/"
        "m24_formation_fov_formation_braid_seed1301/"
        "FORMATION_BRAID_ROUTING_COMPARISON_V241_FULL_EPISODE.mat"
    )
    default_output = repo_root / "RUN/GA/dynamic_topology/figures"
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument("--output-dir", type=Path, default=default_output)
    parser.add_argument(
        "--stem", default="v241_causal_formation_tree_repair_timeline"
    )
    return parser.parse_args()


def as_trace(arm: dict, field: str) -> np.ndarray:
    trace = np.asarray(arm[field], dtype=float).reshape(-1)
    if trace.size != 160 or not np.all(np.isfinite(trace)):
        raise ValueError(f"{field} must be one finite 160-step trace")
    return trace


def centered_mean(values: np.ndarray, width: int = 7) -> np.ndarray:
    if width < 1 or width % 2 == 0:
        raise ValueError("moving-average width must be a positive odd integer")
    pad = width // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, np.ones(width) / width, mode="valid")


def lower_gain(reference: np.ndarray, candidate: np.ndarray, slc: slice) -> float:
    ref = float(np.mean(reference[slc]))
    cand = float(np.mean(candidate[slc]))
    return 100.0 * (ref - cand) / ref


def write_source_data(
    path: Path,
    time: np.ndarray,
    static: dict[str, np.ndarray],
    causal: dict[str, np.ndarray],
    repair_times: np.ndarray,
) -> None:
    fields = [
        "time",
        "repair_event",
        "fixed_route_messages",
        "causal_route_messages",
        "fixed_eospa",
        "causal_eospa",
        "fixed_rmse",
        "causal_rmse",
        "fixed_interformation_disagreement",
        "causal_interformation_disagreement",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    repairs = set(int(value) for value in repair_times)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for idx, current_time in enumerate(time):
            writer.writerow(
                {
                    "time": int(current_time),
                    "repair_event": int(current_time in repairs),
                    "fixed_route_messages": static["messages"][idx],
                    "causal_route_messages": causal["messages"][idx],
                    "fixed_eospa": static["eospa"][idx],
                    "causal_eospa": causal["eospa"][idx],
                    "fixed_rmse": static["rmse"][idx],
                    "causal_rmse": causal["rmse"][idx],
                    "fixed_interformation_disagreement": static["consensus"][idx],
                    "causal_interformation_disagreement": causal["consensus"][idx],
                }
            )


def add_panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.085,
        1.03,
        label,
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=9,
        fontweight="bold",
    )


def add_phase_background(ax: plt.Axes, repair_times: np.ndarray) -> None:
    ax.axvspan(repair_times[0], repair_times[1], color=PHASE_ONE, zorder=0)
    ax.axvspan(repair_times[1], 160.5, color=PHASE_TWO, zorder=0)
    for event in repair_times:
        ax.axvline(event, color=EVENT, lw=0.9, ls=(0, (3, 2)), zorder=2)


def plot_pair(
    ax: plt.Axes,
    time: np.ndarray,
    reference: np.ndarray,
    candidate: np.ndarray,
    ylabel: str,
    repair_times: np.ndarray,
) -> None:
    add_phase_background(ax, repair_times)
    ax.plot(time, reference, color=BASELINE, lw=0.55, alpha=0.28, zorder=1)
    ax.plot(time, candidate, color=CAUSAL, lw=0.55, alpha=0.23, zorder=1)
    ax.plot(
        time,
        centered_mean(reference),
        color=BASELINE,
        lw=1.45,
        label="Fixed formation tree",
        zorder=3,
    )
    ax.plot(
        time,
        centered_mean(candidate),
        color=CAUSAL,
        lw=1.55,
        label="Causal minimal repair",
        zorder=4,
    )
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", color="#D8D8D8", lw=0.45, alpha=0.65)
    ax.margins(x=0)


def annotate_phase_gain(
    ax: plt.Axes,
    x: float,
    gain: float,
    label: str,
) -> None:
    improved = gain >= 0
    direction = "lower" if improved else "higher"
    color = GAIN if improved else LOSS
    ax.text(
        x,
        0.92,
        f"{label}: {abs(gain):.2f}% {direction}",
        transform=ax.get_xaxis_transform(),
        ha="center",
        va="top",
        color=color,
        fontsize=6.7,
        fontweight="bold",
        bbox={
            "boxstyle": "round,pad=0.18",
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.82,
        },
        zorder=6,
    )


def render(input_path: Path, output_dir: Path, stem: str) -> list[Path]:
    envelope = loadmat(input_path, simplify_cells=True)
    result = envelope["result"]
    fixed_arm = result["staticDropout"]
    causal_arm = result["causal"]
    repair_times = np.asarray(causal_arm["treeReselectionTimes"], dtype=int)
    if repair_times.shape != (2,) or not np.array_equal(repair_times, [70, 151]):
        raise ValueError("the V241 repair-event contract changed")

    static = {
        "messages": as_trace(fixed_arm, "routeMessageCountByTime"),
        "eospa": as_trace(fixed_arm, "networkMeanPositionEospaByTime"),
        "rmse": as_trace(fixed_arm, "networkMeanPositionRmseByTime"),
        "consensus": as_trace(fixed_arm, "interFormationPositionOspaByTime"),
    }
    causal = {
        "messages": as_trace(causal_arm, "routeMessageCountByTime"),
        "eospa": as_trace(causal_arm, "networkMeanPositionEospaByTime"),
        "rmse": as_trace(causal_arm, "networkMeanPositionRmseByTime"),
        "consensus": as_trace(causal_arm, "interFormationPositionOspaByTime"),
    }
    time = np.arange(1, 161)
    output_dir.mkdir(parents=True, exist_ok=True)
    source_path = output_dir / "source" / f"{stem}.csv"
    write_source_data(source_path, time, static, causal, repair_times)

    fig = plt.figure(figsize=(7.2, 6.0))
    grid = fig.add_gridspec(
        4,
        1,
        height_ratios=[0.62, 1.35, 1.35, 1.35],
        hspace=0.22,
        left=0.105,
        right=0.985,
        top=0.925,
        bottom=0.09,
    )
    axes = [fig.add_subplot(grid[idx, 0]) for idx in range(4)]

    ax = axes[0]
    add_phase_background(ax, repair_times)
    ax.step(
        time,
        static["messages"],
        where="mid",
        color=BASELINE,
        lw=1.25,
        label="Fixed formation tree",
        zorder=3,
    )
    ax.step(
        time,
        causal["messages"],
        where="mid",
        color=CAUSAL,
        lw=1.45,
        label="Causal minimal repair",
        zorder=4,
    )
    ax.set_ylim(45.65, 48.35)
    ax.set_yticks([46, 47, 48])
    ax.set_ylabel("Messages\nper round")
    ax.grid(axis="y", color="#D8D8D8", lw=0.45, alpha=0.65)
    ax.margins(x=0)
    add_panel_label(ax, "a")
    ax.text(
        repair_times[0],
        1.08,
        "repair 1",
        transform=ax.get_xaxis_transform(),
        ha="center",
        va="bottom",
        color=EVENT,
        fontsize=6.8,
        fontweight="bold",
    )
    ax.text(
        repair_times[1],
        1.08,
        "repair 2",
        transform=ax.get_xaxis_transform(),
        ha="center",
        va="bottom",
        color=EVENT,
        fontsize=6.8,
        fontweight="bold",
    )

    metric_specs = [
        ("eospa", "Position E-OSPA", "b"),
        ("rmse", "Position RMSE", "c"),
        ("consensus", "Inter-formation\ndisagreement", "d"),
    ]
    phase_slices = [slice(69, 150), slice(150, 160)]
    phase_centres = [(70 + 150) / 2, (151 + 160) / 2]
    for ax, (key, ylabel, panel) in zip(axes[1:], metric_specs):
        plot_pair(
            ax,
            time,
            static[key],
            causal[key],
            ylabel,
            repair_times,
        )
        add_panel_label(ax, panel)
        for phase_idx, (phase_slice, centre) in enumerate(
            zip(phase_slices, phase_centres), start=1
        ):
            gain = lower_gain(static[key], causal[key], phase_slice)
            annotate_phase_gain(ax, centre, gain, f"P{phase_idx}")

    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    axes[-1].set_xlabel("Time step")
    axes[-1].set_xlim(1, 160)
    axes[-1].set_xticks([1, 40, 70, 100, 130, 151, 160])

    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.51, 0.995),
        ncol=2,
        handlelength=2.8,
        columnspacing=1.8,
    )
    fig.text(
        0.985,
        0.012,
        "Raw traces (thin) and 7-step moving means (solid); lower is better",
        ha="right",
        va="bottom",
        color="#666666",
        fontsize=6.3,
    )

    outputs = [
        output_dir / f"{stem}.svg",
        output_dir / f"{stem}.pdf",
        output_dir / f"{stem}.png",
        source_path,
    ]
    fig.savefig(outputs[0], bbox_inches="tight")
    fig.savefig(outputs[1], bbox_inches="tight")
    fig.savefig(outputs[2], dpi=360, bbox_inches="tight")
    plt.close(fig)
    return outputs


def main() -> None:
    args = parse_args()
    for output in render(args.input, args.output_dir, args.stem):
        print(output)


if __name__ == "__main__":
    main()
