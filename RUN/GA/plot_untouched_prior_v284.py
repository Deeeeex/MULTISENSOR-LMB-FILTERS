"""Render a completed paired-prefix comparison from numerical CSV exports."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    rows = read_rows(args.directory / "V284_TIME_SERIES.csv")
    metrics = read_rows(args.directory / "V284_JOINT_METRICS.csv")
    assert [int(r["step"]) for r in rows] == list(range(1, 41))
    assert len(metrics) == 5
    changes = np.array([float(r["change_percent"]) for r in metrics])
    assert np.isfinite(changes).all()
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 7.2,
        "axes.labelsize": 7.2, "axes.titlesize": 8.2,
        "xtick.labelsize": 6.8, "ytick.labelsize": 6.8,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.65, "legend.frameon": False,
        "svg.fonttype": "none", "pdf.fonttype": 42,
        "savefig.facecolor": "white", "figure.facecolor": "white",
        "text.color": "#253142", "axes.labelcolor": "#253142",
    })
    figure = plt.figure(figsize=(175 / 25.4, 76 / 25.4))
    ax = figure.add_axes([0.085, 0.205, 0.36, 0.64])
    bx = figure.add_axes([0.70, 0.205, 0.27, 0.64])
    figure.text(0.085, 0.94, "a  Target-set error over time", weight="bold", fontsize=8.2)
    figure.text(0.535, 0.94, "b  Whole-prefix tradeoff", weight="bold", fontsize=8.2)
    t = np.array([int(r["step"]) for r in rows])
    reference = np.array([float(r["reference_eospa"]) for r in rows])
    candidate = np.array([float(r["candidate_eospa"]) for r in rows])
    ax.plot(t, reference, color="#737B86", lw=1.15, ls="--", label="Sparse reference")
    ax.plot(t, candidate, color="#7956A1", lw=1.35, label="Prior exclusion")
    ax.set(xlabel="Time step", ylabel="Network-mean E-OSPA (m)",
           xlim=(1, 40), xticks=[1, 10, 20, 30, 40])
    ax.legend(loc="upper left", bbox_to_anchor=(0, 1.06), fontsize=6.5,
              handlelength=1.8, borderaxespad=0)
    ax.grid(axis="y", color="#E5E8ED", lw=0.5)
    ax.set_axisbelow(True)
    y = np.arange(len(metrics))
    bx.axvline(0, color="#545D69", lw=0.8, zorder=2)
    bx.barh(y, changes, height=0.48, color="#7956A1", zorder=3)
    extent = max(1.0, float(np.max(np.abs(changes))))
    bx.set(xlim=(-1.38 * extent, 1.38 * extent), yticks=y,
           yticklabels=[r["metric"] for r in metrics],
           xlabel="Change vs reference (%)")
    bx.invert_yaxis()
    bx.tick_params(axis="y", length=0, pad=7)
    bx.grid(axis="x", color="#E5E8ED", lw=0.5)
    bx.set_axisbelow(True)
    for i, value in enumerate(changes):
        offset = 0.065 * extent * (1 if value >= 0 else -1)
        bx.text(value + offset, i, f"{value:+.2f}", va="center",
                ha="right" if value < 0 else "left", fontsize=6.6,
                bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.3})
    for suffix in ("svg", "pdf", "png"):
        figure.savefig(args.directory / f"V284_PAIRED_PREFIX.{suffix}", dpi=600)
    plt.close(figure)
    manifest = {
        "backend": f"Python / matplotlib {matplotlib.__version__}",
        "size_mm": [175, 76], "png_dpi": 600,
        "sample": "X36 seed 1301, steps 1-40, 36 sensors; one opened episode",
        "smoothing": "none", "uncertainty": "no independent repeats or interval",
        "rmse": "same finite sensor-time cells, not necessarily same target identities",
        "disagreement": "same six formation representatives; entire prefix",
        "communication": "attempted posterior bytes including lineage metadata and lost attempts",
        "metric_changes_percent": {r["metric"]: float(r["change_percent"]) for r in metrics},
        "paper_insertion": False, "independent_verification": False,
    }
    (args.directory / "V284_FIGURE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest["metric_changes_percent"], indent=2))


if __name__ == "__main__":
    main()
