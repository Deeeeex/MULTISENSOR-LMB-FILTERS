"""Draw a cached FoV-boundary example and population summary, not tracking gains."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Rectangle
import numpy as np


def plot(source: Path) -> None:
    example = json.loads((source / "V289_BOUNDARY_EXAMPLE.json").read_text())
    table = np.genfromtxt(source / "V289_FORMATION_SUMMARY.csv", delimiter=",", names=True)
    groups = table[table["group"] > 0]
    theta = example["sensor_heading_rad"]
    rotation = np.array([[np.cos(theta), np.sin(theta)],
                         [-np.sin(theta), np.cos(theta)]])
    mean = rotation @ (np.asarray(example["means"]) - example["sensor_position"])
    covariance = rotation @ np.asarray(example["covariances"]).reshape(2, 2) @ rotation.T
    assert np.atleast_1d(example["weights"]).size == 1
    radius = example["fov_range_m"]
    half = example["fov_half_angle_deg"]
    extent = 2.8 * np.sqrt(np.diag(covariance)).max()
    x = np.linspace(mean[0] - extent, mean[0] + extent, 321)
    y = np.linspace(mean[1] - extent, mean[1] + extent, 321)
    xx, yy = np.meshgrid(x, y)
    delta = np.stack([xx - mean[0], yy - mean[1]], axis=-1)
    squared = np.einsum("...i,ij,...j->...", delta, np.linalg.inv(covariance), delta)

    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 7.5, "axes.labelsize": 7.5, "xtick.labelsize": 7,
        "ytick.labelsize": 7.5, "axes.spines.top": False,
        "axes.spines.right": False, "axes.linewidth": 0.6,
        "svg.fonttype": "none", "pdf.fonttype": 42, "legend.frameon": False,
        "savefig.facecolor": "white",
    })
    teal, gray, purple = "#168578", "#9CA5AF", "#735C92"
    fig = plt.figure(figsize=(178 / 25.4, 82 / 25.4))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.07], left=0.09, right=0.98,
                         bottom=0.19, top=0.82, wspace=0.34)
    a = fig.add_subplot(gs[0, 0]); b = fig.add_subplot(gs[0, 1])
    a.add_patch(Wedge((0, 0), radius, -half, half, facecolor="#E3F0EF",
                      edgecolor=teal, lw=1.0, zorder=0))
    a.contour(xx, yy, squared, levels=[1, 4], colors=[purple, purple],
              linewidths=[1.35, 0.85], linestyles=["solid", "dashed"])
    a.scatter(*mean, marker="+", s=40, linewidth=1.2, color=purple, zorder=4)
    a.set(xlim=(x[0], x[-1]), ylim=(y[0], y[-1]), aspect="equal",
          xlabel="Along sensor heading (m)", ylabel="Cross-heading position (m)")
    a.text(0.97, 0.98, "Inside FoV", color=teal, transform=a.transAxes,
           ha="right", va="top", fontsize=7)
    a.text(0.05, 0.04, "Predicted density", color=purple, transform=a.transAxes,
           fontsize=7)
    a.set_title("Recorded boundary case", fontsize=8, loc="left", pad=20)
    a.text(0, 1.035, f"F1 · sensor {example['sensor']} · step {example['time']} · r = {example['existence']:.3f}",
           transform=a.transAxes, color="#616A73", fontsize=6.8)
    inset = a.inset_axes([0.02, 0.59, 0.28, 0.36])
    inset.add_patch(Wedge((0, 0), radius, -half, half, facecolor="#E3F0EF",
                          edgecolor=teal, lw=0.65))
    inset.add_patch(Rectangle((x[0], y[0]), x[-1]-x[0], y[-1]-y[0],
                              fill=False, edgecolor=purple, lw=0.7))
    inset.scatter([0], [0], color="#46515E", marker=">", s=14, zorder=4)
    inset.scatter(*mean, color=purple, s=4, zorder=4)
    inset.set(xlim=(-25, 340), ylim=(-335, 300), aspect="equal")
    inset.set(xticks=[], yticks=[])
    inset.set_facecolor("white")
    for spine in inset.spines.values():
        spine.set_visible(True)
        spine.set_color("#D9DFE4")
        spine.set_linewidth(0.5)
    inset.text(0.5, 1.01, "120° / 300 m", ha="center", transform=inset.transAxes,
               fontsize=5.8, color="#616A73")

    all_pct = 100 * groups["abs_pd_difference_ge_005_count"] / groups["label_stages"]
    high_pct = 100 * groups["high_r_abs_pd_difference_ge_005_count"] / groups["high_r_count"]
    yy = np.arange(len(groups))
    for i in yy:
        b.plot([all_pct[i], high_pct[i]], [i, i], color="#D9DFE4", lw=1.5, zorder=1)
    b.scatter(all_pct, yy, s=27, color=gray, marker="o", label="All predicted labels", zorder=3)
    b.scatter(high_pct, yy, s=27, color=teal, marker="D", label="Existence r ≥ 0.5", zorder=4)
    for p, v in zip(high_pct, yy):
        b.text(p + 1.1, v - 0.12, f"{p:.1f}%", color=teal, va="bottom", fontsize=7)
    b.set(yticks=yy, yticklabels=[f"F{int(f)}" for f in groups["group"]],
          ylim=(5.5, -0.7), xlim=(0, 42), xticks=[0, 10, 20, 30, 40],
          xlabel="Label stages with |ΔpD| ≥ 0.05 (%)")
    b.grid(axis="x", color="#EBEEF1", lw=0.55); b.set_axisbelow(True)
    b.set_title("Discrepancies across formations", fontsize=8, loc="left", pad=20)
    b.legend(loc="lower left", bbox_to_anchor=(0, 0.985), fontsize=6.8,
             ncol=2, columnspacing=1.0, handletextpad=0.4, borderaxespad=0)
    for ax, letter in ((a, "a"), (b, "b")):
        ax.tick_params(length=2.5, width=0.6)
        ax.text(-0.15, 1.16, letter, transform=ax.transAxes, weight="bold", fontsize=9)
    fig.text(0.09, 0.035,
             f"Example pD: mean approximation {example['mean_pd']:.3f} → density integration {example['density_pd']:.3f}",
             color="#46515E", fontsize=7.3)
    for extension in ("svg", "pdf", "png"):
        fig.savefig(source / f"FOV_DENSITY_DIAGNOSTIC_V289.{extension}", dpi=600)
    plt.close(fig)
    manifest = {
        "type": "cached-density diagnostic, not a tracking arm",
        "backend": "Python matplotlib", "size_mm": [178, 82], "png_dpi": 600,
        "seed": 1301, "time_steps": 40, "sensor_count": 36,
        "label_stage_count": int(table[0]["label_stages"]),
        "example_selection": example["selection"],
        "contours": "Mahalanobis radii 1 and 2; not 68/95 percent 2D regions",
        "coordinates": "sensor-centered, rotated into current heading; no scale distortion",
        "error_bars": "none; dependent label stages from one opened development episode",
        "quadrature": "8192 fixed Halton points; see report for numerical sensitivity",
        "source_data": ["V289_BOUNDARY_EXAMPLE.json", "V289_FORMATION_SUMMARY.csv"],
        "uses_target_truth": False,
    }
    (source / "FIGURE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("V289 diagnostic figure exported; no main result or board replaced.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    plot(parser.parse_args().source)
