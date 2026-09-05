"""Render the contracted paper figures from saved experiment summaries.

Run exportPaperFigureData in Octave first when the source episodes change.
This script reads CSVs only: it does not rerun or select tracking policies.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch


HERE = Path(__file__).resolve().parent
COLORS = {"fixed": "#6B7280", "full": "#2474AD", "sparse": "#138571", "self": "#9657A4"}
MARKERS = {"fixed": "s", "full": "^", "sparse": "o", "self": "D"}
LABELS = {"fixed": "Fixed", "full": "Full repair", "sparse": "Sparse repair", "self": "Self fallback"}
INK = "#253142"
SELF = "#D3DAE1"
MAIN = COLORS["full"]
WEAK = "#D99131"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 7.5,
    "axes.labelsize": 7.5, "axes.titlesize": 8.5,
    "axes.linewidth": 0.6, "axes.edgecolor": "#717A84",
    "text.color": INK, "axes.labelcolor": INK,
    "xtick.labelsize": 7, "ytick.labelsize": 7,
    "xtick.color": INK, "ytick.color": INK,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.5, "ytick.major.size": 2.5,
    "svg.fonttype": "none", "pdf.fonttype": 42, "ps.fonttype": 42,
    "savefig.facecolor": "white", "figure.facecolor": "white",
})


def export(fig, stem):
    # Fixed canvas size, not bbox_inches='tight': dimensions are the contract.
    for ext in ("svg", "pdf", "png"):
        fig.savefig(HERE / f"{stem}.{ext}", dpi=600)
    plt.close(fig)


def arrow(ax, start, end, color, rad=0, lw=1.0):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=6.4,
        linewidth=lw, color=color, shrinkA=3.5, shrinkB=3.5,
        connectionstyle=f"arc3,rad={rad}", zorder=2,
    ))


def method_figure():
    fig = plt.figure(figsize=(3.46, 2.65))
    ax = fig.add_axes([0.02, 0.55, 0.96, 0.43])
    ax.set(xlim=(0, 3.28), ylim=(0, 1.16))
    ax.axis("off")
    ax.text(0.01, 1.13, "a  Causal sparse backbone", weight="bold", fontsize=8.4, va="top")
    ax.text(0.04, 0.88, "Current links + previous tree", fontsize=7.0, va="center")
    ax.annotate("retain / repair", xy=(1.72, 0.88), xytext=(2.07, 0.88),
                va="center", fontsize=7.0,
                arrowprops={"arrowstyle": "<-", "lw": 0.8, "color": INK})

    nodes = []
    for f, x in enumerate((0.56, 1.64, 2.72)):
        ax.add_patch(FancyBboxPatch(
            (x - 0.45, 0.17), 0.90, 0.56,
            boxstyle="round,pad=0.015,rounding_size=0.04",
            facecolor="#F3F7FA", edgecolor="#D7DFE5", linewidth=0.7, zorder=0,
        ))
        group = [(x - 0.25, 0.59), (x + 0.25, 0.59), (x, 0.28)]
        nodes.append(group)
        for i in range(3):
            arrow(ax, group[i], group[(i + 1) % 3], MAIN, lw=0.9)
        for p in group:
            ax.add_patch(Circle(p, 0.042, facecolor="white", edgecolor=MAIN, lw=1, zorder=3))
        ax.text(x, 0.065, f"Formation {f + 1}", ha="center", fontsize=6.6)
    for left, right in ((0, 1), (1, 2)):
        arrow(ax, nodes[left][1], nodes[right][0], COLORS["sparse"], rad=-0.06, lw=1.25)
        arrow(ax, nodes[right][2], nodes[left][2], COLORS["sparse"], rad=-0.15, lw=1.25)
    ax.text(1.64, -0.035, r"Local cycles + gateway tree: $N + 2(F-1)$ messages",
            fontsize=7.0, ha="center", va="top")

    bx = fig.add_axes([0.035, 0.045, 0.93, 0.405])
    bx.set(xlim=(0, 1), ylim=(0, 1))
    bx.axis("off")
    bx.text(0, 1.0, "b  A dominant packet is unavailable", fontsize=8.4,
            weight="bold", va="top")
    for x, col, name in ((0.00, SELF, "Self"), (0.29, MAIN, "Dominant"), (0.68, WEAK, "Residual")):
        bx.add_patch(plt.Rectangle((x, 0.735), 0.028, 0.07, color=col, lw=0))
        bx.text(x + 0.041, 0.77, name, fontsize=6.8, va="center")
    rows = [
        ("Planned", [0.25, 0.70, 0.05], 0.56, ["0.25", "0.70", "0.05"]),
        ("Renormalize", [5 / 6, 0, 1 / 6], 0.33, ["0.833", "", "0.167"]),
        ("Self fallback", [0.95, 0, 0.05], 0.10, ["0.95", "", "0.05"]),
    ]
    start, width, height = 0.32, 0.60, 0.15
    for label, weights, y, labels in rows:
        assert np.isclose(sum(weights), 1.0)
        bx.text(0, y + height / 2, label, fontsize=6.9, va="center")
        offset = start
        for i, (w, col) in enumerate(zip(weights, (SELF, MAIN, WEAK))):
            if not w:
                continue
            bx.add_patch(plt.Rectangle((offset, y), width * w, height,
                                       facecolor=col, edgecolor="white", lw=0.5))
            if w >= 0.1:
                bx.text(offset + width * w / 2, y + height / 2, labels[i],
                        color="white" if i == 1 else INK, fontsize=6.2,
                        ha="center", va="center")
            else:
                bx.text(start + width + 0.015, y + height / 2, labels[i],
                        fontsize=6.2, va="center")
            offset += width * w
        if label == "Planned":
            # Cross the lost input; do not remove it from the planned weights.
            cx = start + width * (0.25 + 0.35)
            bx.plot([cx - 0.021, cx + 0.021], [y - 0.01, y + height + 0.01],
                    color=INK, lw=0.75)
            bx.plot([cx + 0.021, cx - 0.021], [y - 0.01, y + height + 0.01],
                    color=INK, lw=0.75)
    export(fig, "method_paper")


def read_rows():
    with (HERE / "routing_tradeoff_source.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 7 and len({(r["scale"], r["policy"]) for r in rows}) == 7
    assert all(r["steps"] == "160" and r["seed"] == "1301" for r in rows)
    return rows


def results_figure(rows):
    fig = plt.figure(figsize=(6.90, 2.18))
    axes = [fig.add_axes(rect) for rect in (
        [0.068, 0.255, 0.267, 0.58],
        [0.415, 0.255, 0.267, 0.58],
        [0.778, 0.255, 0.205, 0.58],
    )]
    offsets = {
        ("M24", "fixed"): (0, 8, "center", "bottom"),
        ("M24", "full"): (0, 9, "center", "bottom"),
        ("M24", "sparse"): (0, 9, "center", "bottom"),
        ("X36", "fixed"): (0, 8, "center", "bottom"),
        ("X36", "full"): (0, 9, "center", "bottom"),
        ("X36", "sparse"): (0, 9, "center", "bottom"),
        ("X36", "self"): (7, -10, "left", "top"),
    }
    facts = {}
    for ax, scale, letter in zip(axes, ("M24", "X36"), ("a", "b")):
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(color="#E7EBEF", lw=0.5, zorder=0)
        ax.set_axisbelow(True)
        ax.set_title(f"{letter}  {scale}", loc="left", pad=7, weight="bold")
        ax.set_xlabel("Attempted posterior\npayload (MB)", labelpad=3)
        ax.set_ylabel("Mean E-OSPA (m)", labelpad=4)
        subset = {r["policy"]: r for r in rows if r["scale"] == scale}
        for policy, row in subset.items():
            x = float(row["attempted_payload_bytes"]) / 1e6
            y = float(row["eospa_m"])
            ax.scatter(x, y, s=32, marker=MARKERS[policy], color=COLORS[policy],
                       edgecolor="white", linewidth=0.5, zorder=3)
            dx, dy, ha, va = offsets[(scale, policy)]
            ax.annotate(f"{LABELS[policy]}\n{y:.3f}", (x, y), xytext=(dx, dy),
                        textcoords="offset points", color=COLORS[policy],
                        fontsize=6.5, ha=ha, va=va, linespacing=1.12)
        if scale == "M24":
            ax.set(xlim=(34.2, 48.2), ylim=(121.9, 126.8),
                   xticks=[35, 40, 45], yticks=[122, 123, 124, 125, 126])
        else:
            ax.set(xlim=(55.3, 87.1), ylim=(131.25, 133.2),
                   xticks=[60, 70, 80], yticks=[131.5, 132.0, 132.5, 133.0])
        b, s = subset["fixed"], subset["sparse"]
        facts[scale] = {
            key + "_sparse_reduction_percent": 100 * (1 - float(s[key]) / float(b[key]))
            for key in ("attempted_payload_bytes", "eospa_m", "conditional_rmse_m", "focus_consistency_m")
        }
    with (HERE / "count_budget_source.csv").open(newline="") as handle:
        budget = {r["scale"]: r for r in csv.DictReader(handle) if r["policy"] == "sparse"}
    assert set(budget) == {"M24", "X36"}
    cx = axes[2]
    cx.spines[["top", "right", "left"]].set_visible(False)
    cx.set_title("c  Fixed-count ceiling", loc="left", pad=7,
                 weight="bold", fontsize=7.4)
    cx.set(xlim=(0, 1.72), ylim=(-0.6, 1.6), yticks=[1, 0],
           yticklabels=["M24", "X36"], xticks=[0, 0.5, 1.0, 1.5])
    cx.tick_params(axis="y", length=0, pad=4)
    cx.set_xlabel("E-OSPA reduction ceiling\nat fixed counts (m)", labelpad=3)
    cx.grid(axis="x", color="#E7EBEF", lw=0.5, zorder=0)
    cx.set_axisbelow(True)
    for scale, y in (("M24", 1), ("X36", 0)):
        row = budget[scale]
        ceiling = float(row["localization_only_headroom_m"])
        cx.barh(y, ceiling, height=0.28, facecolor="#CEDAD7", edgecolor=COLORS["sparse"],
                linewidth=0.8, zorder=2)
        cx.text(ceiling / 2, y + 0.25, rf"$\leq {ceiling:.3f}$ m",
                fontsize=7.0, ha="center", va="bottom", color=INK)
        facts[scale]["localization_only_headroom_upper_bound_m"] = ceiling
    export(fig, "routing_tradeoff")
    return facts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-only", action="store_true",
                        help="Leave the unchanged method schematic exports untouched.")
    args = parser.parse_args()
    rows = read_rows()
    if not args.results_only:
        method_figure()
    facts = results_figure(rows)
    manifest = {
        "backend": f"Python / Matplotlib {matplotlib.__version__}",
        "contract": "FIGURE_CONTRACT.md", "data": "routing_tradeoff_source.csv",
        "count_budget_data": "count_budget_source.csv",
        "figures": {"method_paper": {"inches": [3.46, 2.65]},
                    "routing_tradeoff": {"inches": [6.90, 2.18]}},
        "formats": ["pdf", "svg", "png"], "png_dpi": 600,
        "units": {"payload": "decimal MB, attempted posterior payload only", "eospa": "metres"},
        "sample": "one paired 160-step episode, seed 1301, per scale",
        "uncertainty": "no across-seed confidence interval is available",
        "panel_c": "sparse-arm fixed-count E-OSPA reduction upper bounds, not achieved gains",
        "filter_rerun": False, "independent_validation": False,
        "source_commits": sorted({r["source_commit"] for r in rows}),
        "sparse_vs_fixed": facts,
    }
    (HERE / "paper_figures_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(facts, indent=2))


if __name__ == "__main__":
    main()
