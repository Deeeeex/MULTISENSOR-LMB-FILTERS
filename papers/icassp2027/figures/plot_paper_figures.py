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


def arrow(ax, start, end, color, rad=0, lw=1.0, linestyle="-"):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=6.4,
        linewidth=lw, color=color, shrinkA=3.5, shrinkB=3.5,
        linestyle=linestyle,
        connectionstyle=f"arc3,rad={rad}", zorder=2,
    ))


def method_figure():
    fig, axes = plt.subplots(1, 3, figsize=(6.90, 1.95))
    fig.subplots_adjust(left=0.012, right=0.995, bottom=0.015, top=0.985, wspace=0.14)
    centers = np.array([[-0.88, 0.42], [0.88, 0.42], [0, -0.72]])
    offsets = np.array([[-0.22, -0.10], [0, 0.25], [0.22, -0.10]])
    nodes = np.vstack([c + offsets for c in centers])
    local = [(3*f+j, 3*f+(j+1) % 3) for f in range(3) for j in range(3)]
    ab, bc, ac = [(2, 3), (3, 2)], [(5, 8), (8, 5)], [(0, 6), (6, 0)]
    planned = [local + ab + bc, local + ab + ac]
    delivered = local + ab + ac[:1]
    assert [len(set(edges)) for edges in planned] == [13, 13]
    assert len(set(delivered)) == 12
    assert all(max(sum(v == i for _, v in edges) for i in range(9)) == 2 for edges in planned)
    assert not any(u >= 6 and v < 6 for u, v in delivered)
    grey = "#A4ADB7"

    def pair(ax, edges, color, lw=1.2):
        for u, v in edges:
            arrow(ax, nodes[u], nodes[v], color, rad=0.11, lw=lw)

    def cross(ax, x, y):
        ax.plot([x-0.055, x+0.055], [y-0.055, y+0.055], color=INK, lw=1.0, zorder=5)
        ax.plot([x-0.055, x+0.055], [y+0.055, y-0.055], color=INK, lw=1.0, zorder=5)

    titles = ["a  Retain the feasible tree", "b  Repair the failed branch", "c  Delivery is not guaranteed"]
    subtitles = [r"$t-1$: AB + BC", r"$t$: retain AB; add AC", r"$t$: one C-to-A packet lost"]
    for k, ax in enumerate(axes):
        ax.set(xlim=(-1.38, 1.38), ylim=(-1.13, 1.43), aspect="equal")
        ax.axis("off")
        ax.text(-1.35, 1.38, titles[k], fontsize=8.1, weight="bold", va="top")
        ax.text(-1.35, 1.10, subtitles[k], fontsize=7.2, va="top")
        for f, c in enumerate(centers):
            ax.add_patch(FancyBboxPatch(
                (c[0]-0.34, c[1]-0.23), 0.68, 0.60,
                boxstyle="round,pad=0.015,rounding_size=0.07",
                facecolor="#F4F7FA", edgecolor="#D7DFE5", linewidth=0.65, zorder=0))
            ax.text(c[0], c[1]-0.035, "ABC"[f], ha="center", va="center", fontsize=8.0)
        for u, v in local:
            arrow(ax, nodes[u], nodes[v], MAIN, lw=0.8)
        pair(ax, ab, COLORS["sparse"])
        if k == 0:
            pair(ax, bc, COLORS["sparse"])
            ax.plot(nodes[[0, 6], 0], nodes[[0, 6], 1], color=grey, lw=1, ls=":", zorder=1)
        elif k == 1:
            pair(ax, ac, WEAK, lw=1.7)
            ax.plot(nodes[[5, 8], 0], nodes[[5, 8], 1], color=grey, lw=1, ls="--", zorder=1)
            cross(ax, 0.66, -0.25)
        else:
            arrow(ax, nodes[0], nodes[6], WEAK, rad=0.11, lw=1.7)
            arrow(ax, nodes[6], nodes[0], grey, rad=0.11, lw=1.0, linestyle="--")
            cross(ax, -0.54, -0.18)
        for p in nodes:
            ax.add_patch(Circle(p, 0.042, facecolor="white", edgecolor=MAIN, lw=0.9, zorder=4))
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
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--results-only", action="store_true",
                        help="Leave the unchanged method schematic exports untouched.")
    group.add_argument("--method-only", action="store_true",
                       help="Update the method schematic without redrawing result figures.")
    args = parser.parse_args()
    rows = read_rows()
    if not args.results_only:
        method_figure()
    facts = (json.loads((HERE / "paper_figures_manifest.json").read_text())["sparse_vs_fixed"]
             if args.method_only else results_figure(rows))
    manifest = {
        "backend": f"Python / Matplotlib {matplotlib.__version__}",
        "contract": "FIGURE_CONTRACT.md", "data": "routing_tradeoff_source.csv",
        "count_budget_data": "count_budget_source.csv",
        "figures": {"method_paper": {"inches": [6.90, 1.95],
                                    "role": "illustrative retain-repair-deliver sequence",
                                    "scheduled_messages": [13, 13], "delivered_messages": 12},
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
