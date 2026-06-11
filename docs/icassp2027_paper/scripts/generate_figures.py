#!/usr/bin/env python3
"""Generate ICASSP draft figures from fixed held-out summary results."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "figures"


def draw_payload_box(ax, xy, width, height, title, rows, facecolor, edgecolor):
    box = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.035,rounding_size=0.03",
        linewidth=1.1,
        edgecolor=edgecolor,
        facecolor=facecolor,
        zorder=5,
    )
    ax.add_patch(box)
    x, y = xy
    ax.text(x + width / 2, y + height - 0.08, title, ha="center", va="top",
            fontsize=8.4, fontweight="bold", zorder=6)
    for idx, row in enumerate(rows):
        ax.text(x + 0.04, y + height - 0.19 - 0.12 * idx, row, ha="left",
                va="top", fontsize=7.4, zorder=6)


def draw_graph_panel(ax, title, subtitle, mode):
    ax.set_aspect("equal")
    ax.set_xlim(-1.35, 1.35)
    ax.set_ylim(-1.32, 1.42)
    ax.axis("off")
    ax.text(0, 1.31, title, ha="center", va="top", fontsize=10.5,
            fontweight="bold")
    ax.text(0, 1.14, subtitle, ha="center", va="top", fontsize=7.6,
            color="#4b5563")

    coords = []
    for idx in range(8):
        theta = math.pi / 2 - idx * math.tau / 8
        coords.append((0.72 * math.cos(theta), 0.72 * math.sin(theta)))

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
        (0, 5), (1, 4), (2, 7), (3, 6),
    ]
    sparse_missing = {(0, 5), (1, 4), (2, 6), (3, 7), (4, 5)}
    sparse_extra = {(0, 6), (2, 4), (3, 5)}

    if mode == "full":
        edge_color = "#64748b"
        node_color = "#e2e8f0"
        active_edges = set(edges)
    elif mode == "sparse":
        edge_color = "#9ca3af"
        node_color = "#fee2e2"
        active_edges = set(edges) - sparse_missing
    else:
        edge_color = "#2563eb"
        node_color = "#dbeafe"
        active_edges = set(edges)

    for a, b in edges:
        xa, ya = coords[a]
        xb, yb = coords[b]
        if mode == "sparse" and (a, b) in sparse_missing:
            ax.plot([xa, xb], [ya, yb], color="#ef4444", linewidth=1.0,
                    linestyle=(0, (2, 2)), alpha=0.7, zorder=1)
            ax.text((xa + xb) / 2, (ya + yb) / 2, "x", ha="center",
                    va="center", fontsize=8.0, color="#b91c1c", zorder=2)
        elif (a, b) in active_edges:
            ax.plot([xa, xb], [ya, yb], color=edge_color, linewidth=1.25,
                    alpha=0.9, zorder=1)

    if mode == "sparse":
        for a, b in sparse_extra:
            xa, ya = coords[a]
            xb, yb = coords[b]
            ax.plot([xa, xb], [ya, yb], color="#f59e0b", linewidth=1.0,
                    linestyle=(0, (3, 2)), alpha=0.85, zorder=1)

    for idx, (x, y) in enumerate(coords, start=1):
        ax.add_patch(Circle((x, y), 0.095, facecolor=node_color,
                            edgecolor="#334155", linewidth=0.8, zorder=3))
        ax.text(x, y, str(idx), ha="center", va="center", fontsize=7.0,
                color="#111827", zorder=4)

    if mode == "full":
        draw_payload_box(
            ax, (-1.12, -1.20), 2.24, 0.45, "Full GM-LMB label",
            [r"$r_\ell,\{w_m,\mu_m,P_m\}_{m=1}^{M_\ell}$",
             "all mixture components"],
            "#f8fafc", "#64748b")
    elif mode == "sparse":
        ax.text(0, -1.02, "message paths are changed", ha="center",
                va="center", fontsize=8.2, color="#991b1b", fontweight="bold")
        ax.text(0, -1.18, "effective KLA graph can thin out", ha="center",
                va="center", fontsize=7.6, color="#7f1d1d")
    else:
        draw_payload_box(
            ax, (-1.12, -1.20), 2.24, 0.45, "Light LMB label",
            [r"$r_\ell,\bar{\mu}_\ell,\bar{P}_\ell$",
             "single moment-matched Gaussian"],
            "#eff6ff", "#2563eb")


def make_mechanism_figure():
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.25))
    draw_graph_panel(axes[0], "Full posterior", "same graph, large payload", "full")
    draw_graph_panel(axes[1], "Graph sparsification", "fewer or rewired paths", "sparse")
    draw_graph_panel(axes[2], "Light posterior", "same graph, compressed payload", "light")

    for left, right in [(0, 1), (1, 2)]:
        start = axes[left].transAxes.transform((1.00, 0.54))
        end = axes[right].transAxes.transform((0.00, 0.54))
        inv = fig.transFigure.inverted()
        arrow = FancyArrowPatch(
            inv.transform(start), inv.transform(end), transform=fig.transFigure,
            arrowstyle="-|>", mutation_scale=12, color="#6b7280", linewidth=1.0)
        fig.add_artist(arrow)

    fig.tight_layout(rect=(0.01, 0.01, 0.99, 1.0))
    fig.savefig(FIG_DIR / "payload_graph_schematic.pdf", bbox_inches="tight")
    fig.savefig(FIG_DIR / "payload_graph_schematic.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def make_tradeoff_figure():
    arms = [
        {
            "name": "Full-static", "short": "Full\nstatic",
            "bytes_reduction": 0.0, "consensus_delta": 0.0,
            "local_delta": 0.0, "eff_lambda2": 0.373, "pass": "ref.",
            "color": "#64748b",
        },
        {
            "name": "Full-dynamic", "short": "Full\ndynamic",
            "bytes_reduction": -6.1, "consensus_delta": 11.1,
            "local_delta": 4.1, "eff_lambda2": 0.301, "pass": "0/50",
            "color": "#ef4444",
        },
        {
            "name": "Light-static", "short": "Light\nstatic",
            "bytes_reduction": 58.6, "consensus_delta": 0.0,
            "local_delta": 0.0, "eff_lambda2": 0.373, "pass": "50/50",
            "color": "#2563eb",
        },
        {
            "name": "Light-dynamic", "short": "Light\ndynamic",
            "bytes_reduction": 58.3, "consensus_delta": 0.7,
            "local_delta": 0.1, "eff_lambda2": 0.371, "pass": "35/50",
            "color": "#f59e0b",
        },
    ]

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.75))

    ax = axes[0]
    x = list(range(len(arms)))
    bars = ax.bar(x, [a["bytes_reduction"] for a in arms],
                  color=[a["color"] for a in arms], width=0.64)
    ax.axhline(30, color="#16a34a", linewidth=1.0, linestyle="--", alpha=0.75)
    ax.axhline(0, color="#111827", linewidth=0.7)
    ax.text(3.45, 30, "30% gate", ha="right", va="bottom",
            fontsize=7.2, color="#166534")
    ax.set_ylabel("Bytes reduction (%)", fontsize=8.5)
    ax.set_xticks(x)
    ax.set_xticklabels([a["short"] for a in arms], fontsize=7.5)
    ax.set_ylim(-12, 66)
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.7)
    ax.set_title("Payload saving", fontsize=9.5, fontweight="bold")
    for rect, arm in zip(bars, arms):
        y = rect.get_height()
        va = "bottom" if y >= 0 else "top"
        offset = 1.5 if y >= 0 else -1.5
        ax.text(rect.get_x() + rect.get_width() / 2, y + offset,
                f"{y:.1f}%", ha="center", va=va, fontsize=7.0)

    ax = axes[1]
    ax.axvspan(30, 65, ymin=0, ymax=10 / 26, facecolor="#dcfce7",
               alpha=0.65, zorder=0)
    ax.axhline(10, color="#dc2626", linewidth=1.0, linestyle="--", alpha=0.8)
    ax.axvline(30, color="#16a34a", linewidth=1.0, linestyle="--", alpha=0.8)
    for arm in arms:
        size = 72 + 420 * max(arm["eff_lambda2"], 0.0)
        marker = "*" if arm["name"] == "Light-static" else "o"
        ax.scatter(arm["bytes_reduction"], arm["consensus_delta"], s=size,
                   color=arm["color"], edgecolor="white", linewidth=0.9,
                   marker=marker, zorder=3)
        label = arm["name"].replace("-", "\n")
        dx, dy = {
            "Full-static": (2.0, 0.8),
            "Full-dynamic": (1.0, -2.6),
            "Light-static": (-17.5, 1.0),
            "Light-dynamic": (-18.5, 2.6),
        }[arm["name"]]
        ax.text(arm["bytes_reduction"] + dx, arm["consensus_delta"] + dy,
                label, fontsize=6.8, color="#111827")
    ax.text(63, 10, "10% consensus gate", ha="right", va="bottom",
            fontsize=7.0, color="#991b1b")
    ax.text(48, 24.2, "green region: communication\nand consensus gate",
            ha="center", va="top", fontsize=7.0, color="#166534")
    ax.set_xlim(-12, 66)
    ax.set_ylim(-1.5, 26)
    ax.set_xlabel("Bytes reduction (%)", fontsize=8.5)
    ax.set_ylabel("Consensus OSPA change (%)", fontsize=8.5)
    ax.grid(color="#e5e7eb", linewidth=0.7)
    ax.set_title("Held-out trade-off", fontsize=9.5, fontweight="bold")
    ax.text(63, -0.9, r"star: selected; size $\propto\lambda_2^{eff}$",
            ha="right", va="bottom", fontsize=6.8, color="#475569")

    fig.tight_layout(w_pad=1.0)
    fig.savefig(FIG_DIR / "heldout_tradeoff.pdf", bbox_inches="tight")
    fig.savefig(FIG_DIR / "heldout_tradeoff.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def main():
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "mathtext.fontset": "dejavusans",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })
    make_mechanism_figure()
    make_tradeoff_figure()


if __name__ == "__main__":
    main()
