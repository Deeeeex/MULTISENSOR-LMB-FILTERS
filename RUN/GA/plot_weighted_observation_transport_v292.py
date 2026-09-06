"""Plot the registered packet-level diagnostic from its completed source CSV."""
from pathlib import Path
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def main():
    out = Path(__file__).resolve().parent / (
        "dynamic_topology/evidence/tracking_aligned_v292/"
        "weighted_observation_transport_seed1301"
    )
    with (out / "V292_TRANSPORT_METRICS.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    methods = [
        ("Fixed tree", "Fixed tree", "#7c838b", "o"),
        ("Full causal repair", "Full causal repair", "#8173a5", "s"),
        ("Sparse causal repair", "Sparse causal repair", "#237c88", "D"),
    ]
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 7, "svg.fonttype": "none", "pdf.fonttype": 42,
        "axes.spines.right": False, "axes.spines.top": False,
        "axes.linewidth": .6, "legend.frameon": False,
    })
    fig, axes = plt.subplots(1, 2, figsize=(178/25.4, 82/25.4), sharey=True)
    fig.subplots_adjust(left=.092, right=.987, bottom=.20, top=.735, wspace=.19)
    for panel, (ax, count) in enumerate(zip(axes, [24, 36])):
        for key, label, color, marker in methods:
            values = sorted(
                (r for r in rows if int(r["sensors"]) == count and r["arm"] == key),
                key=lambda r: int(r["horizon"]),
            )
            x = [int(r["horizon"]) for r in values]
            assert x == [0, 3, 8, 16], (count, key, x)
            for field, style in [("path_coverage", "--"), ("mean_source_hit", "-")]:
                y = [100*float(r[field]) for r in values]
                ax.plot(x, y, color=color, ls=style, lw=1.35, marker=marker,
                        markersize=3.5, markeredgewidth=.65,
                        markerfacecolor="white" if style == "--" else color)
        ax.set_title(f"{chr(97+panel)}  {'M24' if count == 24 else 'X36'}",
                     loc="left", fontsize=8, fontweight="bold", pad=8)
        ax.set_xlim(-.5, 16.5); ax.set_ylim(0, 100)
        ax.set_xticks([0, 3, 8, 16]); ax.set_yticks([0, 25, 50, 75, 100])
        ax.grid(axis="y", color="#edf0f2", lw=.6, zorder=0)
        ax.tick_params(width=.6, length=2.5)
        ax.set_xlabel("Maximum source age (steps)", labelpad=6)
    axes[0].set_ylabel("Coverage / mean source-hit score (%)", labelpad=7)
    method_handles = [Line2D([], [], color=c, marker=m, lw=1.4, markersize=3.5, label=l)
                      for _, l, c, m in methods]
    fig.legend(handles=method_handles, ncol=3, loc="upper center",
               bbox_to_anchor=(.54, 1.00), columnspacing=2.2, handlelength=2.0)
    kind_handles = [
        Line2D([], [], color="#495361", ls="--", lw=1.3, label="Any geometric source path"),
        Line2D([], [], color="#495361", ls="-", lw=1.3, label="Weight-averaged source-path hit"),
    ]
    fig.legend(handles=kind_handles, ncol=2, loc="upper center",
               bbox_to_anchor=(.54, .916), columnspacing=2.2, handlelength=2.5,
               fontsize=6.8)
    stem = out / "WEIGHTED_OBSERVATION_TRANSPORT_V292"
    for extension in ["svg", "pdf", "png"]:
        fig.savefig(stem.with_suffix("."+extension), dpi=300, facecolor="white")
    plt.close(fig)
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
