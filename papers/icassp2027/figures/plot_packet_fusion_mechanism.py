"""Analytic packet-to-weight-to-Bernoulli mechanism; no tracking samples."""
from pathlib import Path
import csv
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch, FancyArrowPatch


def main():
    here = Path(__file__).resolve().parent
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 7, "svg.fonttype": "none", "pdf.fonttype": 42,
        "axes.spines.right": False, "axes.spines.top": False,
        "axes.linewidth": .6, "legend.frameon": False,
    })
    colors = ["#8395a4", "#e0e4e8", "#b48043"]
    rule_colors = ["#237c88", "#9165a8"]
    rows = [("Planned", [.25, .70, .05]),
            ("Renormalize", [5/6, 0, 1/6]),
            ("Self fallback", [.95, 0, .05])]
    fig = plt.figure(figsize=(178/25.4, 48/25.4))
    ax = fig.add_axes([.157, .23, .287, .55])
    bx = fig.add_axes([.589, .23, .397, .55])
    fig.text(.026, .94, "a  A missing packet changes the weights", fontsize=7.5,
             fontweight="bold")
    fig.text(.545, .94, "b  Spatial overlap changes existence", fontsize=7.5,
             fontweight="bold")
    y_positions = [2.2, 1.1, 0]
    for row, ((name, weights), y) in enumerate(zip(rows, y_positions)):
        left = 0
        for idx, (weight, color) in enumerate(zip(weights, colors)):
            if weight == 0:
                continue
            ax.barh(y, weight, left=left, height=.47, color=color,
                    edgecolor="white", linewidth=.6)
            label = f"{weight:.3f}" if row == 1 else f"{weight:.2f}"
            if weight < .1:
                ax.text(1.025, y, label, ha="left", va="center", fontsize=6.4,
                        color="#704918")
            else:
                ax.text(left+weight/2, y, label, ha="center", va="center",
                        fontsize=6.4, color="white" if idx == 0 else "#26323b")
            left += weight
        ax.text(-.045, y, name, ha="right", va="center", fontsize=6.8,
                color="#26323b" if row == 0 else rule_colors[row-1])
    ax.plot(.84, 2.2, marker="x", color="#3f4a55", markersize=7, markeredgewidth=1.1)
    ax.set_xlim(0, 1.2); ax.set_ylim(-.6, 2.8)
    ax.set_xticks([0, .5, 1]); ax.set_yticks([])
    ax.spines["left"].set_visible(False)
    ax.set_xlabel("Fusion weight", labelpad=3)
    ax.tick_params(width=.6, length=2.4, pad=2)
    fig.legend(handles=[Patch(facecolor=c, label=l) for c, l in zip(
        colors, ["Self", "Dominant", "Gateway"])], ncol=3, fontsize=6.2,
        loc="upper center", bbox_to_anchor=(.268, .894), handlelength=1.0,
        handletextpad=.35, columnspacing=1.0)
    fig.add_artist(FancyArrowPatch((.466, .49), (.537, .49),
        transform=fig.transFigure, arrowstyle="-|>", mutation_scale=10,
        linewidth=.8, color="#8a929c"))

    x = np.linspace(-3, 7, 1001)
    normal = lambda mean: np.exp(-.5*(x-mean)**2)/np.sqrt(2*np.pi)
    r = .8
    input_self, input_gateway = r*normal(0), r*normal(4)
    bx.plot(x, input_self, color="#a1a7ae", ls="--", lw=.9)
    bx.plot(x, input_gateway, color="#a1a7ae", ls="--", lw=.9)
    curves, records = [], []
    for (name, weights), color in zip(rows[1:], rule_colors):
        a = weights[2]
        eta = np.exp(-8*a*(1-a))
        existence = r*eta/(1-r+r*eta)
        mean = 4*a
        intensity = existence*normal(mean)
        bx.plot(x, intensity, color=color, lw=1.7)
        curves.append(intensity)
        records.append([name, *weights, eta, existence, mean, 1])
    bx.text(.00, 1.12, rf"Renormalize: $\bar r={records[0][5]:.3f}$",
            transform=bx.transAxes, fontsize=6.7, color=rule_colors[0])
    bx.text(.57, 1.12, rf"Self: $\bar r={records[1][5]:.3f}$",
            transform=bx.transAxes, fontsize=6.7, color=rule_colors[1])
    bx.set_xlim(-3, 7); bx.set_ylim(0, .38)
    bx.set_xticks([-2, 0, 2, 4, 6]); bx.set_yticks([0, .1, .2, .3])
    bx.set_xlabel("Position x (arbitrary units)", labelpad=3)
    bx.set_ylabel(r"Singleton intensity $r\,p(x)$", labelpad=4)
    bx.tick_params(width=.6, length=2.4, pad=2)
    stem = here / "packet_fusion_mechanism"
    for extension in ["svg", "pdf", "png"]:
        fig.savefig(stem.with_suffix("."+extension), dpi=600, facecolor="white")
    plt.close(fig)
    with (here / "packet_fusion_mechanism_source.csv").open("w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["rule", "self_weight", "dominant_weight", "gateway_weight",
                         "eta", "fused_existence", "fused_mean", "fused_variance"])
        writer.writerows(records)
    with (here / "packet_fusion_curves.csv").open("w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["x", "self_input_intensity", "gateway_input_intensity",
                         "renormalized_intensity", "self_fallback_intensity"])
        writer.writerows(zip(x, input_self, input_gateway, *curves))
    manifest = {
        "backend": f"Python / Matplotlib {matplotlib.__version__}",
        "role": "analytic packet-to-weight-to-Bernoulli mechanism",
        "dimensions_mm": [178, 48],
        "formats": ["svg", "pdf", "png"], "png_dpi": 600,
        "source_data": ["packet_fusion_mechanism_source.csv", "packet_fusion_curves.csv"],
        "assumptions": {"input_existence": r, "input_means": [0, 4], "input_variances": [1, 1]},
        "statistical_object": "deterministic analytic example; no random or tracking samples",
        "accuracy_ranking": False, "ground_truth_specified": False,
        "arbitrary_gm_power_claim": False, "filter_rerun": False,
    }
    (here / "packet_fusion_mechanism_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    print(records)
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
