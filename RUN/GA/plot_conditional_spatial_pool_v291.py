"""Analytic operator comparison; no tracking truth or empirical samples."""
from pathlib import Path
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    out = Path(__file__).resolve().parent / "dynamic_topology/evidence/tracking_aligned_v291/analytic_fusion_control"
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 7, "svg.fonttype": "none", "pdf.fonttype": 42,
        "axes.spines.right": False, "axes.spines.top": False,
        "axes.linewidth": .6, "legend.frameon": False,
    })
    x = np.linspace(-7, 7, 1401)
    normal = lambda m: np.exp(-.5 * (x-m)**2) / np.sqrt(2*np.pi)
    r, d = .8, 6.
    eta = np.exp(-d*d/8)
    r_kla = r*eta/(1-r+r*eta)
    left, right = r*normal(-d/2), r*normal(d/2)
    curves = [r_kla*normal(0), .5*(left+right), r*normal(0)]
    titles = ["a  KLA", "b  LMB-MIL", "c  Conditional GA control"]
    colors = ["#31688e", "#b47c32", "#77619e"]
    rs = [r_kla, r, r]
    fig, axes = plt.subplots(1, 3, figsize=(178/25.4, 65/25.4), sharex=True, sharey=True)
    fig.subplots_adjust(left=.083, right=.985, bottom=.22, top=.80, wspace=.19)
    for ax, y, title, color, existence in zip(axes, curves, titles, colors, rs):
        ax.plot(x, left, color="#aaaaaa", lw=.9, ls="--", label="Inputs")
        ax.plot(x, right, color="#aaaaaa", lw=.9, ls="--")
        ax.plot(x, y, color=color, lw=1.7)
        ax.set_title(title, loc="left", fontsize=7.2, fontweight="bold", pad=14)
        ax.text(.5, .96, rf"$\bar r={existence:.4f}$", transform=ax.transAxes,
                ha="center", va="top", color=color, fontsize=7.2)
        ax.set_xlim(-7, 7); ax.set_ylim(0, .38)
        ax.set_xticks([-6, -3, 0, 3, 6]); ax.set_yticks([0, .1, .2, .3])
        ax.tick_params(width=.6, length=2.5, pad=2)
    axes[0].set_ylabel(r"Singleton intensity $r\,p(x)$")
    fig.supxlabel("Position x (arbitrary units)", x=.53, y=.06, fontsize=7.5)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper right",
               bbox_to_anchor=(.986, .996), fontsize=6.5)
    stem = out / "CONDITIONAL_SPATIAL_POOL_V291"
    for extension in ("svg", "pdf", "png"):
        fig.savefig(stem.with_suffix("."+extension), dpi=300, facecolor="white")
    plt.close(fig)
    with (out / "ANALYTIC_SOURCE.csv").open("w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["x", "input_left_intensity", "input_right_intensity",
                         "kla_intensity", "mil_intensity", "conditional_ga_intensity"])
        writer.writerows(zip(x, left, right, *curves))
    print(f"Analytic r: KLA={r_kla:.9f}; MIL=control={r:.9f}. No tracking claim.")
    print(stem.with_suffix(".png"))


if __name__ == "__main__":
    main()
