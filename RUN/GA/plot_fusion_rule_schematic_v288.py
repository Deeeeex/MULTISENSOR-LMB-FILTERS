"""Analytic shared-label fusion illustration; no simulation results are drawn."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def gaussian(x: np.ndarray, mean: float) -> np.ndarray:
    return np.exp(-0.5 * (x - mean) ** 2) / np.sqrt(2 * np.pi)


def export(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    existence = 0.8
    separation = 6.0
    x = np.linspace(-7, 7, 1401)
    p1 = gaussian(x, -separation / 2)
    p2 = gaussian(x, separation / 2)
    kla_density = gaussian(x, 0)
    mil_density = (p1 + p2) / 2
    distances = np.linspace(0, 6, 601)
    eta = np.exp(-distances ** 2 / 8)
    kla_existence = existence * eta / (1 - existence + existence * eta)
    mil_existence = np.full_like(distances, existence)
    for filename, header, columns in (
        ("SPATIAL_DENSITIES.csv", ["state", "source_1", "source_2", "kla", "mil"],
         [x, p1, p2, kla_density, mil_density]),
        ("EXISTENCE_CURVES.csv", ["separation_in_sigma", "eta", "kla_r", "mil_r"],
         [distances, eta, kla_existence, mil_existence]),
    ):
        with (out / filename).open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(zip(*columns))

    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 7.4, "axes.labelsize": 7.6, "xtick.labelsize": 7,
        "ytick.labelsize": 7, "axes.spines.top": False,
        "axes.spines.right": False, "axes.linewidth": 0.6,
        "svg.fonttype": "none", "pdf.fonttype": 42,
        "legend.frameon": False, "savefig.facecolor": "white",
    })
    blue, amber, gray = "#3677AD", "#B57540", "#969BA3"
    fig, (a, b) = plt.subplots(1, 2, figsize=(178 / 25.4, 77 / 25.4))
    fig.subplots_adjust(left=0.075, right=0.97, bottom=0.18, top=0.83, wspace=0.34)
    a.plot(x, p1, color=gray, ls=(0, (2, 2)), lw=1.0, label="Input densities")
    a.plot(x, p2, color=gray, ls=(0, (2, 2)), lw=1.0)
    a.plot(x, kla_density, color=blue, lw=1.65, label="KLA")
    a.plot(x, mil_density, color=amber, lw=1.65, ls="--", label="LMB-MIL")
    a.set(xlim=(-7, 7), ylim=(0, 0.46), xticks=[-6, -3, 0, 3, 6],
          yticks=[0, 0.2, 0.4], xlabel="Position / input standard deviation",
          ylabel="Conditional spatial density")
    a.annotate("Source 1", xy=(-3, p1.max()), xytext=(-4.8, 0.438),
               color="#646B75", fontsize=7)
    a.annotate("Source 2", xy=(3, p2.max()), xytext=(2.3, 0.438),
               color="#646B75", fontsize=7)
    b.plot(distances, mil_existence, color=amber, lw=1.65, ls="--")
    b.plot(distances, kla_existence, color=blue, lw=1.65)
    b.scatter([6], [kla_existence[-1]], color=blue, s=14, zorder=3)
    b.scatter([6], [existence], color=amber, s=14, marker="s", zorder=3)
    b.set(xlim=(0, 6.3), ylim=(0, 1), xticks=[0, 2, 4, 6],
          yticks=[0, 0.2, 0.4, 0.6, 0.8, 1],
          xlabel="Source separation / standard deviation",
          ylabel="Fused existence probability")
    b.text(4.5, 0.855, "LMB-MIL: 0.800", color=amber, ha="center", fontsize=7)
    b.text(3.2, 0.09, f"KLA: {kla_existence[-1]:.3f}", color=blue, fontsize=7)
    for ax, letter in ((a, "a"), (b, "b")):
        ax.text(-0.15, 1.10, letter, transform=ax.transAxes, fontsize=9, weight="bold")
        ax.tick_params(length=2.5, width=0.6)
        ax.grid(axis="y", color="#E9EBEF", lw=0.55, zorder=0)
        ax.set_axisbelow(True)
    fig.legend(*a.get_legend_handles_labels(), loc="upper center", ncol=3,
               bbox_to_anchor=(0.5, 0.995), handlelength=2.8, columnspacing=2.2)
    stem = out / "FUSION_RULE_SCHEMATIC_V288"
    for extension in ("svg", "pdf", "png"):
        fig.savefig(stem.with_suffix("." + extension), dpi=600)
    plt.close(fig)
    manifest = {
        "type": "analytic illustration, not empirical tracking evidence",
        "input_existence": existence, "input_sigma": 1.0,
        "source_weights": [0.5, 0.5], "panel_a_separation": separation,
        "selected_eta": float(eta[-1]),
        "selected_kla_existence": float(kla_existence[-1]),
        "selected_mil_existence": existence,
        "size_mm": [178, 77], "png_dpi": 600,
        "uncertainty": "none; exact analytic curves for fixed inputs",
        "ground_truth_used": False, "gm_reduction_used": False,
        "source_data": ["SPATIAL_DENSITIES.csv", "EXISTENCE_CURVES.csv"],
    }
    (out / "FUSION_SCHEMATIC_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Analytic figure exported: eta={eta[-1]:.9f}, KLA r={kla_existence[-1]:.9f}, MIL r={existence:.3f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    export(parser.parse_args().output)
