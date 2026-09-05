"""Plot same-query geometric-correspondence restrictions from saved results."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "RUN/GA/dynamic_topology/evidence/tracking_aligned_v287/x36_label_identity_spatial_headroom_seed1301"


def main():
    with (OUT / "V287_IDENTITY_RESTRICTED_HEADROOM.csv").open(newline="") as handle:
        data = {int(r["pool"]): r for r in csv.DictReader(handle)
                if r["birth_stratum"] == "-1" and r["lag"] == "1"}
    order = [4, 5, 1]
    assert set(data) == set(order)
    assert {int(r["queries"]) for r in data.values()} == {7818}
    original = np.array([float(data[p]["original_oracle_pooled_rmse"]) for p in order])
    coherent = np.array([float(data[p]["assignment_coherent_oracle_pooled_rmse"]) for p in order])
    receiver = float(data[1]["receiver_pooled_rmse"])
    assert np.all(original <= coherent) and np.all(coherent <= receiver)

    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 8,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.65, "legend.frameon": False,
        "svg.fonttype": "none", "pdf.fonttype": 42,
        "text.color": "#253142", "axes.labelcolor": "#253142",
        "figure.facecolor": "white", "savefig.facecolor": "white",
    })
    fig = plt.figure(figsize=(150 / 25.4, 77 / 25.4))
    ax = fig.add_axes([0.245, 0.30, 0.69, 0.60])
    y = np.arange(3)
    ax.axvline(receiver, color="#7A828C", lw=1, ls="--", zorder=1)
    ax.text(receiver, -0.58, f"Receiver {receiver:.2f} m", ha="center",
            va="center", color="#626B76", fontsize=7.3,
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.2})
    for i, (a, b) in enumerate(zip(original, coherent)):
        ax.plot([a, b], [i - 0.14, i + 0.14], color="#D4DAE2", lw=1.7, zorder=1)
    ax.scatter(original, y - 0.14, facecolor="white", edgecolor="#2474AD",
               marker="D", s=34, lw=1.2, zorder=3, label="Original same-label oracle")
    ax.scatter(coherent, y + 0.14, color="#7956A1", marker="s", s=31,
               zorder=3, label="Source assignment also agrees with query target")
    for values, offset, color in [(original, -0.14, "#2474AD"), (coherent, 0.14, "#7956A1")]:
        for i, value in enumerate(values):
            ax.annotate(f"{value:.2f}", (value, i + offset), xytext=(-7, 0),
                        textcoords="offset points", ha="right", va="center",
                        color=color, fontsize=7.5,
                        bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.6})
    ax.set(xlim=(0, 39), ylim=(2.55, -0.70), xticks=[0, 10, 20, 30],
           yticks=y, yticklabels=["Delivered input", "Physical one-hop", "Whole network"],
           xlabel="Oracle pooled position RMSE (m)")
    ax.tick_params(axis="y", length=0, pad=8)
    ax.grid(axis="x", color="#E5E8ED", lw=0.5)
    ax.set_axisbelow(True)
    fig.legend(*ax.get_legend_handles_labels(), loc="lower left",
               bbox_to_anchor=(0.245, 0.028), fontsize=7.2,
               handlelength=1.1, handletextpad=0.6, borderaxespad=0)
    for suffix in ("svg", "pdf", "png"):
        fig.savefig(OUT / f"LABEL_COHERENCE_HEADROOM_V287.{suffix}", dpi=600)
    plt.close(fig)
    manifest = {
        "size_mm": [150, 77], "backend": f"Python / Matplotlib {matplotlib.__version__}",
        "source": "V287_IDENTITY_RESTRICTED_HEADROOM.csv", "seed": 1301,
        "scene": "X36", "time_scope": "prefix 1-40, previous-step sources predicted once",
        "queries": 7818, "birth_stratum": "all", "pool_order": order,
        "receiver_pooled_rmse": receiver,
        "original_oracle_pooled_rmse": original.tolist(),
        "assignment_coherent_oracle_pooled_rmse": coherent.tolist(),
        "truth_selected": True, "retains_receiver": True,
        "uncertainty": "One opened development seed; no independent repeats or inference",
        "online_policy": False, "paper_insertion": False,
        "lark_best_table_update": False, "svg_editable_text": True, "png_dpi": 600,
    }
    (OUT / "V287_FIGURE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("V287 label-correspondence figure exported: SVG, PDF, PNG; 7,818 identical saved queries.")


if __name__ == "__main__":
    main()
