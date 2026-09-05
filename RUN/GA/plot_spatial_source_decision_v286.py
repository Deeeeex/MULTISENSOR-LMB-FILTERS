"""Plot source-access and source-ranking diagnostics from saved CSVs only."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "RUN/GA/dynamic_topology/evidence"
ACCESS = EVIDENCE / "tracking_aligned_v285/x36_same_label_spatial_availability_seed1301"
OUT = EVIDENCE / "tracking_aligned_v286/x36_final_snapshot_source_risk_seed1301"


def rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def main():
    access = {int(r["pool"]): r for r in rows(ACCESS / "V285_SOURCE_AVAILABILITY_SUMMARY.csv")
              if r["formation"] == "0" and r["lag"] == "1"}
    ranking = [r for r in rows(OUT / "V286_SOURCE_RISK_SUMMARY.csv")
               if r["formation"] != "0" and r["pool"] == "1"]
    assert len(access) == 5 and len(ranking) == 6
    assert {r["query_count"] for r in access.values()} == {"7818"}
    assert sum(int(r["query_count"]) for r in ranking) == 139
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 7.2,
        "axes.labelsize": 7.2, "xtick.labelsize": 7, "ytick.labelsize": 7,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.65, "legend.frameon": False,
        "svg.fonttype": "none", "pdf.fonttype": 42,
        "text.color": "#253142", "axes.labelcolor": "#253142",
        "figure.facecolor": "white", "savefig.facecolor": "white",
    })
    fig = plt.figure(figsize=(175 / 25.4, 83 / 25.4))
    ax = fig.add_axes([0.165, 0.18, 0.305, 0.60])
    bx = fig.add_axes([0.655, 0.18, 0.325, 0.60])
    fig.text(0.025, 0.94, "a  Source access", fontsize=8.5, weight="bold")
    fig.text(0.025, 0.875, "Previous-step sources; 7,818 cases", fontsize=7)
    fig.text(0.545, 0.94, "b  Full-posterior-risk ranking", fontsize=8.5, weight="bold")
    fig.text(0.545, 0.875, "Final snapshot; 139 cases", fontsize=7)
    order = [4, 3, 2, 5, 1]
    labels = ["Delivered input", "Planned input", "Same formation", "Physical one-hop", "Whole network"]
    values = [float(access[p]["oracle_pooled_rmse"]) for p in order]
    current = float(access[1]["receiver_pooled_rmse"])
    ax.axvline(current, ls="--", color="#7A828C", lw=1, zorder=1)
    ax.scatter(values, np.arange(5), facecolor="white", edgecolor="#2474AD",
               marker="D", linewidth=1.2, s=30, zorder=3)
    for i, value in enumerate(values):
        ax.annotate(f"{value:.2f}", (value, i), xytext=(-6, 0),
                    textcoords="offset points", ha="right", va="center", fontsize=7)
    ax.text(current, -0.65, f"Receiver {current:.2f}", fontsize=6.8,
            color="#626B76", ha="center", va="center")
    ax.set(xlim=(0, 41), ylim=(4.6, -1.0), xticks=[0, 10, 20, 30, 40],
           yticks=np.arange(5), yticklabels=labels, xlabel="Oracle pooled position RMSE (m)")
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="x", color="#E5E8ED", lw=0.5)
    ax.set_axisbelow(True)
    base = np.array([float(r["receiver_pooled_rmse"]) for r in ranking])
    full = np.array([float(r["min_full_risk_pooled_rmse"]) for r in ranking])
    assert np.isfinite(base).all() and np.isfinite(full).all()
    for i, (b, f) in enumerate(zip(base, full)):
        bx.plot([b, f], [i, i], color="#CED3DC", lw=2.3, zorder=1)
    bx.scatter(base, np.arange(6), color="#7A828C", marker="o", s=21,
               label="Receiver", zorder=3)
    bx.scatter(full, np.arange(6), color="#7956A1", marker="s", s=21,
               label="Min. full risk", zorder=3)
    bx.set(xlim=(0, 140), ylim=(5.55, -0.65), xticks=[0, 40, 80, 120],
           yticks=np.arange(6),
           yticklabels=[f"F{r['formation']}  (n={r['query_count']})" for r in ranking],
           xlabel="Pooled position RMSE (m)")
    bx.tick_params(axis="y", length=0)
    bx.grid(axis="x", color="#E5E8ED", lw=0.5)
    bx.set_axisbelow(True)
    bx.legend(loc="lower left", bbox_to_anchor=(-0.005, 1.005),
              ncol=2, fontsize=6.5, handlelength=1.0, columnspacing=0.8, borderaxespad=0)
    for suffix in ("svg", "pdf", "png"):
        fig.savefig(OUT / f"SPATIAL_SOURCE_DECISION_V286.{suffix}", dpi=600)
    plt.close(fig)
    manifest = {
        "size_mm": [175, 83], "backend": f"Python / Matplotlib {matplotlib.__version__}",
        "png_dpi": 600, "editable_svg_text": True, "seed": 1301, "scene": "X36",
        "left": {"source": "V285_SOURCE_AVAILABILITY_SUMMARY.csv", "lag": 1,
                 "queries": 7818, "truth_selected_oracle": True, "retains_receiver": True},
        "right": {"source": "V286_SOURCE_RISK_SUMMARY.csv", "time": 40,
                  "queries": 139, "truth_used_by_risk_selector": False},
        "uncertainty": "No independent repeats, confidence intervals, or significance test",
        "scope": "Different query sets; no across-panel numerical comparison",
        "online_policy": False, "paper_insertion": False, "lark_best_table_update": False,
        "left_oracle_rmse_m": dict(zip(labels, values)),
        "right_rmse_m": {f"F{r['formation']}": {"receiver": float(b), "min_full_risk": float(f)}
                         for r, b, f in zip(ranking, base, full)},
    }
    (OUT / "V286_FIGURE_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("V286 source-decision figure exported: SVG, PDF, PNG; saved CSV inputs only.")


if __name__ == "__main__":
    main()
