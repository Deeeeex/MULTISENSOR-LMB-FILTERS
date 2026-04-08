from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ is None or __package__ == "":
    repo_root = Path(__file__).resolve().parents[3]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from docs.paper.figures.paper_figure_data import get_scalar_figure_data, load_figure4_series


# Brighter paper palette with softened saturation: cool colors lead, with a
# restrained warm accent reserved for the strongest ablation stage.
FIXED_COLOR = "#78A6D8"
ADAPTIVE_COLOR = "#62B7AE"
SECONDARY_COLOR = "#6B93CF"
ABLATION_COLORS = ["#D8E8F6", "#BDD8EF", "#9EC8E6", "#97D0CA", "#E3B36C"]
EDGE_COLOR = "#2F4858"


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def ensure_output_dir(output_dir: str | Path) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    return output_path


def save_figure5(output_path: str | Path, figure5: dict) -> Path:
    output_path = Path(output_path)
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.7), constrained_layout=True)
    x = np.arange(len(figure5["arms"]))
    short_labels = ["fixed", "cov", "link", "exist", "struct"]

    for ax, (metric_name, values) in zip(axes, figure5["metrics"].items()):
        ax.bar(x, values, color=ABLATION_COLORS, edgecolor=EDGE_COLOR, linewidth=0.8)
        ax.set_title(metric_name)
        ax.set_xticks(x, short_labels, rotation=25, ha="right")
        ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
        ax.set_axisbelow(True)

        ymin = min(values)
        ymax = max(values)
        pad = (ymax - ymin) * 0.15 if ymax > ymin else 0.1
        ax.set_ylim(max(0.0, ymin - pad), ymax + pad)

    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    plt.close(fig)
    return output_path


def save_figure3(output_path: str | Path, figure3: dict) -> Path:
    output_path = Path(output_path)
    r = np.asarray(figure3["r"], dtype=float)
    decisiveness = np.abs(2.0 * r - 1.0)
    min_score = float(figure3["existence_confidence_min_score"])
    power = float(figure3["existence_confidence_power"])
    bounded = min_score + (1.0 - min_score) * np.power(decisiveness, power)

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.3), constrained_layout=True, sharex=True)

    axes[0].plot(r, decisiveness, color=ADAPTIVE_COLOR, linewidth=2.2)
    axes[0].set_title("Raw Decisiveness")
    axes[0].set_xlabel("Existence Probability r")
    axes[0].set_ylabel("c = |2r - 1|")
    axes[0].set_xlim(0.0, 1.0)
    axes[0].set_ylim(0.0, 1.05)
    axes[0].grid(alpha=0.25, linestyle="--", linewidth=0.7)
    axes[0].set_axisbelow(True)

    axes[1].plot(r, bounded, color=SECONDARY_COLOR, linewidth=2.2)
    axes[1].axhline(min_score, color=FIXED_COLOR, linewidth=1.2, linestyle=":")
    axes[1].set_title("Bounded Weighting Curve")
    axes[1].set_xlabel("Existence Probability r")
    axes[1].set_ylabel("Existence-Confidence Score")
    axes[1].set_xlim(0.0, 1.0)
    axes[1].set_ylim(min_score - 0.02, 1.02)
    axes[1].grid(alpha=0.25, linestyle="--", linewidth=0.7)
    axes[1].set_axisbelow(True)

    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    plt.close(fig)
    return output_path


def save_figure6(output_path: str | Path, figure6: dict) -> Path:
    output_path = Path(output_path)
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 3.8), constrained_layout=True)
    width = 0.36

    consensus_x = np.arange(len(figure6["consensus"]["labels"]))
    axes[0].bar(
        consensus_x - width / 2,
        figure6["consensus"]["ga"],
        width,
        label="Ordinary GA",
        color=FIXED_COLOR,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
    )
    axes[0].bar(
        consensus_x + width / 2,
        figure6["consensus"]["adaptive"],
        width,
        label="Structure-aware decoupled KLA",
        color=ADAPTIVE_COLOR,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
    )
    axes[0].set_title("Consensus Metrics")
    axes[0].set_xticks(consensus_x, figure6["consensus"]["labels"])
    axes[0].grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
    axes[0].set_axisbelow(True)
    axes[0].legend(frameon=False, loc="upper right")

    local_x = np.arange(len(figure6["local"]["labels"]))
    axes[1].bar(
        local_x - width / 2,
        figure6["local"]["ga"],
        width,
        color=FIXED_COLOR,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
    )
    axes[1].bar(
        local_x + width / 2,
        figure6["local"]["adaptive"],
        width,
        color=ADAPTIVE_COLOR,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
    )
    axes[1].set_title("Aggregate Local Metrics")
    axes[1].set_xticks(local_x, figure6["local"]["labels"])
    axes[1].grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
    axes[1].set_axisbelow(True)

    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    plt.close(fig)
    return output_path


def save_figure4(output_path: str | Path, series: dict) -> Path:
    output_path = Path(output_path)
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.7), sharex=True, constrained_layout=True)
    configs = [
        ("Consensus OSPA", "ospa_fixed", "ospa_adaptive"),
        ("Consensus RMSE", "rmse_fixed", "rmse_adaptive"),
        ("Consensus Card", "card_fixed", "card_adaptive"),
    ]
    time = np.asarray(series["time"])
    for ax, (title, fixed_key, adaptive_key) in zip(axes, configs):
        ax.plot(time, series[fixed_key], color=FIXED_COLOR, linewidth=2.0, label="Fixed")
        ax.plot(time, series[adaptive_key], color=ADAPTIVE_COLOR, linewidth=2.2, label="Adaptive")
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
        ax.set_axisbelow(True)
        ax.set_xlabel("Time Step")

    axes[0].set_ylabel("Metric Value")
    axes[0].legend(frameon=False, loc="upper right")
    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    plt.close(fig)
    return output_path


def render_all_figures(output_dir: str | Path, include_figure4: bool = False) -> dict[str, Path]:
    apply_style()
    data = get_scalar_figure_data()
    output_root = ensure_output_dir(output_dir)
    outputs = {
        "figure3": save_figure3(output_root / "figure3_existence_confidence_curve.pdf", data["figure3"]),
        "figure5": save_figure5(output_root / "figure5_factor_ablation.pdf", data["figure5"]),
        "figure6": save_figure6(output_root / "figure6_ideal_support.pdf", data["figure6"]),
    }

    if include_figure4:
        csv_path = output_root / "figure4_consensus_series.csv"
        if csv_path.exists():
            outputs["figure4"] = save_figure4(
                output_root / "figure4_main_ga_consensus.pdf",
                load_figure4_series(csv_path),
            )

    return outputs


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render paper-ready Figure 4/5/6 PDFs.")
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent),
        help="Directory where PDF outputs will be written.",
    )
    parser.add_argument(
        "--include-figure4",
        action="store_true",
        help="Render Figure 4 if figure4_consensus_series.csv exists in the output directory.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    outputs = render_all_figures(args.output_dir, include_figure4=args.include_figure4)
    for name, path in outputs.items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
