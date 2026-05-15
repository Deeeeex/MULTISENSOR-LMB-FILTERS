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
    fig, axes = plt.subplots(1, 3, figsize=(11.8, 4.0), constrained_layout=True)
    x = np.arange(len(figure5["arms"]))
    display_labels = [
        "Fixed\nMetropolis",
        "Covariance-only\nadaptive",
        "Covariance-link\nadaptive",
        "Three-factor\nbackbone",
        "Balanced\nmode",
    ]

    for ax, (metric_name, values) in zip(axes, figure5["metrics"].items()):
        ax.bar(x, values, color=ABLATION_COLORS, edgecolor=EDGE_COLOR, linewidth=0.8)
        ax.set_title(metric_name)
        ax.set_xticks(x, display_labels)
        ax.tick_params(axis="x", labelsize=7.8)
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

    profiles = figure3["profiles"]
    profile_labels = [profile["label"] for profile in profiles]
    profile_r = [np.asarray(profile["r_values"], dtype=float) for profile in profiles]
    profile_cbar = []
    profile_q = []
    for values in profile_r:
        c_values = np.abs(2.0 * values - 1.0)
        weighted = np.sum(values * c_values) / np.sum(values)
        profile_cbar.append(weighted)
        profile_q.append(min_score + (1.0 - min_score) * weighted**power)

    fig, (ax_left, ax_right) = plt.subplots(
        1,
        2,
        figsize=(8.6, 3.6),
        gridspec_kw={"width_ratios": [1.3, 0.95]},
        constrained_layout=True,
    )

    decisive_fill = "#EFF6FB"
    ambiguous_fill = "#FAEEEE"
    raw_color = "#98A6B5"

    ax_left.axvspan(0.0, 0.18, color=decisive_fill, zorder=0)
    ax_left.axvspan(0.42, 0.58, color=ambiguous_fill, zorder=0)
    ax_left.axvspan(0.82, 1.0, color=decisive_fill, zorder=0)
    ax_left.fill_between(r, bounded, min_score, color=SECONDARY_COLOR, alpha=0.08, zorder=1)
    ax_left.plot(r, bounded, color=SECONDARY_COLOR, linewidth=2.8, zorder=3)
    ax_left.plot(r, decisiveness, color=raw_color, linewidth=1.8, linestyle="--", zorder=2)
    ax_left.axhline(min_score, color=FIXED_COLOR, linewidth=1.4, linestyle=":", zorder=2)

    sample_r = np.array([0.5, 0.9])
    sample_score = min_score + (1.0 - min_score) * np.power(np.abs(2.0 * sample_r - 1.0), power)
    ax_left.scatter(
        sample_r,
        sample_score,
        s=34,
        facecolor="white",
        edgecolor=SECONDARY_COLOR,
        linewidth=1.2,
        zorder=4,
    )

    ax_left.set_xlabel("Existence Probability r")
    ax_left.set_ylabel(r"Score magnitude")
    ax_left.set_xlim(0.0, 1.0)
    ax_left.set_ylim(-0.02, 1.02)
    ax_left.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
    ax_left.set_axisbelow(True)

    ax_left.text(0.50, 1.005, "ambiguous region", fontsize=7.6, color="#8B5A5A", va="bottom", ha="center")
    ax_left.text(0.98, 1.005, "decisive tails", fontsize=7.6, color="#5A738E", va="bottom", ha="right")
    ax_left.text(0.63, min_score + 0.0025, r"$\lambda_{\min}$ floor", fontsize=8, color=FIXED_COLOR)
    ax_left.text(
        0.66,
        0.986,
        r"$q_{\mathrm{exist}}$ in $\tilde{\omega}$",
        fontsize=7.5,
        color=EDGE_COLOR,
        bbox={"boxstyle": "round,pad=0.20", "facecolor": "white", "edgecolor": "#D7DEE7", "linewidth": 0.8},
    )
    ax_left.text(0.18, 0.955, r"bounded $q_{\mathrm{exist}}(r)$", fontsize=7.8, color=SECONDARY_COLOR)
    ax_left.text(0.16, 0.135, r"raw $c(r)=|2r-1|$", fontsize=7.8, color=raw_color)

    y_positions = np.arange(len(profiles))[::-1]
    profile_band = "#EDF3F8"
    bar_height = 0.24
    ax_right.axvline(min_score, color=FIXED_COLOR, linewidth=1.3, linestyle=":")
    ax_right.barh(
        y_positions + 0.14,
        profile_cbar,
        height=bar_height,
        color=profile_band,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
        label=r"aggregated $\bar{c}^{(j)}$",
    )
    ax_right.barh(
        y_positions - 0.14,
        profile_q,
        height=bar_height,
        color=ADAPTIVE_COLOR,
        edgecolor=EDGE_COLOR,
        linewidth=0.8,
        label=r"final $q_{\mathrm{exist}}^{(j)}$",
    )

    for idx, values in enumerate(profile_r):
        y = y_positions[idx]
        ax_right.scatter(
            values,
            np.full_like(values, y + 0.28),
            s=22,
            color=SECONDARY_COLOR,
            edgecolor="white",
            linewidth=0.5,
            zorder=3,
        )
        ax_right.text(
            1.01,
            y + 0.14,
            f"{profile_cbar[idx]:.2f}",
            fontsize=7.5,
            va="center",
            ha="left",
            color=EDGE_COLOR,
        )
        ax_right.text(
            1.01,
            y - 0.14,
            f"{profile_q[idx]:.2f}",
            fontsize=7.5,
            va="center",
            ha="left",
            color=EDGE_COLOR,
        )

    ax_right.set_yticks(y_positions, profile_labels)
    ax_right.set_xlim(0.0, 1.12)
    ax_right.set_ylim(-0.45, y_positions[0] + 0.45)
    ax_right.set_xlabel("Aggregated confidence level")
    ax_right.grid(axis="x", alpha=0.25, linestyle="--", linewidth=0.7)
    ax_right.set_axisbelow(True)
    ax_right.text(0.55, y_positions[1] + 0.40, r"example $r_{k,i}^{(j)}$", fontsize=7.0, color=EDGE_COLOR, ha="center")

    ax_left.text(0.01, 0.99, "(a) Mapping", transform=ax_left.transAxes, fontsize=9.2, fontweight="bold", ha="left", va="top")
    ax_right.text(0.01, 0.99, "(b) Aggregation", transform=ax_right.transAxes, fontsize=9.2, fontweight="bold", ha="left", va="top")

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
        label="Balanced mode",
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
        ("Position disagreement", "rmse_fixed", "rmse_adaptive"),
        ("Cardinality disagreement", "card_fixed", "card_adaptive"),
    ]
    time = np.asarray(series["time"])
    for ax, (title, fixed_key, adaptive_key) in zip(axes, configs):
        ax.plot(time, series[fixed_key], color=FIXED_COLOR, linewidth=2.0, label="Fixed Metropolis")
        ax.plot(time, series[adaptive_key], color=ADAPTIVE_COLOR, linewidth=2.2, label="Cardinality-critical mode")
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
        ax.set_axisbelow(True)
        ax.set_xlabel("Time Step")

    axes[0].set_ylabel("Metric Value")
    axes[0].legend(
        frameon=True,
        loc="lower right",
        facecolor="white",
        edgecolor="#D7DEE7",
        framealpha=0.92,
    )
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
