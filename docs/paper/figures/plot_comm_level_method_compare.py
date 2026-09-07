from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


FIXED_COLOR = "#78A6D8"
BALANCED_COLOR = "#62B7AE"
CARDINALITY_COLOR = "#D08A59"
EDGE_COLOR = "#2F4858"

METHODS = ["Fixed Metropolis", "Balanced mode", "Cardinality-critical mode"]
METHOD_COLORS = {
    "Fixed Metropolis": FIXED_COLOR,
    "Balanced mode": BALANCED_COLOR,
    "Cardinality-critical mode": CARDINALITY_COLOR,
}
METHOD_MARKERS = {
    "Fixed Metropolis": "o",
    "Balanced mode": "s",
    "Cardinality-critical mode": "^",
}

NETWORK_METRICS = [
    "OSPA consensus error",
    "Matched localization disagreement",
    "Cardinality dispersion",
]
METRIC_TITLES = {
    "OSPA consensus error": "OSPA Consensus Error",
    "Matched localization disagreement": "Matched Localization Disagreement",
    "Cardinality dispersion": "Cardinality Dispersion",
}


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "font.size": 10,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "xtick.labelsize": 8.2,
            "ytick.labelsize": 8.5,
            "legend.fontsize": 8.5,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def build_mock_rows() -> list[dict[str, object]]:
    levels = [0, 1, 2, 3]
    level_labels = ["none", "bandwidth cap", "tiered link loss", "node outage"]
    means = {
        "OSPA consensus error": {
            "Fixed Metropolis": [1.62, 1.74, 2.48, 2.56],
            "Balanced mode": [1.41, 1.50, 1.80, 1.84],
            "Cardinality-critical mode": [1.39, 1.47, 1.68, 1.73],
        },
        "Matched localization disagreement": {
            "Fixed Metropolis": [1.38, 1.50, 2.40, 2.49],
            "Balanced mode": [1.19, 1.25, 1.42, 1.48],
            "Cardinality-critical mode": [1.28, 1.31, 1.55, 1.61],
        },
        "Cardinality dispersion": {
            "Fixed Metropolis": [0.10, 0.15, 0.69, 0.76],
            "Balanced mode": [0.07, 0.09, 0.21, 0.20],
            "Cardinality-critical mode": [0.05, 0.06, 0.07, 0.08],
        },
    }
    half_widths = {
        "OSPA consensus error": [0.035, 0.045, 0.105, 0.115],
        "Matched localization disagreement": [0.045, 0.055, 0.135, 0.145],
        "Cardinality dispersion": [0.012, 0.018, 0.055, 0.060],
    }

    rows: list[dict[str, object]] = []
    for metric in NETWORK_METRICS:
        for level, level_label, half_width in zip(levels, level_labels, half_widths[metric]):
            for method in METHODS:
                mean = means[metric][method][level]
                method_scale = 0.85 if method == "Cardinality-critical mode" else 1.0
                ci_half = half_width * method_scale
                rows.append(
                    {
                        "metric_group": "network",
                        "metric": metric,
                        "level": level,
                        "level_label": level_label,
                        "method": method,
                        "mean": mean,
                        "std": ci_half * 3.2,
                        "ci_low": mean - ci_half,
                        "ci_high": mean + ci_half,
                        "n": 50,
                    }
                )
    return rows


def load_rows(csv_path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("metric_group") != "network":
                continue
            if row.get("metric") not in NETWORK_METRICS:
                continue
            rows.append(
                {
                    "metric_group": row["metric_group"],
                    "metric": row["metric"],
                    "level": int(row["level"]),
                    "level_label": row["level_label"],
                    "method": row["method"],
                    "mean": float(row["mean"]),
                    "std": float(row["std"]),
                    "ci_low": float(row["ci_low"]),
                    "ci_high": float(row["ci_high"]),
                    "n": int(float(row["n"])),
                }
            )
    return rows


def rows_to_metric_grid(
    rows: list[dict[str, object]], methods: list[str]
) -> tuple[list[int], list[str], dict[str, dict[str, np.ndarray]]]:
    levels = sorted({int(row["level"]) for row in rows})
    level_labels_by_level = {int(row["level"]): str(row["level_label"]) for row in rows}
    level_labels = [level_labels_by_level[level] for level in levels]
    grid: dict[str, dict[str, np.ndarray]] = {}
    for metric in NETWORK_METRICS:
        grid[metric] = {}
        for method in methods:
            method_rows = [
                row
                for row in rows
                if row["metric"] == metric and row["method"] == method
            ]
            by_level = {int(row["level"]): row for row in method_rows}
            mean = np.array([float(by_level[level]["mean"]) for level in levels], dtype=float)
            ci_low = np.array([float(by_level[level]["ci_low"]) for level in levels], dtype=float)
            ci_high = np.array([float(by_level[level]["ci_high"]) for level in levels], dtype=float)
            grid[metric][method] = np.vstack([mean, ci_low, ci_high])
    return levels, level_labels, grid


def save_plot(rows: list[dict[str, object]], output_base: Path) -> list[Path]:
    apply_style()
    methods = [method for method in METHODS if any(row["method"] == method for row in rows)]
    if not methods:
        raise ValueError("No supported methods were found in the input rows.")
    levels, level_labels, grid = rows_to_metric_grid(rows, methods)
    x = np.asarray(levels, dtype=float)
    offset_values = np.linspace(-0.04, 0.04, len(methods)) if len(methods) > 1 else np.array([0.0])
    offsets = dict(zip(methods, offset_values))

    fig, axes = plt.subplots(1, 3, figsize=(11.8, 4.15))
    fig.subplots_adjust(left=0.065, right=0.99, bottom=0.25, top=0.88, wspace=0.24)
    for ax, metric in zip(axes, NETWORK_METRICS):
        for method in methods:
            mean = grid[metric][method][0]
            ci_low = grid[metric][method][1]
            ci_high = grid[metric][method][2]
            yerr = np.vstack([mean - ci_low, ci_high - mean])
            ax.errorbar(
                x + offsets[method],
                mean,
                yerr=yerr,
                color=METHOD_COLORS[method],
                marker=METHOD_MARKERS[method],
                markersize=5.2,
                markerfacecolor="white",
                markeredgewidth=1.1,
                linewidth=2.0,
                elinewidth=1.2,
                capsize=3.5,
                capthick=1.1,
                label=method,
                zorder=3,
            )
        ax.set_title(METRIC_TITLES[metric])
        ax.set_xticks(x, [f"{level}\n{label}" for level, label in zip(levels, level_labels)])
        ax.set_xlabel("Communication Level")
        ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.7)
        ax.set_axisbelow(True)

    axes[0].set_ylabel("Metric Value")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        frameon=False,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.025),
        ncol=len(methods),
    )
    output_base.parent.mkdir(parents=True, exist_ok=True)
    outputs = [output_base.with_suffix(".pdf"), output_base.with_suffix(".png")]
    for output_path in outputs:
        fig.savefig(output_path, bbox_inches="tight", dpi=220)
    plt.close(fig)
    return outputs


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot communication-level method comparison with error bars."
    )
    parser.add_argument(
        "--input-csv",
        type=Path,
        help="CSV containing network metrics for any supported subset of the compared methods.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use embedded mock data instead of reading --input-csv.",
    )
    parser.add_argument(
        "--output-base",
        type=Path,
        default=Path(__file__).resolve().parent / "comm_level_method_compare",
        help="Output path without extension; .pdf and .png are written.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.mock:
        rows = build_mock_rows()
    else:
        if args.input_csv is None:
            parser.error("--input-csv is required unless --mock is set")
        rows = load_rows(args.input_csv)
    outputs = save_plot(rows, args.output_base)
    for output_path in outputs:
        print(output_path)


if __name__ == "__main__":
    main()
