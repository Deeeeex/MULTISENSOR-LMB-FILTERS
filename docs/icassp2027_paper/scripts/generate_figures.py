#!/usr/bin/env python3
"""Generate traceable ICASSP figures from frozen moment-exchange evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


PAPER_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_EVIDENCE = (
    REPO_ROOT
    / "RUN"
    / "GA"
    / "GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv"
)
DEFAULT_OUTPUT_DIR = PAPER_ROOT / "figures"
FIGURE_NAMES = (
    "payload_graph_schematic.pdf",
    "payload_graph_schematic.png",
    "heldout_tradeoff.pdf",
    "heldout_tradeoff.png",
)

FULL_COLOR = "#5B6472"
MOMENT_COLOR = "#2878B5"
FUSION_COLOR = "#7251B5"
OUTPUT_COLOR = "#3A8D6D"
WIRE_COLOR = "#D18F29"
TEXT_COLOR = "#17202A"
MUTED_COLOR = "#40505E"
GRID_COLOR = "#DCE3E8"
MIN_SOURCE_FONT_PT = 7.6
PAPER_WIDTH_SCALE = 0.94


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT.resolve()))
    except ValueError:
        return str(resolved)


def require_float(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def read_machine_fields(report_path: Path) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in report_path.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"([a-z0-9_]+)=(.*)", line.strip())
        if match:
            key, value = match.groups()
            if key in fields:
                raise ValueError(f"duplicate machine field {key}")
            fields[key] = value
    return fields


def load_evidence(evidence_path: Path) -> dict[str, object]:
    evidence_path = evidence_path.resolve()
    report_path = evidence_path.with_suffix(".md")
    if not report_path.is_file():
        raise FileNotFoundError(f"companion evidence report missing: {report_path}")

    with evidence_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 50:
        raise ValueError(f"expected 50 paired seeds, found {len(rows)}")

    seeds = [int(row["seed"]) for row in rows]
    if seeds != list(range(82, 132)):
        raise ValueError(f"unexpected confirmatory seed interval: {seeds[:1]}..{seeds[-1:]}")

    numeric_fields = (
        "full_attempted_bytes",
        "moment_attempted_bytes",
        "attempted_reduction_percent",
        "full_delivered_bytes",
        "moment_delivered_bytes",
        "delivered_reduction_percent",
        "local_eospa_delta",
        "consensus_ospa_delta",
        "consensus_position_delta",
        "consensus_cardinality_delta",
        "comparison_count",
        "snapshot_count",
        "max_abs_r",
        "max_abs_mu",
        "max_abs_sigma",
        "exact_match",
        "attempted_masks_equal",
        "delivered_masks_equal",
    )
    for row in rows:
        for field in numeric_fields:
            require_float(row, field)

        full_attempted = require_float(row, "full_attempted_bytes")
        moment_attempted = require_float(row, "moment_attempted_bytes")
        recalculated = 100.0 * (full_attempted - moment_attempted) / full_attempted
        if not math.isclose(
            recalculated,
            require_float(row, "attempted_reduction_percent"),
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(f"attempted reduction mismatch for seed {row['seed']}")
        for flag in ("exact_match", "attempted_masks_equal", "delivered_masks_equal"):
            if require_float(row, flag) != 1.0:
                raise ValueError(f"{flag} failed for seed {row['seed']}")

    attempted_reduction = [require_float(row, "attempted_reduction_percent") for row in rows]
    delivered_reduction = [require_float(row, "delivered_reduction_percent") for row in rows]
    mean_attempted = math.fsum(attempted_reduction) / len(attempted_reduction)
    machine = read_machine_fields(report_path)
    evidence_hash = sha256(evidence_path)
    if machine.get("csv_sha256") != evidence_hash:
        raise ValueError("evidence report does not bind the supplied CSV")
    report_mean = float(machine["aggregate_mean_attempted_reduction_percent"])
    if not math.isclose(mean_attempted, report_mean, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("CSV mean differs from the validated evidence report")

    exact_fields = (
        "local_eospa_delta",
        "consensus_ospa_delta",
        "consensus_position_delta",
        "consensus_cardinality_delta",
        "max_abs_r",
        "max_abs_mu",
        "max_abs_sigma",
    )
    max_deltas = {
        field: max(abs(require_float(row, field)) for row in rows) for field in exact_fields
    }
    if any(value != 0.0 for value in max_deltas.values()):
        raise ValueError(f"confirmatory equivalence is not exact: {max_deltas}")

    return {
        "evidence_path": evidence_path,
        "report_path": report_path,
        "rows": rows,
        "seeds": seeds,
        "attempted_reduction": attempted_reduction,
        "delivered_reduction": delivered_reduction,
        "mean_attempted": mean_attempted,
        "ci_low": float(machine["aggregate_bootstrap_ci_low_percent"]),
        "ci_high": float(machine["aggregate_bootstrap_ci_high_percent"]),
        "total_comparisons": int(float(machine["aggregate_total_comparison_count"])),
        "total_snapshots": int(float(machine["aggregate_total_snapshot_count"])),
        "max_deltas": max_deltas,
    }


def configure_matplotlib() -> None:
    mpl.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["DejaVu Sans"],
            "font.size": 8.0,
            "axes.labelsize": 8.0,
            "axes.titlesize": 9.0,
            "xtick.labelsize": MIN_SOURCE_FONT_PT,
            "ytick.labelsize": MIN_SOURCE_FONT_PT,
            "legend.fontsize": MIN_SOURCE_FONT_PT,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.linewidth": 0.65,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "savefig.facecolor": "white",
        }
    )


def rounded_box(
    ax: plt.Axes,
    center: tuple[float, float],
    width: float,
    height: float,
    text: str,
    facecolor: str,
    edgecolor: str,
    fontsize: float = 7.8,
    linewidth: float = 0.9,
) -> None:
    x, y = center
    patch = FancyBboxPatch(
        (x - width / 2, y - height / 2),
        width,
        height,
        boxstyle="round,pad=0.012,rounding_size=0.018",
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=linewidth,
        transform=ax.transAxes,
        clip_on=False,
    )
    ax.add_patch(patch)
    ax.text(
        x,
        y,
        text,
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=fontsize,
        color=TEXT_COLOR,
        linespacing=1.12,
    )


def arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str = MUTED_COLOR,
    style: str = "-|>",
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle=style,
            mutation_scale=9,
            linewidth=0.9,
            color=color,
            transform=ax.transAxes,
            shrinkA=1,
            shrinkB=1,
            clip_on=False,
        )
    )


def save_figure(fig: plt.Figure, pdf_path: Path, png_path: Path) -> None:
    pdf_metadata = {
        "Creator": "ICASSP evidence figure generator",
        "Producer": "Matplotlib",
        "CreationDate": None,
        "ModDate": None,
    }
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.02, metadata=pdf_metadata)
    fig.savefig(
        png_path,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.02,
        metadata={"Software": "Matplotlib"},
    )
    plt.close(fig)


def make_mechanism_figure(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.05, 2.15))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    input_x = 0.105
    output_x = 0.91
    top_y = 0.69
    bottom_y = 0.30
    rounded_box(
        ax,
        (input_x, 0.50),
        0.17,
        0.34,
        "Full GM message\n" + r"$\ell, r, \{w_m,\mu_m,\Sigma_m\}$",
        "#F1F3F5",
        FULL_COLOR,
        7.8,
    )
    rounded_box(
        ax,
        (0.35, top_y),
        0.20,
        0.28,
        "Receiver-side\nprojection\n" + r"$\mathcal{P}$",
        "#EAF3FA",
        MOMENT_COLOR,
        8.0,
    )
    rounded_box(
        ax,
        (0.62, top_y),
        0.18,
        0.28,
        "Projected fusion\n" + r"$\mathcal{G}_{\omega}$",
        "#F1ECF8",
        FUSION_COLOR,
        8.0,
    )
    rounded_box(
        ax,
        (0.32, bottom_y),
        0.20,
        0.28,
        "Sender-side\nprojection\n" + r"$\mathcal{P}$",
        "#EAF3FA",
        MOMENT_COLOR,
        MIN_SOURCE_FONT_PT,
    )
    rounded_box(
        ax,
        (0.54, bottom_y),
        0.20,
        0.28,
        "Fusion-sufficient\nmoment message\n" + r"encode $\rightarrow$ decode",
        "#FFF4DF",
        WIRE_COLOR,
        7.8,
    )
    rounded_box(
        ax,
        (0.75, bottom_y),
        0.14,
        0.28,
        "Projected\nfusion\n" + r"$\mathcal{G}_{\omega}$",
        "#F1ECF8",
        FUSION_COLOR,
        7.8,
    )
    rounded_box(
        ax,
        (output_x, 0.50),
        0.13,
        0.34,
        "Same fused\noutput",
        "#EAF5F0",
        OUTPUT_COLOR,
        8.0,
    )

    arrow(ax, (0.19, 0.56), (0.25, top_y))
    arrow(ax, (0.19, 0.44), (0.22, bottom_y))
    arrow(ax, (0.45, top_y), (0.53, top_y))
    arrow(ax, (0.71, top_y), (0.845, 0.57))
    arrow(ax, (0.42, bottom_y), (0.44, bottom_y))
    arrow(ax, (0.64, bottom_y), (0.68, bottom_y))
    arrow(ax, (0.82, bottom_y), (0.845, 0.43))

    ax.text(0.91, 0.82, r"$\mathcal{F}(X)=\mathcal{F}(\mathcal{P}X)$", ha="center",
            va="center", transform=ax.transAxes, color=OUTPUT_COLOR,
            fontsize=8.8, fontweight="bold")
    ax.text(
        0.50,
        0.01,
        "Same labels, weights, schedule, delivery masks, and canonical projection; no quantization or covariance inflation.",
        ha="center",
        va="bottom",
        transform=ax.transAxes,
        color=MUTED_COLOR,
        fontsize=MIN_SOURCE_FONT_PT,
    )

    save_figure(
        fig,
        output_dir / "payload_graph_schematic.pdf",
        output_dir / "payload_graph_schematic.png",
    )


def make_evidence_figure(output_dir: Path, evidence: dict[str, object]) -> None:
    seeds = evidence["seeds"]
    attempted = evidence["attempted_reduction"]
    mean_attempted = float(evidence["mean_attempted"])
    ci_low = float(evidence["ci_low"])
    ci_high = float(evidence["ci_high"])
    rows = evidence["rows"]
    full_attempted_mb = [
        require_float(row, "full_attempted_bytes") / 1e6 for row in rows
    ]
    moment_attempted_mb = [
        require_float(row, "moment_attempted_bytes") / 1e6 for row in rows
    ]
    mean_full_mb = math.fsum(full_attempted_mb) / len(full_attempted_mb)
    mean_moment_mb = math.fsum(moment_attempted_mb) / len(moment_attempted_mb)
    mean_saved_mb = mean_full_mb - mean_moment_mb

    fig, (ax, payload) = plt.subplots(
        1,
        2,
        figsize=(7.05, 2.55),
        gridspec_kw={"width_ratios": [1.02, 1.18], "wspace": 0.30},
    )

    ax.fill_between([seeds[0], seeds[-1]], ci_low, ci_high,
                    color="#D9EAF5", alpha=0.9, linewidth=0, label="95% bootstrap CI")
    ax.plot(seeds, attempted, color=MOMENT_COLOR, linewidth=0.9,
            marker="o", markersize=2.4, markeredgewidth=0,
            label="Attempted")
    ax.axhline(mean_attempted, color="#174A6E", linewidth=1.0, linestyle="--")
    ax.text(
        0.985,
        0.97,
        f"mean 58.28%\n95% CI [{ci_low:.2f}, {ci_high:.2f}]%",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=MIN_SOURCE_FONT_PT,
        color="#174A6E",
    )
    ax.set_xlim(seeds[0] - 1, seeds[-1] + 1)
    ax.set_ylim(54.8, 62.2)
    ax.set_xticks([82, 90, 100, 110, 120, 131])
    ax.set_xlabel("Paired confirmatory seed")
    ax.set_ylabel("Application-layer byte reduction (%)")
    ax.grid(axis="y", color=GRID_COLOR, linewidth=0.55)
    ax.set_title("a  Per-seed communication reduction", loc="left", fontweight="bold")

    order = sorted(range(len(full_attempted_mb)), key=full_attempted_mb.__getitem__)
    ranked_full_mb = [full_attempted_mb[index] for index in order]
    ranked_moment_mb = [moment_attempted_mb[index] for index in order]
    ranks = list(range(1, len(order) + 1))
    payload.hlines(
        ranks,
        ranked_moment_mb,
        ranked_full_mb,
        color="#B9D5E6",
        linewidth=0.75,
        zorder=1,
    )
    payload.scatter(
        ranked_full_mb,
        ranks,
        s=10,
        color=FULL_COLOR,
        edgecolors="white",
        linewidths=0.25,
        label=f"Full GM (mean {mean_full_mb:.2f} MB)",
        zorder=3,
    )
    payload.scatter(
        ranked_moment_mb,
        ranks,
        s=10,
        color=MOMENT_COLOR,
        edgecolors="white",
        linewidths=0.25,
        label=f"Moment (mean {mean_moment_mb:.2f} MB)",
        zorder=3,
    )
    payload.set_xlim(7.0, 36.0)
    payload.set_ylim(0, len(order) + 1)
    payload.set_xticks([10, 15, 20, 25, 30, 35])
    payload.set_yticks([])
    payload.set_xlabel("Attempted payload (MB/trial)")
    payload.set_ylabel("50 paired trials (sorted)")
    payload.grid(axis="x", color=GRID_COLOR, linewidth=0.55)
    payload.legend(loc="lower right", frameon=False, handletextpad=0.45)
    payload.text(
        0.03,
        0.97,
        f"mean saved {mean_saved_mb:.2f} MB/trial",
        transform=payload.transAxes,
        ha="left",
        va="top",
        fontsize=MIN_SOURCE_FONT_PT,
        color="#174A6E",
        fontweight="bold",
    )
    payload.set_title("b  Paired absolute payloads", loc="left", fontweight="bold")

    save_figure(
        fig,
        output_dir / "heldout_tradeoff.pdf",
        output_dir / "heldout_tradeoff.png",
    )


def write_manifest(output_dir: Path, evidence: dict[str, object]) -> None:
    evidence_path = Path(evidence["evidence_path"])
    report_path = Path(evidence["report_path"])
    max_state_residual = max(float(value) for value in evidence["max_deltas"].values())
    outputs = {
        name: {"sha256": sha256(output_dir / name), "bytes": (output_dir / name).stat().st_size}
        for name in FIGURE_NAMES
    }
    manifest = {
        "schema": "icassp2027-figure-manifest-v1",
        "backend": "python-matplotlib",
        "layout_qa": {
            "minimum_source_font_points": MIN_SOURCE_FONT_PT,
            "paper_width_scale": PAPER_WIDTH_SCALE,
            "minimum_estimated_final_font_points": (
                MIN_SOURCE_FONT_PT * PAPER_WIDTH_SCALE
            ),
        },
        "contract": {
            "figure_1": "Sender-side moment projection commutes with the specified projected receiver fusion.",
            "figure_2": "Relative and paired absolute views quantify workload-scoped application-layer savings.",
        },
        "evidence": {
            "path": stable_path(evidence_path),
            "sha256": sha256(evidence_path),
            "report_path": stable_path(report_path),
            "report_sha256": sha256(report_path),
            "row_count": len(evidence["rows"]),
            "seed_interval": [evidence["seeds"][0], evidence["seeds"][-1]],
        },
        "generator_sha256": sha256(Path(__file__)),
        "summary": {
            "mean_attempted_reduction_percent": evidence["mean_attempted"],
            "mean_full_attempted_mb": math.fsum(
                require_float(row, "full_attempted_bytes") for row in evidence["rows"]
            ) / len(evidence["rows"]) / 1e6,
            "mean_moment_attempted_mb": math.fsum(
                require_float(row, "moment_attempted_bytes") for row in evidence["rows"]
            ) / len(evidence["rows"]) / 1e6,
            "bootstrap_ci_percent": [evidence["ci_low"], evidence["ci_high"]],
            "all_exact_match": True,
            "all_masks_equal": True,
            "max_state_residual": max_state_residual,
            "total_snapshots": evidence["total_snapshots"],
            "total_label_comparisons": evidence["total_comparisons"],
        },
        "outputs": outputs,
    }
    (output_dir / "figure_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_matplotlib()
    evidence = load_evidence(args.evidence)
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    make_mechanism_figure(output_dir)
    make_evidence_figure(output_dir, evidence)
    write_manifest(output_dir, evidence)


if __name__ == "__main__":
    main()
