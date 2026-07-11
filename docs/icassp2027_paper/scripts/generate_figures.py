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
from pypdf import PdfReader


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
MECHANISM_MIN_SOURCE_FONT_PT = 7.6
EVIDENCE_MIN_SOURCE_FONT_PT = 8.5
GUIDELINE_MIN_FINAL_FONT_PT = 9.0
PAPER_TEXT_WIDTH_MM = 178.0
PAPER_WIDTH_SCALE = 0.94
TARGET_INCLUDE_WIDTH_POINTS = (
    PAPER_TEXT_WIDTH_MM / 25.4 * 72.0 * PAPER_WIDTH_SCALE
)


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
            "font.size": EVIDENCE_MIN_SOURCE_FONT_PT,
            "axes.labelsize": EVIDENCE_MIN_SOURCE_FONT_PT,
            "axes.titlesize": 9.4,
            "xtick.labelsize": EVIDENCE_MIN_SOURCE_FONT_PT,
            "ytick.labelsize": EVIDENCE_MIN_SOURCE_FONT_PT,
            "legend.fontsize": EVIDENCE_MIN_SOURCE_FONT_PT,
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
    fig, ax = plt.subplots(figsize=(7.05, 1.85))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    input_x = 0.09
    route_x = 0.31
    moments_x = 0.56
    fusion_x = 0.76
    output_x = 0.94
    top_y = 0.68
    bottom_y = 0.32
    rounded_box(
        ax,
        (input_x, 0.50),
        0.16,
        0.34,
        "Local GM-LMB\nposterior\n" + r"$\ell,r,\{w_m,\mu_m,\Sigma_m\}$",
        "#F1F3F5",
        FULL_COLOR,
        7.8,
    )
    rounded_box(
        ax,
        (route_x, top_y),
        0.19,
        0.28,
        "Full-GM codec " + r"$\mathcal{T}$" + "\n+ receiver projection\n" + r"$\mathcal{P}$",
        "#EAF3FA",
        MOMENT_COLOR,
        8.0,
    )
    rounded_box(
        ax,
        (route_x, bottom_y),
        0.19,
        0.28,
        "Sender projection " + r"$\mathcal{P}$" + "\n+ moment codec\n" + r"$\mathcal{T}$",
        "#FFF4DF",
        WIRE_COLOR,
        8.0,
    )
    rounded_box(
        ax,
        (moments_x, 0.50),
        0.17,
        0.34,
        "Identical receiver\ninputs\n" + r"$\ell,r,\mu,\Sigma$",
        "#EAF5F0",
        OUTPUT_COLOR,
        MECHANISM_MIN_SOURCE_FONT_PT,
    )
    rounded_box(
        ax,
        (fusion_x, 0.50),
        0.13,
        0.34,
        "Projected\nfusion\n" + r"$\mathcal{G}_{\omega,\mathcal{R}}$",
        "#F1ECF8",
        FUSION_COLOR,
        7.8,
    )
    rounded_box(
        ax,
        (output_x, 0.50),
        0.105,
        0.34,
        "Same fused\nLMB output",
        "#EAF5F0",
        OUTPUT_COLOR,
        7.8,
    )

    arrow(ax, (0.17, 0.56), (0.215, top_y))
    arrow(ax, (0.17, 0.44), (0.215, bottom_y))
    arrow(ax, (0.405, top_y), (0.475, 0.56))
    arrow(ax, (0.405, bottom_y), (0.475, 0.44))
    arrow(ax, (0.645, 0.50), (0.695, 0.50))
    arrow(ax, (0.825, 0.50), (0.885, 0.50))

    ax.text(0.83, 0.82,
            r"$\mathcal{F}_{\omega,\mathcal{R}}(\mathcal{T}\pi)="
            r"\mathcal{F}_{\omega,\mathcal{R}}(\mathcal{T}\mathcal{P}\pi)$",
            ha="center",
            va="center", transform=ax.transAxes, color=OUTPUT_COLOR,
            fontsize=8.8, fontweight="bold")
    save_figure(
        fig,
        output_dir / "payload_graph_schematic.pdf",
        output_dir / "payload_graph_schematic.png",
    )


def make_evidence_figure(output_dir: Path, evidence: dict[str, object]) -> None:
    mean_attempted = float(evidence["mean_attempted"])
    mean_delivered = math.fsum(evidence["delivered_reduction"]) / len(
        evidence["delivered_reduction"]
    )
    rows = evidence["rows"]
    full_attempted_mb = [
        require_float(row, "full_attempted_bytes") / 1e6 for row in rows
    ]
    moment_attempted_mb = [
        require_float(row, "moment_attempted_bytes") / 1e6 for row in rows
    ]
    full_delivered_mb = [
        require_float(row, "full_delivered_bytes") / 1e6 for row in rows
    ]
    moment_delivered_mb = [
        require_float(row, "moment_delivered_bytes") / 1e6 for row in rows
    ]
    mean_full_mb = math.fsum(full_attempted_mb) / len(full_attempted_mb)
    mean_moment_mb = math.fsum(moment_attempted_mb) / len(moment_attempted_mb)
    mean_full_delivered_mb = math.fsum(full_delivered_mb) / len(full_delivered_mb)
    mean_moment_delivered_mb = (
        math.fsum(moment_delivered_mb) / len(moment_delivered_mb)
    )
    minimum_reduction = min(float(value) for value in evidence["attempted_reduction"])
    maximum_reduction = max(float(value) for value in evidence["attempted_reduction"])

    fig, (footprint, parity) = plt.subplots(
        1,
        2,
        figsize=(7.05, 2.55),
        gridspec_kw={"width_ratios": [1.06, 0.94], "wspace": 0.30},
    )

    row_y = [1.0, 0.0]
    full_means = [mean_full_mb, mean_full_delivered_mb]
    moment_means = [mean_moment_mb, mean_moment_delivered_mb]
    reductions = [mean_attempted, mean_delivered]
    footprint.barh(
        [value + 0.14 for value in row_y],
        full_means,
        height=0.24,
        color=FULL_COLOR,
        edgecolor="none",
        zorder=2,
    )
    footprint.barh(
        [value - 0.14 for value in row_y],
        moment_means,
        height=0.24,
        color=MOMENT_COLOR,
        edgecolor="none",
        zorder=2,
    )
    for y, full_value, moment_value, reduction in zip(
        row_y, full_means, moment_means, reductions
    ):
        footprint.text(
            0.55,
            y + 0.14,
            "Full GM",
            ha="left",
            va="center",
            color="white",
            fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
            fontweight="bold",
        )
        footprint.text(
            full_value - 0.45,
            y + 0.14,
            f"{full_value:.2f}",
            ha="right",
            va="center",
            color="white",
            fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
            fontweight="bold",
        )
        footprint.text(
            0.55,
            y - 0.14,
            "Moment",
            ha="left",
            va="center",
            color="white",
            fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
            fontweight="bold",
        )
        footprint.text(
            moment_value + 0.35,
            y - 0.14,
            f"{moment_value:.2f}",
            ha="left",
            va="center",
            color=MOMENT_COLOR,
            fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
            fontweight="bold",
        )
        footprint.text(
            30.2,
            y,
            f"{reduction:.2f}%",
            ha="right",
            va="center",
            color="#174A6E",
            fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
            fontweight="bold",
        )
    footprint.set_yticks(row_y)
    footprint.set_yticklabels(["Attempted", "Delivered"])
    footprint.set_xlim(0.0, 31.0)
    footprint.set_xticks([0, 10, 20, 30])
    footprint.set_xlabel(r"Mean payload ($10^6$ bytes/trial)")
    footprint.grid(axis="x", color=GRID_COLOR, linewidth=0.50)
    footprint.set_axisbelow(True)
    footprint.set_title("a  Communication footprint", loc="left", fontweight="bold")

    limit = [0.0, 36.0]
    parity.fill_between(limit, 0.0, limit, color="#F2F7FA", zorder=0)
    parity.plot(
        limit,
        limit,
        color="#8B949E",
        linestyle="--",
        linewidth=0.95,
        zorder=1,
    )
    parity.scatter(
        full_attempted_mb,
        moment_attempted_mb,
        s=21,
        color=MOMENT_COLOR,
        alpha=0.62,
        edgecolors="white",
        linewidths=0.45,
        zorder=3,
    )
    parity.text(
        0.04,
        0.96,
        "50/50 paired trials\nbelow identity",
        transform=parity.transAxes,
        ha="left",
        va="top",
        color=TEXT_COLOR,
        fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
    )
    parity.text(
        0.96,
        0.025,
        f"reduction range\n{minimum_reduction:.2f}-{maximum_reduction:.2f}%",
        transform=parity.transAxes,
        ha="right",
        va="bottom",
        color="#174A6E",
        fontsize=EVIDENCE_MIN_SOURCE_FONT_PT,
        fontweight="bold",
    )
    parity.set_xlim(limit)
    parity.set_ylim(limit)
    parity.set_xticks([0, 10, 20, 30])
    parity.set_yticks([0, 10, 20, 30])
    parity.set_aspect("equal", adjustable="box")
    parity.set_xlabel(r"Full-GM attempted ($10^6$ B/trial)")
    parity.set_ylabel(r"Moment attempted ($10^6$ B/trial)")
    parity.set_title("b  Paired trial footprint", loc="left", fontweight="bold")

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
    source_font_points = {
        "payload_graph_schematic.pdf": MECHANISM_MIN_SOURCE_FONT_PT,
        "heldout_tradeoff.pdf": EVIDENCE_MIN_SOURCE_FONT_PT,
    }
    figure_layout = {}
    for name, minimum_source_font in source_font_points.items():
        media_box = PdfReader(output_dir / name).pages[0].mediabox
        source_width_points = float(media_box.width)
        estimated_scale = TARGET_INCLUDE_WIDTH_POINTS / source_width_points
        figure_layout[name] = {
            "minimum_source_font_points": minimum_source_font,
            "source_width_points": source_width_points,
            "estimated_include_scale": estimated_scale,
            "minimum_estimated_final_font_points": (
                minimum_source_font * estimated_scale
            ),
        }
    minimum_final_font = min(
        item["minimum_estimated_final_font_points"]
        for item in figure_layout.values()
    )
    manifest = {
        "schema": "icassp2027-figure-manifest-v2",
        "backend": "python-matplotlib",
        "layout_qa": {
            "guideline_minimum_final_font_points": GUIDELINE_MIN_FINAL_FONT_PT,
            "paper_text_width_mm": PAPER_TEXT_WIDTH_MM,
            "paper_width_scale": PAPER_WIDTH_SCALE,
            "target_include_width_points": TARGET_INCLUDE_WIDTH_POINTS,
            "minimum_estimated_final_font_points": minimum_final_font,
            "figures": figure_layout,
        },
        "contract": {
            "figure_1": "Canonical transport and sender-side moment projection produce identical inputs to the specified receiver.",
            "figure_2": "Mean attempted/delivered footprints and paired trials quantify application-layer savings without mixing aggregate ratio definitions.",
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
            "mean_full_delivered_mb": math.fsum(
                require_float(row, "full_delivered_bytes") for row in evidence["rows"]
            ) / len(evidence["rows"]) / 1e6,
            "mean_moment_delivered_mb": math.fsum(
                require_float(row, "moment_delivered_bytes") for row in evidence["rows"]
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
