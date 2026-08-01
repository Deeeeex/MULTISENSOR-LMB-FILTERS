#!/usr/bin/env python3
"""Render the tested multistyle formation-FoV geometry with Matplotlib."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Wedge


# Mandatory editable-vector typography contract.
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = [
    "Arial",
    "DejaVu Sans",
    "Liberation Sans",
]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["font.size"] = 7
plt.rcParams["axes.linewidth"] = 0.8
plt.rcParams["legend.frameon"] = False
plt.rcParams["xtick.major.width"] = 0.7
plt.rcParams["ytick.major.width"] = 0.7


COLORS = {
    "sensor": "#0F4D92",
    "formation_path": "#3775BA",
    "formation_start": "#B4C0E4",
    "target": "#B64342",
    "target_path": "#E9A6A1",
    "fov": "#42949E",
    "backbone": "#767676",
    "axis": "#4D4D4D",
    "panel_text": "#272727",
}


STYLE_TITLES = {
    "parallel-convoy": "Parallel convoy",
    "orthogonal-crossing": "Orthogonal crossing",
    "linear-relay": "Linear relay",
}

# A common metre-scale frame makes FoV range and formation spacing directly
# comparable across all six panels while keeping every trajectory in view.
COMMON_X_LIMITS = (-820.0, 820.0)
COMMON_Y_LIMITS = (-650.0, 650.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=(
            "RUN/GA/dynamic_topology/figures/source/"
            "formation_fov_multistyle_suite_seed41.json"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "RUN/GA/dynamic_topology/figures/"
            "formation_fov_multistyle_suite_v1"
        ),
        help="Output path without extension.",
    )
    return parser.parse_args()


def as_float_array(values: list) -> np.ndarray:
    """Convert JSON nulls in inactive target intervals to NaN."""

    return np.asarray(
        [
            [np.nan if value is None else float(value) for value in row]
            for row in values
        ],
        dtype=float,
    )


def load_source(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        source = json.load(stream)
    if source.get("contractVersion") != (
        "formation-fov-multistyle-figure-source-v1"
    ):
        raise ValueError("Unexpected multistyle figure-source contract.")
    if source.get("rendererContract") != "python-matplotlib-only":
        raise ValueError("Figure source does not authorize the Python renderer.")
    if source.get("truthOutcomeUsed") or source.get("trackingResultUsed"):
        raise ValueError("Scenario schematic may not consume tracking outcomes.")
    if len(source.get("scenes", [])) != 6:
        raise ValueError("Expected exactly six multistyle scene panels.")
    return source


def add_panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.08,
        1.03,
        label,
        transform=ax.transAxes,
        fontsize=9,
        fontweight="bold",
        color=COLORS["panel_text"],
        ha="left",
        va="bottom",
    )


def draw_fov_wedges(
    ax: plt.Axes,
    sensor_x: np.ndarray,
    sensor_y: np.ndarray,
    headings: np.ndarray,
    snapshot_index: int,
    half_angle: float,
    radius: float,
) -> None:
    for sensor_idx, heading in enumerate(headings):
        heading_deg = np.degrees(heading)
        wedge = Wedge(
            (sensor_x[sensor_idx, snapshot_index], sensor_y[sensor_idx, snapshot_index]),
            radius,
            heading_deg - half_angle,
            heading_deg + half_angle,
            facecolor=COLORS["fov"],
            edgecolor=COLORS["fov"],
            linewidth=0.22,
            alpha=0.035,
            zorder=0,
        )
        ax.add_patch(wedge)


def draw_scene(ax: plt.Axes, scene: dict, show_title: bool) -> None:
    sensor_x = as_float_array(scene["sensorX"])
    sensor_y = as_float_array(scene["sensorY"])
    target_x = as_float_array(scene["targetX"])
    target_y = as_float_array(scene["targetY"])
    center_x = as_float_array(scene["formationCenterX"])
    center_y = as_float_array(scene["formationCenterY"])
    headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
    formation_adjacency = np.asarray(scene["formationAdjacency"], dtype=bool)
    snapshot_index = int(scene["snapshotTime"]) - 1

    draw_fov_wedges(
        ax,
        sensor_x,
        sensor_y,
        headings,
        snapshot_index,
        float(scene["fovHalfAngleDeg"]),
        float(scene["fovRange"]),
    )

    # Formation-centre paths reveal platform motion without duplicating all
    # six nearly parallel sensor traces in each formation.
    for formation_idx in range(int(scene["formationCount"])):
        ax.plot(
            center_x[formation_idx],
            center_y[formation_idx],
            color=COLORS["formation_path"],
            linewidth=0.85,
            alpha=0.78,
            zorder=1,
        )
        ax.scatter(
            center_x[formation_idx, 0],
            center_y[formation_idx, 0],
            s=12,
            marker="o",
            facecolor="white",
            edgecolor=COLORS["formation_start"],
            linewidth=0.8,
            zorder=3,
        )
        left = max(snapshot_index - 5, 0)
        right = min(snapshot_index + 5, center_x.shape[1] - 1)
        ax.annotate(
            "",
            xy=(center_x[formation_idx, right], center_y[formation_idx, right]),
            xytext=(center_x[formation_idx, left], center_y[formation_idx, left]),
            arrowprops={
                "arrowstyle": "-|>",
                "color": COLORS["formation_path"],
                "lw": 0.75,
                "mutation_scale": 6,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=3,
        )

    # The generated static graph may contain several sensor-level bridges;
    # collapse only their formation-level incidence for a readable schematic.
    for left in range(formation_adjacency.shape[0] - 1):
        for right in range(left + 1, formation_adjacency.shape[1]):
            if formation_adjacency[left, right]:
                ax.plot(
                    [center_x[left, snapshot_index], center_x[right, snapshot_index]],
                    [center_y[left, snapshot_index], center_y[right, snapshot_index]],
                    color=COLORS["backbone"],
                    linewidth=0.55,
                    linestyle=(0, (2.2, 2.2)),
                    alpha=0.65,
                    zorder=2,
                )

    for target_idx in range(target_x.shape[0]):
        active = np.isfinite(target_x[target_idx]) & np.isfinite(target_y[target_idx])
        ax.plot(
            target_x[target_idx, active],
            target_y[target_idx, active],
            color=COLORS["target_path"],
            linewidth=0.38,
            alpha=0.48,
            zorder=1,
        )

    ax.scatter(
        sensor_x[:, snapshot_index],
        sensor_y[:, snapshot_index],
        s=8,
        marker="o",
        facecolor=COLORS["sensor"],
        edgecolor="white",
        linewidth=0.28,
        zorder=5,
    )
    active_targets = np.isfinite(target_x[:, snapshot_index]) & np.isfinite(
        target_y[:, snapshot_index]
    )
    ax.scatter(
        target_x[active_targets, snapshot_index],
        target_y[active_targets, snapshot_index],
        s=9,
        marker="D",
        facecolor=COLORS["target"],
        edgecolor="white",
        linewidth=0.25,
        zorder=6,
    )

    ax.set_xlim(COMMON_X_LIMITS)
    ax.set_ylim(COMMON_Y_LIMITS)
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(
        axis="both",
        labelsize=5.5,
        length=2.2,
        pad=1.5,
        colors=COLORS["axis"],
    )
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#A8A8A8")
    ax.spines["bottom"].set_color("#A8A8A8")
    if show_title:
        ax.set_title(
            STYLE_TITLES[scene["sceneStyle"]],
            fontsize=8,
            fontweight="bold",
            pad=4,
            color=COLORS["panel_text"],
        )
    ax.text(
        0.02,
        0.975,
        (
            f"{scene['nodeCount']} sensors · "
            f"{scene['formationCount']} formations · "
            f"{scene['targetCount']} targets"
        ),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=5.4,
        color=COLORS["axis"],
        zorder=8,
    )
    add_panel_label(ax, scene["panelLabel"])


def build_figure(source: dict) -> plt.Figure:
    figure = plt.figure(figsize=(7.2, 5.15), facecolor="white")
    grid = figure.add_gridspec(
        2,
        3,
        height_ratios=[1.0, 1.0],
        left=0.075,
        right=0.985,
        top=0.945,
        bottom=0.145,
        hspace=0.06,
        wspace=0.19,
    )
    axes = []
    for scene_idx, scene in enumerate(source["scenes"]):
        row, column = divmod(scene_idx, 3)
        ax = figure.add_subplot(grid[row, column])
        draw_scene(ax, scene, show_title=(row == 0))
        if row == 1:
            ax.set_xlabel("x (m)", fontsize=6.5, labelpad=1.5)
        if column == 0:
            ax.set_ylabel("y (m)", fontsize=6.5, labelpad=1.5)
        axes.append(ax)

    figure.text(
        0.012,
        0.69,
        "X36",
        rotation=90,
        ha="center",
        va="center",
        fontsize=8,
        fontweight="bold",
        color=COLORS["panel_text"],
    )
    figure.text(
        0.012,
        0.31,
        "M24",
        rotation=90,
        ha="center",
        va="center",
        fontsize=8,
        fontweight="bold",
        color=COLORS["panel_text"],
    )

    legend_handles = [
        Line2D(
            [0],
            [0],
            marker="o",
            linestyle="none",
            markerfacecolor=COLORS["sensor"],
            markeredgecolor="white",
            markersize=5,
            label="Sensor at t=80",
        ),
        Line2D(
            [0],
            [0],
            color=COLORS["formation_path"],
            linewidth=1.1,
            label="Formation-centre path",
        ),
        Line2D(
            [0],
            [0],
            marker="D",
            linestyle="none",
            markerfacecolor=COLORS["target"],
            markeredgecolor="white",
            markersize=4.5,
            label="Target at t=80",
        ),
        Line2D(
            [0],
            [0],
            color=COLORS["target_path"],
            linewidth=0.9,
            label="Target path",
        ),
        Line2D(
            [0],
            [0],
            color=COLORS["fov"],
            linewidth=3.2,
            alpha=0.28,
            label="Exact FoV (120° / 300 m)",
        ),
        Line2D(
            [0],
            [0],
            color=COLORS["backbone"],
            linewidth=0.9,
            linestyle=(0, (2.2, 2.2)),
            label="Static formation link",
        ),
    ]
    figure.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.53, 0.035),
        ncol=3,
        fontsize=6.1,
        handlelength=2.0,
        columnspacing=1.25,
    )
    return figure


def save_figure(figure: plt.Figure, output_base: Path) -> list[Path]:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_base.with_suffix(".svg"),
        output_base.with_suffix(".pdf"),
        output_base.with_suffix(".png"),
    ]
    # Preserve the registered 7.2 x 5.15 inch canvas exactly.  Every artist is
    # laid out inside that canvas, so tight bounding-box expansion would only
    # make the delivered page size drift from the figure contract.
    figure.savefig(outputs[0])
    figure.savefig(outputs[1])
    figure.savefig(outputs[2], dpi=300)
    plt.close(figure)
    # Matplotlib writes path commands with spaces before line breaks.  Remove
    # only that insignificant whitespace so the generated SVG also satisfies
    # the repository's diff-check contract.
    svg_text = outputs[0].read_text(encoding="utf-8")
    outputs[0].write_text(
        "\n".join(line.rstrip() for line in svg_text.splitlines()) + "\n",
        encoding="utf-8",
    )
    return outputs


def main() -> None:
    args = parse_args()
    source = load_source(Path(args.data))
    figure = build_figure(source)
    outputs = save_figure(figure, Path(args.output))
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
