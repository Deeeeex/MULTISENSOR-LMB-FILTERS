#!/usr/bin/env python3
"""Render the exact V276 zipper-merge geometry and structural stress."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgba
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Wedge
from matplotlib.ticker import MaxNLocator


plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Arial", "DejaVu Sans", "Liberation Sans"]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["svg.hashsalt"] = "zipper-merge-v276"
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["font.size"] = 7
plt.rcParams["axes.linewidth"] = 0.7
plt.rcParams["xtick.major.width"] = 0.6
plt.rcParams["ytick.major.width"] = 0.6


COLORS = {
    "sensor": "#174A7E",
    "sensor_ring": "#8FB4D4",
    "target": "#D46A36",
    "fov": "#2A9D8F",
    "tree": "#59636E",
    "broken": "#C44E52",
    "alternative": "#17806D",
    "intact_band": "#DCEAF4",
    "failure_band": "#F6D7D5",
    "handover": "#D98400",
    "axis": "#4F565D",
    "spine": "#B9BEC4",
    "text": "#22262A",
}


EXPECTED_PRESETS = [
    "m24-formation-fov-zipper-merge",
    "x36-formation-fov-zipper-merge",
]
EXPECTED_SNAPSHOTS = [1, 80, 160]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=(
            "RUN/GA/dynamic_topology/figures/source/"
            "zipper_merge_v276_seed1301.json"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "RUN/GA/dynamic_topology/figures/"
            "zipper_merge_v276_scene"
        ),
        help="Output path without extension.",
    )
    return parser.parse_args()


def as_float_array(values: list) -> np.ndarray:
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
    if source.get("contractVersion") != "zipper-merge-v276-figure-source-v1":
        raise ValueError("Unexpected V276 figure-source contract.")
    if source.get("rendererContract") != "python-matplotlib-only":
        raise ValueError("V276 source does not authorize this renderer.")
    if not source.get("geometryTruthUsed") or not source.get(
        "physicalGraphTruthUsed"
    ):
        raise ValueError("V276 figure requires exact geometry and physical graph data.")
    if source.get("posteriorUsed") or source.get("trackingResultUsed"):
        raise ValueError("V276 scene figure must not consume tracking outcomes.")
    if source.get("snapshotTimes") != EXPECTED_SNAPSHOTS:
        raise ValueError("V276 snapshot times changed.")
    scenes = source.get("scenes", [])
    if [scene.get("presetName") for scene in scenes] != EXPECTED_PRESETS:
        raise ValueError("V276 figure requires the paired M24/X36 presets.")
    for scene in scenes:
        if scene.get("sceneStyle") != "zipper-merge":
            raise ValueError("V276 source is not the zipper-merge geometry.")
        if float(scene.get("fovTotalAngleDeg", 0.0)) != 120.0:
            raise ValueError("V276 source changed the 120-degree FoV.")
        if float(scene.get("fovRange", 0.0)) != 300.0:
            raise ValueError("V276 source changed the 300 m sensing range.")
        if int(scene.get("sensorsPerFormation", 0)) != 6:
            raise ValueError("V276 source changed the six-sensor formation.")
        if not all(scene.get("formationConnectedByTime", [])):
            raise ValueError("The physical formation graph is not always connected.")
        feasible = np.asarray(scene.get("initialTreeFeasibleByTime", []), dtype=bool)
        if feasible.size != int(scene["simulationLength"]):
            raise ValueError("V276 feasibility trace length is inconsistent.")
        if np.all(feasible):
            raise ValueError("V276 source contains no fixed-tree failure.")
    return source


def add_panel_label(
    ax: plt.Axes, label: str, x_offset: float = -0.08
) -> None:
    ax.text(
        x_offset,
        1.03,
        label,
        transform=ax.transAxes,
        fontsize=8.5,
        fontweight="bold",
        color=COLORS["text"],
        ha="left",
        va="bottom",
    )


def draw_edge(
    ax: plt.Axes,
    centres_x: np.ndarray,
    centres_y: np.ndarray,
    left: int,
    right: int,
    color: str,
    linestyle: str | tuple,
    linewidth: float,
    zorder: int,
) -> None:
    ax.plot(
        [centres_x[left], centres_x[right]],
        [centres_y[left], centres_y[right]],
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
        solid_capstyle="round",
        dash_capstyle="round",
        zorder=zorder,
    )


def draw_network_state(
    ax: plt.Axes,
    centres_x: np.ndarray,
    centres_y: np.ndarray,
    initial_tree: np.ndarray,
    physical: np.ndarray,
) -> None:
    for left in range(initial_tree.shape[0] - 1):
        for right in range(left + 1, initial_tree.shape[1]):
            if physical[left, right] and not initial_tree[left, right]:
                draw_edge(
                    ax,
                    centres_x,
                    centres_y,
                    left,
                    right,
                    COLORS["alternative"],
                    (0, (1.4, 1.2)),
                    1.15,
                    3,
                )
    for left in range(initial_tree.shape[0] - 1):
        for right in range(left + 1, initial_tree.shape[1]):
            if not initial_tree[left, right]:
                continue
            available = physical[left, right]
            draw_edge(
                ax,
                centres_x,
                centres_y,
                left,
                right,
                COLORS["tree"] if available else COLORS["broken"],
                "-" if available else (0, (3.0, 2.2)),
                1.15 if available else 1.30,
                4,
            )
            if not available:
                midpoint_x = 0.5 * (centres_x[left] + centres_x[right])
                midpoint_y = 0.5 * (centres_y[left] + centres_y[right])
                ax.scatter(
                    midpoint_x,
                    midpoint_y,
                    marker="x",
                    s=28,
                    linewidth=1.25,
                    color=COLORS["broken"],
                    zorder=5,
                )


def snapshot_bounds(scene: dict, snapshot_index: int) -> tuple:
    sensor_x = as_float_array(scene["sensorX"])[:, snapshot_index]
    sensor_y = as_float_array(scene["sensorY"])[:, snapshot_index]
    target_x = as_float_array(scene["targetX"])[:, snapshot_index]
    target_y = as_float_array(scene["targetY"])[:, snapshot_index]
    finite_target = np.isfinite(target_x) & np.isfinite(target_y)
    headings = as_float_array(scene["sensorHeadingRadByTime"])
    group_ids = np.asarray(scene["sensorGroupIds"], dtype=int)
    sector_x: list[float] = []
    sector_y: list[float] = []
    half_angle = np.radians(float(scene["fovHalfAngleDeg"]))
    radius = float(scene["fovRange"])
    for formation_id in range(1, int(scene["formationCount"]) + 1):
        representative = int(np.flatnonzero(group_ids == formation_id)[0])
        angles = np.linspace(
            headings[representative, snapshot_index] - half_angle,
            headings[representative, snapshot_index] + half_angle,
            41,
        )
        sector_x.extend(
            (sensor_x[representative] + radius * np.cos(angles)).tolist()
        )
        sector_y.extend(
            (sensor_y[representative] + radius * np.sin(angles)).tolist()
        )
    x_values = np.concatenate(
        [sensor_x, target_x[finite_target], np.asarray(sector_x)]
    )
    y_values = np.concatenate(
        [sensor_y, target_y[finite_target], np.asarray(sector_y)]
    )
    x_min, x_max = float(np.min(x_values)), float(np.max(x_values))
    y_min, y_max = float(np.min(y_values)), float(np.max(y_values))
    x_mid, y_mid = 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)
    x_span = (x_max - x_min) * 1.08
    y_span = (y_max - y_min) * 1.08
    # A common data-range aspect gives all three phases equal visual weight
    # while preserving equal metre scales on x and y.
    target_ratio = 1.25
    if x_span / y_span < target_ratio:
        x_span = target_ratio * y_span
    else:
        y_span = x_span / target_ratio
    return (
        (x_mid - 0.5 * x_span, x_mid + 0.5 * x_span),
        (y_mid - 0.5 * y_span, y_mid + 0.5 * y_span),
    )


def draw_snapshot(
    ax: plt.Axes,
    scene: dict,
    snapshot_time: int,
    snapshot_slot: int,
    title: str,
    status: str,
    panel_label: str,
) -> None:
    index = snapshot_time - 1
    sensor_x = as_float_array(scene["sensorX"])
    sensor_y = as_float_array(scene["sensorY"])
    target_x = as_float_array(scene["targetX"])
    target_y = as_float_array(scene["targetY"])
    centres_x = as_float_array(scene["formationCenterX"])
    centres_y = as_float_array(scene["formationCenterY"])
    headings = as_float_array(scene["sensorHeadingRadByTime"])
    group_ids = np.asarray(scene["sensorGroupIds"], dtype=int)
    initial_tree = np.asarray(scene["initialTreeFormationAdjacency"], dtype=bool)
    physical_snapshots = np.asarray(
        scene["physicalFormationAdjacencyAtSnapshots"], dtype=bool
    )
    physical = physical_snapshots[:, :, snapshot_slot]

    # One exact 120-degree, 300 m sector per formation. All six sensors in a
    # formation share the same heading; the first sensor is the shown origin.
    for formation_id in range(1, int(scene["formationCount"]) + 1):
        members = np.flatnonzero(group_ids == formation_id)
        representative = int(members[0])
        heading_deg = float(np.degrees(headings[representative, index]))
        ax.add_patch(
            Wedge(
                (sensor_x[representative, index], sensor_y[representative, index]),
                float(scene["fovRange"]),
                heading_deg - float(scene["fovHalfAngleDeg"]),
                heading_deg + float(scene["fovHalfAngleDeg"]),
                facecolor=to_rgba(COLORS["fov"], 0.035),
                edgecolor=to_rgba(COLORS["fov"], 0.40),
                linewidth=0.55,
                zorder=1,
            )
        )

    draw_network_state(
        ax,
        centres_x[:, index],
        centres_y[:, index],
        initial_tree,
        physical,
    )

    for formation_idx in range(int(scene["formationCount"])):
        members = group_ids == formation_idx + 1
        radii = np.hypot(
            sensor_x[members, index] - centres_x[formation_idx, index],
            sensor_y[members, index] - centres_y[formation_idx, index],
        )
        ax.add_patch(
            Circle(
                (centres_x[formation_idx, index], centres_y[formation_idx, index]),
                float(np.max(radii)) + 5.0,
                facecolor="white",
                edgecolor=to_rgba(COLORS["sensor_ring"], 0.85),
                linewidth=0.60,
                zorder=5,
            )
        )
        ax.text(
            centres_x[formation_idx, index],
            centres_y[formation_idx, index] + float(np.max(radii)) + 13.0,
            f"F{formation_idx + 1}",
            fontsize=5.4,
            fontweight="bold",
            color=COLORS["sensor"],
            ha="center",
            va="bottom",
            zorder=8,
        )

    ax.scatter(
        sensor_x[:, index],
        sensor_y[:, index],
        s=12,
        marker="o",
        facecolor=COLORS["sensor"],
        edgecolor="white",
        linewidth=0.28,
        zorder=7,
    )
    active = np.isfinite(target_x[:, index]) & np.isfinite(target_y[:, index])
    ax.scatter(
        target_x[active, index],
        target_y[active, index],
        s=13,
        marker="D",
        facecolor=COLORS["target"],
        edgecolor="white",
        linewidth=0.25,
        zorder=8,
    )

    limits = snapshot_bounds(scene, index)
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_aspect("equal", adjustable="box")
    ax.xaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.tick_params(
        axis="both",
        labelsize=5.4,
        length=2.1,
        pad=1.3,
        colors=COLORS["axis"],
    )
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(COLORS["spine"])
    ax.set_title(title, fontsize=7.6, fontweight="bold", pad=5, color=COLORS["text"])
    ax.text(
        0.5,
        1.005,
        status,
        transform=ax.transAxes,
        fontsize=5.8,
        color=COLORS["broken"] if "broken" in status else COLORS["alternative"],
        ha="center",
        va="bottom",
    )
    ax.set_xlabel("x (m)", fontsize=6.1, labelpad=1.5)
    if snapshot_slot == 0:
        ax.set_ylabel("y (m)", fontsize=6.1, labelpad=1.5)
    add_panel_label(ax, panel_label)


def draw_timeline(ax: plt.Axes, scenes: list[dict]) -> None:
    y_positions = [1.0, 0.0]
    bar_height = 0.52
    for y, scene, label in zip(y_positions, scenes, ["M24", "X36"]):
        feasible = np.asarray(scene["initialTreeFeasibleByTime"], dtype=bool)
        for time_index, intact in enumerate(feasible, start=1):
            color = COLORS["intact_band"] if intact else COLORS["failure_band"]
            ax.broken_barh(
                [(time_index - 0.5, 1.0)],
                (y - bar_height / 2, bar_height),
                facecolors=color,
                edgecolors="none",
            )
        start = int(scene["failureStartTime"])
        stop = int(scene["failureStopTime"])
        ax.text(
            0.5 * (start + stop),
            y,
            f"fixed tree broken  {start}–{stop}",
            fontsize=6.2,
            fontweight="bold",
            color=COLORS["broken"],
            ha="center",
            va="center",
        )
        ax.text(
            162.5,
            y,
            "physical graph connected",
            fontsize=5.8,
            color=COLORS["alternative"],
            ha="left",
            va="center",
        )
        ax.text(
            -4.0,
            y,
            label,
            fontsize=6.7,
            fontweight="bold",
            color=COLORS["text"],
            ha="right",
            va="center",
        )

    handoffs = np.asarray(scenes[0]["plannedHandoverTimes"], dtype=int)
    for handoff in handoffs:
        ax.axvline(
            handoff,
            color=COLORS["handover"],
            linewidth=0.85,
            linestyle=(0, (2.0, 1.6)),
            zorder=4,
        )
        ax.scatter(
            handoff,
            1.50,
            marker="v",
            s=18,
            color=COLORS["handover"],
            edgecolor="white",
            linewidth=0.25,
            zorder=5,
        )
        ax.text(
            handoff,
            1.66,
            f"handoff {handoff}",
            fontsize=5.4,
            color=COLORS["handover"],
            ha="center",
            va="bottom",
        )

    for snapshot in EXPECTED_SNAPSHOTS:
        ax.axvline(snapshot, color="#9DA4AA", linewidth=0.45, alpha=0.65)
    ax.set_xlim(0.5, 198)
    ax.set_ylim(-0.55, 1.90)
    ax.set_yticks([])
    ax.set_xticks([1, 40, 57, 80, 104, 120, 160])
    ax.tick_params(axis="x", labelsize=5.6, length=2.2, pad=1.5, colors=COLORS["axis"])
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(COLORS["spine"])
    ax.set_xlabel("time step", fontsize=6.2, labelpad=2.0)
    ax.set_title(
        "Scale-matched routing stress and target handoffs",
        fontsize=7.6,
        fontweight="bold",
        loc="left",
        pad=5,
        color=COLORS["text"],
    )
    add_panel_label(ax, "d", x_offset=-0.025)


def build_figure(source: dict) -> plt.Figure:
    figure = plt.figure(figsize=(7.45, 5.10), facecolor="white")
    grid = figure.add_gridspec(
        2,
        3,
        height_ratios=[3.25, 1.0],
        left=0.065,
        right=0.965,
        top=0.945,
        bottom=0.105,
        hspace=0.33,
        wspace=0.24,
    )
    x36 = source["scenes"][1]
    panel_specs = [
        (1, "Separated platoons · t = 1", "fixed tree intact"),
        (80, "Shared zipper bottleneck · t = 80", "fixed tree broken; alternatives available"),
        (160, "Split platoons · t = 160", "fixed tree recovered"),
    ]
    for slot, (snapshot, title, status) in enumerate(panel_specs):
        ax = figure.add_subplot(grid[0, slot])
        draw_snapshot(
            ax,
            x36,
            snapshot,
            slot,
            title,
            status,
            chr(ord("a") + slot),
        )

    timeline_ax = figure.add_subplot(grid[1, :])
    draw_timeline(timeline_ax, source["scenes"])

    legend_handles = [
        Line2D([], [], marker="o", linestyle="none", markersize=4.5,
               markerfacecolor=COLORS["sensor"], markeredgecolor="white",
               label="sensor"),
        Line2D([], [], marker="D", linestyle="none", markersize=4.2,
               markerfacecolor=COLORS["target"], markeredgecolor="white",
               label="target"),
        Patch(facecolor=to_rgba(COLORS["fov"], 0.06),
              edgecolor=COLORS["fov"], label="representative 120° / 300 m FoV"),
        Line2D([], [], color=COLORS["tree"], linewidth=1.2,
               label="initial tree edge available"),
        Line2D([], [], color=COLORS["broken"], linewidth=1.3,
               linestyle=(0, (3.0, 2.2)), marker="x", markersize=4.5,
               label="initial tree edge unavailable"),
        Line2D([], [], color=COLORS["alternative"], linewidth=1.2,
               linestyle=(0, (1.4, 1.2)), label="physical alternative edge"),
    ]
    figure.legend(
        handles=legend_handles,
        loc="center",
        bbox_to_anchor=(0.51, 0.365),
        ncol=3,
        frameon=False,
        fontsize=5.8,
        columnspacing=1.5,
        handlelength=2.1,
        handletextpad=0.55,
    )
    return figure


def save_figure(figure: plt.Figure, output_base: Path) -> list[Path]:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_base.with_suffix(".svg"),
        output_base.with_suffix(".pdf"),
        output_base.with_suffix(".png"),
    ]
    figure.savefig(outputs[0], metadata={"Date": None})
    figure.savefig(
        outputs[1], metadata={"CreationDate": None, "ModDate": None}
    )
    figure.savefig(outputs[2], dpi=300)
    plt.close(figure)
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
