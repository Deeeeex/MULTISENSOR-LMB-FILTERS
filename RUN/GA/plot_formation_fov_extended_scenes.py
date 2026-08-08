#!/usr/bin/env python3
"""Render exact M24/X36 merge-split and curved-corridor geometry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Wedge
from matplotlib.ticker import MaxNLocator


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7,
        "axes.linewidth": 0.75,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "legend.frameon": False,
    }
)


COLORS = {
    "sensor": "#285F8F",
    "sensor_path": "#6E9FC5",
    "sensor_ring": "#A8C7DE",
    "target": "#C65A4E",
    "target_path": "#E1A197",
    "fov": "#3B9B86",
    "link": "#707780",
    "axis": "#555B61",
    "spine": "#B8BDC4",
    "text": "#202429",
}


EXPECTED_PRESETS = [
    "m24-formation-fov-merge-split",
    "m24-formation-fov-curved-corridor",
    "x36-formation-fov-merge-split",
    "x36-formation-fov-curved-corridor",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=(
            "RUN/GA/dynamic_topology/figures/source/"
            "formation_fov_extended_scenes_v1_seed41.json"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "RUN/GA/dynamic_topology/figures/"
            "formation_fov_extended_scenes_v1"
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
    if source.get("contractVersion") != (
        "formation-fov-extended-scene-figure-source-v1"
    ):
        raise ValueError("Unexpected extended-scene source contract.")
    if source.get("rendererContract") != "python-matplotlib-only":
        raise ValueError("Figure source does not authorize the Python renderer.")
    if not source.get("geometryTruthUsed"):
        raise ValueError("Figure must use generated scenario geometry.")
    if source.get("posteriorUsed") or source.get("trackingResultUsed"):
        raise ValueError("Scenario figure must not consume tracker outcomes.")
    scenes = source.get("scenes", [])
    if [scene.get("presetName") for scene in scenes] != EXPECTED_PRESETS:
        raise ValueError("Unexpected preset order in extended-scene source.")
    for scene in scenes:
        if scene.get("sceneCalibrationStatus") != "development-only":
            raise ValueError("Extended scenes must remain development-only.")
        if float(scene.get("fovTotalAngleDeg", 0.0)) != 120.0:
            raise ValueError("The scene changed the 120-degree FoV.")
        if float(scene.get("fovRange", 0.0)) != 300.0:
            raise ValueError("The scene changed the 300 m sensing range.")
        if int(scene.get("sensorsPerFormation", 0)) != 6:
            raise ValueError("The scene changed formation size.")
    return source


def target_group_centres(
    target_x: np.ndarray,
    target_y: np.ndarray,
    target_group_ids: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    centres: list[tuple[np.ndarray, np.ndarray]] = []
    for group_id in np.unique(target_group_ids):
        members = target_group_ids == group_id
        finite = np.isfinite(target_x[members]) & np.isfinite(target_y[members])
        counts = np.sum(finite, axis=0)
        active = counts > 0
        x = np.full(target_x.shape[1], np.nan)
        y = np.full(target_y.shape[1], np.nan)
        x[active] = np.nansum(
            np.where(finite, target_x[members], np.nan), axis=0
        )[active] / counts[active]
        y[active] = np.nansum(
            np.where(finite, target_y[members], np.nan), axis=0
        )[active] / counts[active]
        centres.append((x, y))
    return centres


def add_direction_arrow(
    ax: plt.Axes,
    x: np.ndarray,
    y: np.ndarray,
    index: int,
    color: str,
) -> None:
    finite = np.flatnonzero(np.isfinite(x) & np.isfinite(y))
    local = finite[(finite >= index - 5) & (finite <= index + 5)]
    if local.size < 2:
        return
    ax.annotate(
        "",
        xy=(x[local[-1]], y[local[-1]]),
        xytext=(x[local[0]], y[local[0]]),
        arrowprops={
            "arrowstyle": "-|>",
            "color": color,
            "lw": 0.65,
            "mutation_scale": 5.5,
            "shrinkA": 0,
            "shrinkB": 0,
        },
        zorder=5,
    )


def formation_mst(
    adjacency: np.ndarray, x: np.ndarray, y: np.ndarray
) -> list[tuple[int, int]]:
    """Select a readable physical-connectivity skeleton at the snapshot."""

    candidates: list[tuple[float, int, int]] = []
    for left in range(adjacency.shape[0] - 1):
        for right in range(left + 1, adjacency.shape[1]):
            if adjacency[left, right]:
                distance = float(np.hypot(x[left] - x[right], y[left] - y[right]))
                candidates.append((distance, left, right))
    parent = list(range(adjacency.shape[0]))

    def root(node: int) -> int:
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    selected: list[tuple[int, int]] = []
    for _, left, right in sorted(candidates):
        left_root, right_root = root(left), root(right)
        if left_root == right_root:
            continue
        parent[right_root] = left_root
        selected.append((left, right))
        if len(selected) == adjacency.shape[0] - 1:
            break
    return selected


def scene_bounds(scene: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    sensor_x = as_float_array(scene["sensorX"])
    sensor_y = as_float_array(scene["sensorY"])
    target_x = as_float_array(scene["targetX"])
    target_y = as_float_array(scene["targetY"])
    centre_x = as_float_array(scene["formationCenterX"])
    centre_y = as_float_array(scene["formationCenterY"])
    snapshot = int(scene["snapshotTime"]) - 1
    headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
    groups = np.asarray(scene["sensorGroupIds"], dtype=int)
    xs = np.concatenate(
        [
            sensor_x[:, snapshot],
            centre_x[np.isfinite(centre_x)],
            target_x[np.isfinite(target_x)],
        ]
    ).tolist()
    ys = np.concatenate(
        [
            sensor_y[:, snapshot],
            centre_y[np.isfinite(centre_y)],
            target_y[np.isfinite(target_y)],
        ]
    ).tolist()
    half_angle = np.radians(float(scene["fovHalfAngleDeg"]))
    radius = float(scene["fovRange"])
    for group_id in np.unique(groups):
        sensor_idx = int(np.flatnonzero(groups == group_id)[0])
        angles = np.linspace(
            headings[sensor_idx] - half_angle,
            headings[sensor_idx] + half_angle,
            61,
        )
        xs.extend((sensor_x[sensor_idx, snapshot] + radius * np.cos(angles)).tolist())
        ys.extend((sensor_y[sensor_idx, snapshot] + radius * np.sin(angles)).tolist())
    x_pad = max(30.0, 0.03 * (max(xs) - min(xs)))
    y_pad = max(30.0, 0.03 * (max(ys) - min(ys)))
    return (min(xs) - x_pad, max(xs) + x_pad), (min(ys) - y_pad, max(ys) + y_pad)


def draw_scene(ax: plt.Axes, scene: dict, panel_label: str) -> None:
    sensor_x = as_float_array(scene["sensorX"])
    sensor_y = as_float_array(scene["sensorY"])
    target_x = as_float_array(scene["targetX"])
    target_y = as_float_array(scene["targetY"])
    centre_x = as_float_array(scene["formationCenterX"])
    centre_y = as_float_array(scene["formationCenterY"])
    headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
    sensor_groups = np.asarray(scene["sensorGroupIds"], dtype=int)
    target_groups = np.asarray(scene["targetGroupIds"], dtype=int)
    adjacency = np.asarray(scene["physicalFormationAdjacency"], dtype=bool)
    snapshot = int(scene["snapshotTime"]) - 1

    for formation_idx in range(int(scene["formationCount"])):
        ax.plot(
            centre_x[formation_idx],
            centre_y[formation_idx],
            color=COLORS["sensor_path"],
            linewidth=0.8,
            alpha=0.85,
            zorder=1,
        )
        add_direction_arrow(
            ax,
            centre_x[formation_idx],
            centre_y[formation_idx],
            snapshot,
            COLORS["sensor"],
        )

    for group_x, group_y in target_group_centres(target_x, target_y, target_groups):
        ax.plot(
            group_x,
            group_y,
            color=COLORS["target_path"],
            linewidth=0.72,
            alpha=0.82,
            zorder=1,
        )
        add_direction_arrow(ax, group_x, group_y, snapshot, COLORS["target"])

    unique_groups = np.unique(sensor_groups)
    for group_id in unique_groups:
        representative = int(np.flatnonzero(sensor_groups == group_id)[0])
        heading_deg = float(np.degrees(headings[representative]))
        ax.add_patch(
            Wedge(
                (sensor_x[representative, snapshot], sensor_y[representative, snapshot]),
                float(scene["fovRange"]),
                heading_deg - float(scene["fovHalfAngleDeg"]),
                heading_deg + float(scene["fovHalfAngleDeg"]),
                facecolor=COLORS["fov"],
                edgecolor=COLORS["fov"],
                alpha=0.075,
                linewidth=0.55,
                zorder=0,
            )
        )

    current_x = centre_x[:, snapshot]
    current_y = centre_y[:, snapshot]
    for left, right in formation_mst(adjacency, current_x, current_y):
        ax.plot(
            [current_x[left], current_x[right]],
            [current_y[left], current_y[right]],
            color=COLORS["link"],
            linewidth=0.7,
            linestyle=(0, (2.2, 1.8)),
            alpha=0.8,
            zorder=2,
        )

    for formation_idx, group_id in enumerate(unique_groups):
        members = sensor_groups == group_id
        offsets = np.hypot(
            sensor_x[members, snapshot] - current_x[formation_idx],
            sensor_y[members, snapshot] - current_y[formation_idx],
        )
        ax.add_patch(
            Circle(
                (current_x[formation_idx], current_y[formation_idx]),
                float(np.max(offsets)) + 5.0,
                facecolor="none",
                edgecolor=COLORS["sensor_ring"],
                linewidth=0.48,
                zorder=3,
            )
        )
        ax.text(
            current_x[formation_idx] - float(np.max(offsets)) - 12.0,
            current_y[formation_idx],
            f"F{formation_idx + 1}",
            ha="right",
            va="center",
            fontsize=5.2,
            color=COLORS["sensor"],
            zorder=7,
        )

    ax.scatter(
        sensor_x[:, snapshot],
        sensor_y[:, snapshot],
        s=10,
        marker="o",
        facecolor=COLORS["sensor"],
        edgecolor="white",
        linewidth=0.25,
        zorder=6,
    )
    active = np.isfinite(target_x[:, snapshot]) & np.isfinite(target_y[:, snapshot])
    ax.scatter(
        target_x[active, snapshot],
        target_y[active, snapshot],
        s=11,
        marker="D",
        facecolor=COLORS["target"],
        edgecolor="white",
        linewidth=0.25,
        zorder=7,
    )

    limits = scene_bounds(scene)
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_aspect("equal", adjustable="box")
    ax.xaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.tick_params(axis="both", labelsize=5.2, length=2.0, pad=1.2, colors=COLORS["axis"])
    ax.spines["left"].set_color(COLORS["spine"])
    ax.spines["bottom"].set_color(COLORS["spine"])
    ax.set_xlabel("x (m)", fontsize=6.1, labelpad=1.0)
    ax.set_ylabel("y (m)", fontsize=6.1, labelpad=1.0)
    ax.text(
        -0.10,
        1.02,
        panel_label,
        transform=ax.transAxes,
        fontsize=8.5,
        fontweight="bold",
        color=COLORS["text"],
        ha="left",
        va="bottom",
    )


def build_figure(source: dict) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(7.2, 5.6), facecolor="white")
    figure.subplots_adjust(
        left=0.085,
        right=0.985,
        top=0.92,
        bottom=0.135,
        hspace=0.34,
        wspace=0.23,
    )
    column_titles = ["Merge–split", "Curved corridor"]
    row_labels = ["M24 · 4 formations", "X36 · 6 formations"]
    for column, title in enumerate(column_titles):
        axes[0, column].set_title(
            title,
            fontsize=8.2,
            fontweight="bold",
            pad=5,
            color=COLORS["text"],
        )
    for row, label in enumerate(row_labels):
        figure.text(
            0.015,
            0.715 - 0.455 * row,
            label,
            rotation=90,
            ha="center",
            va="center",
            fontsize=7.0,
            fontweight="bold",
            color=COLORS["text"],
        )
    for axis, scene, label in zip(axes.flat, source["scenes"], "abcd"):
        draw_scene(axis, scene, label)

    legend_handles = [
        Line2D([], [], marker="o", linestyle="none", markersize=4.5, color=COLORS["sensor"], label="Sensor node"),
        Line2D([], [], marker="D", linestyle="none", markersize=4.2, color=COLORS["target"], label="Target node"),
        Line2D([], [], color=COLORS["sensor_path"], linewidth=1.0, label="Formation path"),
        Line2D([], [], color=COLORS["target_path"], linewidth=1.0, label="Target-stream path"),
        Patch(facecolor=COLORS["fov"], edgecolor=COLORS["fov"], alpha=0.12, label="120° / 300 m FoV"),
        Line2D([], [], color=COLORS["link"], linewidth=0.8, linestyle=(0, (2.2, 1.8)), label="Physical-link skeleton"),
    ]
    figure.legend(
        handles=legend_handles,
        loc="lower center",
        ncol=6,
        bbox_to_anchor=(0.54, 0.025),
        fontsize=5.8,
        handlelength=1.8,
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
    figure.savefig(outputs[0], metadata={"Date": None})
    figure.savefig(outputs[1], metadata={"CreationDate": None, "ModDate": None})
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
    outputs = save_figure(build_figure(source), Path(args.output))
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
