#!/usr/bin/env python3
"""Render the exact target-cohort overlap/reseparation scene as SVG/PDF/PNG."""

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


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7.5,
        "axes.linewidth": 0.75,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "legend.frameon": False,
    }
)


COLORS = {
    "sensor": "#235A85",
    "formation": "#7AA6C7",
    "fov": "#3A9D82",
    "link": "#737B84",
    "cohort_a": "#D97706",
    "cohort_b": "#7C3AED",
    "axis": "#42484F",
    "grid": "#D7DBE0",
    "text": "#1E2328",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=(
            "RUN/GA/dynamic_topology/figures/source/"
            "target_group_overlap_split_m24_seed41.json"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "RUN/GA/dynamic_topology/figures/"
            "target_group_overlap_split_m24"
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
        "target-group-overlap-split-figure-source-v1"
    ):
        raise ValueError("Unexpected target-overlap figure source contract.")
    if source.get("rendererContract") != "python-matplotlib-svg-v1":
        raise ValueError("Figure source does not authorize this renderer.")
    if not source.get("geometryTruthUsed"):
        raise ValueError("Figure must use exact generated geometry.")
    if source.get("posteriorUsed") or source.get("trackingResultUsed"):
        raise ValueError("Scenario figure must not consume tracker outcomes.")
    if source.get("sceneStyle") != "target-group-overlap-split":
        raise ValueError("Unexpected scene style.")
    if source.get("sceneCalibrationStatus") != "development-only":
        raise ValueError("Scene must remain development-only.")
    if source.get("snapshotTimes") != [1, 80, 160]:
        raise ValueError("Unexpected phase snapshots.")
    if float(source.get("fovTotalAngleDeg", 0)) != 120.0:
        raise ValueError("The scene changed the 120-degree FoV.")
    if float(source.get("fovRange", 0)) != 300.0:
        raise ValueError("The scene changed the 300 m FoV range.")
    return source


def target_group_centres(source: dict) -> tuple[np.ndarray, np.ndarray]:
    target_x = as_float_array(source["targetX"])
    target_y = as_float_array(source["targetY"])
    group_ids = np.asarray(source["targetGroupIds"], dtype=int)
    group_count = int(source["targetGroupCount"])
    centre_x = np.zeros((group_count, target_x.shape[1]))
    centre_y = np.zeros_like(centre_x)
    for group_idx in range(1, group_count + 1):
        members = group_ids == group_idx
        centre_x[group_idx - 1] = np.nanmean(target_x[members], axis=0)
        centre_y[group_idx - 1] = np.nanmean(target_y[members], axis=0)
    return centre_x, centre_y


def formation_mst(adjacency: np.ndarray, x: np.ndarray, y: np.ndarray) -> list:
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

    selected = []
    for _, left, right in sorted(candidates):
        left_root, right_root = root(left), root(right)
        if left_root == right_root:
            continue
        parent[right_root] = left_root
        selected.append((left, right))
        if len(selected) == adjacency.shape[0] - 1:
            break
    return selected


def add_direction_arrow(
    ax: plt.Axes, x: np.ndarray, y: np.ndarray, time_idx: int, color: str
) -> None:
    start = max(0, time_idx - 6)
    stop = min(x.size - 1, time_idx + 6)
    if stop <= start:
        return
    ax.annotate(
        "",
        xy=(x[stop], y[stop]),
        xytext=(x[start], y[start]),
        arrowprops={
            "arrowstyle": "-|>",
            "color": color,
            "lw": 0.7,
            "mutation_scale": 6,
        },
        zorder=5,
    )


def draw_panel(ax: plt.Axes, source: dict, panel_idx: int) -> None:
    snapshot = int(source["snapshotTimes"][panel_idx]) - 1
    phase = source["phaseNames"][panel_idx]
    sensor_x = as_float_array(source["sensorX"])
    sensor_y = as_float_array(source["sensorY"])
    target_x = as_float_array(source["targetX"])
    target_y = as_float_array(source["targetY"])
    centre_x = as_float_array(source["formationCenterX"])
    centre_y = as_float_array(source["formationCenterY"])
    headings = as_float_array(source["sensorHeadingRadByTime"])
    sensor_groups = np.asarray(source["sensorGroupIds"], dtype=int)
    target_groups = np.asarray(source["targetGroupIds"], dtype=int)
    adjacency = np.asarray(source["physicalFormationAdjacency"], dtype=bool)[
        :, :, panel_idx
    ]
    group_x, group_y = target_group_centres(source)
    cohort_size = int(source["targetGroupCount"]) // 2

    for group_idx in range(group_x.shape[0]):
        color = COLORS["cohort_a"] if group_idx < cohort_size else COLORS["cohort_b"]
        ax.plot(
            group_x[group_idx],
            group_y[group_idx],
            color=color,
            lw=0.75,
            alpha=0.35,
            zorder=1,
        )
        add_direction_arrow(
            ax, group_x[group_idx], group_y[group_idx], snapshot, color
        )

    formation_x = centre_x[:, snapshot]
    formation_y = centre_y[:, snapshot]
    for left, right in formation_mst(adjacency, formation_x, formation_y):
        ax.plot(
            [formation_x[left], formation_x[right]],
            [formation_y[left], formation_y[right]],
            color=COLORS["link"],
            lw=0.75,
            ls=(0, (2.5, 2.0)),
            alpha=0.9,
            zorder=2,
        )

    for formation_idx, group_id in enumerate(np.unique(sensor_groups)):
        members = np.flatnonzero(sensor_groups == group_id)
        representative = int(members[0])
        heading_deg = float(np.degrees(headings[representative, snapshot]))
        ax.add_patch(
            Wedge(
                (sensor_x[representative, snapshot], sensor_y[representative, snapshot]),
                float(source["fovRange"]),
                heading_deg - float(source["fovHalfAngleDeg"]),
                heading_deg + float(source["fovHalfAngleDeg"]),
                facecolor=COLORS["fov"],
                edgecolor=COLORS["fov"],
                alpha=0.075,
                linewidth=0.55,
                zorder=0,
            )
        )
        radius = float(
            np.max(
                np.hypot(
                    sensor_x[members, snapshot] - formation_x[formation_idx],
                    sensor_y[members, snapshot] - formation_y[formation_idx],
                )
            )
        )
        ax.add_patch(
            Circle(
                (formation_x[formation_idx], formation_y[formation_idx]),
                radius + 4,
                fill=False,
                edgecolor=COLORS["formation"],
                linewidth=0.55,
                alpha=0.9,
                zorder=3,
            )
        )

    ax.scatter(
        sensor_x[:, snapshot],
        sensor_y[:, snapshot],
        s=12,
        marker="^",
        facecolor="white",
        edgecolor=COLORS["sensor"],
        linewidth=0.7,
        zorder=6,
    )
    ax.scatter(
        formation_x,
        formation_y,
        s=12,
        marker="o",
        facecolor=COLORS["sensor"],
        edgecolor="white",
        linewidth=0.35,
        zorder=7,
    )
    first_cohort = target_groups <= cohort_size
    for mask, color in (
        (first_cohort, COLORS["cohort_a"]),
        (~first_cohort, COLORS["cohort_b"]),
    ):
        ax.scatter(
            target_x[mask, snapshot],
            target_y[mask, snapshot],
            s=13,
            marker="o",
            facecolor=color,
            edgecolor="white",
            linewidth=0.4,
            zorder=8,
        )

    ax.set_title(
        f"({chr(97 + panel_idx)}) {phase},  k = {snapshot + 1}",
        loc="left",
        fontsize=8.3,
        color=COLORS["text"],
        pad=4,
    )
    ax.set_xlim(-710, 710)
    ax.set_ylim(20, 450)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    if panel_idx == 0:
        ax.set_ylabel("y (m)")
    ax.grid(True, color=COLORS["grid"], linewidth=0.38, alpha=0.55)
    ax.tick_params(length=2.5, width=0.6, colors=COLORS["axis"])


def render(source: dict, output: Path) -> None:
    figure, axes = plt.subplots(1, 3, figsize=(7.2, 2.2), sharex=True, sharey=True)
    for panel_idx, ax in enumerate(axes):
        draw_panel(ax, source, panel_idx)

    legend = [
        Line2D(
            [0],
            [0],
            marker="^",
            color="none",
            markerfacecolor="white",
            markeredgecolor=COLORS["sensor"],
            markersize=5,
            label="sensor node",
        ),
        Line2D(
            [0],
            [0],
            color=COLORS["link"],
            lw=0.8,
            ls=(0, (2.5, 2)),
            label="physical carrier",
        ),
        Patch(facecolor=COLORS["fov"], alpha=0.15, label="representative FoV"),
        Line2D(
            [0], [0], marker="o", color=COLORS["cohort_a"], lw=0.8,
            markersize=4, label="target cohort A"
        ),
        Line2D(
            [0], [0], marker="o", color=COLORS["cohort_b"], lw=0.8,
            markersize=4, label="target cohort B"
        ),
    ]
    figure.legend(
        handles=legend,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=5,
        columnspacing=1.2,
        handletextpad=0.45,
    )
    figure.text(
        0.5,
        0.01,
        "Identical stationary sensor chain in all panels; 120° FoV, 300 m range.",
        ha="center",
        va="bottom",
        fontsize=7.2,
        color=COLORS["axis"],
    )
    figure.subplots_adjust(left=0.07, right=0.99, bottom=0.22, top=0.79, wspace=0.12)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output.with_suffix(".svg"), metadata={"Date": None})
    figure.savefig(output.with_suffix(".pdf"), metadata={"CreationDate": None})
    figure.savefig(output.with_suffix(".png"), dpi=240, metadata={"Date": None})
    plt.close(figure)


def main() -> None:
    args = parse_args()
    source = load_source(Path(args.data))
    render(source, Path(args.output))


if __name__ == "__main__":
    main()
