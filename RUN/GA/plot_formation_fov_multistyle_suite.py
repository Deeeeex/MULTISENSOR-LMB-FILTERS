#!/usr/bin/env python3
"""Render the frozen v5 multistyle formation-FoV geometry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, Wedge
from matplotlib.ticker import MaxNLocator


# Editable-vector typography contract.
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Arial", "DejaVu Sans", "Liberation Sans"]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["svg.hashsalt"] = "formation-fov-multistyle-suite-v2"
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["font.size"] = 7
plt.rcParams["axes.linewidth"] = 0.75
plt.rcParams["xtick.major.width"] = 0.65
plt.rcParams["ytick.major.width"] = 0.65


COLORS = {
    "sensor": "#174A7E",
    "formation_path": "#4C78A8",
    "formation_ring": "#8FB4D4",
    "target": "#C44E52",
    "target_path": "#E6A09B",
    "fov": "#2A9D8F",
    "backbone": "#7A7A7A",
    "axis": "#555B61",
    "spine": "#B5BAC1",
    "panel_text": "#22262A",
}


STYLE_TITLES = {
    "parallel-convoy": "Offset-corridor convoy",
    "linear-relay": "Linear relay",
    "orthogonal-crossing": "Orthogonal crossing",
}


EXPECTED_PRESETS = [
    "x36-formation-fov-convoy",
    "x36-formation-fov-relay",
    "x36-formation-fov-crossing",
    "m24-formation-fov-convoy",
    "m24-formation-fov-relay",
    "m24-formation-fov-crossing",
]


EXPECTED_SCENE_DIGESTS = {
    "x36-formation-fov-convoy": (
        "399ed8ea978df43e15d55489791a9c0e4e11ba99724ea8a760a1f3b68a8177c8"
    ),
    "x36-formation-fov-relay": (
        "7fd7130518b027caaa327b3a8bcf97b7559148d1f4f1c17a1b9ad9640ae7e9a0"
    ),
    "x36-formation-fov-crossing": (
        "69ea8084b1688680765e437f78e8208f1097e2dec80c7e6ff00dcbdb0feea8f0"
    ),
    "m24-formation-fov-convoy": (
        "666def7dbed41c47b9f29e536a369ef24a52077059429e45e5a41852e4071726"
    ),
    "m24-formation-fov-relay": (
        "49005834d31fcc6c729d546e725e79be7ae1c5b542209ea599a944187a31f2a0"
    ),
    "m24-formation-fov-crossing": (
        "3749da4978a5c536c029402b8aa25762f267bee2b9e69a080548657e61a62cc6"
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default=(
            "RUN/GA/dynamic_topology/figures/source/"
            "formation_fov_multistyle_suite_v5_seed41.json"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "RUN/GA/dynamic_topology/figures/"
            "formation_fov_multistyle_suite_v2"
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
        "formation-fov-multistyle-figure-source-v2"
    ):
        raise ValueError("Unexpected multistyle figure-source contract.")
    if source.get("rendererContract") != "python-matplotlib-only":
        raise ValueError("Figure source does not authorize the Python renderer.")
    if not source.get("geometryTruthUsed"):
        raise ValueError("Scenario schematic requires exact generated geometry.")
    if source.get("posteriorUsed") or source.get("trackingResultUsed"):
        raise ValueError("Scenario schematic may not consume posterior outcomes.")
    if len(source.get("scenes", [])) != 6:
        raise ValueError("Expected exactly six multistyle scene panels.")
    if [scene.get("presetName") for scene in source["scenes"]] != EXPECTED_PRESETS:
        raise ValueError("Figure source scene order differs from the v5 contract.")

    for scene in source["scenes"]:
        if scene.get("sceneGeometryVersion") != "formation-fov-multistyle-v5":
            raise ValueError("Figure source is not bound to the v5 geometry.")
        if float(scene.get("fovTotalAngleDeg", 0.0)) != 120.0:
            raise ValueError("Figure source changed the 120-degree FoV contract.")
        if float(scene.get("fovRange", 0.0)) != 300.0:
            raise ValueError("Figure source changed the 300 m range contract.")
        if int(scene.get("sensorsPerFormation", 0)) != 6:
            raise ValueError("Figure source changed the six-sensor formation.")
        if scene.get("trackingOutcomeAuthorized") is not False:
            raise ValueError("Figure source must keep tracking outcomes disabled.")
        if not isinstance(scene.get("sceneContractSha256"), str) or len(
            scene["sceneContractSha256"]
        ) != 64:
            raise ValueError("Figure source lacks the frozen scene digest.")
        if scene["sceneContractSha256"] != EXPECTED_SCENE_DIGESTS[
            scene["presetName"]
        ]:
            raise ValueError("Figure source differs from the registered scene digest.")

        is_stress = scene["sceneStyle"] == "orthogonal-crossing"
        expected_formal = not is_stress
        expected_status = (
            "stress-only-v5"
            if is_stress
            else "held-out-geometry-gate-frozen-v5"
        )
        if scene.get("sceneCalibrationStatus") != expected_status:
            raise ValueError("Figure source has an unexpected v5 scene status.")
        if scene.get("formalValidationAuthorized") is not expected_formal:
            raise ValueError("Figure source has an unexpected geometry gate state.")

        group_ids = np.asarray(scene.get("sensorGroupIds", []), dtype=int)
        headings = np.asarray(scene.get("sensorHeadingRad", []), dtype=float)
        if group_ids.size != int(scene["nodeCount"]) or headings.size != group_ids.size:
            raise ValueError("Figure source has inconsistent sensor headings.")
        for group_id in np.unique(group_ids):
            group_headings = headings[group_ids == group_id]
            if np.max(np.abs(group_headings - group_headings[0])) > 1e-10:
                raise ValueError("Sensors within one formation must share a heading.")

        target_group_ids = np.asarray(scene.get("targetGroupIds", []), dtype=int)
        if target_group_ids.size != int(scene["targetCount"]):
            raise ValueError("Figure source has inconsistent target groups.")
    return source


def add_panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.09,
        1.025,
        label,
        transform=ax.transAxes,
        fontsize=8.5,
        fontweight="bold",
        color=COLORS["panel_text"],
        ha="left",
        va="bottom",
    )


def draw_representative_fov_wedges(
    ax: plt.Axes,
    sensor_x: np.ndarray,
    sensor_y: np.ndarray,
    headings: np.ndarray,
    sensor_group_ids: np.ndarray,
    snapshot_index: int,
    half_angle_deg: float,
    radius: float,
) -> None:
    """Draw one exact sensor-origin sector per formation.

    Every outline is exact.  Only the central formation is lightly filled so
    the range boundary remains legible without recreating the old FoV cloud.
    """

    group_ids = np.unique(sensor_group_ids)
    central_group = group_ids[(len(group_ids) - 1) // 2]
    for group_id in group_ids:
        sensor_idx = int(np.flatnonzero(sensor_group_ids == group_id)[0])
        heading_deg = float(np.degrees(headings[sensor_idx]))
        highlighted = group_id == central_group
        ax.add_patch(
            Wedge(
                (
                    sensor_x[sensor_idx, snapshot_index],
                    sensor_y[sensor_idx, snapshot_index],
                ),
                radius,
                heading_deg - half_angle_deg,
                heading_deg + half_angle_deg,
                facecolor=(
                    to_rgba(COLORS["fov"], 0.075) if highlighted else "none"
                ),
                edgecolor=to_rgba(
                    COLORS["fov"], 0.72 if highlighted else 0.42
                ),
                linewidth=0.72 if highlighted else 0.44,
                zorder=0,
            )
        )


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
        x = np.full(target_x.shape[1], np.nan)
        y = np.full(target_y.shape[1], np.nan)
        active = counts > 0
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
    width: float,
) -> None:
    finite = np.flatnonzero(np.isfinite(x) & np.isfinite(y))
    if finite.size < 2:
        return
    local = finite[
        (finite >= max(index - 6, finite[0]))
        & (finite <= min(index + 6, finite[-1]))
    ]
    if local.size < 2:
        local = finite[-2:]
    left, right = int(local[0]), int(local[-1])
    ax.annotate(
        "",
        xy=(x[right], y[right]),
        xytext=(x[left], y[left]),
        arrowprops={
            "arrowstyle": "-|>",
            "color": color,
            "lw": width,
            "mutation_scale": 6,
            "shrinkA": 0,
            "shrinkB": 0,
        },
        zorder=4,
    )


def scene_plot_bounds(
    scene_pair: list[dict],
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return one unclipped metre-scale frame across the supplied scenes."""

    xs: list[float] = []
    ys: list[float] = []
    for scene in scene_pair:
        snapshot_index = int(scene["snapshotTime"]) - 1
        sensor_x = as_float_array(scene["sensorX"])
        sensor_y = as_float_array(scene["sensorY"])
        target_x = as_float_array(scene["targetX"])
        target_y = as_float_array(scene["targetY"])
        centre_x = as_float_array(scene["formationCenterX"])
        centre_y = as_float_array(scene["formationCenterY"])
        headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
        group_ids = np.asarray(scene["sensorGroupIds"], dtype=int)

        xs.extend(sensor_x[:, snapshot_index].tolist())
        ys.extend(sensor_y[:, snapshot_index].tolist())
        xs.extend(centre_x[np.isfinite(centre_x)].tolist())
        ys.extend(centre_y[np.isfinite(centre_y)].tolist())
        xs.extend(target_x[np.isfinite(target_x)].tolist())
        ys.extend(target_y[np.isfinite(target_y)].tolist())
        radius = float(scene["fovRange"])
        half_angle = np.radians(float(scene["fovHalfAngleDeg"]))
        for group_id in np.unique(group_ids):
            sensor_idx = int(np.flatnonzero(group_ids == group_id)[0])
            angles = np.linspace(
                headings[sensor_idx] - half_angle,
                headings[sensor_idx] + half_angle,
                61,
            )
            origin_x = sensor_x[sensor_idx, snapshot_index]
            origin_y = sensor_y[sensor_idx, snapshot_index]
            xs.extend((origin_x + radius * np.cos(angles)).tolist())
            ys.extend((origin_y + radius * np.sin(angles)).tolist())

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_pad = max(24.0, 0.035 * (x_max - x_min))
    y_pad = max(24.0, 0.035 * (y_max - y_min))
    return (x_min - x_pad, x_max + x_pad), (y_min - y_pad, y_max + y_pad)


def draw_scene(
    ax: plt.Axes,
    scene: dict,
    limits: tuple[tuple[float, float], tuple[float, float]],
    show_title: bool,
    panel_label: str,
) -> None:
    sensor_x = as_float_array(scene["sensorX"])
    sensor_y = as_float_array(scene["sensorY"])
    target_x = as_float_array(scene["targetX"])
    target_y = as_float_array(scene["targetY"])
    centre_x = as_float_array(scene["formationCenterX"])
    centre_y = as_float_array(scene["formationCenterY"])
    headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
    sensor_group_ids = np.asarray(scene["sensorGroupIds"], dtype=int)
    target_group_ids = np.asarray(scene["targetGroupIds"], dtype=int)
    formation_adjacency = np.asarray(scene["formationAdjacency"], dtype=bool)
    snapshot_index = int(scene["snapshotTime"]) - 1

    draw_representative_fov_wedges(
        ax,
        sensor_x,
        sensor_y,
        headings,
        sensor_group_ids,
        snapshot_index,
        float(scene["fovHalfAngleDeg"]),
        float(scene["fovRange"]),
    )

    for formation_idx in range(int(scene["formationCount"])):
        ax.plot(
            centre_x[formation_idx],
            centre_y[formation_idx],
            color=COLORS["formation_path"],
            linewidth=0.82,
            alpha=0.82,
            zorder=1,
        )
        add_direction_arrow(
            ax,
            centre_x[formation_idx],
            centre_y[formation_idx],
            snapshot_index,
            COLORS["formation_path"],
            0.72,
        )

        members = sensor_group_ids == formation_idx + 1
        offsets = np.hypot(
            sensor_x[members, snapshot_index]
            - centre_x[formation_idx, snapshot_index],
            sensor_y[members, snapshot_index]
            - centre_y[formation_idx, snapshot_index],
        )
        ax.add_patch(
            Circle(
                (
                    centre_x[formation_idx, snapshot_index],
                    centre_y[formation_idx, snapshot_index],
                ),
                float(np.max(offsets)) + 5.0,
                facecolor="none",
                edgecolor=COLORS["formation_ring"],
                linewidth=0.42,
                alpha=0.65,
                zorder=3,
            )
        )
        heading = headings[np.flatnonzero(members)[0]]
        arrow_length = 58.0
        ax.annotate(
            "",
            xy=(
                centre_x[formation_idx, snapshot_index]
                + arrow_length * np.cos(heading),
                centre_y[formation_idx, snapshot_index]
                + arrow_length * np.sin(heading),
            ),
            xytext=(
                centre_x[formation_idx, snapshot_index],
                centre_y[formation_idx, snapshot_index],
            ),
            arrowprops={
                "arrowstyle": "-|>",
                "color": COLORS["sensor"],
                "lw": 0.62,
                "mutation_scale": 5,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=5,
        )

    # This is the t=1 geometry-derived formation-level reference skeleton,
    # not a dynamic routing decision at the displayed snapshot.
    for left in range(formation_adjacency.shape[0] - 1):
        for right in range(left + 1, formation_adjacency.shape[1]):
            if formation_adjacency[left, right]:
                ax.plot(
                    [centre_x[left, snapshot_index], centre_x[right, snapshot_index]],
                    [centre_y[left, snapshot_index], centre_y[right, snapshot_index]],
                    color=COLORS["backbone"],
                    linewidth=0.55,
                    linestyle=(0, (2.4, 2.1)),
                    alpha=0.75,
                    zorder=2,
                )

    for group_x, group_y in target_group_centres(
        target_x, target_y, target_group_ids
    ):
        ax.plot(
            group_x,
            group_y,
            color=COLORS["target_path"],
            linewidth=0.72,
            alpha=0.78,
            zorder=1,
        )
        add_direction_arrow(
            ax,
            group_x,
            group_y,
            snapshot_index,
            COLORS["target"],
            0.60,
        )

    ax.scatter(
        sensor_x[:, snapshot_index],
        sensor_y[:, snapshot_index],
        s=10,
        marker="o",
        facecolor=COLORS["sensor"],
        edgecolor="white",
        linewidth=0.28,
        zorder=6,
    )
    active_targets = np.isfinite(target_x[:, snapshot_index]) & np.isfinite(
        target_y[:, snapshot_index]
    )
    ax.scatter(
        target_x[active_targets, snapshot_index],
        target_y[active_targets, snapshot_index],
        s=11,
        marker="D",
        facecolor=COLORS["target"],
        edgecolor="white",
        linewidth=0.25,
        zorder=7,
    )

    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    ax.set_aspect("equal", adjustable="box")
    ax.xaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 2.5, 5, 10]))
    ax.tick_params(
        axis="both",
        labelsize=5.3,
        length=2.1,
        pad=1.3,
        colors=COLORS["axis"],
    )
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(COLORS["spine"])
    ax.spines["bottom"].set_color(COLORS["spine"])

    if show_title:
        ax.set_title(
            STYLE_TITLES[scene["sceneStyle"]],
            fontsize=7.7,
            fontweight="bold",
            pad=5,
            color=COLORS["panel_text"],
        )
    add_panel_label(ax, panel_label)


def build_figure(source: dict) -> plt.Figure:
    figure = plt.figure(figsize=(7.2, 4.8), facecolor="white")
    grid = figure.add_gridspec(
        2,
        2,
        height_ratios=[2.25, 1.0],
        left=0.082,
        right=0.985,
        top=0.925,
        bottom=0.105,
        hspace=0.30,
        wspace=0.22,
    )
    # The source retains both scales for contract validation.  The primary
    # schematic shows X36 because M24 is the same local geometry with two
    # fewer formations; repeating it made the wide relay unreadably small.
    panel_specs = [
        (source["scenes"][0], grid[0, 0], "a"),
        (source["scenes"][2], grid[0, 1], "b"),
        (source["scenes"][1], grid[1, :], "c"),
    ]
    for scene, slot, panel_label in panel_specs:
        ax = figure.add_subplot(slot)
        limits = scene_plot_bounds([scene])
        draw_scene(ax, scene, limits, show_title=True, panel_label=panel_label)
        ax.set_xlabel("x (m)", fontsize=6.2, labelpad=1.2)
        ax.set_ylabel("y (m)", fontsize=6.2, labelpad=1.2)
    return figure


def save_figure(figure: plt.Figure, output_base: Path) -> list[Path]:
    output_base.parent.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_base.with_suffix(".svg"),
        output_base.with_suffix(".pdf"),
        output_base.with_suffix(".png"),
    ]
    # Preserve the registered 7.2 x 4.8 inch canvas exactly.
    figure.savefig(outputs[0], metadata={"Date": None})
    figure.savefig(
        outputs[1],
        metadata={"CreationDate": None, "ModDate": None},
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
