from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle, Wedge


EDGE_COLOR = "#2F4858"
GRID_COLOR = "#DCE3EA"
TEXT_COLOR = "#3F4A56"
MOTION_COLOR = "#5D88B3"
FOV_COLOR = "#9CCBEC"
TARGET_COLORS = ["#2F4858", "#C46A2B", "#4B6A9B", "#0C7C8C"]
DROP_COLORS = {
    0.0: "#E6F1FB",
    0.1: "#DDF3ED",
    0.2: "#F7DE9C",
    0.5: "#F3C8C4",
}


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "font.family": "DejaVu Sans",
            "font.size": 7.4,
            "axes.titlesize": 8.4,
            "axes.labelsize": 7.2,
            "xtick.labelsize": 6.5,
            "ytick.labelsize": 6.5,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.spines.top": True,
            "axes.spines.right": True,
        }
    )


def local_formation_offsets(kind: str, spacing: float, count: int) -> np.ndarray:
    if kind.lower() == "triangle":
        base = np.array([[0.0, -0.5, 0.5], [0.0, -0.866, -0.866]])
    elif kind.lower() == "leader3":
        base = np.array([[0.0, -1.0, -1.0, -2.0], [0.0, -0.7, 0.7, 0.0]])
    else:
        base = np.array([[0.0, -1.0, 1.0], [0.0, -1.0, -1.0]])

    if base.shape[1] < count:
        base = np.hstack([base, np.zeros((2, count - base.shape[1]))])
    return spacing * base[:, :count]


def build_standard_ideal_config() -> dict:
    return {
        "sensor_positions": np.array([[-96, 96, 96, -96], [96, 96, -96, -96]], dtype=float),
        "sensor_edges": [(1, 2), (2, 3), (3, 4), (4, 1)],
        "birth_times": np.array([1, 1, 20, 20, 40, 40, 60, 60, 60, 60], dtype=float),
        "death_times": np.array([70, 70, 80, 80, 90, 90, 100, 100, 100, 100], dtype=float),
        "target_states": np.array(
            [
                [-80.0, -20.0, 0.75, 1.5],
                [-20.0, 80.0, -1.0, -2.0],
                [0.0, 0.0, -0.5, -1.0],
                [40.0, -60.0, -0.25, -0.5],
                [-80.0, -20.0, 1.0, 1.0],
                [40.0, -60.0, -1.0, 2.0],
                [-80.0, -20.0, 1.0, -0.5],
                [-20.0, 80.0, 1.0, -1.0],
                [0.0, 0.0, 1.0, -1.0],
                [40.0, -60.0, -1.0, 0.5],
            ],
            dtype=float,
        ).T,
    }


def build_sensor_initial_states() -> np.ndarray:
    group_centers = np.array([[-80.0, -80.0], [35.0, -35.0]])
    offsets = local_formation_offsets("Leader3", 20.0, 4)
    states = np.zeros((4, 8), dtype=float)
    idx = 0
    for group in range(2):
        center = group_centers[:, group]
        for k in range(4):
            states[:, idx] = np.array([center[0] + offsets[0, k], center[1] + offsets[1, k], 0.8, 0.0])
            idx += 1
    return states


def all_pairs(nodes: range) -> list[tuple[int, int]]:
    node_list = list(nodes)
    return [(a, b) for i, a in enumerate(node_list[:-1]) for b in node_list[i + 1 :]]


def build_neighbor_edges() -> list[tuple[int, int]]:
    return all_pairs(range(1, 5)) + all_pairs(range(5, 9)) + [(1, 5), (2, 6), (3, 7), (4, 8)]


def build_target_birth_states() -> tuple[np.ndarray, np.ndarray]:
    target_center = np.array([0.0, 0.0])
    group_centers = np.array([[70.0, 80.0, 70.0], [80.0, 0.0, -80.0]])
    group_types = ["Triangle", "Triangle", "Leader3"]
    group_counts = [3, 3, 4]
    group_spacing = [30.0, 25.0, 20.0]
    group_speed = [0.45, 0.45, 0.45]
    states: list[np.ndarray] = []
    group_index: list[int] = []

    for group, count in enumerate(group_counts):
        offsets = local_formation_offsets(group_types[group], group_spacing[group], count)
        center = group_centers[:, group]
        direction = target_center - center
        if np.linalg.norm(direction) < 1e-6:
            direction = np.array([-1.0, 0.0])
        velocity = group_speed[group] * direction / np.linalg.norm(direction)
        for k in range(count):
            position = center + offsets[:, k]
            states.append(np.array([position[0], position[1], velocity[0], velocity[1]]))
            group_index.append(group)

    return np.asarray(states, dtype=float).T, np.asarray(group_index, dtype=int)


def build_main_config() -> dict:
    sim_length = 100
    sensor_velocity = np.array([0.8, 0.0])
    sensor_start = build_sensor_initial_states()
    sensor_end = sensor_start.copy()
    sensor_end[:2, :] += ((sim_length - 1) * sensor_velocity)[:, None]
    target_birth_states, target_group_index = build_target_birth_states()
    birth_times = 1 + np.arange(target_birth_states.shape[1]) * 8
    death_times = np.minimum(birth_times + 99, sim_length)
    return {
        "sim_length": sim_length,
        "sensor_start": sensor_start,
        "sensor_end": sensor_end,
        "sensor_velocity": sensor_velocity,
        "edges": build_neighbor_edges(),
        "target_birth_states": target_birth_states,
        "target_group_index": target_group_index,
        "birth_times": birth_times,
        "death_times": death_times,
        "p_drop_by_sensor": np.array([0.1, 0.1, 0.2, 0.1, 0.0, 0.5, 0.5, 0.1]),
    }


def setup_axis(ax: plt.Axes, title: str, xlim: tuple[int, int], ylim: tuple[int, int]) -> None:
    ax.set_title(title, fontweight="bold", pad=4.0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("x position", labelpad=1.5, color="#657386")
    ax.set_ylabel("y position", labelpad=1.5, color="#657386")
    ax.set_xticks([-100, -50, 0, 50, 100])
    ax.set_yticks([-100, -50, 0, 50, 100])
    ax.grid(True, color=GRID_COLOR, linewidth=0.45, alpha=0.65)
    ax.tick_params(direction="in", length=3.0, width=0.65, colors="#657386", top=True, right=True)
    for spine in ax.spines.values():
        spine.set_color("#6E7D90")
        spine.set_linewidth(0.65)


def draw_arrow(ax: plt.Axes, start: np.ndarray, end: np.ndarray, color: str, linewidth: float = 0.8) -> None:
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=5.8,
        linewidth=linewidth,
        color=color,
        shrinkA=0,
        shrinkB=0,
        joinstyle="miter",
        capstyle="round",
    )
    ax.add_patch(patch)


def draw_sensor(ax: plt.Axes, position: np.ndarray, label: str, fill: str) -> None:
    circle = Circle(position, radius=6.8, facecolor=fill, edgecolor=EDGE_COLOR, linewidth=0.75, zorder=6)
    ax.add_patch(circle)
    ax.text(
        position[0],
        position[1],
        label,
        ha="center",
        va="center",
        fontsize=5.0,
        fontweight="bold",
        color="#17202A",
        zorder=7,
    )


def draw_ideal_panel(ax: plt.Axes, cfg: dict) -> None:
    setup_axis(ax, "(a) Standard fixed ideal scenario", (-112, 112), (-112, 112))
    ax.add_patch(
        Rectangle(
            (-100, -100),
            200,
            200,
            fill=False,
            linestyle=(0, (5, 4)),
            linewidth=0.85,
            edgecolor="#8EABC9",
            zorder=1,
        )
    )
    ax.text(52, -88, "common ROI", fontsize=6.3, color=TEXT_COLOR, ha="left", va="center")

    sensor_positions = cfg["sensor_positions"]
    for a, b in cfg["sensor_edges"]:
        p1 = sensor_positions[:, a - 1]
        p2 = sensor_positions[:, b - 1]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=MOTION_COLOR, linewidth=0.75, zorder=2)
    for s in range(sensor_positions.shape[1]):
        draw_sensor(ax, sensor_positions[:, s], f"S{s + 1}", DROP_COLORS[0.0])

    birth_groups = sorted(set(cfg["birth_times"]))
    for i in range(cfg["target_states"].shape[1]):
        x0 = cfg["target_states"][:, i]
        duration = cfg["death_times"][i] - cfg["birth_times"][i]
        p0 = x0[:2]
        p1 = x0[:2] + duration * x0[2:4]
        color = TARGET_COLORS[birth_groups.index(cfg["birth_times"][i]) % len(TARGET_COLORS)]
        draw_arrow(ax, p0, p1, color, linewidth=0.7)
        ax.plot(p0[0], p0[1], "o", markersize=2.7, markerfacecolor="white", markeredgecolor=color, markeredgewidth=0.65)

    ax.text(
        -82,
        89,
        "4 sensors, drop prob.=0",
        fontsize=6.2,
        color=TEXT_COLOR,
        ha="left",
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.86, "pad": 0.6},
    )
    ax.text(
        -82,
        79,
        "fixed births: t=1,20,40,60",
        fontsize=6.2,
        color=TEXT_COLOR,
        ha="left",
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.86, "pad": 0.6},
    )


def draw_fov_samples(ax: plt.Axes, cfg: dict) -> None:
    for sensor_id in (1, 2, 5, 6):
        p = cfg["sensor_start"][:2, sensor_id - 1]
        wedge = Wedge(
            p,
            r=78,
            theta1=-60,
            theta2=60,
            width=0.0,
            fill=False,
            linewidth=0.45,
            edgecolor=FOV_COLOR,
            alpha=0.75,
            zorder=1,
        )
        ax.add_patch(wedge)


def draw_main_panel(ax: plt.Axes, cfg: dict) -> None:
    setup_axis(ax, "(b) Main tiered-loss scenario", (-132, 112), (-114, 112))
    draw_fov_samples(ax, cfg)

    sensor_start = cfg["sensor_start"]
    for a, b in cfg["edges"]:
        p1 = sensor_start[:2, a - 1]
        p2 = sensor_start[:2, b - 1]
        linestyle = "--" if abs(a - b) == 4 else "-"
        linewidth = 0.7 if linestyle == "-" else 0.8
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], linestyle=linestyle, color="#667381", linewidth=linewidth, zorder=2)

    for s in range(sensor_start.shape[1]):
        p0 = cfg["sensor_start"][:2, s]
        p1 = cfg["sensor_end"][:2, s]
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=MOTION_COLOR, linewidth=0.75, alpha=0.9, zorder=3)
        ax.plot(p1[0], p1[1], ".", color=MOTION_COLOR, markersize=3.2, zorder=3)
    draw_arrow(ax, np.array([-26.0, 86.0]), np.array([-7.0, 86.0]), MOTION_COLOR, linewidth=0.75)

    for s, drop in enumerate(cfg["p_drop_by_sensor"]):
        draw_sensor(ax, cfg["sensor_start"][:2, s], f"S{s + 1}", DROP_COLORS[float(drop)])

    for i in range(cfg["target_birth_states"].shape[1]):
        x0 = cfg["target_birth_states"][:, i]
        duration = cfg["death_times"][i] - cfg["birth_times"][i]
        p0 = x0[:2]
        p1 = x0[:2] + duration * x0[2:4]
        color = TARGET_COLORS[cfg["target_group_index"][i]]
        draw_arrow(ax, p0, p1, color, linewidth=0.72)
        ax.plot(p0[0], p0[1], "o", markersize=2.7, markerfacecolor="white", markeredgecolor=color, markeredgewidth=0.65)

    ax.text(-122, 69, "group A", fontsize=6.4, fontweight="bold", color=TEXT_COLOR)
    ax.text(-122, -70, "group B", fontsize=6.4, fontweight="bold", color=TEXT_COLOR)
    ax.text(-33, 93, "mobile sensors", fontsize=6.2, color=TEXT_COLOR, ha="center")
    ax.text(-33, 84, "v=[0.8,0]", fontsize=6.1, color=TEXT_COLOR, ha="center")
    ax.text(54, 88, "wave 1 (3)", fontsize=6.2, color=TEXT_COLOR, ha="left")
    ax.text(82, 2, "wave 2 (3)", fontsize=6.2, color=TEXT_COLOR, ha="left")
    ax.text(47, -93, "wave 3 (4)", fontsize=6.2, color=TEXT_COLOR, ha="left")
    ax.text(-127, 104, "Mmax=80; tiers [1,4,1,2]", fontsize=6.0, color=TEXT_COLOR, ha="left")

    legend_x = -123
    legend_y = -103
    ax.text(legend_x, legend_y + 10.0, "drop prob. by sensor", fontsize=6.2, fontweight="bold", color=TEXT_COLOR)
    for idx, level in enumerate([0.0, 0.1, 0.2, 0.5]):
        x = legend_x + idx * 30
        ax.add_patch(Circle((x, legend_y), radius=4.1, facecolor=DROP_COLORS[level], edgecolor=EDGE_COLOR, linewidth=0.6, zorder=6))
        ax.text(x + 6.4, legend_y, f"{level:g}", fontsize=6.1, va="center", color=TEXT_COLOR)


def render(output_dir: Path, paper_fig_dir: Path | None = None) -> list[Path]:
    apply_style()
    output_dir.mkdir(parents=True, exist_ok=True)
    if paper_fig_dir is not None:
        paper_fig_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(6.35, 3.05))
    draw_ideal_panel(axes[0], build_standard_ideal_config())
    draw_main_panel(axes[1], build_main_config())

    handles = [
        Line2D([0], [0], color=MOTION_COLOR, lw=0.8, label="sensor links/motion"),
        Line2D([0], [0], color=TARGET_COLORS[0], lw=0.8, marker="o", markersize=3, markerfacecolor="white", label="target tracks"),
        Line2D([0], [0], color="#8EABC9", lw=0.8, linestyle=(0, (5, 4)), label="common ROI"),
    ]
    fig.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.50, 0.018),
        ncol=3,
        frameon=False,
        fontsize=6.4,
        handlelength=1.8,
        columnspacing=1.3,
    )
    fig.subplots_adjust(left=0.055, right=0.995, top=0.89, bottom=0.18, wspace=0.19)

    source_pdf = output_dir / "figure_simulation_scenario.pdf"
    fig.savefig(source_pdf, format="pdf")
    paths = [source_pdf]
    if paper_fig_dir is not None:
        paper_pdf = paper_fig_dir / "paper-figure-scenario.pdf"
        fig.savefig(paper_pdf, format="pdf")
        paths.append(paper_pdf)
    plt.close(fig)
    return paths


def build_arg_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Render the paper scenario schematic.")
    parser.add_argument("--output-dir", type=Path, default=script_dir)
    parser.add_argument(
        "--paper-fig-dir",
        type=Path,
        default=script_dir.parent / "els-cas-templates" / "figs",
        help="Directory for the manuscript-adopted copy.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    for path in render(args.output_dir, args.paper_fig_dir):
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
