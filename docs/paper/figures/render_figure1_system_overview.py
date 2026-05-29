from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.path import Path as MplPath


SLATE = "#263443"
MUTED = "#58677a"
PANEL = "#d9e3ee"
BLUE = "#2f6fa5"
BLUE_EDGE = "#3f76a8"
BLUE_FILL = "#edf6fc"
RED = "#9a3d3d"
RED_EDGE = "#b65a5a"
RED_FILL = "#fff8f7"
GREEN = "#25865b"
LIGHT_BG = "#fbfdff"
CALLOUT = "#b8c4d1"
TITLE_SIZE = 10.8
SECTION_SIZE = 8.5
CARD_TITLE_SIZE = 7.4
CARD_SUBTITLE_SIZE = 6.4
NODE_SIZE = 8.0


def _setup_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def _box(ax, x, y, w, h, *, edge=PANEL, face="white", lw=1.1, radius=0.08, z=1):
    patch = patches.FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle=f"round,pad=0.012,rounding_size={radius}",
        linewidth=lw,
        edgecolor=edge,
        facecolor=face,
        zorder=z,
    )
    ax.add_patch(patch)
    return patch


def _node(ax, x, y, label, *, focus=False):
    radius = 0.27 if focus else 0.22
    ax.add_patch(
        patches.Circle(
            (x, y),
            radius,
            facecolor="#fff5f2" if focus else "white",
            edgecolor=RED if focus else "#607080",
            linewidth=1.25 if focus else 1.05,
            zorder=5,
        )
    )
    ax.text(
        x,
        y,
        label,
        ha="center",
        va="center",
        fontsize=NODE_SIZE,
        color=RED if focus else SLATE,
        fontweight="bold",
        zorder=6,
    )


def _line(ax, x1, y1, x2, y2, *, color=BLUE, lw=1.25, style="solid", z=3):
    ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw, linestyle=style, zorder=z)


def _path(ax, points, *, color=SLATE, lw=1.15, style="solid", arrow=True, z=4):
    codes = [MplPath.MOVETO]
    verts = [points[0]]
    for point in points[1:]:
        codes.append(MplPath.LINETO)
        verts.append(point)
    patch = patches.FancyArrowPatch(
        path=MplPath(verts, codes),
        arrowstyle="-|>" if arrow else "-",
        mutation_scale=13,
        linewidth=lw,
        linestyle=style,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=z,
    )
    ax.add_patch(patch)


def _curve(ax, verts, *, color=SLATE, lw=1.15, style="solid", arrow=True, z=4):
    codes = [MplPath.MOVETO, MplPath.CURVE4, MplPath.CURVE4, MplPath.CURVE4]
    patch = patches.FancyArrowPatch(
        path=MplPath(verts, codes),
        arrowstyle="-|>" if arrow else "-",
        mutation_scale=13,
        linewidth=lw,
        linestyle=style,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=z,
    )
    ax.add_patch(patch)


def _card(
    ax,
    x,
    y,
    w,
    h,
    title,
    subtitle=None,
    *,
    edge=BLUE_EDGE,
    face="white",
    title_color=SLATE,
    title_size=CARD_TITLE_SIZE,
    subtitle_size=CARD_SUBTITLE_SIZE,
):
    _box(ax, x, y, w, h, edge=edge, face=face, lw=1.1, radius=0.07, z=2)
    ax.text(
        x + w / 2,
        y + h * 0.62,
        title,
        ha="center",
        va="center",
        fontsize=title_size,
        fontweight="bold",
        color=title_color,
        zorder=5,
    )
    if subtitle:
        if isinstance(subtitle, str):
            subtitle = [subtitle]
        start = y + h * 0.38 + (len(subtitle) - 1) * 0.12
        for index, line in enumerate(subtitle):
            ax.text(
                x + w / 2,
                start - index * 0.28,
                line,
                ha="center",
                va="center",
                fontsize=subtitle_size,
                color=SLATE,
                zorder=5,
            )


def draw_figure() -> plt.Figure:
    _setup_style()
    fig, ax = plt.subplots(figsize=(13.2, 7.0))
    ax.set_xlim(0.0, 17.0)
    ax.set_ylim(0.0, 9.0)
    ax.axis("off")

    # Left network context.
    _box(ax, 0.12, 0.16, 5.95, 8.55, edge="#8b9aab", face="white", lw=1.0, radius=0.08)
    ax.text(
        3.1,
        8.35,
        "Distributed formation-tracking setting",
        ha="center",
        va="center",
        fontsize=TITLE_SIZE,
        fontweight="bold",
        color=SLATE,
    )

    _box(ax, 0.38, 5.20, 5.25, 2.85, edge="#b7c6d8", face=LIGHT_BG, lw=0.9, radius=0.07)
    _box(ax, 0.38, 1.92, 4.75, 2.65, edge="#b7c6d8", face=LIGHT_BG, lw=0.9, radius=0.07)
    ax.text(3.0, 7.74, "Formation A", ha="center", va="center", fontsize=8.6, fontweight="bold", color="#2d65a0")
    ax.text(2.7, 4.27, "Formation B", ha="center", va="center", fontsize=8.6, fontweight="bold", color="#2d65a0")

    nodes_a = {"S1": (1.42, 7.22), "S2": (4.12, 7.22), "S3": (1.42, 5.95), "s": (4.12, 5.95)}
    nodes_b = {"S5": (1.20, 3.75), "S6": (4.10, 3.75), "S7": (1.20, 2.55), "S8": (4.10, 2.55)}

    dash = (0, (5, 4))
    dot = (0, (1.2, 4.0))
    _line(ax, *nodes_a["S1"], *nodes_a["S2"])
    _line(ax, *nodes_a["S1"], *nodes_a["S3"], style=dash)
    _line(ax, *nodes_a["S2"], *nodes_a["s"])
    _line(ax, *nodes_a["S3"], *nodes_a["s"], style=dash)
    _line(ax, *nodes_a["S3"], *nodes_a["S2"], style=dot)
    _line(ax, *nodes_a["s"], *nodes_b["S5"], style=dot)

    _line(ax, *nodes_b["S5"], *nodes_b["S6"])
    _line(ax, *nodes_b["S5"], *nodes_b["S7"], style=dot)
    _line(ax, *nodes_b["S6"], *nodes_b["S8"], style=dash)
    _line(ax, *nodes_b["S7"], *nodes_b["S8"])
    _line(ax, *nodes_b["S7"], *nodes_b["S6"], style=dot)

    for label, (x, y) in nodes_a.items():
        _node(ax, x, y, label, focus=(label == "s"))
    for label, (x, y) in nodes_b.items():
        _node(ax, x, y, label)

    legend_x = 0.82
    legend_y = 1.45
    _line(ax, legend_x, legend_y, legend_x + 0.62, legend_y, lw=1.3)
    _line(ax, legend_x, legend_y - 0.38, legend_x + 0.62, legend_y - 0.38, lw=1.3, style=dash)
    _line(ax, legend_x, legend_y - 0.76, legend_x + 0.62, legend_y - 0.76, lw=1.3, style=dot)
    ax.text(2.25, legend_y, "high-reliability", ha="left", va="center", fontsize=7.4, color=MUTED)
    ax.text(2.25, legend_y - 0.38, "moderate", ha="left", va="center", fontsize=7.4, color=MUTED)
    ax.text(2.25, legend_y - 0.76, "lossy", ha="left", va="center", fontsize=7.4, color=MUTED)

    # Subtle, non-data-flow expansion cue from node s to the local processing view.
    _curve(ax, [(4.40, 6.10), (5.40, 6.75), (6.15, 7.35), (7.05, 7.78)], color=CALLOUT, lw=0.9, style=dash, arrow=False, z=1)
    _curve(ax, [(4.35, 5.75), (5.25, 4.45), (6.00, 2.65), (7.05, 1.66)], color=CALLOUT, lw=0.9, style=dash, arrow=False, z=1)

    # Expanded local node view.
    ax.text(
        12.1,
        8.48,
        "Representative node s local processing",
        ha="center",
        va="center",
        fontsize=TITLE_SIZE,
        fontweight="bold",
        color=SLATE,
    )
    _box(ax, 7.08, 0.16, 9.76, 8.18, edge="#8b9aab", face="white", lw=1.0, radius=0.08)
    _node(ax, 7.62, 7.95, "s", focus=True)
    ax.text(
        8.07,
        7.95,
        "expanded local view of node s",
        ha="left",
        va="center",
        fontsize=8.0,
        style="italic",
        color=MUTED,
    )

    _box(ax, 7.36, 0.74, 3.92, 7.00, edge="#6f99c8", face="#fbfdff", lw=0.95, radius=0.08)
    _box(ax, 11.58, 0.74, 4.98, 7.00, edge="#d27a7a", face="#fffdfc", lw=0.95, radius=0.08)
    ax.text(9.32, 7.44, "Evidence available at node s", ha="center", va="center", fontsize=SECTION_SIZE, fontweight="bold", color="#245e98")
    ax.text(14.07, 7.44, "Branch-decoupled adaptive fusion", ha="center", va="center", fontsize=SECTION_SIZE, fontweight="bold", color=RED)

    _card(
        ax,
        7.78,
        6.08,
        3.00,
        1.08,
        "Local measurements",
        "measurement update at node s",
        edge=BLUE_EDGE,
    )
    _card(
        ax,
        7.78,
        4.25,
        3.00,
        1.08,
        "Local LMB posterior",
        "kinematic states and existence",
        edge=BLUE_EDGE,
        face=BLUE_FILL,
    )
    _card(
        ax,
        7.78,
        2.10,
        3.00,
        1.08,
        "Neighbor information",
        ["neighbor posteriors", "and link outcomes"],
        edge=BLUE_EDGE,
        face=BLUE_FILL,
    )
    _path(ax, [(9.28, 6.08), (9.28, 5.33)], color=SLATE, lw=1.0)

    # Fusion area and internal data/control flow.
    _box(ax, 12.22, 1.25, 4.08, 5.88, edge=RED_EDGE, face="#fffdfc", lw=0.95, radius=0.07)
    _card(
        ax,
        12.78,
        5.92,
        3.42,
        0.98,
        "Quality cues",
        ["shared: precision, link reliability", "existence cue on Bernoulli path"],
        edge=RED_EDGE,
        face=RED_FILL,
        title_color=RED,
        subtitle_size=6.2,
    )
    _card(
        ax,
        12.78,
        4.68,
        3.42,
        0.78,
        "Shared quality score",
        "common evidence before branch split",
        edge=RED_EDGE,
        face=RED_FILL,
        title_color=RED,
        subtitle_size=6.2,
    )
    _card(
        ax,
        12.30,
        2.68,
        1.95,
        1.26,
        "Spatial path",
        "kinematic KLA fusion",
        edge=RED_EDGE,
        face=RED_FILL,
        title_color=RED,
        title_size=7.1,
        subtitle_size=6.1,
    )
    _card(
        ax,
        14.52,
        2.68,
        1.78,
        1.26,
        "Existence path",
        "Bernoulli fusion",
        edge=RED_EDGE,
        face=RED_FILL,
        title_color=RED,
        title_size=7.0,
        subtitle_size=6.1,
    )
    _box(ax, 14.84, 2.78, 1.15, 0.34, edge=GREEN, face="#f2fbf6", lw=0.9, radius=0.04, z=3)
    ax.text(15.42, 2.95, "FID-FIA cue", ha="center", va="center", fontsize=7.2, fontweight="bold", color=GREEN, zorder=5)
    _card(
        ax,
        13.20,
        1.54,
        3.02,
        0.78,
        "Fused local posterior",
        "extraction and forwarding",
        edge=RED_EDGE,
        face=RED_FILL,
        title_color=RED,
        title_size=7.3,
        subtitle_size=6.3,
    )

    _path(ax, [(10.78, 4.79), (11.78, 4.79), (11.78, 6.42), (12.78, 6.42)], color=SLATE, lw=1.05)
    _curve(ax, [(10.78, 2.64), (11.50, 3.15), (11.70, 5.70), (12.78, 6.18)], color=BLUE, lw=1.05)
    _path(ax, [(14.49, 5.92), (14.49, 5.46)], color=SLATE, lw=1.0)
    _path(ax, [(14.49, 4.68), (13.30, 3.94)], color=SLATE, lw=1.0)
    _path(ax, [(14.49, 4.68), (15.41, 3.94)], color=SLATE, lw=1.0)
    _path(ax, [(13.27, 2.68), (14.00, 2.32)], color=SLATE, lw=1.0)
    _path(ax, [(15.41, 2.68), (14.86, 2.32)], color=SLATE, lw=1.0)

    return fig


def render(output_dir: Path, paper_fig_dir: Path, *, dpi: int = 300) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paper_fig_dir.mkdir(parents=True, exist_ok=True)

    fig = draw_figure()
    outputs = {
        "source_pdf": output_dir / "figure1_v2.pdf",
        "source_png": output_dir / "figure1_v2.png",
        "paper_pdf": paper_fig_dir / "paper-figure1-v2.pdf",
        "paper_png": paper_fig_dir / "paper-figure1-v2.png",
    }
    for path in (outputs["source_pdf"], outputs["paper_pdf"]):
        fig.savefig(path, format="pdf", bbox_inches="tight", pad_inches=0.015)
    for path in (outputs["source_png"], outputs["paper_png"]):
        fig.savefig(path, format="png", dpi=dpi, bbox_inches="tight", pad_inches=0.015)
    plt.close(fig)
    return outputs


def build_arg_parser() -> argparse.ArgumentParser:
    default_output = Path(__file__).resolve().parent
    default_paper = default_output.parent / "els-cas-templates" / "figs"
    parser = argparse.ArgumentParser(description="Render Figure 1 as a code-generated vector PDF.")
    parser.add_argument("--output-dir", default=str(default_output), help="Directory for source figure outputs.")
    parser.add_argument("--paper-fig-dir", default=str(default_paper), help="Directory for manuscript figure outputs.")
    parser.add_argument("--dpi", type=int, default=300, help="PNG preview DPI.")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    outputs = render(Path(args.output_dir), Path(args.paper_fig_dir), dpi=args.dpi)
    for name, path in outputs.items():
        print(f"{name}: {path}")


if __name__ == "__main__":
    main()
