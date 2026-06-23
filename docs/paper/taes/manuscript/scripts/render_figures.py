#!/usr/bin/env python3
"""Render first-pass TAES manuscript figures without matplotlib."""

from __future__ import annotations

import shutil
import subprocess
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "figures"


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def convert_svg_to_png(svg: Path, png: Path, density: int = 220) -> None:
    converter = shutil.which("convert")
    if converter is None:
        return
    try:
        subprocess.run(
            [
                converter,
                "-density",
                str(density),
                "-font",
                "Helvetica",
                str(svg),
                "-quality",
                "95",
                str(png),
            ],
            check=True,
        )
    except subprocess.CalledProcessError:
        png.unlink(missing_ok=True)


def method_pipeline_svg() -> str:
    boxes = [
        (40, 56, 170, 72, "Local LMB", "posteriors"),
        (250, 56, 190, 72, "Median-cardinality", "medoid reference"),
        (480, 56, 170, 72, "Hungarian", "label matching"),
        (690, 56, 190, 72, "Matched moment", "barycenter"),
        (920, 56, 170, 72, "Neighborhood", "iteration"),
    ]
    arrow = "#4D4D4D"
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="220" viewBox="0 0 1120 220">',
        '<rect width="1120" height="220" fill="white"/>',
        '<style>text{font-family:Helvetica,sans-serif;} .h{font-size:22px;font-weight:700;fill:#111} .s{font-size:18px;fill:#333}</style>',
    ]
    for x, y, w, h, title, sub in boxes:
        svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#F4F6F8" stroke="#1F4E79" stroke-width="2"/>')
        svg.append(f'<text class="h" x="{x + w/2}" y="{y + 30}" text-anchor="middle">{title}</text>')
        svg.append(f'<text class="s" x="{x + w/2}" y="{y + 56}" text-anchor="middle">{sub}</text>')
    for x1, x2 in [(210, 250), (440, 480), (650, 690), (880, 920)]:
        y = 92
        svg.append(f'<line x1="{x1}" y1="{y}" x2="{x2-12}" y2="{y}" stroke="{arrow}" stroke-width="2.5"/>')
        svg.append(f'<polygon points="{x2-12},{y-7} {x2},{y} {x2-12},{y+7}" fill="{arrow}"/>')
    svg.append('<text class="s" x="560" y="166" text-anchor="middle">Each sensor applies the operator over its communication neighborhood; repeated rounds propagate multi-hop information.</text>')
    svg.append("</svg>")
    return "\n".join(svg)


def n50_results_svg() -> str:
    metrics = [
        ("Network OSPA", 81.59, 38.78),
        ("Loc. disagreement", 85.31, 39.46),
        ("E-OSPA", 17.15, 5.83),
        ("RMSE", 6.35, 0.54),
        ("Card. error", 12.27, 12.27),
    ]
    width, height = 1120, 560
    left, top = 180, 80
    bar_h, gap = 28, 40
    scale_w = 760
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="1120" height="560" fill="white"/>',
        '<style>text{font-family:Helvetica,sans-serif;} .axis{font-size:20px;fill:#222} .small{font-size:17px;fill:#333} .label{font-size:19px;fill:#111}</style>',
        '<text class="axis" x="560" y="36" text-anchor="middle" font-weight="700">Paired reduction over tuned spatial-KLA AA, 50 trials</text>',
    ]
    for tick in range(0, 101, 20):
        x = left + tick / 100 * scale_w
        svg.append(f'<line x1="{x:.1f}" y1="62" x2="{x:.1f}" y2="430" stroke="#E0E0E0" stroke-width="1"/>')
        svg.append(f'<text class="small" x="{x:.1f}" y="456" text-anchor="middle">{tick}%</text>')
    for idx, (name, full, ref) in enumerate(metrics):
        y = top + idx * (2 * bar_h + gap)
        svg.append(f'<text class="label" x="{left-18}" y="{y+22}" text-anchor="end">{name}</text>')
        full_w = full / 100 * scale_w
        ref_w = ref / 100 * scale_w
        svg.append(f'<rect x="{left}" y="{y}" width="{full_w:.1f}" height="{bar_h}" fill="#0072B2"/>')
        svg.append(f'<rect x="{left}" y="{y+bar_h+6}" width="{ref_w:.1f}" height="{bar_h}" fill="#E69F00"/>')
        svg.append(f'<text class="small" x="{left+full_w+8:.1f}" y="{y+21}">{full:.2f}%</text>')
        svg.append(f'<text class="small" x="{left+ref_w+8:.1f}" y="{y+bar_h+27}">{ref:.2f}%</text>')
    svg.append('<rect x="746" y="486" width="26" height="18" fill="#0072B2"/><text class="small" x="780" y="501">Neighborhood label-barycenter</text>')
    svg.append('<rect x="746" y="518" width="26" height="18" fill="#E69F00"/><text class="small" x="780" y="533">Neighborhood reference-only</text>')
    svg.append('<text class="small" x="560" y="486" text-anchor="middle">Reference-only isolates label-set canonicalization; the full method adds matched posterior barycenters.</text>')
    svg.append("</svg>")
    return "\n".join(svg)


def main() -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    render_png = os.environ.get("TAES_RENDER_PNG", "0") == "1"
    assets = {
        "fig_method_pipeline.svg": method_pipeline_svg(),
        "fig_n50_results.svg": n50_results_svg(),
    }
    for name, svg_text in assets.items():
        svg_path = FIG / name
        write(svg_path, svg_text)
        if render_png:
            convert_svg_to_png(svg_path, svg_path.with_suffix(".png"))


if __name__ == "__main__":
    main()
