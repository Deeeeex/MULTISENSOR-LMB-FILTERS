#!/usr/bin/env python3
"""Render TAES manuscript figures without matplotlib."""

from __future__ import annotations

import shutil
import subprocess
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIG = ROOT / "figures"
GEN = ROOT / "generated"


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def convert_svg(svg: Path, target: Path, density: int = 300) -> None:
    converter = shutil.which("magick") or shutil.which("convert")
    if converter is None:
        return
    cmd = [
        converter,
        "-density",
        str(density),
        str(svg),
        "-quality",
        "95",
        str(target),
    ]
    if Path(converter).name == "magick":
        cmd.insert(1, "convert")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        target.unlink(missing_ok=True)


def method_pipeline_svg() -> str:
    boxes = [
        (90, 120, 250, 82, "Reference", "medoid label set", "#F7F7F7", "#4D4D4D"),
        (430, 120, 250, 82, "Assignment", "component map", "#FEF0D9", "#D55E00"),
        (770, 120, 260, 82, "Barycenter", "matched moments", "#EAF4EA", "#009E73"),
    ]
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="430" viewBox="0 0 1120 430">',
        '<rect width="1120" height="430" fill="white"/>',
        '<style>text{font-family:Verdana;} .h{font-size:25px;font-weight:700;fill:#111} .s{font-size:19px;fill:#333} .tiny{font-size:18px;fill:#333}</style>',
        '<text class="h" x="560" y="36" text-anchor="middle">Neighborhood label-barycenter projection</text>',
        '<rect x="110" y="62" width="900" height="42" rx="5" fill="#FFFFFF" stroke="#777" stroke-width="1.6"/>',
        '<text class="tiny" x="560" y="89" text-anchor="middle">Scalar AA weights choose probability mass, not component correspondence.</text>',
    ]
    for x, y, w, h, title, sub, fill, stroke in boxes:
        svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7" fill="{fill}" stroke="{stroke}" stroke-width="2.2"/>')
        svg.append(f'<text class="h" x="{x + w/2}" y="{y + 34}" text-anchor="middle">{title}</text>')
        svg.append(f'<text class="s" x="{x + w/2}" y="{y + 63}" text-anchor="middle">{sub}</text>')
    for x1, x2 in [(340, 430), (680, 770)]:
        y = 161
        svg.append(f'<line x1="{x1}" y1="{y}" x2="{x2-12}" y2="{y}" stroke="#4D4D4D" stroke-width="2.5"/>')
        svg.append(f'<polygon points="{x2-12},{y-7} {x2},{y} {x2-12},{y+7}" fill="#4D4D4D"/>')
    svg.extend(
        [
            '<rect x="90" y="250" width="430" height="48" rx="5" fill="#FFFFFF" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="305" y="280" text-anchor="middle">Existence pass-through: keep active AA existence scores.</text>',
            '<rect x="600" y="250" width="430" height="48" rx="5" fill="#FFFFFF" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="815" y="280" text-anchor="middle">Moment projection: average only assigned posterior states.</text>',
            '<rect x="120" y="336" width="880" height="48" rx="5" fill="#F7F7F7" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="560" y="366" text-anchor="middle">Repeat for H graph-local rounds over N_s; no global label dictionary is read or constructed.</text>',
        ]
    )
    svg.append("</svg>")
    return "\n".join(svg)


def method_pipeline_tex() -> str:
    return r"""\begingroup
\setlength{\unitlength}{1pt}
\setlength{\fboxsep}{0pt}
\begin{picture}(236,148)
\thicklines
\put(0,138){\makebox(236,8){\scriptsize\bfseries Neighborhood label-barycenter projection}}
\put(8,112){\framebox(220,22){\shortstack{\scriptsize Scalar AA weights choose probability mass\\[-1pt]\scriptsize not component correspondence.}}}

\put(5,74){\fcolorbox{black}{black!4}{\parbox[c][31pt][c]{62pt}{\centering\scriptsize\bfseries Reference\\[-1pt]\tiny medoid label set}}}
\put(87,74){\fcolorbox{black}{black!4}{\parbox[c][31pt][c]{62pt}{\centering\scriptsize\bfseries Assignment\\[-1pt]\tiny component map}}}
\put(169,74){\fcolorbox{black}{black!4}{\parbox[c][31pt][c]{62pt}{\centering\scriptsize\bfseries Barycenter\\[-1pt]\tiny matched moments}}}
\put(67,89){\vector(1,0){20}}
\put(149,89){\vector(1,0){20}}
\put(36,68){\makebox(0,0){\tiny 1}}
\put(118,68){\makebox(0,0){\tiny 2}}
\put(200,68){\makebox(0,0){\tiny 3}}

\put(4,38){\framebox(108,19){\shortstack{\scriptsize Existence pass-through\\[-1pt]\tiny keep active AA scores}}}
\put(124,38){\framebox(108,19){\shortstack{\scriptsize Moment projection\\[-1pt]\tiny average assigned states}}}
\put(112,48){\vector(1,0){12}}

\put(12,9){\fcolorbox{black}{black!4}{\parbox[c][19pt][c]{212pt}{\centering\scriptsize Repeat for $H$ rounds over $\mathcal{N}_s$; no global label dictionary is read or constructed.}}}
\put(118,38){\vector(0,-1){10}}
\end{picture}
\endgroup
"""


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
        '<style>text{font-family:Verdana;} .axis{font-size:20px;fill:#222} .small{font-size:17px;fill:#333} .label{font-size:19px;fill:#111}</style>',
        '<text class="axis" x="560" y="36" text-anchor="middle" font-weight="700">Paired reduction over fixed spatial-KLA AA, 50 trials</text>',
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
    GEN.mkdir(parents=True, exist_ok=True)
    render_png = os.environ.get("TAES_RENDER_PNG", "0") == "1"
    render_pdf = os.environ.get("TAES_RENDER_PDF", "0") == "1"
    assets = {
        "fig_method_pipeline.svg": method_pipeline_svg(),
        "fig_n50_results.svg": n50_results_svg(),
    }
    for name, svg_text in assets.items():
        svg_path = FIG / name
        write(svg_path, svg_text)
        if render_pdf:
            convert_svg(svg_path, svg_path.with_suffix(".pdf"))
        if render_png:
            convert_svg(svg_path, svg_path.with_suffix(".png"), density=240)
    write(GEN / "method_pipeline.tex", method_pipeline_tex())


if __name__ == "__main__":
    main()
