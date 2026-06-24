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
        (38, 46, 188, 90, "Neighborhood LMBs", "different local labels", "#E8F1F8", "#1F4E79"),
        (270, 46, 180, 90, "Reference", "median-cardinality medoid", "#F7F7F7", "#4D4D4D"),
        (494, 46, 172, 90, "Assignment", "Hungarian component map", "#FEF0D9", "#D55E00"),
        (710, 46, 186, 90, "Barycenter", "matched posterior moments", "#EAF4EA", "#009E73"),
        (940, 46, 140, 90, "Output", "reference labels", "#F7F7F7", "#4D4D4D"),
    ]
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="320" viewBox="0 0 1120 320">',
        '<rect width="1120" height="320" fill="white"/>',
        '<style>text{font-family:Verdana;} .h{font-size:22px;font-weight:700;fill:#111} .s{font-size:17px;fill:#333} .tiny{font-size:15px;fill:#333}</style>',
        '<text class="h" x="560" y="28" text-anchor="middle">Neighborhood label-barycenter projection</text>',
    ]
    for x, y, w, h, title, sub, fill, stroke in boxes:
        svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7" fill="{fill}" stroke="{stroke}" stroke-width="2.2"/>')
        svg.append(f'<text class="h" x="{x + w/2}" y="{y + 36}" text-anchor="middle">{title}</text>')
        svg.append(f'<text class="s" x="{x + w/2}" y="{y + 66}" text-anchor="middle">{sub}</text>')
    for x1, x2 in [(226, 270), (450, 494), (666, 710), (896, 940)]:
        y = 92
        svg.append(f'<line x1="{x1}" y1="{y}" x2="{x2-12}" y2="{y}" stroke="#4D4D4D" stroke-width="2.5"/>')
        svg.append(f'<polygon points="{x2-12},{y-7} {x2},{y} {x2-12},{y+7}" fill="#4D4D4D"/>')
    svg.extend(
        [
            '<rect x="62" y="184" width="376" height="82" rx="6" fill="#FFFFFF" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="82" y="212">Residual after scalar AA: weights choose probability mass</text>',
            '<text class="tiny" x="82" y="238">but do not infer the b-to-d component correspondence.</text>',
            '<line x1="458" y1="225" x2="652" y2="225" stroke="#4D4D4D" stroke-width="2.5"/>',
            '<polygon points="652,218 666,225 652,232" fill="#4D4D4D"/>',
            '<rect x="688" y="184" width="370" height="82" rx="6" fill="#FFFFFF" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="708" y="212">Projection effect: keep reference labels and active existence</text>',
            '<text class="tiny" x="708" y="238">scores, then average matched states as posterior moments.</text>',
            '<text class="tiny" x="560" y="296" text-anchor="middle">Repeat for H graph-local rounds over N_s; no global label dictionary is read or constructed.</text>',
        ]
    )
    svg.append("</svg>")
    return "\n".join(svg)


def method_pipeline_tex() -> str:
    return r"""\begingroup
\setlength{\unitlength}{1pt}
\setlength{\fboxsep}{0pt}
\begin{picture}(236,172)
\thicklines
\put(0,158){\makebox(236,8){\scriptsize\bfseries Neighborhood label-barycenter projection}}
\put(0,147){\makebox(236,7){\tiny graph-local label alignment before moment fusion}}

\put(0,122){\fcolorbox{black}{black!4}{\parbox[c][30pt][c]{52pt}{\centering\scriptsize Neighborhood\\[-1pt]\scriptsize LMBs\\[-1pt]\tiny label sets differ}}}
\put(62,122){\fcolorbox{black}{black!4}{\parbox[c][30pt][c]{42pt}{\centering\scriptsize Reference\\[-1pt]\tiny median-card.\\[-1pt]\tiny medoid}}}
\put(114,122){\fcolorbox{black}{black!4}{\parbox[c][30pt][c]{44pt}{\centering\scriptsize Assignment\\[-1pt]\tiny Hungarian\\[-1pt]\tiny map}}}
\put(168,122){\fcolorbox{black}{black!4}{\parbox[c][30pt][c]{58pt}{\centering\scriptsize Barycenter\\[-1pt]\tiny matched posterior\\[-1pt]\tiny moments}}}
\put(54,137){\vector(1,0){8}}
\put(106,137){\vector(1,0){8}}
\put(160,137){\vector(1,0){8}}
\put(26,115){\makebox(0,0){\tiny 1}}
\put(83,115){\makebox(0,0){\tiny 2}}
\put(136,115){\makebox(0,0){\tiny 3}}
\put(197,115){\makebox(0,0){\tiny 4}}

\put(2,76){\framebox(108,32){\shortstack{\scriptsize Residual after scalar AA\\[-1pt]\tiny weights choose probability mass\\[-1pt]\tiny but not component match}}}
\put(126,76){\framebox(108,32){\shortstack{\scriptsize Projection effect\\[-1pt]\tiny reference labels and existence kept\\[-1pt]\tiny matched states averaged}}}
\put(110,92){\vector(1,0){16}}

\put(13,24){\fcolorbox{black}{black!4}{\parbox[c][28pt][c]{210pt}{\centering\scriptsize Repeat for $H$ rounds over $\mathcal{N}_s$; no global label dictionary is read or constructed\\[-1pt]\tiny output remains an estimate-level projection layer}}}
\put(118,76){\vector(0,-1){24}}
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
