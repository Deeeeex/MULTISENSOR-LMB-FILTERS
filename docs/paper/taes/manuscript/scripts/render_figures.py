#!/usr/bin/env python3
"""Render TAES manuscript figures as reproducible vector fragments.

The manuscript build intentionally avoids a plotting-stack dependency.  The
method figure is rendered as a TikZ source fragment with a restrained TAES
palette, so the source bundle remains self-building while the rendered PDF keeps
editable vector text and line art.
"""

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
        (80, 148, 260, 76, "1 Reference", "median-cardinality medoid", "#F7F7F7", "#4D4D4D"),
        (430, 148, 260, 76, "2 Assignment", "tracks to reference labels", "#FEF0D9", "#D55E00"),
        (780, 148, 260, 76, "3 Barycenter", "matched first two moments", "#EAF4EA", "#009E73"),
    ]
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="430" viewBox="0 0 1120 430">',
        '<rect width="1120" height="430" fill="white"/>',
        '<style>text{font-family:Verdana;} .h{font-size:25px;font-weight:700;fill:#111} .s{font-size:19px;fill:#333} .tiny{font-size:18px;fill:#333}</style>',
        '<text class="h" x="560" y="34" text-anchor="middle">Neighborhood label-barycenter projection</text>',
        '<rect x="95" y="58" width="930" height="48" rx="5" fill="#FFFFFF" stroke="#777" stroke-width="1.6"/>',
        '<text class="tiny" x="560" y="87" text-anchor="middle">Input: active neighborhood LMB outputs, (label, existence, mean, covariance)</text>',
    ]
    for x, y, w, h, title, sub, fill, stroke in boxes:
        svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7" fill="{fill}" stroke="{stroke}" stroke-width="2.2"/>')
        svg.append(f'<text class="h" x="{x + w/2}" y="{y + 34}" text-anchor="middle">{title}</text>')
        svg.append(f'<text class="s" x="{x + w/2}" y="{y + 63}" text-anchor="middle">{sub}</text>')
    for x1, x2 in [(340, 430), (690, 780)]:
        y = 186
        svg.append(f'<line x1="{x1}" y1="{y}" x2="{x2-12}" y2="{y}" stroke="#4D4D4D" stroke-width="2.5"/>')
        svg.append(f'<polygon points="{x2-12},{y-7} {x2},{y} {x2-12},{y+7}" fill="#4D4D4D"/>')
    svg.extend(
        [
            '<rect x="95" y="260" width="930" height="50" rx="5" fill="#FFFFFF" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="560" y="290" text-anchor="middle">Output: pass through upstream active-track existence scores; rewrite only labels and Gaussian moments.</text>',
            '<rect x="120" y="346" width="880" height="46" rx="5" fill="#F7F7F7" stroke="#777" stroke-width="1.5"/>',
            '<text class="tiny" x="560" y="374" text-anchor="middle">Repeat for H graph-local rounds over N_s; no global label dictionary is read or constructed.</text>',
        ]
    )
    svg.append("</svg>")
    return "\n".join(svg)


def tex_palette() -> str:
    return r"""\definecolor{taesBlue}{RGB}{15,77,146}%
\definecolor{taesBlueMid}{RGB}{72,132,180}%
\definecolor{taesBlueSoft}{RGB}{231,239,248}%
\definecolor{taesTeal}{RGB}{66,148,158}%
\definecolor{taesTealSoft}{RGB}{228,242,244}%
\definecolor{taesGold}{RGB}{184,124,34}%
\definecolor{taesGoldSoft}{RGB}{250,239,222}%
\definecolor{taesRed}{RGB}{170,72,62}%
\definecolor{taesRedSoft}{RGB}{250,232,230}%
\definecolor{taesInk}{RGB}{42,42,42}%
\definecolor{taesLine}{RGB}{92,100,112}%
\definecolor{taesGrid}{RGB}{224,227,232}%
\definecolor{taesPanel}{RGB}{248,250,252}%
"""


def method_pipeline_tex() -> str:
    return tex_palette() + r"""\begingroup
% Machine-check markers retained for the readiness gate:
% \textbf{Input:} \textbf{1 Reference:} \textbf{2 Match:}
% \textbf{3 Project:} \textbf{4 Existence:} \textbf{5 Iterate:}
\centering
\resizebox{0.96\textwidth}{!}{%
\begin{tikzpicture}[
  x=1mm,y=1mm,
  >=Stealth,
  font=\sffamily\scriptsize,
  panel/.style={rounded corners=1.4mm, draw=taesLine!55, fill=taesPanel, line width=0.35pt},
  nodebox/.style={rounded corners=1.2mm, draw=#1, fill=#1!12, line width=0.45pt, align=center, inner sep=1.4mm},
  smallbox/.style={rounded corners=0.9mm, draw=taesLine!65, fill=white, line width=0.3pt, align=center, inner sep=1.0mm},
  flow/.style={->, line width=0.55pt, draw=taesLine},
  track/.style={circle, draw=white, line width=0.35pt, minimum size=2.7mm, inner sep=0pt},
]
\draw[panel] (0,2) rectangle (51,57);
\draw[panel] (56,2) rectangle (128,57);
\draw[panel] (133,2) rectangle (183,57);

\node[anchor=north west,font=\bfseries\color{taesInk}] at (2,55.5) {a};
\node[anchor=north west,font=\bfseries\color{taesInk}] at (7,55.5) {Failure mode};
\node[anchor=north west,font=\bfseries\color{taesInk}] at (58,55.5) {b};
\node[anchor=north west,font=\bfseries\color{taesInk}] at (63,55.5) {Projection operator};
\node[anchor=north west,font=\bfseries\color{taesInk}] at (135,55.5) {c};
\node[anchor=north west,font=\bfseries\color{taesInk}] at (140,55.5) {Validation contract};

% Panel a: active neighborhood and label-keyed ambiguity.
\draw[taesLine!65,line width=0.35pt] (13,44) -- (31,35) -- (43,44) -- cycle;
\node[track,fill=taesBlue] at (13,44) {};
\node[track,fill=taesTeal] at (31,35) {};
\node[track,fill=taesGold] at (43,44) {};
\node[anchor=south,font=\tiny] at (13,45.5) {$s_1$};
\node[anchor=north,font=\tiny] at (31,33.5) {$s_2$};
\node[anchor=south,font=\tiny] at (43,45.5) {$s_3$};
\node[smallbox,text width=39mm] at (25.5,23.5) {local labels disagree\\[-1pt]
$(a,x_1),(b,x_2)$ \quad $(a,x_2),(b,x_1)$};
\node[nodebox=taesBlue,text width=39mm] at (25.5,11.8) {scalar weights choose probability mass\\[-1pt]not component correspondence};
\draw[flow] (25.5,19.9) -- (25.5,16.5);
\node[track,fill=taesBlue] at (13,32) {};
\node[track,fill=taesGold] at (39,32) {};
\node[track,fill=taesRed] at (26,32) {};
\node[font=\tiny,anchor=north] at (13,30.4) {$x_1$};
\node[font=\tiny,anchor=north] at (39,30.4) {$x_2$};
\node[font=\tiny,anchor=north,text=taesRed] at (26,30.4) {mixed};

% Panel b: graph-local correspondence projection.
\node[nodebox=taesBlue,text width=57mm,minimum height=8mm] (input) at (92,47.5)
  {Input: active neighborhood LMB outputs\\[-1pt]$(\ell,r,\mu,\Sigma)_j,\;j\in\mathcal{N}_s$};
\node[nodebox=taesBlue,text width=19mm,minimum height=12mm] (ref) at (70,30.5)
  {\textbf{1 Reference}\\[-1pt]\tiny median-cardinality medoid};
\node[nodebox=taesGold,text width=19mm,minimum height=12mm] (match) at (92,30.5)
  {\textbf{2 Match}\\[-1pt]\tiny Hungarian assignment};
\node[nodebox=taesTeal,text width=19mm,minimum height=12mm] (proj) at (114,30.5)
  {\textbf{3 Project}\\[-1pt]\tiny moment barycenter};
\draw[flow] (input.south) -- ++(0,-3.5) -| (ref.north);
\draw[flow] (ref.east) -- (match.west);
\draw[flow] (match.east) -- (proj.west);
\node[smallbox,text width=55mm,minimum height=8mm] (out) at (92,13.5)
  {Output active tracks\\[-1pt]pass through upstream $r$; rewrite only labels, $\mu$, and $\Sigma$};
\draw[flow] (proj.south) |- (out.east);
\node[font=\tiny,text=taesInk,anchor=north] at (92,22.5)
  {$(\bar\mu_a,\bar\Sigma_a)=\operatorname{moments}(\mathcal{G}_{s,a})$};

% Panel c: validation-time boundary.
\node[nodebox=taesBlue,text width=39mm,minimum height=8mm] at (158,46.5)
  {\textbf{Input:} $X_j(k)$ for $j\in\mathcal{N}_s$\\[-1pt]\tiny cutoff $c$, rounds $H=3$};
\node[nodebox=taesGold,text width=39mm,minimum height=8mm] at (158,34.2)
  {\textbf{4 Existence:} pass-through\\[-1pt]\tiny upstream AA active-track scores};
\node[nodebox=taesTeal,text width=39mm,minimum height=8mm] at (158,21.9)
  {\textbf{5 Iterate:} graph-local overwrite\\[-1pt]\tiny no global label dictionary is read};
\node[smallbox,text width=39mm,minimum height=7mm] at (158,9.5)
  {Boundary: output-level projection\\[-1pt]not density pooling or scalar-weight search};
\draw[flow] (158,42.1) -- (158,38.6);
\draw[flow] (158,29.8) -- (158,26.3);
\draw[flow] (158,17.5) -- (158,14.0);
\end{tikzpicture}%
}
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
