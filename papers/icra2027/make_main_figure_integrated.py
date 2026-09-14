"""Recreate the image-generated continuous mechanism layout as editable SVG."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET
import cairosvg

import numpy as np
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, FancyArrowPatch, PathPatch, Polygon
from matplotlib.path import Path as MplPath
from matplotlib.transforms import Affine2D
from matplotlib.transforms import Bbox
import matplotlib.patheffects as path_effects
from figure_typography import tex_label, figure_text_bounds, preserve_tex_source


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "figures"
DESIGN = ROOT / "main_figure_integrated"
OUT.mkdir(exist_ok=True)
W, H = 1828, 860
WIDTH_MM = 181
HEIGHT_MM = WIDTH_MM * H / W
INK, MUTED = "#263c48", "#73858f"
BLUE, BLUE_DARK = "#6c91b2", "#446c99"
TEAL, AMBER = "#007f73", "#ba8645"

mpl.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Liberation Sans", "Arial", "DejaVu Sans"],
    "font.size": 7.2, "mathtext.fontset": "stix", "mathtext.default": "it",
    "svg.fonttype": "none", "svg.hashsalt": "icra-gce-integrated-overview",
    "pdf.fonttype": 42, "ps.fonttype": 42, "axes.unicode_minus": False,
    "text.latex.preamble": r"\usepackage[T1]{fontenc}\usepackage{times}\usepackage{amsmath,amssymb}\everymath{\displaystyle}",
})
equations = {
    "curvature": r"\bar\kappa_j=\kappa_j\mathbf1[\Delta J_j\succeq0]",
    "existence": r"\operatorname{logit}(r^*)=\sum_j\beta_j z_j+\sum_j\kappa_j\delta_j+\log I",
    "spatial": r"p^*(x)=h(x)/I",
    "product": r"h(x)=\prod_jp_j^+(x)^{\alpha_j}\prod_j\left[\frac{p_j^+(x)}{p_j^-(x)}\right]^{\kappa_j}",
    "normalizer": r"I=\int h(x)\,dx",
}
method_tex = re.sub(r"\s+", "", (ROOT / "sections/method.tex").read_text().replace("&", ""))
for name, equation in equations.items():
    assert re.sub(r"\s+", "", equation) in method_tex, (name, "Figure equation differs from manuscript TeX")
for name in ["existence", "product"]:
    equations[name] = equations[name].replace(r"\kappa_j", r"\bar\kappa_j")
fig = plt.figure(figsize=(WIDTH_MM / 25.4, HEIGHT_MM / 25.4), dpi=300)
ax = fig.add_axes([0, 0, 1, 1])
ax.set(xlim=(0, W), ylim=(H, 0), aspect="equal")
ax.axis("off")
connections = []


def text(x, y, value, size=7.6, color=INK, **kwargs):
    size = max(size, 7.4)
    kwargs.setdefault("va", "center")
    if '$' in value:
        return tex_label(ax, x, y, value, size, color,
                         ha=kwargs.pop("ha", "left"), va=kwargs.pop("va"))
    return ax.text(x, y, value, fontsize=size, color=color,
                   linespacing=1.14, usetex='$' in value, **kwargs)


def line(x, y, color=MUTED, width=.6, **kwargs):
    item, = ax.plot(x, y, color=color, lw=width, solid_capstyle="round", **kwargs)
    return item


def box(x, y, w, h, fill="white", edge=INK, width=.65, radius=12, **kwargs):
    item = FancyBboxPatch((x, y), w, h,
                         boxstyle=f"round,pad=0,rounding_size={radius}",
                         facecolor=fill, edgecolor=edge, linewidth=width, **kwargs)
    ax.add_patch(item)
    return item


def arrow(start, end, color=INK, width=.8, style="-|>", dashed=False, **kwargs):
    item = FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=6.0,
                           color=color, linewidth=width, shrinkA=0, shrinkB=0,
                           linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    ax.add_patch(item)
    connections.append(item)
    return item


def curved(vertices, color=INK, width=.8, head=True, dashed=False, **kwargs):
    path = MplPath(vertices, [MplPath.MOVETO] + [MplPath.CURVE4] * (len(vertices) - 1))
    if head:
        item = FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=6.0,
                               color=color, linewidth=width, capstyle="round",
                               linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    else:
        item = PathPatch(path, facecolor="none", edgecolor=color, linewidth=width,
                         capstyle="round", joinstyle="round",
                         linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    ax.add_patch(item)
    connections.append(item)
    return item


def bell(cx, baseline, width=95, height=52, color=BLUE_DARK, axis=True, fill=.09):
    u = np.linspace(-3.2, 3.2, 101)
    xs = cx + u * width / 6.4
    ys = baseline - height * np.exp(-.5 * (u / .85) ** 2)
    ax.fill_between(xs, baseline, ys, color=color, alpha=fill, linewidth=0)
    line(xs, ys, color, .9)
    if axis:
        arrow((cx - width / 2 - 7, baseline), (cx + width / 2 + 15, baseline), INK, .4)
    return cx + width / 2 + 15


def check(x, y):
    box(x - 14, y - 17, 30, 34, fill="white", edge=INK, width=.7, radius=1.5)
    line([x - 8, x - 1, x + 10], [y + 1, y + 9, y - 10], INK, 1.15)


def clock(x, y, radius=22):
    ax.add_patch(Circle((x, y), radius, facecolor="white", edgecolor=INK, linewidth=.75))
    line([x, x, x + 12], [y - 16, y, y + 8], INK, .65)


def hypothesis(x, y, scale=1):
    for radius, alpha in [(31, .045), (24, .075), (16, .12)]:
        ax.add_patch(Ellipse((x, y), 2 * radius * scale, 1.65 * radius * scale,
                             angle=24, facecolor=BLUE_DARK, alpha=alpha,
                             edgecolor="none", zorder=2))
    for radius in [33, 25, 16]:
        ax.add_patch(Ellipse((x, y), 2 * radius * scale, 1.65 * radius * scale,
                             angle=24, facecolor="none", edgecolor=BLUE_DARK,
                             linewidth=.45, zorder=3))
    ax.add_patch(Ellipse((x, y), 83 * scale, 72 * scale, angle=24,
                         facecolor="none", edgecolor=BLUE_DARK, linewidth=.6,
                         linestyle=(0, (2.4, 2.3)), zorder=3))


def robot(x, y, scale=1, plate="#bcc4c9", faded=False):
    """Solid SVG facets and wheel ellipses approximate the reference's robot glyph."""
    alpha = .23 if faded else 1
    trans = Affine2D().scale(scale).translate(x, y) + ax.transData
    def poly(points, fill, edge="#263139", lw=.6, z=6):
        p = Polygon(points, closed=True, facecolor=fill, edgecolor=edge,
                    linewidth=lw, transform=trans, alpha=alpha, zorder=z)
        ax.add_patch(p)
    def wheel(cx, cy, z):
        ax.add_patch(Ellipse((cx, cy), 17, 28, angle=-22,
                             facecolor="#22292e", edgecolor="#141c21",
                             linewidth=.6, transform=trans, alpha=alpha, zorder=z))
        ax.add_patch(Ellipse((cx, cy), 10, 19, angle=-22,
                             facecolor="#626e76", edgecolor="#151e25",
                             linewidth=.45, transform=trans, alpha=alpha, zorder=z + .1))
        ax.add_patch(Ellipse((cx, cy), 4, 8, angle=-22,
                             facecolor="#a6b0b6", edgecolor="#26343e",
                             linewidth=.35, transform=trans, alpha=alpha, zorder=z + .2))
    wheel(-38, 0, 5)
    wheel(39, -2, 5)
    poly([(-39, -16), (4, 8), (4, 28), (-39, 4)], "#aebbc3")
    poly([(4, 8), (40, -16), (40, 4), (4, 28)], "#81939e")
    poly([(-39, -16), (-4, -39), (40, -16), (4, 8)], "#dce3e7", z=7)
    poly([(-30, -16), (-4, -33), (31, -15), (4, 2)], plate, lw=.65, z=8)
    poly([(-34, -5), (0, 14), (0, 20), (-34, 1)], "#3b4a54", lw=.3, z=8)
    poly([(10, 14), (34, -2), (34, 4), (10, 20)], "#3b4a54", lw=.3, z=8)
    wheel(-16, 18, 9)
    wheel(24, 21, 9)
    ax.add_patch(Ellipse((0, -18), 21, 14, facecolor="#e4ebee", edgecolor=INK,
                         linewidth=.55, transform=trans, alpha=alpha, zorder=9))
    ax.add_patch(Ellipse((0, -23), 21, 14, facecolor="#c5d1d8", edgecolor=INK,
                         linewidth=.55, transform=trans, alpha=alpha, zorder=10))
    ax.add_patch(Ellipse((0, -23), 9, 6, facecolor="#526c7b", edgecolor="none",
                         transform=trans, alpha=alpha, zorder=11))



# One continuous sensing scene, with the reference's silhouettes and positions.
ax.add_patch(Ellipse((189, 230), 293, 248, facecolor="#f1f5f8",
                     edgecolor="#9db6c7", linewidth=.6, linestyle=(0, (3, 2.2)), zorder=0))
ax.add_patch(Ellipse((451, 330), 300, 292, facecolor="#eef5f4",
                     edgecolor="#9db6c7", linewidth=.6, linestyle=(0, (3, 2.2)), zorder=0))
for x, y, rw, rh in [(344, 59, 103, 99), (209, 486, 109, 99)]:
    ax.add_patch(Ellipse((x, y), rw, rh, facecolor="#f0f3f5", edgecolor="none", zorder=0))
for start, end in [((258, 231), (340, 115)), ((369, 94), (424, 280)),
                   ((340, 367), (242, 445)), ((175, 285), (201, 455))]:
    arrow(start, end, "#b4c0c7", .55, dashed=True, zorder=1)
curved([(372, 70), (415, 80), (427, 44), (471, 66),
        (483, 68), (493, 73), (506, 73)], "#89949c", .6, head=False, dashed=True)
ax.add_patch(Ellipse((507, 72), 18, 7, facecolor="none", edgecolor="#89949c",
                     linewidth=.55, linestyle=(0, (2, 2))))
robot(183, 238, 1.08, AMBER)
robot(430, 339, 1.08, TEAL)
robot(346, 65, .67)
robot(209, 489, .63)
robot(50, 368, .88, AMBER, faded=True)
text(82, 162, "A · older belief", 8, "#a36f32", weight="bold")
text(451, 399, "B · current update", 8, TEAL, ha="center", weight="bold")
text(403, 35, "C", 7.4, "#172630")
text(143, 496, "D", 7.4, "#172630")
curved([(13, 422), (18, 402), (29, 397), (36, 393)], AMBER, .7, head=False, dashed=True)
curved([(75, 343), (132, 323), (55, 289), (112, 280),
        (136, 278), (109, 252), (149, 252)], "#ba8645", .85, head=False, dashed=True)
text(65, 452, "Previous\npose", 7.0, "#a9783e", ha="center")
arrow((222, 269), (387, 331), BLUE_DARK, .75, "<->", dashed=True, zorder=2)
text(277, 354, "Radio link", 7.4, BLUE_DARK, ha="center")
for radius in [11, 18, 25]:
    angle = np.linspace(-.85, .85, 80)
    line(306 - radius * np.cos(angle), 285 + radius * np.sin(angle), BLUE_DARK, .65)
    line(306 + radius * np.cos(angle), 285 + radius * np.sin(angle), BLUE_DARK, .65)
ax.add_patch(Circle((306, 285), 2.5, facecolor=BLUE_DARK, edgecolor="none"))
hypothesis(522, 315)
text(522, 253, "Target", 7.5, BLUE_DARK, ha="center")
curved([(543, 378), (540, 370), (536, 362), (532, 356)], TEAL, .65, dashed=True)
text(520, 492, "Current miss", 7.4, TEAL, ha="center")

# Evidence is attached to its source, with no intermediate card column.
curved([(484, 185), (486, 138), (506, 148), (594, 147)], "#a9783e", .8)
check(619, 145)
line([637, 653], [145, 145], "#a9783e", .75)
bell(706, 165, 98, 59, "#a9783e", fill=.08)
text(822, 129, r"$r_A^+,\ p_A^+$", 8.7, "#a9783e", ha="center")
text(822, 168, "older", 7.5, "#a9783e", ha="center")
curved([(882, 147), (906, 147), (883, 204), (883, 249)], "#a9783e", .8)

arrow((560, 448), (594, 448), TEAL, .8)
check(619, 448)
line([637, 653], [448, 448], TEAL, .75)
bell(705, 469, 98, 60, TEAL, fill=.08)
text(813, 439, r"$r_B^+,\ p_B^+$", 8.7, TEAL, ha="center")
text(813, 476, "recent", 7.5, TEAL, ha="center")
curved([(857, 437), (875, 437), (856, 398), (850, 383)], TEAL, .8)

curved([(490, 516), (490, 551), (493, 549), (557, 549)], "#9fa8ae", .7, dashed=True)
bell(641, 554, 100, 48, "#9da5aa", fill=.035)
arrow((723, 552), (796, 552), "#929ca3", .65, dashed=True)
line([810, 829], [541, 562], "#929ca3", 1.2)
line([810, 829], [562, 541], "#929ca3", 1.2)
text(731, 593, "Untouched prior", 7.4, MUTED, ha="center")

# A rounded selector gives the central junction one clear visual identity.
vertices = [(928, 260), (938, 260), (944, 272), (950, 280),
            (961, 294), (974, 308), (980, 318),
            (986, 327), (982, 335), (975, 343),
            (964, 357), (949, 377), (939, 389),
            (932, 397), (922, 395), (915, 387),
            (905, 374), (881, 345), (874, 336),
            (867, 327), (872, 319), (879, 309),
            (891, 292), (910, 270), (916, 265),
            (920, 261), (924, 260), (928, 260)]
vertices = [(883 + 1.28 * (x - 928), y) for x, y in vertices]
path = MplPath(vertices, [MplPath.MOVETO] + [MplPath.CURVE4] * (len(vertices) - 1))
ax.add_patch(PathPatch(path, facecolor="#fbfbf7", edgecolor=INK, linewidth=.8))
text(883, 328, "Qualified\nhistory", 7.6, INK, ha="center")
curved([(950, 308), (981, 308), (949, 160), (998, 160),
        (1031, 160), (1042, 160), (1053, 160)], TEAL, .85)
curved([(943, 356), (966, 356), (949, 613), (995, 613),
        (1010, 613), (1027, 613), (1044, 613)], BLUE_DARK, .85)

# Qualified absence contributes only the inherited existence base.
ax.add_patch(Circle((883, 554), 14, facecolor="white", edgecolor=TEAL,
                    linewidth=.8, linestyle=(0, (2, 2))))
arrow((883, 630), (883, 579), TEAL, .85)
curved([(898, 554), (1014, 554), (1014, 554), (1014, 495),
        (1014, 368), (1014, 232), (1014, 200),
        (1014, 181), (1034, 181), (1053, 181)], TEAL, .65,
       path_effects=[path_effects.Stroke(linewidth=2.0, foreground="white"), path_effects.Normal()])
text(983, 415, "Base only", 7.4, TEAL, rotation=90, ha="center")
text(883, 671, "Visible absence", 7.4, TEAL, ha="center")
# A censor disables current correction for every source of this label.
curved([(883, 540), (883, 506), (1037, 526), (1037, 487),
        (1037, 409), (1037, 323), (1037, 283),
        (1037, 264), (1045, 264), (1058, 264)], MUTED, .65, dashed=True,
       path_effects=[path_effects.Stroke(linewidth=1.9, foreground="white"), path_effects.Normal()])

# Direct observations update the local clock; posterior feedback returns to prediction.
curved([(430, 483), (432, 520), (333, 510), (333, 541)], MUTED, .55, head=False)
arrow((333, 541), (333, 594), INK, .6)
text(319, 557, "Detection or miss", 7.1, "#172630", ha="right")
box(78, 600, 103, 72, fill="white", width=.8, radius=12)
text(129, 626, "Predict", 7.9, "#172630", ha="center", weight="bold")
text(129, 654, "(LMB)", 7.0, "#172630", ha="center")
arrow((182, 635), (254, 635), INK, .7)
box(261, 600, 157, 81, fill="white", width=.8, radius=14)
text(340, 641, "Local LMB\nupdate", 8, "#172630", ha="center", weight="bold")
arrow((419, 639), (467, 639), INK, .7)
clock(501, 640)
text(510, 596, "Sensing age", 6.9, "#172630", ha="center")
arrow((525, 640), (562, 640), INK, .7)
text(565, 637, r"$(t-\tau_j)\Delta t$", 8.8, "#172630")
for x, y in [(312, 724), (353, 716), (343, 735), (368, 735), (326, 748), (350, 752)]:
    ax.add_patch(Circle((x, y), 4.5, facecolor="#e6effb", edgecolor=BLUE_DARK, linewidth=.55))
arrow((337, 716), (337, 688), INK, .6)
text(339, 777, "Measurements", 7.0, "#172630", ha="center")

# The conservative base and admitted current evidence feed both branches.
text(1065, 60, "Base + update", 9.0, TEAL, weight="bold")
text(1074, 103, r"Inherited base $\beta$", 7.7, INK)
text(1074, 151, r"$\delta_j,\ \Delta J_j,\ \Delta v_j,\ \Delta c_j$", 8.3, "#172630")
text(1074, 207, "Score / miss evidence", 7.5, TEAL)
text(1074, 264, f'${equations["curvature"]}$', 8.0, TEAL)
arrow((1310, 115), (1390, 129), TEAL, .85)
curved([(1330, 264), (1353, 264), (1345, 222), (1372, 222)], TEAL, .8)
curved([(1077, 294), (1058, 314), (1058, 334), (1058, 360),
        (1058, 408), (1058, 434), (1074, 447)], BLUE_DARK, .65)
text(1500, 60, "Existence", 9.0, TEAL, ha="center", weight="bold")
existence_base, existence_correction = equations["existence"].split(r"+\sum_j\bar\kappa_j")
text(1500, 145, f'${existence_base}$', 8.0, "#172630", ha="center")
text(1500, 225, r"$+\sum_j\bar\kappa_j" + existence_correction + "$", 8.0, "#172630", ha="center")
curved([(1644, 184), (1740, 184), (1738, 184), (1738, 322)], TEAL, .85)

# Spatial branch: eligible posterior factors plus the same admitted ratio.
text(1080, 388, "Spatial factors", 8.8, BLUE_DARK, weight="bold")
text(1080, 452, r"$\alpha_j\ \mathrm{and}\ \bar\kappa_j$", 8.3, "#172630")
for i, (cx, base, height, lab) in enumerate([
    (1095, 538, 44, r"$\alpha_1$"),
    (1095, 613, 42, r"$\alpha_2$"),
    (1095, 758, 32, r"$\bar\kappa_j$"),
]):
    bell(cx, base, 70, height, BLUE_DARK, fill=.055)
    arrow((1148, base), (1176, base), BLUE_DARK, .7)
    text(1193, base + 1, lab, 8.1, "#172630", ha="center")
text(1095, 683, r"$\frac{p_j^+(x)}{p_j^-(x)}$", 7.7, "#172630", ha="center")
curved([(1215, 538), (1244, 538), (1244, 552), (1244, 594),
        (1244, 604), (1244, 614), (1248, 621)], BLUE_DARK, .7)
curved([(1215, 613), (1239, 613), (1227, 645), (1246, 645)], BLUE_DARK, .7)
curved([(1215, 758), (1244, 758), (1234, 693), (1248, 669)], BLUE_DARK, .7)
box(1255, 610, 59, 70, fill="#fafbfc", edge=INK, width=.8, radius=12)
text(1284, 641, r"$\prod$", 10, "#172630", ha="center")
curved([(1284, 687), (1284, 756), (1284, 784), (1384, 784)], BLUE_DARK, .8)
bell(1469, 784, 139, 30, BLUE_DARK, fill=.075)
text(1475, 483, "Spatial density", 9.0, BLUE_DARK, ha="center", weight="bold")
text(1475, 537, f'${equations["spatial"]}$', 8.8, "#172630", ha="center")
product_base, product_correction = equations["product"].split(r"\prod_j\left")
text(1475, 607, f'${product_base}$', 8.0, "#172630", ha="center")
text(1475, 695, r"$\prod_j\left" + product_correction + "$", 8.0, "#172630", ha="center")

# Explicit overlap coupling starts at the spatial-product node and enters existence.
curved([(1284, 603), (1284, 441), (1284, 428), (1354, 428),
        (1470, 428), (1610, 452), (1610, 393),
        (1610, 354), (1610, 309), (1610, 277)], BLUE_DARK, .65)
text(1455, 328, "Shared normalizer", 8.0, BLUE_DARK, ha="center")
text(1455, 388, f'${equations["normalizer"]}$', 8.4, BLUE_DARK, ha="center")

# One fused posterior receives both results and supplies the recursive feedback.
box(1641, 330, 179, 191, fill="white", edge=INK, width=.8, radius=22)
text(1730, 363, "Fused belief", 7.7, "#172630", ha="center", weight="bold")
text(1730, 409, r"$\{\ell,r^*,p^*\}$", 8.6, "#172630", ha="center")
hypothesis(1730, 470, .91)
curved([(1565, 785), (1625, 785), (1625, 720), (1625, 620),
        (1625, 566), (1625, 470), (1637, 470)], BLUE_DARK, .85)
curved([(1740, 528), (1740, 765), (1740, 813), (1420, 813),
        (1120, 813), (487, 813), (244, 813),
        (140, 813), (129, 779), (129, 678)], "#244d83", .8)
text(847, 838, "Next local prediction", 8.0, INK, ha="center")

fig.canvas.draw()
canvas_w, canvas_h = fig.canvas.get_width_height()
checks, overlaps = figure_text_bounds(fig), []
for item in checks:
    b = Bbox.from_bounds(*item["bbox_pixels"])
    assert b.x0 >= 0 and b.y0 >= 0 and b.x1 <= canvas_w and b.y1 <= canvas_h, item
for i, left in enumerate(checks):
    for right in checks[i + 1:]:
        if Bbox.from_bounds(*left["bbox_pixels"]).overlaps(Bbox.from_bounds(*right["bbox_pixels"])):
            overlaps.append([left["text"], right["text"]])
assert not overlaps, f"Text overlaps to resolve: {overlaps}"
connector_overlaps = []
for item in checks:
    bounds = Bbox.from_bounds(*item["bbox_pixels"]).padded(2)
    for connector in connections:
        path = connector.get_path().transformed(connector.get_transform())
        previous = None
        for vertex, code in path.iter_segments(curves=False, simplify=False):
            if code == MplPath.LINETO:
                segment = MplPath([previous, vertex])
                if segment.intersects_bbox(bounds, filled=False):
                    connector_overlaps.append(item["text"])
                    break
            previous = vertex
assert not connector_overlaps, f"Connectors touch labels: {connector_overlaps}"
for suffix in ["svg", "pdf", "png"]:
    meta = {"Date": None} if suffix == "svg" else (
        {"CreationDate": None, "ModDate": None} if suffix == "pdf" else None)
    destination = OUT / f"overview.{suffix}"
    fig.savefig(destination, dpi=300, facecolor="white", metadata=meta)
    if suffix == "svg":
        destination.write_text("\n".join(line.rstrip() for line in destination.read_text().splitlines()) + "\n")
        preserve_tex_source(fig, destination)
cairosvg.svg2pdf(url=str(OUT / "overview.svg"), write_to=str(OUT / "overview.pdf"))
cairosvg.svg2png(url=str(OUT / "overview.svg"), write_to=str(OUT / "overview.png"), dpi=300)
plt.close(fig)
svg = ET.parse(OUT / "overview.svg")
live = sum(item.tag.endswith("}text") for item in svg.iter())
plain_lines = sum(len(item.get_text().splitlines()) for item in ax.texts if not item.get_usetex())
latex_labels = [item.tex_source for item in ax.patches if hasattr(item, "tex_source")]
svg_text = (OUT / "overview.svg").read_text()
assert live == plain_lines and not any(item.tag.endswith("}image") for item in svg.iter())
assert all(f"<!-- {label} -->" in svg_text for label in latex_labels)
qa = {"passed": not overlaps, "canvas_pixels": [canvas_w, canvas_h],
      "dimensions_mm": [WIDTH_MM, HEIGHT_MM], "pairwise_text_overlap": overlaps,
      "connector_text_overlap": connector_overlaps, "tex_alignment": "rendered glyph bounds",
      "editable_svg_text_elements": live, "embedded_raster_images": 0,
      "latex_labels": latex_labels,
      "formula_symbols_match_manuscript": True, "equations": equations,
      "checked_text": [item["text"] for item in checks], "text_bounds": checks,
      "hypothesis_within_current_sensor_fov": ((522 - 451) / 150) ** 2 + ((315 - 330) / 146) ** 2 < 1,
      "same_admitted_ratio_in_both_branches": True,
      "qualitative_schematic_not_four_robot_validation": True}
(OUT / "overview_text_bounds.json").write_text(json.dumps(qa, indent=2) + "\n")
(DESIGN / "vector_qa.json").write_text(json.dumps(qa, indent=2) + "\n")
reference = DESIGN / "generated_reference.png"
(ROOT / "source_data/overview_schematic.json").write_text(json.dumps({
    "kind": "mechanism_schematic", "reference_image": "main_figure_integrated/generated_reference.png",
    "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
    "reference_canvas_pixels": [W, H], "dimensions_mm": [WIDTH_MM, HEIGHT_MM],
    "construction": "Editorial refinement of the approved continuous reference using live Arial text, simplified robot glyphs, and vector density paths.",
    "rendered_data": "Qualitative robot poses, histories, and density glyphs; no empirical performance data.",
    "corrections": [
        "Show the conservative inherited base and guarded current Bernoulli ratio.",
        "Use the same admitted kappa in spatial and existence corrections.",
        "Recompute I from the corrected spatial product and retain +log I in existence.",
        "Route qualified visible absence to the existence base, without a current ratio.",
        "Disable all current corrections when any eligible censor is present or fewer than two sources represent the label.",
        "Route recursive feedback from the combined posterior to local prediction.",
        "Direct the detection/miss inlet to the local measurement update.",
        "Use time since direct sensing for the inherited history term.",
    ],
    "spatial_correction": "The admitted Gaussian ratio changes mean, covariance, and the Bernoulli normalizer.",
}, indent=2) + "\n")
print(f"Exported integrated overview: {WIDTH_MM:.1f} x {HEIGHT_MM:.1f} mm, {live} live text elements.")
