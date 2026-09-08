"""Recreate the image-generated continuous mechanism layout as editable SVG."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, FancyArrowPatch, PathPatch, Polygon
from matplotlib.path import Path as MplPath
from matplotlib.transforms import Affine2D


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "figures"
DESIGN = ROOT / "main_figure_integrated"
OUT.mkdir(exist_ok=True)
W, H = 1828, 860
WIDTH_MM = 181
HEIGHT_MM = WIDTH_MM * H / W
INK, MUTED = "#253746", "#667985"
BLUE, BLUE_DARK = "#287fb5", "#19569d"
TEAL, AMBER = "#008c7b", "#cc8b42"

mpl.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial Narrow", "Arial", "DejaVu Sans"],
    "font.size": 7.2, "mathtext.fontset": "stix", "mathtext.default": "it",
    "svg.fonttype": "none", "svg.hashsalt": "icra-er-integrated-overview",
    "pdf.fonttype": 42, "ps.fonttype": 42, "axes.unicode_minus": False,
})
fig = plt.figure(figsize=(WIDTH_MM / 25.4, HEIGHT_MM / 25.4), dpi=300)
ax = fig.add_axes([0, 0, 1, 1])
ax.set(xlim=(0, W), ylim=(H, 0), aspect="equal")
ax.axis("off")


def text(x, y, value, size=7.2, color=INK, **kwargs):
    return ax.text(x, y, value, fontsize=size, color=color, va="center",
                   linespacing=1.14, **kwargs)


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
    item = FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=6.5,
                           color=color, linewidth=width, shrinkA=0, shrinkB=0,
                           linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    ax.add_patch(item)
    return item


def curved(vertices, color=INK, width=.8, head=True, dashed=False, **kwargs):
    path = MplPath(vertices, [MplPath.MOVETO] + [MplPath.CURVE4] * (len(vertices) - 1))
    if head:
        item = FancyArrowPatch(path=path, arrowstyle="-|>", mutation_scale=6.5,
                               color=color, linewidth=width, capstyle="round",
                               linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    else:
        item = PathPatch(path, facecolor="none", edgecolor=color, linewidth=width,
                         capstyle="round", joinstyle="round",
                         linestyle=(0, (2.4, 2.3)) if dashed else "-", **kwargs)
    ax.add_patch(item)
    return item


def bell(cx, baseline, width=95, height=52, color=BLUE_DARK, axis=True, fill=.09):
    u = np.linspace(-3.2, 3.2, 101)
    xs = cx + u * width / 6.4
    ys = baseline - height * np.exp(-.5 * (u / .85) ** 2)
    ax.fill_between(xs, baseline, ys, color=color, alpha=fill, linewidth=0)
    line(xs, ys, color, .8)
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
    for cx, cy in [(-26, -19), (20, -20), (1, -32)]:
        item = line([cx, cx], [cy, cy - 15], "#25343d", .55, alpha=alpha, zorder=9)
        item.set_transform(trans)
        ax.add_patch(Circle((cx, cy - 15), 2.1, facecolor="#52616b",
                            edgecolor="#26343e", linewidth=.3, transform=trans,
                            alpha=alpha, zorder=10))


# One continuous sensing scene, with the reference's silhouettes and positions.
ax.add_patch(Ellipse((189, 230), 293, 248, facecolor="#eaf2fb",
                     edgecolor="#76a4cf", linewidth=.6, linestyle=(0, (3, 2.2)), zorder=0))
ax.add_patch(Ellipse((451, 330), 300, 292, facecolor="#edf5fc",
                     edgecolor="#76a4cf", linewidth=.6, linestyle=(0, (3, 2.2)), zorder=0))
for x, y, rw, rh in [(344, 59, 103, 99), (209, 486, 109, 99)]:
    ax.add_patch(Ellipse((x, y), rw, rh, facecolor="#e4f1fb", edgecolor="none", zorder=0))
for start, end in [((211, 198), (317, 83)), ((369, 94), (424, 280)),
                   ((383, 372), (239, 464)), ((196, 279), (242, 450))]:
    arrow(start, end, "#909aa1", .55, dashed=True, zorder=1)
curved([(372, 70), (415, 80), (427, 44), (471, 66),
        (483, 68), (493, 73), (506, 73)], "#89949c", .6, head=False, dashed=True)
ax.add_patch(Ellipse((507, 72), 18, 7, facecolor="none", edgecolor="#89949c",
                     linewidth=.55, linestyle=(0, (2, 2))))
robot(183, 238, 1.08, AMBER)
robot(430, 339, 1.08, TEAL)
robot(346, 65, .67)
robot(209, 489, .63)
robot(50, 368, .88, AMBER, faded=True)
text(82, 179, "A · older belief", 8, "#ac6821", weight="bold")
text(353, 405, "B · recent miss", 8, "#007d70", weight="bold")
text(403, 35, "C", 7.4, "#172630")
text(143, 496, "D", 7.4, "#172630")
curved([(13, 422), (18, 402), (29, 397), (36, 393)], AMBER, .7, head=False, dashed=True)
curved([(75, 343), (132, 323), (55, 289), (112, 280),
        (136, 278), (109, 252), (149, 252)], "#be772b", .85, head=False, dashed=True)
text(47, 435, "Previous\npose", 7.0, "#b5732b", ha="center")
arrow((222, 269), (387, 331), BLUE_DARK, .75, "<->", dashed=True, zorder=2)
text(299, 326, "A–B radio link", 7.0, BLUE_DARK, ha="center")
for radius in [11, 18, 25]:
    angle = np.linspace(-.85, .85, 80)
    line(306 - radius * np.cos(angle), 285 + radius * np.sin(angle), BLUE_DARK, .65)
    line(306 + radius * np.cos(angle), 285 + radius * np.sin(angle), BLUE_DARK, .65)
ax.add_patch(Circle((306, 285), 2.5, facecolor=BLUE_DARK, edgecolor="none"))
hypothesis(504, 274)
text(553, 324, "Target\nhypothesis", 7.0, BLUE_DARK, ha="center")
curved([(550, 405), (542, 382), (524, 350), (512, 316)], TEAL, .65, dashed=True)
text(521, 458, "Missed\ndetection", 6.9, TEAL, ha="center")

# Evidence is attached to its source, with no intermediate card column.
curved([(484, 185), (486, 138), (506, 148), (594, 147)], "#b7742d", .8)
check(619, 145)
line([637, 653], [145, 145], "#b7742d", .75)
bell(706, 165, 98, 59, "#b7742d", fill=.08)
text(771, 184, r"$x$", 7.0)
text(822, 129, r"$r_A,\ p_A$", 8.7, "#b7742d", ha="center")
text(822, 168, "older", 7.5, "#b7742d", ha="center")
curved([(882, 147), (933, 147), (927, 147), (927, 249)], "#b7742d", .8)

arrow((557, 417), (594, 417), TEAL, .8)
check(619, 417)
line([637, 653], [417, 417], TEAL, .75)
bell(705, 445, 98, 60, TEAL, fill=.08)
text(771, 465, r"$x$", 7.0)
text(822, 405, r"$r_B,\ p_B$", 8.7, TEAL, ha="center")
text(822, 447, "recent", 7.5, TEAL, ha="center")
curved([(883, 421), (894, 421), (890, 378), (887, 368)], TEAL, .8)

curved([(450, 487), (449, 553), (445, 549), (557, 549)], "#9fa8ae", .7, dashed=True)
bell(641, 554, 100, 48, "#9da5aa", fill=.035)
text(705, 574, r"$x$", 7.0, "#78858e")
arrow((723, 552), (796, 552), "#929ca3", .65, dashed=True)
line([810, 829], [541, 562], "#929ca3", 1.2)
line([810, 829], [562, 541], "#929ca3", 1.2)
text(639, 603, "Untouched prior", 7.2, "#858f96", ha="center")

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
vertices = [(928 + 1.28 * (x - 928), y) for x, y in vertices]
path = MplPath(vertices, [MplPath.MOVETO] + [MplPath.CURVE4] * (len(vertices) - 1))
ax.add_patch(PathPatch(path, facecolor="#fbfbf7", edgecolor=INK, linewidth=.8))
text(928, 328, "History\nqualification", 7.0, "#172630", ha="center")
curved([(995, 308), (1010, 308), (986, 160), (1018, 160),
        (1031, 160), (1042, 160), (1053, 160)], TEAL, .85)
curved([(988, 356), (1009, 356), (986, 626), (1019, 626),
        (1031, 626), (1036, 626), (1044, 626)], BLUE_DARK, .85)

# Qualified absence bypasses spatial pooling and keeps factor f_j=1.
ax.add_patch(Circle((928, 516), 14, facecolor="white", edgecolor=TEAL,
                    linewidth=.8, linestyle=(0, (2, 2))))
arrow((928, 600), (928, 543), TEAL, .85)
curved([(943, 516), (1018, 516), (1018, 516), (1018, 470),
        (1018, 357), (1018, 232), (1018, 196),
        (1018, 179), (1034, 179), (1053, 179)], TEAL, .6)
text(1029, 399, r"$f_j=1$", 7.1, TEAL, rotation=90)
text(928, 643, "Visible absence", 7.3, TEAL, ha="center")
text(928, 681, "Existence only", 7.0, TEAL, ha="center")

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
text(501, 606, "Local clock", 6.9, "#172630", ha="center")
arrow((525, 640), (562, 640), INK, .7)
text(578, 637, r"$\Delta_j$", 9.5, "#172630")
text(563, 695, "Time since\ndirect sensing", 7.1, "#172630")
for x, y in [(312, 724), (353, 716), (343, 735), (368, 735), (326, 748), (350, 752)]:
    ax.add_patch(Circle((x, y), 4.5, facecolor="#e6effb", edgecolor=BLUE_DARK, linewidth=.55))
arrow((337, 716), (337, 688), INK, .6)
text(339, 777, "Measurements", 7.0, "#172630", ha="center")

# Existence branch: recency changes weights, with the method's nonzero floor.
text(1086, 52, "Recency weights", 8.6, TEAL, weight="bold")
text(1098, 91, r"$q_j\propto w_j\,f(\Delta_j)$", 9.0, "#172630")
arrow((1102, 242), (1293, 242), INK, .5)
arrow((1102, 242), (1102, 116), INK, .5)
ages = np.linspace(0, 5, 160)
factor = .25 + .75 * np.exp(-ages)
line(1102 + ages / 5 * 179, 242 - factor * 99, TEAL, .85)
line([1102, 1282], [217.25, 217.25], "#7f9697", .55, ls=(0, (2.5, 2)))
text(1070, 117, r"$f$", 7.5, "#172630", ha="right")
text(1088, 150, "1", 6.9, "#172630", ha="right")
text(1088, 216, r"$\rho$", 8.0, "#172630", ha="right")
text(1102, 262, "0", 6.9, "#172630", ha="center")
text(1310, 244, r"$\Delta_j$", 8.0, "#172630")
text(1101, 298, r"Decays with age, floor $\rho>0$", 7.1, TEAL)
arrow((1305, 160), (1370, 160), TEAL, .85)
text(1493, 109, "Existence fusion", 8.6, TEAL, ha="center", weight="bold")
text(1494, 158, r"$\mathrm{logit}\,r^*=\sum_jq_j\,\mathrm{logit}\,r_j$", 8.3, "#172630", ha="center")
text(1510, 213, r"$+\log\eta_a$", 8.3, "#172630", ha="center")
curved([(1649, 161), (1740, 161), (1738, 157), (1738, 304)], TEAL, .85)

# Spatial branch: weighted Gaussian product with ordinary eligible weights.
text(1062, 446, "Spatial weights", 8.6, BLUE_DARK, weight="bold")
text(1062, 486, r"$a_j\propto w_j$", 8.8, "#172630")
for i, (cx, base, height, lab) in enumerate([
    (1095, 557, 44, r"$a_1$"),
    (1093, 623, 42, r"$a_2$"),
    (1090, 726, 35, r"$a_J$"),
]):
    bell(cx, base, 70, height, BLUE_DARK, fill=.055)
    text(cx + 49, base + 17, r"$x$", 7.0, "#172630")
    arrow((1148, base), (1176, base), BLUE_DARK, .7)
    text(1193, base + 1, lab, 8.1, "#172630", ha="center")
text(1087, 668, r"$\vdots$", 10.5, "#172630", ha="center")
curved([(1215, 558), (1250, 558), (1246, 558), (1246, 614)], BLUE_DARK, .7)
arrow((1215, 623), (1246, 623), BLUE_DARK, .7)
curved([(1215, 726), (1250, 726), (1227, 663), (1247, 663)], BLUE_DARK, .7)
box(1255, 610, 59, 70, fill="#fafbfc", edge=INK, width=.8, radius=12)
text(1284, 641, r"$\prod$", 16, "#172630", ha="center")
arrow((1315, 645), (1384, 645), BLUE_DARK, .8)
bell(1460, 671, 139, 66, BLUE_DARK, fill=.075)
text(1547, 692, r"$x$", 7.0, "#172630")
text(1459, 501, "Spatial fusion", 8.6, BLUE_DARK, ha="center", weight="bold")
text(1459, 550, r"$p^*(x)=\eta_a^{-1}\prod_jp_j(x)^{a_j}$", 8.6, "#172630", ha="center")

# Explicit overlap coupling starts at the spatial-product node and enters existence.
curved([(1284, 603), (1284, 467), (1290, 464), (1364, 464),
        (1465, 464), (1505, 481), (1505, 422),
        (1505, 354), (1505, 284), (1505, 240)], BLUE_DARK, .65)
text(1487, 357, r"Spatial overlap $\eta_a$", 7.6, BLUE_DARK, ha="right")

# One fused posterior receives both results and supplies the recursive feedback.
box(1641, 312, 179, 191, fill="white", edge=INK, width=.8, radius=22)
text(1730, 342, "Fused Bernoulli", 7.0, "#172630", ha="center", weight="bold")
text(1730, 384, r"$\{\ell,r^*,p^*\}$", 10, "#172630", ha="center")
hypothesis(1730, 450, .91)
curved([(1565, 670), (1625, 670), (1625, 670), (1625, 620),
        (1625, 548), (1625, 450), (1637, 450)], BLUE_DARK, .85)
curved([(1740, 510), (1740, 765), (1740, 813), (1420, 813),
        (1120, 813), (487, 813), (244, 813),
        (140, 813), (129, 779), (129, 678)], "#244d83", .8)
text(847, 838, "Next local step", 8.1, "#244d83", ha="center", weight="bold")

fig.canvas.draw()
renderer = fig.canvas.get_renderer()
canvas_w, canvas_h = fig.canvas.get_width_height()
checks, overlaps = [], []
for item in ax.texts:
    b = item.get_window_extent(renderer)
    assert b.x0 >= 0 and b.y0 >= 0 and b.x1 <= canvas_w and b.y1 <= canvas_h, (item.get_text(), b)
    checks.append({"text": item.get_text(), "font_size_pt": item.get_fontsize(),
                   "bbox_pixels": list(b.bounds)})
for i, left in enumerate(ax.texts):
    for right in ax.texts[i + 1:]:
        if left.get_window_extent(renderer).overlaps(right.get_window_extent(renderer)):
            overlaps.append([left.get_text(), right.get_text()])
assert not overlaps, f"Text overlaps to resolve: {overlaps}"
for suffix in ["svg", "pdf", "png"]:
    meta = {"Date": None} if suffix == "svg" else (
        {"CreationDate": None, "ModDate": None} if suffix == "pdf" else None)
    destination = OUT / f"overview.{suffix}"
    fig.savefig(destination, dpi=300, facecolor="white", metadata=meta)
    if suffix == "svg":
        destination.write_text("\n".join(line.rstrip() for line in destination.read_text().splitlines()) + "\n")
plt.close(fig)
svg = ET.parse(OUT / "overview.svg")
live = sum(item.tag.endswith("}text") for item in svg.iter())
assert live >= 45 and not any(item.tag.endswith("}image") for item in svg.iter())
qa = {"passed": not overlaps, "canvas_pixels": [canvas_w, canvas_h],
      "dimensions_mm": [WIDTH_MM, HEIGHT_MM], "pairwise_text_overlap": overlaps,
      "editable_svg_text_elements": live, "embedded_raster_images": 0,
      "checked_text": [item["text"] for item in checks], "text_bounds": checks,
      "hypothesis_within_current_sensor_fov": ((504 - 451) / 150) ** 2 + ((274 - 330) / 146) ** 2 < 1,
      "age_weight_floor": .25}
(OUT / "overview_text_bounds.json").write_text(json.dumps(qa, indent=2) + "\n")
(DESIGN / "vector_qa.json").write_text(json.dumps(qa, indent=2) + "\n")
reference = DESIGN / "generated_reference.png"
(ROOT / "source_data/overview_schematic.json").write_text(json.dumps({
    "kind": "mechanism_schematic", "reference_image": "main_figure_integrated/generated_reference.png",
    "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
    "reference_canvas_pixels": [W, H], "dimensions_mm": [WIDTH_MM, HEIGHT_MM],
    "construction": "Editable vector paths and text, manually recreated from the image-generation reference.",
    "rendered_data": "Qualitative robot poses, histories, and density glyphs; no empirical performance data.",
    "corrections": [
        "Restore +log eta_a in the existence equation.",
        "Connect the overlap arrow to the spatial-product node.",
        "Route qualified visible absence to existence weighting only, with factor f_j=1.",
        "Route recursive feedback from the combined posterior to local prediction.",
        "Direct the detection/miss inlet to the local measurement update.",
        "Use f on the age-curve vertical axis and keep floor rho=0.25.",
    ],
    "spatial_invariance_conditions": "Fixed eligible input densities and ordinary spatial weights.",
}, indent=2) + "\n")
print(f"Exported integrated overview: {WIDTH_MM:.1f} x {HEIGHT_MM:.1f} mm, {live} live text elements.")
