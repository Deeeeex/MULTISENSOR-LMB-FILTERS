"""Place TeX outlines using the same ink bounds in SVG, PDF, and PNG."""
from html import escape

import matplotlib as mpl
from matplotlib.patches import PathPatch
from matplotlib.textpath import TextPath
from matplotlib.transforms import Affine2D, ScaledTranslation


def tex_label(ax, x, y, value, size, color, ha="left", va="center"):
    path = TextPath((0, 0), value, size=size, usetex=True)
    bounds = path.get_extents()
    anchor_x = {"left": bounds.x0, "center": (bounds.x0 + bounds.x1) / 2,
                "right": bounds.x1}[ha]
    anchor_y = {"bottom": bounds.y0, "center": (bounds.y0 + bounds.y1) / 2,
                "top": bounds.y1, "baseline": 0}[va]
    # Keep point sizes and anchors valid when an export backend changes the DPI.
    transform = (Affine2D().translate(-anchor_x, -anchor_y).scale(1 / 72)
                 + ax.figure.dpi_scale_trans + ScaledTranslation(x, y, ax.transData))
    item = PathPatch(path, transform=transform, facecolor=color, edgecolor="none",
                     linewidth=0, zorder=10, clip_on=False)
    item.tex_source = value
    item.font_size_pt = size
    item.set_gid(f"tex-label-{len(ax.patches)}")
    ax.add_patch(item)
    return item


def figure_text_bounds(fig):
    renderer = fig.canvas.get_renderer()
    bounds = []
    for item in fig.findobj():
        if isinstance(item, mpl.text.Text) and item.get_visible() and item.get_text():
            value, size = item.get_text(), item.get_fontsize()
        elif hasattr(item, "tex_source"):
            value, size = item.tex_source, item.font_size_pt
        else:
            continue
        bounds.append({"text": value, "font_size_pt": size,
                       "bbox_pixels": list(item.get_window_extent(renderer).bounds)})
    return bounds


def preserve_tex_source(fig, destination):
    svg = destination.read_text()
    for item in fig.findobj():
        if hasattr(item, "tex_source"):
            marker = f'<g id="{item.get_gid()}">'
            assert marker in svg
            svg = svg.replace(marker, marker + f"\n    <!-- {escape(item.tex_source)} -->")
    destination.write_text(svg)
