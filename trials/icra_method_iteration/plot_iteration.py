"""Exact source-data figure for the complete method iteration; Python only."""
from pathlib import Path
import csv
import hashlib
import json
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

OUT = Path(__file__).resolve().parent
DEST = OUT / "figures"
COLORS = {"conservative_recency": "#7B5AA6", "confirmation_recency": "#147D92"}
SHORT = {"conservative_recency": "CR", "confirmation_recency": "CGR"}
MARKERS = {"conservative_recency": "o", "confirmation_recency": "D"}


def main():
    DEST.mkdir(exist_ok=True)
    data = json.loads((OUT / "report_data.json").read_text())
    baseline_path = OUT.parent / "icra_reunion_fusion/summary_validation.json"
    baseline = json.loads(baseline_path.read_text())
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Arial", "DejaVu Sans"],
                         "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 8,
                         "xtick.labelsize": 6.5, "ytick.labelsize": 6.5, "legend.fontsize": 6.5,
                         "axes.linewidth": .6, "xtick.major.width": .6, "ytick.major.width": .6,
                         "pdf.fonttype": 42, "svg.fonttype": "none", "svg.hashsalt": "icra-method-iteration",
                         "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.facecolor": "white"})
    fig, axes = plt.subplots(2, 2, figsize=(183 / 25.4, 142 / 25.4))
    fig.subplots_adjust(left=.165, right=.98, bottom=.085, top=.86, wspace=.57, hspace=.65)
    points, intervals = [], []
    rows = [(candidate, condition) for condition in ["reliable", "intermittent"] for candidate in SHORT]
    labels = [f"{SHORT[c]} / {'reliable' if r == 'reliable' else 'intermittent'}" for c, r in rows]

    def draw(ax, panel, y, values, stats, color, marker, attrs, offset=0, units=None):
        values = np.asarray(values, float)
        jitter = np.linspace(-.085, .085, len(values)) if len(values) > 1 else [0]
        ax.scatter(values, y + offset + np.asarray(jitter), s=12, facecolors="none", edgecolors=color,
                   marker=marker, linewidths=.55, alpha=.45, zorder=2)
        ax.plot([stats["low"], stats["high"]], [y + offset] * 2, color=color, lw=1.45, zorder=3)
        ax.scatter([stats["mean"]], [y + offset], color=color, marker=marker, s=25, zorder=4)
        assert np.isclose(values.mean(), stats["mean"], atol=1e-10)
        intervals.append(dict(panel=panel, **attrs, n=len(values), **{k: stats[k] for k in ["mean", "low", "high"]}))
        for i, value in enumerate(values):
            points.append(dict(panel=panel, **attrs, unit_index=i, unit_id=units[i], difference=float(value)))

    def setup(ax, panel, title, labels, xlabel):
        ax.set_yticks(range(len(labels)), labels)
        ax.set_ylim(len(labels) - .45, -.55)
        ax.set_xlabel(xlabel)
        ax.axvline(0, color="#777777", lw=.7, ls=(0, (3, 3)), zorder=1)
        ax.grid(axis="x", color="#E7E7E7", lw=.4, zorder=0)
        ax.tick_params(axis="y", length=0, pad=4)
        ax.set_title(title, loc="left", pad=15)
        ax.text(-.37, 1.12, panel, transform=ax.transAxes, fontsize=9, weight="bold")

    for panel, ax, cohort, n in [("a", axes[0, 0], "development", 9), ("b", axes[0, 1], "reserved", 6)]:
        for y, (candidate, condition) in enumerate(rows):
            pair = next(r for r in data["paired"] if r["cohort"] == cohort and r["candidate"] == candidate
                        and r["condition"] == condition and r["reference"] == "lineage")
            group = [r for r in data["real_runs"] if r["cohort"] == cohort and r["condition"] == condition]
            seqs = sorted(set(r["sequence"] for r in group))
            lookup = {(r["sequence"], r["arm"]): r for r in group}
            values = [lookup[s, candidate]["ospa"] - lookup[s, "lineage"]["ospa"] for s in seqs]
            assert len(values) == n
            draw(ax, panel, y, values, pair["ospa"], COLORS[candidate], MARKERS[candidate],
                 dict(cohort=cohort, condition=condition, candidate=candidate, reference="lineage", metric="ospa", scene=""), units=seqs)
        setup(ax, panel, f"{'Development' if cohort == 'development' else 'Reserved'} sequences (n = {n})", labels,
              "OSPA difference vs no-age (m)")
    # The same scale makes cohort differences directly interpretable.
    bounds = [ax.get_xlim() for ax in axes[0]]
    common = (min(b[0] for b in bounds), max(b[1] for b in bounds))
    for ax in axes[0]:
        ax.set_xlim(common)

    ax = axes[1, 0]
    scene_names = {"split_latebirth": "New targets", "churn_departure": "Churn / departure", "split_no_new": "No new targets"}
    case_labels = []
    for scene, name in scene_names.items():
        for key, candidate in [("cr", "conservative_recency"), ("cgr", "confirmation_recency")]:
            y = len(case_labels); case_labels.append(f"{name} / {SHORT[candidate]}")
            own = {r["seed"]: r for r in data["case_runs"][key] if r["scene"] == scene}
            ref = {r["seed"]: r for r in baseline["runs"] if r["scene"] == scene and r["arm"] == "qualified_exist"}
            values = [own[s]["ospa"] - ref[s]["ospa"] for s in sorted(own)]
            pair = next(r for r in data["case_paired"] if r["scene"] == scene and r["candidate"] == candidate
                        and r["reference"] == "qualified_exist")
            draw(ax, "c", y, values, pair["ospa"], COLORS[candidate], MARKERS[candidate],
                 dict(cohort="synthetic", condition="", candidate=candidate, reference="qualified_exist", metric="ospa", scene=scene), units=sorted(own))
    setup(ax, "c", "Mechanism cases (n = 20 seeds each)", case_labels, "OSPA difference vs original ER (m)")

    ax = axes[1, 1]
    for y, (candidate, condition) in enumerate(rows):
        pair = next(r for r in data["paired"] if r["cohort"] == "reserved" and r["candidate"] == candidate
                    and r["condition"] == condition and r["reference"] == "lineage")
        group = [r for r in data["real_runs"] if r["cohort"] == "reserved" and r["condition"] == condition]
        seqs = sorted(set(r["sequence"] for r in group)); lookup = {(r["sequence"], r["arm"]): r for r in group}
        for metric, color, marker, offset in [("false2", "#AF6D17", "s", -.15), ("miss2", "#2A6FA1", "^", .15)]:
            values = [lookup[s, candidate][metric] - lookup[s, "lineage"][metric] for s in seqs]
            draw(ax, "d", y, values, pair[metric], color, marker,
                 dict(cohort="reserved", condition=condition, candidate=candidate, reference="lineage", metric=metric, scene=""), offset, units=seqs)
    setup(ax, "d", "Reserved false / missed costs (n = 6)", labels, "Squared-cost difference vs no-age (m²)")
    handles = [Line2D([], [], color="#AF6D17", marker="s", lw=1, ms=3, label="False cost"),
               Line2D([], [], color="#2A6FA1", marker="^", lw=1, ms=3, label="Missed cost")]
    ax.legend(handles=handles, frameon=False, loc="upper left", bbox_to_anchor=(-.02, 1.04),
              ncol=2, borderaxespad=0, handlelength=1.2, columnspacing=1.1)
    fig.text(.165, .975, "Recency constraints: gains and tradeoffs", fontsize=10, weight="bold", va="top")
    fig.text(.165, .944, "Open markers: complete units   •   Filled markers / lines: mean / descriptive 95% bootstrap interval", fontsize=6.5, va="top")

    for name, records in [("source_points.csv", points), ("source_intervals.csv", intervals)]:
        with (DEST / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, list(records[0]), lineterminator="\n")
            writer.writeheader(); writer.writerows(records)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer(); bounds = []
    # Matplotlib creates extra locator ticks outside the view, but skips
    # drawing them. Audit only text that the exported axes actually draw.
    undrawn_ticks = set()
    for ax in axes.flat:
        for axis, limits in [(ax.xaxis, ax.get_xlim()), (ax.yaxis, ax.get_ylim())]:
            for tick in axis.get_major_ticks() + axis.get_minor_ticks():
                if not min(limits) <= tick.get_loc() <= max(limits):
                    undrawn_ticks.update([tick.label1, tick.label2])
    for artist in fig.findobj(matplotlib.text.Text):
        if artist.get_visible() and artist.get_text() and artist not in undrawn_ticks:
            bb = artist.get_window_extent(renderer)
            bounds.append(dict(text=artist.get_text(), bbox=list(bb.bounds)))
            assert bb.x0 >= -1 and bb.y0 >= -1 and bb.x1 <= fig.bbox.width + 1 and bb.y1 <= fig.bbox.height + 1, artist.get_text()
    for extension in ["svg", "pdf", "png"]:
        fig.savefig(DEST / f"method_iteration.{extension}", dpi=300, metadata={"Creator": "Matplotlib; audited source data"})
    svg = ET.parse(DEST / "method_iteration.svg")
    text_nodes = svg.findall(".//{http://www.w3.org/2000/svg}text")
    assert len(text_nodes) >= 30
    qa = dict(backend="Python/Matplotlib", width_mm=183, height_mm=142, svg_text_nodes=len(text_nodes),
              source_hashes={str(p.relative_to(OUT.parent.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in [OUT / "report_data.json", baseline_path, DEST / "source_points.csv", DEST / "source_intervals.csv"]},
              points=len(points), intervals=len(intervals), individual_unit_mean_checks="passed",
              text_within_canvas="passed", text_bounds=bounds,
              visual_inspection="pending", statistics="Descriptive 95% percentile bootstrap; 10000 resamples; seed 8301; no multiple-comparison correction.")
    (DEST / "qa.json").write_text(json.dumps(qa, indent=2) + "\n")
    plt.close(fig)
    print("FIGURE EXPORTED", len(points), "individual units;", len(intervals), "paired intervals;", len(text_nodes), "editable SVG text nodes.")


if __name__ == "__main__":
    main()
