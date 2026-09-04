# V276 zipper-merge scene figure QA

## Figure contract

- Core conclusion: the scale-matched zipper maneuver creates one sustained,
  recoverable initial-tree failure aligned with target handoffs in both M24
  and X36.
- Archetype: schematic-led composite with three spatial snapshots and one
  structural timeline.
- Panel map: a--c show exact X36 geometry at steps 1, 80 and 160; d compares
  the M24/X36 fixed-tree feasibility traces and registered handoff times.
- Evidence boundary: generated geometry and physical reachability only; no
  posterior, filter output, tracking score or method outcome is consumed.

## Source and rendering

- Geometry source: `source/zipper_merge_v276_seed1301.json`
- Geometry exporter: `../../exportZipperMergeFigureDataV276.m`
- Exclusive renderer: `../../plot_zipper_merge_v276.py`
- Primary editable output: `zipper_merge_v276_scene.svg`
- Companion outputs: `zipper_merge_v276_scene.pdf` and
  `zipper_merge_v276_scene.png`
- Canvas: 7.45 x 5.10 inches; PNG preview: 2235 x 1530 pixels at 300 dpi.
- Typography: sans-serif with editable SVG text and PDF TrueType embedding.

## Visual checks

- All 36 X36 sensor nodes and 24 target nodes are visible in each snapshot.
- Formation labels, topology edges and the 120-degree FoV boundaries remain
  legible without changing the metre scale on either axis.
- Unavailable initial-tree edges use both red colour and an x marker; the
  distinction does not rely on colour alone.
- M24 and X36 failure bands, handoff markers and the always-connected physical
  graph statement remain readable at the final double-column width.
- No label, legend or plotted element is clipped in the PNG preview.
