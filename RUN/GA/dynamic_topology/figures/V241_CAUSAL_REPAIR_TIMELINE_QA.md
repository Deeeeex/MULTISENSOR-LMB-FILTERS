# V241 causal-repair timeline figure QA

## Figure contract

- Core conclusion: causal formation-tree repair changes tracking only after
  fixed-tree failure; it improves E-OSPA and inter-formation consistency,
  while the second repair exposes a late RMSE trade-off.
- Archetype: aligned quantitative grid with one mechanism strip and three
  outcome panels.
- Backend: Python/matplotlib only.
- Final size: 7.2 x 6.0 inches before tight bounding-box export, suitable for a
  two-column figure.
- Panel map: (a) executed messages, (b) E-OSPA, (c) position RMSE,
  (d) inter-formation disagreement.
- Source data: `source/v241_causal_formation_tree_repair_timeline.csv`.
- Exports: editable SVG, PDF, and 360-dpi PNG preview.

## Evidence and statistics boundary

The figure shows one opened M24 development seed. Thin lines are the raw
160-step traces; thick lines are seven-step centered moving averages used only
for readability. P1 covers `t=70..150` and P2 covers `t=151..160`. Percentages
are paired changes in the raw time-step means, with lower values interpreted as
better. There are no error bands, hypothesis tests, confidence intervals, or
generalization claims.

The fixed and causal traces are identical before the first registered repair.
The dashed vertical lines are the two executed causal tree-reselection times,
not outcome-selected annotations.

## Visual QA

- A single gray/blue method mapping is used in every panel.
- Green/red annotations include the explicit words `lower` and `higher`, so
  color is not the sole direction encoding.
- Panel labels, axes, legend, repair markers, raw traces, and smoothed traces
  remain legible in the two-column PNG preview.
- Shared x coordinates and identical repair shading make temporal comparisons
  explicit without duplicate legends.
- SVG text is exported as editable text nodes; the PDF uses TrueType font
  embedding.

## Suggested caption

**Causal topology repair produces tracking changes only after fixed-tree
failure.** (a) The fixed formation tree loses two directed inputs from
`t=70`, whereas causal repair maintains 48 messages per round and reselects
the formation tree only at `t=70` and `t=151`. (b-d) Network-mean position
E-OSPA, position RMSE, and inter-formation disagreement for the matched fixed
and causal arms. Thin curves show raw values and thick curves show seven-step
moving averages. Percentages summarize paired raw-trace changes in P1
(`t=70..150`) and P2 (`t=151..160`); lower is better. The single opened M24
development seed supports mechanism diagnosis, not a validation claim.
