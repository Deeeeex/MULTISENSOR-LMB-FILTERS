# Method-iteration figure contract

Core conclusion: confirmation gating retains a modest intermittent-link
gain over no-age on development and reserved data, while stricter recency
caps expose a false/missed-target tradeoff and both variants lose some
synthetic-case performance relative to original ER.

Evidence chain and quantitative-grid archetype:

- a: CR/CGR minus no-age OSPA on all nine development sequences, both links.
- b: the same comparison on all six nonduplicate reserved sequences.
- c: CR/CGR minus original ER OSPA on all 20 paired seeds in each of the
  three unchanged simulation families. Show adverse positive differences.
- d: CR/CGR minus no-age false and missed GOSPA squared costs on the six
  reserved sequences, both links, to explain the direction of the tradeoff.

Use the established Python/Matplotlib workflow. Double-column-sized figure,
183 by 142 mm, 7 pt consistent sans-serif text, no decorative diagram or
image generation. Vector PDF and SVG with editable text; 300 dpi PNG for
review. Direct row labels and distinct marker shapes supplement color.
Every point is an actual complete sequence or seed; no generated data,
frame-level pseudo-replication, significance stars or selective exclusions.

All comparison signs are candidate minus named reference. Negative is a
lower cost. Show individual units, mean and descriptive 95% percentile
bootstrap interval with 10,000 resamples and seed 8301. No hypothesis test or
multiple-comparison adjustment. Routes may be related, and the reserved
data come from the original detector's training split. The overlap control
is excluded from these main panels according to the pre-tracking protocol
and is retained in the report tables.

Source data: `report_data.json`, `report_real_runs.csv`, exported
`figures/source_points.csv` and `figures/source_intervals.csv`. Record their
hashes, vector/raster exports and final-size checks in `figures/qa.json`.
Inspect the exact Matplotlib-rendered preview for clipping/overlap; inspect
exported vector text for editability. This is a research iteration figure,
not a claim that current conference submission requirements were audited.
