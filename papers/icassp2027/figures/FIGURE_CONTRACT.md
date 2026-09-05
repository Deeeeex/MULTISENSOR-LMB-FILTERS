# Paper figure contract — 2026-09-06

Backend: Python / Matplotlib, explicitly selected by the user. The Lark board
is a separate artifact and is not overwritten by the paper renderer.

## Figure 1: method, not a performance claim

- Intended conclusion: the scheduled sparse backbone and the inputs actually
  fused are different objects. A lost dominant packet changes surviving
  weights differently under renormalization and self fallback.
- Archetype: two-panel mechanism schematic, single-column width (3.46 inches),
  height 2.65 inches. Panel a uses three illustrative formations with three
  sensors each; it is not a scaled M24/X36 scene. Panel b shows exact weights.
- Semantics: nine directed local-cycle edges plus four directed gateways;
  each tree connection has both directions. The example contains 13 scheduled
  messages, consistent with N + 2(F - 1), not a claimed global minimum.
- Weight example: planned self / dominant / residual = 0.25 / 0.70 / 0.05;
  after the dominant packet is unavailable, renormalization = 5/6 / 0 / 1/6,
  self fallback = 0.95 / 0 / 0.05. The fallback is an ablation, not the winner.
- Statistical object: none. All schematic quantities are illustrative and
  deterministic. No graph-layout coordinate axes are needed.

## Figure 2: communication versus complete-set error and its remaining headroom

- Intended conclusion: sparse repair reduces posterior traffic, while its
  whole-set improvement is modest, particularly on X36. Full repair and
  self fallback show different cost/error tradeoffs. A third panel explains
  why localization alone cannot close the target-set recovery gap.
- Archetype: quantitative grid, two primary scatter panels (M24 and X36)
  plus a compact diagnostic panel, at 6.90 by 2.18 inches total.
  Horizontal axis: attempted posterior payload in decimal MB. Vertical axis:
  mean E-OSPA in metres. Lower left is better. Scales differ between panels
  and remain explicitly ticked; cross-panel distances are not effect sizes.
- Statistical object: one paired 160-step development episode, seed 1301 per
  scale. Dots are episode summaries, not independent sensor-time replicates.
  No confidence intervals, significance stars, smoothing or fitted curves.
- Sources: the saved V248/V274 reference result MAT files used by V279, plus
  the completed V278 MAT for the X36 receiver ablation. Export unrounded
  values through `exportPaperFigureData.m`; do not transcribe plotted values
  from a rounded manuscript table.
- Include fixed, full-repair and sparse-repair arms on both scales; include
  self fallback only where executed (X36). Other metrics stay in Table 1.
- Panel c reads only the sparse rows of `count_budget_source.csv`. Bars are
  upper bounds on the decrease in mean E-OSPA with the output cardinalities
  held fixed: mean(E - sqrt(C_min)). They are not measured improvements,
  confidence intervals, a decomposition of mean OSPA, or evidence of correct
  target identities. Count-sign ambiguity makes the X36 value conservative.
  Explicit upper-bound labels distinguish this panel from achieved results.

## Exports and acceptance

- Commit the source exporter, plotting script, source CSVs, editable-text SVG,
  vector PDF, and 600-dpi PNG. Matplotlib SVG fonttype is none, PDF fonttype 42.
- Use consistent policy colors plus distinct marker shapes and direct labels;
  thin neutral axes, no decorative gradients or statistical embellishments.
- Export a machine-readable summary of units, sample size, sources and caveats.
- Check weight arithmetic and source row counts once, then visually inspect
  both figures and the rendered manuscript at final placement. This is a
  producing-agent self-check, not independent scientific validation.
- Stop when the figures are accurate and readable; do not run new filters or
  parameter searches merely to populate a plot.
