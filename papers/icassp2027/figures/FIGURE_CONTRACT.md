# Paper figure contract — 2026-09-06

Backend: Python / Matplotlib, explicitly selected by the user. The Lark board
is a separate artifact and is not overwritten by the paper renderer.

## Figure 1: causal repair and its delivery boundary

- Intended conclusion: the main method keeps feasible formation connections,
  repairs a failed branch within the same message budget, and must distinguish
  that planned graph from the packets actually delivered.
- Archetype: three-stage mechanism schematic, double-column width 6.90 inches,
  height 1.95 inches. Every stage uses the same three illustrative formations
  with three sensors each. These are abstract graph positions, not M24/X36
  simulation geometry or a quantitative movement trajectory.
- Panel a: retain tree AB + BC although AC is another feasible alternative.
  Panel b: BC becomes infeasible; retain AB and replace BC by AC. A single
  formation-edge replacement suffices in this example, not in every scene.
  Panel c: one C-to-A gateway packet is lost after the repaired plan is set;
  C then has no delivered path out to the other formations.
- Semantics: all stages keep nine directed local-cycle edges. Each planned
  tree has four directed gateway messages, hence 13 scheduled messages,
  consistent with N + 2(F - 1). The delivered example has 12. Each scheduled
  receiver has at most two nonself inputs. Neither 13 nor this repair example
  is a global optimality or tracking-performance claim.
- Blue local cycles, green retained gateways, thicker orange repaired
  gateways. Dotted grey AC in a is an unused physical alternative; crossed
  BC in b is a failed physical link; a crossed directed arrow in c is one
  undelivered packet. The caption explains these distinct meanings.
- The missing-weight numerical example is shown separately in Figure 2;
  the figure does not center the non-dominating self-fallback ablation.
- Statistical object: none. All graph quantities are illustrative and
  deterministic. No coordinate axes, error bars or performance numbers.

## Figure 2: packet loss, effective weights and Bernoulli fusion

- Intended conclusion: omitting a packet changes the weights of the surviving
  inputs, which changes both spatial pooling and the existence probability.
  Preserving a higher existence probability is not proof of better tracking.
- Archetype: schematic-led analytic pair. Python/matplotlib only, 178 by
  48 mm, editable SVG/PDF and 600-dpi PNG. No new tracker or random samples.
- Panel a uses the implemented planned row `(self,dominant,gateway) =
  (0.25,0.70,0.05)` and a lost dominant packet. Renormalization gives
  `(5/6,0,1/6)`; self fallback gives `(0.95,0,0.05)`. Use stacked bars with
  labeled weights, and a crossed dominant segment in the planned row.
- Panel b is an exact one-dimensional Gaussian Bernoulli example, not a
  runtime arbitrary-GM approximation: surviving inputs both have r=0.8,
  variance 1, and means 0 and 4. With gateway weight a, the spatial mean is
  4a, variance is 1, eta=exp(-8a(1-a)), and fused existence is
  0.8*eta/(0.2+0.8*eta). Plot singleton intensity r*p(x), not only normalized
  p(x), so existence and position effects are both visible.
- Draw both inputs in neutral dashed lines, renormalization in teal, self
  fallback in purple. Annotate the two fused existence probabilities. The
  two alternatives are not ranked by accuracy: no true position is specified.
- `packet_fusion_mechanism_source.csv` stores weights, eta and fused Gaussian
  parameters; `packet_fusion_curves.csv` stores every plotted ordinate.
  The caption defines the analytic assumptions and absence of a truth ranking.

## Figure 3: communication versus complete-set error and its remaining headroom

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
- Check illustrative edge counts/input limits and source row counts once, then visually inspect
  all three figures and the rendered manuscript at final placement. This is a
  producing-agent self-check, not independent scientific validation.
- Stop when the figures are accurate and readable; do not run new filters or
  parameter searches merely to populate a plot.
