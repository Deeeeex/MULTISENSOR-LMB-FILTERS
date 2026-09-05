# V284 paired-result figure contract

The contract was written before the result. The completed 40-step screen
now supports a tradeoff, not the original joint-improvement hypothesis:
prior exclusion reduces E-OSPA, count error and disagreement at nearly
unchanged attempted bytes, but markedly increases matched RMSE.

- Core claim to adjudicate: excluding untouched priors improves target-set
  recovery at acceptable localization, disagreement and payload cost. This is
  a hypothesis, not a conclusion; the completed prefix decides its status.
- Evidence chain: panel a shows all 40 network-mean E-OSPA samples, so an early
  gain cannot hide later deterioration. Panel b compares the whole-prefix
  E-OSPA, count error, common-finite-cell RMSE, representative disagreement,
  and attempted bytes. Every quantity is lower-is-better.
- Archetype: quantitative comparison, one temporal panel and one joint-metric
  summary. No third diagnostic panel or decorative schematic.
- Backend: the established Python/matplotlib workflow only; Octave may export
  numerical CSVs from completed results but must not open a graphics device.
- Output: internal experiment figure, potentially reusable for the ICASSP
  draft only after a substantive result. Fixed 175 x 76 mm canvas, sans-serif
  text, editable PDF/SVG and 600-dpi PNG. No automatic insertion into the paper
  or canonical Lark best-method table.
- Sample: one already-opened X36 seed, 40 steps, 36 sensors; the disagreement
  uses the same six representatives over the entire prefix. These time/node
  samples are correlated, not independent replicates; no confidence interval,
  significance test or held-out validation is implied.
- Integrity: raw unsmoothed E-OSPA means, all steps included; percent changes
  use saved unrounded summaries. Common-cell RMSE aligns finite sensor-time
  cells but not necessarily target identities. Attempted bytes include the
  eight-byte-per-label metadata charge and failed attempts.
- Review risk: this is a fusion-input intervention on unchanged dynamic
  routing, not a routing-versus-static comparison. No full-episode or M24 gain
  may be inferred. Per-formation degradation remains in the report, even if
  aggregate changes look favorable.

The final caption must state the observed tradeoff and the one-seed prefix
boundary, not repeat the hypothesis as a proven result.

Final caption: Untouched-prior exclusion improves X36 prefix set accuracy but
not joint accuracy. (a) Unsmoothed network-mean E-OSPA at all 40 steps.
(b) Whole-prefix changes against the unchanged sparse reference; negative
values are improvements. RMSE compares the same 1,440 finite sensor-time
cells, not necessarily the same matched targets. One opened seed; no across-
seed interval or significance claim. Attempted bytes include metadata and
failed attempts. The joint screen fails on RMSE; the figure is an experiment
record, not a new full-episode paper result.
