# TAES Figure/Table Audit Ledger

This file is an internal pre-submission QA artifact. It is not a data source and should not be cited as evidence. Its purpose is to keep every figure and table aligned with the manuscript reader path, generated artifacts, and claim boundaries.

## Rules

- No figure/table may introduce a claim that is absent from `CLAIM_EVIDENCE_BOUNDARY_MAP.md`.
- Generated table/figure fragments must be regenerated from scripts or source reports, not edited by hand.
- Every manuscript-affecting figure/table edit requires `./build.sh`, PDF visual QA, readiness checks, and source-bundle freshness checks.
- Contextual GA rows remain context only and must not be described as paired AA-vs-GA significance evidence.
- Full-topology ceiling output is included only after parsing through `evidence_sources.json` and the scenario-family extractor; it must remain a zero-disagreement equivalence boundary, not a gain claim.

## Reference Style Calibration

This ledger uses public, nearby tracking papers only as visual style calibration; it does not cite them as scientific evidence for this manuscript's claims.

| Reference | What was inspected | Takeaway for this manuscript |
| --- | --- | --- |
| `https://arxiv.org/pdf/1902.02523` | A distributed sensor-registration and multi-target tracking manuscript submitted to IEEE TAES, especially algorithm/table pages and trajectory/error figures. | TAES-adjacent tracking figures are restrained: white background, direct axes, thin lines, compact legends, and table-like algorithms. This supports keeping Fig. 1/Fig. 2 technical and uncluttered rather than using decorative illustration. |
| `https://arxiv.org/pdf/1906.00770` | A decentralized multi-target tracking paper with compact scenario/result figures. | Result figures should prioritize direct labels and immediately readable metric structure over ornamental chart styling. |
| `https://c4i.gmu.edu/~pcosta/F15/data/fileserver/file/472054/filename/Paper_1570112369.pdf` | An IEEE aerospace-style distributed MHT/track-fusion paper with architecture diagrams and scenario/result figures. | Architecture figures use explicit data-flow boxes, arrows, and color-coded processing blocks; this supports the current TikZ data-flow treatment in Fig. 1. |

The current figure standard is therefore: vector/TikZ source where possible, white background, restrained blue/gray/gold palette, direct labels, no ornamental gradients, no unsupported visual claims, no page-count regression, and no visual overlap in the rendered PDF.

## Primary Manuscript Figures And Tables

| Artifact | Label | Reader task | Source / generation path | Claim boundary |
| --- | --- | --- | --- | --- |
| Method composite figure | `fig:method` | Show why scalar weights do not establish component correspondence, then show the active-neighborhood input, reference selection, matching, moment projection, existence pass-through, graph-local iteration, and no-global-label contract. | `scripts/render_figures.py`; TikZ fragment `generated/method_pipeline.tex`; `figures/fig_method_pipeline.svg`; visual QA page `tmp/pdf_visual_qa/main_all_p04.png`. | Must remain an output-level projection, not a density-pooling rule, scalar-weight optimizer, recursive LMB update, or global label-dictionary method. |
| Mechanism-isolation protocol | `tab:design` | Let reviewers scan which mechanism is on/off in each arm. | Handwritten `main.tex`; visual QA page `main_all_p06.png`. | Must show fixed spatial-KLA AA, reference-only, and label-barycenter as controlled arms, not an unconstrained tuning comparison. |
| Primary N50 validation table | `tab:n50` | Report the main paired-run means across network disagreement and local tracking metrics. | `scripts/extract_n50_evidence.py`; `generated/n50_mean_rows.tex`; `generated/N50_EVIDENCE_MANIFEST.md`. | Supports primary fixed-parameter tiered packet-loss formation claims only. |
| Contextual GA reference table | `tab:reference` | Place AA projection results beside tracked GA reference rows under matched seeds/profile. | `scripts/extract_reference_baselines.py`; `generated/reference_baseline_rows.tex`; `generated/REFERENCE_BASELINE_MANIFEST.md`. | Contextual comparison only; not paired AA-vs-GA sign-test evidence. |
| Paired-reduction table | `tab:paired` | Separate full label-barycenter effects from reference-only label copying with reductions, wins, CIs, and sign tests. | `scripts/extract_n50_evidence.py`; `generated/n50_paired_rows.tex`; `generated/N50_VERIFICATION_REPORT.md`. | Spatial RMSE/E-OSPA separation supports matched moment barycentering; identical cardinality-only effects support the shared reference-cardinality projection. |
| Paired-reduction bar figure | `fig:n50` | Make the full-versus-reference separation visually inspectable with bar-end percentage labels, all six paired metrics, a full-minus-reference gap column, and a primary/held-out RMSE replication inset. | `scripts/extract_n50_evidence.py`; TikZ fragment `generated/n50_reduction_bars.tex`; visual QA page `main_all_p08.png`. | Visualizes table-driven reductions only; the cardinality rows are shared reference-cardinality controls, while E-OSPA/RMSE separation supports matched moment barycentering. |
| Held-out N50 table | `tab:heldout` | Show that the full-versus-reference RMSE separation persists under base seed 11 without retuning. | `scripts/extract_heldout_sanity_evidence.py`; `generated/heldout_n50_section.tex`; `generated/HELDOUT_N50_MANIFEST.md`. | Robustness check only; not a parameter-selection source. |

## Response-Ready Or Source-Bundle Tables

| Artifact | Label | Reader task | Source / generation path | Boundary |
| --- | --- | --- | --- | --- |
| Harsh packet-loss stress table | `tab:harsh-stress` | Preserve full fixed-parameter harsh-loss details for supplement or response use. | `generated/stress_harsh_section.tex`; `generated/STRESS_HARSH_MANIFEST.md`. | Not in the main PDF by default; use only as response-ready or selected supplement evidence. |
| Scenario-family table | `tab:scenario-family-aa` | Preserve topology-ring, partial-FOV, and full-topology fixed-parameter checks, plus future tier labels. | `generated/scenario_family_section.tex`; `generated/SCENARIO_FAMILY_MANIFEST.md`. | Present full-topology only as an idealized zero-disagreement equivalence boundary; do not treat it as additional method gain or as the deployed graph-local topology setting. |
| Reproducibility ledger table | `tab:ledger` | Map source reports, generated artifacts, roles, hashes, and evidence tiers. | `scripts/render_reproducibility_ledger.py`; `generated/reproducibility_ledger_table.tex`; `generated/REPRODUCIBILITY_LEDGER_MANIFEST.md`. | Source-package/response-ready provenance, not a main-paper result table by default. |

## Final Visual QA Sequence

1. Run `./build.sh`.
2. Inspect `tmp/pdf_visual_qa/main_contact_sheet.png`.
3. Inspect the most affected full-page PNG, usually one of `main_all_p04.png`, `main_all_p07.png`, or `main_all_p08.png`.
4. Confirm `generated/SUBMISSION_READINESS_REPORT.md` keeps `content_status=content_ready_metadata_pending` and has no non-metadata blocking gates.
5. Extract `tmp/submission_bundle/taes_label_barycenter_submission_source.zip` and run `TAES_EVIDENCE_MODE=bundled ./build.sh`.
