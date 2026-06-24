# TAES Figure/Table Audit Ledger

This file is an internal pre-submission QA artifact. It is not a data source and should not be cited as evidence. Its purpose is to keep every figure and table aligned with the manuscript reader path, generated artifacts, and claim boundaries.

## Rules

- No figure/table may introduce a claim that is absent from `CLAIM_EVIDENCE_BOUNDARY_MAP.md`.
- Generated table/figure fragments must be regenerated from scripts or source reports, not edited by hand.
- Every manuscript-affecting figure/table edit requires `./build.sh`, PDF visual QA, readiness checks, and source-bundle freshness checks.
- Contextual GA rows remain context only and must not be described as paired AA-vs-GA significance evidence.
- Full-topology ceiling output remains excluded until a complete report is configured in `evidence_sources.json`, parsed by the scenario-family extractor, and accepted by readiness gates.

## Primary Manuscript Figures And Tables

| Artifact | Label | Reader task | Source / generation path | Claim boundary |
| --- | --- | --- | --- | --- |
| Method mechanism figure | `fig:method` | Show that the operator consumes active neighborhood LMB outputs, performs reference selection, assignment, and barycentering, then rewrites only labels/moments while passing through upstream existence scores. | `scripts/render_figures.py`; `generated/method_pipeline.tex`; `figures/fig_method_pipeline.svg`; visual QA page `tmp/pdf_visual_qa/main_all_p04.png`. | Must remain an output-level projection, not a density-pooling rule, scalar-weight optimizer, recursive LMB update, or global label-dictionary method. |
| Implementation outline | `fig:algorithm` | Give a compact validation-time procedure: input, reference, match, project, existence pass-through, iterate. | Handwritten `main.tex` algorithm box checked by `method algorithm-box markers`. | Must preserve the active-track existence pass-through and no-global-label wording. |
| Mechanism-isolation protocol | `tab:design` | Let reviewers scan which mechanism is on/off in each arm. | Handwritten `main.tex`; visual QA page `main_all_p06.png`. | Must show fixed spatial-KLA AA, reference-only, and label-barycenter as controlled arms, not an unconstrained tuning comparison. |
| Primary N50 validation table | `tab:n50` | Report the main paired-run means across network disagreement and local tracking metrics. | `scripts/extract_n50_evidence.py`; `generated/n50_mean_rows.tex`; `generated/N50_EVIDENCE_MANIFEST.md`. | Supports primary fixed-parameter tiered packet-loss formation claims only. |
| Contextual GA reference table | `tab:reference` | Place AA projection results beside tracked GA reference rows under matched seeds/profile. | `scripts/extract_reference_baselines.py`; `generated/reference_baseline_rows.tex`; `generated/REFERENCE_BASELINE_MANIFEST.md`. | Contextual comparison only; not paired AA-vs-GA sign-test evidence. |
| Paired-reduction table | `tab:paired` | Separate full label-barycenter effects from reference-only label copying with reductions, wins, CIs, and sign tests. | `scripts/extract_n50_evidence.py`; `generated/n50_paired_rows.tex`; `generated/N50_VERIFICATION_REPORT.md`. | Spatial RMSE/E-OSPA separation supports matched moment barycentering; identical cardinality-only effects support the shared reference-cardinality projection. |
| Paired-reduction bar figure | `fig:n50` | Make the full-versus-reference separation visually inspectable with bar-end percentage labels. | `scripts/extract_n50_evidence.py`; `generated/n50_reduction_bars.tex`; visual QA page `main_all_p08.png`. | Visualizes table-driven reductions only; it must not add metrics absent from the generated N50 evidence. |
| Held-out N50 table | `tab:heldout` | Show that the full-versus-reference RMSE separation persists under base seed 11 without retuning. | `scripts/extract_heldout_sanity_evidence.py`; `generated/heldout_n50_section.tex`; `generated/HELDOUT_N50_MANIFEST.md`. | Robustness check only; not a parameter-selection source. |

## Response-Ready Or Source-Bundle Tables

| Artifact | Label | Reader task | Source / generation path | Boundary |
| --- | --- | --- | --- | --- |
| Harsh packet-loss stress table | `tab:harsh-stress` | Preserve full fixed-parameter harsh-loss details for supplement or response use. | `generated/stress_harsh_section.tex`; `generated/STRESS_HARSH_MANIFEST.md`. | Not in the main PDF by default; use only as response-ready or selected supplement evidence. |
| Scenario-family table | `tab:scenario-family-aa` | Preserve topology-ring and partial-FOV fixed-parameter checks, plus future tier labels. | `generated/scenario_family_section.tex`; `generated/SCENARIO_FAMILY_MANIFEST.md`. | Do not add full-topology or smoke-tier results to paper-facing claims before gates pass. |
| Reproducibility ledger table | `tab:ledger` | Map source reports, generated artifacts, roles, hashes, and evidence tiers. | `scripts/render_reproducibility_ledger.py`; `generated/reproducibility_ledger_table.tex`; `generated/REPRODUCIBILITY_LEDGER_MANIFEST.md`. | Source-package/response-ready provenance, not a main-paper result table by default. |

## Final Visual QA Sequence

1. Run `./build.sh`.
2. Inspect `tmp/pdf_visual_qa/main_contact_sheet.png`.
3. Inspect the most affected full-page PNG, usually one of `main_all_p04.png`, `main_all_p07.png`, or `main_all_p08.png`.
4. Confirm `generated/SUBMISSION_READINESS_REPORT.md` keeps `content_status=content_ready_metadata_pending` and has no non-metadata blocking gates.
5. Extract `tmp/submission_bundle/taes_label_barycenter_submission_source.zip` and run `TAES_EVIDENCE_MODE=bundled ./build.sh`.
