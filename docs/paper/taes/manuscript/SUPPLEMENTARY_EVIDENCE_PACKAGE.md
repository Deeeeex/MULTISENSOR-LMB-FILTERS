# TAES Supplementary / Response Evidence Package

This file is a single handoff point for optional supplementary material or reviewer-response evidence. It does not change the main-paper claims. It organizes generated, report-driven artifacts that are already checked by the manuscript build and readiness pipeline.

## Purpose and Scope

- Candidate Supplementary Material: robustness and provenance tables that may be moved into a supplement if the final TAES page budget or reviewer expectations require them.
- Response-ready evidence: compact tables and manifests that can support reviewer responses without editing generated fragments by hand.
- Supplement README: `SUPPLEMENTARY_README_DRAFT.md` provides the portal-facing readme scaffold required if any selected blocks are converted into formal supplementary material.
- Claim discipline: `CLAIM_EVIDENCE_BOUNDARY_MAP.md` maps paper-facing claims to evidence artifacts, verification level, terminology decisions, and explicit non-claims before those claims enter a supplement or response.
- Boundary control: stress, topology, FOV, and full-neighborhood ceiling checks are fixed-parameter evidence, not a tuning loop, and they do not substitute for target-maneuver, covariance-consistency, or recursive-online validation.

## Current Submission Decision

The current content-ready package default is no separate supplementary upload. The held-out N50 robustness table is already imported in the main manuscript; the harsh packet-loss, scenario-family, and reproducibility-ledger blocks remain response-ready and source-bundle-ready evidence unless the final page budget, TAES portal form, editor, or reviewers require a formal supplement.

If a formal supplement is selected, the eligible block set is limited to:

- `generated/stress_harsh_section.tex`
- `generated/scenario_family_section.tex`
- `generated/reproducibility_ledger_table.tex`
- `generated/heldout_n50_section.tex`, only if the held-out table is moved out of the main manuscript

The default supplement must not include raw `RUN/` logs, stale scratch validation reports, internal QA files, or generated manifests unless those files are explicitly converted into a reviewable supplement format. The completed full-topology N50 result is included only through the parsed scenario-family table and must be described as a zero-disagreement equivalence boundary, not a gain claim.

## Candidate Supplementary Material

| Evidence block | Generated source | Intended use | Current status |
| --- | --- | --- | --- |
| Held-out N50 robustness table | `generated/heldout_n50_section.tex` | Support the claim that the full-versus-reference separation is not a base-seed-1 artifact. | Imported in the main manuscript; keep available for supplement or response. |
| Harsh packet-loss N50 stress table | `generated/stress_harsh_section.tex` | Show fixed-design performance under a harsher packet-loss profile. | Response-ready; the main paper imports only the concise generated Discussion sentence. |
| Scenario-family table | `generated/scenario_family_section.tex` | Show fixed-design topology-ring, partial-FOV, and full-topology equivalence-boundary N50 checks. | Response-ready; the main paper imports only the concise generated Discussion sentence. |
| Reproducibility ledger | `generated/reproducibility_ledger_table.tex` | Explain which reports, seeds, roles, and verifier artifacts support each paper-facing result. | Source-package and response-ready provenance material. |
| Claim-evidence-boundary map | `CLAIM_EVIDENCE_BOUNDARY_MAP.md` | Keep reviewer responses and optional supplement text aligned with verified claims and non-claims. | Internal QA artifact; not a data source and not a default portal upload. |
| Supplement README draft | `SUPPLEMENTARY_README_DRAFT.md` | Provide the readme scaffold for selected supplementary files. | Use only if a formal supplement is submitted. |

## Interpretation Boundaries

- The generated fragments should not be edited directly. Update the source report, extractor, verifier, or `evidence_sources.json`, then run `./build.sh`.
- The held-out, harsh-loss, topology-ring, partial-FOV, and full-topology checks reuse fixed method parameters after method selection. They are not a tuning loop and should not trigger per-scenario threshold, barycenter-weight, or label-rule search.
- The contextual GA rows are reference rows from a tracked GA validation path. They are not paired AA-vs-GA significance-test evidence.
- Scenario-family evidence broadens packet-loss severity, sparse-topology coverage, partial-field-of-view sensing, and an idealized full-neighborhood ceiling. The full-topology row shows zero network disagreement and identical local metrics across arms, so it is an equivalence boundary rather than evidence of additional method gain.
- If the final submission includes a supplement, convert only the selected generated fragments into the portal-compliant supplement format after final metadata replacement and one last `./build.sh` run.
- Do not restate the full-topology result in supplement, response, cover-letter, or manuscript text as a deployed graph-local topology claim; it is only an idealized full-neighborhood ceiling parsed through the same scenario-family evidence gate as topology-ring and partial-FOV evidence.

## File Map

| File | Role |
| --- | --- |
| `generated/HELDOUT_N50_MANIFEST.md` | Protocol and source manifest for the held-out N50 check. |
| `generated/STRESS_HARSH_MANIFEST.md` | Protocol and source manifest for the harsh packet-loss N50 check. |
| `generated/SCENARIO_FAMILY_MANIFEST.md` | Protocol, tier labels, source hashes, and interpretation classes for topology/FOV/full-topology checks. |
| `generated/REPRODUCIBILITY_LEDGER_MANIFEST.md` | Source and SHA-256 ledger for manuscript-facing evidence roles. |
| `generated/N50_VERIFICATION_REPORT.md` | Independent recomputation of network disagreement, runtime, and local metrics. |
| `generated/SUBMISSION_READINESS_REPORT.md` | Current machine-readable submission-readiness summary in human-readable form. |
| `CLAIM_EVIDENCE_BOUNDARY_MAP.md` | Claim, evidence, boundary, terminology, and non-claim map for final paper-readiness review. |
| `SUPPLEMENTARY_README_DRAFT.md` | Draft readme for formal supplementary material upload, if selected. |
