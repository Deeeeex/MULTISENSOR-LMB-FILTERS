# TAES Supplementary / Response Evidence Package

This file is a single handoff point for optional supplementary material or reviewer-response evidence. It does not change the main-paper claims. It organizes generated, report-driven artifacts that are already checked by the manuscript build and readiness pipeline.

## Purpose and Scope

- Candidate Supplementary Material: robustness and provenance tables that may be moved into a supplement if the final TAES page budget or reviewer expectations require them.
- Response-ready evidence: compact tables and manifests that can support reviewer responses without editing generated fragments by hand.
- Boundary control: stress, topology, and FOV checks are fixed-parameter evidence, not a tuning loop, and they do not substitute for target-maneuver, covariance-consistency, or recursive-online validation.

## Candidate Supplementary Material

| Evidence block | Generated source | Intended use | Current status |
| --- | --- | --- | --- |
| Held-out N50 robustness table | `generated/heldout_n50_section.tex` | Support the claim that the full-versus-reference separation is not a base-seed-1 artifact. | Imported in the main manuscript; keep available for supplement or response. |
| Harsh packet-loss N50 stress table | `generated/stress_harsh_section.tex` | Show fixed-design performance under a harsher packet-loss profile. | Response-ready; the main paper imports only the concise generated Discussion sentence. |
| Topology/FOV scenario-family table | `generated/scenario_family_section.tex` | Show fixed-design topology-ring and partial-FOV N50 boundary checks. | Response-ready; the main paper imports only the concise generated Discussion sentence. |
| Reproducibility ledger | `generated/reproducibility_ledger_table.tex` | Explain which reports, seeds, roles, and verifier artifacts support each paper-facing result. | Source-package and response-ready provenance material. |

## Interpretation Boundaries

- The generated fragments should not be edited directly. Update the source report, extractor, verifier, or `evidence_sources.json`, then run `./build.sh`.
- The held-out, harsh-loss, topology-ring, and partial-FOV checks reuse fixed method parameters after method selection. They are not a tuning loop and should not trigger per-scenario threshold, barycenter-weight, or label-rule search.
- The contextual GA rows are reference rows from a tracked GA validation path. They are not paired AA-vs-GA significance-test evidence.
- Scenario-family evidence broadens packet-loss severity, sparse-topology coverage, and partial-field-of-view sensing. It still preserves the formation-family assumptions stated in the main Discussion.
- If the final submission includes a supplement, convert only the selected generated fragments into the portal-compliant supplement format after final metadata replacement and one last `./build.sh` run.

## File Map

| File | Role |
| --- | --- |
| `generated/HELDOUT_N50_MANIFEST.md` | Protocol and source manifest for the held-out N50 check. |
| `generated/STRESS_HARSH_MANIFEST.md` | Protocol and source manifest for the harsh packet-loss N50 check. |
| `generated/SCENARIO_FAMILY_MANIFEST.md` | Protocol, tier labels, and source hashes for topology/FOV checks. |
| `generated/REPRODUCIBILITY_LEDGER_MANIFEST.md` | Source and SHA-256 ledger for manuscript-facing evidence roles. |
| `generated/N50_VERIFICATION_REPORT.md` | Independent recomputation of network disagreement, runtime, and local metrics. |
| `generated/SUBMISSION_READINESS_REPORT.md` | Current machine-readable submission-readiness summary in human-readable form. |

