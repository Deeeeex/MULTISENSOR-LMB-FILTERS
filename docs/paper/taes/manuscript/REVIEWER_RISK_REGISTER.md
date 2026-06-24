# TAES Reviewer Risk Register

This register is an internal pre-submission response map. It is not an additional claim source and should not be cited as evidence. It maps likely reviewer concerns to manuscript locations, generated artifacts, and explicit boundaries so revision planning stays aligned with the current evidence package.

## Reviewer-Style Preflight Synthesis

Input scope: current TAES manuscript, cover-letter draft, generated readiness report, supplementary evidence map, and tracked evidence manifests. Assessment boundary: this is an author-side pre-submission review, not an editorial decision and not a substitute for new experiments.

Shared manuscript claim: after target-wise AA weights have reached the existence and spatial consumers, the remaining failure is Bernoulli component correspondence; a graph-local reference, assignment, and moment-barycenter projection repairs that output-level correspondence for active tracks.

Visible evidence base: primary paired N50 validation, reference-only ablation, held-out N50 replication, harsh packet-loss N50 stress check, topology-ring and partial-FOV N50 scenario-family checks, contextual GA rows, independent N50 verifier, DOI resolver check, PDF visual QA, and source-bundle rebuild.

Missing materials affecting confidence: final author/funding/repository metadata, full-topology ceiling result while the run is still in progress, maneuvering-target validation, covariance-consistency validation, and recursive-online LMB validation with lifecycle guards. The method-level follow-up design for these validation families is recorded in `docs/AA_NEXT_STAGE_GENERALIZATION_PROTOCOL_CN.md`.

### Reviewer 1: Technical Soundness Emphasis

- Overall assessment: technically coherent and unusually well bounded for an algorithmic tracking paper, with the strongest case coming from the reference-only ablation and fixed-design robustness checks.
- Who would be interested: distributed multi-target tracking, labeled RFS fusion, and multi-sensor systems readers who need output consistency under unreliable links.
- Major strengths: the method separates scalar fusion weight routing from component correspondence, states an active-output scope, and backs the mechanism with paired trials plus a held-out replication.
- Major concerns: recursive validity, assignment ambiguity, covariance quality, and target-maneuver behavior are not established by the current evidence.
- Technical failings to address before a stronger case: do not imply recursive LMB correctness; do not treat equal barycenters as covariance-consistent; do not use the running full-topology ceiling run until it is complete and parsed through the scenario-family gate.
- Assessment against reviewer axes: original within the paper's AA-LMB correspondence framing; technically sound for output-level active tracks; broader generality remains bounded.
- Recommendation posture: defensible for TAES as a bounded method paper if the scope language remains as narrow as the current manuscript.

### Reviewer 2: Originality and Significance Emphasis

- Overall assessment: the contribution is not another AA/KLA weighting rule; its value is the explicit correspondence contract between local LMB outputs.
- Who would be interested: readers working on AA/KLA fusion, peer-to-peer sensor networks, and practical labeled-track fusion under packet loss.
- Major strengths: the first-page story distinguishes probability-mass allocation from component matching, and the ablation makes the novelty testable rather than rhetorical.
- Major concerns: the method may be perceived as an engineering projection unless the paper keeps the correspondence-contract framing, structural properties, and prior-work differentiation from multiview and efficient-label-matching LMB work visible.
- Technical failings to address before a stronger case: prior-work contrast should remain about active-output correspondence projection and ablation-based mechanism isolation, not about claiming that existing AA/KLA or label-matching methods are generally inadequate.
- Assessment against reviewer axes: significance is field-local but credible for TAES; broad interdisciplinary reach is limited, which is acceptable for the target journal but should not be overstated.
- Recommendation posture: likely strongest when submitted as a targeted aerospace/sensor-fusion contribution rather than as a universal RFS-fusion theory.

### Reviewer 3: Readability and Reuse Emphasis

- Overall assessment: the manuscript is now readable as a mechanism paper, but final reviewers will need a clean path from problem framing to evidence provenance.
- Who would be interested: implementers who need a graph-local module that can be inserted after an existing distributed LMB/AA output path.
- Major strengths: Fig. 1, the compact algorithm box, and the generated evidence manifests reduce ambiguity about what the operator consumes and what it changes.
- Major concerns: response-ready evidence is extensive; the submitted main paper must keep the reader's path simple and avoid turning the Discussion into a catalog of every check.
- Technical failings to address before a stronger case: keep supplementary tables optional unless page budget allows; keep generated fragments source-driven; keep final metadata and repository instructions synchronized.
- Assessment against reviewer axes: readability for TAES specialists is strong; nonspecialist accessibility is adequate but not the target strength of the submission.
- Recommendation posture: ready for final metadata replacement and one last visual/source-bundle QA once running evidence is either integrated or explicitly left out.

### Cross-Review Synthesis

- Consensus strengths: a clear residual failure mode, a bounded graph-local mechanism, reference-only ablation, held-out replication, fixed-parameter boundary checks, and reproducibility scaffolding.
- Consensus technical risks: recursive LMB validity, assignment ambiguity, covariance/reliability weighting, target maneuvers, and the still-running full-topology ceiling check.
- Where emphasis differs: Reviewer 1 weighs technical boundaries, Reviewer 2 weighs originality and positioning, and Reviewer 3 weighs handoff/readability and submission-package discipline.
- Broad-interest / significance readout: the case is strongest as a TAES target-tracking and distributed-sensor contribution; it should not be framed as a broad Nature-style general fusion result.
- Most important issues to resolve before a stronger case: finish or explicitly defer the full-topology ceiling run, keep all generalization claims tied to parsed evidence, and avoid expanding the method beyond active-output correspondence projection.

## Risk / Unsupported Claims

- Not assessable from current evidence: recursive online filtering with feedback, births, deaths, close crossings, target maneuvers, and lifecycle-consistency guards.
- Not assessable from current evidence: covariance consistency or reliability-weighted optimality of the equal moment barycenter.
- Not assessable until completion: the full-topology ceiling run started on 2026-06-25; it must not enter manuscript claims before a report is complete, configured in `evidence_sources.json`, and parsed by `extract_scenario_family_evidence.py`.
- Unsupported if overstated: claims that the operator replaces AA/KLA density fusion, optimizes fusion weights, guarantees finite-round consensus, or solves general label management.
- Unsupported if overstated: claims that prior multiview LMB fusion or efficient label-matching work did not address labels; the manuscript may only claim a different active-output projection layer and a reference-only ablation that separates label copying from matched posterior barycentering.
- Follow-up protocol status: `docs/AA_NEXT_STAGE_GENERALIZATION_PROTOCOL_CN.md` defines fixed no-search candidate validations for maneuver/crossing assignment stress, covariance/reliability mismatch, and recursive guarded projection. These are future risk-reduction plans, not current manuscript evidence.

## Risk Map

| Reviewer concern | Current manuscript answer | Evidence / artifact | Boundary / residual risk |
| --- | --- | --- | --- |
| This is not another AA/KLA weighting rule. | The method is a correspondence-and-projection layer applied after the upstream AA existence consumer, not a density-pooling or scalar-weight replacement. | Introduction, Related Work, Fig. 1, and the `discussion interpretation markers` readiness gate. | Keep the claim focused on residual label/moment correspondence; do not describe it as a general AA/KLA weight optimizer. |
| The baseline or method may look tuned to the reported data. | The manuscript now uses `fixed target-wise spatial-KLA AA baseline` and `Fixed spatial-KLA AA` for paper-facing comparisons. It keeps raw validation-report implementation labels only in provenance artifacts, while the Experimental Setup states that all comparisons use a fixed parameterization rather than per-scenario retuning, with no per-scenario search over projection cutoffs, barycenter weights, thresholds, or trial-specific label rules. | Abstract, Introduction, Experimental Setup, generated table fragments, `fixed-design baseline wording` readiness gate, and `CLAIM_EVIDENCE_BOUNDARY_MAP.md`. | Do not describe held-out, harsh-loss, topology-ring, partial-FOV, or future full-topology runs as tuning sources. Keep method-level innovations separate from earlier diagnostic baseline development. |
| How is this different from multiview LMB fusion and efficient label matching? | The manuscript now cites both adjacent lines and frames the distinction narrowly: the paper isolates an active-output correspondence projection layer and tests label copying against matched posterior barycentering through a reference-only ablation. | Related Work, `Jin2023MultiviewLMB`, `Ding2025EfficientLabelMatching`, DOI resolver report, `recent label-matching literature positioning` readiness gate, and `CLAIM_EVIDENCE_BOUNDARY_MAP.md`. | Do not claim those prior works ignore labels or are inferior. The defensible contrast is mechanism isolation, output-layer scope, and ablation design under the present fixed scenarios. |
| Label copying alone may explain the gain. | The reference-only ablation isolates the label-set effect, while the full method adds matched moment barycenters and produces the reported RMSE separation. | Main paired table, held-out table, Fig. 3, Discussion, `generated/heldout_n50_section.tex`, and `generated/N50_VERIFICATION_REPORT.md`. | The mechanism interpretation is supported under the fixed-design scenarios reported here, not under arbitrary target dynamics. |
| Reviewers may question recursive LMB validity. | The paper presents an output-level active-track correspondence module, not a recursive LMB update. The conclusion states that recursive LMB use still requires lifecycle and consistency guards. | Method existence pass-through, Structural Properties, Discussion, and `conclusion boundary wording` readiness gate. | A recursive online filter with births, deaths, close crossings, and feedback consistency is not yet validated. |
| Equal moment barycenters may ignore covariance/reliability quality. | The equal barycenter is a first-two-moment least-squares representative of each matched group, deliberately kept simple to isolate correspondence repair after target-wise AA routing is fixed. | Method definition, Moment-space projection proposition, Discussion limitations, and fixed-design experimental setup. | Covariance-aware or reliability-weighted barycenters remain future design points; covariance-consistency validation is not part of the current evidence. |
| Generality beyond the main formation scenario may be limited. | The manuscript includes held-out, harsh packet-loss, topology-ring, and partial-FOV checks while keeping parameters fixed. | `generated/HELDOUT_N50_MANIFEST.md`, `generated/STRESS_HARSH_MANIFEST.md`, `generated/SCENARIO_FAMILY_MANIFEST.md`, and `SUPPLEMENTARY_EVIDENCE_PACKAGE.md`. | These checks do not substitute for maneuvering-target, covariance-consistency, or recursive-online validation. |
| Running full-topology evidence could be overinterpreted. | The full-topology run is documented only as an idealized ceiling check until it completes and is parsed through the scenario-family gate. | `docs/AA_GENERALIZATION_SCENARIO_PROTOCOL_CN.md`, `READINESS_AUDIT_CN.md`, and the running log/pid path. | Do not add full-topology results to `main.tex`, generated fragments, or cover-letter evidence until the report exists and source hashes are refreshed. |
| Runtime overhead may be too high. | The runtime paragraph presents the cost as a paired Octave prototype comparison driven mainly by assignment and moment-barycenter operations. | Runtime table/paragraph, trial log, `generated/N50_VERIFICATION_REPORT.md`, and reproducibility ledger. | Absolute seconds are hardware/prototype dependent; the current evidence supports a relative paired comparison, not an optimized implementation benchmark. |
| Reproducibility and generated evidence integrity may be questioned. | The package uses generated fragments, manifests, DOI verification, readiness checks, a source bundle, and bundled-fragment rebuild mode. | `generated/SUBMISSION_READINESS_REPORT.md`, `generated/SUBMISSION_BUNDLE_MANIFEST.md`, `generated/BIBTEX_DOI_VERIFICATION.md`, and extracted source-bundle rebuild. | Raw `RUN/` logs are not part of the submitted source zip; generated fragments and manifests are included, and raw evidence regeneration requires the full repository. |

## Use in Revision / Response

- Use this register for response planning only.
- If a formal supplement is needed, use `SUPPLEMENTARY_EVIDENCE_PACKAGE.md` and selected generated fragments as the source map.
- For clarity: do not cite this register as a data source.
- Do not add new claims from this register without a corresponding manuscript edit, generated artifact, and readiness gate.
