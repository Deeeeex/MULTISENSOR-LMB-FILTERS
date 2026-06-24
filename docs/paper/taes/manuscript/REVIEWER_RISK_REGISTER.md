# TAES Reviewer Risk Register

This register is an internal pre-submission response map. It is not an additional claim source and should not be cited as evidence. It maps likely reviewer concerns to manuscript locations, generated artifacts, and explicit boundaries so revision planning stays aligned with the current evidence package.

## Risk Map

| Reviewer concern | Current manuscript answer | Evidence / artifact | Boundary / residual risk |
| --- | --- | --- | --- |
| This is not another AA/KLA weighting rule. | The method is a correspondence-and-projection layer applied after the upstream AA existence consumer, not a density-pooling or scalar-weight replacement. | Introduction, Related Work, Fig. 1, and the `discussion interpretation markers` readiness gate. | Keep the claim focused on residual label/moment correspondence; do not describe it as a general AA/KLA weight optimizer. |
| Label copying alone may explain the gain. | The reference-only ablation isolates the label-set effect, while the full method adds matched moment barycenters and produces the reported RMSE separation. | Main paired table, held-out table, Fig. 3, Discussion, `generated/heldout_n50_section.tex`, and `generated/N50_VERIFICATION_REPORT.md`. | The mechanism interpretation is supported under the fixed-design scenarios reported here, not under arbitrary target dynamics. |
| Reviewers may question recursive LMB validity. | The paper presents an output-level active-track correspondence module, not a recursive LMB update. The conclusion states that recursive LMB use still requires lifecycle and consistency guards. | Method existence pass-through, Structural Properties, Discussion, and `conclusion boundary wording` readiness gate. | A recursive online filter with births, deaths, close crossings, and feedback consistency is not yet validated. |
| Equal moment barycenters may ignore covariance/reliability quality. | The equal barycenter is a first-two-moment least-squares representative of each matched group, deliberately kept simple to isolate correspondence repair after target-wise AA routing is fixed. | Method definition, Moment-space projection proposition, Discussion limitations, and fixed-design experimental setup. | Covariance-aware or reliability-weighted barycenters remain future design points; covariance-consistency validation is not part of the current evidence. |
| Generality beyond the main formation scenario may be limited. | The manuscript includes held-out, harsh packet-loss, topology-ring, and partial-FOV checks while keeping parameters fixed. | `generated/HELDOUT_N50_MANIFEST.md`, `generated/STRESS_HARSH_MANIFEST.md`, `generated/SCENARIO_FAMILY_MANIFEST.md`, and `SUPPLEMENTARY_EVIDENCE_PACKAGE.md`. | These checks do not substitute for maneuvering-target, covariance-consistency, or recursive-online validation. |
| Runtime overhead may be too high. | The runtime paragraph presents the cost as a paired Octave prototype comparison driven mainly by assignment and moment-barycenter operations. | Runtime table/paragraph, trial log, `generated/N50_VERIFICATION_REPORT.md`, and reproducibility ledger. | Absolute seconds are hardware/prototype dependent; the current evidence supports a relative paired comparison, not an optimized implementation benchmark. |
| Reproducibility and generated evidence integrity may be questioned. | The package uses generated fragments, manifests, DOI verification, readiness checks, a source bundle, and bundled-fragment rebuild mode. | `generated/SUBMISSION_READINESS_REPORT.md`, `generated/SUBMISSION_BUNDLE_MANIFEST.md`, `generated/BIBTEX_DOI_VERIFICATION.md`, and extracted source-bundle rebuild. | Raw `RUN/` logs are not part of the submitted source zip; generated fragments and manifests are included, and raw evidence regeneration requires the full repository. |

## Use in Revision / Response

- Use this register for response planning only.
- If a formal supplement is needed, use `SUPPLEMENTARY_EVIDENCE_PACKAGE.md` and selected generated fragments as the source map.
- For clarity: do not cite this register as a data source.
- Do not add new claims from this register without a corresponding manuscript edit, generated artifact, and readiness gate.
