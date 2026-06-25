# TAES Cover Letter and Submission Metadata Draft

This draft is a submission-package aid for the TAES portal. Replace bracketed placeholders and verify the current online instructions before submission.

## Cover Letter Draft

Dear Editor-in-Chief and Senior Editor,

We submit the manuscript titled "Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking Under Unreliable Communication" for consideration as a Regular Paper in IEEE Transactions on Aerospace and Electronic Systems. The most appropriate technical area is Target Tracking and Multi-Sensor Systems.

The manuscript addresses a failure mode directly relevant to distributed aerospace and surveillance systems. Under unreliable peer-to-peer communication, local labeled multi-Bernoulli filters can retain incompatible Bernoulli component labels after scalar AA/KLA weights reach the correct existence and spatial consumers. The resulting estimates can appear cardinality-plausible while averaging states from different physical targets.

The paper proposes a graph-local neighborhood label-barycenter projection layer. The layer treats the residual error as a correspondence problem, not as another scalar-weight search. Each sensor selects a neighborhood reference label set, assigns neighboring tracks to that reference, and replaces matched Gaussian posterior states by first-two-moment barycenters. This separates label-set canonicalization from posterior barycentering. A reference-only ablation then tests whether label agreement alone explains the observed gain.

The evaluation is fixed-design and report-driven. We compare the method with a fixed target-wise spatial-KLA AA baseline, a reference-only ablation, and contextual GA reference rows. The GA rows provide matched-seed context only, not AA-versus-GA significance evidence. Raw validation reports retain their internal implementation labels for provenance. The manuscript uses fixed-baseline terminology throughout. The evidence package also includes a separate held-out N50 replication, a harsh packet-loss stress check, topology-ring and partial-FOV scenario checks, and a full-topology equivalence-boundary check. Generated manifests, an independent N50 verifier, DOI-resolved references, PDF visual QA, and a source bundle preserve these evidence roles.

The primary paired N50 run reports 81.59% lower network OSPA disagreement, 17.15% lower local E-OSPA, and 6.35% lower RMSE than the fixed AA baseline. The reference-only arm reduces RMSE by 0.54%. Its shared cardinality effect is therefore interpreted as a reference-cardinality control, while the E-OSPA/RMSE separation is attributed to matched posterior barycentering. The held-out N50 replication preserves the full-versus-reference separation (6.64% versus 0.82% RMSE reduction). These results support the mechanism claim while keeping the interpretation tied to fixed-parameter distributed tracking scenarios.

We believe the paper fits TAES because it contributes a concrete distributed tracking mechanism, a compact structural analysis, and reproducible multi-sensor evidence under unreliable communication. The claims remain deliberately bounded. The projection is an active-output label-and-moment layer, not a replacement for the Bernoulli existence update. The Discussion identifies assignment ambiguity, lifecycle handling, and covariance-aware recursive deployment as limitations.

This submission is original work and is not under consideration elsewhere. All authors have approved the manuscript and agree to its submission to IEEE Transactions on Aerospace and Electronic Systems. The manuscript uses simulated multi-target tracking data only; no human or animal subjects are involved.

[If applicable: A preprint version has been posted at [preprint URL]. The submitted manuscript complies with IEEE preprint policy.]

[If applicable: The authors declare no conflicts of interest. / Disclose any conflicts here.]

OpenAI Codex assisted with code navigation, log organization, evidence checking, and language polishing. The authors remain responsible for all technical content.

Thank you for considering this manuscript.

Sincerely,

[Corresponding Author Name]

[Institution]

[Email]

## Portal Metadata Checklist

| Field | Draft value | Final action |
| --- | --- | --- |
| Journal | IEEE Transactions on Aerospace and Electronic Systems | Verify in Atypon/REX portal. |
| Manuscript type | Regular Paper | Select Regular Paper, not Correspondence or Letter. |
| Technical area | Target Tracking and Multi-Sensor Systems | Verify the portal wording. |
| Title | Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking Under Unreliable Communication | Match `main.tex` exactly. |
| Running head | LABEL-BARYCENTER LMB FUSION | Match `\markboth`. |
| Corresponding author | [FIRST AUTHOR] | Replace placeholder and confirm email/ORCID. |
| ORCID | [All authors] | IEEE requires ORCID for all authors. |
| Funding | [Funding Agency] under Grant [Grant Number] | Replace or remove if unfunded. |
| Repository / code availability | [repository DOI/URL] | Replace with a stable URL/DOI or remove if unavailable. |
| Code Ocean | [none / URL] | Decide whether to use IEEE Code Ocean; if used, keep the repository and cover-letter wording synchronized. |
| DataPort | [none / DOI] | Decide whether any large simulation dataset is deposited in IEEE DataPort. |
| Supplementary material | [no separate supplement by default / selected supplement files] | Current package keeps candidate blocks response-ready and source-bundle-ready; if submitted, provide it during peer review and ensure the main paper references it. |
| Graphical/video abstract | [none / file] | Decide whether to submit an optional graphical or video abstract. |
| Preprint | [none / URL] | Decide before submission and ensure IEEE-compliant wording. |
| Conflicts of interest | [none / disclosure] | Confirm with all authors. |
| AI assistance disclosure | Codex/OpenAI assistance disclosed in acknowledgments and cover letter | Recheck current IEEE/AESS wording before submission. |
| Suggested reviewers | [optional] | Add only if the portal requests them and conflicts are checked. |
| Opposed reviewers | [optional] | Add only with concrete conflict rationale if needed. |

## Final Pre-Submission Replacements

- Replace all author, affiliation, funding, DOI, email, repository, and ORCID placeholders in `main.tex`.
- Record no separate supplement upload or the exact selected supplement files; also decide whether to provide optional graphical/video abstract, Code Ocean, or DataPort artifacts.
- Rebuild with `./build.sh` and confirm `generated/SUBMISSION_READINESS_REPORT.md` has no hard errors.
- Render and inspect the latest `main.pdf` pages after metadata replacement.
- Regenerate the source bundle and confirm `generated/SUBMISSION_BUNDLE_MANIFEST.md` records the final bundle checksum.
- Attach the final PDF and source bundle according to the TAES portal instructions.
