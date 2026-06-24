# TAES Cover Letter and Submission Metadata Draft

This draft is a submission-package aid for the TAES portal. Replace bracketed placeholders and verify the current online instructions before submission.

## Cover Letter Draft

Dear Editor-in-Chief and Senior Editor,

We submit the manuscript titled "Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking Under Unreliable Communication" for consideration as a Regular Paper in IEEE Transactions on Aerospace and Electronic Systems. We believe the most appropriate technical area is Target Tracking and Multi-Sensor Systems.

The manuscript addresses a target-tracking failure mode that is directly relevant to distributed aerospace and surveillance systems: unreliable peer-to-peer communication can cause local labeled multi-Bernoulli filters to retain incompatible Bernoulli component labels, even when scalar AA/KLA fusion weights are routed to the correct existence and spatial consumers. The resulting estimates can appear cardinality-plausible while averaging states that correspond to different physical targets.

The paper proposes a graph-local neighborhood label-barycenter projection layer that treats this as a correspondence problem rather than another scalar-weight search. Each sensor selects a neighborhood reference label set, assigns neighboring tracks to that reference, and then fuses matched Gaussian posterior moments. This design separates label-set canonicalization from posterior barycentering, allowing a reference-only ablation to test whether label agreement alone explains the observed gain.

The evaluation is fixed-design and report-driven. The manuscript compares the method with a fixed target-wise spatial-KLA AA baseline (retaining the validation-report label Tuned spatial-KLA AA), a reference-only label-consensus ablation, and contextual GA reference rows. It also includes a separate 50-trial held-out replication, a harsh packet-loss N50 stress check, and fixed-parameter topology-ring and partial-FOV N50 scenario-family checks. The source package keeps these evidence roles explicit through generated manifests, an independent N50 verifier, DOI-resolved references, rendered PDF visual QA, and a reproducible source bundle.

The primary paired N50 run reports 81.59% lower network OSPA disagreement, 17.15% lower local E-OSPA, and 6.35% lower RMSE than the fixed AA baseline. The reference-only ablation reduces RMSE by only 0.54%, and the held-out N50 replication preserves the separation between full barycentering and label copying alone (6.64% versus 0.82% RMSE reduction). These results support the paper's mechanism claim while keeping the interpretation tied to fixed-parameter distributed tracking scenarios.

We believe the paper fits TAES because it contributes a concrete distributed tracking mechanism, a compact structural analysis, and reproducible multi-sensor evidence under unreliable communication. The claims are deliberately bounded: the projection is an active-output label-and-moment layer rather than a replacement for the Bernoulli existence update, and the Discussion identifies assignment ambiguity, lifecycle handling, and covariance-aware recursive deployment as limitations.

This submission is original work and is not under consideration elsewhere. All authors have approved the manuscript and agree with its submission to IEEE Transactions on Aerospace and Electronic Systems. The manuscript uses simulated multi-target tracking data only; no human or animal subjects are involved.

[If applicable: A preprint version has been posted at [preprint URL]. The submitted manuscript complies with IEEE preprint policy.]

[If applicable: The authors declare no conflicts of interest. / Disclose any conflicts here.]

OpenAI Codex assisted with code navigation, experiment-log organization, evidence cross-checking, and language polishing. The authors remain responsible for all technical claims, derivations, citations, experiments, and manuscript content.

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
