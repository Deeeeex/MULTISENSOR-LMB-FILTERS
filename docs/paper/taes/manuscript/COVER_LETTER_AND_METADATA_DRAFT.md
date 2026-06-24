# TAES Cover Letter and Submission Metadata Draft

This draft is a submission-package aid for the TAES portal. Replace bracketed placeholders and verify the current online instructions before submission.

## Cover Letter Draft

Dear Editor-in-Chief and Senior Editor,

We submit the manuscript titled "Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking Under Unreliable Communication" for consideration as a Regular Paper in IEEE Transactions on Aerospace and Electronic Systems. We believe the most appropriate technical area is Target Tracking and Multi-Sensor Systems.

The manuscript addresses distributed labeled multi-Bernoulli tracking when unreliable peer-to-peer communication causes local filters to retain incompatible Bernoulli component labels. The paper proposes a graph-local neighborhood label-barycenter projection layer that first canonicalizes active output labels by medoid reference selection and assignment, then fuses matched Gaussian posterior moments. The method is evaluated against a tuned spatial-KLA AA baseline, a reference-only label-consensus ablation, and contextual GA reference rows using report-driven and independently checked validation artifacts.

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
| Preprint | [none / URL] | Decide before submission and ensure IEEE-compliant wording. |
| Conflicts of interest | [none / disclosure] | Confirm with all authors. |
| AI assistance disclosure | Codex/OpenAI assistance disclosed in acknowledgments and cover letter | Recheck current IEEE/AESS wording before submission. |
| Suggested reviewers | [optional] | Add only if the portal requests them and conflicts are checked. |
| Opposed reviewers | [optional] | Add only with concrete conflict rationale if needed. |

## Final Pre-Submission Replacements

- Replace all author, affiliation, funding, DOI, email, repository, and ORCID placeholders in `main.tex`.
- Rebuild with `./build.sh` and confirm `generated/SUBMISSION_READINESS_REPORT.md` has no hard errors.
- Render and inspect the latest `main.pdf` pages after metadata replacement.
- Regenerate the source bundle and confirm `generated/SUBMISSION_BUNDLE_MANIFEST.md` records the final bundle checksum.
- Attach the final PDF and source bundle according to the TAES portal instructions.
