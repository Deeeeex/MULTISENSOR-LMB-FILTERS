# TAES Final Metadata Closure Checklist

This checklist is the last-step placeholder closure artifact for the TAES manuscript package. It does not add scientific claims or evidence. Its purpose is to convert the current metadata-pending readiness state into a concrete replacement plan for `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md`, the TAES portal form, and any optional upload decisions.

Generated status reviewed: 2026-06-25 05:17 CST.

## Current Readiness Boundary

- `generated/SUBMISSION_READINESS_REPORT.md` reports `content_ready_metadata_pending`.
- Metadata placeholders are allowed only for internal content-readiness review.
- The same placeholders remain blocking for actual TAES portal submission.
- This file should be rechecked after every author, funding, repository, cover-letter, or portal-metadata edit.

## Required Author And Front-Matter Replacements

| Placeholder or field | File(s) | Final replacement / decision |
| --- | --- | --- |
| `FIRST AUTHOR`, `SECOND AUTHOR`, `THIRD AUTHOR` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with the final author names and author order approved by all authors. |
| `First Author`, `Second Author`, `Third Author`, `AUTHOR ET AL.` | `main.tex` | Replace author-address and running-head placeholders consistently. |
| `Institution, City, Country`, `[Institution]`, `[City]`, `[Country]` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with final affiliations and mailing metadata. |
| `[author@example.com]`, `[Email]` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with the corresponding-author email and any required author emails. |
| `[FIRST AUTHOR]`, `[Corresponding Author Name]` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with the final corresponding author. |
| `[All authors]` | `COVER_LETTER_AND_METADATA_DRAFT.md` | Enter ORCID values for all authors, or record the portal-provided ORCID confirmation. |

## Funding, Repository, And Availability Decisions

| Placeholder or field | File(s) | Final replacement / decision |
| --- | --- | --- |
| `[Funding Agency]`, `[Grant Number]` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with grant details or a deliberate no-funding statement. |
| `[repository DOI/URL]` | `main.tex`, `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with a stable code/reproducibility URL or DOI, or state the no-public-code decision consistently. |
| `[none / URL]` | `COVER_LETTER_AND_METADATA_DRAFT.md` | Decide whether Code Ocean or preprint URLs are used and keep wording synchronized with the portal. |
| `[none / disclosure]` | `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with final conflict-of-interest disclosure. |
| `[optional]` | `COVER_LETTER_AND_METADATA_DRAFT.md` | Fill suggested/opposed reviewer fields only if the portal requests them and conflicts are checked. |
| `[preprint URL]` | `COVER_LETTER_AND_METADATA_DRAFT.md` | Replace with the final preprint URL or remove the bracketed sentence. |

## TAES Front-Matter And Post-Acceptance Fields

| Placeholder or field | File(s) | Final replacement / decision |
| --- | --- | --- |
| `Month 00` | `main.tex` | For initial submission, either keep the TAES template placeholder if required by the class or replace according to final portal/template guidance. |
| `XX`, `Draft` | `main.tex` | Keep only if required by the submitted template draft; otherwise replace with final journal metadata after acceptance. |
| `TAES.2026.Doi Number` | `main.tex` | Replace only when TAES assigns the final DOI, unless the portal/template requires a blank placeholder at submission. |
| `Graphical/video abstract` decision | `COVER_LETTER_AND_METADATA_DRAFT.md`, portal | Explicitly decide whether no graphical/video abstract is submitted or prepare an optional file. |
| Supplementary material decision | `SUPPLEMENTARY_README_DRAFT.md`, `SUPPLEMENTARY_EVIDENCE_PACKAGE.md`, portal | Use no separate supplementary upload by default. Override only if selected generated evidence is converted into a peer-review supplement; otherwise keep those blocks as response-ready/source-bundle evidence. |
| Code Ocean / DataPort decision | `COVER_LETTER_AND_METADATA_DRAFT.md`, portal | Decide whether either IEEE platform is used, and keep repository/data wording synchronized. |

## Closure Sequence

1. Replace author, affiliation, corresponding-author, ORCID, funding, repository, and disclosure placeholders in `main.tex` and `COVER_LETTER_AND_METADATA_DRAFT.md`.
2. Decide optional graphical/video abstract, supplementary material, Code Ocean, DataPort, preprint, suggested reviewer, and opposed reviewer fields; record no separate supplement upload unless a formal supplement is selected.
3. Run `./build.sh` from `docs/paper/taes/manuscript`.
4. Confirm `generated/SUBMISSION_READINESS_REPORT.md` has no non-metadata blocking gates and no remaining unintended metadata placeholders.
5. Render and inspect the final `main.pdf`, including the full-page PNG set and contact sheet under `tmp/pdf_visual_qa/`; confirm it remains within the intended TAES page budget.
6. Confirm `generated/SUBMISSION_BUNDLE_MANIFEST.md` records the final source-bundle checksum.
7. Extract `tmp/submission_bundle/taes_label_barycenter_submission_source.zip` and run `TAES_EVIDENCE_MODE=bundled ./build.sh`.
8. Fill the TAES portal from `COVER_LETTER_AND_METADATA_DRAFT.md` only after the rebuilt PDF, source bundle, and cover letter agree.

## Do Not Change During Metadata Closure

- Do not change experiment parameters, evidence-source paths, or generated metric fragments while replacing metadata.
- Do not reinterpret the parsed full-topology results as additional method gain; they are only a full-neighborhood zero-disagreement equivalence boundary.
- Do not broaden the method beyond active-output label/moment correspondence projection while editing author-facing text.
