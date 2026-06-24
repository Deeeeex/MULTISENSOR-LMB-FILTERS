# TAES 稿件 Readiness 审计

日期: 2026-06-24 12:37 CST

目标稿件: `Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication`

目标期刊: IEEE Transactions on Aerospace and Electronic Systems (TAES), Regular Paper, Technical area `Target Tracking and Multi-Sensor Systems`

## 当前判断

稿件已经进入 content-ready 阶段: TAES 模板、核心方法叙事、理论性质、N50 主实验、ablation、runtime、baseSeed=11 held-out N50 robustness check 和 disclosure skeleton 都已经落到 `main.tex` 和 `generated/` evidence fragments 中。按“作者/基金/repository 可先占位”的项目约定，当前内容、证据、引用、PDF 和 source bundle gate 已经闭合；但还不能标记为 portal-submission-ready，因为投稿元数据仍缺最后闭环:

1. 作者、基金、repository DOI/URL、corresponding author 等投稿元数据仍是占位符。
2. 当前实证仍集中在 tiered packet-loss formation 场景；baseSeed=11 held-out N50 已关闭跨 seed robustness gate，但更广 packet-loss/topology/target-maneuver 场景族验证仍会降低审稿风险。

## Claim-to-Evidence Matrix

| Paper-facing claim | Manuscript location | Primary evidence | Verification level | Remaining gate |
| --- | --- | --- | --- | --- |
| AA/KLA scalar weights decide fusion mass/trust but do not solve Bernoulli component correspondence across local LMB posteriors. | Introduction, Problem Formulation, Structural Properties | AA fusion literature synthesis; `Weighting is not matching` label-swap counterexample in `main.tex` | Conceptual, citation-backed, and formalized by a minimal counterexample | Final language pass should keep this as the central motivation and avoid overclaiming all AA failures. |
| The proposed operator first canonicalizes labels by assignment and then fuses matched posterior moments. | Method, Fig. 1, Algorithm box | `scripts/render_figures.py`; `generated/method_pipeline.tex`; algorithm text in `main.tex` | Source-backed and rendered in PDF | Final PDF visual check after every figure/table edit. |
| The proposed layer is a label-and-moment projection on active output tracks, not a replacement for the AA Bernoulli existence consumer. | Problem Formulation, Matched Moment Barycenter, Structural Properties, Experimental Setup | `multisensorLmb/applyCrossLocalLabelConsensusProjection.m`; `multisensorLmb/runDistributedLmbFilter.m`; revised `main.tex` wording | Code-aligned and manuscript-explicit | Keep existence-branch claims tied to upstream AA convex weighting; do not imply the projection estimates new existence probabilities. |
| The operator is graph-local in the neighborhood version and does not require global label-set access. | Method, Graph locality paragraph | `RUN/AA/runAaBalancedCardinalityValidation.m`; neighborhood N50 report | Code path and report-backed | Keep centralized upper-bound language out of the main claim. |
| Under stable matching and connected repeated neighborhood averaging, local moments converge to the centralized equal-weight barycenter. | Structural Properties | Proposition in `main.tex` | Theory stated with explicit assumptions | Do not present as a finite-round guarantee; check proof wording before final submission. |
| Matched posterior barycenters, not label copying alone, drive the spatial tracking gain. | Results and Discussion | Main N50 full-vs-reference-only ablation; held-out baseSeed=11 N50 RMSE separation; paired RMSE reductions/wins/sign-test p-values in Table IV and held-out fragment | Report-driven fragments plus independent local-metric verifier and held-out robustness gate | Broader scenario-family validation remains useful but is no longer the immediate held-out blocker. |
| Neighborhood label-barycenter improves local E-OSPA/RMSE/CardErr relative to tuned spatial-KLA AA in the N50 validation. | Results, Table I/II, Fig. 3 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `generated/n50_evidence.json`; `generated/N50_VERIFICATION_REPORT.md` | Local metrics independently recomputed from per-trial local rows | None for current N50; broaden scenario coverage before submission. |
| Neighborhood label-barycenter is lower than the two tracked GA reference rows on the six reported N50 disagreement/tracking metrics. | Results, contextual reference table | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Report-driven contextual comparison | Keep phrasing as contextual reference rows, not paired GA-vs-AA significance evidence. |
| Network disagreement and runtime numbers are reproducible from raw per-trial artifacts. | Results, reproducibility notes | `verify_n50_evidence.py`; `N50_VERIFICATION_REPORT.md` | Independently recomputed | None for network/runtime; keep source hash in manifest. |
| The main mechanism is not a base-seed-1 accident. | Results optional held-out fragment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_evidence.json`; `generated/heldout_n50_section.tex` | Strict held-out gate checks baseSeed=11, N50, trial seeds 12..61, three-arm metric coverage, paired evidence, and RMSE mechanism separation | If moved to supplement, keep the source artifact and manifest referenced in the response-ready evidence package. |
| Runtime overhead is mainly due to repeated assignment/moment-barycenter operations, not hidden global communication. | Runtime table, complexity paragraph | Runtime log and graph-locality method text | Independently recomputed runtime | Add a short scaling note if broader N or topology experiments are added. |
| Absolute runtime seconds are prototype-specific; the paper-facing runtime comparison is the relative cost within the paired Octave validation. | Experimental Setup, Runtime table | Local environment check: GNU Octave 11.1.0 on Apple M4, 16 GB memory | Manuscript-explicit and environment-backed | Keep absolute runtime claims modest; do not present them as hardware-independent benchmarks. |
| Each experiment arm isolates a specific mechanism rather than serving as an unconstrained tuning comparison. | Experimental Setup | Claim-to-arm mapping in `main.tex`; fixed-design statement that $H=3$, the existence threshold, projection cutoffs, barycenter weights, and trial-specific label rules are not searched per scenario | Manuscript-explicit and report-driven | Maintain this mapping when adding held-out or recursive-online experiments; held-out base-seed runs should remain robustness checks, not tuning sources. |

## TAES Compliance Gates

| Gate | Current state | Required before submission |
| --- | --- | --- |
| Official template | `IEEEtaes.cls` and `IEEEtaes.bst` are used by `main.tex`; `./build.sh` compiles the manuscript. | Rebuild after final edits; render pages and inspect figures/tables. |
| Template and author-guideline archive | `generated/SUBMISSION_READINESS_REPORT.md` now checks the local TAES requirements document, official template zip, and regular-paper template source as required artifacts. | Recheck online TAES/AESS pages immediately before final submission. |
| Clean source bundle | `./build.sh` now writes `tmp/submission_bundle/taes_label_barycenter_submission_source.zip` and `generated/SUBMISSION_BUNDLE_MANIFEST.md`; the zip was extracted to `/tmp/taes_submission_bundle_check` and compiled with Tectonic. | Regenerate after final evidence or metadata edits; submit the current PDF plus a clean source bundle according to the portal instructions. |
| Manuscript type | Regular Paper is the selected target. | Submit as `Regular Paper`; technical area `Target Tracking and Multi-Sensor Systems`. |
| Page budget | Current manuscript is within a plausible Regular Paper range, but final printed page count is not guaranteed. | Keep estimated TAES pages near or below 10, or accept overlength charges. |
| Title/abstract/keywords | The readiness checker now verifies that the title/abstract avoid `new`/`novel`, the abstract is a single paragraph without citation/footnote/display equation, and keywords are alphabetized. | Re-run `./build.sh` after final title/abstract edits. |
| Author metadata | Bracketed author/funding/repository placeholders remain in submission-style prose. | Fill real author list, affiliations, ORCID, funding, acknowledgments, corresponding author metadata. |
| AI disclosure | Provisional Codex/OpenAI disclosure exists in Acknowledgment. | Recheck current IEEE/AESS wording and decide final disclosure scope. |
| Citations | Core bibliography has been DOI/source checked during draft construction; the readiness checker verifies cited keys, DOI fields, and uncited BibTeX entries. | Final bibliography scan for malformed entries and unsupported claims. |
| Preprint/reuse | No final decision recorded. | Decide whether to post preprint and prepare IEEE-compliant preprint notice if needed. |
| Submission files | PDF builds locally; source bundle exists in manuscript directory. | Prepare clean source zip, PDF, cover metadata, and any supplementary material. |
| Machine-checkable readiness | `generated/SUBMISSION_READINESS_REPORT.md` is written by `./build.sh`; current portal status is `draft_with_pending_gates`, while content status is `content_ready_metadata_pending`. | Close the metadata placeholders before actual portal submission; content review can proceed under the project convention that these placeholders are allowed. |
| Cover letter and portal metadata | `COVER_LETTER_AND_METADATA_DRAFT.md` now provides an editable cover-letter draft and portal metadata checklist. | Replace author/funding/repository/preprint/conflict placeholders and recheck the current TAES portal wording before submission. |

## Evidence Gates

| Gate | Status | Evidence | Next action |
| --- | --- | --- | --- |
| Report-driven tables/figure fragments | Passed | `extract_n50_evidence.py`; `generated/N50_EVIDENCE_MANIFEST.md` | Keep generated fragments read-only by convention. |
| Report-driven GA reference rows | Passed | `extract_reference_baselines.py`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Maintain the contextual-comparison caveat in the manuscript. |
| Independent network disagreement verifier | Passed | `verify_n50_evidence.py`; `generated/N50_VERIFICATION_REPORT.md` | Maintain hash check against source report. |
| Independent runtime verifier | Passed | `verify_n50_evidence.py`; trial log parsing | Maintain relative-cost check after source report swap. |
| Independent local metric verifier | Passed | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`; `generated/N50_VERIFICATION_REPORT.md` | Maintain `evidence_sources.json` as the single report/log source manifest. |
| Held-out scenario evidence | Passed for baseSeed=11 N50 robustness | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_SANITY_MANIFEST.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_evidence.json`; `generated/heldout_n50_section.tex` | Keep the compact held-out table/paragraph in the manuscript or move it to supplementary/response-ready material after final page-budget review. The readiness checker requires base seed 11, at least 50 trials, parsed trial seeds, generated manifest/LaTeX fragment, all three arms, all manuscript metrics, paired CI/wins/p-values, and a visible full-barycenter-vs-reference-only RMSE separation check. |
| PDF visual QA | Passed for current checkpoint | `./build.sh`; ImageMagick-rendered checks of the title/abstract page, problem/method pages, structural-properties pages, and result/reference pages after the scalar-weight counterexample edit | Re-render final `main.pdf` after every manuscript-affecting checkpoint. |
| Submission readiness checker | Passed for all non-metadata gates | `check_submission_readiness.py`; `generated/SUBMISSION_READINESS_REPORT.md`; `generated/submission_readiness.json` | Checker now reports both `portal_status` and `content_status`: the portal status remains blocked by metadata placeholders, while the content status ignores only those placeholders and should be `content_ready_metadata_pending` when all non-metadata gates pass. |
| Source-bundle rebuild check | Passed | `generated/SUBMISSION_BUNDLE_MANIFEST.md`; `/tmp/taes_submission_bundle_check` Tectonic compile | Re-run after final manuscript-affecting edits. |

## Immediate Execution Order

1. Replace author/funding/repository/corresponding-author/OA/preprint/conflict placeholders once the real submission metadata is available.
2. Rebuild `main.pdf` and re-render the title/abstract page, method pages, held-out table page, result tables/figure page, and final reference page after any evidence or metadata edit.
3. Decide whether the compact held-out N50 table/paragraph stays in the main paper, moves to supplementary material, or remains response-ready evidence depending on final page budget.
4. Consider broader packet-loss/topology/target-maneuver scenario-family validation as the next evidence-risk reduction step, not as a blocker for the current held-out gate.
