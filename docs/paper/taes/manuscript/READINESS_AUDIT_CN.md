# TAES 稿件 Readiness 审计

日期: 2026-06-24 10:15 CST

目标稿件: `Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication`

目标期刊: IEEE Transactions on Aerospace and Electronic Systems (TAES), Regular Paper, Technical area `Target Tracking and Multi-Sensor Systems`

## 当前判断

稿件已经进入 paper-facing draft 阶段: TAES 模板、核心方法叙事、理论性质、N50 主实验、ablation、runtime 和 disclosure skeleton 都已经落到 `main.tex` 和 `generated/` evidence fragments 中。当前还不能标记为 submission-ready，主要原因不是 LaTeX 或故事线，而是投稿元数据和场景覆盖还缺最后闭环:

1. 作者、基金、repository DOI/URL、corresponding author 等投稿元数据仍是占位符。
2. 当前实证集中在 tiered packet-loss formation 场景；paper-grade held-out base-seed-11 N50 已启动但尚未完成。

## Claim-to-Evidence Matrix

| Paper-facing claim | Manuscript location | Primary evidence | Verification level | Remaining gate |
| --- | --- | --- | --- | --- |
| AA/KLA scalar weights decide fusion mass/trust but do not solve Bernoulli component correspondence across local LMB posteriors. | Introduction, Related Work | AA fusion literature synthesis; method contrast in `main.tex` | Conceptual and citation-backed | Final language pass should keep this as the central motivation and avoid overclaiming all AA failures. |
| The proposed operator first canonicalizes labels by assignment and then fuses matched posterior moments. | Method, Fig. 1, Algorithm box | `scripts/render_figures.py`; `generated/method_pipeline.tex`; algorithm text in `main.tex` | Source-backed and rendered in PDF | Final PDF visual check after every figure/table edit. |
| The operator is graph-local in the neighborhood version and does not require global label-set access. | Method, Graph locality paragraph | `RUN/AA/runAaBalancedCardinalityValidation.m`; neighborhood N50 report | Code path and report-backed | Keep centralized upper-bound language out of the main claim. |
| Under stable matching and connected repeated neighborhood averaging, local moments converge to the centralized equal-weight barycenter. | Structural Properties | Proposition in `main.tex` | Theory stated with explicit assumptions | Do not present as a finite-round guarantee; check proof wording before final submission. |
| Matched posterior barycenters, not label copying alone, drive the spatial tracking gain. | Results and Discussion | N50 full-vs-reference-only ablation; paired RMSE reductions/wins/sign-test p-values in Table III | Report-driven fragments plus independent local-metric verifier | Broaden scenario coverage before submission. |
| Neighborhood label-barycenter improves local E-OSPA/RMSE/CardErr relative to tuned spatial-KLA AA in the N50 validation. | Results, Table I/II, Fig. 3 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `generated/n50_evidence.json`; `generated/N50_VERIFICATION_REPORT.md` | Local metrics independently recomputed from per-trial local rows | None for current N50; broaden scenario coverage before submission. |
| Neighborhood label-barycenter is lower than the two tracked GA reference rows on the six reported N50 disagreement/tracking metrics. | Results, contextual reference table | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Report-driven contextual comparison | Keep phrasing as contextual reference rows, not paired GA-vs-AA significance evidence. |
| Network disagreement and runtime numbers are reproducible from raw per-trial artifacts. | Results, reproducibility notes | `verify_n50_evidence.py`; `N50_VERIFICATION_REPORT.md` | Independently recomputed | None for network/runtime; keep source hash in manifest. |
| Runtime overhead is mainly due to repeated assignment/moment-barycenter operations, not hidden global communication. | Runtime table, complexity paragraph | Runtime log and graph-locality method text | Independently recomputed runtime | Add a short scaling note if broader N or topology experiments are added. |
| Absolute runtime seconds are prototype-specific; the paper-facing runtime comparison is the relative cost within the paired Octave validation. | Experimental Setup, Runtime table | Local environment check: GNU Octave 11.1.0 on Apple M4, 16 GB memory | Manuscript-explicit and environment-backed | Keep absolute runtime claims modest; do not present them as hardware-independent benchmarks. |
| Each experiment arm isolates a specific mechanism rather than serving as an unconstrained tuning comparison. | Experimental Setup | Claim-to-arm mapping in `main.tex`; N50 full/reference-only/baseline evidence | Manuscript-explicit and report-driven | Maintain this mapping when adding held-out or recursive-online experiments. |

## TAES Compliance Gates

| Gate | Current state | Required before submission |
| --- | --- | --- |
| Official template | `IEEEtaes.cls` and `IEEEtaes.bst` are used by `main.tex`; `./build.sh` compiles the manuscript. | Rebuild after final edits; render pages and inspect figures/tables. |
| Template and author-guideline archive | `generated/SUBMISSION_READINESS_REPORT.md` now checks the local TAES requirements document, official template zip, and regular-paper template source as required artifacts. | Recheck online TAES/AESS pages immediately before final submission. |
| Manuscript type | Regular Paper is the selected target. | Submit as `Regular Paper`; technical area `Target Tracking and Multi-Sensor Systems`. |
| Page budget | Current manuscript is within a plausible Regular Paper range, but final printed page count is not guaranteed. | Keep estimated TAES pages near or below 10, or accept overlength charges. |
| Title/abstract/keywords | The readiness checker now verifies that the title/abstract avoid `new`/`novel`, the abstract is a single paragraph without citation/footnote/display equation, and keywords are alphabetized. | Re-run `./build.sh` after final title/abstract edits. |
| Author metadata | Bracketed author/funding/repository placeholders remain in submission-style prose. | Fill real author list, affiliations, ORCID, funding, acknowledgments, corresponding author metadata. |
| AI disclosure | Provisional Codex/OpenAI disclosure exists in Acknowledgment. | Recheck current IEEE/AESS wording and decide final disclosure scope. |
| Citations | Core bibliography has been DOI/source checked during draft construction. | Final bibliography scan for missing DOIs, malformed entries, uncited entries, and unsupported claims. |
| Preprint/reuse | No final decision recorded. | Decide whether to post preprint and prepare IEEE-compliant preprint notice if needed. |
| Submission files | PDF builds locally; source bundle exists in manuscript directory. | Prepare clean source zip, PDF, cover metadata, and any supplementary material. |
| Machine-checkable readiness | `generated/SUBMISSION_READINESS_REPORT.md` is written by `./build.sh`; current status is `draft_with_pending_gates`. | Close the listed pending gates before treating the draft as submission-ready. |

## Evidence Gates

| Gate | Status | Evidence | Next action |
| --- | --- | --- | --- |
| Report-driven tables/figure fragments | Passed | `extract_n50_evidence.py`; `generated/N50_EVIDENCE_MANIFEST.md` | Keep generated fragments read-only by convention. |
| Report-driven GA reference rows | Passed | `extract_reference_baselines.py`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Maintain the contextual-comparison caveat in the manuscript. |
| Independent network disagreement verifier | Passed | `verify_n50_evidence.py`; `generated/N50_VERIFICATION_REPORT.md` | Maintain hash check against source report. |
| Independent runtime verifier | Passed | `verify_n50_evidence.py`; trial log parsing | Maintain relative-cost check after source report swap. |
| Independent local metric verifier | Passed | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`; `generated/N50_VERIFICATION_REPORT.md` | Maintain `evidence_sources.json` as the single report/log source manifest. |
| Held-out scenario evidence | Partial sanity plus active N50 run | `generated/HELDOUT_SANITY_MANIFEST.md` from tracked N5 base-seed-11 report; active log `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log` | Wait for `AA_TAES_HELDOUT_N50_REPORT=...`, inspect the report, then decide whether to promote the evidence into the manuscript or supplementary material. |
| PDF visual QA | Passed for current checkpoint | `./build.sh`; rendered checks of the title/abstract page, method pages, results page, and final reference page after the contribution-sentence layout fix | Re-render final `main.pdf` after every manuscript-affecting checkpoint. |
| Submission readiness checker | Passed for mechanical gates | `generated/SUBMISSION_READINESS_REPORT.md`; `generated/submission_readiness.json` | Current pending gate is submission metadata placeholders; held-out evidence remains a warning because only N5 sanity exists. |

## Immediate Execution Order

1. Monitor `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log` until the held-out N50 report path is printed.
2. If the held-out result supports the mechanism claim, add the evidence path to the manuscript evidence pipeline and decide whether it belongs in the main paper, a compact robustness paragraph, or supplementary material.
3. Rebuild `main.pdf` and re-render the title/abstract page, method pages, result tables/figure page, and final reference page after any held-out evidence edit.
4. Replace author/funding/repository placeholders once the real submission metadata is available.
