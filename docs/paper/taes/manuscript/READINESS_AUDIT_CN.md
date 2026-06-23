# TAES 稿件 Readiness 审计

日期: 2026-06-24 00:29 CST

目标稿件: `Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication`

目标期刊: IEEE Transactions on Aerospace and Electronic Systems (TAES), Regular Paper, Technical area `Target Tracking and Multi-Sensor Systems`

## 当前判断

稿件已经进入 paper-facing draft 阶段: TAES 模板、核心方法叙事、理论性质、N50 主实验、ablation、runtime 和 disclosure skeleton 都已经落到 `main.tex` 和 `generated/` evidence fragments 中。当前还不能标记为 submission-ready，主要原因不是 LaTeX 或故事线，而是证据链还缺两类最后闭环:

1. 当前 paper-facing N50 local E-OSPA/RMSE/CardErr 仍来自 report summary trace，尚未由 per-trial local rows 独立复算。
2. 当前实证集中在 tiered packet-loss formation 场景，还缺至少一个 held-out base seed 或 packet-loss family 来证明不是单场景特化。

## Claim-to-Evidence Matrix

| Paper-facing claim | Manuscript location | Primary evidence | Verification level | Remaining gate |
| --- | --- | --- | --- | --- |
| AA/KLA scalar weights decide fusion mass/trust but do not solve Bernoulli component correspondence across local LMB posteriors. | Introduction, Related Work | AA fusion literature synthesis; method contrast in `main.tex` | Conceptual and citation-backed | Final language pass should keep this as the central motivation and avoid overclaiming all AA failures. |
| The proposed operator first canonicalizes labels by assignment and then fuses matched posterior moments. | Method, Fig. 1, Algorithm box | `scripts/render_figures.py`; `generated/method_pipeline.tex`; algorithm text in `main.tex` | Source-backed and rendered in PDF | Final PDF visual check after every figure/table edit. |
| The operator is graph-local in the neighborhood version and does not require global label-set access. | Method, Graph locality paragraph | `RUN/AA/runAaBalancedCardinalityValidation.m`; neighborhood N50 report | Code path and report-backed | Keep centralized upper-bound language out of the main claim. |
| Under stable matching and connected repeated neighborhood averaging, local moments converge to the centralized equal-weight barycenter. | Structural Properties | Proposition in `main.tex` | Theory stated with explicit assumptions | Do not present as a finite-round guarantee; check proof wording before final submission. |
| Matched posterior barycenters, not label copying alone, drive the spatial tracking gain. | Results and Discussion | N50 full-vs-reference-only ablation; paired RMSE reductions/wins/sign test | Report-driven fragments, local summary trace | Upgrade to independent local-metric recomputation after the active N50 rerun completes. |
| Neighborhood label-barycenter improves local E-OSPA/RMSE/CardErr relative to tuned spatial-KLA AA in the N50 validation. | Results, Table I/II, Fig. 3 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`; `generated/n50_evidence.json` | Report-driven; local metrics currently summary-traced | Replace source with rerun report containing per-trial local rows and rerun verifier. |
| Network disagreement and runtime numbers are reproducible from raw per-trial artifacts. | Results, reproducibility notes | `verify_n50_evidence.py`; `N50_VERIFICATION_REPORT.md` | Independently recomputed | None for network/runtime; keep source hash in manifest. |
| Runtime overhead is mainly due to repeated assignment/moment-barycenter operations, not hidden global communication. | Runtime table, complexity paragraph | Runtime log and graph-locality method text | Independently recomputed runtime | Add a short scaling note if broader N or topology experiments are added. |

## TAES Compliance Gates

| Gate | Current state | Required before submission |
| --- | --- | --- |
| Official template | `IEEEtaes.cls` and `IEEEtaes.bst` are used by `main.tex`; `./build.sh` compiles the manuscript. | Rebuild after final edits; render pages and inspect figures/tables. |
| Manuscript type | Regular Paper is the selected target. | Submit as `Regular Paper`; technical area `Target Tracking and Multi-Sensor Systems`. |
| Page budget | Current manuscript is within a plausible Regular Paper range, but final printed page count is not guaranteed. | Keep estimated TAES pages near or below 10, or accept overlength charges. |
| Author metadata | Bracketed author/funding/repository placeholders remain in submission-style prose. | Fill real author list, affiliations, ORCID, funding, acknowledgments, corresponding author metadata. |
| AI disclosure | Provisional Codex/OpenAI disclosure exists in Acknowledgment. | Recheck current IEEE/AESS wording and decide final disclosure scope. |
| Citations | Core bibliography has been DOI/source checked during draft construction. | Final bibliography scan for missing DOIs, malformed entries, uncited entries, and unsupported claims. |
| Preprint/reuse | No final decision recorded. | Decide whether to post preprint and prepare IEEE-compliant preprint notice if needed. |
| Submission files | PDF builds locally; source bundle exists in manuscript directory. | Prepare clean source zip, PDF, cover metadata, and any supplementary material. |

## Evidence Gates

| Gate | Status | Evidence | Next action |
| --- | --- | --- | --- |
| Report-driven tables/figure fragments | Passed | `extract_n50_evidence.py`; `generated/N50_EVIDENCE_MANIFEST.md` | Keep generated fragments read-only by convention. |
| Independent network disagreement verifier | Passed | `verify_n50_evidence.py`; `generated/N50_VERIFICATION_REPORT.md` | Maintain hash check against source report. |
| Independent runtime verifier | Passed | `verify_n50_evidence.py`; trial log parsing | Maintain relative-cost check after source report swap. |
| Independent local metric verifier | Pending | Active N50 rerun log `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log` | When the log prints `AA_TAES_N50_LOCAL_VERIFIER_REPORT=...`, switch scripts to that report/log and rebuild. |
| Held-out scenario evidence | Pending | No committed held-out packet-loss/base-seed report yet | Run one additional held-out validation after local verifier closes. |
| PDF visual QA | Partial | Prior rendered inspections after recent commits | Re-render final `main.pdf` after every manuscript-affecting checkpoint. |

## Immediate Execution Order

1. Let the current N50 local-verifier rerun finish; do not track the live log or pid file in git.
2. Switch `extract_n50_evidence.py` and `verify_n50_evidence.py` to the new report/log, then run `./build.sh`.
3. Confirm `generated/N50_VERIFICATION_REPORT.md` says local metrics are independently recomputed from per-trial local tracking rows.
4. Render `main.pdf` to page PNGs and inspect title/abstract, method figures, results tables, and reference page spacing.
5. Commit and push the source/report-script checkpoint.
6. Add one held-out validation report and decide whether it enters the main paper, a compact robustness paragraph, or supplementary material.
