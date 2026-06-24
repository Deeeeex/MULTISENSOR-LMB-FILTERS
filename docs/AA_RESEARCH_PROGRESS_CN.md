# AA Fusion 大目标进度

最后更新: 2026-06-24 11:25 CST

## 当前结论

当前最有希望的方向不是继续搜索 `existenceThreshold`、support count 或 bridge prior，而是把 AA 的剩余问题重写成跨 local filters 的 label canonicalization 与 posterior barycenter 问题。

论文写作准备包已整理到 `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md`，包含 paper-ready positioning、method outline、理论推导、N50 实验表、ablation 解释、图表计划和 claim 边界。

TAES 投稿源文件首版已建立到 `docs/paper/taes/manuscript/`，当前可用 `./build.sh` 生成 `main.pdf`。这一版已经把 title/abstract/intro/method/theory/experiment/results/limitations/conclusion 和 DOI 核验过的核心引用落入真实 TAES template；方法图和 N50 reduction 图已经过矢量化 polish。但还不是 submission-ready: 作者信息仍是占位，证据仍集中在 N50 tiered packet-loss 场景，还缺更广泛场景验证。

第二阶段稿件增强已开始: `main.tex` 现在补入了 DOI 程序化核验过的近期 AA/RFS/LMB fusion 文献，Related Work 已按方法线重写；方法部分增加了 graph-local operator 伪代码和 reference-label invariance 性质；实验设置已经明确三臂设计对应的可证伪 claim；结果部分从均值表扩展为均值、95% CI、paired reduction、wins、sign-test evidence、runtime 和 PDF 内可见的 N50 reduction 图。

第三阶段证据链增强已开始: `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py` 会从 tracked N50 validation report 自动生成 `generated/n50_*` LaTeX 片段、CSV、JSON 和 `N50_EVIDENCE_MANIFEST.md`。`main.tex` 的主结果表、paired reduction/sign-test 表、runtime 表和 N50 reduction 图现在由报告解析结果驱动，不再手工维护核心实验数字。Table III 已直接显示 compact sign-test p-values，使 full barycenter 的 RMSE 支持与 reference-only RMSE 的非显著结果在 PDF 中可见。

第三阶段 verifier 已闭合: `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py` 会从 per-trial Markdown network table 独立复算 network disagreement 的均值/CI/paired reductions/wins/sign-test p，从 trial log 独立复算 runtime mean/std/relative cost，并从新 N50 report 的 per-trial local tracking rows 独立复算 local E-OSPA/RMSE/CardErr；输出 `generated/N50_VERIFICATION_REPORT.md` 和 `generated/n50_verification.json`。当前 `evidence_sources.json` 已指向完成的 TAES N50 local-verifier report/log。

第四阶段稿件质量增强已开始: 方法图已改为 build 生成的原生 LaTeX 矢量 fragment 并通过 PDF 渲染检查；N50 reduction 图已从拥挤单栏图改为 full-width report-driven 矢量图；`main.tex` 已加入 graph-locality/complexity 说明，并补入 stable-matching consensus limit 命题，把 graph-local moment iteration 与 centralized equal-weight moment barycenter 的条件性关系写清楚。当前又强化了摘要和 Introduction 的审稿故事: 摘要现在从 packet-loss 下的 component-correspondence failure 开始；Intro 明确 AA/KLA 权重解决“信谁、信多少”，但不解决跨 local LMB posterior 的 Bernoulli component correspondence；因此本文的问题被明确表述为 label canonicalization + matched posterior barycenter，而不是 scalar weight search。

当前稿件又补入一个 `Weighting is not matching` 结构命题: 通过两传感器、两目标的 label-swap 反例说明，即便使用 target-wise scalar weights，正权重融合仍会把错误对应的物理目标拉到中间；端点权重只能丢弃一个传感器，不能推断跨传感器 label permutation。Introduction 的 contribution paragraph 也已改为四点式 paper-facing 表述，使核心 claim、operator、理论边界和 50-trial/ablation 证据链更直接地对齐。

当前稿件又补入 mechanism-isolation protocol 表: Experimental Setup 现在明确 tuned spatial-KLA AA、neighborhood reference-only 和 neighborhood label-barycenter 三个 arm 分别检验 corrected scalar-weight routing、label-set canonicalization alone、matched posterior barycenter 三个 claim，并声明 50 个 paired trials 内参数固定、measurement 和 packet-loss realization 成对复用。正文进一步写明 neighborhood operator 在 N50 前固定使用 `H=3`、复用 tuned AA baseline 的 existence threshold、不对 projection cutoff、barycenter weights 或 trial-specific label rules 做逐场景搜索，held-out base-seed run 只作为 robustness check。这样可以回应“不要变成针对当前数据搜索式调参”的方法论要求。

当前稿件又完成了一次 implementation-alignment 修订: `main.tex` 现在明确 proposed layer 是 applied to active output tracks 的 label-and-moment projection，不替代上游 AA Bernoulli existence consumer；stable-matching consensus limit 命题也从 moment vector 中移除了 existence probability，把 existence convexity 单独表述为 upstream AA existence update 的性质。这个修订对齐了 `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` 的实际代码路径，避免把方法 claim 写得比实现更强。`check_submission_readiness.py` 现在也新增 `implementation-alignment wording` gate，后续若正文删掉这些边界会在 readiness report 中暴露为 warning。

当前稿件进一步补入 runtime reproducibility 说明: Experimental Setup 现在写明 runtime 是 GNU Octave 11.1.0 在 Apple M4 / 16 GB 本地工作站上的 wall-clock filter runtime，绝对秒数只表征当前 Octave/MATLAB-compatible prototype，paper-facing runtime claim 应主要解读为 paired validation 内的相对实现开销。

投稿 readiness 审计已新增到 `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`。该文档把 TAES compliance gates、paper-facing claims、当前证据级别和剩余关闭条件逐项列出；当前 local-metric independent-verifier gate 已关闭，剩余重点是 held-out 场景和投稿元数据。

投稿 readiness 现在也有机器检查产物: `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md` 和 `generated/submission_readiness.json` 会在 `./build.sh` 后自动更新。当前 overall status 是 `draft_with_pending_gates`，机械 gate 已通过；readiness checker 已新增 TAES-specific 机械检查，包括投稿要求文档与官方模板归档、标题/摘要避免 `new`/`novel`、摘要无 citation/footnote/display equation、关键词按字母序。剩余 pending gate 是作者/基金/repository 元数据占位符；held-out scenario evidence 仍是 warning，因为当前只有 N5 sanity，而 paper-grade held-out N50 正在运行。当前 build pipeline 已预置 `heldout_n50_report` 可选证据源，长跑完成后只需把 report path 接入 manifest 即可生成 `HELDOUT_N50_MANIFEST.md`、`generated/heldout_n50_section.tex`，并让 readiness checker 识别 paper-grade held-out evidence；`main.tex` 已用 `\IfFileExists` 预留正文 hook，未生成 N50 fragment 时当前 PDF 不变化。

投稿包现在补入 `docs/paper/taes/manuscript/COVER_LETTER_AND_METADATA_DRAFT.md`: 包含 TAES cover letter 草稿、Regular Paper / Target Tracking and Multi-Sensor Systems portal metadata、原创性声明、AI assistance disclosure、ORCID/funding/repository/preprint/conflict placeholders 和最终替换清单。`check_submission_readiness.py` 与 source bundle 生成脚本已纳入该文件，避免最终投稿时只准备 PDF/source 而漏掉 portal/cover metadata。

当前已补入一个 tracked held-out sanity evidence 包: `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md` 解析 baseSeed=11、N=5 的同三臂 neighborhood report。它显示 full label-barycenter 在 seed-11 小样本上仍降低 Network OSPA、local E-OSPA 和 RMSE，且 reference-only 的 RMSE 为负收益，继续支持“barycenter 不只是复制 label reference”的解释。这个证据只关闭“无跨 seed 迹象”的弱问题，不替代后续 N50 或 packet-loss-family held-out validation。

稿件元数据也已做 submission-style polish: `main.tex` 中的作者、基金和 repository 信息保留为 bracketed placeholders，但移除了“draft version / before submission / will be provided”这类内部状态口吻。`./build.sh` 已重新生成 `main.pdf`，并用 ImageMagick/Poppler 渲染抽查首页、方法图页、结果图表页、致谢/参考文献页和末页；当前无明显溢出、重叠或图表不可读问题。本轮又专门压缩 Introduction 的贡献句，解决了贡献段在首页末尾跨页断开的排版问题，并重新渲染确认首页、方法页、结果页和参考文献末页可读。当前 build pipeline 还会生成 deterministic TAES source bundle 到 `docs/paper/taes/manuscript/tmp/submission_bundle/taes_label_barycenter_submission_source.zip`，并写出 `generated/SUBMISSION_BUNDLE_MANIFEST.md`；该 zip 已在 `/tmp/taes_submission_bundle_check` 独立解压并用 Tectonic 编译通过。

当前稿件又补入了 GA reference rows 的可追溯证据链: `scripts/extract_reference_baselines.py` 从 tracked AA/GA N50 reports 自动生成 `generated/reference_baseline_rows.tex`、`generated/reference_baseline_evidence.json` 和 `generated/REFERENCE_BASELINE_MANIFEST.md`。正文把这些 GA rows 明确写成 contextual reference baselines，不进入 paired AA sign-test；在相同 base seed、trial seeds 和 tiered packet-loss profile 下，neighborhood label-barycenter row 在六个 disagreement/tracking metrics 上均低于两个 GA reference rows。

已经验证的两个原型是:

1. `Cross-local label-consensus spatial-KLA AA`: centralized/output-level upper-bound prototype。
2. `Neighborhood label-barycenter spatial-KLA AA`: 只使用 `neighborMap{s}` 的 output-level neighborhood iterative prototype。

核心流程:

```text
local outputs -> median-cardinality medoid reference label set
              -> Hungarian matching
              -> matched posterior barycenter
              -> shared labels/states written back to every local output
```

重要边界: centralized 版本会按构造让 consensus OSPA/Loc/Card 归零，不能单独作为论文 claim；neighborhood 版本不再全局读取 label set，N50 上 consensus 指标不为零但大幅优于 tuned baseline。两者仍是 output-level projection，不是递归滤波内部的最终 online AA 算法。

## Goal 状态

| 目标项 | 状态 | 证据 | 备注 |
| --- | --- | --- | --- |
| 开 AA 分支并修复 AA consumer-side 权重/label 问题 | 已完成并 push | branch `codex/aa-target-wise-fix`；commits `c3d26a5`, `d12fb92`, `7de3c0c` | 当前分支已推到 origin。 |
| 避免搜索式调参，转向方法层面设计 | 已执行 | `docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md` | 文档明确否定阈值搜索，转向 label-consensus/barycenter。 |
| N1 sanity: 新方法必须强于 tuned AA | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_121119.md` | local E-OSPA/RMSE/CardErr: tuned `2.0289/4.1450/0.06625` -> projection `1.6343/3.7789/0.06000`。 |
| N5 sanity: 不是单 seed 偶然 | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_121330.md` | local E-OSPA/RMSE/CardErr: tuned `2.032799/3.588145/0.086250` -> projection `1.660378/3.290166/0.068000`。 |
| N50 validation: AA 新方法优于 GA 两个 mode | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_122813.md`; `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md` | projection local E-OSPA/RMSE/CardErr `1.645476/3.343985/0.071200`，低于 GA structure-aware `2.213284/4.228361/0.203975` 和 GA FID-FIA `2.179948/4.695098/0.151025`。 |
| N50 ablation: 证明设计组件合理 | 已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_145052.md`; `RUN/AA/AA_CROSS_LOCAL_LABEL_CONSENSUS_ABLATION_N50_SEED1_20260622_145045.log` | full barycenter local E-OSPA/RMSE/CardErr `1.645476/3.343985/0.071200`，reference-only `1.781249/3.463664/0.071200`。 |
| 算法设计与理论证明 | 已建立，online 化第一步通过 N50，本轮增强 | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md`; `docs/paper/taes/manuscript/main.tex`; `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` | 已整理 output-level projection 的可证明性质、reference-only ablation 解释、online/distributed label-barycenter AA 的收敛条件和边界；TAES 正文已补 stable-matching consensus limit，并新增 `Weighting is not matching` label-swap 反例命题；本轮又明确 projection 是 active-track label/moment 层，existence convexity 属于上游 AA existence consumer；neighborhood iterative prototype 已完成 N50。 |
| Neighborhood online 化 sanity/validation | N1/N5/N50 已通过 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_171542.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md` | N50 consensus OSPA/Loc/Card `0.309818/0.216372/0.021200`，local E-OSPA/RMSE/CardErr `1.681483/3.449035/0.077200`，均低于两个 GA reference；runtime 约 `1.644x` tuned，local metrics 已从 per-trial rows 独立复算。 |
| Paper-writing ready 技术包 | 已完成 | `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md` | 已把方法、理论、实验结果、ablation、图表计划和 claim 边界整理成可拆入 manuscript 的中文材料。 |
| TAES manuscript first draft | 已完成首版，可编译 PDF | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/references.bib` | 使用官方 `IEEEtaes.cls/.bst`，`tectonic` 编译通过，当前 `main.pdf` 为 8 页；方法图已换成原生 LaTeX 矢量图；只剩 underfull/internal-consistency 类非阻塞 warning。 |
| TAES manuscript evidence-chain pass | 进行中，本轮已增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/references.bib`; `docs/paper/taes/manuscript/evidence_sources.json`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md` | 已补近期 DOI 核验引用、Related Work 方法线、operator 伪代码、reference-label invariance、paired CI/wins/sign-test p 表、runtime 表和 full-width N50 reduction 图；Table III 现在直接显示 full vs reference-only 的 sign-test evidence；仍需更广 scenario 和最终语言压缩。 |
| TAES manuscript reproducible-results pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py`; `docs/paper/taes/manuscript/generated/N50_EVIDENCE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/n50_evidence.json` | N50 paper-facing tables/figure fragments now regenerate from the tracked validation report during `./build.sh`; manifest records report SHA256 and key paper-facing checks. |
| TAES manuscript GA reference evidence pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_reference_baselines.py`; `docs/paper/taes/manuscript/generated/REFERENCE_BASELINE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/reference_baseline_rows.tex` | Contextual GA reference rows now regenerate from tracked AA/GA N50 reports during `./build.sh`; manuscript caveat says these rows are reference baselines, not paired AA sign-test inputs. |
| TAES manuscript held-out sanity pass | sanity checkpoint 已完成，paper-grade N50 正在运行 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md`; `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log`; `docs/paper/taes/manuscript/scripts/extract_heldout_sanity_evidence.py` | Build now records a tracked N5 base-seed-11 sanity check: Network OSPA full `0.305452` vs tuned `1.702915`, local E-OSPA full `1.691451` vs tuned `2.032799`, RMSE full `3.450998` vs tuned `3.588145`; reference-only RMSE reduction is `-3.59%`. Paper-grade N50/baseSeed=11 run has started with PID `92443`; the extractor and `main.tex` hook are now prewired so adding `heldout_n50_report` will generate a compact held-out table/paragraph automatically. |
| TAES manuscript independent-verifier pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py`; `docs/paper/taes/manuscript/evidence_sources.json`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log` | Independent verifier now recomputes network disagreement from per-trial report rows, runtime from trial log, and local E-OSPA/RMSE/CardErr from per-trial local rows; `generated/N50_VERIFICATION_REPORT.md` confirms all three paths. |
| TAES manuscript polish/complexity pass | 进行中，本轮增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/generated/method_pipeline.tex`; `docs/paper/taes/manuscript/generated/n50_reduction_bars.tex` | 已补 graph-locality/complexity paragraph，明确 Hungarian matching 是 runtime overhead 的主要来源；Introduction 已强化 component-correspondence failure framing，并改成四点 contribution paragraph；Figure 1 已改为 `scripts/render_figures.py` 生成的 LaTeX fragment；Structural Properties 已补 stable-matching consensus limit 和 `Weighting is not matching` label-swap 反例；本轮又把 stable-matching consensus limit 收紧为 matched moment coordinates，并把 existence convexity 明确归为 upstream AA existence update；Discussion/Conclusion/Acknowledgment 已避免使用“下一步/投稿前还需”这类内部状态口吻；N50 reduction 图已改为 full-width 矢量图并通过 PDF 渲染检查；Experimental Setup 已补 paired statistical protocol 和 mechanism-isolation protocol 表，把 baseline/proposed/reference-only 三臂分别映射到 scalar-weight consumer、full label-barycenter、label-set-only ablation 三个 claim，并声明固定参数/成对 packet-loss realization；本轮又补入 fixed-design/no per-scenario search 说明，明确 held-out base-seed run 不用于选参；本轮又补入 Octave/Apple M4 runtime 环境和绝对秒数解释边界；当前稿件元数据以 submission-style placeholders 呈现；使用 `placeins` 的 `\FloatBarrier` 防止结果浮动跨入 Discussion。 |
| TAES submission-readiness checklist | 进行中，本轮增强 | `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md`; `docs/paper/taes/manuscript/README.md`; `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`; `docs/paper/taes/manuscript/COVER_LETTER_AND_METADATA_DRAFT.md`; `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md`; `docs/paper/taes/manuscript/generated/SUBMISSION_BUNDLE_MANIFEST.md`; `docs/paper/taes/manuscript/scripts/check_submission_readiness.py` | 官方 template 和当前 manuscript 均已用 Tectonic 编译验证；稿件已加入 provisional AI-assistance disclosure；build 后会自动生成 readiness snapshot 和 clean source bundle。当前机械 gate 通过，且 checker 已覆盖模板归档、标题/摘要/关键词、clean source bundle、implementation-alignment wording、cover letter/portal metadata draft 的 TAES-specific/claim-boundary 形式要求；N50 local independent verifier 已通过；N5 held-out sanity 已存在但仍是 warning，剩余为 paper-grade held-out、作者/基金/OA/preprint 等投稿表单信息确认。 |
| 文档维护 | 已建立，持续维护 | 本文件；`docs/AA_LABEL_UNCERTAINTY_AWARE_FUSION_RULE_CN.md`; `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` | 当前 checkpoint 已回填 N50 validation、N50 ablation、recommendation、theory boundary 和 neighborhood N50。 |

## 已完成的负结果

这些结果帮助排除“针对当前数据调参”的路径:

| 方向 | 结果 | 结论 |
| --- | --- | --- |
| full label-uncertainty rule | N1 未通过 | support-mass existence tempering 过强，导致 cardinality collapse。 |
| spatial-overlap only | N1 未通过 | 没有改善 OSPA/Loc。 |
| covariance-inflation only | N1 未通过 | 更干净但不改善 Loc/RMSE，runtime 上升。 |
| naive lifecycle split | N1 未通过 | 保留低置信 label 导致 clutter/runtime 上升。 |
| mature-label lifecycle | N1 未通过 | 收益太小且 Card/CardErr 变差。 |
| output-history lifecycle | N1/N5 near-neutral | 基本无副作用，但只有 `1e-4` 级收益，不值得 N50。 |
| support-consensus lifecycle | N1 未通过 | 单一 Neff survival gate 不足以修复 Loc gap。 |

## N50 ablation 结果

目的: 证明 full projection 的收益不只是“把所有节点复制成同一个 medoid output”，而是来自 matched posterior barycenter。

| Arm | 含义 | 预期判据 |
| --- | --- | --- |
| `Cross-local label-consensus spatial-KLA AA` | reference label set + Hungarian matching + posterior barycenter | 应在 local E-OSPA/RMSE 上优于 reference-only。 |
| `Reference-only label-consensus spatial-KLA AA` | 只复制 medoid reference output，不做 barycenter averaging | 若弱于 full projection，说明 barycenter 是必要组件。 |

N1/N5/N50 ablation 都支持该假设:

- N1: full `1.6343/3.7789/0.060000` vs reference-only `1.8111/3.9361/0.060000`。
- N5: full `1.6604/3.2902/0.068000` vs reference-only `1.7976/3.4685/0.068000`。
- N50: full `1.645476/3.343985/0.071200` vs reference-only `1.781249/3.463664/0.071200`。
- N50 paired deltas show reference-only is worse by `8.25%` E-OSPA and `3.58%` RMSE; CardErr is unchanged.

Neighborhood iterative prototype 的 N50 ablation 也支持该假设:

- tuned baseline local `2.029641/3.682880/0.088000`。
- neighborhood full `1.681483/3.449035/0.077200`。
- neighborhood reference-only `1.911286/3.662955/0.077200`。
- full 相对 tuned 的 paired local reductions 为 `17.15%` E-OSPA、`6.35%` RMSE、`12.27%` CardErr；wins 为 `50/50`、`48/50`、`46/50`。
- reference-only 的 RMSE reduction 只有 `0.54%`，wins `27/50`，说明 state barycenter 是空间性能收益的必要组件。
- TAES Table III 现在显式显示 sign-test p-values: full RMSE 为 `p<10^-3`，reference-only RMSE 为 `p=0.672`，使“label copy 不足以解释空间收益”的统计边界在正文表格中可直接审阅。

## 剩余风险

- 当前最佳 N50 结果仍来自 output-level projection；neighborhood iterative prototype 已通过 N50，但还不是递归滤波内部 online method。
- consensus 指标归零是构造结果；paper-facing claim 必须依赖 local metrics、GA reference 对照和 ablation。
- Independent verifier 已经覆盖 network disagreement、runtime 和 local E-OSPA/RMSE/CardErr；当前剩余证据风险主要是场景覆盖仍偏集中。
- 当前 ablation 证明了 barycenter 组件有用，理论文档也给出稳定 matching 下 online moment-consensus 收敛到 centralized moment barycenter 的条件；当前实现是 output-level neighborhood iterative prototype，不是递归滤波内部的最终 online method。
- TAES 首稿已经可编译，且 N50 主表/paired 表/runtime 表/N50 reduction 图已经由 report-driven generated fragments 驱动；方法图和结果图已完成一次矢量化 polish。最终稿仍需要更多 scenario/seed coverage、作者/基金/AI disclosure 最终措辞确认和人工审读。

## 已完成长跑

- 目的: 重新生成 paper-facing N50 neighborhood validation，使 report 包含 `## Per-Trial Local Tracking Metrics`，从而让 local E-OSPA/RMSE/CardErr 进入 independent-verifier path。
- 启动脚本: `RUN/AA/launchAaTaesN50LocalVerifierRerun.sh`。
- PID file: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.pid`。
- Log: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`。
- 结果:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`。
- Log: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`。
- `docs/paper/taes/manuscript/evidence_sources.json` 已接入这对 report/log；`./build.sh` 后 `generated/N50_VERIFICATION_REPORT.md` 显示 local metrics are independently recomputed from per-trial local tracking rows。

## 当前长跑

- 目的: 生成 paper-grade held-out N50 evidence，用 baseSeed=11 检查 neighborhood label-barycenter 是否跨 seed 维持相同机制收益。
- 启动脚本: `RUN/AA/launchAaTaesHeldoutN50BaseSeed11.sh`。
- PID: `92443`。
- PID file: `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.pid`。
- Log: `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log`。
- 启动状态: 2026-06-24 09:49 CST 时进程存活，log 已进入 `AA validation trial 1/50`。
- 当前状态: 2026-06-24 11:25 CST 时进程仍存活，log 已进入 `AA validation trial 32/50`。
- 查看进度:

```bash
tail -f /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS/RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log
```

完成后检查 log 中的 `AA_TAES_HELDOUT_N50_REPORT=...` 路径，确认 report 包含 per-trial local rows；若结果支持主 claim，则把路径接入 `docs/paper/taes/manuscript/evidence_sources.json`，运行 `./build.sh`，审查 `generated/HELDOUT_N50_MANIFEST.md`、`generated/heldout_n50_section.tex` 和重建后的 PDF。

## 下一步

1. 把 output-level iterative prototype 下沉到递归滤波内部的 online label message / moment consensus。
2. 给 online 版本设计新的 method-level ablation: label canonicalization only、state barycenter only、iterative local consensus。
3. 等当前 paper-grade held-out N50 完成后，接入 `heldout_n50_report`，审查自动生成的 held-out 表/段落，并决定是否保留在正文、移到 supplementary，或只作为 response-ready robustness evidence。
4. 继续压缩和润色 TAES `main.tex`: 加入更广场景后的图表，同时控制 TAES 页数。
