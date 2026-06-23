# AA Fusion 大目标进度

最后更新: 2026-06-24 02:14 CST

## 当前结论

当前最有希望的方向不是继续搜索 `existenceThreshold`、support count 或 bridge prior，而是把 AA 的剩余问题重写成跨 local filters 的 label canonicalization 与 posterior barycenter 问题。

论文写作准备包已整理到 `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md`，包含 paper-ready positioning、method outline、理论推导、N50 实验表、ablation 解释、图表计划和 claim 边界。

TAES 投稿源文件首版已建立到 `docs/paper/taes/manuscript/`，当前可用 `./build.sh` 生成 `main.pdf`。这一版已经把 title/abstract/intro/method/theory/experiment/results/limitations/conclusion 和 DOI 核验过的核心引用落入真实 TAES template；方法图和 N50 reduction 图已经过矢量化 polish。但还不是 submission-ready: 作者信息仍是占位，证据仍集中在 N50 tiered packet-loss 场景，缺最终 local-metric independent verifier 和更广泛场景验证。

第二阶段稿件增强已开始: `main.tex` 现在补入了 DOI 程序化核验过的近期 AA/RFS/LMB fusion 文献，Related Work 已按方法线重写；方法部分增加了 graph-local operator 伪代码和 reference-label invariance 性质；实验设置已经明确三臂设计对应的可证伪 claim；结果部分从均值表扩展为均值、95% CI、paired reduction、wins、sign-test evidence、runtime 和 PDF 内可见的 N50 reduction 图。

第三阶段证据链增强已开始: `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py` 会从 tracked N50 validation report 自动生成 `generated/n50_*` LaTeX 片段、CSV、JSON 和 `N50_EVIDENCE_MANIFEST.md`。`main.tex` 的主结果表、paired reduction/sign-test 表、runtime 表和 N50 reduction 图现在由报告解析结果驱动，不再手工维护核心实验数字。Table III 已直接显示 compact sign-test p-values，使 full barycenter 的 RMSE 支持与 reference-only RMSE 的非显著结果在 PDF 中可见。

第三阶段 verifier 已补入: `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py` 会从 per-trial Markdown network table 独立复算 network disagreement 的均值/CI/paired reductions/wins/sign-test p，并从 trial log 独立复算 runtime mean/std/relative cost；输出 `generated/N50_VERIFICATION_REPORT.md` 和 `generated/n50_verification.json`。本轮已把 validation runner 改为对新报告输出 per-trial local E-OSPA/RMSE/CardErr rows，并用 N1 smoke 报告验证 local trial rows 可复算 summary；边界是当前 paper-facing archived N50 report 仍缺该表，因此 N50 local tracking metrics 仍是 summary-traced。已启动 TAES N50 local-verifier rerun，完成后应把新 report/log 接入 manuscript evidence scripts。

第四阶段稿件质量增强已开始: 方法图已改为 build 生成的原生 LaTeX 矢量 fragment 并通过 PDF 渲染检查；N50 reduction 图已从拥挤单栏图改为 full-width report-driven 矢量图；`main.tex` 已加入 graph-locality/complexity 说明，并补入 stable-matching consensus limit 命题，把 graph-local moment iteration 与 centralized equal-weight moment barycenter 的条件性关系写清楚。当前又强化了摘要和 Introduction 的审稿故事: 摘要现在从 packet-loss 下的 component-correspondence failure 开始；Intro 明确 AA/KLA 权重解决“信谁、信多少”，但不解决跨 local LMB posterior 的 Bernoulli component correspondence；因此本文的问题被明确表述为 label canonicalization + matched posterior barycenter，而不是 scalar weight search。

投稿 readiness 审计已新增到 `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`。该文档把 TAES compliance gates、paper-facing claims、当前证据级别和剩余关闭条件逐项列出，后续 N50 local-verifier 完成后可直接按该表更新 source report/log、重建 PDF 并关闭 local-metric independent-verifier gate。

投稿 readiness 现在也有机器检查产物: `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md` 和 `generated/submission_readiness.json` 会在 `./build.sh` 后自动更新。当前 overall status 是 `draft_with_pending_gates`，机械 gate 已通过；readiness checker 已新增 TAES-specific 机械检查，包括投稿要求文档与官方模板归档、标题/摘要避免 `new`/`novel`、摘要无 citation/footnote/display equation、关键词按字母序。剩余 pending gate 明确为作者/基金/repository 元数据占位符、N50 local-metric independent verifier 和 held-out scenario evidence。

当前已补入一个 tracked held-out sanity evidence 包: `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md` 解析 baseSeed=11、N=5 的同三臂 neighborhood report。它显示 full label-barycenter 在 seed-11 小样本上仍降低 Network OSPA、local E-OSPA 和 RMSE，且 reference-only 的 RMSE 为负收益，继续支持“barycenter 不只是复制 label reference”的解释。这个证据只关闭“无跨 seed 迹象”的弱问题，不替代后续 N50 或 packet-loss-family held-out validation。

稿件元数据也已做 submission-style polish: `main.tex` 中的作者、基金和 repository 信息保留为 bracketed placeholders，但移除了“draft version / before submission / will be provided”这类内部状态口吻。`./build.sh` 已重新生成 `main.pdf`，并用 ImageMagick 渲染抽查首页、方法图页、结果图表页、致谢/参考文献页和末页；当前无明显溢出、重叠或图表不可读问题。

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
| 算法设计与理论证明 | 已建立，online 化第一步通过 N50 | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md`; `docs/paper/taes/manuscript/main.tex` | 已整理 output-level projection 的可证明性质、reference-only ablation 解释、online/distributed label-barycenter AA 的收敛条件和边界；TAES 正文已补 stable-matching consensus limit；neighborhood iterative prototype 已完成 N50。 |
| Neighborhood online 化 sanity/validation | N1/N5/N50 已通过 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_171542.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` | N50 consensus OSPA/Loc/Card `0.309818/0.216372/0.021200`，local E-OSPA/RMSE/CardErr `1.681483/3.449035/0.077200`，均低于两个 GA reference；runtime 约 `1.647x` tuned。 |
| Paper-writing ready 技术包 | 已完成 | `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md` | 已把方法、理论、实验结果、ablation、图表计划和 claim 边界整理成可拆入 manuscript 的中文材料。 |
| TAES manuscript first draft | 已完成首版，可编译 PDF | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/references.bib` | 使用官方 `IEEEtaes.cls/.bst`，`tectonic` 编译通过并渲染检查 6 页 PDF；方法图已换成原生 LaTeX 矢量图；只剩标题页 underfull 类非阻塞警告。 |
| TAES manuscript evidence-chain pass | 进行中，本轮已增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/references.bib`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` | 已补近期 DOI 核验引用、Related Work 方法线、operator 伪代码、reference-label invariance、paired CI/wins/sign-test p 表、runtime 表和 full-width N50 reduction 图；Table III 现在直接显示 full vs reference-only 的 sign-test evidence；仍需更广 scenario 和最终语言压缩。 |
| TAES manuscript reproducible-results pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py`; `docs/paper/taes/manuscript/generated/N50_EVIDENCE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/n50_evidence.json` | N50 paper-facing tables/figure fragments now regenerate from the tracked validation report during `./build.sh`; manifest records report SHA256 and key paper-facing checks. |
| TAES manuscript GA reference evidence pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_reference_baselines.py`; `docs/paper/taes/manuscript/generated/REFERENCE_BASELINE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/reference_baseline_rows.tex` | Contextual GA reference rows now regenerate from tracked AA/GA N50 reports during `./build.sh`; manuscript caveat says these rows are reference baselines, not paired AA sign-test inputs. |
| TAES manuscript held-out sanity pass | 已完成 sanity checkpoint，仍需 paper-grade held-out | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md` | Build now records a tracked N5 base-seed-11 sanity check: Network OSPA full `0.305452` vs tuned `1.702915`, local E-OSPA full `1.691451` vs tuned `2.032799`, RMSE full `3.450998` vs tuned `3.588145`; reference-only RMSE reduction is `-3.59%`. This remains warning-level evidence, not final held-out validation. |
| TAES manuscript independent-verifier pass | 部分完成，N50 local rerun 正在运行 | `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py`; `RUN/AA/launchAaTaesN50LocalVerifierRerun.sh`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log` | Independent verifier recomputes network disagreement from per-trial report rows and runtime from trial log; new validation reports now expose per-trial local metrics. 当前已启动 50-trial rerun，完成后把新 report/log 接入 `extract_n50_evidence.py` 和 `verify_n50_evidence.py`，再重建 PDF。 |
| TAES manuscript polish/complexity pass | 进行中，本轮增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/generated/method_pipeline.tex`; `docs/paper/taes/manuscript/generated/n50_reduction_bars.tex` | 已补 graph-locality/complexity paragraph，明确 Hungarian matching 是 runtime overhead 的主要来源；Introduction 已强化 component-correspondence failure framing；Figure 1 已改为 `scripts/render_figures.py` 生成的 LaTeX fragment；Structural Properties 已补 stable-matching consensus limit；Discussion/Conclusion/Acknowledgment 已避免使用“下一步/投稿前还需”这类内部状态口吻；N50 reduction 图已改为 full-width 矢量图并通过 PDF 渲染检查；Experimental Setup 已补 paired statistical protocol，并把 baseline/proposed/reference-only 三臂分别映射到 scalar-weight consumer、full label-barycenter、label-set-only ablation 三个 claim；当前稿件元数据以 submission-style placeholders 呈现；使用 `placeins` 的 `\FloatBarrier` 防止结果浮动跨入 Discussion。 |
| TAES submission-readiness checklist | 进行中，本轮增强 | `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md`; `docs/paper/taes/manuscript/README.md`; `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`; `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md` | 官方 template 和当前 manuscript 均已用 Tectonic 编译验证；稿件已加入 provisional AI-assistance disclosure；build 后会自动生成 readiness snapshot。当前机械 gate 通过，且 checker 已覆盖模板归档、标题/摘要/关键词的 TAES-specific 形式要求；N5 held-out sanity 已存在但仍是 warning，剩余为 N50 local independent verifier、paper-grade held-out、作者/基金/OA/preprint 等投稿表单信息确认。 |
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
- 已有 partial independent verifier: network disagreement 和 runtime 可从 per-trial artifacts 独立复算；validation runner 已补 per-trial local raw rows，TAES N50 rerun 正在运行。运行完成前，当前 paper-facing N50 local tracking metrics 仍是 summary-traced。
- 当前 ablation 证明了 barycenter 组件有用，理论文档也给出稳定 matching 下 online moment-consensus 收敛到 centralized moment barycenter 的条件；当前实现是 output-level neighborhood iterative prototype，不是递归滤波内部的最终 online method。
- TAES 首稿已经可编译，且 N50 主表/paired 表/runtime 表/N50 reduction 图已经由 report-driven generated fragments 驱动；方法图和结果图已完成一次矢量化 polish。最终稿仍需要更多 scenario/seed coverage、作者/基金/AI disclosure 最终措辞确认、N50 local independent verification 接入和人工审读。

## 当前长跑

- 目的: 重新生成 paper-facing N50 neighborhood validation，使 report 包含 `## Per-Trial Local Tracking Metrics`，从而让 local E-OSPA/RMSE/CardErr 进入 independent-verifier path。
- 启动脚本: `RUN/AA/launchAaTaesN50LocalVerifierRerun.sh`。
- PID file: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.pid`。
- Log: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`。
- 查看进度:

```bash
tail -f /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS/RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log
```

完成后检查 log 中的 `AA_TAES_N50_LOCAL_VERIFIER_REPORT=...` 路径，确认新 report 有 per-trial local table，然后更新 `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py` 和 `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py` 的 source report/log。

## 下一步

1. 把 output-level iterative prototype 下沉到递归滤波内部的 online label message / moment consensus。
2. 给 online 版本设计新的 method-level ablation: label canonicalization only、state barycenter only、iterative local consensus。
3. 等当前 N50 local-verifier rerun 完成后，将新 report/log 接入 manuscript evidence scripts，重建 PDF，并让 verifier 复算 paper-facing local metrics；随后做一个不同 baseSeed 或不同 packet-loss family 的 held-out run。
4. 继续扩写 TAES `main.tex`: 强化 Introduction 的审稿故事，把 Figure 1 换成更高级的矢量流程图，加入更广场景/独立复跑后的图表，并压缩正文以控制 TAES 页数。
