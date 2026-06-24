# AA Fusion 大目标进度

最后更新: 2026-06-24 17:08 CST

## 当前结论

当前最有希望的方向不是继续搜索 `existenceThreshold`、support count 或 bridge prior，而是把 AA 的剩余问题重写成跨 local filters 的 label canonicalization 与 posterior barycenter 问题。

论文写作准备包已整理到 `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md`，包含 paper-ready positioning、method outline、理论推导、N50 实验表、ablation 解释、图表计划和 claim 边界。

TAES 投稿源文件首版已建立到 `docs/paper/taes/manuscript/`，当前可用 `./build.sh` 生成 `main.pdf`。这一版已经把 title/abstract/intro/method/theory/experiment/results/limitations/conclusion 和 DOI 核验过的核心引用落入真实 TAES template；方法图和 N50 reduction 图已经过矢量化 polish；baseSeed=11 的 held-out N50 robustness evidence 和 baseSeed=21 的 harsh packet-loss N50 stress evidence 也已接入正文/generated evidence/readiness gate。按“作者信息等可以先占位”的项目约定，当前目标应拆成 content-ready 与 portal-submission-ready: 前者要求内容、证据、PDF 和 source bundle gate 闭合；后者还要求作者/基金/repository/corresponding author 等投稿元数据替换完成。证据主线已覆盖主 tiered packet-loss profile、跨 seed held-out robustness 和更严 packet-loss profile；当前已开始把剩余风险转向 topology/partial-FOV/full-topology 场景族验证，避免继续围绕现有数据搜索式调参。

本轮新增 `docs/AA_GENERALIZATION_SCENARIO_PROTOCOL_CN.md` 和通用 launcher `RUN/AA/launchAaTaesScenarioFamilySmoke.sh`。`RUN/AA/runAaBalancedCardinalityValidation.m` 新增可选 `scenarioOverrides` 参数，默认保持原 4+4 formation 实验不变；显式传入时可固定切换 `neighborMapMode` 或 FOV，并把 `scenarioLabel`、topology/FOV/sensor-motion metadata 写入 report。首批预注册场景族为 `topology-ring`、`partial-fov35` 和 `full-topology`，用于检验方法泛化和边界，不允许根据结果回调 `H`、threshold、projection cutoff、barycenter weights 或 label rules。当前 `topology-ring` 已完成 N1 smoke 并已启动固定 N50，`partial-fov35` 已完成固定 N5 smoke；N50 完成前二者都不能直接当 paper-grade conclusion。

本轮又把 scenario-family evidence 接入 TAES source package: `docs/paper/taes/manuscript/scripts/extract_scenario_family_evidence.py` 会解析 `scenario_topology_ring_report`、`scenario_partial_fov35_report` 或 `scenario_full_topology_report`，生成 `generated/SCENARIO_FAMILY_MANIFEST.md`、`generated/scenario_family_evidence.json` 和 response-ready 的 `generated/scenario_family_section.tex`。readiness checker 现在会检查场景源覆盖、topology/FOV metadata、三臂 network/local means、paired CI/wins/p-values 和 evidence tier；当前 topology-ring 被标记为 `n1_smoke`，partial-FOV 被标记为 `multi_trial_smoke`，因此只作为 supplement/response/boundary planning，不作为 paper-grade robustness claim。

2026-06-24 16:54 CST 状态检查: topology-ring N50 仍在运行，PID `37456` 存活，log 已进入 trial `15/50`，尚未生成 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED31_*.md` report。因此 TAES readiness 的唯一内容 warning 仍是 scenario-family evidence tier；不能把该 N50 写成完成或 paper-grade 证据。

本轮又收紧了 Discussion/Limitations 的 claim boundary: 主文现在明确 harsh packet-loss stress 只扩展 packet-loss severity，仍保留 formation-family assumptions，不能替代 sparse topology、partial FOV、maneuvering-target 或 covariance-consistency studies。这个改动避免把 stress N50 过度解释为全场景泛化证据。

本轮进一步把这条 stress/generalization 边界纳入 `check_submission_readiness.py`: checker 现在要求 Discussion 保留 packet-loss severity、formation-family assumptions、not a substitute、sparse topology、partial FOV、maneuvering target 和 covariance-consistency 等 marker，避免后续语言压缩时丢失 claim boundary。

第二阶段稿件增强已开始: `main.tex` 现在补入了 DOI 程序化核验过的近期 AA/RFS/LMB fusion 文献，Related Work 已按方法线重写；方法部分增加了 graph-local operator 伪代码和 reference-label invariance 性质；实验设置已经明确三臂设计对应的可证伪 claim；结果部分从均值表扩展为均值、95% CI、paired reduction、wins、sign-test evidence、runtime 和 PDF 内可见的 N50 reduction 图。

第三阶段证据链增强已开始: `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py` 会从 tracked N50 validation report 自动生成 `generated/n50_*` LaTeX 片段、CSV、JSON 和 `N50_EVIDENCE_MANIFEST.md`。`main.tex` 的主结果表、paired reduction/sign-test 表、runtime 表和 N50 reduction 图现在由报告解析结果驱动，不再手工维护核心实验数字。Table IV 已直接显示 compact sign-test p-values，使 full barycenter 的 RMSE 支持与 reference-only RMSE 的非显著结果在 PDF 中可见。

第三阶段 verifier 已闭合: `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py` 会从 per-trial Markdown network table 独立复算 network disagreement 的均值/CI/paired reductions/wins/sign-test p，从 trial log 独立复算 runtime mean/std/relative cost，并从新 N50 report 的 per-trial local tracking rows 独立复算 local E-OSPA/RMSE/CardErr；输出 `generated/N50_VERIFICATION_REPORT.md` 和 `generated/n50_verification.json`。当前 `evidence_sources.json` 已指向完成的 TAES N50 local-verifier report/log。

第四阶段稿件质量增强已开始: 方法图已改为 build 生成的原生 LaTeX 矢量 fragment 并通过 PDF 渲染检查；N50 reduction 图已从拥挤单栏图改为 full-width report-driven 矢量图；`main.tex` 已加入 graph-locality/complexity 说明，并补入 stable-matching consensus limit 命题，把 graph-local moment iteration 与 centralized equal-weight moment barycenter 的条件性关系写清楚。当前又强化了摘要和 Introduction 的审稿故事: 摘要现在从 packet-loss 下的 component-correspondence failure 开始；Intro 明确 AA/KLA 权重解决“信谁、信多少”，但不解决跨 local LMB posterior 的 Bernoulli component correspondence；因此本文的问题被明确表述为 label canonicalization + matched posterior barycenter，而不是 scalar weight search。

当前稿件又补入一个 `Weighting is not matching` 结构命题: 通过两传感器、两目标的 label-swap 反例说明，即便使用 target-wise scalar weights，正权重融合仍会把错误对应的物理目标拉到中间；端点权重只能丢弃一个传感器，不能推断跨传感器 label permutation。Introduction 的 contribution paragraph 也已改为四点式 paper-facing 表述，使核心 claim、operator、理论边界和 50-trial/ablation 证据链更直接地对齐。

当前稿件又补入 mechanism-isolation protocol 表: Experimental Setup 现在明确 tuned spatial-KLA AA、neighborhood reference-only 和 neighborhood label-barycenter 三个 arm 分别检验 corrected scalar-weight routing、label-set canonicalization alone、matched posterior barycenter 三个 claim，并声明 50 个 paired trials 内参数固定、measurement 和 packet-loss realization 成对复用。正文进一步写明 neighborhood operator 在 N50 前固定使用 `H=3`、复用 tuned AA baseline 的 existence threshold、不对 projection cutoff、barycenter weights 或 trial-specific label rules 做逐场景搜索，held-out base-seed run 只作为 robustness check。这样可以回应“不要变成针对当前数据搜索式调参”的方法论要求。

当前稿件又完成了一次 implementation-alignment 修订: `main.tex` 现在明确 proposed layer 是 applied to active output tracks 的 label-and-moment projection，不替代上游 AA Bernoulli existence consumer；stable-matching consensus limit 命题也从 moment vector 中移除了 existence probability，把 existence convexity 单独表述为 upstream AA existence update 的性质。这个修订对齐了 `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` 的实际代码路径，避免把方法 claim 写得比实现更强。`check_submission_readiness.py` 现在也新增 `implementation-alignment wording` gate，后续若正文删掉这些边界会在 readiness report 中暴露为 warning。

当前稿件又做了一轮首页审稿路径 polish: 摘要末句现在直接补入 baseSeed=11 held-out robustness 结果，用 `6.64%` vs `0.82%` 的 RMSE separation 把“不是 base-seed-1 偶然”的证据前置到第一页；首页数据声明也从宽泛 IRB 口吻改为 simulated-data / no human-subject data statement，减少与工程仿真实验无关的噪声。

当前稿件进一步补入 runtime reproducibility 说明: Experimental Setup 现在写明 runtime 是 GNU Octave 11.1.0 在 Apple M4 / 16 GB 本地工作站上的 wall-clock filter runtime，绝对秒数只表征当前 Octave/MATLAB-compatible prototype，paper-facing runtime claim 应主要解读为 paired validation 内的相对实现开销。

投稿 readiness 审计已新增到 `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`。该文档把 TAES compliance gates、paper-facing claims、当前证据级别和剩余关闭条件逐项列出；当前 local-metric independent-verifier gate 与 baseSeed=11 held-out N50 gate 已关闭，剩余硬阻塞主要是投稿元数据占位符。

投稿 readiness 现在也有机器检查产物: `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md` 和 `generated/submission_readiness.json` 会在 `./build.sh` 后自动更新。当前 checker 输出两个状态: `portal_status` 保留真实投稿表单语义，仍因作者/基金/repository 元数据占位符显示 `draft_with_pending_gates`；`content_status` 则只忽略这些 metadata placeholders，用来判断论文内容、证据、引用、PDF 和 source bundle 是否已闭合。机械 gate 已通过；readiness checker 覆盖投稿要求文档与官方模板归档、标题/摘要避免 `new`/`novel`、摘要无 citation/footnote/display equation、关键词按字母序、引用 key 完整性、未引用 BibTeX 条目、DOI 字段、BibTeX DOI resolver verification、source-bundle manifest hash freshness、bundled-fragment fallback build mode 和 source-package reproducibility ledger。baseSeed=11 的 paper-grade held-out N50 已完成并接入 `heldout_n50_report`，`./build.sh` 会生成 `HELDOUT_N50_MANIFEST.md`、`generated/heldout_n50_section.tex`，并让 readiness checker 识别 paper-grade held-out evidence；该 gate 检查 baseSeed=11、trial-seed 列表、generated manifest/fragment、三臂均值指标、paired CI/wins/p-values 和 full-barycenter-vs-reference-only RMSE separation，避免弱证据被误判为 paper-grade robustness evidence。

本轮又把 citation hygiene 从“有 DOI 字段”升级为“DOI 可解析”: 新增 `docs/paper/taes/manuscript/scripts/verify_bibtex_dois.py`，`./build.sh` 会逐条通过 `doi.org/api/handles/{doi}` 校验 `references.bib`，生成 `generated/BIBTEX_DOI_VERIFICATION.md` 和 `generated/bibtex_doi_verification.json`。readiness checker 要求该报告的 BibTeX SHA-256 与当前 `references.bib` 一致，并且全部条目 resolver response 为有效，降低投稿阶段被 malformed/hallucinated citation 伤害的风险。

本轮又刷新了 TAES 官方要求文档: `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md` 现在使用 `2026-06-24 Live Source Refresh` 标记记录当前官方页面核对，并修正了此前“本机未安装 TeX/尚未本地编译验证”的 stale caveat。`check_submission_readiness.py` 新增 `TAES requirements live-source refresh` gate，检查 Atypon portal、Regular Paper、page charge、two-column/single-spaced/10-point format、AI disclosure、ORCID、preprint、technical area 和本地 Tectonic 验证等关键锚点，避免投稿要求文档与实际构建状态漂移。

投稿包现在补入 `docs/paper/taes/manuscript/COVER_LETTER_AND_METADATA_DRAFT.md`: 包含 TAES cover letter 草稿、Regular Paper / Target Tracking and Multi-Sensor Systems portal metadata、原创性声明、AI assistance disclosure、ORCID/funding/repository/preprint/conflict placeholders 和最终替换清单。`check_submission_readiness.py` 与 source bundle 生成脚本已纳入该文件，避免最终投稿时只准备 PDF/source 而漏掉 portal/cover metadata。

投稿包现在又补入 `docs/paper/taes/manuscript/SUBMISSION_PACKAGE_INDEX.md`: 该索引把最终 portal upload set、内部 QA artifacts、metadata placeholders 和 final rebuild/source-bundle verification sequence 分开列出。`check_submission_readiness.py` 会检查该索引的关键锚点，`create_submission_bundle.py` 也会把它打入 source zip，避免最终提交时 PDF、source、cover letter、portal metadata 和内部证据清单混在一起。

本轮继续把 PDF 视觉 QA 机器化: `docs/paper/taes/manuscript/scripts/render_pdf_visual_qa.py` 会在完整仓库 build 后渲染首页、方法页、主结果页、held-out/runtime 页、Discussion/Conclusion 页和参考文献末页到 `tmp/pdf_visual_qa/`，并生成 `generated/PDF_VISUAL_QA_MANIFEST.md` 与 `generated/pdf_visual_qa.json`。readiness checker 会检查这些代表页是否渲染、尺寸是否合理、页面是否非空，从而把此前手工抽查的版面 QA 固化为可复现 gate。

本轮又做了一次首页叙事 polish: abstract 现在把主 N50 写成 `50 paired trials`，把 held-out evidence 明确成独立的 `50-trial base-seed-11 run`；Introduction 不再泛泛说“starts from failure mode”，而是直接说本文针对 residual correspondence failure，并明确 proposed gain 不归因于 scalar-weight routing。贡献列表也同步加入 held-out 50-trial replication，使第一页的 what/why/evidence 路径更完整。

本轮把首页叙事也纳入机器 readiness gate: `check_submission_readiness.py` 现在检查 abstract/intro/body 是否保留 component-correspondence failure、scalar-weight boundary、residual correspondence failure、held-out 50-trial replication 和 reference-only ablation 等关键 marker，同时检查正文中是否残留 TODO/TBD/before-submission/placeholder 等内部状态口吻。这样后续语言压缩不会无意中删掉核心 story。

本轮继续把投稿前 metadata consistency 机器化: `check_submission_readiness.py` 现在会把 `COVER_LETTER_AND_METADATA_DRAFT.md` 中的 cover-letter title sentence、portal title、running head、journal、Regular Paper type、technical area、simulated-data statement 和 OpenAI Codex disclosure 与 `main.tex` 对齐检查；`SUBMISSION_PACKAGE_INDEX.md` 也明确记录 `cover letter and portal metadata source synchronization` gate，避免最终提交时 cover letter / portal form 与正文标题或 disclosure 漂移。

本轮又完成一次结果图版面 polish: `scripts/extract_n50_evidence.py` 现在生成更干净的 full-width N50 reduction bar figure，去掉右侧重复数值堆叠，改用 Full/Ref.-only legend；精确百分比与置信区间仍保留在 Table IV。当时 `./build.sh` 后 `main.pdf` 为 8 页；当前 readiness snapshot 显示加入后续 held-out/stress/readiness 内容后 PDF 为 9 页，仍低于 Regular Paper overlength charge 起点。

本轮继续补强 held-out 证据呈现: `scripts/extract_heldout_sanity_evidence.py` 现在把 baseSeed=11 N50 robustness fragment 生成为 full-width table，并在正文 PDF 中直接显示 full/reference-only reductions、absolute paired 95% CI、wins 和 sign-test p。这样 held-out 表与主 paired table 的统计证据强度对齐，而不是只给 wins 缩略值。该轮重构建时 `main.pdf` 为 8 页且 held-out/conclusion page 无表格溢出或重叠；当前综合版为 9 页。

当前开始补充更广 packet-loss stress family 的预注册验证路径: `docs/AA_STRESS_SCENARIO_VALIDATION_CN.md` 记录了 harsh-loss profile `[0.2 0.35 0.5 0.7] / [1 3 2 2]`、固定方法参数、N1 smoke 结果和 formal N50 launcher。N1 smoke 不是论文结论，但已经给出一个重要边界信号: full 方法在 harsh loss 下仍显著降低 network disagreement 并改善 E-OSPA/CardErr，但单 trial RMSE 略差于 tuned AA，因此后续 N50 需要按 stress/boundary evidence 解读，不能回头调参包装成单向正结果。

当前又把 harsh packet-loss stress family 接入 TAES build 的可选证据通道: `docs/paper/taes/manuscript/scripts/extract_stress_evidence.py` 会在 `evidence_sources.json` 含有 `stress_harsh_n50_report` 时生成 `generated/STRESS_HARSH_MANIFEST.md`、`generated/stress_harsh_evidence.json` 和 response-ready 的 `generated/stress_harsh_section.tex`；如果 key 缺失则不阻塞当前 manuscript build。这个设计让 N50 完成后可以直接沉淀证据，同时避免把 stress result 自动变成主文 claim 或新的调参目标。

当前 readiness checker 也已补入 optional harsh-stress evidence path: 未配置 `stress_harsh_n50_report` 时记录为非阻塞通过；一旦配置则检查 harsh profile、baseSeed/trial count、三臂 network/local mean metrics、paired CI/wins/p-values 和 stress manifest/fragment 是否齐全。这个 gate 只检查 protocol/coverage，不要求 stress run 每个 metric 都改善。

本轮 harsh packet-loss N50 stress validation 已完成并接入 build: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md` 使用固定 profile `[0.2 0.35 0.5 0.7] / [1 3 2 2]`、baseSeed=21、trials `22..71`，无参数回调搜索。结果显示 full 方法在更严 packet-loss family 下仍降低 network OSPA `79.57%`、local E-OSPA `17.06%`、RMSE `7.98%`、CardErr `24.33%`；reference-only RMSE reduction 为 `2.47%`。`extract_stress_evidence.py` 现在生成 `STRESS_HARSH_MANIFEST.md`、`stress_harsh_evidence.json`、response-ready `stress_harsh_section.tex` 和主文插入用 `stress_harsh_summary_sentence.tex`；`main.tex` 的 Discussion 只插入一条 concise generated stress sentence，避免把 stress 表格塞进主文。

本轮又补强了理论边界: `main.tex` 的 Structural Properties 新增 `Sufficient assignment stability` 命题，给出同 cardinality、参考目标间距为 `Delta`、邻居状态误差小于 `Delta/(m+1)` 时 Hungarian matching 返回物理对应的充分条件。该命题同时明确这不是必要条件，也不覆盖 cardinality mismatch、birth/death 和 close crossing。为避免新增理论破坏结果区版面，Results section 也已把 Table II/III/IV 与 Fig. 3 的 full-width floats 集中排列，再统一解释；重新渲染后结果页没有大块空白或图表重叠。

当前已补入一个 tracked held-out sanity evidence 包: `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md` 解析 baseSeed=11、N=5 的同三臂 neighborhood report。它显示 full label-barycenter 在 seed-11 小样本上仍降低 Network OSPA、local E-OSPA 和 RMSE，且 reference-only 的 RMSE 为负收益，继续支持“barycenter 不只是复制 label reference”的解释。这个证据只关闭“无跨 seed 迹象”的弱问题，不替代后续 N50 或 packet-loss-family held-out validation。

稿件元数据也已做 submission-style polish: `main.tex` 中的作者、基金和 repository 信息保留为 bracketed placeholders，但移除了“draft version / before submission / will be provided”这类内部状态口吻。`./build.sh` 已重新生成 `main.pdf`，并用 ImageMagick/Poppler 渲染抽查首页、方法图页、结果图表页、致谢/参考文献页和末页；当前无明显溢出、重叠或图表不可读问题。本轮又专门压缩 Introduction 的贡献句，解决了贡献段在首页末尾跨页断开的排版问题，并重新渲染确认首页、方法页、结果页和参考文献末页可读。当前 build pipeline 还会生成 deterministic TAES source bundle 到 `docs/paper/taes/manuscript/tmp/submission_bundle/taes_label_barycenter_submission_source.zip`，并写出 `generated/SUBMISSION_BUNDLE_MANIFEST.md`；该 zip 已在 `/tmp/taes_source_bundle_check.*` 独立解压，并用 `TAES_EVIDENCE_MODE=bundled ./build.sh` 编译通过。当前 source bundle 进一步纳入 `build.sh` 与 `scripts/*.py`；`build.sh` 在完整仓库中刷新 report-driven artifacts，在解压后的投稿包中若找不到 raw `RUN/` evidence 则自动使用 bundled `generated/` fragments 编译，readiness checker 也会检查 manifest hash freshness 和 fallback build mode。

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
| 算法设计与理论证明 | 已建立，online 化第一步通过 N50，本轮增强 | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md`; `docs/paper/taes/manuscript/main.tex`; `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` | 已整理 output-level projection 的可证明性质、reference-only ablation 解释、online/distributed label-barycenter AA 的收敛条件和边界；TAES 正文已补 stable-matching consensus limit，并新增 `Weighting is not matching` label-swap 反例命题；本轮又明确 projection 是 active-track label/moment 层，existence convexity 属于上游 AA existence consumer；本轮新增 `Sufficient assignment stability` 命题，把 Hungarian matching 正确性的 separation/error 充分条件写清楚；neighborhood iterative prototype 已完成 N50。 |
| Neighborhood online 化 sanity/validation | N1/N5/N50 已通过 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED1_20260622_171542.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md` | N50 consensus OSPA/Loc/Card `0.309818/0.216372/0.021200`，local E-OSPA/RMSE/CardErr `1.681483/3.449035/0.077200`，均低于两个 GA reference；runtime 约 `1.644x` tuned，local metrics 已从 per-trial rows 独立复算。 |
| Paper-writing ready 技术包 | 已完成 | `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md` | 已把方法、理论、实验结果、ablation、图表计划和 claim 边界整理成可拆入 manuscript 的中文材料。 |
| TAES manuscript first draft | 已完成首版，可编译 PDF | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/references.bib` | 使用官方 `IEEEtaes.cls/.bst`，`tectonic` 编译通过，当前 `main.pdf` 为 9 页；方法图已换成原生 LaTeX 矢量图；只剩 underfull/internal-consistency 类非阻塞 warning。 |
| TAES manuscript evidence-chain pass | 进行中，本轮已增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/references.bib`; `docs/paper/taes/manuscript/evidence_sources.json`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md` | 已补近期 DOI 核验引用、Related Work 方法线、operator 伪代码、reference-label invariance、paired CI/wins/sign-test p 表、runtime 表和 full-width N50 reduction 图；Table IV 现在直接显示 full vs reference-only 的 sign-test evidence；本轮新增 scenario-family optional evidence path，但当前 topology/FOV 条目分别是 topology N1 smoke 与 partial-FOV N5 smoke，需要 N50 升级后才能作为 paper-grade generalization evidence。 |
| TAES manuscript reproducible-results pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py`; `docs/paper/taes/manuscript/generated/N50_EVIDENCE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/n50_evidence.json` | N50 paper-facing tables/figure fragments now regenerate from the tracked validation report during `./build.sh`; manifest records report SHA256 and key paper-facing checks. |
| TAES manuscript GA reference evidence pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/extract_reference_baselines.py`; `docs/paper/taes/manuscript/generated/REFERENCE_BASELINE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/reference_baseline_rows.tex` | Contextual GA reference rows now regenerate from tracked AA/GA N50 reports during `./build.sh`; manuscript caveat says these rows are reference baselines, not paired AA sign-test inputs. |
| TAES manuscript held-out robustness pass | baseSeed=11 N5 sanity 与 N50 robustness 均已完成 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED11_20260622_172034.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `docs/paper/taes/manuscript/generated/HELDOUT_SANITY_MANIFEST.md`; `docs/paper/taes/manuscript/generated/HELDOUT_N50_MANIFEST.md`; `docs/paper/taes/manuscript/generated/heldout_n50_section.tex` | Build records the tracked base-seed-11 sanity and N50 robustness checks. Held-out N50 uses trials `12..61`: full barycenter reduces network OSPA by `81.85%`, local E-OSPA by `17.15%`, and local RMSE by `6.64%` with RMSE CI `[0.210, 0.281]`, wins `48/50`, and `p<10^-3`; reference-only RMSE reduction is only `0.82%` with CI `[-0.008, 0.069]`, wins `28/50`, and `p=0.480`, preserving the same mechanism separation as the main N50 run. |
| TAES harsh packet-loss stress validation | N1 smoke 与 fixed-design N50 均已完成，N50 已接入 manuscript/generated evidence/readiness gate | `docs/AA_STRESS_SCENARIO_VALIDATION_CN.md`; `RUN/AA/launchAaTaesHarshLossN50BaseSeed21.sh`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED21_20260624_130131.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md`; `docs/paper/taes/manuscript/generated/STRESS_HARSH_MANIFEST.md`; `docs/paper/taes/manuscript/generated/stress_harsh_summary_sentence.tex`; `docs/paper/taes/manuscript/scripts/extract_stress_evidence.py`; `docs/paper/taes/manuscript/scripts/check_submission_readiness.py` | Harsh-loss profile 固定为 `[0.2 0.35 0.5 0.7] / [1 3 2 2]`，方法参数不变。Formal N50: full method reduces Net OSPA `2.224259 -> 0.454475`, E-OSPA `2.523393 -> 2.092956`, RMSE `4.235071 -> 3.897040`, CardErr `0.146025 -> 0.110500`; paired reductions are `79.57% / 17.06% / 7.98% / 24.33%`. Reference-only RMSE reduction is `2.47%`, preserving a mechanism gap without tuning. |
| TAES generalization scenario-family protocol | 已建立 code/protocol/launcher；`topology-ring` N50 正在运行，`partial-fov35` N5 smoke 已接入 build/readiness | `docs/AA_GENERALIZATION_SCENARIO_PROTOCOL_CN.md`; `RUN/AA/runAaBalancedCardinalityValidation.m`; `RUN/AA/launchAaTaesScenarioFamilySmoke.sh`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N1_SEED31_20260624_155105.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N5_SEED41_20260624_160335.md`; `RUN/AA/AA_TAES_SCENARIO_partial_fov35_N5_BASESEED41_20260624_160334.log`; `RUN/AA/AA_TAES_SCENARIO_topology_ring_N50_BASESEED31_20260624_162437.log`; `docs/paper/taes/manuscript/scripts/extract_scenario_family_evidence.py`; `docs/paper/taes/manuscript/generated/SCENARIO_FAMILY_MANIFEST.md`; `docs/paper/taes/manuscript/generated/scenario_family_section.tex` | 验证入口新增可选 `scenarioOverrides`，默认不改变既有 evidence；显式场景族支持 `topology-ring`、`partial-fov35`、`full-topology`，report 会记录 `scenarioLabel`、`neighborMapMode`、FOV 和 sensor-motion settings。`topology-ring` N1 smoke 显示 full method 在 sparse ring 下相对 tuned AA 降低 Network OSPA `2.771921 -> 1.348601`、E-OSPA `2.580668 -> 2.010548`、RMSE `4.568272 -> 4.035532`；固定 N50 已于 2026-06-24 16:24 CST 启动，PID `37456`。`partial-fov35` N5 smoke 显示 full method 降低 Network OSPA `2.297790 -> 1.341733`、Loc disagreement `1.415682 -> 0.213073`、E-OSPA `2.837543 -> 2.642142`、RMSE `4.120264 -> 3.727303`；Card dispersion/CardErr 仅小幅改善且 CI 覆盖零，是需要 N50 确认的边界信号。`./build.sh` 现在会生成 scenario-family manifest/JSON/response fragment；readiness gate 对当前条目给出有意保留的 smoke-tier warning，避免误称 paper-grade conclusion。 |
| TAES manuscript independent-verifier pass | 已完成本轮 checkpoint | `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py`; `docs/paper/taes/manuscript/evidence_sources.json`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log` | Independent verifier now recomputes network disagreement from per-trial report rows, runtime from trial log, and local E-OSPA/RMSE/CardErr from per-trial local rows; `generated/N50_VERIFICATION_REPORT.md` confirms all three paths. |
| TAES manuscript polish/complexity pass | 进行中，本轮增强 | `docs/paper/taes/manuscript/main.tex`; `docs/paper/taes/manuscript/main.pdf`; `docs/paper/taes/manuscript/generated/method_pipeline.tex`; `docs/paper/taes/manuscript/generated/n50_reduction_bars.tex`; `docs/paper/taes/manuscript/generated/heldout_n50_section.tex` | 已补 graph-locality/complexity paragraph，明确 Hungarian matching 是 runtime overhead 的主要来源；Introduction 已强化 component-correspondence failure framing，并改成四点 contribution paragraph；Figure 1 已改为 `scripts/render_figures.py` 生成的 LaTeX fragment；Structural Properties 已补 stable-matching consensus limit 和 `Weighting is not matching` label-swap 反例；本轮又把 stable-matching consensus limit 收紧为 matched moment coordinates，并把 existence convexity 明确归为 upstream AA existence update；本轮新增 assignment-stability sufficient condition，并明确该条件不覆盖 close crossing/birth/death/cardinality mismatch；Discussion/Conclusion/Acknowledgment 已避免使用“下一步/投稿前还需”这类内部状态口吻；N50 reduction 图已改为 full-width 矢量图并通过 PDF 渲染检查，本轮进一步去掉右侧重复数值堆叠、改用 legend，精确数值保留在 Table IV；本轮又把 held-out N50 robustness table 升级为 full-width CI/wins/p-value 证据表；本轮又重排 Results section 的 full-width floats，让 Table II/III/IV 和 Fig. 3 集中显示，避免理论增强后结果页出现大块空白；Experimental Setup 已补 paired statistical protocol 和 mechanism-isolation protocol 表，把 baseline/proposed/reference-only 三臂分别映射到 scalar-weight consumer、full label-barycenter、label-set-only ablation 三个 claim，并声明固定参数/成对 packet-loss realization；本轮又补入 fixed-design/no per-scenario search 说明，明确 held-out base-seed run 不用于选参；本轮又补入 Octave/Apple M4 runtime 环境和绝对秒数解释边界；当前稿件元数据以 submission-style placeholders 呈现；使用 `placeins` 的 `\FloatBarrier` 防止结果浮动跨入 Discussion；本轮又把 held-out base-seed N50 的 RMSE mechanism separation 前置进 abstract，并清理首页 simulated-data statement；本轮继续压紧 abstract/intro 的 evidence story，把 held-out run 明确为独立 50-trial replication，并把 proposed gain 与 scalar-weight routing 分离。 |
| TAES submission-readiness checklist | content-ready 判定已拆出，portal metadata 仍 pending | `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md`; `docs/paper/taes/manuscript/README.md`; `docs/paper/taes/manuscript/READINESS_AUDIT_CN.md`; `docs/paper/taes/manuscript/COVER_LETTER_AND_METADATA_DRAFT.md`; `docs/paper/taes/manuscript/SUBMISSION_PACKAGE_INDEX.md`; `docs/paper/taes/manuscript/generated/BIBTEX_DOI_VERIFICATION.md`; `docs/paper/taes/manuscript/generated/SUBMISSION_READINESS_REPORT.md`; `docs/paper/taes/manuscript/generated/SUBMISSION_BUNDLE_MANIFEST.md`; `docs/paper/taes/manuscript/generated/REPRODUCIBILITY_LEDGER_MANIFEST.md`; `docs/paper/taes/manuscript/generated/PDF_VISUAL_QA_MANIFEST.md`; `docs/paper/taes/manuscript/generated/STRESS_HARSH_MANIFEST.md`; `docs/paper/taes/manuscript/generated/SCENARIO_FAMILY_MANIFEST.md`; `docs/paper/taes/manuscript/scripts/check_submission_readiness.py`; `docs/paper/taes/manuscript/scripts/verify_bibtex_dois.py`; `docs/paper/taes/manuscript/scripts/render_pdf_visual_qa.py`; `docs/paper/taes/manuscript/scripts/render_reproducibility_ledger.py` | 官方 template 和当前 manuscript 均已用 Tectonic 编译验证；稿件已加入 provisional AI-assistance disclosure；build 后会自动生成 readiness snapshot、clean source bundle、PDF visual-QA manifest、BibTeX DOI resolver verification 和 source-package reproducibility ledger。Checker 已覆盖模板归档、TAES requirements live-source refresh、标题/摘要/关键词、first-page narrative markers、paper-facing wording hygiene、citation keys、uncited BibTeX、DOI fields、DOI resolver verification、clean source bundle、source-bundle reproducibility scripts、submission package index、PDF visual-QA render、source-bundle manifest hash freshness、bundled fallback build mode、implementation-alignment wording、cover letter/portal metadata draft、cover letter/portal metadata source synchronization、held-out N50 strict gate、harsh-stress N50 protocol/coverage/generated summary sentence、scenario-family source/tier/metric coverage、source-package reproducibility ledger 与 N50 local independent verifier；当前 `content_status=candidate_with_warnings`，唯一内容 warning 是 scenario-family 仍为 smoke-tier，`portal_status` 继续因作者/基金/repository metadata placeholders 保留真实投稿阻塞。 |
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
- TAES Table IV 现在显式显示 sign-test p-values: full RMSE 为 `p<10^-3`，reference-only RMSE 为 `p=0.672`，使“label copy 不足以解释空间收益”的统计边界在正文表格中可直接审阅。

## 剩余风险

- 当前最佳 N50 结果仍来自 output-level projection；neighborhood iterative prototype 已通过 N50，但还不是递归滤波内部 online method。
- consensus 指标归零是构造结果；paper-facing claim 必须依赖 local metrics、GA reference 对照和 ablation。
- Independent verifier 已经覆盖 network disagreement、runtime 和 local E-OSPA/RMSE/CardErr；baseSeed=11 held-out N50 已通过；harsh packet-loss N50 stress 已完成并接入 evidence manifest。当前剩余证据风险主要是 topology、partial-FOV、target maneuver 和 recursive-online 实现覆盖不足，而不是 packet-loss severity 单一；其中 topology-ring N50 已启动，partial-FOV 已有 N5 smoke 但尚未升到 paper-grade。
- 当前 ablation 证明了 barycenter 组件有用，理论文档也给出稳定 matching 下 online moment-consensus 收敛到 centralized moment barycenter 的条件；当前实现是 output-level neighborhood iterative prototype，不是递归滤波内部的最终 online method。
- TAES 首稿已经可编译，且 N50 主表/paired 表/runtime 表/N50 reduction 图、held-out N50 table/paragraph、harsh stress summary/response fragment 都已经由 report-driven generated fragments 驱动；方法图和结果图已完成一次矢量化 polish。最终稿仍需要 topology/FOV/maneuver 场景族验证、作者/基金/AI disclosure 最终措辞确认和人工审读。

## 已完成长跑

- 目的: 重新生成 paper-facing N50 neighborhood validation，使 report 包含 `## Per-Trial Local Tracking Metrics`，从而让 local E-OSPA/RMSE/CardErr 进入 independent-verifier path。
- 启动脚本: `RUN/AA/launchAaTaesN50LocalVerifierRerun.sh`。
- PID file: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.pid`。
- Log: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`。
- 结果:

- Report: `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`。
- Log: `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`。
- `docs/paper/taes/manuscript/evidence_sources.json` 已接入这对 report/log；`./build.sh` 后 `generated/N50_VERIFICATION_REPORT.md` 显示 local metrics are independently recomputed from per-trial local tracking rows。

## 已完成长跑: held-out baseSeed=11 N50 robustness

- 目的: 生成 paper-grade held-out N50 evidence，用 baseSeed=11 检查 neighborhood label-barycenter 是否跨 seed 维持相同机制收益。
- 启动脚本: `RUN/AA/launchAaTaesHeldoutN50BaseSeed11.sh`。
- PID: `92443`。
- PID file: `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.pid`。
- Log: `RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_20260624_094911.log`。
- 启动状态: 2026-06-24 09:49 CST 时进程存活，log 已进入 `AA validation trial 1/50`。
- 完成状态: 2026-06-24 12:22 CST 生成 report `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`。
- 关键结果: full barycenter 相对 tuned AA 的 network OSPA / local E-OSPA / local RMSE reductions 为 `81.85% / 17.15% / 6.64%`；wins 为 `50/50`, `50/50`, `48/50`。Reference-only RMSE reduction 为 `0.82%`，wins `28/50`，sign-test `p=0.4799`，因此 held-out run 继续支持“matched posterior barycenter 贡献空间收益，而不只是复制 reference labels”。
- 接入状态: `docs/paper/taes/manuscript/evidence_sources.json` 已加入 `heldout_n50_report`，`./build.sh` 生成 `generated/HELDOUT_N50_MANIFEST.md`、`generated/heldout_n50_evidence.json` 和 `generated/heldout_n50_section.tex`。

## 已完成长跑: harsh packet-loss baseSeed=21 N50 stress

- 目的: 检查 fixed-design neighborhood label-barycenter 在更严 packet-loss profile 下是否仍保持 agreement/local tracking 方向的收益，同时避免把 stress result 变成调参入口。
- 启动脚本: `RUN/AA/launchAaTaesHarshLossN50BaseSeed21.sh`。
- PID file: `RUN/AA/AA_TAES_STRESS_HARSH_N50_BASESEED21_20260624_130728.pid`。
- Log: `RUN/AA/AA_TAES_STRESS_HARSH_N50_BASESEED21_20260624_130728.log`。
- 完成状态: 2026-06-24 15:33 CST 生成 report `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md`。
- 关键结果: full barycenter 相对 tuned AA 的 network OSPA / local E-OSPA / local RMSE / CardErr reductions 为 `79.57% / 17.06% / 7.98% / 24.33%`；wins 为 `50/50`, `50/50`, `48/50`, `50/50`。Reference-only RMSE reduction 为 `2.47%`，wins `38/50`，因此 stress N50 继续支持“matched posterior barycenter 提供额外空间收益”，并且没有改变方法参数。
- 接入状态: `docs/paper/taes/manuscript/evidence_sources.json` 已加入 `stress_harsh_n50_report`，`./build.sh` 生成 `generated/STRESS_HARSH_MANIFEST.md`、`generated/stress_harsh_evidence.json`、`generated/stress_harsh_section.tex` 和 `generated/stress_harsh_summary_sentence.tex`；主文 Discussion 引入 concise generated sentence，完整表格保留为 response-ready fragment。

## 正在运行长跑: topology-ring baseSeed=31 N50 generalization

- 目的: 生成 paper-grade topology-family evidence，检验 fixed-design neighborhood label-barycenter 在 sparse 8-node ring neighborhoods 下是否仍保持同一机制解释。
- 启动命令: `AA_SCENARIO_FAMILY=topology-ring AA_SCENARIO_TRIALS=50 AA_SCENARIO_BASE_SEED=31 RUN/AA/launchAaTaesScenarioFamilySmoke.sh`。
- PID: `37456`。
- PID file: `RUN/AA/AA_TAES_SCENARIO_topology_ring_N50_BASESEED31_20260624_162437.pid`。
- Log: `RUN/AA/AA_TAES_SCENARIO_topology_ring_N50_BASESEED31_20260624_162437.log`。
- 启动状态: 2026-06-24 16:24 CST 时已成功 detach；后续用 `tail -f RUN/AA/AA_TAES_SCENARIO_topology_ring_N50_BASESEED31_20260624_162437.log` 查看。
- 最新检查: 2026-06-24 17:08 CST 时 PID `37456` 仍存活，log 已进入 trial `21/50`，尚未生成 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED31_*.md` report。
- 接入计划: 完成后把生成的 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED31_*.md` 配到 `scenario_topology_ring_report`，重跑 `./build.sh`，检查 `SCENARIO_FAMILY_MANIFEST.md`、`scenario_family_evidence.json`、`scenario_family_section.tex` 和 `submission_readiness.json`。若结果 mixed，保留 boundary 解释，不回调参数。

## 下一步

1. 等 topology-ring N50 完成后接入 scenario-family evidence/readiness；若通过，再决定是否启动 partial-FOV N50 或 full-topology N50。
2. 把 output-level iterative prototype 下沉到递归滤波内部的 online label message / moment consensus。
3. 给 online 版本设计新的 method-level ablation: label canonicalization only、state barycenter only、iterative local consensus。
4. 继续压缩和润色 TAES `main.tex`: 控制 stress/held-out/scenario-family 证据进入正文的篇幅，避免超过 TAES page budget。
5. 替换作者、基金、repository、corresponding author、OA/preprint/conflict 等投稿表单 metadata placeholders，并做最终人工审读。
