# AA Label-Barycenter 目标期刊/会议选择建议

日期: 2026-06-23

## Question

基于当前 AA label-barycenter 工作的学术价值、同领域论文发表 venue、以及当前证据强度，选择最合适的目标期刊/会议，并给出投稿前需要补强的 gate。

## Scope

包含:

- 当前工作: `Neighborhood label-barycenter spatial-KLA AA`、centralized cross-local upper-bound、reference-only ablation、N50 paired validation。
- 本地证据: `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md`、`docs/AA_LABEL_BARYCENTER_THEORY_CN.md`、`RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`。
- 同领域 venue 线索: labeled RFS / LMB fusion、AA density fusion、distributed sensor fusion、information fusion、target tracking。
- 外部 venue scope: Information Fusion、IEEE Transactions on Signal Processing、FUSION conference，以及 TAES/Signal Processing 的同领域发表事实。

排除:

- 不做投稿时间表承诺；正式 deadline 需要投稿前再次核对。
- 不把当前 output-level prototype 直接包装成最终 online filter。
- 不凭 impact factor 单独选择 venue。

## Risk Tier

L2。目标期刊/会议选择会影响论文叙事和后续实验投入；当前建议是 self-check only，未经过导师或独立同行确认。

## Recommendation

用户决策: **IEEE Transactions on Aerospace and Electronic Systems (TAES)** 作为当前主目标。

主推荐: **IEEE TAES**。当前 AA label-barycenter 的最稳写法不是泛化成一个 broad information-fusion framework，而是写成 distributed LMB multi-target tracking 中的 label-aligned fusion operator。TAES 对 multi-target tracking、sensor fusion、RFS/LMB 方法和系统级实验更自然；AA density fusion Part III 已发表在 TAES，说明 AA/RFS fusion 主题在 TAES 社区可被接受。

TAES 模板、作者须知和投稿清单已整理到 `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md`；regular paper LaTeX 入口为 `docs/paper/taes/template_regular/IEEE_TAES_orig-research/TAES_template.tex`。

Stretch target: **Information Fusion**。如果后续能完成 recursive online arm、independent verifier 和跨场景验证，Information Fusion 仍是更高认可度的增强版目标。但在当前证据阶段，TAES-first 更稳。

备选 B: **Signal Processing**。如果论文重心改成“labeled RFS / information-geometric 或 statistical signal-processing operator”，并强化理论推导、复杂度和估计性能分析，Signal Processing 也合适。Cao-Zhao 信息几何 labeled RFS fusion 线在 Signal Processing，说明该领域有直接 precedent。

会议建议: **FUSION 2027** 作为快速反馈与社区曝光目标，而不是 FUSION 2026。FUSION 2026 官网显示会议日期为 2026-06-23 到 2026-06-26；当前日期已经是 2026-06-23，投稿窗口不现实。若要先投 conference，应把它作为 journal paper 的短版/早期版，避免把完整 novelty 消耗在 proceedings 上。

不建议作为第一目标:

- IEEE Transactions on Signal Processing: 只有在 online method + convergence/optimality theorem + rigorous performance analysis 都显著增强后才考虑。当前方法创新偏 fusion/tracking operator，TSP 会要求更强的信号处理理论普适性。
- Automatica / IEEE TAC: 除非把问题重写成 consensus/control theorem paper，否则 scope 不自然。
- IEEE Sensors Journal / Sensors: 可以作为保底，但当前工作若补强 online 化，价值不应先降到这个层级。
- Journal of Advances in Information Fusion: 社区匹配，但综合认可度和影响力低于 Information Fusion；适合作为备选保底或 extended technical note，不建议首选。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | TAES 是当前主目标，因为该工作最自然的论文形态是 distributed LMB multi-target tracking 下的 label-aligned AA/RFS fusion operator。 | High | E2, E4, E5, E6 | 仍需强化 tracking-system 叙事、runtime 和可复现实验。 |
| C2 | Information Fusion 是更高认可度的 stretch target，但当前 self-check output-level prototype 不足以直接支撑该层级的 broad fusion claim。 | High | E1, E2, E3, E4 | 需要 recursive online arm、independent verifier、跨场景验证、统一 runtime。 |
| C3 | TAES 的同领域 precedent 更直接，因为 AA/RFS fusion 已有 TAES 发表记录，tracking/radar-sensor community 更容易接受 method+experiment 组合。 | Medium-High | E5, E6 | TAES 对系统/应用可信度要求高，不能只给 algorithm toy evidence。 |
| C4 | Signal Processing 是理论化备选；适合更强数学推导、filter/operator 分析，而不是以系统实验为主。 | Medium | E7, E8 | 当前 narrative 若保持 fusion-system 取向，Signal Processing 不如 Information Fusion 直观。 |
| C5 | FUSION conference 是社区反馈通道，但 FUSION 2026 已到会议期，不是当前投稿目标。 | High | E9 | 可面向 FUSION 2027 或相关 special session。 |
| C6 | TSP/Automatica/TAC 不是当前第一目标，因为现有 novelty 尚未达到这些 venue 对通用理论贡献的预期。 | Medium | E8, E10 | 若完成严格 distributed moment-consensus theorem 和 recursive filter proof，可重新评估。 |

## Venue Fit

| Rank | Venue | Fit | Recognition | Acceptance risk | Why |
| --- | --- | --- | --- | --- | --- |
| 1 | IEEE TAES | High | High in tracking/aerospace | Medium | RFS/AA density fusion precedent 强；适合 multi-target tracking、sensor fusion、radar/ground/air systems；与当前 evidence maturity 最匹配。 |
| 2 | Information Fusion | Very high | Very high | High | Scope 正中 multi-sensor/multi-source information fusion、distributed sensor networks、fusion algorithms、imperfect environments；但需要更强 online/distributed 和跨场景证据。 |
| 3 | Signal Processing | Medium-high | High | Medium-high | 有 LMB fusion 和 information geometry precedent；但需要更数学化的 signal-processing/operator story。 |
| 4 | FUSION 2027 | High as feedback | Medium | Medium | 信息融合旗舰会议，适合短版、early community feedback、建立审稿语境。 |
| 5 | IEEE TSP | Medium | Very high | Very high | Scope 覆盖 filtering/estimation theory，但当前工作必须强化为通用 signal-processing contribution。 |
| 6 | JAIFF / Sensors-tier venues | Medium | Lower | Lower | 可以保底，但与当前工作潜在价值不匹配。 |

## Paper-Value Assessment

当前工作的价值不在“又调出一个 AA baseline”，而在把 AA-LMB 的失败模式重新定义成一个更基础的问题:

```text
cross-local label canonicalization + matched posterior barycenter
```

这个切入点有三个值得认可的地方:

1. 它正中 labeled RFS / LMB fusion 的长期痛点: labels 在 distributed local filters 中不一致时，直接融合会退化。
2. 它不是简单权重搜索，而是把 fusion operator 拆成 reference label set、assignment、moment barycenter 和 neighborhood iteration。
3. 当前 N50 evidence 同时有 full method、reference-only ablation、centralized upper-bound 和 GA reference 对照，已经具备 paper seed 的形状。

但当前仍有三个会影响高水平 venue 的短板:

1. 当前主方法还是 output-level post-pass，不是 recursive online distributed filter。
2. N50 是 self-check deterministic paired validation，缺 independent verifier。
3. 场景还集中在 tiered packet-loss dual-formation scenario，泛化证据不足。

因此，选择策略应是:

```text
当前主线: TAES-first，写成 distributed LMB multi-target tracking method。
增强版: 补 online + independent verifier + cross-scenario validation 后再考虑 Information Fusion。
反馈线: 做 FUSION 2027 短版，但保留 journal novelty。
```

## Positioning by Target

### IEEE TAES

推荐题目风格:

```text
Label-Barycenter Average Fusion for Distributed LMB Multi-Target Tracking
```

更 TAES-friendly 的备选题目:

```text
Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication
```

主叙事:

- distributed multi-target tracking in sensor networks;
- robust track/label alignment under communication loss;
- practical LMB implementation and reproducible tracking metrics;
- comparison with GA/KLA and AA/RFS baselines;
- method complexity, runtime and deployability.

投稿前 gate:

- 强化 scenario realism: radar-like tracking、sensor trajectories、packet drops、runtime。
- 增加 per-sensor / per-target qualitative failure examples，展示 label split 和 matched-state spread 如何被修复。
- 明确 method complexity 和 deployability，尤其是 Hungarian matching、neighborhood iterations 和 memory/runtime cost。
- 保持 centralized consensus=0 在 appendix/diagnostic，不放 main headline。
- 补 independent verifier 或至少独立复跑 N50。

### Information Fusion

推荐题目风格:

```text
Neighborhood Label-Barycenter Average Fusion for Distributed LMB Tracking under Unreliable Communication
```

主叙事:

- distributed information fusion under unreliable communication;
- labeled RFS/LMB posterior fusion;
- label-space misalignment as a fusion-system failure mode;
- graph-local label-barycenter operator;
- theory + ablation + N50 paired validation.

投稿前 gate:

- 把 neighborhood operator 下沉为 online/recursive arm，或明确证明 output-level graph-local projection 在目标任务中是合法 post-fusion estimator。
- 增加至少一个 cross-scenario validation: different topology, packet-loss regime, sensor count, or partial-FOV stress。
- independent verifier 复跑 N50 或审查 report generation。
- 统一 AA/GA runtime denominator。
- 把 centralized consensus=0 降到 appendix/diagnostic。

### Signal Processing

推荐题目风格:

```text
Label-Aligned Moment Barycenters for Distributed Labeled RFS Fusion
```

主叙事:

- labeled RFS fusion as a statistical signal-processing problem;
- assignment-induced barycenter operator;
- moment projection and stable consensus theorem;
- less emphasis on application-specific tuning.

投稿前 gate:

- 补强 theorem: stable matching, convergence, boundedness, and maybe consistency diagnostics。
- 降低工程叙事，增加 operator-level analysis。
- 增加 baselines from AA density fusion / PHD-AA family if feasible。

### FUSION 2027

推荐题目风格:

```text
Graph-Local Label Barycenters for Distributed LMB Fusion
```

主叙事:

- short paper / full conference paper as community feedback;
- emphasize problem insight and ablation;
- avoid revealing every journal-only extension if journal-first strategy更重要。

## Evidence Ledger

| ID | Type | Source or artifact | What it supports |
| --- | --- | --- | --- |
| E1 | venue scope | ScienceDirect Information Fusion aims and scope: multi-sensor/multi-source information fusion, distributed/wireless sensor networks, adaptive fusion architectures, imperfect environments | Information Fusion fit |
| E2 | local paper brief | `docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md` | Current method, theory, N50 evidence, limitations |
| E3 | local theory doc | `docs/AA_LABEL_BARYCENTER_THEORY_CN.md` | Theoretical claims and output-level boundary |
| E4 | local experiment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md` | N50 neighborhood full/reference-only/tuned AA results |
| E5 | local bibliography | `docs/paper/els-cas-templates/paper-refs.bib`, entry `Li2024AADensityFusion` | AA density fusion precedent in IEEE TAES |
| E6 | venue/source lookup | IEEE TAES public description and local bibliography | TAES fit for aerospace/electronic tracking systems |
| E7 | local bibliography | `CaoZhao2025InfoGeometryFusion`, `Wang2018CentralizedLMBFusion`, `Gostar2021CentralizedCooperativeLMB` | Signal Processing and TSP precedent for LMB/fusion |
| E8 | venue scope | IEEE TSP official scope: novel theory, algorithms, performance analyses and applications of filtering/estimating/detecting signals | TSP requires stronger general signal-processing contribution |
| E9 | conference scope | FUSION 2026 official page: flagship ISIF conference; topics include random sets, tracking, distributed systems, uncertainty, performance | FUSION fit and timing |
| E10 | local positioning | `docs/paper/00_positioning.md` and `docs/paper/03_related_work.md` | Existing GA/KLA paper scope and relation to AA line |

## Verification Record

Independence status: self-check only. 本建议由同一 worker lane 基于本地仓库、既有 bibliography 和外部 venue pages 生成，尚未经过 independent verifier 或导师确认。

已检查:

- 当前分支为 `codex/aa-target-wise-fix`，仅存在未跟踪历史 run logs，无 tracked 修改。
- `AA_LABEL_BARYCENTER_PAPER_READY_CN.md` 已记录当前方法是 output-level iterative prototype，不是最终 recursive online filter。
- `paper-refs.bib` 显示同领域论文分布在 IEEE TSP、Signal Processing、IEEE TAES、Information Fusion、IEEE TCYB、FUSION conference。
- Information Fusion 官方 scope 明确覆盖 multi-sensor/multi-source information fusion、distributed/wireless sensor networks、adaptive fusion systems、imperfect environments。
- IEEE TSP 官方 scope 覆盖 novel theory/algorithms/performance analyses of signal processing, filtering and estimation，但对通用理论贡献要求更高。
- FUSION 2026 官网显示 conference date 为 2026-06-23 到 2026-06-26，当前时间点不适合作为投稿目标。

## Risk and Escalation

最高风险是用 Information Fusion 式的 broad fusion narrative 去写 TAES paper。TAES-first 应把文章收敛到 distributed multi-target tracking、LMB implementation、label/track alignment、runtime 和可部署性，而不是把所有贡献包装成宏大的 general information fusion framework。

第二风险是把 AA label-barycenter 和现有 GA/KLA paper 混成同一篇。建议把它作为独立论文方向，而不是附录更新；否则 novelty 会被主线 GA adaptive weights 稀释。

第三风险是 conference-first 消耗 novelty。如果选择 FUSION 2027，应写成较短的 problem+operator+ablation paper，并保留 online extension 和 cross-scenario validation 给 journal version。

## Reproducibility

本地核查命令:

```bash
git -C /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS status --short --branch
sed -n '1,240p' docs/AA_LABEL_BARYCENTER_PAPER_READY_CN.md
sed -n '1,220p' docs/paper/03_related_work.md
rg -n "CaoZhao2025|Li2024AADensity|Li2026FIMultirate|Shen2022Consensus|Gao2023Resilient|Wang2018Centralized|Gostar2021|journal|booktitle" docs/paper/els-cas-templates/paper-refs.bib
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/AA_TARGET_VENUE_RECOMMENDATION_CN.md
```

外部核查页面:

- Information Fusion aims and scope: `https://www.sciencedirect.com/journal/information-fusion/about/aims-and-scope`
- IEEE TSP scope: `https://signalprocessingsociety.org/publications-resources/ieee-transactions-signal-processing`
- FUSION 2026 page: `https://www.ntnu.edu/fusion2026`

## Open Issues

- 需要导师确认更看重“高认可度”还是“中等风险可发表性”。
- 需要决定 AA label-barycenter 是否作为独立 paper，而不是塞回现有 GA/KLA manuscript。
- 需要补 Zotero AA Fusion 论文的 venue map，尤其是最近 AA density fusion Part IV/V 是否已有正式 venue。
- 需要确认目标期刊当前 author guide、page limit、open-access cost 和 expected review time。

## Recommendation

按用户决策，当前采用 TAES-first 三层策略:

1. **主投 IEEE TAES**。把论文写成 distributed LMB multi-target tracking 方法，核心是 label-barycenter average fusion 如何修复 unreliable communication 下的 label split 和 same-label spatial spread。
2. **保留 Information Fusion 作为增强版目标**。如果后续 online 化、independent verifier 和 cross-scenario validation 都完成，可以重新评估是否升级目标。
3. **FUSION 2027 作反馈通道**，不投 FUSION 2026。若需要早期社区反馈，准备一个短版，核心是 problem diagnosis + neighborhood operator + reference-only ablation；journal version 保留 online extension 和更完整验证。

我不建议现在把 IEEE TSP 作为第一目标。除非我们把理论部分推进到严格 recursive consensus/barycenter theorem，并把实验从 tracking-specific validation 扩展到更一般的 signal-processing estimator comparison，否则 TSP 的 scope 和门槛都不如 TAES 自然。
