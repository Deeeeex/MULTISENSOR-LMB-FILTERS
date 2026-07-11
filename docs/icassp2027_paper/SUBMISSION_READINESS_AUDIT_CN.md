# ICASSP 2027 投稿就绪完成性审计

审计日期：2026-07-11

审计对象：`docs/icassp2027_paper/main.tex` 与 `main.pdf`

目标：证明当前稿件是否满足“技术严谨、证据闭合、视觉合理、文字准确、可直接投稿”，而不是仅确认测试未报错。

## 1. 最终裁决

| 层级 | 状态 | 含义 |
|---|---|---|
| Scientific/implementation readiness | **PASS** | 当前主张、实现路径、冻结证据、图表与文字边界一致 |
| PDF/layout readiness | **PASS** | 4 页技术内容 + 第 5 页参考文献；无 overfull、裁切、字体或链接边框问题 |
| Author metadata readiness | **PASS** | Jinhao Chen 第一作者；Tianyu Wo 通讯作者；机构顺序按第一作者 |
| Venue-policy readiness | **PASS** | single-anonymous、页数、ethics、funding 与 COI 均已核对并写入 PDF |
| Final-template readiness | **EXTERNAL PENDING** | 2027 detailed submission/template URL 截至本次审计仍返回 HTTP 404 |
| Overall submission status | **PRE-SUBMISSION PASS / FINAL-TEMPLATE PENDING** | 稿件自身门禁均通过；仅待官方 2027 kit 上线后的模板终检 |

## 2. 目标逐项证据表

| 要求 | 权威证据 | 审计结论 |
|---|---|---|
| 最后一次专业对抗性审稿 | 三路只读技术、原创性、可读性复审；处置记录见 `REVISION_GUIDE_CN.md` 第 12 节 | 完成 |
| 核心故事不夸大 | 标题改为 `Receiver-Induced Moment Exchange for Distributed LMB Fusion`；摘要/引言明确 custom receiver、standard moment matching 与 receiver-induced certificate | 通过 |
| Receiver provenance 清楚 | 正文说明为 authors' custom presence-conditioned geometric-average LMB receiver，不是 full-source density-KLA reference implementation | 通过 |
| 数学命题覆盖真实路径 | `F(pi_i,T pi_-i)=F(pi_i,T P pi_-i)`；self 不再被误写为经过 wire transport | 通过 |
| 数值定义域完整 | admissible domain 要求 projected fields 和全部 quadratic/logdet/exp/Bernoulli intermediates 有限，且每次 `R` 成功 | 通过 |
| 权重术语准确 | 实际 base weights 写为 fixed symmetric degree-based：self `1/3`、四邻居各 `1/6`；不再称标准 Metropolis | 通过 |
| 实验因果隔离 | frozen two-arm protocol；同 seeds、轨迹、测量、schedule、labels、weights、delivery uniforms 与 masks | 通过 |
| 实验口径准确 | N50 snapshots 明确位于 common `r>1e-2` retention 后；raw pre-retention equality 由 Proposition 1 给出 | 通过 |
| 主通信数字可复核 | validator：50 trials，mean `58.277264228259%`，95% CI `[57.923221880703,58.636094998469]%` | 通过 |
| Exact audit 可复核 | 40,000 retained post-step snapshots，1,119,037 label instances，`max r/mu/Sigma residual=[0,0,0]` | 通过 |
| 实验大表不会漂移 | `tests/test_icassp2027_experiment_report.py` 对 50 seed 行逐项回读 frozen CSV | 通过 |
| 相关工作覆盖 | 20 条实际渲染引用；density rule、triggering、component selection、track-level projection 与 moment reduction 均有来源 | 通过 |
| 引文元数据 | 19 个非 arXiv DOI 的 Crossref 标题/年份一致；Fantacci 2015 由 arXiv 原文核验 | 通过 |
| Fig. 1 概念正确 | local GM-LMB posterior、full-GM codec/receiver projection、sender projection/moment codec 与正文符号一致 | 通过 |
| Fig. 2 信息有效 | mean footprint + 50 paired trials；无文字交叠；单位统一为 `10^6 B/trial` | 通过 |
| 图中文字可读 | manifest 按 PDF mediabox 与实际 include scale 计算；最小估算 final font `9.038 pt` | 通过 |
| 视觉与排版 | 全 5 页逐页 PNG 复核；页 1--4 密度均衡；页 5 两栏参考文献平衡 | 通过 |
| PDF 结构 | US Letter，5 pages，TeX Gyre Termes regular/bold/italic 嵌入，无 Latin Modern fallback，链接边框隐藏 | 通过 |
| Ethics | 独立 `Compliance with Ethical Standards`，使用官方 numerical-simulation 表述 | 通过 |
| AI disclosure | 识别 OpenAI Codex、使用范围和建议级别；作者验证责任明确 | 通过 |
| Funding/COI | 作者确认本稿无 funding，且全体作者无相关 financial/nonfinancial interests；官方示例声明已写入 Acknowledgment | **PASS** |
| 2027 template | 当前 `spconf.sty` 与仓库保存的 official ICASSP 2026 template SHA-256 相同；2027 detailed URL 为 404 | **EXTERNAL PENDING** |

## 3. Funding/COI 作者确认与落位

作者于 2026-07-11 确认：本稿无 funding；全体作者无需要披露的 COI。论文据此采用 ICASSP 官方示例：

> No funding was received for conducting this study. The authors have no relevant financial or nonfinancial interests to disclose.

该声明位于 `Acknowledgment` 首句，并由严格 PDF gate 检查。若事实状态发生变化，作者必须同步修改声明，不得继续使用无冲突文本。

## 4. 官方核对入口

- CFP/page rule：<https://2027.ieeeicassp.org/wp-content/uploads/sites/13/2019/08/ICASSP2027-CallForPapers-July1.pdf>
- Funding/COI 与 ethics：<https://2027.ieeeicassp.org/about/sps-policies/>
- Single-anonymous 与 review criteria：<https://2027.ieeeicassp.org/about/editorial-policies/>
- Detailed submission/template（本次审计 HTTP 404）：<https://2027.ieeeicassp.org/paper-submission-instructions/>

## 5. 最终复验命令

```bash
pytest -q tests/test_icassp2027_figures.py tests/test_icassp2027_experiment_report.py
octave-cli --quiet --eval "setPath; addpath('tests'); test_lmb_moment_projection; test_lmb_wire_codec; test_lmb_posterior_equivalence;"
octave-cli --quiet --eval "setPath; addpath('tests'); addpath('RUN/GA'); test_icassp_moment_exchange_runner; test_icassp_moment_exchange_evidence;"
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); v=validateFusionSufficientEvidence('RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md'); disp(v);"
cd docs/icassp2027_paper && tectonic --keep-logs --keep-intermediates main.tex
cd ../.. && /Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py --pdf docs/icassp2027_paper/main.pdf
# 作者声明已写入，以下 submission gate 必须通过：
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py --pdf docs/icassp2027_paper/main.pdf --require-submission-declarations
git diff --check
```

## 6. 完成定义

只有下列事项满足时，才可把 thread goal 标记为 complete：

- [x] 作者确认 funding/COI，并将真实声明写入最终 PDF；
- official ICASSP 2027 paper kit 上线后，对模板、边距、字体、页数和 PDF eXpress 要求重新检查并通过。
