# ICASSP 修改方向证据包

## Question

当前 `effective-kla-lmb` 稿件应采用什么中心故事，以及哪些代码、实验、图表与
正文修改足以把该故事变成可审计的 ICASSP 2027 投稿？本证据包支持“是否执行
`Receiver-Induced Moment Exchange` 重写方案”的决定。

## Scope

纳入范围：当前分支代码与测试、冻结实验协议、N50 MAT/CSV/Markdown、当前
LaTeX/PDF、三路独立 reviewer 报告、Figure 1/2 生成器与 manifest，以及六项
closest-work DOI 的 Crossref/出版商元数据核验。

明确排除：一般 Gaussian-mixture KLA 的无损性、MAC/PHY/transport/MTU/重传、
payload-size-dependent loss、event-trigger/dynamic-topology 优越性、多轮 consensus
与 mixture-aware receiver。

## Risk Tier

`L3`。结论会直接进入论文标题、命题、数字与新颖性陈述；错误会造成投稿与学术
声誉风险，因此需要 independent verifier、auditor 和作者最终审批。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|---|---|---|---|---|
| C1 | 当前旧稿的 graph-preservation 四臂故事与最新证据错位，不应继续投稿 | High | E1, E2, E7 | 评价针对审查时旧稿 |
| C2 | 可辩护贡献是作者自定义 presence-conditioned geometric-average receiver 诱导的 moment message 与 executable interface certificate，不是新 KLA 算法或 moment matching | High | E3, E4, E8 | 必须紧邻定义 receiver provenance 与适用边界 |
| C3 | 在 admissible numerical contract 下，命题给出 raw receiver-field equality；冻结 N50 的 retained post-step fields 为 bit-exact | High | E3, E4, E5 | 不等价于 full GM density 相同；N50 snapshots 在 common retention 后捕获 |
| C4 | 50 个 paired confirmatory seeds 的 attempted application-layer byte reduction 均值为 58.277264%，95% CI 为 [57.923222%, 58.636095%] | High | E4, E5 | 不是 airtime、energy 或 end-to-end traffic |
| C5 | 最新证据可从 tracked MAT 重算 CSV、aggregate、bootstrap 与 Markdown | High | E5, E6 | 执行 seeds 已 burn，只允许只读 validation |
| C6 | 新 Figure 1/2 已与中心 claim 对齐且直接绑定冻结 evidence | High | E9 | 最终稿必须按双栏全宽使用并重新视觉检查 |
| C7 | 重写后的正文、图表、Discussion 与 4+1 PDF 已通过 claim、页数和全页视觉 gate | High | E10 | 作者已确认无 funding、无 COI；detailed 2027 template 链接尚未上线 |
| C8 | 最小投稿源闭包可确定性打包，并能在隔离目录重建出通过严格 gate 的五页 PDF | High | E11 | 正式 2027 kit 上线后仍需替换/比对模板并重跑 |

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
|---|---|---|---|---|
| E1 | manuscript baseline | commit `44e75dd` 中的 `main.tex`、Introduction 与 Experiments | C1：旧 58.6%、held-out、四臂与 dynamic 叙事曾在稿件 | strong |
| E2 | rendered baseline | commit `44e75dd` 中的 `main.pdf` | C1：旧稿机械 4+1 合规但图内字约 3--5 pt、第五页极空 | strong |
| E3 | code | `projectLmbObjectMoments.m`; `compressLmbPosterior.m`; `fuseLmbPosteriorsByLabel.m`; `runEventTriggeredDistributedLmbFilter.m` | C2/C3：shared projection、投影后 fusion、encode-before-draw/decode-before-use | strong |
| E4 | protocol/data | `EXPERIMENT_PROTOCOL_CN.md`; `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.{mat,csv,md}` | C2--C4：唯一变量、seeds、字节口径、retained post-step exact audit | strong |
| E5 | command | `octave-cli --quiet --eval "setPath; addpath('RUN/GA'); v=validateFusionSufficientEvidence('RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md'); ..."` -> `valid=1 trials=50 mean=58.277264228259 ci=[57.923221880703,58.636094998469] max=[0,0,0]` | C3--C5 | strong |
| E6 | test | `test_icassp_moment_exchange_evidence` -> passed; MAT SHA `8ad63429...ab81f` | C5：descendant-safe validator 与原始 artifact 一致 | strong |
| E7 | independent review | technical, novelty, and readability reviewer lanes; final synthesis in `REVISION_GUIDE_CN.md` section 12 | C1--C3：三路共同确认结果有效，并识别 receiver provenance、weight naming、numerical domain 与 retained-audit 边界 | strong |
| E8 | literature | Crossref metadata and publisher pages for DOIs listed in `REVISION_GUIDE_CN.md` section 5 | C2：schedule reduction、component selection、state/covariance projection 的边界 | medium |
| E9 | figure/report tests | `figure_manifest.json`; `tests/test_icassp2027_figures.py`; `tests/test_icassp2027_experiment_report.py` -> `2 passed` | C4/C6：图与实验大表均绑定 frozen CSV；图表双生成一致，50-seed 行逐项回读一致 | strong |
| E10 | manuscript/PDF/test | rewritten `main.tex`, all six sections, `refs.bib`, `main.pdf`; `tests/check_icassp2027_pdf.py` -> exact 5 pages, TeX Gyre font-shape gate, float-placement gate, and rendered page 1--5 visual QA | C7：Story A 完整落稿，第 5 页无正文/图/表，图内估算最小 9.0 pt 以上且版面密度均衡 | strong |
| E11 | submission bundle/test | `submission/build_submission_bundle.py`; `submission/upload_files/{manuscript.pdf,manuscript_source.zip,SHA256SUMS}`; `tests/test_icassp2027_submission_bundle.py` | C8：ZIP 仅含 12 个真实 source + build/manifest；重复生成 SHA 一致；隔离解压编译和严格 PDF gate 通过 | strong |

## Verification Record

- Independence status: `independent verifier`。技术 lane 尝试证伪 exact equivalence、
  唯一变量与 byte accounting，并发现 evidence-commit provenance 悖论。
- Independence status: `independent verifier`。新颖性 lane 未接收主代理结论，独立
  将旧稿评为 Weak Reject，并把 tautology/state-projection closest work 列为最高风险。
- Independence status: `independent verifier`。可读性 lane 独立渲染五页 PDF，测得
  旧图最终字号不足、标签重叠和第 5 页利用率极低。
- Independence status: `scripted check`。production validator 在 evidence/figure
  descendant commit 上返回 `valid=1`；figure test 两次生成的 manifest 与文件 SHA
  完全一致。
- Independence status: `scripted and visual check`。最终稿重新构建后，PDF checker
  确认恰好 5 页、references 只在第 5 页、所有正文标题位于前 4 页；随后逐页检查
  PNG，确认图表/公式/表格无裁切，第 4 页 Discussion 与结论无大面积异常留白。
  checker 另核对 TeX Gyre Termes regular/bold/italic 均嵌入且无 Latin Modern
  fallback；figure manifest 按实际 PDF mediabox 与 0.94 text-width include scale 计算，
  两图最终最小估算字号均不低于 9 pt。
- Independence status: `isolated rebuild`。确定性 source ZIP 在全新临时目录解压，
  逐文件核对内部 SHA-256 manifest 后从 `main.tex` 重建；重建 PDF 再通过五页、字体、
  内容与 funding/COI 严格 gate。第二次打包得到相同 ZIP SHA-256。
- Disagreement log: 三路对删除旧 topology 故事、保留 frozen N50 与应用层 byte
  边界没有分歧。最终复审进一步一致要求：把 receiver 说明为 custom variant，
  删除标题中的 `KLA-LMB`，把 historical `metropolis` 纠正为 fixed degree-based
  weights，并把 N50 口径限定为 retained post-step fields。原创性仍属 focused，
  因而禁止 `first`、`novel KLA algorithm` 或一般性 sufficiency 包装。

## Risk and Escalation

若方案错误，最可能的后果是把 implementation factorization 夸大成一般 KLA 理论、
把 application-layer bytes 误写成 network/energy reduction，或让旧混杂实验进入
主结论。实施可继续，因为用户已经授权修改与 checkpoint push；但最终投稿、作者
声明和 venue compliance 仍需作者人工批准。

## Reproducibility

核心只读验证：

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); v=validateFusionSufficientEvidence('RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md'); disp(v);"
```

图表重建与 gate：

```bash
SOURCE_DATE_EPOCH=0 /Users/dex/miniconda3/bin/python3 docs/icassp2027_paper/scripts/generate_figures.py --evidence RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv
/Users/dex/miniconda3/bin/python3 -m pytest -q tests/test_icassp2027_figures.py tests/test_icassp2027_experiment_report.py
```

正文构建与 4+1 gate：

```bash
cd docs/icassp2027_paper && tectonic --keep-logs --keep-intermediates main.tex
cd ../.. && /Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py
```

作者声明写入后的最终 submission gate：

```bash
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 tests/check_icassp2027_pdf.py --pdf docs/icassp2027_paper/main.pdf --require-submission-declarations
```

当前普通 PDF gate 与严格 submission gate 均通过；后者锁定作者确认的无 funding、无 COI 声明。

确定性投稿包与隔离重建 gate：

```bash
/Users/dex/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  docs/icassp2027_paper/submission/build_submission_bundle.py --verify
pytest -q tests/test_icassp2027_submission_bundle.py
```

执行 provenance 为 commit `7974f10179a8973875bec9f301b8a5f84477d860`；验证可在
其 descendant 上运行。`82:131` 已永久 burn，不得使用报告中的历史 regeneration
命令重跑。

## Open Issues

- state/covariance projection 工作只构成部分背景支撑，论文不得声称 exhaustive
  novelty search 或 `first`。
- ICASSP 2027 已确认 single-anonymous、4+1 页、funding/COI 与 ethics requirements；
  作者已于 2026-07-11 确认本稿无 funding，且全体作者无相关 financial/nonfinancial interests。
- 官方 detailed submission/template 链接截至 2026-07-11 返回 404；上线后必须
  对比官方 `spconf`/模板并重新运行 PDF gate。
- 作者需决定是否公开代码与冻结 MAT/CSV/MD，以及可写入论文的稳定 URL。

## Recommendation

以高置信度执行 Story A：`custom receiver factorization -> sender-side projection -> conditional
receiver-field equivalence -> typed message -> frozen paired evidence`。删除所有 dynamic/
event-trigger superiority 与 symmetrized connectivity 主张。该推荐现已由 C1--C8
支持并完成实施，证据包 gate 为 `implementation pass`，funding/COI 声明 gate 亦已通过。
最终 submission pass 仅待 official paper kit 上线后完成模板终检。
