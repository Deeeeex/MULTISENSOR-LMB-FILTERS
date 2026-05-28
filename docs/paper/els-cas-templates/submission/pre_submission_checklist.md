# Information Fusion 投稿前 Checklist / TODO

Updated: 2026-05-20

Target journal: Information Fusion

Manuscript root: `docs/paper/els-cas-templates`

Status legend: `[x]` 已完成或当前检查通过；`[ ]` 投稿前仍需处理；`[~]` 最终提交前需人工确认。

## Blocking TODO

- [ ] 替换主稿作者占位符：`manuscript.tex` 中的 `Author Name`、`author@example.com`、`Affiliation Placeholder`、地址、城市、邮编、州/省、国家。
- [ ] 确认 short author 信息：`manuscript.tex` 中的 `\shortauthors{Author}` 需要与最终作者列表一致。
- [ ] 将参考文献和正文引用改为期刊要求的编号引用格式。当前仍是 `\usepackage[authoryear,longnamesfirst]{natbib}` 和 `\bibliographystyle{cas-model2-names}`。
- [ ] 将 CRediT 中的 `First Author`、`Second Author`、`Third Author` 替换为最终作者姓名，并确认贡献分配。
- [ ] 完成 competing interest 声明；用 Elsevier declarations tool 生成最终 Word 文件并在上传步骤作为单独文件上传。
- [ ] 将 funding 中的 `[Funding Agency Name]` 和 `[Grant Number]` 替换为真实基金信息；如果无资助，替换为无资助声明。
- [ ] 人工确认 generative AI disclosure 是否准确覆盖论文准备过程，并由全体作者接受。
- [ ] 决定 data/code availability 的最终口径：公开仓库/Zenodo/OSF DOI，或保持 corresponding author on reasonable request。
- [ ] 在 cover letter 中补齐 `[Date]`、`[Corresponding Author Name]`、affiliation、email。
- [ ] 人工确认 previous publication / preprint / concurrent submission 状态；如果存在预印本、会议版、学位论文或重叠稿件，需要在 cover letter 中披露。

## 已准备的投稿材料

- [x] 主稿源文件：`manuscript.tex`。
- [x] 主稿章节：`sections/*.tex`。
- [x] 参考文献库：`paper-refs.bib`。
- [x] 论文图件：`figs/paper-figure*.pdf`、`figs/paper-figure*.png`、必要的 SVG 源文件。
- [x] Highlights 独立 editable 文件：`submission/highlights.txt`。
- [x] Cover letter 草稿：`submission/cover_letter_information_fusion.md`。
- [x] 三个上传入口文件拆分目录：`submission/upload_files/`。
- [x] 投稿声明拆分草稿目录：`submission/declarations/`。
- [x] 投稿声明源文件：`sections/declarations.tex`。
- [x] 当前 PDF 可生成：`manuscript.pdf`。
- [ ] Graphical abstract：当前未准备。Information Fusion 页面通常是 encouraged 而非必需；如果准备，应使用非生成式 AI 的代码/手工图件，并保证可编辑源文件可追溯。
- [ ] Supplementary / reproducibility package：当前还没有统一打包目录；投稿前应决定是否随稿提交。

## 当前合规状态快照

- [x] Abstract 长度当前约 206 词，低于 250 词上限。
- [x] Highlights 当前为 3 条，字符数分别为 69、67、67，满足每条不超过 85 字符。
- [x] Highlights 已从主稿源文件中拆出，主稿不再内嵌 `highlights` 环境。
- [x] Acknowledgements 已删除。
- [x] Competing-interest 声明已从主稿拆出；正文保留 CRediT、funding、data availability 和 generative AI disclosure。
- [x] 当前 BibTeX 输出引用 43 篇，低于 50 篇上限。
- [x] 当前 build 日志显示 PDF 为 27 页，处于 research article 10-35 页范围内。
- [x] 当前 build 日志未发现 undefined citation 或 undefined reference。
- [~] 当前 build 仍有 overfull/underfull box 和 hyperref warning；这些不是阻断项，但最终投稿前应再快速扫一遍 PDF。
- [ ] 正文引用格式仍需从 author-year 改成 numbered citation。
- [~] 图件来源需要人工确认：正式投稿图不应使用生成式 AI 创建或修改。

## 实验证据与可复现材料 TODO

- [x] 主 20-trial tiered heterogeneous packet-loss 结果已进入主稿。
- [x] PD-weighted GA 和 FI-weighted GA 外部动态权重 baseline 已进入主稿。
- [x] FID-FIA baseline、Balanced mode、Cardinality-critical mode 已进入主结果表。
- [x] local safeguards 已进入主稿：local E-OSPA、local RMSE、local cardinality error。
- [x] runtime/cost 已作为第三评价轴进入主稿。
- [x] communication-level robustness probe 已作为表格进入主稿。
- [ ] 归档主结果脚本、seed、报告和 PDF 对应 commit hash。
- [ ] 决定公开代码版本：建议做一个 release tag，并在 data availability 中写明。
- [ ] 如公开数据/结果包，至少包含 `RUN/GA` 中支撑主表、runtime、ideal-communication、communication-level robustness 的报告文件。
- [ ] 确认 `.mat`、大型输出、临时文件是否需要进入公开包；不建议把未说明用途的中间输出直接随稿提交。

Recommended evidence files to keep traceable:

- `RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md`
- `RUN/GA/Del_GA_TIERED_LINK_ABLATION_N20_SEED1_20260520_001252.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED1_20260515_105137.md`
- `RUN/GA/GA_IDEAL_COMM_MAIN20_PAIRED_20260507.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED31_20260520_111403.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED41_20260520_112605.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED51_20260520_113506.md`
- `RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED61_20260520_114216.md`

## Cover Letter 最终检查

- [x] 已回答是否与 state of the art 对比。
- [x] 已列出 5 篇关键参考文献。
- [x] 已说明不使用公共真实数据集，实验为 deterministic synthetic simulation。
- [x] 已说明验证指标：OSPA consensus error、matched localization disagreement、cardinality dispersion、local E-OSPA、local RMSE、local cardinality error、runtime。
- [x] 已说明 main claim 及其意义。
- [x] 已给出主实验、local safeguards、robustness probe、runtime supplement 作为证据。
- [x] 已说明最相关工作及差异。
- [ ] 仍需人工确认前期发表、预印本、会议版、学位论文、并投状态。
- [ ] 如最终提供公开代码/数据 DOI，需要同步写入 cover letter。

## 投稿包打包清单

Include:

- [ ] `manuscript.tex`
- [ ] `sections/*.tex`
- [ ] `paper-refs.bib`
- [ ] `cas-sc.cls`
- [ ] `cas-common.sty`
- [ ] bibliography style file required by the final reference mode
- [ ] `figs/paper-figure*` files actually used by `manuscript.tex`
- [ ] `submission/highlights.txt`
- [ ] `submission/upload_files/manuscript.pdf`
- [ ] `submission/upload_files/cover_letter.txt`
- [ ] `submission/upload_files/highlights.txt`
- [ ] finalized cover letter text
- [ ] optional supplementary/reproducibility archive, if selected

Exclude from journal source upload unless explicitly needed:

- [ ] Elsevier sample files: `cas-*-sample.*`, `cas-*-template.tex`
- [ ] template documentation under `doc/`
- [ ] original sample figures such as `figs/cas-*`
- [ ] generated build artifacts: `.aux`, `.bbl`, `.blg`, `.log`, `.out`, `.xdv`, `.fls`, `.fdb_latexmk`
- [ ] `.DS_Store`
- [ ] uncurated `outputs/` and ignored `.mat` files
- [ ] exploratory reports that are not cited or included in the reproducibility package

## Final QA Commands

Run from `docs/paper/els-cas-templates` after all metadata and declarations are final:

```bash
./build.sh
```

Check for unresolved references or citations:

```bash
rg -n "undefined|Citation .* undefined|Reference .* undefined|There were undefined" manuscript.log
```

Check highlights character lengths:

```bash
awk '{ print length($0) ":" $0 }' submission/highlights.txt
```

Check abstract word count approximately:

```bash
sed -n '/\\begin{abstract}/,/\\end{abstract}/p' manuscript.tex | tr -cs '[:alnum:]-' '\n' | rg -v '^(begin|end|abstract)$' | wc -l
```

Check reference count:

```bash
rg -n "bibitem" manuscript.bbl | wc -l
```

Check source package does not include generated or sample-only files before upload:

```bash
find . -maxdepth 3 \( -name '*.aux' -o -name '*.log' -o -name '*.blg' -o -name '*.out' -o -name '*.xdv' -o -name '.DS_Store' \) -print
```
