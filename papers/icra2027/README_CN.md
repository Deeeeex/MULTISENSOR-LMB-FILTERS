# ICRA 2027 完整论文初稿与复现说明

论文：**Observation Recency in Bernoulli Fusion for Intermittent Multirobot Tracking**。
当前 7 页英文完整初稿包含正文、5 幅 SVG/PDF 图、3 张表、参考文献和 AI 使用披露。
正式模板、引用元数据、图表源数据和构建脚本已放在本目录。
这是供作者审阅的完整研究稿；自动检查不等于独立复现、作者批准或投稿系统验收。

2026-09-08 基线复核：历史记录中的 Lineage 是已有历史资格筛选的内部消融，3.1% / 24.7%
是加入时效的增量收益，不能解读为相对公开强基线的优势。外部基线的完整性仍需补足。
机器人融合路线、代码来源和具体候选见 [基线复核报告](ROBOTICS_FUSION_BASELINE_REVIEW_CN.md)。
正文、图例和表格现统一使用 `ER w/o age`，组合空间/存在时效的消融使用 `ER (both)`；
实验文件中的原始方案键保持不变。已清理正文中的种子编号、冻结/审计记录和重复防御性说明。
方法定义、必要实验设置、统计方法和负面结果仍在正文中。

主图按用户选定的一体化生图逐元素重绘，保留布局与配色，并修正公式与连线。
最终矢量图为 `figures/overview.svg`；设计参考、prompt 和说明见
[一体化主图目录](main_figure_integrated/README.md)。

- 阅读：`output/pdf/icra2027_draft.pdf`
- 可移植论文源码包：`output/icra2027_review_source.zip`
- 机器检查：`output/qa/artifact_qa.json`；人工版面记录：`output/qa/visual_review.md`
- 源码入口：`main.tex`、`sections/`、`generated/`、`figures/`
- 数值入口：`source_data/validation_summary.json`、逐种子 CSV
- 完整实验：仓库中的 `trials/icra_reunion_fusion/`；运行日志：`RUN/ICRA/reunion_fusion_v1/`

## 主线与实际结论

场景切换为八个低速移动机器人在局部视野和间歇通信下进行协作目标跟踪。
重逢时区分未观测先验、已有观测来源和直接观测时效；ER 仅将时效用于存在概率，
保留原有空间权重和空间重叠归一化项。近期漏检也计入直接观测机会，中继不刷新本地时效。
固定输入下的空间不变性有简短变分推导，递归定位结果由实验检验。
原有 LMB/KLA 核心未修改，没有增加规划器、控制器、学习系统或真实感知工程。

验证使用 3 个场景家族 × 20 个新种子 × 9 个方案，共 540 次运行。
每个种子共享真值、量测、丢包随机数和路径偏移。下表均为验证集均值：

| 指标 | 分队后重逢 | 反复相遇及目标离场 | 无新目标控制 |
|---|---:|---:|---:|
| ER w/o age OSPA (m) | 2.651 | 1.123 | 0.797 |
| ER OSPA (m) | 2.569 | 0.846 | 0.668 |
| 相对改善 | 3.1% | 24.7% | 16.2% |

两个事件场景的共同目标 RMSE 比值约为 0.9999 / 0.9998，基本不变；同时改变空间和
存在权重的消融则恶化约 16.8% / 3.1%。这支持将直接观测时效放在存在判断上的有限结论。

负面结果已写入正文、图表和讨论：

- ER 的离场后误报平方代价在新种子上为 12.255 m²，Lineage 为 10.785 m²；开发阶段的
  34.5% 改善未复现。配对差值 +1.470 m²，描述性 95% bootstrap 区间 [-0.705, 3.795]。
- 无新目标控制中 ER 误报平方代价为 0.570 m²，FoV 为 0.00375 m²；较低 OSPA 不表示更少误报。
- 两次本地命中确认降低误报，但使反复相遇场景 OSPA 恶化为 1.519 m，且有 8/160 个远端发现
  查询未成功。成功查询的平均时延为 7.63 s，ER 为 2.47 s；失败查询未混入该均值。
- v1 Age-all 通过初始粗筛，但输给更强的 Lineage 对照并恶化定位；v2/v3 组合均未通过后续
  完整平衡门槛。验证后没有改方法、阈值、先验、路径或筛除种子。

贡献限于时效权重在 Bernoulli 融合中的具体放置方式及其权衡。可变融合权重、多视域处理、
存在/空间分解均已有文献；相关工作明确引用，MIL 对照仅为共享标签下的有界实现，
不声称复现完整独立标签多视域系统。创新幅度和机器人适用范围仍需作者审阅判断。

## 必须保留的实验边界

所有机器人共享出生时刻、候选出生区域和标签，位姿及时钟精确已知；运动轨迹预设，
量测为合成二维点。仅有小目标集合、固定噪声/视野/通信范围和适度路径偏移，
不验证任意新生、独立标签匹配、遮挡、定位误差、传感器异构或真实机器人执行。
可靠的中心几何协调器选择每个连通分量的 MST，其控制量计费，估计消息有丢包。
每个方案使用同样的固定包槽；这不是通信节省或分布式网络协议的证明。

Python 用独立的穷举指派实现复算全部 518,400 个验证节点帧，并核验 1,169 个冻结源码散列。
这是对指标实现与来源记录的核对，不是第三方实验验证。区间按 20 个配对 episode 重采样，
不把机器人/帧作为独立样本，不作多重比较调整或确认性显著性宣称。

## 构建论文与图表

需要 Tectonic，以及带 NumPy、matplotlib 的 Python；PDF 检查需要 pypdf 和 PyMuPDF。
脚本自动探测可用 Python，也可用 `PAPER_PLOT_PYTHON`、`PAPER_PDF_PYTHON` 指定解释器。
本机验证环境：MATLAB R2024a；NumPy 2.4.4；matplotlib 3.10.9；Tectonic 0.15.0。

```sh
cd /Users/dex/Desktop/Code/icra27/papers/icra2027
python3 build.py
python3 build.py --regenerate
```

第一条构建命令使用已生成图表；第二条重新生成参考文献、表格、四组数值图和矢量主图。
二者均编译、检查字体/页数/图文边界/引用/数值来源并打包。Tectonic 首次运行可能需要联网
下载 TeX 依赖。固定两次重跑避免 IEEEtran 与 Tectonic 的参考文献稳定性重复警告。
在加载官方类之前指定 T1 编码，使 Tectonic 正确使用 Times 字体及粗体、斜体；
类文件保持原样。末页使用官方类提供的参考文献分栏命令整理版面。
常规 LaTeX 环境也可执行 `pdflatex main; bibtex main; pdflatex main; pdflatex main`，
但这里实际验证的是 Tectonic，不声称验证了另一编译链。

解压源码包后，在解压的 `icra2027` 目录运行相同命令即可。包内的 `source_data/` 支持
图表与论文重建；大体积原始 MAT 输入和逐帧结果保留在版本化仓库中，没有放进论文 ZIP。
此 ZIP 是作者审阅包，包含来源记录及脚本；未作为匿名补充材料向任何网站上传。
源码包重建的具体检查项与页数见 `output/qa/portable_rebuild.json`。
该检查使用同一本机运行时与字体，不替代 MATLAB 实验复跑。

版本节点：`ddd49a4c` 保存初始方向筛选，`72d4e3ef` 保存确认消融及验证协议冻结，
`f712a749` 保存全部新种子验证输入、逐帧结果和审计。论文包作为后续独立提交保存。

## 重新运行实验

完整实验需本仓库的 `common/`、`lmb/`、`multisensorLmb/` 与既有场景辅助函数。
运行器会重写对应结果，建议在独立 checkout/worktree 中复跑并保留已提交结果以便比较。
冻结协议按 v1/v2/v3/v4 保存在实验目录，`VALIDATION_NOTES.md` 单独澄清后验措辞，未改注册文本。

```sh
cd /Users/dex/Desktop/Code/icra27
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_reunion_fusion'); runRobotReunionValidation(2901,2920);" 2>&1 | tee RUN/ICRA/reunion_fusion_v1/validation_replay.log
/Users/dex/miniconda3/bin/python3 trials/icra_reunion_fusion/analyze_validation.py
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_reunion_fusion'); checkReunionFusion; checkConfirmedFusion; checkValidationControls;"
```

日志查看：`tail -f RUN/ICRA/reunion_fusion_v1/validation_replay.log`。
`-singleCompThread` 是本机实测必要选项；默认多线程对这些小矩阵运算明显更慢。

## 模板与格式依据

2026-09-08 核对 [ICRA 2027 官方 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)：
最多 8 页，包括参考文献、致谢及补充内容；双盲；AI 生成内容需要披露。
本稿使用 [Papercept 官方模板](https://ras.papercept.net/conferences/support/tex.php)，
下载压缩包和 SHA-256 记录在 `official_template/download_manifest.json`。
根目录 `ieeeconf.cls` 与 `IEEEtran.bst` 均与下载副本逐字节一致。
作者栏留空，正文以第三人称引用已有工作；已写明 OpenAI Codex 对代码、图表和正文的使用范围。
最终作者身份、AI 披露措辞、与已有/在审稿件的重叠，以及会务系统的实时检查应由作者完成。
