# GCE 论文与源码

**Guarded Current-Evidence Fusion for Cooperative Multitarget Tracking**

本版按 ICRA 2027 官方模板整理为 **正文完整 7 页 + Ack/Ref 单独 1 页**。
正文包含五幅可编辑矢量图、四张表；第 8 页集中放置 AI 使用披露与三十条参考文献。
尚未上传会务系统。

## 阅读入口

- 论文：`output/pdf/icra2027_draft.pdf`
- 可移植源码包：`output/icra2027_review_source.zip`
- 首页 Intro 图：`figures/intro.svg`（场景、车辆、箭头为矢量路径，文字和公式可编辑）
- Intro 母版、三次生图记录与复刻检查：`intro_design/README.md`
- 方法总图：`figures/overview.svg`
- 逐序列双条件 OSPA 收益图：`figures/gaussian_paired.svg`
- 精度与通信开销图：`figures/gaussian_communication.svg`
- 断链阶段比较：`figures/gaussian_phases.svg`
- 逐序列视图：`figures/gaussian_sequence_differences.svg`、`figures/gaussian_components.svg`
- 完整数值：`source_data/gaussian_paper_evidence.json`
- 新增机制与阶段诊断：`source_data/mechanism_analysis.json`

每幅图均提供 SVG、矢量 PDF 和 PNG。Fig. 3 的每个点对应同一序列在两种通信条件
下的 OSPA 收益，两个对照各保留全部 25 个序列，共 50 个双坐标点。其他两幅
逐序列视图保留全部六种几何对照及消融比较的观测值与区间。

## 本版重点

首页 Intro 图采用更丰富的合作驾驶场景，参考两篇 ICRA 论文的首页表达方式，
经过三版内置生图迭代后，保留母版构图和对象坐标复刻为 SVG。完整保留车辆、
共享历史轨迹、感知扇区、融合对比及联合输出，文字和公式重新排为可编辑内容。
图中场景是概念插画。生图记录与逐路径几何核对结果随源码提供。
方法部分推导共同先验条件下的有效似然
指数：接纳权重将当前似然指数从保守池的权重恢复到不超过 1，同时保留一份先验。
共同先验的解释与实际不同历史下的可计算规则明确区分。

新增两类完整诊断，方案记录于 `ANALYSIS_PROTOCOL.md`：按精确断链边界比较全部
25 段的三个阶段；固定 GCE 的输入、匹配、基底权重与曲率接纳结果，评估空间均值
和存在积分的全部四种组合。后者只替换当前输出，不将替代输出反馈到下一帧。
重新计算了 150 份既有轨迹的 67,212 个机器人—帧 OSPA，并验证 19,235 个接收帧
中的完整组合重现原 GCE 输出。原始跟踪结果和算法实现均未改动。

三十条参考文献保留完整期刊名称，并显示已有核验记录中的 DOI，方便定位原文。
官方 BibTeX、来源记录、类文件和参考文献样式均随包提供。

全文围绕“继承后验与当前证据分离，再联合归一化存在概率和空间密度”展开。
摘要、引言和结果首先说明方法收益，方法部分保留完整推导；重复数字、运行过程
描述和重复的边界声明已精简。数据来源与开发范围集中说明在实验设置中。

主图沿用连续的机器人场景、来源密度、资格判断、两条融合分支及反馈路径，
统一 Arial 字体、线条和颜色，并简化机器人图形。Fig. 3 以 No-age KLA 和 Scalar
为两个核心对照，展示逐序列收益的大小、跨通信条件的一致性和符号反转；两种条件
均改善的序列分别为 20/25 和 14/25。颜色与符号共同编码结果，坐标不加抖动。
通信图将两种条件的全部八个均值放在同一坐标系，显示完整高斯包到精确编码包的移动。

Table I 按各列实际最优值加粗。Table II 将 Scalar 参考、三个单项消融与完整
GCE 分开，完整方法置于最后。Table III 比较四种表示的原始与分片开销。
Table IV 展示固定输入下的全部四种均值/积分组合，Fig. 5 展示三个通信阶段的配对区间。

## 方法与结果

GCE 在保守后验基底上补入当前 Bernoulli 更新与预测的比值。关联、检测分数、
漏检证据与曲率条件决定可接纳的权重。同一比值同时修改高斯空间密度与存在概率，
由重新计算的空间积分保持两者一致。精确零向量编码省略零增量，保留所有浮点值。

主实验使用 25 个完整序列、两种通信条件和相同的 score-aware 本地模型。
序列等权 OSPA（m，越低越好）如下：

| 方法 | 可靠通信 | 间歇通信 |
| --- | ---: | ---: |
| No-age KLA | 3.702 | 3.988 |
| Recency | 3.672 | 3.952 |
| Scalar | 3.526 | 3.832 |
| GCE | **3.456** | **3.761** |

GCE 相对 No-age KLA 改善 6.7% / 5.7%，相对 Recency 改善 5.9% / 4.8%；
漏检、误报代价和共同匹配目标定位同时改善。对五种几何控制及 Scalar，
两种条件的配对 OSPA 区间均低于零。

Scalar 是存在概率修正参考，未使用高斯比值或曲率检验；三个单项消融分别移除
曲率保护、保守历史开关和正证据分数约束。历史项影响较小；分数约束主要控制
误报，其移除也会减少漏检。相应总 OSPA 区间的具体结果保存在逐序列视图和数据中。

新增诊断中，间歇通信下 GCE 减 No-age KLA 的 OSPA 差值，在断链前、断链中、
恢复后分别为 −0.266、−0.076、−0.264 m。前后两个区间低于零，断链期间的区间
包含零。固定输入下，空间均值更新的约 0.003 m 即时收益具有较稳定的方向；
联合更新相对基底组合的均值差为 −0.014/−0.010 m，对应区间包含零。
这些是已有开发轨迹上的补充分析，不是新采集数据或新增递推实验。

编码后的跟踪结果与完整高斯表示一致。相对完整高斯包，原始字节减少
20.3% / 21.1%，分片与控制开销减少 11.3% / 10.8%；相对 No-age KLA，
编码后的分片开销增加 4.1% / 7.0%。

实验采用已发布的 V2V4Real/DMSTrack 检测、二维两车跟踪和模拟通信。
九个开发序列与二十五个主实验序列均参与过方法开发，主实验检测来自检测器的
训练划分。MIL-AM 和 TC 是共享输入下的适配比较。新路线、不同检测器和实际
无线网络上的迁移仍需后续实验。

## 构建

需要 Tectonic、NumPy、matplotlib、Pillow、PyMuPDF 和 pypdf。脚本自动选择可用 Python，
也可通过 `PAPER_PLOT_PYTHON` 和 `PAPER_PDF_PYTHON` 指定解释器。

```sh
cd /Users/dex/Desktop/Code/icra27/papers/icra2027
/Users/dex/miniconda3/bin/python3 build.py --regenerate
```

命令从十五份主实验快照和一份逐帧诊断快照重建数值、表格和全部矢量图，编译 PDF，
完成自动检查并生成 ZIP。检查强制要求正文七页、Ack/Ref 位于第八页、正文末页两栏
接近页底，并核对字体、图表顺序、引用与 DOI、数值、全部诊断组合和消融行顺序。
省略 `--regenerate` 可直接使用已生成图表。首次使用 Tectonic 可能需要下载 TeX 包。

解压后在 `icra2027` 目录运行相同命令即可重建，无需原始 MATLAB 输出。
在原始仓库内，构建还会核对对应实验文件。构建过程只处理论文材料。
诊断快照由 `prepare_mechanism_analysis.py --extract` 首次提取，提取需要 SciPy
和完整研究仓库；常规构建直接使用随包保存的逐帧分数，不需要 SciPy。

- 自动检查：`output/qa/artifact_qa.json`
- 逐页视觉检查：`output/qa/visual_review.md`
- 独立目录重建记录：`output/qa/portable_rebuild.json`
- 打包文件清单：`output/review_manifest.json`
- 数值快照来源：`source_data/gaussian_source_manifest.json`
- 引用与外部实现说明：`LITERATURE_SCOPE.md`

完整实验入口位于仓库的 `trials/icra_gaussian_evidence/`、
`trials/icra_gaussian_components/` 和 `trials/icra_gaussian_zero_codec/`；
各目录的协议和结果说明记录具体配置与输出。

## 投稿格式

2026-09-09 核对的 [ICRA 2027 官方 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)
规定总页数不超过八页，采用双盲审稿并披露 AI 生成内容。本稿使用 US Letter、
10 pt 双栏官方模板，作者栏留空；`ieeeconf.cls` 与 `IEEEtran.bst` 保持官方原件。
最终 PDF 的字体、页数、注释及版面检查结果随源码一并提供。
