# GCE 论文初稿与复现说明

论文：**Guarded Current-Evidence Fusion for Cooperative Multitarget Tracking**。
本稿围绕已完成的 Gaussian current-evidence 主实验重写，包含完整正文、四幅可编辑
SVG/PDF 图、三张表、十八条正文引用及 AI 使用披露。当前按“篇幅先不管”的要求
保留方法和实验细节；最终页数与版面检查见 `output/qa/artifact_qa.json`。
这是作者审阅稿，尚未向会务系统提交。

## 阅读入口

- PDF：`output/pdf/icra2027_draft.pdf`
- 可移植源码包：`output/icra2027_review_source.zip`
- 方法总图：`figures/overview.svg`；PNG 预览及矢量 PDF 位于同一目录。
- 数值入口：`source_data/gaussian_paper_evidence.json`；十五份实验快照及其来源记录：
  `source_data/gaussian_source_manifest.json`。
- 自动检查：`output/qa/artifact_qa.json`；逐页视觉检查：`output/qa/visual_review.md`；
  解压重建检查：`output/qa/portable_rebuild.json`。
- 完整实验、日志和逐帧结果保存在仓库的 `trials/icra_gaussian_evidence/`、
  `trials/icra_gaussian_components/`、`trials/icra_gaussian_zero_codec/` 及对应 `RUN/` 目录。

## 当前方法和主结果

方法保留 LMB 本地跟踪与已有独立标签对齐，在融合层区分继承的后验与当前局部更新。
先形成保守的存在概率基底，再以关联、检测分数或漏检证据及曲率条件决定当前
Bernoulli 比值的权重。同一比值同时修正高斯空间密度和存在概率，重新计算空间积分。
不合适的源增量整项拒绝；不可积的最终聚合回到基底。共享先验精确情形、零增量、
冲突观测、单源与回退均有数值检查。先验/似然分离本身是已有原理，不能作为首创贡献。

主表使用 25 个完整序列、5,601 帧，两种固定通信条件，共享同一个 score-aware 本地模型。
下表按完整序列等权，单位为 m，越低越好：

| 方法 | 可靠通信 OSPA | 间歇通信 OSPA |
| --- | ---: | ---: |
| No-age KLA | 3.702 | 3.988 |
| Recency | 3.672 | 3.952 |
| Scalar | 3.526 | 3.832 |
| GCE | **3.456** | **3.761** |

GCE 相对 No-age KLA 改善 6.7% / 5.7%，相对 Recency 改善 5.9% / 4.8%。
对 No-age、Recency、Conservative、两个 capped 控制及 Scalar，两种条件下的配对
OSPA 区间都低于零。图中保留每一个序列的点，没有按改进大小剔除序列。
相对 No-age 与 Recency，漏检、误报平方代价和共同匹配目标定位一起改善。
共同目标 RMSE 使用双方都匹配到的同一 robot–time–truth 支持，不能直接混用不同支持的比值。

完整消融保留相反或不确定结果：无曲率保护的 OSPA 区间只有间歇条件排除零；
可靠条件出现两次需要聚合回退的不可积结果。无历史项的贡献很小，两种区间都含零。
去掉正证据分数约束减少漏检、增加误报，总 OSPA 区间也含零。Scalar 的空间修正对照
有主实验支持，但九序列开发子集上的同类区间含零。

零向量编码器省略恰好为零的十五维高斯增量，保留有符号零及所有非零浮点值。
完整 34 序列 × 两种条件的原生回放验证所有跟踪输出和原有诊断完全一致。
主实验相对完整高斯数据包，raw 字节下降 20.3% / 21.1%，含分片与控制量的字节下降
11.3% / 10.8%。相对 No-age，编码后的 raw 仍增加 17.3% / 21.5%，含分片与控制量仍增加
4.1% / 7.0%；不是同等字节预算的精度优势。字节数来自实际回放，不是平均轨迹数估算。

## 结论边界

九个开发序列和主实验的二十五个序列都已参与方法开发。主实验使用检测器训练划分
中的已发布检测；它不是独立测试集。区间按二十五个完整序列重采样 10,000 次，
是这批开发数据的描述，不能把帧或两车当作独立样本。

真实数据回放是两车、二维 car center、40 m 感知域与明确裁剪范围，采用移动 ego 坐标、
固定观测模型及模拟丢包，没有自车运动补偿或机器人实机通信。检测分数拟合使用开发
数据真值，在线过程使用检测及后验；其可分 mark likelihood 是近似。新路线、不同检测器
及独立数据上的迁移效果仍需另外实验。四机器人总图只是机制示意。

MIL-AM 为公共作者稿的标签指派/子空间适配；TC 使用作者原生轨迹函数及共享 Local-LMB
输出，其不回馈本地密度的架构保持不变。这里不声称复现各论文原生协议或三维榜单。
没有通过新的实验验证多机器人实机、规划、定位误差鲁棒性或共识收敛。

## 构建与可移植性

需要 Tectonic、NumPy、matplotlib、PyMuPDF 和 pypdf；脚本自动选择可用 Python。
可通过 `PAPER_PLOT_PYTHON` 和 `PAPER_PDF_PYTHON` 指定解释器。本机已验证的绘图解释器为
`/Users/dex/miniconda3/bin/python3`，Tectonic 为 `/opt/homebrew/bin/tectonic`。

```sh
cd /Users/dex/Desktop/Code/icra27/papers/icra2027
/Users/dex/miniconda3/bin/python3 build.py --regenerate
```

这会从完整实验快照重建文献、所有数字与表格、三幅数值图和一幅连续式主图，编译 PDF，
检查字体嵌入、文本边界、全部浮动体在参考文献之前、引用、550 个主表运行摘要及数值来源，
然后生成源码 ZIP。省略 `--regenerate` 可使用已生成图表直接编译。
Tectonic 首次使用可能需要联网下载 TeX 包。

解压后在 `icra2027` 目录运行同一 `build.py --regenerate`，不需要原始 MATLAB 结果。
在原始仓库构建时会核对实验文件；解压目录使用十五份完整快照和摘要生成图表。
该重建检查验证论文材料可移植，不代表从原始检测重新运行 MATLAB 实验。
`package_review.py` 只收录当前正文实际使用的文件；仓库中未被本文引用的历史实验与旧稿
图表仍保留在 Git，未混入本次源码包。

完整原生实验的入口和已完成阶段分别见：

- `../../trials/icra_gaussian_evidence/README_CN.md`、`PROTOCOL.md` 与 `RESULTS_seen_transfer_CN.md`。
- `../../trials/icra_gaussian_components/PROTOCOL.md` 与 `RESULTS_seen_transfer_CN.md`。
- `../../trials/icra_gaussian_zero_codec/PROTOCOL.md` 与 `RESULTS_seen_transfer_CN.md`。

如需重跑，使用独立 checkout 保存现有结果。本文构建不会启动实验或改变方法参数。
已提交的版本节点：`bab88ce3` 为完整 25 序列主实验，`64449548` 为九序列原生编码验证，
`2c6ee6c5` 为开发组件分析，`955a8fc6` 为完整主实验编码与组件证据。

## 模板与引用

`ieeeconf.cls`、`IEEEtran.bst` 与保存的 Papercept 官方模板逐字节一致。
2026-09-08 核对的 [ICRA 2027 官方 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)
要求完整投稿不超过八页，采用双盲审稿，并披露 AI 生成内容。本稿暂不按投稿页数压缩；
作者栏留空，AI 使用范围已写入致谢。实际提交前仍需作者审阅研究贡献、版本重叠、
披露措辞与最终长度。文献事实与外部复现边界见 `LITERATURE_SCOPE.md`。
