# GCE 论文与可复核源码

**Guarded Current-Evidence Fusion for Cooperative Multitarget Tracking**

版面约束为正文完整七页，Ack/Ref 单独一页；四幅正文矢量图、四张表和三十一条参考文献。
本版补入已完成的扩展数据和固定强度复核，并据结果收紧结论。未提交会议。

## 阅读入口

- 论文：`output/pdf/icra2027_draft.pdf`
- 可移植源码包：`output/icra2027_review_source.zip`
- 本次 43 段 V2V、五段 V2X 与七点固定强度修订：`FOLLOWUP_REVISION_CN.md`
- 扩展证据与完整来源：`source_data/followup_evidence.json`、`source_data/admission_followup/`
- 审稿修改、全部新数据与敏感性结果：`REVIEW_REVISION_CN.md`
- 新增实验数值：`source_data/reviewer_evidence.json`
- 来源快照和协议：`source_data/reviewer_revision/`
- 原有 25 段数值：`source_data/gaussian_paper_evidence.json`
- 固定输入与断链阶段：`source_data/mechanism_analysis.json`

## 本轮结果应如何理解

以下比较使用已参与方法开发的 25 个完整序列，不能当作冻结后的独立测试。

| 方法 | 可靠 OSPA（m） | 间歇 OSPA（m） |
| --- | --- | --- |
| No-age KLA | 3.702 | 3.988 |
| Scalar | 3.526 | 3.832 |
| Guarded Scalar | 3.508 | 3.793 |
| Fixed Ratio (0) | 3.672 | 3.956 |
| GCE | 3.456 | 3.761 |

Guarded Scalar 使用与 GCE 相同的曲率保护，并运行自己的完整递归。
GCE 对它的均值优势较小；校正×曲率交互的两个区间均包含零。
完整七点固定强度搜索由旧九段开发数据选出 η=0；原 η=0.25 结果仍保留。

新增官方 validation 有三个成对片段、748 帧，来自两个原始记录。
仅 409 帧的那个记录此前缺席；GCE 在其原主配置评估中反而逊于 No-age 和 Scalar。
扩展到全部 43 个去重片段后，GCE 相对 No-age 的平均改善为 5.8%/4.4%，
但录制等权 GCE−GS 在间歇链路下的区间跨零。五段 V2X-Real、619 对帧的两种条件
均未优于 No-age。扩展 V2V 包含开发数据，不能称为独立验证。
真实位姿补偿和全部 pD 敏感性结果
也同时保留，未按新结果重选 GCE。相关高斯控制则显示，即使全部 PSD 检验通过，
名义 95% 区域仍可明显欠覆盖。

通信图比较精度与字节开销。相对完整高斯包，编码节省分片字节 11.3%/10.8%；
相对 No-age，仍增加 raw 字节 17.3%/21.5% 和分片字节 4.1%/7.0%。
这是规定记账模型下的字节变化，未测量实际无线时延、容量或能耗。

## 图表与来源

- Fig. 1：`figures/intro.svg`；生图母版、提示词及可编辑复刻记录在 `intro_design/`。
- Fig. 2：`figures/overview.svg`，保留连续场景、两条融合分支及递归反馈。
- Fig. 3：`figures/gaussian_robustness.svg`，包含全部四个 pD、两种通信及五个相关系数。
- Fig. 4：`figures/gaussian_communication.svg`，保留相对 No-age 的额外成本。
- Table I/II：原 25 段完整基线与校正×曲率控制；Table III：同一批数据的通信成本；Table IV：43 段 V2V 与五段 V2X。
- 附带矢量图：`gaussian_paired`、`gaussian_components`、`gaussian_sequence_differences`、`gaussian_phases`。

每幅图有 SVG、矢量 PDF 和 PNG。旧逐序列图保留全部点；固定输入的四个替换
和全部断链阶段仍在源码数据内。它们与新增完整递归 GS 回答的问题不同。

## 构建

需要 Tectonic、NumPy、Matplotlib、Pillow、PyMuPDF 和 pypdf。在本目录执行：

```sh
python3 build.py --regenerate
```

脚本会探测可用 Python，也可用 `PAPER_PLOT_PYTHON`、`PAPER_PDF_PYTHON` 指定。
省略 `--regenerate` 可复用已经生成的图表。命令重建数值、表格和图形，编译 PDF，
核对完整七页正文与第八页 Ack/Ref、字体、浮动图表、引用/DOI 和来源数值，再生成 ZIP。

解压后在 `icra2027` 目录运行相同命令即可重建图表和论文，无需原始 MATLAB 轨迹。
完整跟踪重放另依赖研究仓库的 `trials/icra_reviewer_revision/`、MATLAB 与公开原始数据。
数 GiB 的完整后验轨迹保留在本地，不写入 Git 或论文 ZIP；随包清单记录其完整 SHA-256。
原算法、检测器和校准保持情况见 `source_data/reviewer_revision/REPLAY_ACCEPTANCE.json`。
扩展实验的 458 份轨迹、187,444 个机器人帧的验收记录见
`source_data/admission_followup/icra_admission_final/REPLAY_ACCEPTANCE.json`。
其独立汇总核查器随源包运行，复算 854 行结果和 180 项配对比较。

## 验收边界

- 自动检查：`output/qa/artifact_qa.json`
- 逐页视觉检查：`output/qa/visual_review.md`
- 独立目录重建：`output/qa/portable_rebuild.json`
- ZIP 文件清单：`output/review_manifest.json`
- 文献来源：`LITERATURE_SCOPE.md`

这些是代码/制品检查与另一实现路径的数值复算，不表示第三方评审、作者批准或投稿完成。
稳定泛化、完整检测器训练独立性、真实跟踪协方差一致性及实测无线性能仍未建立。

官方 `ieeeconf.cls` 与 `IEEEtran.bst` 未修改，作者栏留空，AI 使用在 Ack 中披露。
页数约束依据此前核对的 [ICRA 2027 官方 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)，
最终文件状态以匹配 PDF 标识的验收记录为准。
