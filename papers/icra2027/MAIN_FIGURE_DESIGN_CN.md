# 主图设计草案：从机器人相遇到两条融合路径

## 一句话结论

机器人相遇时先依据观测历史确定参与融合的信息，再让直接观测时效影响存在判断，
同时保留合格输入的普通空间权重；空间重叠项仍进入存在概率。

## 构图约定（绘图前确定）

- 类型：机制示意图，右侧双路径为视觉中心；左侧机器人场景和中间输入解释动机。
- 后端：沿用现有 Python/matplotlib 流程；输出可编辑文字 SVG、矢量 PDF、300 dpi PNG。
- 成品宽 181 mm、高 89 mm，双栏通栏；正文 7--8 pt，标记 9 pt，白底。
- 三个区域：(a) 相遇但观测历史不同；(b) 同一标签的输入资格；(c) 存在/空间双权重。
- 绿色编码存在概率路径，蓝色编码空间路径，灰色编码机器人及观测历史。
- 此图为明确标注的示意，不使用验证结果、不伪造轨迹/概率数值、不表达硬件验证。

## 每个区域必须讲清什么

**a：机器人动机。** A 过去直接观测过目标，当前在其视野外；B 在当前视野内出现漏检。
两者相遇并交换后验。用过去/当前两幅小场景和交换箭头表达：包刚收到并不刷新
接收机器人的本地直接观测时间。示意图无坐标轴、不按比例，不对应某个实验帧。
这里只画一对机器人来解释相遇机制，八机器人完整轨迹仍由实验场景图承担。

**b：信息资格。** A 的旧观测后验和 B 的近期负证据均有观测来源，进入融合。
U 是同一标签的未观测先验；当前已有具备观测来源的输入，因此 U 被排除。
这一资格处理是现有基线组成，不能用醒目“新模块”标记。缺失标签只在满足可观测
条件时成为存在概率的负证据，不能凭空成为空间密度。P/E 两个支持集合保留差别。

**c：核心机制。** 上方绿色存在路径用 q_j ∝ w_j f(Δ_j)，下方蓝色空间路径用 a_j ∝ w_j。
给出两条融合公式，并用蓝色箭头将 η_a 从空间路径送入存在路径，避免画成完全独立。
文字明确：近期检测和近期漏检均可刷新直接观测机会；空间不变性只针对固定输入、
固定资格和固定普通空间权重，不推广为整个递归滤波器的定位保证。

## 放进论文的建议

建议作为新的 Fig. 1，置于 Introduction 后的通栏顶部，让读者先看到机制。
原场景图后移到 Experimental Protocol；现有两高斯曲线保留为方法段的解析说明，
如果版面紧张，优先合并或缩减解析说明图，而不是删掉负面结果图。
本次先交付独立机制图草案，不改已提交论文的图序和数值。

## Caption draft

**Observation recency at a robot encounter (schematic).**
(a) Robot A retains an older directly observed belief, while robot B has a
recent missed detection. Exchanging a posterior does not refresh the
receiver's local direct-opportunity age. (b) Observation lineage qualifies
both inputs; an untouched prior is excluded when informed inputs exist.
Observable label absence may contribute existence evidence without a
spatial density. (c) Direct-opportunity recency changes existence weights
q, while ordinary spatial weights a are retained on eligible inputs.
The spatial overlap term η_a still enters existence fusion. The fixed-input
spatial invariance does not imply identical recursive localization.

## 审阅风险与检查

不能把漏检画成目标已消失的真值；不能让中继刷新本地时钟；不能把 U 无条件排除；
不能漏掉 η_a 的连接；不能把固定输入空间不变性画成真实实验中的完美定位。
导出后检查全部文字边界、SVG 文字元素、PDF 字体和箭头关系，并按成品尺寸查看。

## 交付与重画

`main_figure_draft/overview.svg` 为可编辑主图；同名 PDF/PNG 可直接预览。
`main_figure_draft/figure.tex` 提供插入片段，当前尚未改动正文图序。
用 `python3 make_main_figure.py` 重画，需要当前已有的 NumPy/matplotlib 环境。
`source_data.json` 明确标注为示意，`qa.json` 记录文字边界与重叠检查。
