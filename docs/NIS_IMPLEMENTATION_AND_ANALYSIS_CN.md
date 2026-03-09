# NIS 实现、耦合性分析与解耦方案

本文档说明当前工程中 NIS 的实现方式、它与协方差因子的数学耦合关系，以及本次改造后为何更适合作为“统计一致性惩罚项”使用。

## 1. 当前工程中的两个相关因子

在自适应融合权重中，当前主要有两个和估计不确定性相关的因子：

1. `covScore`
   - 实现位置：`multisensorLmb/computeAdaptiveFusionWeights.m`
   - 形式上近似为：

$$
\mathrm{covScore}_j \propto \frac{1}{\mathrm{tr}\!\left(P_j^+\right)}
$$

其中，$P_j^+$ 是第 $j$ 个节点 measurement-updated 分布经 m-projection 后的协方差。

2. `innovationPenalty`
   - 实现位置：`multisensorLmb/generateLmbSensorAssociationMatrices.m`
   - 由 NIS 统计构造。

## 2. NIS 的基本定义

对某个量测 $z$，创新向量与创新协方差分别为：

$$
\nu = z - \hat z
$$

$$
S = C P^- C^\top + R
$$

其中：

- $P^-$：预测协方差
- $R$：量测噪声协方差
- $S$：创新协方差

对应的 NIS 定义为：

$$
\epsilon = \nu^\top S^{-1} \nu
$$

若模型一致，则：

$$
\epsilon \sim \chi^2(d)
$$

其中 $d$ 为量测维度，因此：

$$
\mathbb{E}[\epsilon] = d
$$

把 NIS 按维度归一化后：

$$
\bar \epsilon = \frac{\epsilon}{d}
$$

理论上其期望接近 $1$，而不是“越小越好”。

## 3. 为什么 NIS 和协方差强耦合

### 3.1 结构性耦合：二者共享同一个 $S$

NIS 里直接出现了：

$$
\epsilon = \nu^\top S^{-1} \nu
$$

而卡尔曼更新中：

$$
K = P^- C^\top S^{-1}
$$

$$
P^+ = P^- - P^- C^\top S^{-1} C P^-
$$

可见：

- NIS 依赖 $S^{-1}$
- 更新后的协方差 $P^+$ 也依赖 $S^{-1}$

因此，`NIS` 和 `covScore` 在数学上共享同一个核心对象

$$
S = C P^- C^\top + R
$$

这不是经验相关，而是公式层面的同源耦合。

### 3.2 一维情形下的直观看法

在一维下，设 $C = 1$，则：

$$
S = P^- + R
$$

$$
\epsilon = \frac{\nu^2}{P^- + R}
$$

$$
P^+ = \frac{P^- R}{P^- + R}
$$

若固定同一个残差 $\nu$，则有：

$$
\frac{\partial \epsilon}{\partial P^-}
= -\frac{\nu^2}{(P^- + R)^2} < 0
$$

说明预测协方差 $P^-$ 越大，NIS 越小。

另一方面，若融合权重使用

$$
\mathrm{covScore} \propto \frac{1}{P^+}
$$

则 $P^-$ 增大时，`covScore` 往往下降。

这意味着：

- 协方差变大时，`covScore` 会惩罚该节点
- 但传统的单调 NIS 映射又会因为 NIS 变小而“奖励”该节点

两者都受同一个协方差变化驱动，但被解释成两个看似独立的因子，这就是工程上最麻烦的耦合来源。

### 3.3 旧设计中的“双重计数”问题

旧版本对 NIS 的映射是：

$$
\mathrm{score} = \frac{1}{1 + \mathrm{nisAgg}}
$$

或：

$$
\mathrm{score} = \exp\!\left(-\frac{1}{2}\frac{\mathrm{nisAgg}}{d}\right)
$$

这两种映射都有一个共同特征：NIS 越小，分数越高。

但从统计意义上讲，NIS 的正确含义不是“越小越好”，而是“是否与理论卡方分布一致”。因此旧设计会把“协方差变大导致 NIS 变小”误当成更高质量，从而放大了与 `covScore` 的耦合。

## 4. 本次解耦的核心思路

本次不再把 NIS 当作“第二个连续质量分数”，而是改成“统计一致性惩罚项”。

### 4.1 先聚合，再归一化

先对传感器内量测的 NIS 做聚合：

- `robust NIS`：优先用分位数聚合，默认使用 $0.7$ 分位；否则退回 `median`
- 普通 `NIS`：使用 `mean`

得到：

$$
\mathrm{nisNorm} = \frac{\mathrm{nisAgg}}{d}
$$

### 4.2 不再奖励“小 NIS”，只惩罚“偏离一致性区间”

设定一个基于卡方分布的中心置信区间：

$$
\mathrm{lower} = \frac{\chi^2_d\!\left(\frac{1-c}{2}\right)}{d}
$$

$$
\mathrm{upper} = \frac{\chi^2_d\!\left(1-\frac{1-c}{2}\right)}{d}
$$

其中，$c$ 是一致性置信水平，当前默认：

$$
\mathrm{nisConsistencyConfidence} = 0.5
$$

然后定义：

- 若 $\mathrm{nisNorm} \in [\mathrm{lower}, \mathrm{upper}]$，则不惩罚
- 若落在区间外，则按对数距离惩罚

即：

$$
\mathrm{deviation} = 0, \qquad \mathrm{nisNorm} \in [\mathrm{lower}, \mathrm{upper}]
$$

$$
\mathrm{deviation} = \log\!\left(\frac{\mathrm{lower}}{\mathrm{nisNorm}}\right), \qquad \mathrm{nisNorm} < \mathrm{lower}
$$

$$
\mathrm{deviation} = \log\!\left(\frac{\mathrm{nisNorm}}{\mathrm{upper}}\right), \qquad \mathrm{nisNorm} > \mathrm{upper}
$$

当前实现使用不对称软惩罚：

$$
\mathrm{innovationPenalty}
= \exp\!\left(
    - \beta_{\text{low}} \, \mathrm{dev}_{\text{low}}^{p_{\text{low}}}
    - \beta_{\text{up}} \, \mathrm{dev}_{\text{up}}^{p_{\text{up}}}
\right)
$$

其中：

$$
\mathrm{dev}_{\text{low}} = \log\!\left(\frac{\mathrm{lower}}{\mathrm{nisNorm}}\right), \qquad \mathrm{nisNorm} < \mathrm{lower}
$$

$$
\mathrm{dev}_{\text{up}} = \log\!\left(\frac{\mathrm{nisNorm}}{\mathrm{upper}}\right), \qquad \mathrm{nisNorm} > \mathrm{upper}
$$

当 $\mathrm{nisNorm}$ 落在区间内时，上下两个偏离量都为 $0$。

当前默认设置为：

$$
\mathrm{nisConsistencyConfidence} = 0.7
$$

$$
\beta_{\text{low}} = \mathrm{nisPenaltyLowerScale} = 1.0
$$

$$
\beta_{\text{up}} = \mathrm{nisPenaltyUpperScale} = 4.0
$$

$$
p_{\text{low}} = \mathrm{nisPenaltyLowerPower} = 2.0, \qquad
p_{\text{up}} = \mathrm{nisPenaltyUpperPower} = 2.0
$$

这表示：

- 对“过小 NIS”的惩罚较弱
- 对“过大 NIS”的惩罚较强
- 区间边界附近用平方型软惩罚，避免过于尖锐的权重突变

`robust NIS` 还会额外加一个下界：

$$
\mathrm{innovationPenalty} \ge \mathrm{nisPenaltyMin}
$$

## 5. 改造后为什么耦合显著减弱

### 5.1 语义解耦

改造后两个因子的职责明确分开：

- `covScore`：度量“估计是否集中、是否精确”
- `innovationPenalty`：只度量“创新统计是否一致”

也就是说：

- `covScore` 仍然是质量项
- `NIS` 不再是质量项，而是 gate / penalty

### 5.2 不再奖励“小 NIS”

这是最关键的一点。

旧设计里，如果节点把协方差报得偏大，那么：

- $S$ 变大
- $\epsilon = \nu^\top S^{-1} \nu$ 会变小
- 节点可能被错误奖励

新设计里：

- $\mathrm{nisNorm}$ 过小同样会被视为偏离一致性区间
- 因而不会再因为“协方差膨胀导致 NIS 变小”而得到额外好处

这就切断了“协方差变大 $\rightarrow$ NIS 变小 $\rightarrow$ 权重反而上升”这条错误通路。

### 5.3 仍保留 NIS 的价值

虽然做了解耦，但 NIS 仍然有用，因为：

- 当创新显著偏大时，它能识别滤波不一致
- 当创新异常偏小时，它也能识别过度保守或协方差膨胀
- 它不再和 `covScore` 一起重复表达“精度”，而是专门表达“一致性”

## 6. 当前工程中的实现形态

实现后，自适应权重的结构为：

$$
\mathrm{baseScore} = \mathrm{mask} \cdot \mathrm{covScore} \cdot \mathrm{linkQuality}
$$

$$
\mathrm{rawScore} = \mathrm{baseScore} \cdot \mathrm{innovationPenalty} \cdot \mathrm{historyScore}
$$

其中当前为了专门研究 NIS 解耦，默认先将：

$$
\mathrm{useHistory} = \mathrm{false}
$$

这样当前实验里更接近：

$$
\mathrm{rawScore} = \mathrm{mask} \cdot \mathrm{covScore} \cdot \mathrm{linkQuality} \cdot \mathrm{innovationPenalty}
$$

这能更清楚地观察：

- `covScore` 负责精度
- `innovationPenalty` 负责一致性约束

## 7. 当前默认配置

推荐的当前默认配置为：

```matlab
model.adaptiveFusion.useNIS = true;
model.adaptiveFusion.robustNIS = true;
model.adaptiveFusion.nisQuantileEnabled = true;
model.adaptiveFusion.nisQuantile = 0.7;
model.adaptiveFusion.nisConsistencyConfidence = 0.7;
model.adaptiveFusion.nisPenaltyScale = 4.0;
model.adaptiveFusion.nisPenaltyMin = 0.3;
model.adaptiveFusion.nisPenaltyLowerScale = 1.0;
model.adaptiveFusion.nisPenaltyUpperScale = 4.0;
model.adaptiveFusion.nisPenaltyLowerPower = 2.0;
model.adaptiveFusion.nisPenaltyUpperPower = 2.0;
model.adaptiveFusion.nisEmaEnabled = true;
model.adaptiveFusion.nisEmaAlpha = 0.7;

model.adaptiveFusion.useHistory = false;
```

## 8. 后续更值得验证的方向

在当前“先解耦 NIS，再谈 history”的阶段，更值得做的实验是：

1. 比较 `w/o NIS -> robust NIS -> NIS` 在 `useHistory = false` 下的 20 次 Monte Carlo 结果
2. 观察 decoupled NIS 是否比旧版单调 NIS 更稳定
3. 在此基础上，再决定是否把 `historyScore` 重新加回来

## 9. Decoupled NIS 的 20 次 Monte Carlo 结果（2026-03-09）

对比脚本：

- `RUN/GA/runMultisensorFilters_formation_4plus4_NISCompare.m`

本轮对比固定：

- `useHistory = false`
- `nisQuantileEnabled = true`
- `nisQuantile = 0.7`
- `nisConsistencyConfidence = 0.7`
- `nisPenaltyScale = 4.0`
- `nisPenaltyMin = 0.3`
- `nisPenaltyLowerScale = 1.0`
- `nisPenaltyUpperScale = 4.0`
- `nisPenaltyLowerPower = 2.0`
- `nisPenaltyUpperPower = 2.0`
- `nisEmaEnabled = true`

比较顺序：

- `w/o NIS`
- `robust NIS`
- `NIS`

20 次均值结果如下：

$$
\text{Comprehensive (OSPA) consensus: } 1.811 \rightarrow 1.811 \rightarrow 1.898
$$

$$
\text{Position (RMSE) consensus: } 3.173 \rightarrow 3.159 \rightarrow 3.337
$$

$$
\text{Cardinality consensus: } 0.214 \rightarrow 0.209 \rightarrow 0.234
$$

当前可以得到两个直接结论：

1. 调整后的 `robust NIS` 已经把 OSPA 从上一版的轻微退化拉回到基线附近，同时仍保留一点 RMSE / cardinality 收益。
2. 普通 `NIS` 仍然明显不稳定，说明仅仅做语义解耦还不够，鲁棒聚合与较缓的惩罚结构仍然有必要。

也就是说，当前最合理的 GA 默认选择仍然是：

- `robustNIS = true`
- `useHistory = false`

对应报告文件：

- `RUN/GA/GA_NIS_COMPARE_20260309_152647.md`
