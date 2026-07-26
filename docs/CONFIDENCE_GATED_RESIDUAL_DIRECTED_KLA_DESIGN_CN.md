# 支持域门控的定向 KLA 路由：方法重设计

> **2026-07-26 收益归因修正：** 本文所称
> `reliability + w=0.50` “解析骨架”已被证明是固定策略，不是动态解析
> 路由。M24/X36、seeds 7/17/27、完整 160 步中，其 sender map 都恒为
> 每个六节点编队的 `[2,1,1,1,1,1]`；此前的正收益应重新标记为强固定
> 定向基线收益。支持域门控残差又从未覆盖该基线，因此当前既没有学习
> 增益，也没有动态拓扑增益。后续方法与实验门槛见
> `docs/DIRECTED_ROUTING_ATTRIBUTION_CORRECTION_CN.md`。

## 当前结论

第一版 kNN 路由不能作为最终方法。它只在 M24 seed 7、`t=75` 的一个
后验快照上训练，把选边收益和融合权重分别回归，再在预测收益不足时拒绝
通信。M24 的 held-out seed 17 仍有正收益，但 X36 seed 17 只比 local
改善 1.05%。诊断表明，这不是 X36 缺少可利用的信息，而是模型在严重
分布外输入上出现了覆盖塌缩：

| 诊断量 | M24 训练内部 | M24 seed 17 | X36 seed 7 | X36 seed 17 |
|:--|--:|--:|--:|--:|
| 最近邻距离中位数 | 1.165 | 2.513 | 15.067 | 15.489 |
| 第 40 邻距离中位数 | 4.216 | 4.165 | 15.586 | 16.042 |
| 至少一个特征越过训练范围 | — | 63.4% | 100% | 100% |
| `t=75` 被选择的接收节点 | — | 14/24 | 18/36 | 16/36 |

X36 的 source/receiver expected cardinality 比 M24 训练均值偏移约
`10.5–10.8` 个标准差。当前所谓 confidence 只是远距离 kNN 邻居中正
标签的比例，并不是分布支持或统计置信度。因此，“低 confidence 就不通信”
在 X36 上实际变成了“让一半以上节点失去有用通信”。

更关键的是，解析权重筛查曾推翻“X36 没有定向通信收益”的判断，但现在
必须把该结果重新归类为固定定向控制：

| 场景 | Seed | 策略 | E-OSPA | Local | Static | 相对 local | 相对 static |
|:--|--:|:--|--:|--:|--:|--:|--:|
| M24-hard | 7/17 平均 | Full reliability, `w=0.50` | 18.6913 | 23.2228 | 24.1369 | 19.51% | 22.56% |
| X36-clean-scale | 7 | Full reliability, `w=0.50` | 37.9153 | 43.1425 | 46.3716 | 12.12% | 18.24% |
| X36-clean-scale | 17 | Full reliability, `w=0.50` | 35.5685 | 39.4500 | 43.8843 | 9.84% | 18.95% |

这些结果均来自 `t=75–77`、共享 static prefix 的三步条件续跑，属于
方法开发证据，不是完整 episode 或论文级统计结果。X36 seed 17 的
`w=0.50` 使用 36 条定向消息和 8.06 MB attempted posterior bytes，
约为 static 的 39.7%。`w=0.70` 在 seed 17 进一步降到 34.4973，
但缺少完整的 seed-7 配对报告，因此没有取代 `w=0.50`。M24 上
`w=0.30` 的均值 17.9975 优于 `w=0.50` 的 18.6913；选择 `w=0.50`
不是因为它在每个开发场景都最优，而是因为它是目前在 M24 和 X36
两个尺度、两个开发种子上都有完整证据的共同注册点。后续全时审计表明，
该共同点从不改变 sender；它现在是必须击败的 `fixed-index` 强对照，
不是可作为最终方法的动态骨架。

当前学习残差并未超过这个骨架。用 M24 seeds 7/17 训练、seed 11 校准后，
基于一个完整 seed/time block 的经验最坏误差门没有放行任何覆盖；在
M24 seeds 7/17 的三步续跑中，
残差策略与 `reliability + w=0.50` 的 E-OSPA、最差节点、bytes 和 routes
完全相同。最终冻结报告中，残差臂总 policy time 为 17.11 秒，解析骨架
为 0.03 秒，净增加 17.074 秒。策略计时会随机器负载波动，此处只说明
额外计算明显，不把秒数视为算法常数。因此，现阶段正结果属于解析定向
路由，不能写成数据驱动方法收益。

当前最重要的判断是：

> sender 和 KLA 权重不是两个可以独立学习的附属量，而是一个联合离散
> 动作。跨规模稳定的解析动作应作为默认骨架，学习模型只在有统计支持时
> 修改它。是否通信仍是尚未接入当前学习器的下一阶段动作。

## 问题定义

对接收节点 \(i\)，注册联合动作

\[
\mathcal A_i(t)=
\{(j,\alpha):j\in\mathcal N_i(t),
\alpha\in\{0.15,0.30,0.50,0.70\}\}.
\]

\((j,\alpha)\) 表示发送节点 \(j\) 向接收节点 \(i\) 发送 posterior，
并执行

\[
\pi_i^+=\operatorname{KLA}_{\alpha}(\pi_i,\pi_j),\qquad
w_{ii}=1-\alpha,\quad w_{ij}=\alpha .
\]

当前原型固定每个具有物理邻居的 receiver 都选择一个联合动作；local-only
是实验比较臂，还不是学习器可选的 \(\varnothing\) 动作。后续只有在训练
数据覆盖“通信会伤害该 receiver”的环境后，才增加 no-message 动作，不能
在现有论文叙述中提前声称已经学会拒绝通信。

解析骨架取当前最可靠的物理邻居和等权融合：

\[
j_i^0=\arg\max_{j\in\mathcal N_i(t)}q_{ji}(t),\qquad
a_i^0=(j_i^0,0.50).
\]

等权 KLA 有清楚的对称解释，且在 M24/X36 的开发种子上均超过 5% 的
tracking 门槛。它现在是必须击败的强基线，而不是普通消融。

## KLA 兼容度骨架

仅按链路可靠性选 sender 仍可能把彼此冲突的后验强行融合。对标签
\(\ell\) 和候选权重 \(\alpha\)，定义空间 Chernoff 重叠量

\[
\eta_{ij}^{(\ell)}(\alpha)=
\int
p_i^{(\ell)}(x)^{1-\alpha}
p_j^{(\ell)}(x)^\alpha\,dx .
\]

结合 Bernoulli 存在概率后，标签兼容系数为

\[
\beta_{ij}^{(\ell)}=
(1-r_i^{(\ell)})^{1-\alpha}
 (1-r_j^{(\ell)})^\alpha
+
\eta_{ij}^{(\ell)}
(r_i^{(\ell)})^{1-\alpha}
(r_j^{(\ell)})^\alpha .
\]

实际代码按乘法解释上式中的相邻因子；这里换行只用于排版。由于
\(\eta\le 1\)，空间冲突会通过 KLA 归一化项压低融合后的存在概率。
因此，兼容度不是任意的神经网络特征，而是 KLA 本身产生的风险信号。
当前实现对每个标签的 Gaussian mixture 做 moment matching 后解析计算
Chernoff 重叠，并同时记录 source 新标签质量和 source 相对 receiver 的
精度劣势。它不读取真值。

代码入口：

- `common/computeLmbKlaCompatibilityMatrix.m`
- `common/selectAnalyticDirectedRoutingPolicy.m`

兼容度策略仍属于解析基线。只有它在新的 paired seeds 上相对 reliability
骨架形成稳定 Pareto 收益后，才进入最终方法。

当前仓库还有一个必须披露的 KLA 边界：某标签若被一个来源剪枝，该来源
不会以 \(r=0\) 参加该标签的融合；代码只在仍保留该标签的来源之间重新
归一化权重。这是为了避免独立剪枝造成 zero-existence veto 的工程规则，
不是无需说明的标准完整密度 LMB-KLA。因此本文所有开发数值都条件于该
receiver 实现；若要写严格的 LMB-KLA 理论性质，必须改用明确处理缺失标签
的 reference，或把这条规则作为算法定义的一部分。

## 正确的教师标签

旧教师把链路可靠率乘进 KLA 权重再归一化。这与真实执行过程不等价：
真实系统是“成功则按完整权重融合，失败则保持 local”。候选动作的正确
期望风险应为

\[
C_i(j,\alpha)=
q_{ji}R_i\!\left(
\operatorname{KLA}_{\alpha}(\pi_i,\pi_j)
\right)
+
(1-q_{ji})R_i(\pi_i).
\]

下一版数据集必须保存完整的
`receiver × sender × weight` 条件风险和期望风险张量。学习目标改为相对
解析骨架的残差

\[
\Delta_i(j,\alpha)=
\frac{C_i(a_i^0)-C_i(j,\alpha)}
{\max\{R_i(\pi_i),\varepsilon\}},
\]

这个无量纲目标与代码一致，并避免绝对风险随场景尺度改变。负收益边不再
提供融合权重监督。

## 支持域门控的学习残差

当前原型先用共享岭回归器为每个联合动作给出残差点预测
\(\widehat{\Delta}_{ij\alpha}\)。模型冻结后，每个接收节点只在训练支持域内
寻找预测最高的非骨架动作；独立校准集再估计“被选动作的预测值减真实残差”
的上分位数 \(q_{1-\delta}\)，并构造保守分数

\[
L_{ij\alpha}=
\widehat{\Delta}_{ij\alpha}-q_{1-\delta}.
\]

只有当 \(L_{ij\alpha}>\epsilon\) 时，动作才允许覆盖 reliability
骨架；否则完整回退到 \(a_i^0\)。校准单位必须按完整
`scenario × seed × time block` 做 calibration，不能再使用同一快照的
receiver-fold 正邻居比例。模型还必须具有显式的特征范围或距离 OOD 门；
X36 越过训练支持时自动回退，不允许强制外推。

当前实现使用 25 个动作前特征和 7 个 weight/action 特征。跨规模处理包括：

- 用已注册 birth model 容量归一化基数统计，不读取保存的目标轨迹；
- 除以通信半径的几何距离；
- 归一化的共享标签状态差异、精度差和 active-label overlap；
- 链路可靠性、编队关系、source/receiver 相对质量；
- receiver 内的可靠性/兼容度秩、previous-edge；
- 候选 KLA 权重及其与可靠性、状态差异、精度差和存在收益的交互项。

payload、edge age、最近成功传输年龄等闭环量尚未接入，不能在论文中写成
已有特征。岭回归只是用于检验残差信号能否跨种子泛化；只有在多个独立
seed/time block 留出上稳定放行无害覆盖后，才值得升级为共享
MLP/DeepSets。邻域
上下文被消融证明有额外价值后，再考虑一层 GNN。

### 当前校准状态

最终保留的开发协议用 M24 seeds 7/17 训练、seed 11 校准：

- 训练 4,416 个、校准 2,208 个联合动作，32 个特征；
- 82.25% 的校准动作位于训练支持盒内；
- seed-11 block 的 selected-action 经验最坏过估计校正量为 1.4011；
- 门控没有放行任何覆盖，harmful override 因而为 0。

这里的校准集只有一个完整 block，1.4011 只是该 block 上的经验最坏值，
不能解释为 95% 的总体覆盖保证。早期按 receiver 分组的门槛试验因同一
快照内 receiver 共享后验环境、样本不独立而作废，不进入证据链。这些
结果说明单一 snapshot 不能提供足够的跨环境学习证据；“无覆盖”不是
“学习模块已经可靠有效”的正结果。下一步需要增加
独立训练/校准种子和时间块。若增加数据后门控仍总是回退，则应保留解析
策略作为方法结论，并停止把学习残差包装成贡献。

## 硬投影和信息合同

当前代码已经执行的确定性投影只有：

1. 仅选择当前物理图中的 `sender → receiver`；
2. 每个 receiver 每步至多一个 sender；
3. 权重只能来自注册集合，且每行非负、和为 1；
4. 消息条数不超过注册的 receiver/message budget；
5. OOD 或没有正的保守残差时回退到解析骨架。

总 posterior-byte 预算、churn 上限、编队覆盖和滑动窗口联合连通尚未
接入这个策略。它们是下一阶段可能加入的约束，不能作为当前实现的性质。
当前权重集合的最大 source weight 为 0.70，因此已测试动作的 self weight
不低于 0.30；这是动作集合的结果，不是另一个独立投影器。

若暂时不实现联合连通，论文问题必须明确写成
`tracking-oriented receiver-centric message scheduling`，不能同时宣称
渐近共识保证。

当前策略在调度前读取所有节点 posterior，这不是免费的分布式本地信息。
正式实现必须明确采用中央控制器，或只使用 receiver 本地状态与上一时刻
缓存的固定长度 sender beacon。总通信量统一记为

\[
B_{\mathrm{total}}=
B_{\mathrm{posterior}}+
B_{\mathrm{beacon}}+
B_{\mathrm{schedule}}+
B_{\mathrm{ACK/retry}}.
\]

在控制面字节加入前，现有报告中的 attempted bytes 只能解释为 posterior
payload 下界。

## 冻结后的验收门槛

M24 和 X36 分别判定，不允许用两者平均掩盖失败：

- 相对同字节下 `local、固定定向图、最强解析定向策略` 中 tracking 最好的
  一个，平均 E-OSPA 改善至少 5%；
- paired bootstrap 95% CI 下界大于 0，中位改善至少 3%，至少 70% seeds
  同向；
- 完整 episode 和预注册 focus window 同向；
- node/time CVaR90 或 p95 的恶化不超过 2%；
- 通信匹配误差不超过 2%，并计入 payload、beacon、调度、ACK 和失败发送；
- 所声明的物理图、byte、degree、churn 和联合连通约束零违规；
- 若主张在线，X36 决策时间 p95 小于 1 秒；
- 数据驱动方法必须在相同总字节下进一步优于最强解析策略，或在 tracking
  相当时至少再节省 20% 总字节。

seed 7、17 和 `t=75` 已用于方法开发。seed 27 虽未参与当前参数拟合，
但已用于 D12 方法筛查以及 M24/X36 场景几何审计，只能作为
“方法 held-out、场景已审计”的一次性 canary，不能称为完全无偏的最终
验证。完整论文结论仍需要全新的 10-seed screening 和另行预注册的
30-seed paired held-out。
