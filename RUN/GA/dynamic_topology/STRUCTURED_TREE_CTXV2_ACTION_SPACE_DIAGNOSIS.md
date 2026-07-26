# Corrected ctxv2 action-space diagnosis

## 结论

修正邻接方向后，joint-tree v4 的主要瓶颈不再能归因于 edge scorer。
当前动作空间要求每一步都用恰好 \(G-1\) 条跨编队边覆盖一棵 rooted tree，
并要求相邻两步的 formation graph 并图强连通。M24 有 \(G=4\)，因此每步
必须让三个 receiver 放弃 fixed-index 编队内 sender，改用跨编队 sender。
当某个编队当前没有有利的跨编队输入时，即使精确 oracle 也必须选择负收益
桥边。

在 corrected ctxv2 的五个 seed、每个 seed 六个快照上，当前 exact
joint-tree projector 有 9/30 个块达不到 5% 一步期望风险改善，最差为
seed 23、t=77 的 -1.5963%。因此预注册的“每块至少 5%”门槛对当前动作
空间不可达，不应冻结或运行 learned structured-tree artifact。

## 数据与边界

- 场景：`m24-hard`
- Seed：19、23、29、31、37
- 快照：t=75:80
- KLA source weight：0.70
- 基线：`fixed-index-star`
- 数据 contract：
  `directed-teacher-series-v2-receiver-row-previous-adjacency`
- 邻接约定：receiver 行、sender 列
- action feature 不读真值；teacher 标签和 oracle-v4 行为使用真值
- 因此以下结果是 truth-assisted posterior 状态分布上的一步 proxy
  诊断，不是无真值闭环 E-OSPA 证据

五个 seed 的数据均已在本轮方法诊断中读取。若据此修改动作空间或门槛，
它们都只能继续作为 development data；新版本必须重新分配未查看的
validation/held-out seed。

## 当前 joint-tree v4 的结构上界

表中收益按六个快照的 baseline risk 加权汇总，不是简单平均百分比。

| Seed | Exact joint-tree v4 六步 proxy gain |
|---:|---:|
| 19 | 11.09% |
| 23 | 3.06% |
| 29 | 13.01% |
| 31 | 27.05% |
| 37 | 3.69% |

逐块检查中有 9/30 个块低于 5%。去掉 previous/current 联合强连通约束、
只保留“当步必须是一棵 rooted tree”后，低于 5% 的块仍为 9/30；最差值
只从 -1.5963% 变为 -1.2429%。因此 pairwise-union 约束只贡献次要损失，
主要问题是“每步都必须覆盖所有编队的树”不包含编队内 no-op/fallback。

seed 23、t=77 给出了直接反例：

| 约束 | 所选跨编队边的 proxy gain | 合计 |
|:--|:--|--:|
| rooted tree + previous union | -2.3429%、+0.1711%、+0.5755% | -1.5963% |
| rooted tree only | +0.1711%、+0.4381%、-1.8520% | -1.2429% |

当步存在若干正收益跨边，但它们不能覆盖一棵连接四个编队的有向树。
树约束必须加入一条负收益桥，所以学习器无法突破这个结构上界。

## Rolling-\(B\) 可达性诊断

新的诊断动作允许每步选择 0 到 \(G-1\) 条跨编队 override；未被 override
的 receiver 保留 fixed-index 编队内 sender。每步仍满足：

1. 每个 receiver 最多一个跨编队 override；
2. 每个跨编队 sender 最多使用一次；
3. 跨编队 override 不超过 \(G-1=3\)。

连通性不再要求每一步形成树，而是通过有向 cut constraints 要求每个
连续 \(B\) 步窗口的 formation-graph 并图强连通。以下是精确二进制规划
在同一批一步标签上的六步加权 proxy 上界。无窗口、\(B=2\) 和 \(B=6\)
列只施加结构约束；作为下一候选的 \(B=3\) 列还要求 attempted payload
相对当步 fixed-index 不增加超过 2%：

| Seed | 无窗口连通 | \(B=2\) | \(B=3\) | \(B=6\) | \(B=3\) payload ratio | \(B=3\) 最大单步 ratio |
|---:|---:|---:|---:|---:|---:|---:|
| 19 | 12.65% | 11.15% | 11.92% | 12.65% | 0.9967 | 1.0117 |
| 23 | 5.70% | 4.99% | 5.43% | 5.69% | 0.9983 | 1.0178 |
| 29 | 15.83% | 14.58% | 15.55% | 15.82% | 0.9953 | 1.0036 |
| 31 | 30.33% | 28.85% | 30.22% | 30.33% | 0.9967 | 1.0062 |
| 37 | 6.23% | 4.88% | 6.04% | 6.23% | 0.9994 | 1.0016 |

\(B=2\) 在 seed 23 和 37 上仍低于 5%，而 \(B=3\) 在五个 development
seed 上都超过 5%，同时满足逐步不超过 2% 的 payload 增量约束。这只是
动作空间存在可用信号的诊断；它没有证明 truth-free scorer 能找出这些边，
也没有证明闭环 E-OSPA、tail 或 consensus 会改善。

## 方法方向

当前 structured-tree v4 停止在 proxy 阶段，不冻结模型。下一版本应改为
“学习边价值 + rolling-\(B=3\) 安全投影”：

1. 保留 fixed-index 编队内 sender 作为显式 no-op/fallback；
2. 学习器只给跨编队 override 评分，不再强迫当步覆盖所有编队；
3. 投影器维护三步 connectivity debt，在截止前补齐尚未满足的有向 cut；
4. 优化目标改成三步累计价值，而不是要求每个快照都达到 5%；
5. proxy gate 使用逐步非退化、按 seed 的窗口累计收益、payload 和
   oracle-capture；最终结论仍由无真值闭环 E-OSPA/tail/consensus 决定；
6. 当前五个 seed 全部降为 development data，冻结结构与阈值后再生成
   新 validation seed，并保留新的 closed-loop held-out seed。

理论上可利用“正自权重、非零融合权重下的重复联合强连通”给出三步窗口
的信息传播/收缩保证；数据驱动部分负责在安全可行集内估计边价值，而不是
替代连通性保证。
