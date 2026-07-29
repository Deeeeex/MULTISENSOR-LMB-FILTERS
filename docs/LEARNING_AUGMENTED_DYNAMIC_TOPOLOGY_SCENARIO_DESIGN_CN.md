# 面向动态通信拓扑的多编队 LMB 跟踪场景设计

> 状态：旧的一步诊断教师已被否定；目前已经新增 D12/M24/X36 的
> difficulty-gated hard presets、可扩展候选拓扑池和不含切换惩罚的
> counterfactual task-risk teacher。三类 hard scene 的 3-seed
> 几何审计已通过；纯 current task-risk teacher 已在 D12 的单 seed
> 60 步闭环筛查中优于解析动态和强化固定对照，并在 M24 的单快照上显示
> 可扩展的动作区分度。以上仍是 screening evidence，尚未授权 GNN
> 训练或论文效果主张。第 12 节保留历史停止点，第 13 节记录本轮重启后
> 的实现与新证据。
>
> 研究边界：新的动态拓扑方向与此前的 full/light payload 等价叙事解耦。论文级实验必须使用遵循 LMB-KLA 密度级公式、保留单目标混合结构且清楚记录数值近似边界的 reference；当前投影到单高斯的融合路径只能用于场景与软件 smoke test，不能支撑最终估计或理论结论。

## 1. 核心决定

旧的 `4+4` 场景保留为回归测试，不再作为动态拓扑工作的主场景。新的实验按 `8 → 12 → 24 → 36` 四级组织：

| 层级 | 编队结构 | 传感器数 | 主要用途 | 是否进入主结果 |
|:--|:--|--:|:--|:--:|
| R8-Legacy | 2 个编队 × 4 节点 | 8 | 复现旧结果、检查代码兼容性 | 否 |
| D12-Diagnostic | 3 个编队 × 4 节点 | 12 | 枚举跨编队桥接拓扑，测量 oracle gap | 诊断图/附录 |
| M24-Main | 4 个编队 × 6 节点 | 24 | 动态观测交接、相关链路退化和通信预算下的主实验 | 是 |
| X36-Scale | 6 个编队 × 6 节点 | 36 | 未见编队数量与网络规模的外推测试 | 是 |

这一规模选择同时增加了“每个编队的节点数”和“编队数量”。无向候选节点对从 8 节点时的 28 对增加到 24 节点时的 276 对、36 节点时的 630 对，分别约为旧场景的 9.9 倍和 22.5 倍。M24 足以形成有意义的选边空间，X36 则用于检验方法是否只记住了固定规模。

暂不把 48 个或更多节点列为必做项。只有当 X36 的单次 160 步运行时间、峰值内存和 30 次配对 Monte Carlo 均可接受时，才增加 `6×8` 的 X48 工程扩展实验。

## 2. 为什么旧 4+4 场景不能回答新问题

旧实验入口不是通用场景生成器：

- 传感器数固定为 8，丢包等级数量也按 8 个节点写死；
- 邻接图固定为两个 4 节点组内连接，再用四组固定配对边连接两个编队；
- 两个编队使用同一速度，缺少独立接近、分离、转向和重新交接目标的过程；
- FoV 距离设为 `60000`，在当前坐标尺度上几乎不构成有限视野，因此不同节点掌握的信息差异不够强；
- 既有 N50 结果中，动态 full 拓扑相对静态 full 增加约 6.1% 字节，并使 consensus OSPA 退化约 11.1%；动态 light 也被静态 light 支配。

因此，旧结果不能推出“动态拓扑没有价值”。它只能说明：在一个固定双编队、近乎全局可见、桥边结构预先给定的 8 节点场景中，现有启发式重连没有形成有效收益。

两个代表性的 LMB 工作也说明现有数值实验通常较小：Shen 等人的不同 FoV consensus-LMB 实验展示 4 个传感器；其 event-triggered consensus-LMB 实验使用 9 个传感器、最多 4 个目标和 50 次 Monte Carlo。新的 M24/X36 不是为了单纯追求大，而是要把“动态选择谁与谁通信”从少数固定桥边扩展为真正的组合决策问题。

## 3. 要由场景回答的研究问题

### Q1：是否存在动态拓扑的客观上限收益

在相同的边预算、发送频率、payload 形式和融合算法下，可以使用全局 posterior、仿真真值或未来链路结果的离线 oracle，能否稳定优于最强静态拓扑和非学习动态启发式？

如果 oracle 都没有显著优势，说明场景中不存在足够的动态选边价值，应停止该方向，而不是继续训练 GNN。

### Q2：收益来自哪里

动态拓扑的收益应能被拆成两个来源：

1. **观测交接**：目标逐渐离开一个编队的 FoV、进入另一个编队的 FoV，最有价值的信息边随时间转移；
2. **链路绕行**：某一批跨编队链路同时变差时，网络可以利用其他编队或其他网关节点恢复有效信息流。

两类因素先分别测试，再在主场景中组合。这样可以避免只在一个“所有困难同时出现”的场景上得到无法解释的结果。

### Q3：是否确实需要数据驱动方法

即使 oracle 显著优于静态图，也要继续比较可靠性、几何距离、谱连通性和 posterior discrepancy 等强启发式。如果一个简单启发式已经获得几乎全部 oracle 收益，就没有必要引入 GNN；此时应优先发展更简洁、可分析的算法。

## 4. 场景族

### 4.1 R8-Legacy：兼容性回归

现有 `2×4`、100 步、10 个目标和固定 4 组桥边的旧 runner 继续作为
数值回归的权威入口；新框架中的 R8 preset 只检查统一接口、维度和固定
拓扑退化，不替代旧 runner 的逐随机数复现。它只回答：

- 新场景生成器是否改变旧数据；
- 新的可行拓扑投影是否能退化为旧静态图；
- 通信和 tracking 指标是否保持数值兼容。

R8 不参与新方法的主效果量计算，也不用于证明可扩展性。

### 4.2 D12-Handover：可枚举的观测交接诊断

三个 4 节点编队位于三角形的三个扇区，目标沿三条相交走廊依次从一个编队的 FoV 进入另一个编队的 FoV。主诊断关闭全部丢包，以隔离“信息位置随时间变化”的作用；随后只增加一组弱的独立距离丢包作为稳健性检查。

每个编队保留一个固定 4 边环形内部骨架。对每一对编队，只允许几何边界上最接近对方的两个节点成为跨编队候选，因此每个编队对只有 `2×2=4` 条候选桥边。选择两条桥边构成编队级生成树时，共有 `3×4²=48` 个可行拓扑；选择三个编队对各一条桥边时共有 `4³=64` 个可行拓扑。这个空间可以逐时刻完整枚举，用于计算一步 oracle，而不需要先假设学习模型。

建议参数：

| 参数 | 值 |
|:--|:--|
| 仿真长度 | 120 步 |
| 目标数 | 3 组 × 3 个，最多 9 个 |
| FoV | 距离 320 m，半角 75° |
| 主诊断丢包率 | 0 |
| 稳健性丢包率 | 编队内 0.02，编队间按距离为 0.05–0.25 |
| 边预算 | 14（12 条内部环边 + 2 条桥边）和 15 两档 |
| 拓扑切换预算 | 每步最多替换 1 条无向桥边 |

### 4.3 D12-Outage：可枚举的链路绕行诊断

传感器和目标轨迹与 D12-Handover 相同，但将 FoV 扩大到所有编队都能维持基本目标观测，避免观测交接成为主导因素。仿真中依次对两个编队对施加 15–20 步的相关丢包抬升，且始终保留一条绕行路径。

它只回答：在相同 posterior 信息量下，动态桥边是否能比静态桥边更快恢复有效信息流。

### 4.4 M24-Main：四编队组合主场景

基础 M24 使用 4 个 6 节点编队、最多 12 个目标和 160 步仿真。场景
同时包含观测交接和相关链路退化，但两者的发生时间错开，以便按时间
窗口解释结果。为保证首轮 teacher screening 有足够的观测归属变化，
`m24-hard` 将目标数提高到 16；基础版本和 hard 版本必须分开报告。

同一组轨迹应形成三个可开关版本：

1. `M24-Handover`：关闭丢包，只保留有限 FoV 和目标交接；
2. `M24-Link`：扩大 FoV 以维持基本观测，只保留距离丢包和相关阻塞；
3. `M24-Composite`：组合两类因素，作为最终主场景。

主论文不能只给 Composite；至少要用 Handover/Link 说明收益来源。

#### 空间与编队几何

- 监视区域：`[-800, 800] m × [-800, 800] m`；
- 每个编队由 6 个传感器组成，初始相对位置位于半径 35 m 的正六边形上；
- 六边形随编队中心航向旋转；每个 episode 对半径和角度加入不超过 10% 的固定扰动，避免所有样本拥有完全相同的几何；
- 不向未来的边价值模型提供编队编号，编队信息只用于场景生成和可行性检查。

编队中心在 `k={1,40,80,120,160}` 的建议航路点如下，步间使用保持速度连续的分段三次插值：

| 编队 | k=1 | k=40 | k=80 | k=120 | k=160 |
|:--|:--|:--|:--|:--|:--|
| G1 | (-520,-180) | (-340,-120) | (-120,80) | (260,200) | (520,180) |
| G2 | (-180,520) | (-120,340) | (80,120) | (200,-260) | (180,-520) |
| G3 | (520,180) | (340,120) | (120,-80) | (-260,-200) | (-520,-180) |
| G4 | (180,-520) | (120,-340) | (-80,-120) | (-200,260) | (-180,520) |

这一设计使四个编队先从不同扇区接近中心、发生短时重叠，再交换负责区域并重新分离。采样周期取 1 s，建议将编队中心速度限制在 12 m/s、加速度限制在 1.5 m/s²。正式实现前必须通过轨迹检查，保证传感器间没有重复位置，速度、加速度和转向率均处于设定上限内。

#### 目标过程

- 四组目标，每组 3 个，最多 12 个；
- 四组分别从西、北、东、南走廊进入，在 `k≈65–95` 经过中心区域，再从相对方向离开；
- 各组出生时刻建议为 `1、11、21、31`，死亡时刻位于 `145–160`；
- 目标速度控制在 6–10 m/s；
- 每组内部目标间距保持在 50–90 m，中心阶段加入一次中等幅度协调转弯；
- 所有节点共享同一组标签化出生模板，但是否形成有效检测由有限 FoV 决定，避免把标签初始化差异与拓扑收益混在一起。

第一轮 oracle-gap 实验应使用较易分辨的目标间距。更密集的多峰 crossing 只能在 mixture-aware LMB-KLA 参考实现通过单元测试后作为独立压力测试加入。

#### 传感与杂波

| 参数 | 主值 | 压力变化 |
|:--|--:|:--|
| 检测概率 | 0.90 | 节点异质性 0.80–0.95 |
| FoV 距离 | 360 m | 300–420 m |
| FoV 半角 | 75° | 60°–90° |
| 位置测量标准差 | 5 m | 3–8 m |
| 每节点每步平均杂波数 | 3 | 1、5、10 |

主结果只改变一个压力因素，不能把异质 FoV、强杂波、密集 crossing 和严重丢包同时扫成一个不可解释的“困难场景”。

#### 物理通信图

时刻 `k` 的候选物理图记为 `G_phy(k)`：

- 节点距离不超过 900 m 时形成候选边；
- 编队内基础丢包率接近 0.02；
- 编队间边的基础丢包率随距离平滑增加，可使用
  `p_drop(i,j,k)=clip(0.02+0.25(d_ij/900)^2+b_ij(k),0,0.95)`；
- 在 `k=91–110` 和 `k=116–135` 分别对不同的跨编队走廊施加相关阻塞项 `b_ij(k)`，但场景生成器必须验证仍存在预算内的替代连通拓扑；
- 方法可以读取当前几何、历史 ACK 和链路质量估计，不能读取未来阻塞计划或未来随机丢包结果。

场景还必须保证存在一个在全部时刻均物理可行的强静态拓扑，避免通过让固定图直接失效来人为制造动态方法优势。

#### 拓扑预算与安全约束

设传感器总数为 `N`、编队数为 `G`。主预算采用：

| 预算档 | M24 边数 | X36 边数 | 含义 |
|:--|--:|--:|:--|
| Tight | `N-1=23` | `N-1=35` | 仅允许一棵全局生成树 |
| Medium | `N+ceil(1.5G)=30` | 45 | 主实验预算 |
| Loose | `ceil(1.5N)=36` | 54 | 较宽松通信 |

所选拓扑必须满足：

1. `E(k) ⊆ E_phy(k)`；
2. 边数不超过当步预算；
3. 每个编队的诱导子图连通；
4. 名义阶段全局图连通；若物理图临时不允许，则满足预先指定的 3 步联合连通，并显式报告不可行时刻；
5. 节点总度不超过 4、跨编队度不超过 2；
6. M24 每步最多替换 2 条无向边，X36 每步最多替换 3 条；
7. 拓扑协商、握手、ACK 和失败尝试均计入通信开销。

这些是场景和系统约束，不依赖将来采用 GNN、解析评分还是其他边价值模型。

### 4.5 X36-Scale：六编队未见规模

X36 使用 6 个 6 节点编队。编队中心沿六边形扇区生成与 M24 同类型、但旋转角和到达时间不同的航路。方法开发和超参数选择阶段不得使用该规模。

X36 分成两个子测试：

1. **Topology-only scale**：保持最多 12 个目标，只增加节点与候选边，隔离拓扑算法的规模效应；
2. **Joint-load scale**：增加到 6 组 × 3 个目标，共 18 个目标，同时测试过滤和通信负载，只在运行成本允许时执行。

主张“可扩展”至少需要报告从 N=12、24 到 36 的拓扑打分时间、投影求解时间、完整过滤时间、峰值内存和通信量，而不能只报告 tracking 指标。

## 5. 对照组：先验证问题，再设计方法

第一阶段固定使用同一种标准 posterior、同一发送频率和同一 LMB-KLA 实现，只改变拓扑。事件触发、payload 压缩和 full/light 机制全部关闭，避免把三个问题混成一个结果。受控资源首先定义为“激活的无向边数 × 固定发送轮数”，实际 attempted bytes 仍逐消息计量；如果不同发送节点的 posterior 大小导致字节数不完全相同，则在性能—attempted-bytes 曲线上做匹配比较，不能把边数相同直接写成字节数相同。

建议对照如下：

| 类别 | 对照 | 作用 |
|:--|:--|:--|
| 下界 | Local-only | 判断无通信时的损失 |
| 高通信参考 | Full physical graph | 给出不受边预算限制的性能上限，不参与同预算胜负 |
| 强静态 | Offline robust static | 在训练场景上离线选择、在所有时刻物理可行且满足同一预算的最优固定图 |
| 随机动态 | Random feasible | 检查收益是否只来自重连本身 |
| 解析动态 | Reliability-first | 只看链路质量 |
| 解析动态 | Spectral/geometry | 优化代数连通度或距离结构 |
| 任务启发式 | Posterior-discrepancy | 使用局部 posterior 差异和覆盖互补性 |
| 诊断上限 | Exact one-step oracle | D12 中枚举所有可行桥接拓扑 |
| 近似上限 | Beam/look-ahead oracle | M24 中只用于估计上限，不作为可部署方法 |

`Offline robust static` 必须是经过优化的强基线，而不是任意指定的环或旧 4+4 图。所有动态方法与它使用相同的边预算、度约束和切换成本核算。

## 6. Oracle 的定义与边界

D12 的一步 oracle 在完成各节点本地更新后，复制同一组本地 posterior，对每个可行拓扑执行一次相同的 LMB-KLA 融合，选择使下式最小的拓扑：

`L_oracle = L_consensus + α L_tracking-surrogate + β C_switch`

其中：

- `L_consensus` 是逐标签存在概率和空间密度的网络分歧；
- `L_tracking-surrogate` 只使用仿真中可获得的真值计算，用于离线诊断，不能作为部署时输入；
- `C_switch` 是相对上一时刻替换边的成本。

必须同时报告只优化 consensus 的 oracle 和加入真值 surrogate 的 oracle，避免结果依赖某一个人为加权目标。

这里需要增加一个严格限定：**一步 oracle 不是整段闭环跟踪性能的全局
上界**。它的选边会改变下一时刻 posterior，因此逐时最优动作可能被
后续反馈反超。它只能称为“精确枚举的一步诊断策略”。若要使用“上限”
措辞，必须另做 teacher-forced 单步价值评估，或在短时域内计算
look-ahead/动态规划上界。M24 的 beam/look-ahead 也只能称为“近似诊断
参考”，不能写成全局最优。

## 6.1 当前实现入口

| 功能 | 入口 |
|:--|:--|
| 一键场景 preset | `common/buildDynamicTopologyScenarioConfig.m` |
| Paired 场景输入 | `common/generateDynamicTopologyScenarioInputs.m` |
| 多编队/走廊轨迹 | `common/generateMultiFormationTrajectories.m`、`common/generateCorridorTargetTrajectories.m` |
| 物理图、静态图和 D12 的 48 个候选图 | `common/buildDynamicTopologyGraphs.m` |
| 场景硬验证 | `common/validateDynamicTopologyScenario.m` |
| 场景难度度量 | `common/measureDynamicTopologyScenarioDifficulty.m` |
| D12 解析策略与一步枚举策略 | `common/selectD12TopologyPolicy.m` |
| D12/M24/X36 通用候选池 | `common/buildDynamicTopologyCandidatePool.m` |
| 当前/预测 task-risk teacher | `common/selectCounterfactualTopologyTeacher.m`、`common/evaluateLmbTopologyTaskRisk.m` |
| 未来测量闭环 teacher | `common/selectClosedLoopCounterfactualTopologyTeacher.m`、`common/evaluateClosedLoopTopologyTaskRisk.m` |
| M24/X36 解析投影基线 | `common/selectProjectedTopologyPolicy.m` |
| topology-only paired runner | `RUN/GA/runDynamicTopologyOracleGapScreen.m` |
| hard scene 几何审计 | `RUN/GA/runDynamicTopologyScenarioDifficultyAudit.m` |
| teacher signal 配对快照筛查 | `RUN/GA/runDynamicTopologyTeacherSignalScreen.m` |

实验脚本只需切换 `d12-handover`、`d12-link`、`m24-handover`、
`m24-link`、`m24-composite`、`x36-topology`、`x36-joint`、
`d12-hard`、`m24-hard`、`x36-hard`、`x36-matched` 或
`x36-clean-scale` 字符串；局部参数通过一个 `overrides` struct 覆盖。

## 7. 指标

### 跟踪结果

- GOSPA 作为 tracking 主指标，并拆分定位、漏检和虚警分量；现有 E-OSPA/H-OSPA 作为连续性与标签表现的补充；
- 基数绝对误差；
- 标签切换和轨迹碎片数；
- 全网平均、最差节点和 p90 节点结果。

### Posterior 一致性

- 逐标签存在概率的节点间方差和最大差；
- 逐标签空间密度分歧；mixture-aware 实现下使用适合混合密度的散度或数值近似；
- 基数 dispersion；
- 观测交接后达到给定一致性阈值所需的步数。

### 通信与拓扑

- **attempted bytes 为主通信成本**，delivered bytes 单独报告；
- payload、控制、握手、ACK 和失败发送分项；
- 当前图、滑动窗口图和实际融合权重图的连通性；
- 逐标签当前/窗口连通违例率；
- p90/p95 posterior stale age；
- 拓扑 churn、节点度分布和节点通信负载公平性；
- 阻塞结束后的恢复时间。

### 计算开销

- 候选边打分时间；
- 安全投影/组合优化时间；
- LMB 更新与融合时间；
- 峰值内存；
- 以上结果随 N 和候选边数的增长曲线。

## 8. 预注册式继续/停止门槛

下列数值是建议的实用效果门槛，需由作者确认后冻结。

### Gate A：场景有效性

- 同一 seed 下所有对照共享完全相同的目标轨迹、测量、FoV 扰动和链路随机数；
- 所有被比较拓扑满足同一物理图、边数、度和 churn 约束；
- 不允许出现重复传感器位置、越界轨迹或未记录的不可行拓扑；
- 强静态图、full physical graph 和 local-only 能形成合理的性能区间。
- OSPA/GOSPA 的距离截断必须与场景米制尺度匹配；不得沿用旧小场景中
  `eC=5 m` 后再把大范围场景的饱和结果解释为算法差异。

### Gate B：是否存在值得研究的动态拓扑空间

在 D12 的 10 个 screening seeds 和至少 30 个 held-out paired seeds 上：

- exact oracle 相对最强静态/解析基线，在匹配的 attempted communication 下，使主 consensus AUC 至少降低 10%，或使 tracking 主指标至少改善 5%；匹配采用 attempted bytes ±2% 或同一性能—通信曲线上的插值点；
- paired 95% bootstrap CI 不跨 0；
- oracle 的性能—通信曲线在 Tight/Medium/Loose 三档中至少支配两档；
- 逐标签窗口连通违例率不得变差。

若上述条件不成立，停止 GNN 方向。若效果只有 5%–10%，先检查场景是否缺少真正的观测交接或链路替代路径，不直接扩大模型。

### Gate C：是否需要学习

- 最强人工动态启发式若已获得超过 90% 的 oracle 增益，则优先采用解析方法；
- 只有当 locally available features 对 oracle 边排序在 held-out 场景上具有稳定预测力，且解析启发式仍留下明确 residual gap，才进入 GNN 设计；
- 学习方法的首个目标不是“超过静态图”，而是至少回收最强解析基线到 oracle 之间 50% 的剩余增益。

### Gate D：规模与安全

- M24 held-out 结果通过后，冻结模型和阈值，零样本运行 X36；
- 安全约束违例数必须为 0；物理不可行时刻应由场景检查器单独标记，不能算作算法违规；
- X36 上仍需保留统计显著的同向收益，同时报告计算和内存增长；
- 若只在固定 4×6 规模有效，不主张规模泛化。

## 9. 数据划分与统计

- 场景几何 seed、目标过程 seed、测量 seed、链路 seed 分开保存；
- 每个 arm 使用配对 seed 和同一随机输入；
- D12 先用 10 个 screening seeds，只用于判断 oracle gap；
- 超参数和未来训练只使用 train/validation seeds；
- 最终 M24 与 X36 使用至少 30 个从未用于调参的 paired test seeds；若效果接近门槛，再扩展到 50；
- 报告逐 seed 差值、均值/中位数、95% bootstrap CI 和 Pareto 关系，不能只给平均曲线。

## 10. 对现有代码的影响

### 可复用部分

- `generateMultisensorModel`、本地 LMB 更新、distributed/event-trigger 主循环大多按 `numberOfSensors` 分配；
- 当前边打分已经按通信距离过滤候选边；
- attempted/delivered bytes、滑动窗口图、effective-weight 图、逐标签连通性和 stale age 诊断可以继续使用。

### 必须先改的部分

1. 新增通用多编队轨迹生成器，支持任意编队数、每队节点数、航路点、编队旋转和 episode 扰动；
2. 新增目标走廊与显式 birth/death 轨迹生成器，不能用当前单一 CV formation 近似中心 crossing；
3. 将 observation/ground-truth 空间从当前默认的 `[-100,100]²` 改为场景配置，并同步更新 birth prior、杂波空间体积和可视化坐标；
4. 用通用物理图/场景图构造替代 `buildNeighborMap4Plus4`；
5. 增加相关链路阻塞生成器，并保存完整的 `p_drop(i,j,k)` 与配对随机数；
6. 增加场景有效性检查：轨迹、FoV 覆盖、物理图、预算可行性和强静态图交集；
7. 重写拓扑选择的不可行处理。当前选择器在没有有限候选边时会退回全连接图，可能越过通信距离；候选图不连通时也缺少明确的 infeasible 状态；
8. 当前 distance-balanced bridge 逻辑只支持两个等规模编队且每队不超过 7 个节点，不能直接用于 4/6 编队；
9. 把 O(N²T) 诊断按需要改为稀疏存储或可选记录，避免 X36/X48 多 arm Monte Carlo 的内存浪费；
10. 在论文级实验前实现并验证遵循密度级公式、保留混合结构的 LMB-KLA reference，并记录其中不可避免的数值近似；当前单高斯投影融合只保留为 legacy/prototype arm。

## 11. 实施顺序

1. 冻结本场景规范和 Gate A–D；
2. 实现通用多编队场景生成器与有效性检查，不实现 GNN；
3. 修复拓扑可行性处理，并建立保留混合结构的 LMB-KLA reference；
4. 跑 R8 回归和 D12 exact-oracle screening；
5. 只有 Gate B 通过，才跑 M24 非学习基线；
6. 只有 Gate C 通过，才开始设计 GNN 边价值预测与安全拓扑投影；
7. M24 冻结后执行 X36 零样本外推；
8. 最终再决定是否把事件触发或 payload 机制作为独立扩展，而不是混入动态拓扑主贡献。

## 12. 三次配对筛查结果与当前停止点

筛查使用 seeds `[7,17,27]`、步骤 `1–95`，重点检查 handover
窗口 `35–95`。四个 arm 使用相同轨迹、测量、融合实现和 14 条边预算，
attempted payload bytes 的配对偏差最大为 1.37%，拓扑不可行率为 0。

| 比较 | Focus E-OSPA 改善 | Focus posterior 分歧改善 | 胜出 seeds | 结论 |
|:--|--:|--:|--:|:--|
| Posterior-discrepancy vs geometry-static | 6.77% | 10.76% | 3/3、3/3 | 存在候选动态信号 |
| Consensus one-step diagnostic vs discrepancy | -7.79% | -10.90% | 0/3、0/3 | 一步一致性目标被支配 |
| Truth one-step diagnostic vs discrepancy | -4.73% | -12.08% | 0/3、0/3 | 当前真值 surrogate 也不是有效教师 |

这组结果给出三个不同层次的判断：

1. **场景不是完全没有动态选边信号。** 简单的 posterior-discrepancy
   策略在重点交接窗口稳定优于当前固定图，说明有限 FoV 下的信息位置变化
   确实会影响有价值的通信边。
2. **当前 oracle-gap 设计失效。** 一致性诊断每个 seed 只访问 2–6 个
   不同候选图，真值诊断只访问 1–5 个，而 discrepancy 访问 28–35 个。
   瞬时目标、闭环反馈和切换代价共同使诊断策略接近静态锁定；“精确枚举
   一步动作”并没有产生可用的闭环上界。
3. **当前还不能证明解析策略充分。** 名为 `robust-static` 的实现实际只按
   全时段几何距离选图，并未在 48 个固定候选中离线优化 tracking 表现。
   因此 6.77% 只能写成“相对 geometry-static 的候选信号”，不能写成
   “击败最强静态基线”，负的 oracle gap 也不能写成“GNN 没有理论空间”。

当时的研究停止点如下：

- 暂停 GNN 训练，不把一步 posterior 一致性或当前 truth surrogate 当作标签；
- posterior-discrepancy 保留为当前最强的可部署对照，但不提升为论文贡献；
- 若重启学习方向，先补齐 48 个固定候选的 train/held-out 离线静态基线，
  再设计 teacher-forced 单步价值或 2–5 步短时闭环 look-ahead；
- 在新的教师参考能够稳定支配强静态与 discrepancy、并留下预注册的
  residual gap 以前，不进入 M24 学习实验或 X36 外推。

完整逐 seed 结果见
`RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_FINDINGS_CN.md`。

## 13. 重启后的 hard scene 与 teacher 设计

### 13.1 难度不再等同于“看不见”

旧 M24/X36 preset 的主要问题是目标完全不可见的比例过高。这样的场景
即使使所有方法都变差，也不能说明拓扑选择有价值。新的 hard presets
把难度拆成四项，并写成 fail-closed gate：

1. 完全不可见样本必须很少；
2. 必须同时存在“只有一个编队看见”的信息归属阶段和“多个编队同时
   看见”的交接阶段；
3. 重点窗口内要有足够多的观测归属切换与跨目标群近距离相遇；
4. M24/X36 的相关阻塞必须与重点窗口实质重合。

对 seeds `[7,17,27]` 的几何审计结果如下。表中的比例均以“有效目标 ×
时刻”样本为分母；它们只证明场景结构有效，不替代跟踪滤波健康度检查。

| Preset | 节点/目标 | 完全不可见 | 单编队可见 | 多编队可见 | 重点窗口交接数 | 跨群近距离时刻 | 归属熵 | 阻塞重合 |
|:--|:--:|--:|--:|--:|--:|--:|--:|--:|
| D12-hard | 12/12 | 0.000 | 0.806–0.809 | 0.191–0.194 | 18 | 0.474 | 1.000 | 0 |
| M24-hard | 24/16 | 0.028–0.030 | 0.349–0.351 | 0.620–0.622 | 44–46 | 0.667 | 0.978–0.981 | 0.765 |
| X36-hard | 36/24 | 0.029–0.031 | 0.371–0.382 | 0.587–0.599 | 69–70 | 0.837 | 0.999–1.000 | 0.733 |
| X36-matched | 36/24 | 0.0036–0.0042 | 0.311–0.319 | 0.677–0.686 | 69–70 | 0.837 | 1.000 | 0.733 |
| X36-clean-scale | 36/24 | 0 | 0.093–0.096 | 0.905–0.907 | 69–70 | 0.837 | 1.000 | 0.733 |

hard screening 为提高选边压力，分别采用 D12/M24/X36 的
`14/29/44` 条边预算；第 4 节的 `30/45` 仍是 M24/X36 基础场景的
Medium 档，二者不应混写。

两个附加 X36 preset 用于拆开尺度与感知难度。`x36-matched` 匹配
M24-hard 的总体可见性比例；`x36-clean-scale` 则随场景尺寸等比例扩大
单个传感器的有效感知范围，以保持每个传感器的工作条件。前者是中间
诊断，后者才用于检查“只增加节点、编队和目标数量”是否仍有选边收益。

审计报告为
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260725_173202.md`。
回归测试还会故意把 D12 视场缩到 40 m，并要求验证器以
`difficulty-blackout` 失败，确保 gate 不是只在正例上运行。

### 13.2 通用候选动作

D12 继续使用全部 48 个注册候选图。M24/X36 不枚举组合爆炸的全部图，
而是用同一个接口生成确定性的 proposal pool：

- 固定保留每个编队的环形骨架；
- 先使编队级图连通，再填满共同边预算；
- 同时满足物理边、总度、跨编队度和全局连通约束；
- 用链路可靠性、posterior discrepancy、几何与混合分数提出候选；
- 通过逐条禁用已选桥边产生可复现的局部替代图并去重。
- 围绕上一张图显式生成单边替换邻域，保证 M24/X36 在每步 churn
  约束下仍有可执行候选，而不是只保留“上一图不动”。

这不是“找到全局最优图”的求解器，而是 M24/X36 的可控候选动作接口。
它使 teacher、解析策略和未来 GNN 都在同一安全投影空间内比较。当前
单步 smoke 已确认 M24 能返回 29 条边的可行图，D12/M24/X36 的候选池
也都经过边预算、度约束和连通性回归测试。

### 13.3 新 teacher 的监督量

旧 teacher 把 posterior 一致性、弱真值项和切换惩罚混在一个目标里。
它被 posterior-discrepancy 支配后，无法判断失败来自任务价值、权重，
还是图锁定。新 teacher 对每个候选图执行一轮与接收端一致的
mixture-aware LMB 融合，然后计算纯 tracking task risk：

`R_task = R_exist + R_position + 0.15 R_velocity`。

其中 `R_exist` 是逐标签 Bernoulli 存在概率的平方误差；位置和速度项是
混合分量下的期望平方误差，并显式包含 posterior covariance，再按场景
的 OSPA 距离和目标速度尺度归一化。`predictive` 版本只在当前时刻融合
一次，然后在不读取未来测量的条件下传播到可配置的未来时刻，以奖励在
即将发生观测交接前仍有用的信息传播。

监督标签定义为同一状态下相对固定参考图的
`advantage = R_static - R_candidate`。切换次数和切换惩罚只用于安全投影
后的动作选择，不进入 `R_task` 或 advantage。测试会把切换惩罚从 0
改到 10，并要求每个候选图的 task-risk 数组逐项完全不变。
即使注册静态图因当前 churn 约束暂时不可选，teacher 也会额外评估它
作为固定诊断参考；这样 advantage 的零点不会随上一时刻动作漂移，同时
不会把该图放回可选集合。

### 13.4 当前证据边界

首个 D12-hard 配对快照（seed 7，`t=30`，horizon `[0,3,6]`）显示：

- 候选图预测风险跨度为 3.676%，说明图动作在该状态下不是完全等价；
- 最优候选相对当前 geometry-static 的预测风险低 1.601%；
- 但 current-risk 与 6-step predictive teacher 选择同一候选图。

把开环 horizon 拉长到 `[0,10,20]` 并移动到 `t=40/60/80` 后，三个
快照仍与 current teacher 选择相同候选；平均风险跨度为 2.965%，相对
geometry-static 的平均风险优势为 1.117%，但相对 current teacher 的
额外优势仍为 0。

随后又实现了读取未来真实测量的 teacher-forced 闭环 rollout。它先用
候选图融合当前 posterior，再逐步执行 LMB 预测、本地测量更新和固定
continuation graph 融合：

- `t=30`、3-step rollout、候选图只作用当前一步：风险跨度 2.070%，
  相对静态图优势 1.501%，但与 current teacher 选图相同；
- `t=60`、3-step rollout、候选图持续 3 步：风险跨度扩大到 4.270%，
  相对静态图优势 2.709%，但仍与 current teacher 选图相同。

这说明未来测量与动作持续时间会放大“图是否重要”，但在当前 D12-hard
状态上没有改变“哪张图最好”的排序。因此继续增加 horizon 并不是当前
最有价值的复杂化方向；最简且随后进入闭环验证的监督量是纯 current
task-risk advantage。

首个闭环策略筛查已在 D12-hard seed 7、步骤 `1–60` 上完成。三个 arm
共享测量、链路随机数、14 条边预算和 heavy mixture-aware posterior：

| Arm | Focus E-OSPA | Focus 基数误差 | Focus posterior 分歧 | Attempted bytes |
|:--|--:|--:|--:|--:|
| Geometry-static | 73.3108 | 6.5054 | 0.4988 | 32,026,488 |
| Posterior-discrepancy | 72.1001 | 6.3065 | 0.4865 | 32,541,840 |
| Pure current task-risk teacher | 61.7041 | 4.5968 | 0.4693 | 32,127,360 |

相对 posterior-discrepancy，纯任务风险 teacher 的 focus E-OSPA 改善
14.42%，基数误差改善 27.11%，posterior 分歧改善 3.54%，attempted
bytes 减少 1.27%；拓扑不可行率为 0。这个结果第一次说明新标签不只是
在自己的 surrogate 上自洽，而是能形成更好的闭环状态轨迹。

它仍然只是一个 seed 的 privileged-teacher screening：真值不能作为
部署输入，且 geometry-static 不是最强固定图。因此随后检查由 teacher
快照筛出的 fixed candidate 16，并把同一候选池/标签接口放到 M24 做
尺度验证；这两项都只是进入多 seed 标签数据生成之前的证据门。

候选 16 的 60 步闭环检查随后完成：其 focus E-OSPA 为 67.5372，优于
geometry-static 的 73.3108，说明它确实是更强固定对照；纯任务风险
teacher 仍进一步达到 61.7041，相对改善 8.64%。基数误差和 posterior
分歧分别改善 16.34% 和 8.00%，attempted bytes 增加 1.28%，仍处于
预设的 ±2% 通信匹配带内。这个反证使 D12 的正信号不再只依赖弱的
geometry-static，但候选 16 仍是用同一 seed 的 teacher 快照筛出的，
正式实验必须改为 train seeds 选图、held-out seeds 评估。

M24-hard 的首个尺度快照也已完成。seed 7 的固定拓扑行为轨迹运行到
`t=75`，24 个节点和 16 个目标的过滤耗时 559.37 s；此时行为
E-OSPA 为 25.5087、基数误差为 0.9167，未出现整体失效。通用候选池
生成 28 个满足物理图、29 条边预算、节点度和每步最多替换 2 条边约束
的可选动作。纯 current task-risk 在这些动作上的相对风险跨度为
15.373%，最优动作相对注册静态图降低风险 2.346%，候选评分耗时
24.70 s。

这个快照结果最初只回答“监督量和动作接口能否扩展到 M24、且动作是否
仍有区分度”。为了进一步判断正信号是否只是 surrogate 自洽，随后从
同一个 `t=75` 本地后验快照启动六步条件续跑。所有 arm 共享静态前缀、
29 条边预算、物理图、测量与链路随机数，并要求 attempted bytes 相对
静态图偏差不超过 2%。

| Arm | `t=75–80` E-OSPA | 最差节点 E-OSPA | MAP-set 分歧 | 基数误差 | Attempted bytes |
|:--|--:|--:|--:|--:|--:|
| Geometry-static | 23.0892 | 47.3908 | 25.6338 | 0.8056 | 17,091,984 |
| Reliability dynamic | 23.1891 | 47.3611 | 26.5435 | 0.8333 | 17,120,928 |
| Posterior-discrepancy dynamic | 26.5522 | 47.3866 | 29.5231 | 1.0556 | 17,054,832 |
| Pure current task-risk teacher | **21.1925** | **35.9637** | **23.8603** | **0.6944** | 17,101,848 |
| Mean-CVaR task-risk teacher | 22.7065 | 42.5722 | 25.1841 | 0.7778 | 17,158,032 |
| Pure-CVaR task-risk teacher | 22.7065 | 42.5722 | 25.1841 | 0.7778 | 17,158,032 |

纯均值 current task-risk teacher 是六个已评估 arm 中的最优策略：相对
geometry-static，E-OSPA 改善 8.21%，最差节点 E-OSPA 改善 24.11%，
MAP-set 分歧改善 6.92%，基数误差改善 13.80%；attempted-byte 偏差
仅 0.0577%，拓扑不可行率为 0。修正续跑边界计数后，它的 churn 为
0.0227。Reliability 与 posterior-discrepancy 没有形成 tracking
收益；在这个窗口里增加尾部风险权重也弱于纯均值目标。补跑的 pure
CVaR 与 mean-CVaR 得到相同的 22.7065，因此没有留下未评估的风险聚合
候选。

这个结论仍是单 seed、六个条件步骤和有限候选池上的
privileged-teacher screening。它不能称为全局最优，也不能替代可部署
策略：teacher 评分读取真值，且 posterior 分歧从 0.8205 增至 0.8446，
所以目前证据只支持“tracking-primary 的策略存在性”，不支持所有一致性
指标同步改善。快照报告、五臂筛选和边界修正确认分别见
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_M24_HARD_N1_20260725_183112.md`、
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_221559.md`
和
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_222820.md`；
pure-CVaR 补充对照见
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_225246.md`。

### 13.5 X36 尺度检查：健康场景下仍无实质收益

M24 的参数和六个策略随后被冻结并原样迁移到 X36，没有利用 X36 结果
反向调整 teacher 权重。这个过程先后检查了三个 X36 场景，避免把
“节点更多”和“感知更困难”混成一个变量：

| 场景 | `t=75` 静态 E-OSPA | `t=75` 基数误差 | 三步数值最优 | 相对静态收益 | 判断 |
|:--|--:|--:|:--|--:|:--|
| X36-hard | 107.3636 | 12.3056 | Pure mean teacher | 0.358% | 滤波状态不健康，且收益可忽略 |
| X36-matched | 90.0227 | 8.6944 | 未进入策略比较 | — | 两项健康门槛均失败 |
| X36-clean-scale | 46.8048 | 2.4444 | Pure mean teacher | 0.961% | 状态健康，但未达到 5% 实用门槛 |

健康门槛把 E-OSPA 除以场景截断距离，把基数误差除以当前有效目标数；
要求两者分别不超过 0.50 和 0.25。X36-hard 的归一化结果为
0.716/0.513，X36-matched 为 0.600/0.362，不能把其策略差异解释成
可靠的尺度结论。X36-clean-scale 则为 0.312/0.102，且 33 个可行动作
的 task-risk 跨度达到 11.237%，因此它提供了一个健康且动作有区分度的
检查点。

在 X36-clean-scale 的 `t=75–77` 条件续跑中，所有策略共享同一个静态
前缀、44 条边预算、每步最多替换 3 条边、测量和链路随机数：

| Arm | E-OSPA | 最差节点 E-OSPA | MAP-set 分歧 | 基数误差 | Attempted bytes |
|:--|--:|--:|--:|--:|--:|
| Geometry-static | 46.3716 | 63.8858 | 35.7744 | 2.3981 | 19,957,824 |
| Reliability dynamic | 46.2599 | 63.8827 | **34.9606** | 2.3796 | 19,994,952 |
| Posterior-discrepancy dynamic | 48.4618 | 65.4138 | 36.7017 | 2.6111 | 19,910,736 |
| Pure current task-risk teacher | **45.9258** | 68.4314 | 35.8124 | **2.3704** | 20,016,096 |
| Mean-CVaR task-risk teacher | 46.2599 | **63.8827** | **34.9606** | 2.3796 | 19,994,952 |
| Pure-CVaR task-risk teacher | 46.2599 | **63.8827** | **34.9606** | 2.3796 | 19,994,952 |

纯均值 teacher 是当前有限候选池内的平均 E-OSPA 数值最优策略，但
0.961% 的收益远低于预先设定的 5% 实用门槛，并且最差节点从 63.8858
恶化到 68.4314，MAP-set 分歧也没有改善。更保守的
reliability/Mean-CVaR/Pure-CVaR 保护了尾部节点并改善 MAP-set 分歧，
但平均 E-OSPA 只改善 0.241%。所有 arm 的 attempted-byte 偏差均在
2% 内，拓扑不可行率为 0，所以这个负结论不能归因于通信预算或约束
失配。

因此，当前证据支持把纯均值 task-risk 保留为 **M24 的离线监督候选**，
但不支持把它称为 X36 的可扩展策略；更不能把读取真值的 teacher 当作
部署方法。继续把相同六个 arm 拉长或增加 seed 的信息增量有限。下一轮
应先改变方法，例如显式建模多轮信息传播或在目标中加入最差节点保护，
再从 D12/M24/X36 重新过门槛。M24 的 8.21% 结果也仍需更多 paired
seeds、更长窗口，以及只读取本地可用信息的模仿策略验证。

X36 的健康度和三步筛查报告分别见
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_X36_CLEAN_SCALE_N1_20260726_003744.md`
和
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_013455.md`。

### 13.6 M24 提议学习重构：集合预测而不是固定类别

六个设计 seed、54 个真值隔离行为状态上的第一版非线性提议器在训练
状态达到 54/54 top-16 捕获，但逐 seed 留一只达到 2/54；相同安全投影
器下的 96.296 个百分点差距说明，继续加宽同一个逐边分类器并不能解决
问题。更关键的是，teacher code 90/91/92 只是“从当前最优图中分别排除
一条已选边”后得到的候选编号。不同状态的同一个编号不具有稳定语义，
把它固定绑定到某一个输出头会人为制造跨 seed 的标签错位。

下一轮 M24 开发实验因此预先冻结为以下协议：

1. 数据覆盖全部九个已注册设计 seed
   `[7,11,17,19,23,27,29,31,37]`，每个 seed 使用真值隔离的
   action-80 行为从 `t=75` 运行到 `t=83`，共 81 个预决策状态。原六
   seed artifact 保持不可变，只追加 seed 27/31/37 的独立 shard。
2. 部署输入只保留接收节点内、编队对内和当前状态块内的相对标准化
   特征，不读取绝对 seed 身份、真值或未来回报。离线 teacher 仍可读取
   当前真值生成 3–4 张完整、精确 rolling-\(B=3\) 安全图，但它们只作
   开发监督，不是可部署动作。
3. 模型使用四个可交换的共享逐边评分头。每个状态在训练时枚举输出头
   与 3–4 张目标图之间的一一匹配，选择当前损失最小的匹配；因此学习
   目标是“覆盖这一组可接受图”，而不是复现 00/90/91/92 的任意编号。
4. 每个头对完整候选图做集合级 softmax：图分数为其入选跨编队边分数
   之和，候选集合由 teacher 正例、当前行为图以及 24 个冻结的真值
   隔离随机评分方向经同一个精确安全投影器产生。推理时仍只允许通过
   原 rolling-\(B=3\) 投影器输出图，并由至多排除一条已选边形成
   top-16 提议集。
5. 在查看新结果前冻结门槛：逐 seed 留一 top-16 状态捕获率至少 80%，
   每个 seed 至少捕获 6/9 状态，旧 10 个 value-bearing 状态捕获率
   至少 80%。三项同时通过才允许生成 H=3 配对回报；它不能直接授权
   critic、M24 效果结论或 X36 实验。

这一重构同时改变了数据覆盖、特征不变性和监督目标，但没有改变安全
层、每步三条跨编队边的动作预算或后续 tracking/通信门槛。若仍未通过，
应首先检查目标图在真值隔离候选库中的可辨识性和跨 seed 条件分布，而
不是继续扩大网络容量或提前运行 X36。

## 参考

1. K. Shen, P. Dong, Z. Jing, and H. Leung, “Consensus-Based Labeled Multi-Bernoulli Filter for Multitarget Tracking in Distributed Sensor Network,” *IEEE Transactions on Cybernetics*, 2022. https://doi.org/10.1109/TCYB.2021.3087521
2. K. Shen, C. Zhang, P. Dong, Z. Jing, and H. Leung, “Consensus-Based Labeled Multi-Bernoulli Filter With Event-Triggered Communication,” *IEEE Transactions on Signal Processing*, 2022. https://doi.org/10.1109/TSP.2022.3154227
3. J. K. Verma, J. K. Chhabra, and V. Ranga, “Track Consensus-Based Labeled Multi-Target Tracking in Mobile Distributed Sensor Network,” *IEEE Transactions on Mobile Computing*, 2024. https://doi.org/10.1109/TMC.2023.3333916
4. Y.-C. Liu et al., “When2com: Multi-Agent Perception via Communication Graph Grouping,” *CVPR*, 2020. https://openaccess.thecvf.com/content_CVPR_2020/html/Liu_When2com_Multi-Agent_Perception_via_Communication_Graph_Grouping_CVPR_2020_paper.html
5. J. Vanneste, S. Vanneste, A. E. C. Meyes, and T. Demeester, “Learning to Communicate Using Counterfactual Reasoning,” 2020. https://arxiv.org/abs/2006.07200
6. S. Kesper, S. Trimpe, and D. Baumann, “Learning When to Communicate at Scale in Multiagent Cooperative and Competitive Tasks,” *Proceedings of The 5th Annual Learning for Dynamics and Control Conference*, 2023. https://proceedings.mlr.press/v211/kesper23a.html
7. A. Ramachandran, A. S. Leong, S. Dey, and D. E. Quevedo, “Wireless Sensor Network Topology Reconfiguration for Multi-Target Tracking,” 2020. https://arxiv.org/abs/2004.07197
