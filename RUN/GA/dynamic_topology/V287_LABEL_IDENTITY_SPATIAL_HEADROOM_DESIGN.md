# V287：同标签空间改进是否依赖身份不一致

Status: completed saved-result analysis; no runtime candidate. Findings are in
`V287_LABEL_IDENTITY_SPATIAL_HEADROOM_FINDING.md`.

复用 V282/V284 的 X36、seed 1301、前 40 步和 V285 的全部新增匹配查询。
先确认场景是每个真实目标对应一个初始出生位置、全程 24 个目标；在这个
明确约定下，出生标签可作为额外诊断锚点，但不改原有无标签 OSPA/RMSE。

比较两组已存输出的出生标签—几何匹配关系，并统计同标签输出在不同节点
是否匹配到不同位置目标。另报最近真值位置，避免仅把一对一指派约束造成
的差异解释成身份冲突；报告发生分歧时的真值位置距离，不新设距离阈值。
全网同标签对与实际已投递边分别统计。这些是轮末几何对应诊断，不是已
验证的轨迹身份、标签切换率或该轮发送前的可见信息。

在 V285 原来的全部查询上保留原理想选源结果，再增加两个更严格的来源
限制：来源在其自身时刻的一对一匹配、或最近真值位置，与接收端查询目标
一致。两个限制分别计算，不混为一个真值身份保证；继续允许保留接收端。
使用当前轮末和上一轮预测一步两层，区分全网、物理一跳、实际入邻居。
再按查询是否符合出生身份分层。不筛掉不好看的查询，不改变官方门槛。

如果限制后主要改进空间消失，说明应优先解决标签关联/来源可比性，而非
继续加快同标签信息传递；如果仍明显存在，再检查新位置观测的来源和时间。
所有身份限制使用真值，都是诊断条件，不是可部署的控制器特征。无滤波
重跑、无新消息、不增加主表方法行；L2 exploratory，self-check only。

执行：

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeLabelIdentitySpatialHeadroomV287('RUN/GA/dynamic_topology/evidence/tracking_aligned_v285/x36_same_label_spatial_availability_seed1301/SAME_LABEL_SPATIAL_AVAILABILITY_V285.mat');"
```
