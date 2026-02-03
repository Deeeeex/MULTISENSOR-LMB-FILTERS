# 4+4 编队场景配置与结果记录

记录日期：2026-02-03  
脚本：`RUN/GA/runMultisensorFilters_formation_4plus4.m`  
说明：以下内容基于当前脚本配置整理，仿真结果为用户提供的运行输出，未在此复现。

## 场景开关
- 目标分批出生：`staggeredBirths = true`（间隔 8）
- 分布式本地融合：`useDistributedFusion = true`
- 权重策略：`fusionWeighting = 'Metropolis'`
- Adaptive Weights 对比：`compareAdaptiveWeights = true`（`emaAlpha = 0.7`, `minWeight = 0.05`）
- 可视化：`makeGif = true`（FOV 半角 60 deg、范围 60）

## 传感器配置
- 数量：`numberOfSensors = 8`
- 杂波率：`clutterRates = 3 * ones(1, 8)`
- 探测概率：`detectionProbabilities = 0.9 * ones(1, 8)`
- 量测噪声：`q = 3 * ones(1, 8)`

## 通信层配置（applyCommunicationModel）
- 等级：`commConfig.level = 2`
- 全局每步最大量测：`globalMaxMeasurementsPerStep = 80`
- 选择策略：`priorityPolicy = 'weightedPriority'`, `measurementSelectionPolicy = 'random'`
- 链路模型：`linkModel = 'fixed'`, `pDrop = 0.2`
- 最大失联节点：`maxOutageNodes = 1`

## 传感器运动与编队（CV）
- 运动类型：`sensorMotionConfig.motionType = 'CV'`
- 过程噪声：`processNoiseStd = 0.0`
- 两个 4 机编队（4+4），均从左向右匀速飞行
- 编队构型：`Leader3`（四机人字形），间距 `groupSpacing = 20`
- 编队中心：
  - 编队 A：`[-80; 35]`
  - 编队 B：`[-80; -35]`
- 速度：`[0.8; 0]`

## 拓扑连接（固定邻接）
编队内全联通，跨编队一一对应：
- 1 ↔ 5, 2 ↔ 6, 3 ↔ 7, 4 ↔ 8

对应邻接集合（从运行输出）：
- 1: [1 2 3 4 5]
- 2: [1 2 3 4 6]
- 3: [1 2 3 4 7]
- 4: [1 2 3 4 8]
- 5: [1 5 6 7 8]
- 6: [2 5 6 7 8]
- 7: [3 5 6 7 8]
- 8: [4 5 6 7 8]

## 目标编队（Formation）
- 目标总数：`3 + 3 + 4 = 10`
- 三组目标（右上 / 正右 / 右下）向中心运动
- 组中心：
  - 组 1：`[70; 80]`（Triangle）
  - 组 2：`[80; 0]`（Triangle）
  - 组 3：`[70; -80]`（Leader3）
- 组内间距：`[30, 25, 20]`
- 目标速度：`0.45`（朝向 `targetCenter = [0; 0]`）
- 出生策略：分批出生（间隔 8），寿命 100

## 滤波配置
- 多传感器 LMB：`lmbParallelUpdateMode = 'GA'`
- 数据关联：`dataAssociationMethod = 'LBP'`
- 场景类型：`'Formation'`

## 仿真结果（用户运行输出）
```
=====================================
Per-Sensor OSPA (local fusion)
=====================================
Sensor 1: E-OSPA 2.500 -> 2.336 (-0.164), RMSE 1.596 -> 1.485 (-0.110), neighbors=[1 2 3 4 5]
Sensor 2: E-OSPA 2.502 -> 2.361 (-0.141), RMSE 1.497 -> 1.446 (-0.051), neighbors=[1 2 3 4 6]
Sensor 3: E-OSPA 2.373 -> 2.412 (0.039), RMSE 1.551 -> 1.487 (-0.063), neighbors=[1 2 3 4 7]
Sensor 4: E-OSPA 2.583 -> 2.443 (-0.140), RMSE 1.653 -> 1.547 (-0.106), neighbors=[1 2 3 4 8]
Sensor 5: E-OSPA 2.536 -> 2.453 (-0.083), RMSE 1.564 -> 1.567 (0.004), neighbors=[1 5 6 7 8]
Sensor 6: E-OSPA 2.633 -> 2.458 (-0.175), RMSE 1.517 -> 1.553 (0.036), neighbors=[2 5 6 7 8]
Sensor 7: E-OSPA 2.667 -> 2.209 (-0.458), RMSE 1.588 -> 1.578 (-0.009), neighbors=[3 5 6 7 8]
Sensor 8: E-OSPA 3.087 -> 2.324 (-0.764), RMSE 1.549 -> 1.545 (-0.004), neighbors=[4 5 6 7 8]
=====================================
Consensus Metrics (base -> adaptive)
=====================================
Comprehensive (OSPA) consensus: 2.203 -> 1.686 (-0.517)
Position (RMSE) consensus: 1.942 -> 1.440 (-0.502)
Cardinality consensus: 0.604 -> 0.158 (-0.446)
```
