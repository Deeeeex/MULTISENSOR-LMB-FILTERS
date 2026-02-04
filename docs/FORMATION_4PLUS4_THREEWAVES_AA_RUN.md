# 4+4 三波目标场景（AA）配置与结果记录

记录日期：2026-02-03  
脚本：`RUN/AA/runMultisensorFilters_formation_4plus4_threewaves_AA.m`  
说明：以下配置来自当前脚本；仿真结果为用户运行输出，未在此复现。

## 场景开关
- 目标分批出生：`staggeredBirths = true`（间隔 8）
- 分布式本地融合：`useDistributedFusion = true`
- 权重策略：`fusionWeighting = 'Metropolis'`
- Adaptive Weights 对比：`compareAdaptiveWeights = true`（`emaAlpha = 0.7`, `minWeight = 0.05`）
- 可视化：`makeGif = true`（FOV 半角 60 deg、范围 60000）

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

## FoV 配置
- 启用：`model.sensorFovEnabled = true`
- 半角：`sensorFovHalfAngleDeg = 60`
- 距离：`sensorFovRange = 60000`
- 视轴：传感器速度方向

## 拓扑连接（固定邻接）
编队内全联通，跨编队一一对应（按脚本配置）：
- 1 ↔ 8, 2 ↔ 7, 3 ↔ 6, 4 ↔ 5

邻接集合（来自运行输出）：
- 1: [1 2 3 4 8]
- 2: [1 2 3 4 7]
- 3: [1 2 3 4 6]
- 4: [1 2 3 4 5]
- 5: [1 5 6 7 8]
- 6: [2 5 6 7 8]
- 7: [3 5 6 7 8]
- 8: [4 5 6 7 8]

## 目标编队（Three Waves）
- 目标总数：`3 + 3 + 4 = 10`
- 三波目标：
  - 上方中间 → 向下
  - 右侧中间 → 向左
  - 下方中间 → 向上
- 组中心：`[0; 80]`、`[80; 0]`、`[0; -80]`
- 组内间距：`[30, 25, 20]`
- 速度：`0.45`
- 出生策略：分批出生（间隔 8），寿命 100

## 滤波配置
- 多传感器 LMB：`lmbParallelUpdateMode = 'AA'`
- 数据关联：`dataAssociationMethod = 'LBP'`
- 场景类型：`'Formation'`

## 仿真结果（用户运行输出）
```
=====================================
Per-Sensor OSPA (local fusion)
=====================================
Sensor 1: E-OSPA 4.088 -> 4.241 (0.154), RMSE 5.155 -> 5.113 (-0.042), neighbors=[1 2 3 4 8]
Sensor 2: E-OSPA 4.144 -> 4.288 (0.144), RMSE 4.028 -> 5.026 (0.999), neighbors=[1 2 3 4 7]
Sensor 3: E-OSPA 4.106 -> 4.249 (0.144), RMSE 4.741 -> 5.441 (0.700), neighbors=[1 2 3 4 6]
Sensor 4: E-OSPA 4.060 -> 4.271 (0.211), RMSE 4.736 -> 5.419 (0.683), neighbors=[1 2 3 4 5]
Sensor 5: E-OSPA 4.487 -> 4.324 (-0.163), RMSE 9.943 -> 5.899 (-4.044), neighbors=[1 5 6 7 8]
Sensor 6: E-OSPA 4.189 -> 4.198 (0.009), RMSE 4.628 -> 4.634 (0.007), neighbors=[2 5 6 7 8]
Sensor 7: E-OSPA 4.310 -> 4.240 (-0.070), RMSE 4.933 -> 5.437 (0.505), neighbors=[3 5 6 7 8]
Sensor 8: E-OSPA 4.089 -> 4.198 (0.109), RMSE 3.738 -> 4.712 (0.975), neighbors=[4 5 6 7 8]
=====================================
Consensus Metrics (base -> adaptive)
=====================================
Comprehensive (OSPA) consensus: 4.339 -> 3.868 (-0.471)
Position (RMSE) consensus: 18.424 -> 17.358 (-1.066)
Cardinality consensus: 0.485 -> 0.315 (-0.170)
```

## 仿真结果（更新后的 Adaptive 策略：加入 NIS）
```
=====================================
Per-Sensor OSPA (local fusion)
=====================================
Sensor 1: E-OSPA 3.908 -> 4.015 (0.107), RMSE 5.220 -> 4.230 (-0.991), neighbors=[1 2 3 4 8]
Sensor 2: E-OSPA 4.095 -> 4.176 (0.081), RMSE 4.520 -> 4.354 (-0.166), neighbors=[1 2 3 4 7]
Sensor 3: E-OSPA 4.083 -> 4.190 (0.107), RMSE 4.074 -> 4.237 (0.163), neighbors=[1 2 3 4 6]
Sensor 4: E-OSPA 4.091 -> 4.199 (0.108), RMSE 4.073 -> 4.777 (0.704), neighbors=[1 2 3 4 5]
Sensor 5: E-OSPA 4.492 -> 4.200 (-0.293), RMSE 10.546 -> 5.774 (-4.772), neighbors=[1 5 6 7 8]
Sensor 6: E-OSPA 4.157 -> 4.212 (0.055), RMSE 4.553 -> 4.646 (0.093), neighbors=[2 5 6 7 8]
Sensor 7: E-OSPA 4.388 -> 4.236 (-0.152), RMSE 6.907 -> 5.178 (-1.729), neighbors=[3 5 6 7 8]
Sensor 8: E-OSPA 4.199 -> 4.239 (0.039), RMSE 4.931 -> 4.954 (0.023), neighbors=[4 5 6 7 8]
=====================================
Consensus Metrics (base -> adaptive)
=====================================
Comprehensive (OSPA) consensus: 4.349 -> 3.811 (-0.538)
Position (RMSE) consensus: 19.098 -> 16.472 (-2.626)
Cardinality consensus: 0.421 -> 0.307 (-0.114)
```

### 增益（基于更新后的 Adaptive）
- 共识指标（越小越好）：
  - 综合 OSPA：减少 0.538（约 12.37%）
  - 位置 RMSE：减少 2.626（约 13.75%）
  - 基数一致性：减少 0.114（约 27.08%）
- 传感器平均变化（Adaptive - Base）：
  - 平均 E-OSPA：+0.0065（略降）
  - 平均 RMSE：-0.8344（提升）
