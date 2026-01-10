# Communication Level Impact Analysis

## 概述

`analyzeCommunicationLevelImpact.m` 是一个用于分析通信约束对多传感器融合滤波器性能影响的综合分析脚本。该脚本可以系统地评估不同通信级别（commConfig.level）对 GA 和 AA 融合模式的影响，并支持自适应融合权重的对比测试。

## 功能特性

### 通信级别测试

脚本支持 4 个通信级别的性能评估：

- **Level 0 - 理想通信**：无任何通信约束，所有量测完整传输
- **Level 1 - 带宽限制**：全局量测数量上限约束
- **Level 2 - 链路丢包**：带宽限制 + 固定概率丢包
- **Level 3 - 节点失联**：带宽限制 + 链路丢包 + 随机节点失联

### 融合模式对比

#### 基础融合模式
- **GA (Geometric Average)**：加权几何平均融合
- **AA (Arithmetic Average)**：加权算术平均融合

#### 自适应融合对比
脚本支持对比自适应融合开启/关闭的性能差异：
- `GA` vs `GA-Adaptive`
- `AA` vs `AA-Adaptive`

### 性能指标

脚本收集以下关键性能指标：

1. **OSPA Distance**（定位和基数精度）
   - Euclidean-OSPA：欧氏距离度量
   - 包含定位误差和基数误差的综合评估

2. **Cardinality Error**（目标数估计误差）
   - 估计目标数与真实目标数的绝对误差

3. **Runtime**（运行时间）
   - 完整仿真周期的执行时间

4. **通信统计**
   - 总丢包量
   - 平均传输率
   - 失联事件数量

## 使用方法

### 基本用法

```matlab
% 默认参数运行（对比 GA/AA 的自适应融合效果）
results = analyzeCommunicationLevelImpact();

% 只对比基础融合模式（不启用自适应融合）
results = analyzeCommunicationLevelImpact('EnableAdaptiveFusion', false);
```

### 自定义参数

```matlab
% 自定义试验次数和通信级别
results = analyzeCommunicationLevelImpact(...
    'NumberOfTrials', 20, ...
    'CommLevels', [0, 2, 3], ...
    'FusionModes', {'GA', 'AA'});

% 调整传感器配置
results = analyzeCommunicationLevelImpact(...
    'NumberOfSensors', 4, ...
    'ClutterRates', [5, 8, 10, 12], ...
    'DetectionProbabilities', [0.7, 0.75, 0.8, 0.85], ...
    'SensorPrecision', [5, 4, 3, 2], ...
    'GlobalMaxMeasurements', 30);

% 不生成图表，仅保存结果
results = analyzeCommunicationLevelImpact(...
    'GeneratePlots', false);
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|---------|------|
| `CommLevels` | numeric | `[0, 1, 2, 3]` | 测试的通信级别数组 |
| `FusionModes` | cell | `{'GA', 'AA'}` | 基础融合模式列表 |
| `EnableAdaptiveFusion` | logical | `true` | 是否启用自适应融合对比 |
| `NumberOfTrials` | numeric | `10` | Monte Carlo 试验次数 |
| `NumberOfSensors` | numeric | `3` | 传感器数量 |
| `ClutterRates` | numeric | `[5, 5, 5]` | 各传感器杂波率 |
| `DetectionProbabilities` | numeric | `[0.67, 0.70, 0.73]` | 各传感器检测概率 |
| `SensorPrecision` | numeric | `[4, 3, 2]` | 各传感器测量精度（q值） |
| `GlobalMaxMeasurements` | numeric | `25` | 全局量测数量上限 |
| `Verbose` | logical | `true` | 是否显示进度信息 |
| `GeneratePlots` | logical | `true` | 是否生成分析图表 |

## 输出说明

### 1. 控制台输出

运行时显示实时进度和摘要：

```
=====================================
Communication Level Impact Analysis
=====================================
Parameters:
  Communication levels: [0  1  2  3]
  Fusion modes: GA, AA, GA-Adaptive, AA-Adaptive
  Number of trials: 10
  Sensors: 3

=== Testing fusion mode: GA ===
  Communication level 0: done
  ...

=====================================
Analysis Summary
=====================================

--- GA Fusion Mode ---
Comm Level | OSPA (mean±std) | Card Error (mean±std) | Runtime (s)
-----------+-----------------+----------------------+------------
         0 |    5.123 ± 0.456 |        0.890 ± 0.123 |     12.34
         1 |    5.678 ± 0.523 |        1.012 ± 0.145 |     12.56
         ...

--- GA-Adaptive Fusion Mode ---
...
```

### 2. 可视化图表

脚本生成 5 个分析图表：

#### 图 1: OSPA Performance vs Communication Level
对比不同融合模式和通信级别下的 OSPA 距离

#### 图 2: Cardinality Error vs Communication Level
基数误差随通信级别变化的趋势

#### 图 3: Runtime vs Communication Level
运行时间分析

#### 图 4: Communication Statistics
- 上子图：总丢包量
- 下子图：平均传输率

#### 图 5: Combined Performance Heatmap
各融合模式在不同通信级别下的 OSPA 性能对比

### 3. 结果文件

脚本自动保存完整结果到 MAT 文件：
```
comm_level_impact_YYYYMMDD_HHMMSS.mat
```

文件包含以下字段：

- `metadata`：运行元数据（时间戳、参数、MATLAB 版本）
- `ospa`：OSPA 距离均值矩阵（融合模式 × 通信级别）
- `ospaStd`：OSPA 距离标准差
- `cardinalityError`：基数误差均值
- `cardinalityErrorStd`：基数误差标准差
- `runtime`：运行时间均值
- `runtimeStd`：运行时间标准差
- `communication`：通信统计（丢包量、传输率、失联事件）

## 技术细节

### 自适应融合实现

脚本集成了项目的自适应融合权重计算功能：

```matlab
model.adaptiveFusion.enabled = true;  // 启用自适应权重
model.adaptiveFusion.emaAlpha = 0.7;     // EMA 平滑系数
model.adaptiveFusion.minWeight = 0.05;    // 最小权重阈值
```

自适应权重在滤波器内部根据通信质量和传感器性能动态调整。

### 通信约束模型

使用 `applyCommunicationModel` 函数应用多级通信约束：

```matlab
commConfig.level = commLevel;  // 0-3
commConfig.globalMaxMeasurementsPerStep = maxMeasurements;
commConfig.pDrop = 0.2;  // 固定丢包率
commConfig.maxOutageNodes = 1;  // 最大失联节点数
```

### 参数顺序修复

修复了以下关键函数调用的参数顺序问题：

- **`computeSimulationOspa`**：正确顺序为 `(model, groundTruthRfs, stateEstimates)`
- **`runParallelUpdateLmbFilter`**：传入 `commStats` 参数以支持自适应权重计算

### 数组维度处理

修复了基数误差计算中的维度问题：

```matlab
% 错误：stateEstimates.labels 是 2xN 矩阵
estimatedCardinality = cellfun(@(x) length(x), stateEstimates.labels);

% 正确：使用列数获取目标数量
estimatedCardinality = cellfun(@(x) size(x, 2), stateEstimates.labels);
```

## 版本历史

### v1.1 (当前版本)
- [新增] 支持 `EnableAdaptiveFusion` 参数，对比自适应融合效果
- [新增] 扩展融合模式标签（如 'GA-Adaptive', 'AA-Adaptive'）
- [修复] `computeSimulationOspa` 参数顺序错误
- [修复] 基数误差计算的维度问题
- [修复] `errorbar` 绘图格式字符串错误

### v1.0
- [初始] 基础通信级别分析功能
- [初始] 支持 GA 和 AA 融合模式对比
- [初始] 完整的性能指标收集和可视化

## 相关文档

- [用户指南](USER_GUIDE_CN.md)：滤波器使用详细说明
- [通信仿真文档](COMMUNICATION_SIMULATION_CN.md)：通信模型详细定义
- [API 参考](API_REFERENCE.md)：函数接口说明

## 示例工作流

### 完整分析流程

```matlab
% 1. 设置路径
setPath;

% 2. 运行通信级别分析
results = analyzeCommunicationLevelImpact(...
    'EnableAdaptiveFusion', true, ...
    'FusionModes', {'GA', 'AA'}, ...
    'NumberOfTrials', 20);

% 3. 加载保存的结果（可选）
load('comm_level_impact_YYYYMMDD_HHMMSS.mat', 'commImpactResults');

% 4. 自定义分析或绘图
figure;
plot(results.metadata.parameters.CommLevels, results.ospa, 'o-');
xlabel('Communication Level');
ylabel('OSPA Distance');
legend(results.metadata.actualFusionModes);
```

### 对比分析

```matlab
% 对比自适应融合带来的性能提升
baseline = analyzeCommunicationLevelImpact('EnableAdaptiveFusion', false);
adaptive = analyzeCommunicationLevelImpact('EnableAdaptiveFusion', true);

% 计算提升比例
improvement = (baseline.ospa - adaptive.ospa) ./ baseline.ospa * 100;

fprintf('Average OSPA improvement with adaptive fusion:\n');
fprintf('  GA: %.2f%%\n', mean(improvement(1, :)));
fprintf('  AA: %.2f%%\n', mean(improvement(2, :)));
```

## 故障排除

### 常见问题

**Q: 脚本运行缓慢**
- A: 减少 `NumberOfTrials` 或限制 `CommLevels` 范围

**Q: 内存不足错误**
- A: 减少传感器数量或目标数量，或者减小仿真时长

**Q: 自适应融合未生效**
- A: 确保 `model.adaptiveFusion.enabled` 设置正确，且 `commStats` 已传入滤波器

## 贡献者

- 初始版本开发团队
- 自适应融合集成和优化

## 许可证

MIT License - 参见项目根目录的 LICENSE 文件。
