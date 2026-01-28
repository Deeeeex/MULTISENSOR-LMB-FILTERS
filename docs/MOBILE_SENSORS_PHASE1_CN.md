# Mobile Sensor Support (Phase 1)

## 概述

Phase 1 实现了机动传感器的基础框架支持，允许传感器按照恒定速度（CV）运动模型在观测空间中移动。传感器运动会影响目标测量方程，使用相对位置而非绝对位置进行测量。

## 修改的文件

### 1. `generateMultisensorModel.m` - 添加传感器运动配置

#### 新增字段
```matlab
model.sensorMotionEnabled           % 是否启用传感器运动（默认 false）
model.sensorMotionType             % 运动模型类型（'CV'）
model.sensorProcessNoiseStd         % 传感器运动过程噪声标准差
model.sensorInitialState             % 传感器初始状态（cell 数组）
model.sensorProcessNoise            % 传感器过程噪声协方差（cell 数组）
```

#### 使用方法
```matlab
% 启用传感器运动
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CV';
sensorMotionConfig.processNoiseStd = 0.1;
sensorMotionConfig.initialStates = {
    [-50; 0; 0.5; 0],    % 传感器 1 初始状态
    [50; 0; -0.5; 0],      % 传感器 2 初始状态
    [0; 70; 0; -0.2]        % 传感器 3 初始状态
};

model = generateMultisensorModel(numberOfSensors, clutterRates, ...
    detectionProbabilities, q, 'GA', 'LBP', 'Fixed', sensorMotionConfig);
```

#### 兼容性
- 默认 `sensorMotionEnabled = false`，保持向后兼容
- 通过 `varargin` 参数传递配置，不破坏现有代码

### 2. `generateMultisensorGroundTruth.m` - 传感器轨迹生成

#### 修改内容
1. **新增输出参数** `sensorTrajectories`：记录每个传感器在仿真期间的轨迹
2. **传感器轨迹生成**：使用 CV 运动模型更新传感器位置
3. **测量方程修改**：使用相对位置测量（目标位置 - 传感器位置）

#### 传感器运动模型
```matlab
% CV (Constant Velocity) 运动模型
传感器状态：[sx, sy, vx, vy]（4 维）

状态转移：
sensorTrajectory{s}(:, t) = model.A * sensorTrajectory{s}(:, t-1) + process_noise

其中：
- model.A = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1]
- process_noise ~ N(0, model.sensorProcessNoise{s})
```

#### 测量方程（相对位置）
```matlab
if model.sensorMotionEnabled
    % 机动传感器：测量目标相对于传感器的位置
    sensorPos = sensorTrajectories{s}(1:2, t);  % 当前传感器位置
    targetPos = x(1:2);                      % 目标位置
    relativePos = targetPos - sensorPos;         % 相对位置
    
    % 测量 = 传感器位置 + 相对位置 + 噪声
    y = sensorPos + model.C{s} * [relativePos; 0; 0] + noise;
else
    % 静态传感器（原代码）
    y = model.C{s} * x + noise;
end
```

#### 输出格式
```matlab
[sensorTrajectories] = sensorTrajectories;  % 新增输出
sensorTrajectories{s} = zeros(4, simulationLength);
```

### 3. `generateLmbSensorAssociationMatrices.m` - 支持机动传感器预测

#### 函数签名修改
```matlab
% 原函数签名
[associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, z, model, s)

% 新函数签名（兼容）
[associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, z, model, s, currentTime)
```

#### 预测测量修改
```matlab
% 原代码（静态传感器）
muZ = model.C{s} * objects(i).mu{j};

% 改为（机动传感器）
if model.sensorMotionEnabled && nargin >= 4
    sensorPos = model.sensorTrajectories{s}(1:2, currentTime);
    targetPos = objects(i).mu{j}(1:2);
    muZ = sensorPos + model.C{s} * [targetPos - sensorPos; 0; 0];
else
    muZ = model.C{s} * objects(i).mu{j};
end
```

### 4. `runParallelUpdateLmbFilter.m` - 传递传感器轨迹

#### 函数签名修改
```matlab
% 原函数签名
function stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats)

% 新函数签名（支持机动传感器）
function stateEstimates = runParallelUpdateLmbFilter(model, measurements, commStats, sensorTrajectories)
```

#### 传感器轨迹传递
```matlab
% 在函数开头添加（第 28-32 行附近）
% Phase 1: Mobile Sensor Support - Pass sensor trajectories to model
if nargin >= 3 && ~isempty(sensorTrajectories)
    model.sensorTrajectories = sensorTrajectories;
end
```

### 5. `runIcLmbFilter.m` - 传递传感器轨迹

#### 函数签名修改
```matlab
% 原函数签名
function stateEstimates = runIcLmbFilter(model, measurements)

% 新函数签名（支持机动传感器）
function stateEstimates = runIcLmbFilter(model, measurements, sensorTrajectories)
```

#### 传感器轨迹传递
```matlab
% 在函数开头添加（第 23-27 行附近）
% Phase 1: Mobile Sensor Support - Pass sensor trajectories to model
if nargin >= 3 && ~isempty(sensorTrajectories)
    model.sensorTrajectories = sensorTrajectories;
end
```

### 6. 调用脚本修改 - 传递传感器轨迹

#### `runMultisensorFilters_mobile.m` 修改（第 76-81 行附近）
```matlab
% 原代码
stateEstimates = runParallelUpdateLmbFilter(model, measurements);

% 改为
stateEstimates = runParallelUpdateLmbFilter(model, measurements, [], sensorTrajectories);
```

注意：第二个参数 `[]` 是 `commStats`，传递空值表示没有通信统计。

### 5. `runIcLmbFilter.m` - 传递传感器时间

#### 修改位置（第 38 行附近）
```matlab
if model.sensorMotionEnabled
    [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, measurements{s, t}, model, s, t);
else
    [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, measurements{s, t}, model, s);
end
```

## 新增脚本

### 1. `runMultisensorFilters_mobile.m` - 演示机动传感器

功能：运行完整的多传感器滤波器流程，可视化传感器轨迹和跟踪结果。

使用示例：
```matlab
runMultisensorFilters_mobile;
```

### 2. `test_mobile_sensors.m` - 单元测试

功能：测试静态和机动传感器配置，对比性能。

测试内容：
- Test 1：静态传感器（baseline）
- Test 2：机动传感器（CV 运动）
- Test 3：不同运动噪声水平

使用示例：
```matlab
test_mobile_sensors;
```

### 3. `runMultisensorFilters_formation.m` - 分布式编队场景（新增）

功能：编队传感器 + 编队目标的分布式本地融合示例，默认包含通信层（Level 1）、Metropolis 权重、Adaptive Weights 对比、动图输出与视角范围可视化。

特点：
- **传感器编队**：Leader+4（人字形），左→右运动
- **目标编队**：3-3-4 三组（右上/正右/右下）向中心运动
- **分布式融合**：每个传感器本地融合邻居测量，无中心节点
- **通信层**：默认 Level 1（带宽约束）
- **权重策略**：Metropolis / Uniform 可切换
- **Adaptive Weights**：开/关对比并输出两份 GIF
- **视角范围**：±60° 以虚线展示

使用示例：
```matlab
runMultisensorFilters_formation;
```

核心开关与参数（脚本顶部）：
```matlab
useDistributedFusion = true;    % 分布式本地融合
leaderSensor = 1;               % 可视化展示的传感器节点
fusionWeighting = 'Metropolis'; % 'Metropolis' / 'Uniform'
compareAdaptiveWeights = true;  % 是否对比 Adaptive Weights
commConfig.level = 1;           % 通信层等级（默认 Level 1）
```

## 测试方法

### 快速测试
```matlab
% 运行基础测试
test_mobile_sensors;

% 运行完整演示
runMultisensorFilters_mobile;
```

### 对比测试
```matlab
% 对比静态 vs 机动
sensorMotionConfig = struct();
sensorMotionConfig.enabled = false;
model_static = generateMultisensorModel(3, [5 5 5], [0.9 0.9 0.9], [4 3 2], 'GA', 'LBP', 'Fixed', sensorMotionConfig);

sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CV';
sensorMotionConfig.processNoiseStd = 0.1;
sensorMotionConfig.initialStates = {...};
model_mobile = generateMultisensorModel(3, [5 5 5], [0.9 0.9 0.9], [4 3 2], 'GA', 'LBP', 'Fixed', sensorMotionConfig);
```

## 参数说明

### 传感器运动配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|---------|------|
| `enabled` | boolean | false | 是否启用传感器运动 |
| `motionType` | char | 'CV' | 运动模型类型（仅支持 'CV'） |
| `processNoiseStd` | double | 无 | 传感器运动过程噪声标准差 |
| `initialStates` | cell | 无 | 各传感器初始状态 [x, y, vx, vy] |

### 传感器初始状态格式
```matlab
sensorInitialStates = {
    [-50; 0; 0.5; 0],    % 传感器 1
    [50; 0; -0.5; 0],      % 传感器 2
    [0; 70; 0; -0.2]        % 传感器 3
};
```

## 技术细节

### 1. 相对位置测量原理

目标状态：`x = [px, py, vx, vy]`

传感器状态：`s = [sx, sy, svx, svy]`

相对位置：
```
Δx = px - sx
Δy = py - sy
```

测量模型：
```
z = s(1:2) + [I 0] * [Δx; Δy] + w
```
其中 `w ~ N(0, Q)` 为测量噪声

### 2. CV 运动模型

状态转移矩阵：
```
A = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1]
```

过程噪声：
```
R_s = (σ_s²) * I_4
```

传感器状态更新：
```
s_t = A * s_{t-1} + v, v ~ N(0, R_s)
```

### 3. 滤波器适配

在滤波器中预测测量时，需要知道当前传感器的位置。为此：
- `sensorTrajectories` 在 ground truth 生成时记录
- 传递 `currentTime` 参数给 `generateLmbSensorAssociationMatrices`
- 在滤波器循环中使用 `model.sensorTrajectories{s}(1:2, t)`

## 已知限制

### Phase 1 限制
1. **仅支持 CV 模型**：恒定速度运动，未实现转弯、加速等复杂运动
2. **无边界约束**：传感器可能飞出观测空间
3. **无传感器协同**：各传感器独立运动
4. **测量模型简化**：假设理想相对位置测量，未考虑传感器姿态、误差椭圆等

### 兼容性
- 所有现有代码在 `sensorMotionEnabled = false` 时保持不变
- 向后兼容性完整

## 阶段2已实现功能

### 新增文件
1. **`common/generateMultisensorModelEnhanced.m`** - 增强模型生成器
2. **`common/generateMultisensorGroundTruthEnhanced.m`** - 增强数据生成器
3. **`common/compareSensorMotionPerformance.m`** - 性能对比分析脚本
4. **`demo_ct_motion.m`** - CT 运动模型演示

### 新增功能

#### 1. CT（Coordinated Turn）运动模型
- **恒定转弯率**：`sensorTurnRate = 0.05` rad/step
- **转弯噪声**：白噪声或恒定转弯
- **速度旋转**：在时间步内进行旋转变换
- **状态转移**：
  ```matlab
  cos_w = cos(turn_rate * T);
  sin_w = sin(turn_rate * T);
  new_vel = [cos_w * vx - sin_w * vy;
               sin_w * vx + cos_w * vy];
  ```

#### 2. 编队飞行（Formation Flight）
- **编队类型**：
  - `Line`：直线编队
  - `Triangle`：三角编队
  - `Circle`：圆形编队
- **相对位置**：传感器围绕中心点保持固定几何关系
- **编队移动**：中心点统一移动，所有传感器跟随

#### 3. 自适应速度
- **速度限制**：`maxSensorVelocity`
- **自适应调整**：超速时按比例缩放速度
- **实时约束**：在轨迹生成时应用

#### 4. 传感器调度
- **可用性矩阵**：`sensorSchedule`（sensors × time）
- **主动/被动模式**：传感器可在某些时间步不可用
- **隐藏位置**：将传感器移出观测范围（如 `[9999, 9999]`）

#### 5. 性能分析
- **对比维度**：
  - 运动类型：Static、CV、CT、Formation
  - 噪声水平：多个噪声标准差
  - 试验次数：Monte Carlo 仿真
- **输出指标**：
  - E-OSPA、H-OSPA
  - 运行时间
  - 统计分析（均值、标准差）
- **可视化**：
  - 误差条形图
  - 运行时间图
  - 热力图

### 使用示例

#### CT 运动模型
```matlab
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CT';
sensorMotionConfig.processNoiseStd = 0.1;
sensorMotionConfig.turnRate = 0.05;
sensorMotionConfig.turnModel = 'WhiteNoiseTurn';

model = generateMultisensorModel(3, [5 5 5], [0.9 0.9 0.9], ...
    [4 3 2], 'GA', 'LBP', 'Fixed', sensorMotionConfig);
```

#### 编队飞行
```matlab
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'Formation';
sensorMotionConfig.formation = 'Triangle';
sensorMotionConfig.formationSpeed = 1.0;
sensorMotionConfig.formationCenter = [0; 0];

model = generateMultisensorModel(3, [5 5 5], [0.9 0.9 0.9], ...
    [4 3 2], 'GA', 'LBP', 'Fixed', sensorMotionConfig);
```

#### 性能对比
```matlab
% 对比 Static、CV、CT 三种运动模式
compareSensorMotionPerformance(...
    'MotionTypes', {'Static', 'CV', 'CT'}, ...
    'NoiseLevels', [0.05, 0.1, 0.2], ...
    'NumberOfTrials', 10);

% 或使用增强生成器
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CT';
model = generateMultisensorModelEnhanced(3, [5 5 5], [0.9 0.9 0.9], ...
    [4 3 2], 'GA', 'LBP', 'Fixed', sensorMotionConfig);
```

## 分布式一致性指标（新增）

针对“每个传感器本地融合”的场景，除了单节点精度指标外，新增两个一致性指标，用来衡量**编队内部对目标的共同认知一致性**（不依赖真值）：

### 1) 位置一致性（Position Consensus）
用**平均两两 OSPA**衡量各节点估计集合的一致性：

```
C_pos(t) = (2 / (S*(S-1))) * Σ_{i<j} OSPA( X_i(t), X_j(t) )
```

数值越小表示越一致。实现中对两个方向的 OSPA 取平均（对称化）。

### 2) 基数一致性（Cardinality Consensus）
对每个时刻的估计目标数取**中位数偏差的平均值**（MAD）：

```
C_card(t) = (1/S) * Σ_s | n_s(t) - median(n(t)) |
```

数值越小表示越一致。

### 输出位置
在 `runMultisensorFilters_formation.m` 中默认输出：
- 时间序列曲线（Position / Cardinality）
- 全程均值（并在 Adaptive 对比时输出差值）

## 下一步（Phase 3）

### 计划实现
1. **高级可视化**：
   - `plotMultisensorResults.m` 完整集成传感器轨迹
   - 3D 可视化（如扩展到3D）
   - 动画生成
   
2. **传感器协作**：
   - 多传感器协同优化
   - 传感器间通信模型
   
3. **复杂场景**：
   - 传感器故障/死亡
   - 动态传感器添加/移除
   
4. **自适应策略**：
   - 基于目标分布的传感器调度
   - 在线参数学习

### 已知限制

### Phase 2 限制
1. **简化 CT 模型**：未实现转弯角约束
2. **无碰撞检测**：传感器可能重叠
3. **固定编队**：编队形状不可动态调整
4. **调度简化**：仅支持预定义调度表

### 兼容性
- 所有 Phase 1 功能保持兼容
- 新功能通过 `motionType` 标志启用
- 原有代码在默认配置下不变

## 故障排除

### 常见问题

**Q: 传感器轨迹异常**
- A: 检查 `processNoiseStd` 是否过大，调整初始状态

**Q: 测量不准确**
- A: 确认传感器位置传递正确，检查 `sensorTrajectories` 字段

**Q: 向后兼容问题**
- A: 确保旧代码中 `sensorMotionEnabled` 默认为 false

## 示例工作流

### 完整流程
```matlab
% 1. 配置传感器运动
sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CV';
sensorMotionConfig.processNoiseStd = 0.1;
sensorMotionConfig.initialStates = {
    [-80; 0; 0.3; 0], [0; 80; -0.3; 0], [80; 0; 0; -0.3]
};

% 2. 生成模型
model = generateMultisensorModel(3, [5 5 5], [0.9 0.9 0.9], [4 3 2], ...
    'GA', 'LBP', 'Fixed', sensorMotionConfig);

% 3. 生成数据和传感器轨迹
[groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruth(model);

% 4. 运行滤波器
stateEstimates = runParallelUpdateLmbFilter(model, measurements);

% 5. 评估性能
[eOspa, hOspa] = computeSimulationOspa(model, groundTruthRfs, stateEstimates);
fprintf('E-OSPA: %.3f\n', mean(eOspa));

% 6. 可视化（需要修改 plotMultisensorResults 以支持传感器轨迹）
plotMultisensorResults(model, measurements, groundTruth, stateEstimates, groundTruthRfs);
```

## 版本历史

### v1.1 (Phase 1)
- [修复] `sensorProcessNoise` 维度错误（从 `eye(4,1)` 改为 `eye(4)`）
- [修复] 传感器轨迹传递到滤波器（添加可选参数 `sensorTrajectories`）
- [更新] `runParallelUpdateLmbFilter` 和 `runIcLmbFilter` 支持接收传感器轨迹

### v1.0 (Phase 1)
- [新增] 传感器运动配置结构
- [新增] CV 运动模型实现
- [新增] 相对位置测量支持
- [新增] `sensorTrajectories` 输出
- [修改] 4 个核心函数以支持机动传感器
- [新增] 2 个测试脚本
- [兼容] 完整向后兼容性

## 参考文档

- [用户指南](USER_GUIDE_CN.md)
- [多传感器用户指南](MULTISENSOR_GUIDE_CN.md)（待创建）
- [API 参考](API_REFERENCE.md)

## 贡献者

Phase 1 实现团队
