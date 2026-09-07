# 移动机器人相遇后的融合：开发阶段结果

> 最新进展：v3 确认消融及 v4 的 20 个新种子验证已经完成，完整论文、图表和复现入口见
> [论文说明](../../papers/icra2027/README_CN.md) 与 [验证结果说明](VALIDATION_NOTES.md)。
> 下文保留 v1/v2 开发阶段快照，不能当作新种子验证结论。

2026-09-08；分支 `codex/icra`。用户限定：以现有融合为基础切换机器人场景，
允许小幅方法修改，不引入运动控制、规划、学习系统等大工程。

## 当前判断

相比上一轮几何分组的微弱信号，**局部视野下的新目标传播与过时存在判断**
已经产生明确、可定位的融合困难。最有价值的小改动是：把信息来源是否具有
观测依据与依据的新旧区分开，同时避免时效权重改变空间融合。

但当前候选仍不是通过全部条件的最终方法。`qualified_exist` 改善目标集合
恢复并保持共同目标定位，但继承了无新目标控制中的七个误报节点帧。
不能删除这个控制、改判定阈值或称其已满足 ICRA 投稿要求。

## 已完成的证据

- v1：3 个场景 × 3 个配对种子 × 6 个方案，54 次完整运行。
- v2：复用同九份模型、真值、观测、丢包数和拓扑缓存，增加两个融合消融，
  共 18 次运行。原输入、源码和结果未覆盖。
- Python 用穷举矩形指派独立于 MATLAB Hungarian 复算了 69,120 个节点帧，
  核对融合前后 OSPA、数量误差、匹配平方误差和匹配数，并计算 GOSPA 分解。
  这是指标独立复算，不是第三方方法验证。
- 所有通信方案实际序列化/还原后验，计入控制与失败包；固定包总流量相同。
- `common/`、`lmb/`、`multisensorLmb/` 未修改。

## 场景与假设

八个低速移动节点，沿旧轨迹缩放后的二维路径巡检，最高速度 2.96 m/s，
通信范围 24 m、感知范围 14 m；仍为预设运动学模拟，无真实机器人数据。
初始有两个目标，分离期间可能出现两个新目标；另一场景第 91 帧起一个
目标消失；第三场景保留候选出生先验但没有新目标，检查误报。

所有节点事先知道六个候选出生区域和出生时刻，实际活跃区域、目标数量及
偏移不告知滤波器。共享出生标签是简化，不包含独立标签匹配或量测驱动出生。
详细边界见 `PROTOCOL.md` 和 `EXTENSION_PROTOCOL.md`。

## 结果（3 个开发种子均值，OSPA 单位 m）

| 方案 | 分队后新生 | 反复相遇及目标消失 | 无新生控制 |
|---|---:|---:|---:|
| 普通 KLA | 3.3594 | 2.2988 | 0.9182 |
| 当前 FoV 缺失标签处理 | 3.3622 | 2.2762 | 0.9443 |
| 排除未观测先验 `lineage` | 2.6606 | 0.7696 | 0.8135 |
| 共享标签 MIL | 4.3544 | 3.9202 | 2.3967 |
| 直接观测时效加权 `recent` | 2.8708 | 1.3347 | 0.7482 |
| 资格＋空间/存在时效 `lineage_recent` | 2.5511 | 0.7041 | 0.7022 |
| 资格＋仅存在时效 `qualified_exist` | **2.5106** | **0.6616** | **0.6141** |

`qualified_exist` 相对 `lineage` 的共同匹配目标 RMSE 比值为
0.9999999 / 1.0000073；目标消失后误报 GOSPA 平方代价从 17.1 降到 11.2 m²。
但无新生控制的平均误报平方代价仍为 0.175 m²（7/2,880 个节点帧），
FoV 对照为零，故 v2 的两种组合都未通过完整筛选。`recent` 虽通过 v1 的
描述性门槛，共同目标定位却退化，也没有胜过 `lineage` 的事件场景集合误差。

这些是开发集结果，不是独立种子或跨真实场景验证。不将 MIL、当前视野处理
或现有 V284 开关改称新算法；完整多视域文献方法仍需核对/补充对照。

## 复现

```sh
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_reunion_fusion'); runRobotReunionPilot('full');"
/Users/dex/miniconda3/bin/python3 trials/icra_reunion_fusion/analyze_results.py
python3 trials/icra_reunion_fusion/make_extension_runner.py
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_reunion_fusion'); runRobotReunionExtension();"
/Users/dex/miniconda3/bin/python3 trials/icra_reunion_fusion/analyze_extension.py
```

数值与逐种子记录在 `summary_full.json`、`summary_extension.json`；完整输入和
估计在 `results/`；日志在 `RUN/ICRA/reunion_fusion_v1/`。

此前 `icra_group_tracking/source_sha256.json` 的四个公共函数散列差异已查明：
当前 LF 源文件转为 CRLF 后全部匹配旧散列，内容没有差异。新实验同时记录
原始及 LF 规范化散列。

实验注册时外网故障；之后已恢复访问官方 ICRA 网站和 arXiv。最新文献核对、
正式模板、确认误报机制以及论文建设在后续版本中继续，不改写冻结协议。
