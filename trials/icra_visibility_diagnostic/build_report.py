"""Render verified source-geometry diagnosis without a method claim."""
from pathlib import Path
import hashlib
import json

import numpy as np

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
verification=json.loads((OUT/'DIAGNOSTIC_VERIFICATION.json').read_text())
assert verification['passed']
for name,value in verification['report_sha256'].items():assert sha(OUT/name)==value
boxes=json.loads((OUT/'BOX_VISIBILITY_DIAGNOSTIC.json').read_text())
cal=json.loads((OUT/'RANGE_RECALL_CALIBRATION.json').read_text())
alignment=json.loads((OUT/'RAW_ALIGNMENT_SEQUENCE.json').read_text())
visibility=['| 诊断子集 | 链路 | 独立源查询数 | 射线穿过更近检测框 | 源距离中位数（m） |',
            '| --- | --- | ---: | ---: | ---: |']
for item in boxes['summaries']:
    part=[row for row in boxes['rows'] if all(row[k]==item[k] for k in ['sequence','condition','group'])]
    ranges=[np.linalg.norm(np.array(row['predicted_xy'])-row['source_xy']) for row in part]
    label=item['sequence']+(' 真实漏检源' if item['group']=='repaired_true_source2' else '新增虚警源')
    visibility.append(f'| {label} | {item["condition"]} | {len(part)} | {item["center_ray_occluded"]} | {np.median(ranges):.3f} |')
probabilities=cal['full_fit']['probabilities_by_range']
probtable=['| 源距离（m） | 5 | 10 | 20 | 30 | 39 |','| --- | ---: | ---: | ---: | ---: | ---: |',
          '| 全九段拟合的平均检出概率 | '+' | '.join(f'{probabilities[str(r)]:.3f}' for r in [5,10,20,30,39])+' |']
text='''# GCE 负证据：源距离与点云对齐诊断

**所查案例呈现两类不同问题：远处目标的真实漏检，以及伴随明显跨车空间不一致的近处额外输出。** 连续无检本身无法区分这两类情况；上一轮历史折扣修复了前者，却在后者增加虚警。本轮完成诊断和检出率校准，尚未运行新的融合方法。

## 检测框遮挡没有解释已知漏检

从冻结检测中恢复已有的框尺寸、方向和中心，逐帧确认它们与原输入的二维检测中心一致。对候选实际使用的局部预测均值，检查源端到该位置的射线是否穿过一个完全在其前方的检测框。排除包含源或查询点本身的框，不放大框、不拟合角度阈值。全部 305 个查询的几何判断同时通过解析计算和 Shapely 多边形交点复核。

{visibility}

真实目标的可靠链路 70 个源端查询均为负增量，遮挡提示全部为零。间歇链路中，第 101–122 帧该源已没有目标 2 m 邻域内的保留预测，缺失的 22 帧单列。上述结果终止了“由现有检测框中心射线解释这个案例”的假设；它不证明物理上完全可见，因为未检测出的遮挡物及高度都未被该线索表示。

## 原始回波与空间对齐

固定查看真实案例第 53 帧，以及主要虚警标签 67 个源端记录的中间一帧（第 107 帧）。

| 检查 | 源 1 | 源 2 |
| --- | ---: | ---: |
| 真实目标 ID 5 的诊断车框内回波 | 2,045 | 26 |
| 虚警位置的另一源检测框内回波 | 290 | 1,038 |

这两个框用于诊断计数：真实案例采用既有标注框，虚警案例采用支持该位置的现有检测框。原始点云与原检测预处理后的计数相同。初次矩阵乘法输出过运行时警告，随后用有限值检查和显式逐坐标求和独立复算，四组计数完全一致；警告来源未确认。

![原始点云与检测框](raw_samples.png)

在虚警帧 107，现有姿态下高处回波的重叠为 214 个 0.5 m 网格；将源 2 仅用于诊断地平移 (−6.5, +0.5) m 后变为 505。三个固定高度范围的最大重叠均出现在这一位置。真实漏检帧 53 的三个范围都在原位达到最大重叠。

再对两个完整片段各取五个预先固定的等间隔帧，结果为：

- 虚警片段 v2x_0002：第 1 帧最大重叠位移为 (0, 0)；第 36 帧为 (−4, 0)；第 71、106、140 帧均为 (−6.5, +0.5) m。后面三个帧的高处回波重叠增至原位的 2.33–2.38 倍。
- 真实漏检片段 v2xt_0001：第 1、60、120、180、240 帧均在 (0, 0) 达到最大重叠。

这些文件均与已下载归档的 CRC、SHA 记录一致；检查的 `lidar_pose` 与 `true_ego_pose` 也相同。现有转换继续符合固定的作者位姿实现。这些证据支持该虚警片段存在明显空间不一致，尚不足以判定具体是定位、时间同步还是其他源数据问题，也不支持直接替换位姿或标注。

数据集论文明确讨论了车辆定位误差，并规定多车框不一致时采用 ego 标注。本文沿用这一已固定的评测约定。[V2X-Real，3.2 与 4.2 节](https://arxiv.org/html/2403.16034v1#S3.SS2)

## 距离对检出率有独立的预测信息

在开始拟合前，`RANGE_RECALL_PROTOCOL.md` 固定了单一模型和退出条件。使用全部九段 V2V 开发数据、36,836 个目标—传感器帧；每次排除一个完整采集记录，共六个记录分组。标签为该源已有检测在 2 m 内的一对一匹配，不使用任何跟踪结果或 V2X 拟合数据。

单一模型为 `sigmoid(a + b × distance/40)`，`b ≤ 0`，正则项固定为 `0.001 b²`。拟合样本按片段等权；没有搜索函数族、正则强度或匹配阈值。

| 概率模型 | 九段留出平均对数损失 | 九段留出平均 Brier |
| --- | ---: | ---: |
| 原名义概率 0.9 | 0.514723 | 0.159047 |
| 留出拟合常数 | 0.500286 | 0.157200 |
| 留出拟合距离模型 | 0.451976 | 0.144088 |

距离模型在八段上的对数损失优于常数，在 0000 上退步 0.071812；全部片段均保留。独立求解七组凸目标的一阶条件（六个留出拟合及一个全量拟合），与原优化器的最大参数差为 1.30×10⁻⁷。

{probtable}

该模型通过了预先固定的校准检查。它提供了继续检验“按源距离校准负证据”的依据；它估计的是包含遮挡、定位误差和检测缺失的平均召回，不能解释成真实的逐目标可见概率。V2X 的跨域校准和完整递归效果仍未验证。

## 后续实验边界

上一轮连续无检历史规则已按失败标准关闭。下一轮应固定一个距离校准方案，明确区分“仅改变融合中的额外负证据”与“所有方法共用新的局部检测概率模型”，并为所选方案提供对应的 No-age／GuardedScalar 对照。必须按完整片段和两种链路运行各自的递归，再判断 GCE 的收益是否超出概率校准本身；本轮没有产生可以替换论文主结果的性能证据。

复核入口和机器可读记录见 `README.md` 与 `DIAGNOSTIC_VERIFICATION.json`。
'''.format(visibility='\n'.join(visibility),probtable='\n'.join(probtable))
destination=OUT/'RESULTS_CN.md';assert not destination.exists();destination.write_text(text)
(OUT/'REPORT_BUILD.json').write_text(json.dumps(dict(passed=True,verification_sha256=sha(OUT/'DIAGNOSTIC_VERIFICATION.json'),builder_sha256=sha(Path(__file__)),report_sha256=sha(destination)),indent=2)+'\n')
print('VISIBILITY REPORT BUILT',flush=True)
