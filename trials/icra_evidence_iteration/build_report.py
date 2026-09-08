"""Preserve the full negative JE round and the diagnostics that motivated ECR."""
from pathlib import Path
import json

OUT=Path(__file__).resolve().parent
data=json.loads((OUT/'summary_development.json').read_text())
false=json.loads((OUT/'false_output_diagnosis.json').read_text())
names={'joint_evidence':'JE','joint_evidence_recency':'JE-R','qualified_exist':'ER',
       'lineage':'No-age','conservative_recency':'CR','confirmation_recency':'CGR',
       'mil_support':'MIL-AM','tc_ospa2_w5':'TC-5','tc_ospa2_w10':'TC-10','local':'Local'}
conditions={'reliable':'可靠链路','intermittent':'间歇链路'}
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])
rows=[]
for r in data['aggregate']:
    rows.append([conditions[r['condition']],names[r['arm']]]+[f"{r[k]['mean']:.5f}" for k in ['ospa','miss2','false2']])
paired=[]
for r in data['paired']:
    if r['reference'] not in ['lineage','qualified_exist']:continue
    p=r['ospa'];paired.append([conditions[r['condition']],names[r['candidate']],names[r['reference']],
                              f"{p['mean']:+.5f} [{p['low']:+.5f}, {p['high']:+.5f}]",f"{r['ospa_wins']}/9"])
diagnostic=[]
for condition in conditions:
    for arm in ['qualified_exist','lineage']:
        group=[r for r in false['runs'] if r['condition']==condition and r['arm']==arm]
        count=sum(r['false'] for r in group);near=sum(r['false_near_measurement_3m'] for r in group)
        diagnostic.append([conditions[condition],names[arm],count,near,f'{near/count:.1%}'])
text='''# 联合当前证据：完整开发实验记录

两种联合证据方法均未改善原 ER 的完整队列平均 OSPA，不能作为本轮论文主方法。
保留全部 9 个序列、两种链路、两种候选及原 ER 对照，不按结果筛除序列。
这是已经用于方法开发的数据；无独立测试集胜出或通用改进结论。

## 固定方法与实现边界

完整定义与运行前的两处单元检查修正在 [PROTOCOL.md](PROTOCOL.md)。记局部
更新后的 logit 为 l_j，当前更新增量为 d_j，普通/时效权重为 b_j/q_j，
空间池的对数归一化常数为 h。对于至少两个具有正权重的合格且有该标签的
来源，且没有合格的缺标签删失项，JE 使用 sum(b_j(l_j-d_j))+sum(d_j)+h，
JE-R 使用 sum(q_j(l_j-d_j))+sum(d_j)+h；其余情形分别退回 No-age 和 ER。

这是分离预测先验与当前局部更新的近似 Bernoulli 存在概率规则。关联更新
增量不等于独立且已校准的测量似然比，不同来源可以共享历史或检测误差；
不能据此声称精确中央 Bayes 更新。分离先验与似然共识已有先例，见
[Fantacci 2015](https://flore.unifi.it/handle/2158/1003256)。空间融合、检测、
出生、FoV、删失资格、时效参数、剪枝及无线随机数均沿用冻结版本。

每个 Bernoulli 增加一个 float64 更新增量：216 B/object，加 32 B 包头。
每次本地更新重写该增量，接收不能产生新的本地观测时间。

## 完整队列结果

OSPA 单位 m；漏检和虚假项是 GOSPA 平方分解中的平均代价，单位 m²。
表中按序列等权平均，不能把序列相关的逐帧样本当独立重复。

'''+table(['链路','方法','OSPA ↓','漏检代价 ↓','虚假代价 ↓'],rows)+'''

下表为候选减参照的配对 OSPA 差。区间为 9 个序列的 10000 次百分位
bootstrap，固定种子 8301，未校正多重比较；负数才表示候选更好。

'''+table(['链路','候选','参照','差值及 95% 区间','胜出序列'],paired)+'''

联合证据降低了漏检代价，却提高了虚假目标代价。可靠链路的 JE/JE-R
与 No-age 差距尤其说明：加强新信息本身不能排除重复出现的错误检测。

## 后验诊断及下一轮动机

在原 ER 和 No-age 已见输出上，先按原 12 m 一对一指派确定虚假输出，
再统计其是否位于任一当前输入检测的 3 m 内。3 m 仅为诊断分箱，未用于
跟踪或调整结果；这是节点—帧输出实例数，不是独立车辆数。

'''+table(['链路','方法','虚假输出实例','邻近当前检测','比例'],diagnostic)+'''

因此主要问题不能简单归为失去检测支撑的漂移轨迹。随后从冻结的原始检测
文件取回 38038 条分数，逐帧核验位置、顺序和裁剪与原 MAT 输入完全一致。
12 m 指派下，匹配检测的分数中位数为 0.5811，未匹配为 0.2598。
这提供下一轮检验连续证据上限的动机，不构成其有效性证据。完整分箱和
原始诊断行分别在 detection_score_diagnosis.json / .csv。

## 核验与复现

'''+f"独立重算 {data['audited_node_frames']} 个节点—帧输出；其中原 ER 的全部 {data['original_er_bitwise_parity_node_frames']} 个节点—帧输出与冻结原结果逐位一致。验证 {data['source_hashes_verified']} 个源码及注册输入哈希。"+'''
同时核验裁剪、真实/模拟无线输入、存在概率解析值、输出基数提取和同输入
反事实。所有 18 个完整压缩结果位于 results_development，逐序列数据为
development_runs.csv，统计与诊断为 summary_development.json。

从仓库根目录运行：

```sh
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_evidence_iteration'); checkJointEvidence();"
./tmp/external_baselines/venv/bin/python trials/icra_evidence_iteration/analyze_evidence.py --output-dir tmp/je_independent_audit
```

需要重新跟踪时使用 runJointEvidenceReplay(0,8,false)，先复制输出目录以
保留本轮冻结原件。make_replay.py 是历史生成/冻结脚本，不是重跑入口。
本目录不修改论文；论文是否重写由后续完整主实验结果决定。
'''
(OUT/'README_CN.md').write_text(text)
print('JE full negative report written from verified summary.')
