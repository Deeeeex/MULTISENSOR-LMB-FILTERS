"""Write the complete ECR development and synthetic readout from audit outputs."""
from pathlib import Path
import json
OUT=Path(__file__).resolve().parent
real=json.loads((OUT/'summary_development.json').read_text())
cases=json.loads((OUT/'summary_cases.json').read_text())
cal=json.loads((OUT/'calibration.json').read_text())
NAMES={'ceiling_association':'ECR-A','ceiling_score':'ECR-S','ceiling_calibrated':'ECR-C',
       'qualified_exist':'ER','lineage':'No-age','conservative_recency':'CR','confirmation_recency':'CGR',
       'mil_support':'MIL-AM','tc_ospa2_w5':'TC-5','tc_ospa2_w10':'TC-10','local':'Local','confirmed_exist':'原确认消融'}
CONDITIONS={'reliable':'可靠','intermittent':'间歇'}
SCENES={'split_latebirth':'断连后新生','churn_departure':'节点变化与目标离开','split_no_new':'无新增目标'}
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])
def ci(p):return f"{p['mean']:+.5f} [{p['low']:+.5f}, {p['high']:+.5f}]"
rows=[]
for r in real['aggregate']:
    rows.append([CONDITIONS[r['condition']],NAMES[r['arm']]]+[f"{r[k]['mean']:.5f}" for k in ['ospa','miss2','false2']])
pairs=[]
for r in real['paired']:
    if r['reference'] not in ['lineage','qualified_exist','conservative_recency']:continue
    pairs.append([CONDITIONS[r['condition']],NAMES[r['candidate']],NAMES[r['reference']],ci(r['ospa']),f"{r['ospa_wins']}/9"])
synthetic=[]
for r in cases['aggregate']:
    synthetic.append([SCENES[r['scene']],NAMES[r['arm']]]+[f"{r[k]['mean']:.5f}" for k in ['ospa','miss2','false2']]+[f"{r['remote_acquisitions']['mean']:.2f}"])
calrows=[[r['excluded_sequence'],f"{r['a']:.6f}",f"{r['b']:.6f}",
          f"{r['raw']['brier']:.6f}",f"{r['calibrated']['brier']:.6f}"] for r in cal['folds']]
common=[]
for c in CONDITIONS:
    for candidate in ['ceiling_score','ceiling_calibrated']:
        for reference in ['lineage','qualified_exist']:
            group=[r for r in real['common_target'] if r['condition']==c and r['candidate']==candidate and r['reference']==reference]
            support=sum(r['support'] for r in group)
            common.append([CONDITIONS[c],NAMES[candidate],NAMES[reference],support,
                           f"{(sum(r['candidate_sse'] for r in group)/support)**.5:.5f}",
                           f"{(sum(r['reference_sse'] for r in group)/support)**.5:.5f}"])
text='''# 连续直接证据上限：完整开发轮结果

ECR-S/C 在完整九序列、两种链路的平均 OSPA 均优于自身 No-age；ECR-C
在两种链路均胜出 8/9 个序列。但它们使用了原基线未利用的检测分数，
这仍是开发结果，不能单独证明融合规则的独立贡献或预留队列收益。
同信息的标记似然基线实验位于相邻 icra_marked_iteration 目录。

## 固定规则

先按原规则计算 No-age 存在概率 r0 和 ER 存在概率 rER，保持同一空间池
与删失/来源资格。对来源 j，W_jm 是当前局部更新的归一化关联边缘概率，
包含 m=0 的漏检分支；c_j=sum_{m>0} W_jm v_m。没有当前本地直接观测
机会或测量为空时 c_j=0。来源接收不能更新本地直接观测时间。

只在合格、有标签、q_j>b_j+1e-12 且 r_j>=0.5 的来源中取 c=max_j c_j，
空集取零。最终存在概率为 rECR=min(rER,max(r0,c))。

固定空间池下，ER 的标量目标可写成

    F(r) = r log r + (1-r) log(1-r) - r logit(rER) + constant.

F 严格凸；在附加约束 r<=max(r0,c) 下，其唯一解正是上述 rECR。
因此 min(r0,rER)<=rECR<=rER，负时效修正完整保留，c=0 退回 CR，
单来源/相同年龄在该规则下保持原结果。这里 c_j 是经验支持分数，
不是对真实目标存在概率的置信上界；凸优化表达不构成统计安全保证。

三个预注册候选：A 将所有观测分数设为 1；S 使用原始检测分数；C 使用
逐序列交叉拟合的单调 logistic 校准值。未搜索新的跟踪阈值或融合权重。
见 [PROTOCOL.md](PROTOCOL.md) 和 [CASES_PROTOCOL.md](CASES_PROTOCOL.md)。

## 完整真实检测开发队列

9 个序列、1993 帧、2 个节点、2 种模拟链路。位置裁剪、2D 检测输入、
原局部更新、出生、动态模型、空间融合、通信随机数、剪枝与 MAP 输出
均未更改。OSPA 单位 m；miss/false 为 GOSPA 平方分解的 m² 代价。
以下为序列等权均值。

'''+table(['链路','方法','OSPA ↓','漏检代价 ↓','虚假代价 ↓'],rows)+'''

候选减参照的配对 OSPA 差，负数为改善。区间以序列为单位，固定种子
8301、10000 次 bootstrap，未校正多重比较；这些序列已用于方法开发。

'''+table(['链路','候选','参照','差值 [95% 区间]','胜出序列'],pairs)+'''

ECR-S 的均值稍低，但 ECR-C 的序列胜率更高；不能把某一项最优当作
通用胜出。与 CR 比较，两者保留更多发现能力，真实数据上的虚假代价
仍高于 CR。下面另用双方共同匹配到的真值实例核验定位，支持量按比较
变化；它是 pooled RMSE，不是只在各自幸存轨迹上计算的 RMSE。

'''+table(['链路','候选','参照','共同实例数','候选 RMSE m','参照 RMSE m'],common)+'''

## 分数校准

检测分数与原始检测行按坐标、裁剪和顺序逐帧核验。标签由每帧每传感器
与裁剪真值的 12 m 一对一最小代价指派定义；它不是 3D 检测标签。
每个序列只使用另外八个序列拟合的 a,b：sigmoid(a logit(score)+b)，
a>=0，序列平衡 logistic 损失加 0.001*a²。模型、惩罚及数值裁剪均
在跟踪输出前固定。不能把交叉拟合后的本轮队列称为独立方法选择测试。

'''+table(['被排除序列','a','b','原分数 Brier','交叉拟合 Brier'],calrows)+f'''

全部 38038 条检测的逐检测汇总：原始/交叉拟合 Brier 分别为
{cal['raw']['brier']:.6f}/{cal['held_out']['brier']:.6f}；log loss 为
{cal['raw']['log_loss']:.6f}/{cal['held_out']['log_loss']:.6f}。该汇总按检测条数
加权，不能与上表或跟踪的序列宏平均混用。

## 全部模拟机制场景

复用冻结的 60 组输入：三个场景各 20 个种子（2901--2920）。所有测量，
包括杂波，都使用同一个分数 1，所以 S/C 在分数不可用时均退化为 A。
没有使用真值给真实观测和杂波分配不同分数。目标发现数为每回合远端
查询成功数的均值；无新增目标场景没有该类查询。

'''+table(['场景','方法','OSPA ↓','漏检代价 ↓','虚假代价 ↓','远端发现数'],synthetic)+'''

新方法保留三个场景中大部分相对 No-age 的时效收益，但都略差于 ER。
节点变化场景中 ECR 成功 158/160 个远端查询，ER 为 160/160；不能将
“保留发现能力”写成完全无损或不增加漏检。所有不利种子保留。

## 核验与复现

'''+f"真实开发独立重算 {real['audited_node_frames']} 个节点—帧，全部 {real['original_er_bitwise_parity_node_frames']} 个原 ER 节点—帧逐位一致；模拟独立重算 {cases['audited_node_frames']} 个节点—帧。"+'''
哈希复核见 CURRENT_QA.json。包括解析存在概率、同输入反事实、MAP
提取、裁剪、真实检测分数映射、无线投递、输出误差和共同定位支持。

真实检测包每个 Bernoulli 比原 ER 多一个 float64 支持量：216 B/object
加 32 B 包头；模拟包保持原混合高斯编码并增加同一标量。全部模拟包
仍在 16 KiB 内，线缆字节数与原 ER 相同。真实数据中 Bernoulli 数及
分片数可能变化；不声称免通信开销。报告原 ER 字节数使用原始基线，
没有把新增的调试字段算成原方法必需字段。

```sh
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_ceiling_iteration'); checkEvidenceCeiling();"
./tmp/external_baselines/venv/bin/python trials/icra_ceiling_iteration/analyze_ceiling.py --output-dir tmp/ecr_reaudit
./tmp/external_baselines/venv/bin/python trials/icra_ceiling_iteration/analyze_cases.py --output-dir tmp/ecr_reaudit
```

重新跟踪入口为 runEvidenceCeilingReplay(0,8,false) 与
runEvidenceCeilingCases(2901,2920)。先复制保存原输出目录，避免覆盖本轮
原件。prepare/make/freeze 脚本是历史准备和注册过程，不是审计重跑入口。
原核心代码、先前 IR/CR/CGR/JE 实验和论文均未因本轮而修改。
'''
(OUT/'README_CN.md').write_text(text)
print('ECR complete development/cases report written.')
