"""Report the complete fixed selective experiment, including all ablations."""
from pathlib import Path
import argparse
import json
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent / 'icra_marked_control'))
from analyze_control import interval

LABEL = {'marked_lineage': 'M-No-age', 'marked_er': 'M-ER', 'marked_conservative': 'M-CR',
         'marked_ceiling_calibrated': 'M-ECR-C', 'marked_ceiling_score': 'M-ECR-S',
         'marked_selective': 'M-SI（主）', 'marked_selective_no_history': '去除历史保守项',
         'marked_selective_no_mark': '去除分数约束', 'marked_selective_signed': '放行正负增量',
         'marked_joint_evidence': '前轮 M-JE', 'marked_joint_evidence_recency': '前轮 M-JE-R'}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = p.parse_args().cohort
    d = json.loads((OUT / f'summary_{cohort}.json').read_text())
    f = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    primary, n = d['primary'], d['sequences']
    assert primary == 'marked_selective' and n == (9 if cohort == 'development' else 25)
    names = [f"{u['sequence']:04d}" for u in f['units'] if u['cohort'] == cohort]
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in d['runs']}
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    samples = np.random.default_rng(8301).integers(0, n, (10000, n))
    ablations = []
    for c, _ in conditions:
        for ref in f['arms'][1:]:
            ablations.append(dict(condition=c, candidate=primary, reference=ref,
                **{key: interval([lookup[s, c, primary][key]-lookup[s, c, ref][key] for s in names], samples)
                   for key in ['ospa', 'miss2', 'false2']}))
    (OUT / f'ablation_comparisons_{cohort}.json').write_text(json.dumps(ablations, indent=2) + '\n')
    if cohort == 'development':
        decision = ('主方法通过规定的继续条件，允许执行完整 25 序列的已见迁移实验。' if d['continuation_gate_passed'] else
                    '主方法未通过规定的继续条件，停止扩大这个候选；保留所有消融和全部序列。')
    else:
        decision = ('四项主要配对区间均低于零，但这仍是已见数据上的开发证据。' if d['all_four_primary_intervals_below_zero'] else
                    '四项主要配对区间未全部低于零，尚无稳定的同信息主结果。')
    lines = [f'# 选择性本帧增量：{n} 个' + ('开发序列' if cohort == 'development' else '已见迁移序列'), '', decision, '',
             '完整联合增量曾因虚假目标代价增加而失败。这一轮保留保守继承项，',
             '只允许当前关联且分数更支持目标的正向局部增量增加存在概率。',
             '所有真实序列此前均已用于方法诊断；消融不能因结果较好而替换主方法。', '',
             '## 固定方法', '',
             '局部增量 δ=logit(r后验)−logit(r预测)，门控 g=sum W·max(0,(L−1)/(L+1))，',
             '其中 L 为之前拟合的分数似然比，漏检分支贡献零，非本帧观测时 g 清零。',
             '时效修正为负时使用 q，否则使用 b，得到保守继承权重 β；对至少两个',
             '合格且都表示该标签的来源，加入 sum(1−β)·g·max(δ,0)。数值相等的',
             '对数几率采用事先规定的 1e−12 容差与 β=b。融合后清除 g，下一次本地',
             '更新重新计算，防止直接把融合结果标为新的本帧证据。', '',
             '三项消融分别恒用 β=b、仅以检测关联质量设 g、放行正负 δ。全部方法',
             '共享相同分数似然的局部更新、动态、出生、关联、标识匹配、空间融合',
             '及输出提取。门控是有界启发式，不是新的似然或误差改善保证。', '',
             '## 全部同信息方法', '',
             '各序列等权；OSPA 单位 m，漏检/虚假 GOSPA 平方代价单位 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    ag = {(r['condition'], r['arm']): r for r in d['aggregate']}
    for c, title in conditions:
        for arm in f['primary_references']+f['additional_references']+f['arms']:
            r = ag[c, arm]
            lines.append(f"| {title} | {LABEL[arm]} | {r['ospa']['mean']:.6f} | {r['miss2']['mean']:.6f} | {r['false2']['mean']:.6f} |")
    lines += ['', '## 主方法配对比较', '',
              '差值为主方法减参照，负数有利于主方法。95% 区间为 10000 次序列重采样',
              '的描述性百分位区间，未调整多重比较，也不是未见验证。', '',
              '| 链路 | 参照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 | 胜出序列 |',
              '| --- | --- | --- | --- | --- | --- |']
    for r in d['paired']:
        if r['candidate'] != primary:
            continue
        v = r['ospa']
        lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} | {r['ospa_wins']}/{n} |")
    lines += ['', '## 全部组件消融', '',
              '仍为主方法减消融。若区间包含零，则该组件的必要性尚未证实。', '',
              '| 链路 | 移除或替换组件 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 |',
              '| --- | --- | --- | --- | --- |']
    for r in ablations:
        v = r['ospa']
        lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} |")
    if d['preceding_marked_joint_comparison']:
        lines += ['', '## 与完整联合增量的开发比较', '',
                  '前轮 M-JE/M-JE-R 使用已保存并独立复算的全部九序列摘要；本轮未重跑。', '',
                  '| 链路 | 前轮规则 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 |',
                  '| --- | --- | --- | --- | --- |']
        for r in d['preceding_marked_joint_comparison']:
            v = r['ospa']
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} |")
    lines += ['', '## 所有序列', '',
              '差值依次为主方法减 M-No-age / M-ER / M-CR / M-ECR-S。', '',
              '| 序列 | 帧数 | 可靠 OSPA | 可靠 Δ | 间歇 OSPA | 间歇 Δ |',
              '| --- | --- | --- | --- | --- | --- |']
    for name in names:
        row = [name, str(lookup[name, 'reliable', primary]['frames'])]
        for c, _ in conditions:
            value = lookup[name, c, primary]['ospa']
            row += [f'{value:.6f}', ' / '.join(f"{value-lookup[name, c, ref]['ospa']:+.6f}" for ref in
                    ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_score'])]
        lines.append('| '+' | '.join(row)+' |')
    lines += ['', '## 共同支持与固定输入诊断', '',
              '定位仅比较双方共同匹配到的同一真值实例；支持量随参照改变。', '',
              '| 链路 | 参照 | 共同实例 | 主方法 RMSE m | 参照 RMSE m |',
              '| --- | --- | --- | --- | --- |']
    for r in d['common_aggregate']:
        if r['candidate'] == primary:
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {r['support']} | {r['candidate_rmse']:.6f} | {r['reference_rmse']:.6f} |")
    lines += ['', '下表在主方法已经产生的输入与空间密度上更换存在规则、重新提取。',
              '仅统计实际融合帧并按序列平均，不能解释为另一个方法的完整递推结果。', '',
              '| 链路 | 同输入规则 | OSPA | 漏检代价 | 虚假代价 |',
              '| --- | --- | --- | --- | --- |']
    for c, title in conditions:
        group = [r for r in d['diagnostics'] if r['condition'] == c and r['arm'] == primary]
        assert len(group) == n
        for rule in ['no_age', 'ER', 'CR', 'candidate']:
            values = [np.mean([r['same_input'][rule][key] for r in group]) for key in ['ospa', 'miss2', 'false2']]
            lines.append(f'| {title} | {rule} | {values[0]:.6f} | {values[1]:.6f} | {values[2]:.6f} |')
    count_local = sum(r['local_update_records'] for r in d['diagnostics'])
    count_join = sum(r['joined_source_records'] for r in d['diagnostics'])
    lines += ['', '## 验证与适用范围', '',
              f"本阶段 {n} 序列、{d['frames']} 帧、{8*n} 个新文件均有成功的实际 MATLAB",
              f"退出、完成标记及完整文件检查。Python 复算 {d['new_node_frames']} 个新增",
              f"节点—帧，以及 {d['rescored_baseline_node_frames']} 个共同参照节点—帧。",
              f"逐项重算 {count_local} 条本地增量，并把 {count_join} 个融合输入按来源、",
              '时间和对齐前的标签连接到本地记录，核对收到的存在值、增量与门控。',
              '预检额外重算 2940 个节点—帧；其中 588 个 CR 节点—帧的轨迹、指标、',
              '包长和原有 26 项诊断与原版精确相同（耗时除外）。', '',
              '新包为 224 B/Bernoulli + 32 B 包头；CR 为原生 208 B/Bernoulli。旧开发',
              'No-age/ER 曾记录未使用的额外标量，不能直接把该记录当成原生协议。',
              '完整原始、投递和分片字节，以及所有消融与参照的区间均保留在 JSON。', '',
              '检测器训练划分、相关路线、二维相对坐标、近似关联与仿真链路限制继续',
              '成立。检查是同一工作流程中独立实现的复算，尚不是第三方复现。',
              '通过继续条件只允许扩大开发实验；论文尚未据本轮结果修改。', '']
    (OUT / f'RESULTS_{cohort}_CN.md').write_text('\n'.join(lines))
    print('Selective complete result written:', cohort, 'continuation:', d['continuation_gate_passed'])


if __name__ == '__main__':
    main()
