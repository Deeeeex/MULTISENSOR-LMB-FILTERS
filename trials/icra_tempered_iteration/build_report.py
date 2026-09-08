"""Write the complete tempered-stage result without promoting an ablation."""
from pathlib import Path
import argparse
import json

OUT = Path(__file__).resolve().parent
LABEL = {'marked_lineage': 'M-No-age', 'marked_er': 'M-ER',
         'marked_conservative': 'M-CR', 'marked_ceiling_calibrated': 'M-ECR-C',
         'marked_ceiling_score': 'M-ECR-S', 'marked_tempered_calibrated': 'T-C（主）',
         'marked_tempered_score': 'T-S', 'marked_tempered_association': 'T-A'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    args = parser.parse_args()
    result = json.loads((OUT / f'summary_{args.cohort}.json').read_text())
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    primary, n = result['primary'], result['sequences']
    assert primary == 'marked_tempered_calibrated'
    assert n == (9 if args.cohort == 'development' else 25)
    title = '九个开发序列' if args.cohort == 'development' else '25 个已见迁移序列'
    gate = result['continuation_gate_passed']
    if args.cohort == 'development':
        decision = ('主方法通过预先规定的继续条件，可以执行完整已见迁移队列。' if gate else
                    '主方法未通过预先规定的继续条件，停止该候选的扩大回放；不改用较好的次要臂。')
    else:
        decision = ('四项主要配对区间均低于零，但方法设计已见过这些数据，仍是开发证据。' if result['all_four_primary_intervals_below_zero'] else
                    '四项主要配对区间未全部低于零，尚未得到稳定的同信息主结果。')
    lines = [f'# 证据调制正时效修正：{title}', '', decision, '',
             '这个版本在完整 ECR 与 CR 比较之后提出。全部真实序列此前均已用于方法',
             '诊断；本轮不能再把 25 个序列称为未见测试，也不把主方法换成结果最好的消融。', '',
             '## 固定改动', '',
             '以原始无时效对数几率 z0 为基础，令 Δ 为原 ER 的时效对数几率修正，',
             'c 为当前直接关联支持，则 zT=z0+min(Δ,0)+c·max(Δ,0)。',
             '负向修正完全保持，正向修正在 CR 和 ER 之间随当前证据强度变化。',
             'c=0 为 CR，c=1 为 ER，单源或等龄输入恢复无时效结果。', '',
             '主方法 T-C 使用此前已拟合的校准分数；T-S 用原始分数，T-A 用单位分数。',
             '三者与所有 M 参照共享相同分数似然更新。没有重新拟合或搜索年龄、',
             '出生、关联、裁剪、提取或通信参数。逐点界限不是递归误差改善保证。', '',
             '## 全部方法与误差分解', '',
             '各序列等权，OSPA 单位 m，GOSPA 漏检/虚假平方代价单位 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    aggregate = {(r['condition'], r['arm']): r for r in result['aggregate']}
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    menu = frozen['primary_references'] + ['marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score'] + frozen['arms']
    for condition, name in conditions:
        for arm in menu:
            r = aggregate[condition, arm]
            lines.append(f"| {name} | {LABEL[arm]} | {r['ospa']['mean']:.6f} | {r['miss2']['mean']:.6f} | {r['false2']['mean']:.6f} |")
    lines += ['', '## 主要配对结果', '',
              '差值为 T-C 减参照，负数有利于 T-C。区间是 10000 次序列 bootstrap 的',
              '95% 百分位区间，固定随机数 8301；属于开发阶段描述性区间，未调整多重比较。', '',
              '| 链路 | 参照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 | OSPA 胜出 |',
              '| --- | --- | --- | --- | --- | --- |']
    for r in result['paired']:
        if r['candidate'] != primary:
            continue
        v = r['ospa']
        lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} | {r['ospa_wins']}/{n} |")
    lines += ['', '## 每个序列', '',
              '每个差值依次为 T-C 减 M-No-age / M-ER / M-CR，包含全部不利结果。', '',
              '| 序列 | 帧数 | 可靠 OSPA | 可靠 Δ | 间歇 OSPA | 间歇 Δ |',
              '| --- | --- | --- | --- | --- | --- |']
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in result['runs']}
    for unit in frozen['units']:
        if unit['cohort'] != args.cohort:
            continue
        name = f"{unit['sequence']:04d}"
        columns = [name, str(lookup[name, 'reliable', primary]['frames'])]
        for condition, _ in conditions:
            v = lookup[name, condition, primary]['ospa']
            differences = [v - lookup[name, condition, ref]['ospa'] for ref in ['marked_lineage', 'marked_er', 'marked_conservative']]
            columns += [f'{v:.6f}', ' / '.join(f'{d:+.6f}' for d in differences)]
        lines.append('| ' + ' | '.join(columns) + ' |')
    lines += ['', '## 共同真值支持与当前作用', '',
              '定位只比较双方同时匹配到的同一真值实例；该支持量随比较变化。', '',
              '| 链路 | 参照 | 共同实例 | T-C RMSE m | 参照 RMSE m |',
              '| --- | --- | --- | --- | --- |']
    for r in result['common_aggregate']:
        if r['candidate'] == primary:
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {r['support']} | {r['candidate_rmse']:.6f} | {r['reference_rmse']:.6f} |")
    lines += ['', '下面保持候选实际递推输入及空间密度，替换存在规则后重新提取并评分。',
              '只在发生融合的节点—帧上先按序列平均，不等同于重新运行其他方法的历史。', '',
              '| 链路 | 同输入规则 | OSPA | 漏检代价 | 虚假代价 |',
              '| --- | --- | --- | --- | --- |']
    for condition, name in conditions:
        group = [r for r in result['diagnostics'] if r['condition'] == condition and r['arm'] == primary]
        assert len(group) == n
        for rule in ['no_age', 'ER', 'candidate']:
            values = [sum(r['same_input'][rule][key] for r in group) / n for key in ['ospa', 'miss2', 'false2']]
            lines.append(f"| {name} | {rule} | {values[0]:.6f} | {values[1]:.6f} | {values[2]:.6f} |")
    lines += ['', '## 核验与范围', '',
              f"全部 {result['frames']} 帧、{n} 序列及 {6*n} 个新增文件均完成真实进程退出检查；",
              f"Python 独立重算 {result['new_node_frames']} 个新增节点—帧，并重新评分",
              f"{result['rescored_baseline_node_frames']} 个原有参照节点—帧以计算相同真值支持。",
              '预检另重算 2352 个节点—帧，其中 588 个 ER 节点—帧的全部输出与',
              '诊断字段（耗时除外）精确等于原运行器。全部输入、无线随机数、包长、',
              '解析存在公式及当前输入上的候选提取结果均核对。旧参照没有重新跟踪。', '',
              'T 包为 216 B/Bernoulli + 32 B 包头；原生密度参照为 208 B/Bernoulli。',
              '旧开发 No-age/ER 记录曾带未使用的支持标量，所以不能把其记录字节当作',
              '原生 208 B 协议。完整原始/投递/分片字节及所有消融的区间均保留在摘要。', '',
              '这些检查属于同一工作流程中的独立实现复算，不是第三方复现。公开检测器',
              '训练划分、路线相关性、二维相对坐标模型及仿真通信的限制继续成立。',
              '达到继续条件只允许扩大开发实验，不能据此宣布论文已经完成。', '']
    (OUT / f'RESULTS_{args.cohort}_CN.md').write_text('\n'.join(lines))
    print('Complete tempered result written:', title, 'continuation:', gate)


if __name__ == '__main__':
    main()
