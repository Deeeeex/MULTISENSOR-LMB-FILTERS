"""Report the complete marked-innovation experiment and its earlier failure control."""
from pathlib import Path
import argparse
import json

OUT = Path(__file__).resolve().parent
LABEL = {'marked_lineage': 'M-No-age', 'marked_er': 'M-ER', 'marked_conservative': 'M-CR',
         'marked_ceiling_calibrated': 'M-ECR-C', 'marked_ceiling_score': 'M-ECR-S',
         'marked_joint_evidence': 'M-JE（主）', 'marked_joint_evidence_recency': 'M-JE-R',
         'joint_evidence': 'JE（不使用分数）', 'joint_evidence_recency': 'JE-R（不使用分数）'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    args = parser.parse_args()
    d = json.loads((OUT / f'summary_{args.cohort}.json').read_text())
    f = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    n, primary = d['sequences'], d['primary']
    assert n == (9 if args.cohort == 'development' else 25) and primary == 'marked_joint_evidence'
    label = '九个开发序列' if args.cohort == 'development' else '25 个已见迁移序列'
    if args.cohort == 'development':
        decision = ('主方法通过事先规定的继续条件，可执行完整的已见迁移队列。' if d['continuation_gate_passed'] else
                    '主方法未通过事先规定的继续条件，不扩大这条分支；完整保留两种规则与全部序列。')
    else:
        decision = ('四项主要配对区间均低于零，但这些序列已参与诊断，仍属开发证据。' if d['all_four_primary_intervals_below_zero'] else
                    '四项主要配对区间未全部低于零，尚无稳定的同信息主结果。')
    lines = [f'# 共享分数观测模型的本轮增量融合：{label}', '', decision, '',
             '此前不使用分数的 JE/JE-R 已在全部九个开发序列上失败。本轮保留原增量',
             '规则，接入已经核验的共享分数似然更新，检验两者组合；不能把分数后端',
             '带来的改善全部归因于新的融合增益。全部真实序列此前均已用于诊断。', '',
             '## 固定方法', '',
             '逐来源在本轮本地更新后重新计算 δ=logit(r后验)−logit(r预测)。只有至少两个',
             '合格的正权重来源都表示该标签时，才进行本轮增量合并。主方法使用',
             'z=sum b·(logit(r)−δ)+sum δ+logη；次要 M-JE-R 在继承项中用时效权重 q。',
             '缺标签删失或单源时，分别恢复已有无时效或 ER 规则。空间密度仍由同一',
             '组输入及原空间权重计算。每目标增加一个 8 B 的本轮增量字段。', '',
             '单一 Bernoulli、相同先验及条件独立观测下的恒等式不代表近似 LMB 更新',
             '或相关检测器满足精确 Bayes 条件。当前误检相关时，合并增量仍可能过度',
             '增加确信。分开池化先验与似然是已有思想，本报告不把它声称为新原理。', '',
             '## 全部同信息结果', '',
             '序列等权 OSPA 单位 m；GOSPA 漏检与虚假平方代价单位 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    ag = {(r['condition'], r['arm']): r for r in d['aggregate']}
    for c, name in conditions:
        for arm in f['primary_references'] + f['additional_references'] + f['arms']:
            r = ag[c, arm]
            lines.append(f"| {name} | {LABEL[arm]} | {r['ospa']['mean']:.6f} | {r['miss2']['mean']:.6f} | {r['false2']['mean']:.6f} |")
    lines += ['', '## 主方法的配对差', '',
              '差值为 M-JE 减参照；95% 区间为 10000 次序列 bootstrap 百分位区间，',
              '固定随机数 8301。区间仅描述这些已见序列，没有多重比较调整。', '',
              '| 链路 | 参照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 | OSPA 胜出 |',
              '| --- | --- | --- | --- | --- | --- |']
    for r in d['paired']:
        if r['candidate'] == primary:
            v = r['ospa']
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} | {r['ospa_wins']}/{n} |")
    if d['backend_interaction']:
        lines += ['', '## 对照此前不使用分数的同一规则', '',
                  '这里复用完整旧 JE/JE-R 轨迹。两者差异属于共享分数观测模型与该规则',
                  '的组合效应，不能代替上面的同信息比较。', '',
                  '| 链路 | 标记版本 − 旧规则 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 |',
                  '| --- | --- | --- | --- | --- |']
        for r in d['backend_interaction']:
            v = r['ospa']
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['candidate']]} − {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} |")
    lines += ['', '## 每个序列', '',
              '差值顺序为 M-JE 减 M-No-age / M-ER / M-CR。', '',
              '| 序列 | 帧数 | 可靠 OSPA | 可靠 Δ | 间歇 OSPA | 间歇 Δ |',
              '| --- | --- | --- | --- | --- | --- |']
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in d['runs']}
    for unit in f['units']:
        if unit['cohort'] != args.cohort:
            continue
        name = f"{unit['sequence']:04d}"
        columns = [name, str(lookup[name, 'reliable', primary]['frames'])]
        for c, _ in conditions:
            value = lookup[name, c, primary]['ospa']
            delta = [value - lookup[name, c, a]['ospa'] for a in ['marked_lineage', 'marked_er', 'marked_conservative']]
            columns += [f'{value:.6f}', ' / '.join(f'{v:+.6f}' for v in delta)]
        lines.append('| ' + ' | '.join(columns) + ' |')
    lines += ['', '## 共同真值支持上的定位', '',
              '| 链路 | 参照 | 共同实例 | M-JE RMSE m | 参照 RMSE m |',
              '| --- | --- | --- | --- | --- |']
    for r in d['common_aggregate']:
        if r['candidate'] == primary:
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {r['support']} | {r['candidate_rmse']:.6f} | {r['reference_rmse']:.6f} |")
    lines += ['', '## 复算与边界', '',
              f"全部 {4*n} 个新增结果来自成功结束的完整序列进程，独立重算 {d['new_node_frames']} 个新增节点—帧；",
              f"另重新评分 {d['rescored_baseline_node_frames']} 个共享后端参照节点—帧和 {d['rescored_unmarked_backend_node_frames']} 个旧 JE 节点—帧。",
              '所有局部更新的预测/后验存在概率、当前增量、标签、来源、时间与感知',
              '机会逐行核对；没有当前机会时，增量必须为零。融合权重、解析存在值、',
              '完整无线投递和包字节也独立检查。固定候选输入下的 No-age / ER 反事实',
              '与当次候选提取保留在摘要；这些反事实不重新运行其他规则的历史。', '',
              '预检核对 1764 个节点—帧，其中 588 个 ER 节点—帧的轨迹、指标、包字段',
              '与原端口逐项相同，公共标量诊断在浮点精度内一致。增量记录与旧支持',
              '记录语义不同，按各自公式核对。所有失败候选、输入和旧输出均未改写。', '',
              '所有新增方法的原生包为 216 B/Bernoulli + 32 B 包头。旧开发 No-age/ER',
              '文件曾带未使用的支持标量，不将其当成原生 208 B 编码的通信基准。',
              '源文件摘要、全部分量与通信区间、每个序列及标量作用记录见完整摘要。', '',
              '检测器训练划分、路线相关性、近似相对坐标模型与模拟通信的限制均继续',
              '成立。这里的独立实现复算属于同一工作流程，不是第三方复现或新测试集。', '']
    (OUT / f'RESULTS_{args.cohort}_CN.md').write_text('\n'.join(lines))
    print('Complete marked-joint report written:', label, 'continuation:', d['continuation_gate_passed'])


if __name__ == '__main__':
    main()
