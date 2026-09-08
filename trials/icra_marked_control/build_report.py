"""Report all extra-control results, including adverse comparisons."""
from pathlib import Path
import json

OUT = Path(__file__).resolve().parent
LABEL = {'marked_conservative': 'M-CR', 'marked_ceiling_score': 'M-ECR-S',
         'marked_lineage': 'M-No-age', 'marked_er': 'M-ER'}


def main():
    result = json.loads((OUT / 'summary_control.json').read_text())
    frozen = json.loads((OUT / 'CONTROL_FREEZE.json').read_text())
    assert result['new_control_node_frames'] == 30376 and result['rescored_baseline_node_frames'] == 91128
    lines = ['# 共享分数观测模型的保守截断消融', '',
             'M-CR 令 r=min(r0,rER)，完全去掉正向时效增益；M-ECR-S 允许当前直接',
             '关联支持授权有限的正向增益。两者共享已经拟合的分数似然更新。该比较',
             '用于分辨保守截断的作用与恢复正向增益的增量，没有增加或调节新参数。', '',
             f"这项消融在 {len(frozen['completed_primary_sequences_at_registration'])} 条原主实验序列已经完整结束、但全部主实验统计尚未生成时补充。",
             '它是后补的次要比较，不能被称为最初预注册的主对照；原主方法和原定比较',
             '保持不变。全部九个开发序列和全部 25 个预留融合序列均完成两种链路回放。', '',
             '## 完整结果', '',
             'OSPA 为序列等权均值，单位 m；漏检和虚假项为 GOSPA 平方代价 m²。', '',
             '| 队列 | 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- | --- |']
    for cohort, cohort_label in [('development', '开发 9'), ('holdout', '预留融合 25')]:
        content = result['cohorts'][cohort]
        rows = {(r['condition'], r['arm']): r for r in content['aggregate']}
        for condition, condition_label in [('reliable', '可靠'), ('intermittent', '间歇')]:
            for arm in ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_score']:
                r = rows[condition, arm]
                lines.append(f"| {cohort_label} | {condition_label} | {LABEL[arm]} | {r['ospa']['mean']:.6f} | {r['miss2']['mean']:.6f} | {r['false2']['mean']:.6f} |")
    lines += ['', '## 配对差与代价分解', '',
              '差值为候选减参照；负数有利于候选。括号为 10000 次序列 bootstrap 的',
              '95% 百分位区间，固定随机数 8301。这些是描述性区间，没有多重比较调整。', '',
              '| 队列 | 链路 | 候选 − 参照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 | OSPA 胜出 |',
              '| --- | --- | --- | --- | --- | --- | --- |']
    for cohort, cohort_label in [('development', '开发 9'), ('holdout', '预留融合 25')]:
        content = result['cohorts'][cohort]
        for r in content['paired']:
            v = r['ospa']
            condition_label = '可靠' if r['condition'] == 'reliable' else '间歇'
            lines.append(f"| {cohort_label} | {condition_label} | {LABEL[r['candidate']]} − {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} | {r['ospa_wins']}/{content['sequences']} |")
    lines += ['', '各漏检、虚假代价差的区间也保留在 summary_control.json，不能只凭 OSPA',
              '均值声称正向恢复在所有情形都优于保守截断。', '',
              '## 共同真值支持上的定位', '',
              '| 队列 | 链路 | 与 M-CR 比较的方法 | 共同实例 | 方法 RMSE m | M-CR RMSE m |',
              '| --- | --- | --- | --- | --- | --- |']
    for cohort, cohort_label in [('development', '开发 9'), ('holdout', '预留融合 25')]:
        for r in result['cohorts'][cohort]['common_aggregate']:
            condition_label = '可靠' if r['condition'] == 'reliable' else '间歇'
            lines.append(f"| {cohort_label} | {condition_label} | {LABEL[r['candidate']]} | {r['support']} | {r['candidate_rmse']:.6f} | {r['reference_rmse']:.6f} |")
    lines += ['', '## 预留融合队列的通信量', '',
              '下表是每序列平均值。M-CR、M-No-age、M-ER 原生包为 208 B/Bernoulli +',
              '32 B 包头，M-ECR-S 额外携带一个 8 B 关联支持标量。分片大小与控制字节',
              '保持原协议。开发阶段旧 M-No-age/M-ER 记录曾携带未使用的额外标量，',
              '因此这里仅比较全部采用原生编码的 25 序列通信量。', '',
              '| 链路 | 方法 | 原始 MiB/序列 | 投递原始 MiB/序列 | 分片及控制 MiB/序列 |',
              '| --- | --- | --- | --- | --- |']
    for r in result['cohorts']['holdout']['aggregate']:
        condition_label = '可靠' if r['condition'] == 'reliable' else '间歇'
        values = [r[k]['mean'] / 2**20 for k in ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']]
        lines.append(f"| {condition_label} | {LABEL[r['arm']]} | {values[0]:.6f} | {values[1]:.6f} | {values[2]:.6f} |")
    lines += ['', '## 每个序列的正向恢复增量', '',
              '以下是 M-ECR-S 减 M-CR 的 OSPA；按原始序列顺序展示，包含全部不利结果。', '',
              '| 队列 | 序列 | 帧数 | 可靠 ΔOSPA | 间歇 ΔOSPA |',
              '| --- | --- | --- | --- | --- |']
    for cohort, cohort_label in [('development', '开发'), ('holdout', '预留融合')]:
        lookup = {(r['sequence'], r['condition'], r['arm']): r for r in result['cohorts'][cohort]['runs']}
        for unit in frozen['units']:
            if unit['cohort'] != cohort:
                continue
            name = f"{unit['sequence']:04d}"
            frames = lookup[name, 'reliable', 'marked_conservative']['frames']
            delta = [lookup[name, c, 'marked_ceiling_score']['ospa'] - lookup[name, c, 'marked_conservative']['ospa']
                     for c in ['reliable', 'intermittent']]
            lines.append(f"| {cohort_label} | {name} | {frames} | {delta[0]:+.6f} | {delta[1]:+.6f} |")
    lines += ['', '## 范围与复现', '',
              '预检独立重算 1176 个节点—帧，其中 588 个 ER 节点—帧的全部输出与',
              '诊断字段（耗时除外）与原端口完全一致。完整新臂独立重算 30376 个',
              '节点—帧，并对 91128 个原有基线节点—帧重新评分以建立共同真值支持。',
              '原有基线轨迹没有重新回放；原始文件摘要、输入、无线投递、包长和',
              'M-CR 的 min(r0,rER) 公式均逐项核对。', '',
              '该报告不是独立检测器评测：九个发布序列已用于本研究的方法开发；',
              '额外 25 个序列虽然未用于此前的融合选择，其所属训练划分已用于训练',
              '公开检测器。驾驶路线相关性仍可能存在。三维官方指标和真实通信没有',
              '在这里复现。论文应保留这些实质限制，但不把运行记录写成正文叙述。', '',
              '没有分数时的机制对照直接复用已有全部 60 个 CR 与 ECR-A 配对案例，',
              '见相邻 icra_method_iteration/README_CN.md 和 icra_ceiling_iteration/README_CN.md。',
              '这些统一单位标记案例检验正向恢复与发现目标的权衡，不能证明分数校准的效果。', '',
              '```sh',
              './tmp/external_baselines/venv/bin/python trials/icra_marked_control/analyze_full.py',
              './tmp/external_baselines/venv/bin/python trials/icra_marked_control/build_report.py',
              '```', '',
              '源文件和 CONTROL_FREEZE.json 不可覆盖。run_control.py 对已有完整结果',
              '有覆盖保护；需要重新回放时使用新的输出目录和明确的执行记录。', '']
    (OUT / 'README_CN.md').write_text('\n'.join(lines))
    print('Complete additional-control report written for 9 development and 25 reserved sequences.')


if __name__ == '__main__':
    main()
