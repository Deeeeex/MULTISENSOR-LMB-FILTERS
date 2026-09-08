"""Render complete-cohort comparisons without selecting or changing methods."""
from pathlib import Path
import json

OUT = Path(__file__).resolve().parent
NAMES = {
    'local': 'Local', 'lineage': 'No-age', 'qualified_exist': 'ER', 'mil_support': 'MIL-AM',
    'tc_ospa2_w5': 'TC-5', 'tc_ospa2_w10': 'TC-10', 'ceiling_calibrated': 'ECR-C',
    'marked_local': 'M-Local', 'marked_lineage': 'M-No-age', 'marked_er': 'M-ER',
    'marked_mil_support': 'M-MIL-AM', 'marked_tc_ospa2_w5': 'M-TC-5', 'marked_tc_ospa2_w10': 'M-TC-10',
    'marked_ceiling_association': 'M-ECR-A', 'marked_ceiling_score': 'M-ECR-S',
    'marked_ceiling_calibrated': 'M-ECR-C',
}


def main():
    result = json.loads((OUT / 'summary_holdout.json').read_text())
    freeze = json.loads((OUT / 'METHOD_FREEZE.json').read_text())
    input_shift = json.loads((OUT / 'input_shift.json').read_text())
    assert input_shift['no_parameter_fitting'] and input_shift['no_tracking_outputs_read']
    assert result['sequences'] == 25 and result['audited_node_frames'] == 358464
    primary = freeze['primary']
    aggregate = {(r['condition'], r['arm']): r for r in result['aggregate']}
    paired = {(r['condition'], r['candidate'], r['reference']): r for r in result['paired']}
    runs = {(r['sequence'], r['condition'], r['arm']): r for r in result['runs']}
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    gate = result['registered_primary_intervals_all_below_zero']
    lines = ['# 全部 25 个预留融合序列：独立核验结果', '',
             ('预先固定的 M-ECR-S 在两种链路下，相对同信息 M-No-age 和 M-ER 的四个配对' if gate else
              '预先固定的 M-ECR-S 尚未在两种链路下同时通过相对同信息 M-No-age 和 M-ER 的四个配对'),
             ('OSPA 区间均低于零。下面同时列出代价分解、共同定位支持、字节成本和每个序列。' if gate else
              'OSPA 区间要求；不能仅凭较低均值宣布得到稳定的主实验收益。全部结果仍按登记输出。'), '',
             '这里预留的是融合方法选择结果：公开检测器在训练划分训练过，驾驶路线也可能相关。',
             '本结果不能称为独立检测器测试、官方三维榜单或真实无线通信实验。', '',
             '## 固定范围与方法', '',
             '开发阶段使用作者发布代码 val 目录中的全部九个序列；作者把这些序列用于',
             '测试，本研究已将其用于方法开发和校准。目录名与原数据集角色的对应见',
             'DATA_SPLIT_NOTE.md。此前已见训练序列为 0000、0005、0010、',
             '0015、0020、0025、0030；其中 0000 是已知的跨划分真值重复控制。此次使用其余',
             '全部 25 个训练序列，共 5601 帧。没有按跟踪结果选取序列、帧窗或链路条件。',
             '主方法、两项内部参照、外部适配参照和完整 16 臂比较，在首次新队列回放前登记。', '',
             'M 表示各方法共享同一检测分数似然更新。校准系数及正例比例只从九个开发序列',
             '拟合并冻结；预留序列的真值未参与拟合、出生、关联或跟踪。原始分数、校准概率',
             '及似然比与检测行一起逐帧核对。详见相邻 icra_marked_iteration/README_CN.md。', '',
             'ECR 的正时效增益受当前关联支持限制：c_j=sum_m W_jm v_m，漏检分支取零；',
             '无本地直接观测机会或无测量时 c_j=0。从获得正向加权的合格正存在来源中取',
             'c=max_j c_j，并令 r=min(rER,max(r0,c))。A/S/C 分别用单位分数、原分数和校准概率。',
             'M-ECR-S 是按全部开发结果选择的固定主方法。c 不是统计置信上界；负时效修正',
             '仍保留，固定输入下的空间融合仍使用原始空间权重。', '',
             '## 同信息主要比较', '',
             '下表为各序列等权均值。OSPA 单位 m；漏检和虚假项是 GOSPA 平方代价 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    primary_menu = ['marked_lineage', 'marked_er', 'marked_ceiling_association', primary,
                    'marked_ceiling_calibrated', 'marked_mil_support', 'marked_tc_ospa2_w5', 'marked_tc_ospa2_w10', 'marked_local']
    for condition, label in conditions:
        for arm in primary_menu:
            row = aggregate[condition, arm]
            lines.append(f"| {label} | {NAMES[arm]} | {row['ospa']['mean']:.6f} | {row['miss2']['mean']:.5f} | {row['false2']['mean']:.5f} |")
    lines += ['', 'M-ECR-S 减参照的配对差，负数为改善。10000 次序列 bootstrap，固定种子 8301，',
              '95% 百分位区间，未做多重比较调整；相关路线使其仍属于描述性区间。', '',
              '| 链路 | 参照 | OSPA 差 [95% 区间] | 相对改善 | 胜出序列 |',
              '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        for arm in freeze['primary_references'] + freeze['external_references']:
            row = paired[condition, primary, arm]
            v = row['ospa']
            gain = -100 * v['mean'] / aggregate[condition, arm]['ospa']['mean']
            lines.append(f"| {label} | {NAMES[arm]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {gain:+.2f}% | {row['ospa_wins']}/25 |")
    lines += ['', '## 固定候选输入时的即时作用', '',
              '以下保持 M-ECR-S 实际递推产生的同一批局部输入、标签和空间融合密度，',
              '仅分别代入 r0、rER、rECR，重新执行同一提取与评分。统计范围为发生融合',
              '的节点—帧，先在序列内平均，再对全部 25 个序列等权平均。',
              '这项解析反事实不重新运行 No-age 或 ER 的历史，不等同于完整递推消融，',
              '也不能把差值作为一般因果效应。它用于检查当前约束的直接作用。', '',
              '| 链路 | 同输入存在规则 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
              '| --- | --- | --- | --- | --- |']
    cap_diagnostics = {}
    for condition, label in conditions:
        group = [r for r in result['diagnostics'] if r['condition'] == condition and r['arm'] == primary]
        assert len(group) == 25 and all('same_input' in r for r in group)
        cap_diagnostics[condition] = {key: sum(r[key] for r in group) for key in
                                      ['labels', 'positive_age_labels', 'support_above_noage',
                                       'constrained_below_er', 'preserved_above_cr']}
        for rule, name in [('no_age', 'r0'), ('ER', 'rER'), ('candidate', 'rECR')]:
            values = [sum(r['same_input'][rule][key] for r in group) / len(group)
                      for key in ['ospa', 'miss2', 'false2']]
            lines.append(f"| {label} | {name} | {values[0]:.6f} | {values[1]:.5f} | {values[2]:.5f} |")
    lines += ['', '逐标签诊断计数如下。它们是递推中的重复标签事件，不是独立样本；',
              '部分计数可以重叠。ER 正向加权事件受上限约束时仍保留全部负向修正。', '',
              '| 链路 | 融合标签事件 | 正时效事件 | 上限实际截断 ER | 比保守控制保留更多存在概率 |',
              '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        d = cap_diagnostics[condition]
        lines.append(f"| {label} | {d['labels']} | {d['positive_age_labels']} | {d['constrained_below_er']} | {d['preserved_above_cr']} |")
    lines += ['', '## 支持相同的定位比较与通信量', '',
              '定位只在双方都匹配到的同一节点—时刻—真值实例上计算 pooled RMSE。',
              '该支持量随比较变化，不把各自幸存轨迹上的误差当作相同定位任务。', '',
              '| 链路 | 参照 | 共同实例 | M-ECR-S RMSE m | 参照 RMSE m |',
              '| --- | --- | --- | --- | --- |']
    for row in result['common_aggregate']:
        if row['reference'] in freeze['primary_references'] + freeze['external_references']:
            label = dict(conditions)[row['condition']]
            lines.append(f"| {label} | {NAMES[row['reference']]} | {row['support']} | {row['candidate_rmse']:.6f} | {row['reference_rmse']:.6f} |")
    lines += ['', '所有通信方法共享相同投递矩阵和每方向每帧一个发包机会。候选密度包为',
              '216 B/Bernoulli + 32 B 包头；原密度控制为 208 B/Bernoulli + 32 B。',
              'TC 使用其对应局部后端的真实 5/10 帧历史包。逐来源实际包长、投递字节、',
              '16 KiB 分片和控制字节均独立核对；方法间可能有不同分片数，因此不是等字节实验。', '',
              '| 链路 | 方法 | 平均原始 MiB/序列 | 平均投递原始 MiB/序列 | 平均分片及控制 MiB/序列 |',
              '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        for arm in ['marked_lineage', 'marked_er', primary] + freeze['external_references']:
            row = aggregate[condition, arm]
            values = [row[key]['mean'] / 2**20 for key in ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']]
            lines.append(f"| {label} | {NAMES[arm]} | {values[0]:.5f} | {values[1]:.5f} | {values[2]:.5f} |")
    lines += ['', '## 分数信息本身的贡献', '',
              '以下把每个标记后端与其不使用分数的同一融合规则比较。这个差距属于局部',
              '观测模型的信息收益，不能全部归因于正时效上限。', '',
              '| 链路 | 共享标记版本 | 原版本 | 标记版本 OSPA | 原版本 OSPA | 配对差 [95% 区间] |',
              '| --- | --- | --- | --- | --- | --- |']
    controls = [('marked_lineage', 'lineage'), ('marked_er', 'qualified_exist'),
                ('marked_mil_support', 'mil_support'), ('marked_tc_ospa2_w5', 'tc_ospa2_w5'),
                ('marked_tc_ospa2_w10', 'tc_ospa2_w10'), ('marked_local', 'local')]
    for condition, label in conditions:
        for marked, original in controls:
            v = paired[condition, marked, original]['ospa']
            a = aggregate[condition, marked]['ospa']['mean']
            b = aggregate[condition, original]['ospa']['mean']
            lines.append(f"| {label} | {NAMES[marked]} | {NAMES[original]} | {a:.6f} | {b:.6f} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] |")
    lines += ['', '## 冻结校准的输入分布检查', '',
              '这是方法登记后的补充诊断，使用两个队列的全部输入，没有读取跟踪结果、',
              '重新拟合参数或改变主方法。正例仍按每传感器 12 m 截断匹配定义；该标签是',
              '二维关联代理。原始分数的 Brier 值仅用于说明其与这一代理事件的偏差，',
              '不假定检测器原本就把分数定义为该事件的概率。', '',
              '下表所有量均按序列等权。开发概率来自留一序列拟合，预留概率来自已冻结',
              '的全部开发序列拟合；两者均没有使用对应被评估序列拟合校准参数。', '',
              '| 队列 | 序列 / 帧 | 真值数/帧 | 检测数/传感器帧 | 代理正例比例 | 原分数 Brier | 校准 Brier |',
              '| --- | --- | --- | --- | --- | --- | --- |']
    for cohort, label in [('development', '开发'), ('reserved', '预留融合')]:
        row = input_shift['cohorts'][cohort]
        m = row['sequence_macro']
        lines.append(f"| {label} | {row['sequences']} / {row['frames']} | {m['truth_per_frame']:.3f} | {m['detections_per_sensor_frame']:.3f} | {m['positive_fraction']:.4f} | {m['raw_brier']:.6f} | {m['calibrated_brier']:.6f} |")
    lines += ['', '逐检测汇总和各序列明细见 input_shift.json / input_shift_sequences.csv。',
              '校准代理误差下降不等于融合增益，更不能代替前面的同信息配对比较；',
              '检测器已经见过训练划分这一限制仍然成立。']
    lines += ['', '## 全部序列，按原始编号排列', '',
              '每格分别是 M-ECR-S 的 OSPA，以及相对 M-No-age / M-ER 的差。', '',
              '| 序列 | 帧数 | 可靠 OSPA | 可靠 ΔNo-age / ΔER | 间歇 OSPA | 间歇 ΔNo-age / ΔER |',
              '| --- | --- | --- | --- | --- | --- |']
    for seq in freeze['units']:
        name = f'{seq:04d}'
        columns = [name, str(runs[name, 'reliable', primary]['frames'])]
        for condition, _ in conditions:
            value = runs[name, condition, primary]['ospa']
            deltas = [value - runs[name, condition, reference]['ospa'] for reference in freeze['primary_references']]
            columns.extend([f'{value:.6f}', f'{deltas[0]:+.6f} / {deltas[1]:+.6f}'])
        lines.append('| ' + ' | '.join(columns) + ' |')
    lines += ['', '## 核验、复现与结论边界', '',
              '独立重建所有 5601 帧原始输入、11202 个变换矩阵、75 个检测/真值源文件，',
              '并逐项核对冻结的分数映射。完整输出为 25 序列 × 2 种链路 × 16 方法 =',
              '800 个文件，独立重算 358464 个节点—帧。Python 重新生成 MATLAB 的无线',
              '随机数与完整投递矩阵；解析计算权重、时效修正、存在概率及候选同输入反事实。',
              '跟踪进程均要求真实退出成功、完整结束标志和 32 个序列结果文件同时满足。', '',
              '核验脚本最初把零时间戳统一解释为已有但未观测的标签，漏掉缺失标签的中性',
              '年龄因子 1，因而首次停止。修正分析器以遵从已经冻结的缺失标签规则后重新核验；',
              '跟踪公式、输入、预注册主方法和输出没有因此改动。原失败日志保留。', '',
              '前六个序列完成后，将剩余工作改为三个相互独立的 MATLAB 单计算线程进程，',
              '每进程执行一个独立序列；没有更改科学协议或公共运行器。原 0008 进程被有意',
              '中断（返回 -15），它的三个完整结果另存后在新进程重跑。具体等价性检查见',
              'parallel_equivalence.json；并行执行的 wall time 不作为受控方法耗时基准。', '',
              '所有基线使用相同局部观测信息时才评价融合规则增量。MIL-AM 是公开作者稿',
              '的 Gaussian/common-exclusive-label 实现，完整 TAES 2022 协议等价性未确立。',
              'TC 使用作者运动学融合函数及对应局部历史，保持不向局部密度反馈的架构。',
              '因此外部结果是统一后端适配比较，不是原论文整套性能复现。', '',
              '二维裁剪、相对 ego 坐标的常速度模型、固定近似检测似然和模拟通信保持既有',
              '设置；没有自车运动补偿、遮挡真值、位姿不确定性或真实机器人闭环。',
              '共同定位的数值变化和漏检/虚假代价应与 OSPA 一起解释。此审计不是第三方',
              '实验复现；统计区间也不证明普适收益或统计置信上界。', '',
              '```sh',
              './tmp/external_baselines/venv/bin/python trials/icra_fusion_holdout/audit_holdout_inputs.py',
              './tmp/external_baselines/venv/bin/python trials/icra_fusion_holdout/analyze_holdout.py --output-dir tmp/holdout_reaudit',
              './tmp/external_baselines/venv/bin/python trials/icra_fusion_holdout/plot_holdout.py',
              './tmp/external_baselines/venv/bin/python trials/icra_fusion_holdout/build_report.py',
              '```', '',
              'METHOD_FREEZE.json、源文件清单及其注册摘要不可在原位置重写。原始跟踪入口',
              'run_holdout.py 的覆盖保护会拒绝已有输出；完整重跑应使用独立目录并保留原件。',
              '论文图表应来自本完整摘要，不能由训练过程屏幕日志或部分序列摘录重算。', '']
    (OUT / 'README_CN.md').write_text('\n'.join(lines))
    decision = dict(primary=primary, all_registered_primary_intervals_below_zero=gate,
                    full_sequence_set_retained=True, same_information_references=freeze['primary_references'],
                    scope=freeze['limitations'], paper_gate=freeze['paper_gate'],
                    cap_label_event_diagnostics=cap_diagnostics,
                    main_contrasts=[paired[condition, primary, reference] for condition, _ in conditions
                                    for reference in freeze['primary_references']])
    (OUT / 'decision.json').write_text(json.dumps(decision, indent=2, allow_nan=False) + '\n')
    print('Complete 25-sequence report written; registered interval gate:', gate)


if __name__ == '__main__':
    main()
