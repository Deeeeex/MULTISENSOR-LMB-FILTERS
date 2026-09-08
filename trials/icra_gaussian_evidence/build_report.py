"""Report every fixed arm, native costs, guard event and component comparison."""
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
         'marked_asymmetric': 'M-AE 空间近似对照', 'marked_gaussian_evidence': 'M-GE（主）',
         'marked_gaussian_evidence_no_curvature': '去除曲率保护',
         'marked_gaussian_evidence_no_history': '去除历史保守项',
         'marked_gaussian_evidence_no_mark': '去除正向分数约束'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    data = json.loads((OUT / f'summary_{cohort}.json').read_text())
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    n, primary = data['sequences'], data['primary']
    assert primary == 'marked_gaussian_evidence' and n == (9 if cohort == 'development' else 25)
    names = [f"{u['sequence']:04d}" for u in frozen['units'] if u['cohort'] == cohort]
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in data['runs']}
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    samples = np.random.default_rng(8301).integers(0, n, (10000, n))
    ablations = []
    for condition, _ in conditions:
        for reference in frozen['arms'][1:] + frozen['component_controls']:
            ablations.append(dict(condition=condition, candidate=primary, reference=reference,
                **{key: interval([lookup[s, condition, primary][key]-lookup[s, condition, reference][key] for s in names], samples)
                   for key in ['ospa', 'miss2', 'false2']}))
    (OUT / f'ablation_comparisons_{cohort}.json').write_text(json.dumps(ablations, indent=2) + '\n')
    decision = ('主方法通过预先固定的继续条件，允许执行 25 段已见迁移序列。' if data['continuation_gate_passed'] else
                '主方法未通过预先固定的继续条件，本候选不扩大至 25 段；保留全部结果。') if cohort == 'development' else (
                '已完成全部 25 段已见迁移序列。四项主要配对区间' + ('均低于零。' if data['all_four_primary_intervals_below_zero'] else '未全部低于零。'))
    lines = [f'# 高斯空间与存在一致校正：{n} 段完整结果', '', decision, '',
             '全部真实轨迹已经参与此前开发；本轮仍是已见数据上的方法开发，不能称为独立测试。',
             '固定主方法、三项消融和标量 M-AE 组件对照均公开，次要版本不能替代主方法。', '',
             '## 方法与消息', '',
             '沿用 M-AE 的本地更新、正负证据门控与历史保守权重，同时把本地更新/预测',
             '高斯比值乘入空间密度，并重算存在归一化项。主方法拒绝负曲率来源，',
             '非可积合并退回基础融合。完整数学边界见 DERIVATION_CN.md。', '',
             '新包 352 B/Bernoulli + 32 B 包头；标量 M-AE 为 232 B/Bernoulli。',
             '共同原始观测与本地关联设置并不意味着相同消息或相同字节开销。', '',
             '## 全部方法', '',
             '各序列等权；OSPA 单位 m，漏检和虚假 GOSPA 平方代价单位 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    aggregate = {(r['condition'], r['arm']): r for r in data['aggregate']}
    arms = list(dict.fromkeys(frozen['primary_references']+frozen['additional_references']+frozen['component_controls']+frozen['arms_by_cohort'][cohort]))
    for condition, label in conditions:
        for arm in arms:
            r = aggregate[condition, arm]
            lines.append(f"| {label} | {LABEL[arm]} | {r['ospa']['mean']:.6f} | {r['miss2']['mean']:.6f} | {r['false2']['mean']:.6f} |")
    lines += ['', '## 主方法的所有配对比较', '',
              '差值为主方法减参照，负数有利。区间为 10000 次序列重采样的描述性 95%',
              '百分位区间，未调整多重比较；不能代替新的验证集。', '',
              '| 链路 | 参照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 | 胜出序列 |',
              '| --- | --- | --- | --- | --- | --- |']
    for r in data['paired']:
        if r['candidate'] != primary:
            continue
        v = r['ospa']
        lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} | {r['ospa_wins']}/{n} |")
    lines += ['', '## 组件消融', '', '区间包含零时，该组件的必要性仍未证实。', '',
              '| 链路 | 对照 | ΔOSPA [95% 区间] | Δ漏检 | Δ虚假 |',
              '| --- | --- | --- | --- | --- |']
    for r in ablations:
        v = r['ospa']
        lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} |")
    lines += ['', '## 共同真值定位', '', '仅比较双方共同匹配到的真值实例，支持量随参照改变。', '',
              '| 链路 | 参照 | 共同实例 | 主方法 RMSE | 参照 RMSE |',
              '| --- | --- | --- | --- | --- |']
    for r in data['common_aggregate']:
        if r['candidate'] == primary:
            lines.append(f"| {dict(conditions)[r['condition']]} | {LABEL[r['reference']]} | {r['support']} | {r['candidate_rmse']:.6f} | {r['reference_rmse']:.6f} |")
    lines += ['', '## 固定输入的空间与存在诊断', '',
              '仅在主方法已产生的实际融合输入上替换存在率或空间均值后重新提取，',
              '按序列平均。scalar_AS 不是重新递推的 M-AE；完整 M-AE 对照在上表。', '',
              '| 链路 | 同输入规则 | OSPA | 漏检代价 | 虚假代价 |',
              '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        group = [r for r in data['diagnostics'] if r['condition'] == condition and r['arm'] == primary]
        assert len(group) == n
        for rule in ['scalar_AS', 'new_r_old_space', 'old_r_new_space', 'candidate']:
            values = [np.mean([r['same_input'][rule][key] for r in group]) for key in ['ospa', 'miss2', 'false2']]
            lines.append(f'| {label} | {rule} | {values[0]:.6f} | {values[1]:.6f} | {values[2]:.6f} |')
    lines += ['', '## 曲率与回退', '',
              '| 链路 | 方法 | 已检查融合标签 | 实际补充 | 曲率拒绝来源 | 整体回退 | 最大位置变动 m |',
              '| --- | --- | --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        for arm in frozen['arms']:
            group = [r for r in data['diagnostics'] if r['condition'] == condition and r['arm'] == arm]
            values = [sum(r[key] for r in group) for key in ['labels', 'corrected_labels', 'curvature_rejected_sources', 'aggregate_fallbacks']]
            lines.append(f"| {label} | {LABEL[arm]} | {values[0]} | {values[1]} | {values[2]} | {values[3]} | {max(r['max_spatial_shift_m'] for r in group):.6f} |")
    lines += ['', '## 原始与分片开销', '',
              '表中是每序列总字节的序列平均，除以 1024 为 KiB。开发期旧 M-No-age/M-ER',
              '曾携带未使用标量，不能把其记录字节当作原生 208 B 包成本。', '',
              '| 链路 | 方法 | 原始 KiB | 实际投递 KiB | 分片及控制 KiB |',
              '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        for arm in arms:
            r = aggregate[condition, arm]
            values = [r[key]['mean']/1024 for key in ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']]
            lines.append(f'| {label} | {LABEL[arm]} | {values[0]:.3f} | {values[1]:.3f} | {values[2]:.3f} |')
    count = lambda key: sum(r[key] for r in data['diagnostics'])
    lines += ['', '## 完整性与复算', '',
              f"完成 {n} 段、{data['frames']} 帧、{2*n*len(frozen['arms_by_cohort'][cohort])} 个新文件；实际 MATLAB 退出均为 0。",
              f"Python 复算 {data['new_node_frames']} 个新节点—帧和 {data['rescored_baseline_node_frames']} 个参照节点—帧。",
              f"核对 {count('local_update_records')} 条本地增量、{count('joined_source_records')} 个来源连接，",
              f"并从 {count('local_gaussian_records')} 对本地预测/后验矩重建 {count('packet_gaussian_records')} 条传输高斯比值。",
              f"按原来源与标签连接 {count('gaussian_source_joins')} 个融合高斯输入，独立重算完整均值、协方差、归一化项和存在率。",
              '预检复算 2940 个节点—帧；588 个原 M-AE 节点—帧的轨迹、指标、原生包',
              '和全部原 37 列融合及 12 列本地诊断逐值相同。源文件与注册文件散列均验证。', '',
              '单位夹具最初要求单源输入按位相同，诊断确认新旧实现相同而原公式本身',
              '存在约 1e-15 舍入；按输入 1e-12 容差与新旧按位一致分别验证，修复发生',
              '在任何跟踪与源冻结之前。首次审计协议字符串的检查拼写不一致，修正',
              '审计器后重审，未修改模型、源冻结或跟踪输出。原日志均保留。', '',
              '此为同一工作流程中的独立实现复算，并非第三方复现。近似关联、检测',
              '相关性、异质先验、二维相对坐标和仿真链路仍限制结论。论文尚未据本轮改写。', '']
    (OUT / f'RESULTS_{cohort}_CN.md').write_text('\n'.join(lines))
    print('Complete Gaussian evidence report written:', cohort, 'continuation:', data['continuation_gate_passed'])


if __name__ == '__main__':
    main()
