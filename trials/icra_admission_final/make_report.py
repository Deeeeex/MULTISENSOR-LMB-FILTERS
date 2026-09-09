"""Write the reviewable Chinese result report directly from verified rows."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
ORIGINAL = 'marked_gaussian_evidence'
PROJECTED = ORIGINAL + '_projected_space'
FIXED = ORIGINAL + '_fixedx_000'
GS = ORIGINAL + '_guarded_scalar'
LABELS = {
    'marked_lineage': 'No-age KLA', 'marked_er': 'Recency', 'marked_asymmetric': 'Scalar',
    GS: 'Guarded Scalar', ORIGINAL + '_no_curvature': 'Joint w/o curvature',
    ORIGINAL + '_fixed_025': 'Fixed Ratio 0.25', FIXED: 'Fixed Ratio 0',
    ORIGINAL: '原 GCE', PROJECTED: '空间投影候选',
}


def main():
    evaluation = json.loads((HERE / 'EVALUATION_ANALYSIS.json').read_text())
    acceptance = json.loads((HERE / 'REPLAY_ACCEPTANCE.json').read_text())
    summary_audit = json.loads((HERE / 'SUMMARY_AUDIT.json').read_text())
    selection = json.loads((HERE.parent / 'icra_compatible_admission/FINAL_SELECTION.json').read_text())
    rejection = json.loads((HERE.parent / 'icra_admission_revision/REJECTION_DIAGNOSTIC.json').read_text())
    support = json.loads((HERE / 'DETECTION_SUPPORT_DIAGNOSTIC.json').read_text())
    scan = json.loads((HERE.parent / 'icra_scan_admission/SCAN_DIAGNOSTIC.json').read_text())
    assert all(r['passed'] for r in [evaluation, acceptance, summary_audit, selection, rejection, support, scan])

    def mean(dataset, arm, condition, scope='all'):
        return next(r['sequence_macro']['ospa'] for r in evaluation['aggregates']
                    if (r['dataset'], r['scope'], r['condition'], r['arm']) == (dataset, scope, condition, arm))

    def table(dataset, arms, scope='all'):
        lines = ['| 方法 | 可靠链路 OSPA（m） | 间歇链路 OSPA（m） |', '| --- | ---: | ---: |']
        lines += [f'| {LABELS[arm]} | {mean(dataset, arm, "reliable", scope):.6f} | {mean(dataset, arm, "intermittent", scope):.6f} |' for arm in arms]
        return lines

    text = ['# 当前证据准入：完整实验结果', '',
            '**本轮没有得到可替换原 GCE 的新方法。** 三类方案共九个候选完成全部九段开发数据的完整递归。开发集选出的空间投影候选在全量 V2V4Real 的两种链路下均退步，在新增 V2X-Real 上也没有改善双条件均值，因此保留原 GCE 作为工作基线。', '',
            '## 全量 V2V4Real', '',
            '43 个去重片段、9,699 对双车帧、17 个录制分组。训练/测试中相同的 0000 只计一次。表中片段等权；九段开发数据和其余 34 段也在 JSON 中分别汇总。', '']
    arms = ['marked_lineage', 'marked_er', 'marked_asymmetric', GS, ORIGINAL + '_no_curvature', FIXED, ORIGINAL, PROJECTED]
    text += table('v2v', arms)
    gains = [100*(1-mean('v2v', ORIGINAL, c)/mean('v2v', 'marked_lineage', c)) for c in ['reliable', 'intermittent']]
    text += ['', f'原 GCE 相对 No-age 改善 {gains[0]:.3f}%/{gains[1]:.3f}%。空间投影候选相对原 GCE 的 OSPA 变化为 '
             f'+{mean("v2v", PROJECTED, "reliable")-mean("v2v", ORIGINAL, "reliable"):.6f}/+'
             f'{mean("v2v", PROJECTED, "intermittent")-mean("v2v", ORIGINAL, "intermittent"):.6f} m。', '',
             '按录制等权计算原 GCE − Guarded Scalar，并对 17 个录制分组做配对重采样：', '']
    for condition, label in [('reliable', '可靠'), ('intermittent', '间歇')]:
        r = next(r for r in evaluation['paired'] if (r['dataset'], r['scope'], r['condition'], r['candidate'], r['reference']) == ('v2v','all',condition,ORIGINAL,GS))
        text.append(f'- {label}：{r["group_macro_difference"]:.6f} m，描述性 95% 区间 [{r["low"]:.6f}, {r["high"]:.6f}]。')
    text += ['', '## 冻结后的 V2X-Real 评测', '',
             '官方 64 线 validation 发布中，全部五个具有双车文件的片段共 619 对帧，来自三个采集日期。仅有单车加路侧节点的另一个片段在查看跟踪分数前按输入结构排除。检测器、校准、局部跟踪器和通信条件沿用固定配置。', '']
    text += table('v2x', arms)
    text += ['', '全部五段均保留。两个片段的双车距离始终超过 80 m，两个 40 m 感知圆不重叠；另三个片段始终重叠。按事先由位姿定义的重叠组统计，原 GCE 为 '
             f'{mean("v2x", ORIGINAL, "reliable", "overlapping_supports"):.6f}/{mean("v2x", ORIGINAL, "intermittent", "overlapping_supports"):.6f} m，No-age 为 '
             f'{mean("v2x", "marked_lineage", "reliable", "overlapping_supports"):.6f}/{mean("v2x", "marked_lineage", "intermittent", "overlapping_supports"):.6f} m。', '',
             '## 门控反转的诊断', '',
             '在此前指出的 409 帧录制上，被拒绝的增量以负证据为主：', '',
             '| 链路 | 拒绝的源使用次数 | 负增量 | 正增量 |', '| --- | ---: | ---: | ---: |']
    for r in rejection['rejection_statistics']:
        if r['cohort'] == 'new_recording_diagnostic':
            text.append(f'| {r["condition"]} | {r["rejected_source_uses"]} | {r["rejected_negative_sources"]} | {r["rejected_positive_sources"]} |')
    text += ['', '固定输入下恢复负证据能减少该片段的虚警，但在旧开发集的拥挤片段上增加漏检。空间/存在概率分离无法单独解决这个取舍。多数被整体拒绝的空间增量仍有一个收缩方向；保留该方向的归一化边缘比率通过了公式和坐标变换检查，但完整递归收益未延续到评测数据。', '',
             '## 全部开发选择', '',
             '以下按两种链路的九段等权 OSPA 均值排序。所有条目均完成自身递归，旧 GCE 输入上的替换结果没有混入该表。', '',
             '| 候选 | 双条件开发均值（m） |', '| --- | ---: |']
    for r in selection['candidate_ranking']:
        name = r['arm'].replace('marked_gaussian_evidence', 'GCE')
        text.append(f'| {name} | {r["selection_mean_ospa"]:.9f} |')
    text += ['', '固定强度扩展为 {0, 0.05, 0.1, 0.125, 0.25, 0.5, 1}，完整递归选择 η=0。其双条件开发均值为 '
             f'{selection["selected_fixed"]["selection_mean_ospa"]:.9f} m；η=0.25 的旧结果仍保留于逐片段数据。', '',
             '## 下一项问题：漏检证据的可信度', '',
             '按与校准一致的 12 m 一对一中心配对，在每个传感器的固定 40 m 支持内：', '',
             '| 数据 | 匹配检测/标注机会 | 汇总召回 | 汇总精确率 |', '| --- | ---: | ---: | ---: |']
    for r in support['aggregate']:
        if r['cutoff_m'] == 12:
            text.append(f'| {r["dataset"]} | {r["matches"]}/{r["truth_sensor_opportunities"]} | {100*r["pooled_recall"]:.2f}% | {100*r["pooled_precision"]:.2f}% |')
    text += ['', '外部数据的漏检支持发生明显变化，统一 pD=0.9 仍用于所有方法。这里的标注机会包含遮挡目标，检测召回不能直接当作逐目标可见性。', '',
             '额外检查了两种仅由当前关联质量估计负证据强度的规则：用预测存在概率、或其平方作为权重。它们只在固定输入上给 V2X-Real 带来约 0.001 m 的均值变化，同时使旧开发集变差，均未通过预设推进条件，未启动完整递归。下一步需要能区分遮挡与真实消失的逐目标信息；继续整体缩放当前负增量没有获得稳定收益。', '',
             '## 执行与复核', '',
             f'- 本轮保留 {acceptance["completed_native_result_files"]} 份完整原生轨迹，独立复算 {acceptance["independently_audited_node_frames"]:,} 个机器人帧；八个成功批次均退出为 0。',
             f'- 原有 {acceptance["original_protected_sources_unchanged"]} 项受保护源码保持一致；八项旧方法预检比较逐字段一致。',
             '- 首次混合输入清单因 MATLAB 结构字段不一致而退出，未产生轨迹；修复只统一配置字段，原失败日志和修复记录均保留。独立审计器的导入修复也使用新版本文件保留原记录。',
             '- 第三类候选定义和选择规则写入后，一次日志查看提前显示了 train_0014 可靠链路的两条分数；候选和选择规则未变，详情见 `../icra_compatible_admission/DATA_EXPOSURE.json`。V2X 分数在最终开发选择写入之后才查看。',
             '- 当前关联率检查是在查看 V2X 结果之后开展，这五段已是该后续方向的开发数据，不能再次作为其独立验证。', '',
             '结果入口：`all_evaluation_scores.csv`、`EVALUATION_ANALYSIS.json`；原生文件清单：`RESULT_FILES_MANIFEST.json`；执行验收：`REPLAY_ACCEPTANCE.json`；汇总复算：`SUMMARY_AUDIT.json`。', '',
             '只读复核命令：', '', '```sh',
             'tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_admission_final/accept_results.py --verify-only',
             'tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_admission_final/verify_summary.py', '```', '',
             '第一条需要本研究目录中保留的完整原生轨迹；第二条只需随包 JSON、Python 和 NumPy，可在独立解压目录复算汇总。', '',
             '数据来源：[V2X-Real 官方代码](https://github.com/ucla-mobility/V2X-Real)、[ECCV 2024 论文](https://doi.org/10.1007/978-3-031-72943-0_26)。本轮没有将失败候选替换进论文主方法，也没有把本轮结果表述为效果提升。', '']
    (HERE / 'RESULTS_CN.md').write_text('\n'.join(text))
    print('Wrote complete evidence-based report:', HERE / 'RESULTS_CN.md', flush=True)


if __name__ == '__main__':
    main()
