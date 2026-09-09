"""Render the completed frozen development, exposed and additional-test study."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

from identity_metrics import pooled

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
BASE = 'marked_gaussian_evidence'
GS = BASE + '_guarded_scalar'
QN = BASE + '_assoc_quality_nis'
GQN = GS + '_assoc_quality_nis'
NOAGE = 'marked_lineage'
CONDITIONS = ['reliable', 'intermittent']
LABELS = {BASE: '原 GCE', GS: 'Guarded Scalar', NOAGE: 'No-age KLA',
    QN: 'GCE + QN', GQN: 'Guarded Scalar + QN',
    BASE + '_assoc_direct': 'D：当前观测匹配',
    BASE + '_assoc_temporal': 'T：三帧观测匹配',
    BASE + '_assoc_reopen': 'R：仅重开冲突关联',
    BASE + '_assoc_split': 'S：重开并保留分支',
    GS + '_assoc_split': 'Guarded Scalar + S',
    BASE + '_assoc_quality': 'Q：置信度门槛 0.9',
    BASE + '_assoc_nis': 'N：冲突分位 0.999',
    'marked_asymmetric': 'Asymmetric', 'marked_er': 'ER',
    BASE + '_fixed_025': 'Fixed strength 0.25',
    BASE + '_fixedx_000': 'Fixed strength 0',
    BASE + '_no_curvature': 'No curvature',
    BASE + '_projected_space': 'Projected space'}
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def pair(report, scope, candidate=QN, reference=BASE):
    return {r['condition']: r for r in report['paired_recording']
            if r['scope'] == scope and r['candidate'] == candidate and r['reference'] == reference}


def groups(report, scope):
    return {(r['arm'], r['condition']): r for r in report['aggregate'] if r['scope'] == scope}


def ospa_table(report, scope, arms):
    lookup = groups(report, scope)
    lines = ['| 版本 | 可靠 OSPA | 间歇 OSPA |', '| --- | ---: | ---: |']
    for arm in arms:
        values = [lookup[arm, c]['sequence_macro']['ospa'] for c in CONDITIONS]
        lines.append(f'| {LABELS.get(arm, arm)} | {values[0]:.6f} | {values[1]:.6f} |')
    return lines


def effect_table(report, scope):
    lookup = groups(report, scope); pairs = pair(report, scope)
    fields = {k: [] for k in ['OSPA 相对改善', 'GOSPA（m）', '定位平方项', '漏检平方项', '虚警平方项', '基数误差', 'wire 字节增幅', 'payload 字节增幅',
        '已评分关联错误数／总数（2 m）', '共同目标漏配数／机会数（2 m）',
        '同一标注目标的相邻输出标签变化（2 m）', '同一输出标签的相邻标注目标变化（2 m）',
        '分支创建次数', '组等权 OSPA 差及 95% 区间（m）', '改善／变差／不变片段数']}
    for c in CONDITIONS:
        a, b = lookup[BASE, c], lookup[QN, c]
        fields['OSPA 相对改善'].append(f"{100*(1-b['sequence_macro']['ospa']/a['sequence_macro']['ospa']):+.3f}%")
        for title, key in [('GOSPA（m）','gospa'),('定位平方项','loc2'),('漏检平方项','miss2'),('虚警平方项','false2'),('基数误差','countError')]:
            fields[title].append(f"{a['sequence_macro'][key]:.5f} → {b['sequence_macro'][key]:.5f}")
        for title, key in [('wire 字节增幅', 'wire_bytes'), ('payload 字节增幅', 'raw_payload_bytes')]:
            fields[title].append(f"{100*(b['communication'][key]/a['communication'][key]-1):+.2f}%")
        ia, ib = a['identity']['2'], b['identity']['2']
        fields['已评分关联错误数／总数（2 m）'].append(
            f"{ia['wrong_pairs']:,}/{ia['scored_pairs']:,} → {ib['wrong_pairs']:,}/{ib['scored_pairs']:,}")
        fields['共同目标漏配数／机会数（2 m）'].append(
            f"{ia['counts'].get('common_identity_pairs_missed',0):,}/{ia['counts'].get('common_identity_opportunities',0):,} → "
            f"{ib['counts'].get('common_identity_pairs_missed',0):,}/{ib['counts'].get('common_identity_opportunities',0):,}")
        for title, key in [('同一标注目标的相邻输出标签变化（2 m）','truth_label_switches'),
                           ('同一输出标签的相邻标注目标变化（2 m）','label_truth_switches')]:
            fields[title].append(f"{ia['counts'].get(key,0):,} → {ib['counts'].get(key,0):,}")
        fields['分支创建次数'].append(str(b['communication']['split_branches']))
        p = pairs[c]
        fields['组等权 OSPA 差及 95% 区间（m）'].append(
            f"{p['recording_macro_difference']:+.5f} [{p['low']:+.5f}, {p['high']:+.5f}]")
        fields['改善／变差／不变片段数'].append(f"{p['improved']}／{p['worsened']}／{p['unchanged']}")
    return ['| QN 相对原 GCE | 可靠链路 | 间歇链路 |', '| --- | ---: | ---: |'] + [
        f'| {k} | {v[0]} | {v[1]} |' for k, v in fields.items()]


def main():
    paths = {name: OUT / filename for name, filename in [
        ('development', 'SCREENED_DEVELOPMENT_SELECTION.json'),
        ('restored', 'RESTORED_ASSESSMENT.json'),
        ('exposed', 'SCREENED_ASSESSMENT.json'),
        ('test', 'ADDITIONAL_TEST_ANALYSIS.json')]}
    reports = {k: json.loads(p.read_text()) for k, p in paths.items()}
    assert all(r['passed'] for r in reports.values())
    dev, exposed, test = [reports[k] for k in ['development', 'exposed', 'test']]
    assert dev['advance'] and dev['selected']['arm'] == exposed['selected'] == test['selected'] == QN
    assert len(exposed['rows']) == 1066 and len(test['rows']) == 140
    assert test['unique_segments'] == 14 and test['paired_frames'] == 2172 and test['collection_dates'] == 5
    cohort_path = OUT.parent / 'icra_association_test/COHORT_FREEZE.json'
    cohort = json.loads(cohort_path.read_text())
    adapter_path = OUT.parent / 'icra_association_test/ADAPTER_CHECK.json'
    adapter = json.loads(adapter_path.read_text()); assert adapter['passed']
    assert cohort['development_selection_sha256'] == sha(paths['development'])
    assert datetime.fromisoformat(dev['created_utc']) < datetime.fromisoformat(cohort['created_utc'])
    lookup, fresh = groups(exposed, 'v2v_all'), groups(test, 'v2x_test')
    gains = [100*(1-lookup[QN,c]['sequence_macro']['ospa']/lookup[BASE,c]['sequence_macro']['ospa']) for c in CONDITIONS]
    newgains = [100*(1-fresh[QN,c]['sequence_macro']['ospa']/fresh[BASE,c]['sequence_macro']['ospa']) for c in CONDITIONS]
    transfer = test['transfer_improves_both_conditions']
    text = ['# 跨车关联与时序一致性：完整冻结评估', '',
        f"固定的 QN 版本在全部 43 段 V2V 上，相对原 GCE 的可靠／间歇 OSPA 改善 **{gains[0]:.2f}%／{gains[1]:.2f}%**。",
        f"新增 14 段 V2X 测试上的相对改善为 **{newgains[0]:+.3f}%／{newgains[1]:+.3f}%**；" +
        ('通过两种链路均改善的迁移门槛。' if transfer else '未通过两种链路均改善的迁移门槛。'), '',
        '所有原生运行已退出，逐轨迹核验通过。QN 在开发集选择后保持不变；测试集用于一次冻结评估。原论文尚未据此改写。', '',
        '## 方法与归因', '',
        '每条高斯分量增加当前观测均值、协方差、关联质量等 8 个 float64 摘要，分量从 352 B 增至 416 B。接收端只积累实际收到的当前及前两帧证据；至少两个有效样本且两端置信度均达到 0.9，累计位置差异超过 0.999 名义卡方门槛时，才重开冲突共享标签。保留独立分支，并通过源特定弃权避免把被拒绝的对应当作负观测。门槛没有被解释成校准后的错误概率。', '',
        '检测器、分数校准、局部滤波、运动模型、提取规则和链路随机数全部保持一致；每种方法执行自己的完整递归。同一 QN 前端分别接入 GCE 和 Guarded Scalar，用于区分关联变化与融合公式的贡献。', '',
        '## 九段开发集', '',
        '共 1,993 对双车帧。表中 OSPA 为片段等权均值，单位 m；所有有效候选均列出。', '',
        '| 版本 | 可靠 OSPA | 间歇 OSPA | 已评分关联错误数／总数（2 m，两种链路合并） |',
        '| --- | ---: | ---: | ---: |']
    for arm in [BASE]+[BASE+'_assoc_'+m for m in ['direct','temporal','reopen','split','quality','nis','quality_nis']]:
        values = {r['condition']: r['ospa'] for r in dev['aggregate'] if r['arm'] == arm}
        identity = pooled([r for r in dev['identity_rows'] if r['arm'] == arm])['2']
        text.append(f"| {LABELS[arm]} | {values['reliable']:.6f} | {values['intermittent']:.6f} | {identity['wrong_pairs']:,}／{identity['scored_pairs']:,} |")
    text += ['', 'Q/N/QN 的预设筛选要求两种链路 OSPA 均改善至少 1%，且合并的已评分错误关联率不升，再按两种链路平均 OSPA 选择。该筛选选出 QN。全部开发结果及早期 D/T/R/S 的失败均保留。', '',
        '## 全部 V2V：43 段、9,699 对帧、17 个原始记录组', '',
        '这些数据此前已被本项目使用；全量结果用于固定版本评估。按原始记录整组配对重采样 10,000 次，种子 8301。', '']
    full_arms = [NOAGE, 'marked_asymmetric', 'marked_er', BASE+'_fixed_025', BASE+'_no_curvature',
                 BASE+'_projected_space', GS, BASE, GS+'_assoc_split', BASE+'_assoc_split', GQN, QN]
    available = {r['arm'] for r in exposed['aggregate'] if r['scope'] == 'v2v_all'}
    text += ospa_table(exposed, 'v2v_all', [a for a in full_arms if a in available]) + ['']
    text += effect_table(exposed, 'v2v_all') + ['']
    text += ['关联错误率下降没有同时转化为更少的输出标签变化；表中同时保留共同目标漏配和双向相邻标签变化。2 m／12 m 的一对一标注分配仅在离线诊断中使用，不能等同于标准 MOT IDSW。', '',
        '## 既有 V2X 验证：5 段、619 对帧', '']
    text += ospa_table(exposed, 'v2x_val', [NOAGE, GS, BASE, GQN, QN]) + ['']
    text += ['## 新增 V2X 测试：14 段、2,172 对帧、5 个采集日期', '',
        '测试队列先按公开目录登记全部双车片段，在固定 QN 之后获取原始数据。所有 8,688 个文件完成 CRC／SHA-256 核验；与既有验证集无完全相同的点云文件。相关片段按采集日期分组，未按结果筛除片段。输入转换逐帧核对官方位姿函数、原始标注、相同裁剪、检测输出和固定校准。', '']
    text += ospa_table(test, 'v2x_test', [NOAGE, GS, BASE, GQN, QN]) + ['']
    text += effect_table(test, 'v2x_test') + ['',
        '| 对照差值：GCE + QN 减去参考方法 | 链路 | 片段等权差（m） | 日期等权差及 95% 区间（m） |',
        '| --- | --- | ---: | ---: |']
    for reference in [BASE, GQN, NOAGE]:
        for c, row in pair(test, 'v2x_test', QN, reference).items():
            text.append(f"| {LABELS[reference]} | {c} | {row['sequence_macro_difference']:+.6f} | {row['recording_macro_difference']:+.6f} [{row['low']:+.6f}, {row['high']:+.6f}] |")
    text += ['', '区间重采样单位为五个采集日期，反映这五组数据的配对差异；不把 14 个相关片段当作 14 次独立采集。', '',
        '### 预先登记的几何分层', '',
        '两车已知平台位置的平面距离小于 80 m 时，两个 40 m 传感器圆盘相交。以下是按合格机器人帧加权的次要结果；主结论始终使用全部 14 段。', '',
        '| 几何分层 | 双车帧数 | 链路 | 原 GCE | GCE + QN | Guarded Scalar + QN |',
        '| --- | ---: | --- | ---: | ---: | ---: |']
    geo = {(r['stratum'],r['condition'],r['arm']):r for r in test['geometry_aggregate']}
    for stratum in ['overlapping_disks','nonoverlapping_disks']:
        for c in CONDITIONS:
            rows = [geo[stratum,c,a] for a in [BASE,QN,GQN]]
            values = ['—' if r['frame_weighted']['ospa'] is None else f"{r['frame_weighted']['ospa']:.6f}" for r in rows]
            text.append(f"| {stratum} | {rows[0]['paired_frames']} | {c} | {' | '.join(values)} |")
    text += ['', '## 结果边界与复核入口', '',
        '这是冻结的二维递归融合评估，包含截断 OSPA、GOSPA 分解和基数误差；它不代表 V2X-Real 官方三维检测榜单。', '',
        f"新增数据中，两车对共同标注 ID 的中心并非完全相同：中位差 {adapter['shared_annotation_centers']['median_m']:.3f} m，最大差 {adapter['shared_annotation_centers']['maximum_m']:.3f} m。沿用作者的首来源优先去重规则；这一原始标注差异限制了细粒度身份诊断的解释。", '',
        '- `SCREENED_DEVELOPMENT_SELECTION.json`：开发选择、全部候选和身份诊断。',
        '- `SCREENED_ASSESSMENT.json`：1,066 行已曝光全量／验证结果及配对区间。',
        '- `ADDITIONAL_TEST_ANALYSIS.json`：140 行新增测试结果、五种方法、全部 14 段、两种链路及几何分层。',
        '- `additional_test_scores.csv`、`additional_test_geometry.csv`：逐段结果及几何分层导出。',
        '- `PERSISTENT_METHOD_SPEC.md`、`CANDIDATES_V3.md`、`../icra_association_test/PROTOCOL.md`：算法、选择规则和迁移协议。',
        '- `README.md`：有效原生执行入口、审计入口和修复记录。', '',
        '早期路径遮蔽错误产生的 50 份轨迹及旧 V1 选择报告已排除。No-age 预检的物理轨迹逐项一致，仅两列不参与更新的旧 ceiling 诊断存在已记录差异。失败文件、原始日志和修复凭据全部保留。', '']
    output = OUT / 'FINAL_RESULTS_CN.md'; output.write_text('\n'.join(text))
    receipt = dict(completed_utc=datetime.now(timezone.utc).isoformat(), passed=True,
        inputs={str(p.relative_to(ROOT)):sha(p) for p in list(paths.values())+[cohort_path,adapter_path]},
        generator_sha256=sha(Path(__file__)), output_path=str(output.relative_to(ROOT)), output_sha256=sha(output),
        transfer_improves_both_conditions=transfer)
    (OUT/'FINAL_RESULTS_RENDER.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('FINAL ASSOCIATION REPORT RENDERED', output.relative_to(ROOT), flush=True)


if __name__ == '__main__':
    main()
