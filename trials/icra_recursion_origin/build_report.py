"""Summarize the entire predeclared causal experiment without selecting a method."""
from pathlib import Path
import csv
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence_range'
GS='marked_gaussian_evidence_guarded_scalar_range'
NOAGE='marked_lineage_range'
MODES=['joint','existence','spatial']
LABELS={GCE:'持续 GCE',GS:'持续 GS',NOAGE:'No-age（已审核旧轨迹）'}
LABELS.update({'marked_gaussian_evidence_guarded_scalar_once_'+m+'_range':v for m,v in zip(MODES,['第 3 帧联合替换','第 3 帧只换存在率','第 3 帧只换空间分布'])})
ORDER=[GCE,GS]+['marked_gaussian_evidence_guarded_scalar_once_'+m+'_range' for m in MODES]+[NOAGE]

def main():
    destination=OUT/'INTERVENTION_RESULTS.json';assert not destination.exists()
    trace=json.loads((OUT/'ORIGIN_TRACE.json').read_text())
    assert trace['passed'] and json.loads((OUT/'TRACE_VERIFICATION.json').read_text())['passed']
    audits=[json.loads((OUT/('audit_recursion_'+k+'.json')).read_text()) for k in ['preflight','interventions']]
    assert all(a['passed'] for a in audits)
    rows={r['arm']:r for a in audits for r in a['rows']}
    diagnostics={r['arm']:r for a in audits for r in a['diagnostics']}
    assert set(rows)==set(ORDER) and sum(a['audited_robot_frames'] for a in audits)==2400
    for a in audits:
        for name,expected in a['inputs'].items():assert sha(ROOT/name)==expected,name
    effects=[]
    for arm in ORDER[2:5]:
        r=rows[arm];s=rows[GS];target=diagnostics[arm]['target'];base=diagnostics[GS]['target']
        effects.append(dict(arm=arm,window_detection_delta=target['windows']['original_window']['detected']-base['windows']['original_window']['detected'],
            full_detection_delta=target['windows']['full']['detected']-base['windows']['full']['detected'],
            ospa_change_percent=100*(r['ospa']/s['ospa']-1),wire_byte_change_percent=100*(r['wire_bytes']/s['wire_bytes']-1)))
    result=dict(passed=True,scope='One previously exposed range/intermittent case; no method promotion or further intervention search',
        sequence='v2xt_0001',truth_id=5,intervention_frame=3,rows=[rows[k] for k in ORDER],
        diagnostics=[diagnostics[k] for k in ORDER],effects=effects,
        audited_new_robot_frames=2400,audited_reused_robot_frames=480,
        window_gce=diagnostics[GCE]['target']['windows']['original_window']['detected'],
        window_gs=diagnostics[GS]['target']['windows']['original_window']['detected'],
        any_window_improvement=any(e['window_detection_delta']>0 for e in effects),
        inputs={str(p.relative_to(ROOT)):sha(p) for p in [OUT/'ORIGIN_TRACE.json',OUT/'TRACE_VERIFICATION.json',
            OUT/'INTERVENTION_FREEZE.json',OUT/'INTERVENTION_PROTOCOL.md',
            OUT/'audit_recursion_preflight.json',OUT/'audit_recursion_interventions.json']})
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'ALL_INTERVENTION_SCORES.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[GCE]),lineterminator='\n');writer.writeheader();writer.writerows(result['rows'])
    lines=['# 第一次递推分化与单次状态替换实验','',
        '本实验回答一个有限的问题：GCE 与 Guarded Scalar（GS）最早的融合分布变化，是否足以改变后续候选标签碎裂。它延续原 GCE 失败原因诊断，不构成新的方法性能证据。共享距离检出率模型的筛选已关闭，本实验不会重新选择该模型或延长干预时间。','',
        '## 已核实的起点','',
        '12 条原生轨迹覆盖同一个 `v2xt_0001` 场景的 nominal/range 检出率、可靠/间歇链路，以及 GCE、GS、No-age。目标为 GT 5；分析使用完整 240 帧。4 个模型/链路组合中，GCE 与 GS 首次超过预定容差的融合后分布差异均在第 3 帧、机器人 1，局部更新差异均从第 4 帧开始。','',
        '第 3 帧的局部输入只在数值容差内相同，仍有浮点舍入差异，不能称为逐位相同。匹配的源标签相同。首个事件中存在率差异可由空间归一化积分变化解释；最早改变的标签并非直接落在该目标的 2 m 邻域内。因此，时间先后本身不能证明第 3 帧造成后来失效。','',
        '| 检出率模型 | 首次融合差异帧 | 最大存在率差 | 最大四维均值分量差 | 最大协方差元素差 | 首次局部更新差异帧 |',
        '|---|---:|---:|---:|---:|---:|']
    for mode in ['nominal','range']:
        origin=next(x for x in trace['origins'] if x['mode']==mode and x['condition']=='intermittent');first=origin['first']['fused']
        lines.append(f"| {mode} | {first['frame']} | {first['maximum_existence_difference']:.8f} | {first['maximum_mean_difference']:.6f} | {first['maximum_covariance_difference']:.6f} | {origin['first']['local']['frame']} |")
    lines+=['','均值列是位置与速度组成的四维状态中的最大分量差，不应解释为纯位置误差或统一的米数。可靠链路的首个事件数值与对应间歇链路相同。','',
        '## 冻结的原生干预','',
        '仅对 range 检出率、间歇链路、`v2xt_0001` 运行。三条干预轨迹均从 GS 开始，在第 3 帧每个实际接收到的融合处，对全部输出标签计算同一组执行输入下的 GCE 候选。分别替换存在率、完整空间高斯分布，或两者；第 4 帧恢复 GS。没有用真值、目标邻域或评分选择标签。','',
        '两条未修改的 GCE/GS 控制轨迹先重跑，所有原有共同记录字段（运行耗时除外）与已审核旧轨迹逐项精确相同。三个干预此前的递推，以及第 3 帧局部更新和发包记录，与重跑 GS 精确相同。独立检查重建全序列的局部概率、空间自然参数、曲率准入、归一化、标签匹配、发包和 MAP 提取；干预日志逐标签核对旧值、GCE 候选与实际应用值。','',
        '## 所有结果','',
        'OSPA/GOSPA 为整段 240 帧、两机器人均值，越低越好。目标检出以最终裁剪输出与所有真值执行 2 m 门限的一对一分配，分母为目标存在的机器人帧数。payload 为实际尝试发送的未填充字节；wire 含 16384 字节分块和每帧 256 字节控制量。','',
        '| 轨迹 | OSPA | GOSPA | 目标检出：53–122 帧 | 目标检出：全段 | payload 字节 | wire 字节 |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for arm in ORDER:
        r=rows[arm];w=diagnostics[arm]['target']['windows']
        lines.append(f"| {LABELS[arm]} | {r['ospa']:.6f} | {r['gospa']:.6f} | {w['original_window']['detected']}/{w['original_window']['robot_frames']} | {w['full']['detected']}/{w['full']['robot_frames']} | {int(r['raw_bytes']):,} | {int(r['wire_bytes']):,} |")
    lines+=['','| 轨迹 | 定位平方代价均值 | 漏检平方代价均值 | 误检平方代价均值 | 基数绝对误差均值 | 窗口内目标附近候选峰值 | 窗口内全部候选峰值 |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for arm in ORDER:
        r=rows[arm];w=diagnostics[arm]['target']['windows']['original_window']
        lines.append(f"| {LABELS[arm]} | {r['loc2']:.4f} | {r['miss2']:.4f} | {r['false2']:.4f} | {r['countError']:.4f} | {w['maximum_near_components']} | {w['maximum_components']} |")
    lines+=['','候选计数指存在率大于 0.001 的内部 Bernoulli 标签，不能当成最终误检数。所有逐帧记录见两份 `target_frames_recursion_*.csv`，完整分数见 `ALL_INTERVENTION_SCORES.csv`。','',
        '| 单次替换相对持续 GS | 原窗口检出次数变化 | 全段检出次数变化 | OSPA 变化 | wire 变化 |',
        '|---|---:|---:|---:|---:|']
    for e in effects:lines.append(f"| {LABELS[e['arm']]} | {e['window_detection_delta']:+d} | {e['full_detection_delta']:+d} | {e['ospa_change_percent']:+.4f}% | {e['wire_byte_change_percent']:+.4f}% |")
    lines+=['','第 53 帧、机器人 1 的同一预定检查点：有效标签数由距离真值最近的当前检测列的 `r × W` 归一化熵计算。检测与真值的距离同时列出，不能把归一化列质量直接当作校准后的检出概率。','',
        '| 轨迹 | 目标附近候选数 | 最大存在率 | 最近检测距离 m | 有效标签数 | 最大归一化份额 |',
        '|---|---:|---:|---:|---:|---:|']
    fmt=lambda v:'—' if v is None else f'{v:.6f}'
    for arm in ORDER:
        r=next(x for x in diagnostics[arm]['target']['frame53'] if x['robot']==1)
        lines.append(f"| {LABELS[arm]} | {r['near_active_components']} | {r['near_max_r']:.6f} | {fmt(r['association_nearest_distance_m'])} | {fmt(r['association_effective_labels'])} | {fmt(r['association_largest_share'])} |")
    lines+=['','## 解释边界','']
    if result['any_window_improvement']:
        lines+=['至少一种预先声明的单次替换提高了该目标在原窗口内的检出次数。这支持第 3 帧的状态变化可以改变后续轨迹；仍须同时看全段漏检、误检与通信代价，不能据此断言初次事件是唯一原因。']
    else:
        lines+=['三种单次替换均未提高该目标在原窗口内的检出次数。第 3 帧是最早可测的分化点，但本实验没有证明只修正这一帧足以恢复后续检出。按冻结协议结束本次实验，不扩大时间窗口或继续搜索起始帧。']
    lines+=['','本实验只涉及一个已经看过结果的诊断案例，而且是 range 检出率下 GS 的失效。它不能直接证明原 nominal GCE 输给 No-age 的全部原因，也不能支撑一般性能改进或论文方法升级。单次替换前的已审核轨迹、失败的共享模型筛选和联合准入筛选均保留。','',
        '5 条新轨迹均须真实退出成功并完成审核；共核对 2400 个新机器人帧和 480 个复用 No-age 机器人帧。原点追踪另已独立核对 5760 个目标机器人帧。最终文件与来源完整性见 `FINAL_VERIFICATION.json`。','']
    report=OUT/'RESULTS_CN.md';report.write_text('\n'.join(lines))
    receipt=dict(passed=True,source_sha256=sha(Path(__file__)),results_sha256=sha(destination),report_sha256=sha(report),
        csv_sha256=sha(OUT/'ALL_INTERVENTION_SCORES.csv'))
    (OUT/'REPORT_BUILD.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('RECURSION REPORT BUILT',json.dumps(effects),flush=True)

if __name__=='__main__':main()
