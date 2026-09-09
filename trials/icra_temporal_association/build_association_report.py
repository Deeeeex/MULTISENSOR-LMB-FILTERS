"""Render the audited results and current assessment state for review."""
from pathlib import Path
import hashlib
import json

from identity_metrics import pooled

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE='marked_gaussian_evidence'
GS=BASE+'_guarded_scalar'
LABELS={BASE:'原 GCE',GS:'Guarded Scalar','marked_lineage':'No-age KLA',
    BASE+'_assoc_direct':'D：当前观测匹配',BASE+'_assoc_temporal':'T：三帧观测匹配',
    BASE+'_assoc_reopen':'R：仅重开冲突关联',BASE+'_assoc_split':'S：重开并保留分支',
    GS+'_assoc_split':'Guarded Scalar + S',
    BASE+'_assoc_quality':'Q：提高置信度门槛',BASE+'_assoc_nis':'N：提高冲突门槛',
    BASE+'_assoc_quality_nis':'QN：双门槛',GS+'_assoc_quality_nis':'Guarded Scalar + QN'}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    dev_path=OUT/'SCREENED_DEVELOPMENT_SELECTION.json'
    dev=json.loads(dev_path.read_text());assert dev['passed']
    full_path=OUT/'RESTORED_ASSESSMENT.json'
    full=json.loads(full_path.read_text());assert full['passed']
    selected=dev['selected']['arm'] if dev['advance'] else None
    conditions=['reliable','intermittent']
    baseline={r['condition']:r for r in dev['aggregate'] if r['arm']==BASE}
    chosen={r['condition']:r for r in dev['aggregate'] if r['arm']==selected}
    text=['# 跨车关联与时序证据：当前实验结果','']
    if selected:
        gains=[100*(baseline[c]['ospa']-chosen[c]['ospa'])/baseline[c]['ospa'] for c in conditions]
        text += [f"{LABELS[selected]}通过开发门槛：可靠／间歇链路的 OSPA 分别改善 **{gains[0]:.2f}%／{gains[1]:.2f}%**。参数已固定，继续核验完整发布数据和新增测试。",'']
    else:
        text += ['三个严格版本均未通过开发门槛；新增测试保持未评估状态。','']
    text += ['## 九段开发集','',
        '共 1,993 对双车帧。每个版本执行完整递归；检测器、校准、局部更新、运动模型和链路随机数一致。OSPA 为片段等权均值，单位为米。',
        '', '| 版本 | 可靠 OSPA | 间歇 OSPA | 已评分关联错误数／总数（2 m） |',
        '| --- | ---: | ---: | ---: |']
    arms=[BASE]+[BASE+'_assoc_'+m for m in ['direct','temporal','reopen','split','quality','nis','quality_nis']]
    for arm in arms:
        values={r['condition']:r['ospa'] for r in dev['aggregate'] if r['arm']==arm}
        identity=pooled([r for r in dev['identity_rows'] if r['arm']==arm])['2']
        text.append(f"| {LABELS[arm]} | {values['reliable']:.6f} | {values['intermittent']:.6f} | {identity['wrong_pairs']:,}／{identity['scored_pairs']:,}（{100*identity['wrong_pair_rate']:.3f}%） |")
    text += ['', 'Q/N/QN 的选择门槛是在两种链路下均降低 OSPA 至少 1%，且合并后的已评分关联错误率不升；合格者按两种链路的平均 OSPA 选择。全部早期有效版本保留在表中。',
        '', '关联诊断使用独立的 2 m 一对一标注中心分配。仅当两个输入都能评分时，才判定该关联对正确或错误；未评分数量、12 m 结果和共同目标漏配另见 JSON。标注不进入跟踪算法。',
        '', '## 原始拆分版 S 的完整评估','',
        '43 个去重 V2V 片段、9,699 对帧，按 17 个原始记录分组；新增 S 和同一关联规则下的 Guarded Scalar 对照均已完成独立核验。',
        '', '| 版本 | 可靠 OSPA | 间歇 OSPA |', '| --- | ---: | ---: |']
    for arm in ['marked_lineage',GS,BASE,GS+'_assoc_split',BASE+'_assoc_split']:
        values={r['condition']:r['sequence_macro']['ospa'] for r in full['aggregate'] if r['scope']=='v2v_all' and r['arm']==arm}
        text.append(f"| {LABELS[arm]} | {values['reliable']:.6f} | {values['intermittent']:.6f} |")
    text += ['', '| S 相对原 GCE | 可靠链路 | 间歇链路 |', '| --- | ---: | ---: |']
    groups={(r['arm'],r['condition']):r for r in full['aggregate'] if r['scope']=='v2v_all' and r['arm'] in [BASE,BASE+'_assoc_split']}
    pairs={r['condition']:r for r in full['paired_recording'] if r['scope']=='v2v_all' and r['candidate']==BASE+'_assoc_split' and r['reference']==BASE}
    gain=[];wire=[];raw=[];identity=[];changes=[];intervals=[]
    for c in conditions:
        a,b=groups[BASE,c],groups[BASE+'_assoc_split',c]
        gain.append(f"{100*(a['sequence_macro']['ospa']-b['sequence_macro']['ospa'])/a['sequence_macro']['ospa']:.2f}%")
        wire.append(f"+{100*(b['communication']['wire_bytes']/a['communication']['wire_bytes']-1):.2f}%")
        raw.append(f"+{100*(b['communication']['raw_payload_bytes']/a['communication']['raw_payload_bytes']-1):.2f}%")
        identity.append(f"{100*a['identity']['2']['wrong_pair_rate']:.3f}% → {100*b['identity']['2']['wrong_pair_rate']:.3f}%")
        changes.append(f"{a['identity']['2']['counts']['truth_label_switches']} → {b['identity']['2']['counts']['truth_label_switches']}")
        p=pairs[c];intervals.append(f"{p['recording_macro_difference']:+.4f} [{p['low']:+.4f}, {p['high']:+.4f}]")
    for label,values in [('OSPA 相对改善',gain),('真实 wire 字节',wire),('原始 payload 字节',raw),
                         ('已评分关联错误率',identity),('相邻帧同一标注目标的输出标签变化',changes),
                         ('记录等权 OSPA 差及 95% 重采样区间（m）',intervals)]:
        text.append(f'| {label} | {values[0]} | {values[1]} |')
    text += ['', '两种链路下均有 10 段改善；可靠链路另有 8 段变差、25 段不变，间歇链路有 7 段变差、26 段不变。上述区间按整条记录配对重采样 10,000 次，种子 8301。相邻标签变化是本实验的诊断量。',
        '', '已曝光的五个外部 V2X 验证片段上，S 的可靠 OSPA 为 8.408140（原 GCE 为 8.408389），间歇 OSPA 为 8.346338（原 GCE 为 8.338196）。完整结果包含每段及按采集日期分组的均值。',
        '', '## 固定版本的后续验证','']
    for stage,label,total in [('association_screen_selected_controls_development','九段开发集的 Guarded Scalar 同关联对照',9),
        ('association_screen_selected_v2v','其余 34 段的固定版本及同关联对照',34),
        ('association_screen_selected_v2x','五段外部验证数据',5),
        ('association_screen_selected_test','新增 14 段测试数据',14)]:
        audit=OUT/('audit_'+stage+'.json');runtime=OUT/('runtime_'+stage+'.json')
        if audit.exists():
            state=json.loads(audit.read_text());assert state['passed'];status='完整独立核验通过'
        elif runtime.exists():
            runs=json.loads(runtime.read_text());n=sum(r['returncode']==0 and r['completion_line'] for r in runs)
            status=f'原生运行完成 {n}/{total} 段，待完整核验'
        else:
            status='尚未完成跟踪运行'
        text.append(f'- {label}：{status}。')
    text += ['', '新增测试是预先登记的全部 14 个双车片段，共 2,172 对帧、五个采集日期；相关片段按日期分组。处理与推理沿用既有 V2X 适配器和冻结的检测器。选择完成后开始下载，参数不会根据测试结果调整。',
        '', '## 复核入口','',
        '- `SCREENED_DEVELOPMENT_SELECTION.json`：八版本开发比较及固定版本选择。',
        '- `RESTORED_ASSESSMENT.json`：S 的全量和外部验证比较，874 行结果。',
        '- `PERSISTENT_METHOD_SPEC.md`、`CANDIDATES_V3.md`：具体规则及门槛。',
        '- `README.md`、`PATH_RESTORATION.md`：有效结果入口和已撤回的早期执行记录。',
        '', '早期路径遮蔽错误产生的 50 份轨迹及旧 V1 选择报告未用于上述比较。No-age 预检的物理轨迹逐项一致；两个不参与 No-age 更新的旧 ceiling 诊断列差异见 `BENCHMARK_DIAGNOSTIC_PARITY.json`。', '']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(text))
    receipt=dict(inputs={str(p.relative_to(ROOT)):sha(p) for p in [dev_path,full_path]},generator_sha256=sha(Path(__file__)),
        output_sha256=sha(OUT/'RESULTS_CN.md'))
    (OUT/'RESULTS_RENDER.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('ASSOCIATION RESULTS REPORT UPDATED')


if __name__=='__main__':main()
