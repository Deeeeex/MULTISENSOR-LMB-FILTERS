"""Check the full preserved evidence and present all fixed decisions."""
from collections import Counter
from pathlib import Path
import csv
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
NAMES={'original':'原 GCE 负准入','peer_conditional':'对端条件关联','peer_joint':'对端联合检出（主）','no_negative':'取消额外负项'}
DATA={'v2v_development':'V2V4Real 开发 9 段','v2x_val':'V2X-Real 已见 5 段'}
LINK={'reliable':'可靠','intermittent':'间歇'}
MODE={'original':'原实现','local':'接收端本地裁剪信息','shared':'双方当前裁剪信息'}

def main():
    destination=OUT/'FINAL_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    report=json.loads((OUT/'SCREEN_RESULTS.json').read_text())
    verification=json.loads((OUT/'SCREEN_VERIFICATION.json').read_text())
    trace=json.loads((OUT/'MOTIVATING_TRACE.json').read_text())
    execution=json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    preflight=json.loads((OUT/'PREFLIGHT.json').read_text())
    assert all(v['passed'] for v in [report,verification,trace,preflight])
    assert execution['completed'] and execution['returncode']==0
    assert execution['source_sha256']==sha(OUT/'execute_screen.py')
    freeze_sha=sha(OUT/'SCREEN_FREEZE.json')
    assert report['freeze_sha256']==verification['freeze_sha256']==trace['freeze_sha256']==execution['freeze_sha256']==freeze_sha
    assert report['source_sha256']==cfg['source_sha256']
    assert verification['report_sha256']==sha(OUT/'SCREEN_RESULTS.json')
    assert verification['verifier_sha256']==sha(OUT/'verify_screen.py')
    assert verification['execution_sha256']==sha(OUT/'SCREEN_EXECUTION.json')
    assert verification['csv_sha256']==sha(OUT/'ALL_SCREEN_SCORES.csv')
    assert trace['source_sha256']==sha(OUT/'trace_motivation.py')
    assert trace['table_sha256']==sha(OUT/'MOTIVATING_TRACE.csv')
    assert trace['input_sha256']==cfg['motivation_inputs'] and len(trace['input_sha256'])==12
    assert report['source_runs']==verification['source_runs']==len(cfg['cells'])==56
    assert report['robot_frames']==verification['alternate_robot_frames']==83584
    assert report['original_parity_robot_frames']*4==report['robot_frames']
    assert report['native_runs']==0 and report['fixed_input_only']
    assert len(report['rows'])==verification['rows']==224 and len(report['artifacts'])==112
    assert len({c['sequence'] for c in cfg['cells']})==14
    assert not any(c['sequence']=='v2xt_0001' for c in cfg['cells'])
    gates=[]
    for dataset in DATA:
        for condition in LINK:
            for metric in ['ospa','gospa']:
                values={}
                for rule in ['peer_joint','original']:
                    group=[r for r in report['rows'] if (r['dataset'],r['condition'],r['backend'],r['rule'])==(dataset,condition,'GCE',rule)]
                    assert len(group)==(9 if dataset=='v2v_development' else 5)
                    values[rule]=math.fsum(r[metric] for r in group)/len(group)
                gate=next(g for g in report['gates'] if (g['dataset'],g['condition'],g['metric'])==(dataset,condition,metric))
                independent=next(g for g in verification['gates'] if (g['dataset'],g['condition'],g['metric'])==(dataset,condition,metric))
                assert abs(gate['difference']-(values['peer_joint']-values['original']))<1e-12
                assert gate['passed']==independent['passed']==(values['peer_joint']<values['original'] if metric=='ospa' else values['peer_joint']<=values['original'])
                gates.append(gate)
    advance=all(g['passed'] for g in gates)
    assert report['advance_to_recursion']==verification['advance_to_recursion']==advance
    with (OUT/'MOTIVATING_TRACE.csv').open(newline='') as stream:trace_rows=list(csv.DictReader(stream))
    assert len(trace_rows)==trace['checked_robot_frames']==5760
    assert len(trace['summaries'])==24 and len(trace['snapshots'])==24
    for summary in trace['summaries']:
        chosen=[r for r in trace_rows if (r['condition'],r['mode'],r['backend'])==(summary['condition'],summary['mode'],summary['backend'])
            and (summary['window']=='full' or 53<=int(r['frame'])<=122)]
        assert len(chosen)==summary['robot_frames']
        assert Counter((int(r['frame']),int(r['robot'])) for r in chosen).most_common(1)[0][1]==1
        near=[r for r in chosen if r['root_near_target']=='True' and r['root_active']=='True']
        counts=dict(detected=sum(r['target_detected']=='True' for r in chosen),near_root_frames=len(near),
            near_root_misses=sum(r['root_extracted']=='False' for r in near),
            near_root_negative_extra=sum(float(r['extra_negative'])<0 for r in near),
            both_curvature_allowed_with_negative=sum(float(r['extra_negative'])<0 and r['both_curvature_allowed']=='True' for r in near),
            immediate_target_rescues=sum(r['target_detected']=='False' and r['without_negative_target_detected']=='True' for r in chosen))
        for key,value in counts.items():assert summary[key]==value,(summary,key)
    # The raw association arrays are unchanged from the prior arithmetic audit.
    arithmetic_path=OUT.parent/'icra_joint_admission/WEIGHT_ARITHMETIC_VERIFICATION.json'
    arithmetic=json.loads(arithmetic_path.read_text());assert arithmetic['passed']
    raw=[d for d in report['diagnostics'] if d['raw_weights']['available']]
    assert len(raw)==arithmetic['source_runs']==18
    assert sum(d['raw_weights']['local_rows_with_W'] for d in raw)==arithmetic['checked_rows']==407972
    for d in raw:
        cell=d['cell'];assert arithmetic['inputs'][cell['path']]==cell['sha256']
        assert arithmetic['inputs'][cell['ratios_path']]==cfg['source_sha256'][cell['ratios_path']]
        old=next(r for r in arithmetic['checks'] if (r['sequence'],r['condition'],r['backend'])==(cell['sequence'],cell['condition'],cell['backend']))
        assert old['checked_rows']==d['raw_weights']['local_rows_with_W']
    log_path=ROOT/execution['log'];assert sha(log_path)==execution['log_sha256']
    log=log_path.read_text();assert log.count('SUBSTITUTION COMPLETE ')==56
    assert f'SCREEN COMPLETE advance {advance} robot frames 83584' in log
    passed=sum(g['passed'] for g in gates)
    verdict=(f'主规则通过 {passed}/8 项固定比较，允许另行冻结完整递推实验。' if advance else
             f'主规则只通过 {passed}/8 项固定比较；关闭这一个对端检出调节规则，不进入递推或拟合系数。')
    lines=['# 对端当前检出支持：额外负项的固定输入检验','',verdict,'',
        '唯一主规则是 `g_minus_new = g_minus_old × (1 − r_peer × a_peer)`。只采用当前、已表示且有正权重的对端来源。本地 pD=0.9 漏检更新、原正准入、历史权重和匹配保留；负指数同时作用于标量和高斯比值，并重算原曲率保护与不可积回退。该乘积是近似联合检出支持，不能解释为已校准的可见性或跨源独立概率。','',
        '以下均是在原方法已访问的输入上进行一次融合替换，结果没有进入下一帧。14 段数据此前均已用于开发；诊断案例 v2xt_0001 完全排除在选优输入之外。本轮没有新的完整递推，也没有新方法的物理字节成本或泛化结论。','',
        '## 原 GCE 状态上的全部规则','',
        '每段等权。两条链路分别计算，OSPA/GOSPA 单位为 m；所有规则均保留，次要规则不得替换失败主规则。','',
        '| 数据 | 链路 | 规则 | OSPA | GOSPA | 漏检平方代价 | 虚假平方代价 | 定位平方代价 | 输出数 |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for backend in ['GCE','Guarded Scalar']:
        if backend!='GCE':
            lines+=['','## Guarded Scalar 自身状态上的对照','',
                '这些输入来自 Guarded Scalar 自己的原递推，用于检查输入状态依赖，不参与主规则晋级，也不能作为同一输入上的 GCE/Scalar 因果比较。','',
                '| 数据 | 链路 | 规则 | OSPA | GOSPA | 漏检平方代价 | 虚假平方代价 | 定位平方代价 | 输出数 |',
                '|---|---|---|---:|---:|---:|---:|---:|---:|']
        for dataset in DATA:
            for condition in LINK:
                for rule in cfg['rules']:
                    row=next(r for r in report['aggregate'] if (r['dataset'],r['backend'],r['rule'])==(dataset,backend,rule))['conditions'][condition]
                    lines.append(f"| {DATA[dataset]} | {LINK[condition]} | {NAMES[rule]} | {row['ospa']:.9f} | {row['gospa']:.9f} | {row['miss2']:.6f} | {row['false2']:.6f} | {row['loc2']:.6f} | {row['outputCount']:.6f} |")
    lines+=['','## 八项预先固定的晋级比较','',
        '差值为主规则减原 GCE。每个数据/链路组合要求 OSPA 严格下降且 GOSPA 不增加；不跨链路抵消。','',
        '| 数据 | 链路 | 指标 | 原值 | 主规则 | 差值 | 通过 |','|---|---|---|---:|---:|---:|---|']
    for gate in gates:
        lines.append(f"| {DATA[gate['dataset']]} | {LINK[gate['condition']]} | {gate['metric'].upper()} | {gate['reference_mean']:.9f} | {gate['candidate_mean']:.9f} | {gate['difference']:+.9f} | {'是' if gate['passed'] else '否'} |")
    lines+=['','## 已见诊断标签的完整追踪（不参与筛选）','',
        '追踪 v2xt_0001 的既定标签 (3, 100004)，保留其全部 240 帧、两个机器人、两种链路及三个既有上下文。标签不被假定为始终代表同一个物体；“近目标”要求当前距既定真值目标 5 不超过 2 m。每次只移除这一标签已接受的额外负标量贡献，其他密度和标签保持当前输入。即时恢复数量仅描述这种单步替换，不代表递推可恢复的数量。','',
        '| 链路 | 上下文 | 方法 | 范围 | 机器人帧 | 实际检出 | 近目标活跃标签帧 | 此标签未提取 | 有额外负项 | 负项且两侧曲率允许 | 单步移除负项后即时恢复 |',
        '|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for s in trace['summaries']:
        lines.append(f"| {LINK[s['condition']]} | {MODE[s['mode']]} | {s['backend']} | {'完整' if s['window']=='full' else '既定 53–122'} | {s['robot_frames']} | {s['detected']} | {s['near_root_frames']} | {s['near_root_misses']} | {s['near_root_negative_extra']} | {s['both_curvature_allowed_with_negative']} | {s['immediate_target_rescues']} |")
    lines+=['','## 核验与限制','',
        f"56 个源运行全部完成，四种规则共 {report['robot_frames']:,} 个机器人帧、224 行序列评分；原分布和输出在 {report['original_parity_robot_frames']:,} 个机器人帧复现。生产计算从本地预测和后验矩开始，独立验证从归一化融合密度减旧传输残差、再加新残差开始，重建 {verification['fusion_distributions']:,} 个分布，并以另一套基数递推及指派评分核对所有输出。这是同一工作流程内的独立实现复算。",'',
        '原始关联 W 在 18 个源运行中可用，其余 38 个较早运行只保存了已审计的关联质量。现有逐元素算术核验覆盖 407,972 条记录；本轮重新核对其输入散列与完整记录数，沿用该核验，未将其计为新实验。近似关联的联合列和不作裁剪，也不宣称满足精确互斥匹配。六个精确指派穷举和七个端点/来源可用性夹具另行通过。','',
        '已失败的正项增强、取消负项完整递推、联合正准入、曲率冲突、范围校准与裁剪信息规则仍保持关闭。新筛选不能用有利单帧、次要规则、某一链路或某一来源方法替代原定八项判据。若全部通过，也须另行冻结全 14 段递推，加入匹配的 No-age/Scalar 对照和实际序列化字段成本，才可讨论方法收益。','',
        '完整逐段表见 [ALL_SCREEN_SCORES.csv](ALL_SCREEN_SCORES.csv)，诊断逐帧表见 [MOTIVATING_TRACE.csv](MOTIVATING_TRACE.csv)，独立分布核验见 [SCREEN_VERIFICATION.json](SCREEN_VERIFICATION.json)，固定判据见 [PROTOCOL.md](PROTOCOL.md)。','']
    report_md=OUT/'RESULTS_CN.md';assert not report_md.exists();report_md.write_text('\n'.join(lines))
    sources=cfg['source_sha256'].copy();sources.update(report['artifacts']);sources.update(trace['input_sha256'])
    sources.update(verification['inputs']);sources.update(arithmetic['inputs'])
    sources[str(log_path.relative_to(ROOT))]=sha(log_path)
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in OUT.iterdir() if p.is_file()})
    for path in (ROOT/'RUN/ICRA_PEER_DETECTION').glob('*.log'):sources[str(path.relative_to(ROOT))]=sha(path)
    for name,digest in sources.items():assert sha(ROOT/name)==digest,name
    result=dict(passed=True,protected_files=len(sources),source_runs=56,new_recursive_runs=0,
        fixed_input_robot_frames=83584,original_parity_robot_frames=20896,
        fusion_distributions=verification['fusion_distributions'],exploratory_trace_robot_frames=5760,
        reused_raw_weight_verification_rows=arithmetic['checked_rows'],gate_count=8,gate_passes=passed,
        advance_to_recursion=advance,source_sha256=sources,verifier_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('PEER DETECTION FINAL VERIFIED',len(sources),'files;',verdict,flush=True)

if __name__=='__main__':main()
