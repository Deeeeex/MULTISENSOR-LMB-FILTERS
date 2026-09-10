"""Verify all declared inputs, boundary checks, scores and fixed decisions."""
from pathlib import Path
import csv
import hashlib
import itertools
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
DATA={'v2v_development':'V2V4Real 开发 9 段','v2x_val':'V2X-Real 已见 5 段'}
LINK={'reliable':'可靠','intermittent':'间歇'}
NAMES={'original':'原规则','nonreversal':'双向约束（主）','negative_reversal':'仅负向反转','positive_reversal':'仅正向反转'}

def main():
    final=OUT/'FINAL_VERIFICATION.json';assert not final.exists()
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text());report=json.loads((OUT/'SCREEN_RESULTS.json').read_text())
    verification=json.loads((OUT/'SCREEN_VERIFICATION.json').read_text());execution=json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    trace=json.loads((OUT/'REFERENCE_TRACE.json').read_text());preflight=json.loads((OUT/'PREFLIGHT.json').read_text())
    assert all(x['passed'] for x in [report,verification,trace,preflight])
    assert cfg['direction_tolerance']==1e-7 and cfg['bisections']==60 and cfg['primary']=='nonreversal'
    freeze=sha(OUT/'SCREEN_FREEZE.json')
    assert report['freeze_sha256']==verification['freeze_sha256']==execution['freeze_sha256']==trace['freeze_sha256']==preflight['freeze_sha256']==freeze
    assert report['source_sha256']==cfg['source_sha256']
    assert execution['completed'] and execution['returncode']==0
    assert execution['source_sha256']==sha(OUT/'execute_screen.py') and execution['preflight_sha256']==sha(OUT/'PREFLIGHT.json')
    assert preflight['source_sha256']==sha(OUT/'preflight.py')
    assert verification['report_sha256']==sha(OUT/'SCREEN_RESULTS.json')
    assert verification['verifier_sha256']==sha(OUT/'verify_screen.py')
    assert verification['execution_sha256']==sha(OUT/'SCREEN_EXECUTION.json')
    assert verification['csv_sha256']==sha(OUT/'ALL_SCREEN_SCORES.csv')
    assert trace['source_sha256']==sha(OUT/'trace_reference.py') and trace['table_sha256']==sha(OUT/'REFERENCE_TRACE.csv')
    assert trace['source_table_sha256']==sha(OUT.parent/'icra_peer_detection/MOTIVATING_TRACE.csv')
    assert trace['native_inputs']==cfg['reference_inputs']
    assert report['source_runs']==verification['source_runs']==len(cfg['cells'])==56
    assert report['robot_frames']==verification['alternate_robot_frames']==83584
    assert report['original_parity_robot_frames']==20896 and report['native_runs']==0 and report['fixed_input_only']
    assert len(report['rows'])==verification['rows']==224 and len(report['artifacts'])==112
    assert not any(c['sequence']=='v2xt_0001' for c in cfg['cells'])
    assert len({c['sequence'] for c in cfg['cells']})==14
    gates=[]
    for dataset in DATA:
        for condition in LINK:
            for metric in ['ospa','gospa']:
                values={}
                for rule in ['original','nonreversal']:
                    group=[r for r in report['rows'] if (r['dataset'],r['condition'],r['backend'],r['rule'])==(dataset,condition,'GCE',rule)]
                    assert len(group)==(9 if dataset=='v2v_development' else 5)
                    values[rule]=math.fsum(r[metric] for r in group)/len(group)
                gate=next(g for g in report['gates'] if (g['dataset'],g['condition'],g['metric'])==(dataset,condition,metric))
                v=next(g for g in verification['gates'] if (g['dataset'],g['condition'],g['metric'])==(dataset,condition,metric))
                expected=values['nonreversal']<values['original'] if metric=='ospa' else values['nonreversal']<=values['original']
                assert expected==gate['passed']==v['passed']
                assert abs(gate['difference']-(values['nonreversal']-values['original']))<1e-12
                gates.append(gate)
    advance=all(g['passed'] for g in gates)
    assert report['advance_to_recursion']==verification['advance_to_recursion']==advance
    with (OUT/'REFERENCE_TRACE.csv').open(newline='') as stream:reference_rows=list(csv.DictReader(stream))
    assert len(reference_rows)==trace['robot_frames']==960
    for summary in trace['groups']:
        rows=[r for r in reference_rows if (r['condition'],int(r['robot']))==(summary['condition'],summary['robot'])]
        assert [int(r['frame']) for r in rows]==list(range(1,241))
        for direction in ['negative','positive']:
            observed=[]
            for row in rows:
                d0=None if row['base_direction']=='' else float(row['base_direction'])
                d1=None if row['actual_direction']=='' else float(row['actual_direction'])
                hit=False if d0 is None else (d0>1e-7 and d1<-1e-7 if direction=='negative' else d0<-1e-7 and d1>1e-7)
                assert hit==(row[direction+'_reversal']=='True');observed.append(hit)
            spans=[]
            for hit,group in itertools.groupby(enumerate(observed,1),key=lambda item:item[1]):
                values=list(group)
                if hit:spans.append([values[0][0],values[-1][0]])
            assert spans==summary[direction+'_spans'] and sum(observed)==summary[direction+'_reversals']
    old_arithmetic=OUT.parent/'icra_joint_admission/WEIGHT_ARITHMETIC_VERIFICATION.json'
    arithmetic=json.loads(old_arithmetic.read_text());assert arithmetic['passed']
    raw=[d for d in report['diagnostics'] if d['raw_weights']['available']]
    assert len(raw)==arithmetic['source_runs']==18
    assert sum(d['raw_weights']['local_rows_with_W'] for d in raw)==arithmetic['checked_rows']==407972
    for d in raw:
        c=d['cell'];assert arithmetic['inputs'][c['path']]==c['sha256']
        assert arithmetic['inputs'][c['ratios_path']]==cfg['source_sha256'][c['ratios_path']]
    log_path=ROOT/execution['log'];assert sha(log_path)==execution['log_sha256'];log=log_path.read_text()
    assert log.count('SUBSTITUTION COMPLETE ')==56 and f'SCREEN COMPLETE advance {advance} robot frames 83584' in log
    warnings=log.count('RuntimeWarning:')
    passed=sum(g['passed'] for g in gates)
    verdict=(f'主规则通过 {passed}/8 项固定比较，允许另行冻结全 14 段原生递推。' if advance else
             f'主规则仅通过 {passed}/8 项固定比较，关闭该方向约束，不选择单向控制或调整边界继续筛选。')
    lines=['# 合并当前证据的方向约束：完整固定输入筛选','',verdict,'',
        '参考量使用相同 alpha、beta 对本地预测分布融合后的存在率。先计算原空间池相对该参考的当前更新方向，再检查原 GCE 额外 ratio 是否使该方向反转。发生反转时，沿整份已接纳 ratio 的共同指数缩放路径求边界；存在率和高斯因子一起变化，原曲率拒绝仍然有效。','',
        '这是一个经验约束：原合并方向本身也可能错误。主规则对两个方向对称，两个单向控制只用于解释机制。数值死区 1e−7 和 60 次二分在任何真实输入替换前固定。','',
        '全部 14 段均为已见开发输入；56 条源轨迹来自 GCE 和 Guarded Scalar 各自的原递推。替换输出没有进入下一帧，因此以下不代表新方法的完整递推性能。诊断案例 v2xt_0001 不参与筛选。','',
        '## 全部规则和源状态','',
        '每段等权、链路分开。OSPA/GOSPA 单位为 m，定位/漏检/虚假项为 GOSPA 平方代价。两个后端的输入状态不同，其表间差异不能作为同一输入上的空间校正归因。','',
        '| 来源 | 数据 | 链路 | 规则 | OSPA | GOSPA | 定位 | 漏检 | 虚假 | 输出数 |',
        '|---|---|---|---|---:|---:|---:|---:|---:|---:|']
    for backend in ['GCE','Guarded Scalar']:
        for dataset in DATA:
            for condition in LINK:
                for rule in cfg['rules']:
                    row=next(r for r in report['aggregate'] if (r['dataset'],r['backend'],r['rule'])==(dataset,backend,rule))['conditions'][condition]
                    lines.append(f"| {backend} | {DATA[dataset]} | {LINK[condition]} | {NAMES[rule]} | {row['ospa']:.9f} | {row['gospa']:.9f} | {row['loc2']:.6f} | {row['miss2']:.6f} | {row['false2']:.6f} | {row['outputCount']:.6f} |")
    lines+=['','## 八项固定比较','',
        '差值为主规则减原 GCE。每个数据/链路组合要求 OSPA 严格下降、GOSPA 不增加。','',
        '| 数据 | 链路 | 指标 | 原 GCE | 主规则 | 差值 | 通过 |','|---|---|---|---:|---:|---:|---|']
    for g in gates:
        lines.append(f"| {DATA[g['dataset']]} | {LINK[g['condition']]} | {g['metric'].upper()} | {g['reference_mean']:.9f} | {g['candidate_mean']:.9f} | {g['difference']:+.9f} | {'是' if g['passed'] else '否'} |")
    lines+=['','## 被限制的方向反转','',
        '以下是原始融合标签事件数，同一标签会跨时刻重复；它们不是独立样本或真实目标数量。','',
        '| 来源 | 数据 | 链路 | 融合标签 | 正更新变负 | 负更新变正 | 主规则改变标签 |',
        '|---|---|---|---:|---:|---:|---:|']
    for backend in ['GCE','Guarded Scalar']:
        for dataset in DATA:
            for condition in LINK:
                group=[next(v for v in d['rules'] if v['rule']=='nonreversal') for d in report['diagnostics'] if (d['cell']['backend'],d['cell']['dataset'],d['cell']['condition'])==(backend,dataset,condition)]
                total=lambda key:sum(d[key] for d in group)
                lines.append(f"| {backend} | {DATA[dataset]} | {LINK[condition]} | {total('fusion_labels')} | {total('negative_reversals')} | {total('positive_reversals')} | {total('changed_labels')} |")
    lines+=['','## 既定诊断标签的原始方向轨迹','',
        '对既定标签 (3, 100004) 保留两种链路、两个机器人、全部 240 帧。这里仅分类原轨迹，不对该案例运行新方法或移动单次干预时刻。各原生来源的预测参考另以编码比值重构核对，逐帧结果见 REFERENCE_TRACE.csv；近真值标记沿用上一轮已验证的完整标签追踪。','',
        '| 链路 | 机器人 | 正更新变负次数 | 完整连续区间 | 负更新变正次数 | 完整连续区间 |',
        '|---|---:|---:|---|---:|---|']
    for s in trace['groups']:
        lines.append(f"| {LINK[s['condition']]} | {s['robot']} | {s['negative_reversals']} | {s['negative_spans']} | {s['positive_reversals']} | {s['positive_spans']} |")
    lines+=['','## 核验和边界','',
        f"原分布和输出在 20,896 个机器人帧复现。四规则的全部 83,584 个机器人帧、224 行分数和 {verification['fusion_distributions']:,} 个融合分布通过独立实现复算。生产端使用本地预测/后验矩和二分，验证端使用编码残差、Brent 求根、另一套高斯积分和基数/指派评分。四个非线性高斯夹具还以标量数值积分独立检查了归一化路径。",'',
        f"原始 W 在 18 个来源可用，其余 38 个较早运行只保留已审计的关联质量。既有逐元素算术核验的 407,972 条记录与本轮输入散列、记录数完全匹配，此处是复用核验。原始 W 审计日志保留 {warnings} 条 RuntimeWarning；它不生成方向约束的参数。有限性和输出一致性均通过，但 NumPy 警告的底层原因未确定。",'',
        '所有八项门槛都在真实替换前固定。任一失败即关闭该规则，不用某个方向、片段、链路或后端替代主规则。通过固定输入门槛也只允许下一步原生递推实验；它不证明通信成本、长期跟踪收益或未见录制上的稳定优势。先前各失败家庭及论文主方法保持原有记录。','',
        '逐段评分见 [ALL_SCREEN_SCORES.csv](ALL_SCREEN_SCORES.csv)，全部判据见 [PROTOCOL.md](PROTOCOL.md)，独立核验见 [SCREEN_VERIFICATION.json](SCREEN_VERIFICATION.json)，诊断方向轨迹见 [REFERENCE_TRACE.csv](REFERENCE_TRACE.csv)。','']
    md=OUT/'RESULTS_CN.md';assert not md.exists();md.write_text('\n'.join(lines))
    sources=cfg['source_sha256'].copy();sources.update(report['artifacts']);sources.update(verification['inputs']);sources.update(arithmetic['inputs'])
    for p in OUT.iterdir():
        if p.is_file():sources[str(p.relative_to(ROOT))]=sha(p)
    for p in (ROOT/'RUN/ICRA_DIRECTION_CONSTRAINT').glob('*.log'):sources[str(p.relative_to(ROOT))]=sha(p)
    for name,h in sources.items():assert sha(ROOT/name)==h,name
    result=dict(passed=True,protected_files=len(sources),source_runs=56,new_recursive_runs=0,
        fixed_input_robot_frames=83584,original_parity_robot_frames=20896,fusion_distributions=verification['fusion_distributions'],
        reference_trace_robot_frames=960,reference_source_distributions=sum(s['fusion_distributions'] for s in trace['source_checks']),
        reused_arithmetic_rows=407972,recorded_runtime_warnings=warnings,gate_count=8,gate_passes=passed,
        advance_to_recursion=advance,source_sha256=sources,verifier_sha256=sha(Path(__file__)))
    final.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('DIRECTION CONSTRAINT VERIFIED',len(sources),'files;',verdict,flush=True)

if __name__=='__main__':main()
