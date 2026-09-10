"""Verify whole native trajectories and apply the predeclared fair case gate."""
from pathlib import Path
import csv
import hashlib
import json
import sys

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_known_censor'
sys.path.insert(0,str(OLD))
from finish_v2 import read,verify_target,verify_events
sys.path.insert(0,str(OUT))
from reentry_audit import census
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
ARMS=['marked_gaussian_evidence','marked_gaussian_evidence_guarded_scalar','marked_lineage']
NAMES=dict(zip(ARMS,['GCE','Guarded Scalar','No-age']))

def main():
    assert not (OUT/'RESULTS.json').exists() and not (OUT/'FINAL_VERIFICATION.json').exists()
    freeze=json.loads((OUT/'FREEZE.json').read_text());assert freeze['passed'];hashes={};scores={};diags={};reentry={}
    targets=0;event_count=0;parity_count=0;prefix_count=0
    for stage in ['prune_info_controls','prune_info_shared']:
        cfgpath=OUT/'stages'/(stage+'.json');assert sha(cfgpath)==freeze['configurations'][str(cfgpath.relative_to(ROOT))]
        cfg=json.loads(cfgpath.read_text());auditpath=OUT/('audit_'+stage+'.json');audit=json.loads(auditpath.read_text())
        assert audit['passed'] and audit['audited_robot_frames']==2880 and len(audit['parity'])==6
        for name,h in {**cfg['source_sha256'],**audit['inputs'],**audit['source_sha256']}.items():assert sha(ROOT/name)==h,name;hashes[name]=h
        runtime=json.loads((OUT/('runtime_'+stage+'.json')).read_text())
        assert len(runtime)==2 and all(r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files']==3 for r in runtime)
        scores.update({(r['condition'],r['arm']):r for r in audit['rows']});diags.update({(d['condition'],d['arm']):d for d in audit['diagnostics']})
        with (OUT/('target_frames_'+stage+'.csv')).open(newline='') as stream:frames=list(csv.DictReader(stream))
        lookup={(r['condition'],r['arm'],int(r['frame']),int(r['robot'])):r for r in frames};assert len(lookup)==len(frames)==2880
        with (OUT/('population_frames_'+stage+'.csv')).open(newline='') as stream:population=list(csv.DictReader(stream))
        assert len(population)==2880
        for r in population:assert int(r['delta'])==int(r['births'])-int(r['local_pruned'])-int(r['local_lost_at_fusion'])+int(r['remote_only_retained'])
        for u in cfg['units']:
            c=u['execution_id']
            for arm in u['arms']:
                path=OUT/'results'/stage/f"{u['sequence']}_{c}_{arm}.json.gz";data=read(path);d=diags[c,arm]
                targets+=verify_target(data,{(t,n):lookup[c,arm,t,n] for t in range(1,241) for n in [1,2]},d['target'])
                eventpath=ROOT/d['reimport_event_path'];event_count+=verify_events(data,eventpath,d['population']['reimport_summaries'])
                reentrypath=path.with_name(path.name.removesuffix('.json.gz')+'_reentry.csv.gz')
                rr=census(data,eventpath,reentrypath);reentry[c,arm]=rr;hashes[str(reentrypath.relative_to(ROOT))]=sha(reentrypath)
                p=next(p for p in audit['parity'] if p['condition']==c and p['arm']==arm)
                if stage=='prune_info_controls':
                    ref=u['references'][arm];assert sha(ROOT/ref['path'])==ref['sha256'];old=read(ROOT/ref['path'])['runs']
                    assert set(p['exact_fields'])==set(old)-{'runtimeSeconds'}
                    for name in p['exact_fields']:assert old[name]==data['runs'][name]
                    previous=json.loads((OLD/'reentry/RESULTS.json').read_text())
                    for s in rr['summaries']:
                        r=next(r for r in previous['summaries'] if r['condition']==c and r['arm']==arm and r['window']==s['window'] and r['scope']==s['scope'])
                        for k in ['remote_only_retained','same_frame_pruned','first_local_arrival','return_after_gap','gap_counts']:assert r[k]==s[k],k
                    parity_count+=1
                else:assert len(p['exact_prefix_fields'])==18 and p['first_changed_frame']>=2;prefix_count+=1
                print('INDEPENDENT PRUNE INFORMATION VERIFIED',c,arm,rr['verified_events'],'events',flush=True)
    assert targets==5760 and parity_count==prefix_count==6 and len(scores)==12
    for receipt in [*json.loads((OUT/'PORTS.json').read_text()),json.loads((OUT/'AUDIT_PORT.json').read_text())]:
        source=ROOT/receipt['source'];target=ROOT/receipt['target'];assert sha(source)==receipt['source_sha256'];content=source.read_text()
        for p in receipt['changes']:assert content.count(p['before'])==1;content=content.replace(p['before'],p['after'])
        assert content==target.read_text() and sha(target)==receipt['target_sha256']
    previous=json.loads((OLD/'RESULTS.json').read_text());previous_reentry=json.loads((OLD/'reentry/RESULTS.json').read_text())
    for c in ['reliable','intermittent']:
        for a in ARMS:
            scores[c,a]=next(r for r in previous['rows'] if r['condition']==c and r['arm']==a)
            diags[c,a]=next(d for d in previous['diagnostics'] if d['condition']==c and d['arm']==a)
            summaries=[dict(r,recurrences=r['same_frame_pruned']+r['return_after_gap']) for r in previous_reentry['summaries'] if r['condition']==c and r['arm']==a]
            assert len(summaries)==4;reentry[c,a]=dict(reused_prior_audit=True,summaries=summaries)
    def recurring(c,a):return next(r['recurrences'] for r in reentry[c,a]['summaries'] if r['window']=='original_window' and r['scope']=='target_neighbourhood')
    effects=[];fair=[];gate=[];thresholds=cfg['expansion_gate']
    for c in ['reliable','intermittent']:
        for a in ARMS:
            old,new=scores[c,a+'_known_censor'],scores[c,a+'_prune_info'];ow=diags[c,a+'_known_censor']['target']['windows'];nw=diags[c,a+'_prune_info']['target']['windows']
            effects.append(dict(condition=c,backend=NAMES[a],ospa_improvement=1-new['ospa']/old['ospa'],gospa_improvement=1-new['gospa']/old['gospa'],
                window_detection_gain=nw['original_window']['detected']-ow['original_window']['detected'],full_detection_gain=nw['full']['detected']-ow['full']['detected'],
                window_recurrence_delta=recurring(c,a+'_prune_info')-recurring(c,a+'_known_censor'),wire_ratio=new['wire_bytes']/old['wire_bytes']))
        g=scores[c,ARMS[0]+'_prune_info'];n=scores[c,ARMS[2]+'_prune_info'];e=next(e for e in effects if e['condition']==c and e['backend']=='GCE')
        fair.append(dict(condition=c,shared_gce_minus_shared_noage_ospa=g['ospa']-n['ospa'],shared_gce_minus_shared_noage_gospa=g['gospa']-n['gospa'],
            shared_gce_minus_original_gce_ospa=g['ospa']-scores[c,ARMS[0]]['ospa'],shared_gce_minus_original_noage_ospa=g['ospa']-scores[c,ARMS[2]]['ospa'],
            shared_gce_minus_original_gce_gospa=g['gospa']-scores[c,ARMS[0]]['gospa'],shared_gce_minus_original_noage_gospa=g['gospa']-scores[c,ARMS[2]]['gospa']))
        gate.append(dict(condition=c,ospa_improves_at_least_one_percent=e['ospa_improvement']>=thresholds['minimum_ospa_improvement'],
            gospa_improves_at_least_one_percent=e['gospa_improvement']>=thresholds['minimum_gospa_improvement'],beats_shared_noage_ospa=g['ospa']<n['ospa'],
            beats_shared_noage_gospa=g['gospa']<n['gospa'],window_detection_improves=e['window_detection_gain']>0,
            full_detection_nonworsening=e['full_detection_gain']>=0,recurrences_decrease=e['window_recurrence_delta']<0,wire_within_cap=e['wire_ratio']<=thresholds['maximum_wire_ratio']))
    expand=all(v for row in gate for k,v in row.items() if k!='condition')
    order=[(c,a+s) for c in ['reliable','intermittent'] for a in ARMS for s in ['', '_known_censor','_prune_info']]
    result=dict(passed=True,scope='One previously exposed complete recursive causal case',new_native_runs=12,reused_original_references=6,
        native_robot_frames=5760,independently_verified_target_frames=targets,independently_verified_remote_only_events=event_count,
        rows=[scores[k] for k in order],diagnostics=[diags[k] for k in order],reentry=[dict(condition=c,arm=a,**reentry[c,a]) for c,a in order],
        effects=effects,fair_comparisons=fair,gate=gate,expand_current_pruning_information=expand)
    (OUT/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'ALL_SCORES.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(result['rows'][0]),lineterminator='\n');writer.writeheader();writer.writerows(result['rows'])
    lines=['# 传递本帧裁剪信息：完整递归因果检验','',
        '在已曝光的 v2xt_0001 全 240 帧上，六条新控制与上一轮接收端本地修正逐字段精确一致（运行时间除外）。六条通信干预使用同一份当前裁剪标量协议，分别覆盖 GCE、Guarded Scalar、No-age 与两条链路。另列六条已经审计的原始轨迹。','',
        '12 条新轨迹全部正常退出，5760 个机器人帧完成概率、高斯、匹配、预测与裁剪平衡、MAP 提取和评分核对。每包裁剪 trailer 的 uint8 字节独立按 little-endian float64 解码，并从发送方本帧更新记录完整重建；每个使用的远端标量均来自当帧实际送达的消息。消息头 32 字节、每个条目 32 字节全部计入尝试、送达及 16384 字节块填充成本。','',
        '下表窗口仍是第 53–122 帧、目标 ID 5；检出按所有真值的 2 m 一对一匹配计算。复现次数包括同帧裁剪后返回和跨预测空档返回，避免把一帧延迟误当作消除。它们是相关标签事件数，不是独立物理目标数。','',
        '| 链路 | 方法 | OSPA ↓ | GOSPA ↓ | 窗口检出 /140 | 全段检出 /480 | 窗口标签复现 | wire 字节 |','|---|---|---:|---:|---:|---:|---:|---:|']
    def title(a):
        base=a.removesuffix('_known_censor').removesuffix('_prune_info');return NAMES[base]+(' + 本地实际值' if a.endswith('_known_censor') else ' + 当前裁剪通信' if a.endswith('_prune_info') else ' 原始')
    for c,a in order:
        r=scores[c,a];w=diags[c,a]['target']['windows'];lines.append(f"| {c} | {title(a)} | {r['ospa']:.6f} | {r['gospa']:.6f} | {w['original_window']['detected']} | {w['full']['detected']} | {recurring(c,a):,} | {int(r['wire_bytes']):,} |")
    lines+=['','| 链路 | 方法 | OSPA 改善 % | GOSPA 改善 % | 窗口检出变化 | 全段检出变化 | 复现变化 | wire 变化 % |','|---|---|---:|---:|---:|---:|---:|---:|']
    for e in effects:lines.append(f"| {e['condition']} | {e['backend']} | {100*e['ospa_improvement']:+.3f} | {100*e['gospa_improvement']:+.3f} | {e['window_detection_gain']:+d} | {e['full_detection_gain']:+d} | {e['window_recurrence_delta']:+d} | {100*(e['wire_ratio']-1):+.3f} |")
    lines+=['','上述变化均相对于本轮精确复现的接收端本地修正控制。','',
        '| 链路 | 方法 | 尝试 payload | 送达 payload | 尝试 trailer | 送达 trailer | 发送裁剪条目 |','|---|---|---:|---:|---:|---:|---:|']
    for c,a in order:
        if not a.endswith('_prune_info'):continue
        r=scores[c,a];v=diags[c,a]['communication'];lines.append(f"| {c} | {title(a)} | {int(r['raw_bytes']):,} | {int(r['delivered_raw_bytes']):,} | {v['attempted_trailer_bytes']:,} | {v['delivered_trailer_bytes']:,} | {v['reported_pruned_rows']:,} |")
    lines+=['','公平比较：']
    for f in fair:lines.append(f"- {f['condition']}：GCE − 同信息 No-age 的 OSPA 差为 {f['shared_gce_minus_shared_noage_ospa']:+.6f}，GOSPA 差为 {f['shared_gce_minus_shared_noage_gospa']:+.6f}。负值有利于 GCE。")
    lines+=['','预先冻结的扩大验证门槛：']
    for row in gate:lines.append('- '+row['condition']+'：'+'；'.join(k+'='+('通过' if v else '未通过') for k,v in row.items() if k!='condition')+'。')
    lines+=['',('所有门槛通过，只支持另行冻结完整数据集验证。' if expand else '门槛未全部通过，本次当前裁剪标量通信规则按协议关闭；不调整阈值、延长保留期或改选方法补救。'),'',
        '这是一项已经曝光的单案例因果诊断，不是独立方法验证，也不能据此声称泛化或统计显著性。即使标签复现减少，也须同时查看最终检出、完整误差和通信成本。主论文与共享生产融合实现未改动。全部原始/本地/通信三组结果和 GOSPA 分解保存于 RESULTS.json、ALL_SCORES.csv 及逐帧记录中。','']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(lines))
    for p in OUT.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and '.next.' not in p.name:hashes[str(p.relative_to(ROOT))]=sha(p)
    (OUT/'FINAL_VERIFICATION.json').write_text(json.dumps(dict(passed=True,exact_receiver_only_controls=6,exact_causal_prefixes=6,
        new_native_runs=12,native_robot_frames=5760,independently_verified_target_frames=targets,
        independently_verified_remote_only_events=event_count,expand_current_pruning_information=expand,input_sha256=hashes),indent=2)+'\n')
    print('PRUNE INFORMATION FINAL VERIFIED',json.dumps(dict(expand=expand,effects=effects,fair=fair)),flush=True)

if __name__=='__main__':main()
