"""Recount target/reimport events independently and apply the frozen case gate."""
from collections import defaultdict
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
ARMS=['marked_gaussian_evidence','marked_gaussian_evidence_guarded_scalar','marked_lineage']
SUFFIX='_known_censor'
NAMES=dict(zip(ARMS,['GCE','Guarded Scalar','No-age']))

def read(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)

def verify_events(data,path,summaries):
    run=data['runs'];inc={tuple(map(int,r[:4])):r for r in run['localIncrementRecords']}
    local={tuple(map(int,r[:4])) for r in run['localGaussianRecords']};sources={tuple(map(int,r[:4])):r for r in run['fusionSourceRecords']}
    outputs={tuple(map(int,r[:4])):r for r in run['fusionOutputRecords']};records={tuple(map(int,r[:4])):r for r in run['iterationRecords']}
    with gzip.open(path,'rt',newline='') as stream:saved=list(csv.DictReader(stream))
    events={tuple(int(e[k]) for k in ['frame','robot','birth_frame','birth_location']):e for e in saved};assert len(events)==len(saved)
    expected={k for k,r in outputs.items() if r[4]>.001 and k not in local};assert set(events)==expected
    by_label=defaultdict(list);values=[]
    for k in expected:
        t,n,bt,bl=k;e=events[k];r=outputs[k];source=sources[k];current=inc.get(k);deleted=current is not None
        assert source[4:6]==[0,0] and source[6]>0 and data['delivered'][n-1][2-n][t-1]
        assert deleted==(e['same_frame_deleted']=='True')
        if deleted:
            assert current[5]<=.001 and float(e['local_r'])==current[5];by_label[n,bt,bl].append(t)
        else:assert e['local_r']=='' and int(e['consecutive_returns'])==0
        qualified=deleted and records[k][13]>0;op=bool(current[7]) if deleted else False
        assert qualified==(e['qualified_censor']=='True') and op==(e['current_opportunity']=='True')
        assert float(e['returned_r'])==r[4]
        ids=data['truthIds'][t-1];target=list(np.asarray(ids).ravel()).index(5);xy=np.asarray(data['truth'][t-1]).reshape(4,-1)[:2,target]
        near=math.hypot(r[5]-xy[0],r[6]-xy[1])<=2.;assert near==(e['near_target']=='True')
        values.append(dict(t=t,n=n,deleted=deleted,qualified=qualified,op=op,near=near,streak=int(e['consecutive_returns'])))
    for (n,bt,bl),times in by_label.items():
        previous=None;length=0
        for t in sorted(times):
            length=length+1 if previous==t-1 else 1;assert length==int(events[t,n,bt,bl]['consecutive_returns']);previous=t
    for summary in summaries:
        left,right=(1,240) if summary['window']=='full' else (53,122)
        selected=[e for e in values if left<=e['t']<=right and (summary['scope']=='all' or e['near'])];deleted=[e for e in selected if e['deleted']]
        expected_counts=dict(remote_only_retained=len(selected),same_frame_deleted=len(deleted),qualified_censor=sum(e['qualified'] for e in deleted),
            qualified_with_opportunity=sum(e['qualified'] and e['op'] for e in deleted),at_least_two_consecutive_returns=sum(e['streak']>=2 for e in deleted),
            maximum_consecutive_returns=max((e['streak'] for e in deleted),default=0))
        for k,v in expected_counts.items():assert summary[k]==v,(k,v)
    return len(expected)

def verify_target(data,saved,diagnostic):
    run=data['runs'];total=[];pool=defaultdict(list);inc={tuple(r[:4]):r[5] for r in run['localIncrementRecords']}
    for r in run['localGaussianRecords']:pool['local',int(r[0]),int(r[1])].append([*r[:4],inc[tuple(r[:4])],*r[18:32]])
    for r in run['fusionOutputRecords']:pool['fusion',int(r[0]),int(r[1])].append(r)
    for t in range(1,241):
        truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T;target=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[target,:2]
        for n in [1,2]:
            row=saved[t,n];est=np.asarray(run['estimates'][n-1+2*(t-1)],float).reshape(-1,4)
            distance=np.array([[math.hypot(a[0]-b[0],a[1]-b[1]) for b in est] for a in truth]).reshape(len(truth),len(est))
            a,b=linear_sum_assignment(np.where(distance<=2,distance,1e6));found=any(i==target and distance[i,j]<=2 for i,j in zip(a,b))
            phase='fusion' if data['delivered'][n-1][2-n][t-1] else 'local';active=[r for r in pool[phase,t,n] if r[4]>.001]
            near=[r for r in active if math.hypot(r[5]-xy[0],r[6]-xy[1])<=2]
            assert found==(row['detected']=='True') and len(est)==int(row['output_count'])
            assert len(active)==int(row['active_components']) and len(near)==int(row['near_active_components'])
            assert abs(math.fsum(r[4] for r in near)-float(row['near_total_r']))<1e-11 and max((r[4] for r in near),default=0)==float(row['near_max_r'])
            total.append(dict(t=t,n=n,found=found,near=len(near),active=len(active)))
    for name,w in diagnostic['windows'].items():
        left,right=(1,240) if name=='full' else (53,122);selected=[r for r in total if left<=r['t']<=right]
        assert w['detected']==sum(r['found'] for r in selected) and w['robot_frames']==len(selected)
        assert w['by_robot']==[sum(r['found'] for r in selected if r['n']==n) for n in [1,2]]
        assert w['maximum_near_components']==max(r['near'] for r in selected) and w['maximum_components']==max(r['active'] for r in selected)
    return len(total)

def main():
    assert not (OUT/'RESULTS.json').exists() and not (OUT/'FINAL_VERIFICATION.json').exists()
    freeze=json.loads((OUT/'FREEZE.json').read_text());assert freeze['passed'];protected={};scores={};diags={};paths={};target_count=0;event_count=0
    v2=json.loads((OUT/'AUDIT_FREEZE_V2.json').read_text())
    for name,h in v2['source_sha256'].items():assert sha(ROOT/name)==h,name
    for stage in ['known_censor_controls','known_censor_refined']:
        cfgpath=OUT/'stages'/(stage+'.json');assert sha(cfgpath)==freeze['configurations'][str(cfgpath.relative_to(ROOT))];cfg=json.loads(cfgpath.read_text())
        auditpath=OUT/('audit_v2_'+stage+'.json');audit=json.loads(auditpath.read_text());assert audit['passed'] and audit['audited_robot_frames']==2880 and len(audit['parity'])==6
        for name,h in {**cfg['source_sha256'],**audit['inputs'],**audit['source_sha256']}.items():assert sha(ROOT/name)==h,name;protected[name]=h
        runtime=json.loads((OUT/('runtime_'+stage+'.json')).read_text())
        assert len(runtime)==2 and all(r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files']==3 for r in runtime)
        scores.update({(r['condition'],r['arm']):r for r in audit['rows']});diags.update({(d['condition'],d['arm']):d for d in audit['diagnostics']})
        with (OUT/('target_frames_v2_'+stage+'.csv')).open(newline='') as stream:frames=list(csv.DictReader(stream))
        lookup={(r['condition'],r['arm'],int(r['frame']),int(r['robot'])):r for r in frames};assert len(lookup)==len(frames)==2880
        with (OUT/('population_frames_v2_'+stage+'.csv')).open(newline='') as stream:population=list(csv.DictReader(stream))
        assert len(population)==2880
        for r in population:
            assert int(r['delta'])==int(r['births'])-int(r['local_pruned'])-int(r['local_lost_at_fusion'])+int(r['remote_only_retained'])
        for u in cfg['units']:
            c=u['execution_id']
            for arm in u['arms']:
                path=OUT/'results'/stage/f"{u['sequence']}_{c}_{arm}.json.gz";data=read(path);d=diags[c,arm];paths[c,arm]=str(path.relative_to(ROOT))
                target_count+=verify_target(data,{(t,n):lookup[c,arm,t,n] for t in range(1,241) for n in [1,2]},d['target'])
                event_count+=verify_events(data,ROOT/d['reimport_event_path'],d['population']['reimport_summaries'])
                p=next(p for p in audit['parity'] if p['condition']==c and p['arm']==arm)
                if stage=='known_censor_controls':
                    old=read(ROOT/u['references'][arm]['path'])['runs'];assert set(p['exact_fields'])==set(old)-{'runtimeSeconds'}
                    for name in p['exact_fields']:assert old[name]==data['runs'][name]
                else:assert len(p['exact_prefix_fields'])==21 and p['first_changed_frame']>=2
                print('INDEPENDENT TARGET AND REIMPORT VERIFIED',c,arm,flush=True)
    assert len(paths)==len(scores)==len(diags)==12 and target_count==5760
    receipts=[json.loads((OUT/'RUNNER_PATCH.json').read_text()),*json.loads((OUT/'AUDITOR_PATCHES.json').read_text())]
    for receipt in receipts:
        source=ROOT/receipt['source'];target=ROOT/receipt['target'];assert sha(source)==receipt['source_sha256'];content=source.read_text()
        if receipt.get('extract'):
            left,right=receipt['extract'];content=content[content.index(left):content.index(right)]
        content=receipt.get('prefix','')+content
        for p in receipt['changes']:assert content.count(p['before'])==1;content=content.replace(p['before'],p['after'])
        assert content==target.read_text() and sha(target)==receipt['target_sha256']
    def reimports(c,a):
        return next(s['same_frame_deleted'] for s in diags[c,a]['population']['reimport_summaries'] if s['window']=='original_window' and s['scope']=='target_neighbourhood')
    effects=[];gate=[]
    for c in ['reliable','intermittent']:
        for a in ARMS:
            old,new=scores[c,a],scores[c,a+SUFFIX];ow=diags[c,a]['target']['windows'];nw=diags[c,a+SUFFIX]['target']['windows']
            effects.append(dict(condition=c,backend=NAMES[a],ospa_delta=new['ospa']-old['ospa'],gospa_delta=new['gospa']-old['gospa'],
                ospa_improvement_percent=100*(1-new['ospa']/old['ospa']),window_detection_delta=nw['original_window']['detected']-ow['original_window']['detected'],
                full_detection_delta=nw['full']['detected']-ow['full']['detected'],wire_byte_change_percent=100*(new['wire_bytes']/old['wire_bytes']-1),
                target_reimport_delta=reimports(c,a+SUFFIX)-reimports(c,a)))
        e=next(e for e in effects if e['condition']==c and e['backend']=='GCE')
        gate.append(dict(condition=c,ospa_improves=e['ospa_delta']<0,gospa_improves=e['gospa_delta']<0,
            window_detections_improve=e['window_detection_delta']>0,full_detections_nonworsening=e['full_detection_delta']>=0,
            wire_nonworsening=scores[c,ARMS[0]+SUFFIX]['wire_bytes']<=scores[c,ARMS[0]]['wire_bytes'],
            window_target_reimports_decrease=e['target_reimport_delta']<0))
    expand=all(all(v for k,v in g.items() if k!='condition') for g in gate)
    comparisons=[dict(condition=c,refined_gce_minus_refined_noage_ospa=scores[c,ARMS[0]+SUFFIX]['ospa']-scores[c,ARMS[2]+SUFFIX]['ospa'],
        refined_gce_minus_refined_noage_gospa=scores[c,ARMS[0]+SUFFIX]['gospa']-scores[c,ARMS[2]+SUFFIX]['gospa']) for c in ['reliable','intermittent']]
    order=[(c,a+s) for c in ['reliable','intermittent'] for a in ARMS for s in ['',SUFFIX]]
    result=dict(passed=True,scope='One exposed complete recursive causal case, not independent method validation',rows=[scores[k] for k in order],
        diagnostics=[diags[k] for k in order],effects=effects,fair_comparisons=comparisons,gate=gate,expand_same_refinement=expand,
        checked_target_frames=target_count,checked_reimport_events=event_count,native_runs=12,native_robot_frames=5760)
    (OUT/'RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'ALL_SCORES.csv').open('w') as stream:
        w=csv.DictWriter(stream,fieldnames=list(result['rows'][0]),lineterminator='\n');w.writeheader();w.writerows(result['rows'])
    lines=['# 使用本地已知存在率：完整递归因果检验','',
        '控制审计 v1 在六条轨迹的全部数值与逐字段检查通过后，因 NumPy int64 无法直接写入 JSON 而退出。v2 只将 NumPy 标量转换为等值 Python 标量并使用新审计输出路径；原源码、部分 CSV/事件文件及失败日志保留。原生算法、六条控制、六条待运行干预配置、公式、容差与门槛未改变。v2 完成了整个审计。','',
        '在原 nominal pD=0.9 的 `v2xt_0001` 上，对 GCE、Guarded Scalar 和 No-age 施加同一接收端干预。只有本帧本地已删除、存在当前观测机会、且原融合已允许缺失项参与的标签，才将 0.001 上界替换为已知的本地实际存在率。匹配、空间融合、额外当前证据和消息格式保持原规则。局部缓存不发包。','',
        '六条新控制与原轨迹逐字段精确一致（运行时间除外）；六条干预都从第 1 帧完整运行。12 条轨迹均退出成功，覆盖 5760 个机器人帧，并完成概率、高斯、匹配、发包、预测/裁剪平衡、MAP 提取及评分核对。所有修改事件独立从原生源标签和本地更新记录重建；原提议参数与实际干预参数分别记录。','',
        '这是已曝光的单案例诊断。目标检出按最终输出与全部真值在 2 m 内一对一匹配计数；窗口仍为第 53–122 帧。回流次数是相关的标签事件数，不是物理目标数。','',
        '| 链路 | 方法 | OSPA ↓ | GOSPA ↓ | 窗口检出 /140 | 全段检出 /480 | 窗口目标回流 | wire 字节 |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for c,a in order:
        r=scores[c,a];d=diags[c,a];name=NAMES[a.removesuffix(SUFFIX)]+(' + 已知本地值' if a.endswith(SUFFIX) else ' 原始')
        lines.append(f"| {c} | {name} | {r['ospa']:.6f} | {r['gospa']:.6f} | {d['target']['windows']['original_window']['detected']} | {d['target']['windows']['full']['detected']} | {reimports(c,a):,} | {int(r['wire_bytes']):,} |")
    lines+=['','| 链路 | 方法 | 定位平方代价 | 漏检平方代价 | 误检平方代价 | 基数误差 | 尝试 payload | 已送达 payload | 窗口附近标签峰值 |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for c,a in order:
        r=scores[c,a];name=NAMES[a.removesuffix(SUFFIX)]+(' + 已知本地值' if a.endswith(SUFFIX) else ' 原始')
        lines.append(f"| {c} | {name} | {r['loc2']:.6f} | {r['miss2']:.6f} | {r['false2']:.6f} | {r['countError']:.6f} | {int(r['raw_bytes']):,} | {int(r['delivered_raw_bytes']):,} | {diags[c,a]['target']['windows']['original_window']['maximum_near_components']} |")
    lines+=['','| 链路 | 方法 | OSPA 改善 % | ΔGOSPA | 窗口检出变化 | 全段检出变化 | 目标回流变化 | wire 变化 % |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for e in effects:lines.append(f"| {e['condition']} | {e['backend']} | {e['ospa_improvement_percent']:+.3f} | {e['gospa_delta']:+.6f} | {e['window_detection_delta']:+d} | {e['full_detection_delta']:+d} | {e['target_reimport_delta']:+d} | {e['wire_byte_change_percent']:+.3f} |")
    lines+=['','## 预先冻结的扩大验证门槛','']
    for g in gate:lines.append('- '+g['condition']+'：'+ '；'.join(k+'='+('通过' if v else '未通过') for k,v in g.items() if k!='condition')+'。')
    lines+=['',('全部门槛通过；下一步仅可对同一规则另行冻结完整数据验证。' if expand else '至少一项门槛未通过，本规则按协议结束，不换阈值、时机或缓存条件继续搜索。'),'',
        '公平比较：']
    for c in comparisons:lines.append(f"- {c['condition']}：同样使用本地实际值后，GCE−No-age 的 OSPA 差为 {c['refined_gce_minus_refined_noage_ospa']:+.6f}，GOSPA 差为 {c['refined_gce_minus_refined_noage_gospa']:+.6f}（负值有利于 GCE）。")
    lines+=['','即使回流被压低，也必须同时看完整检出与误差；内部标签减少不能自行证明 GCE 的方法价值。这里不能从相关标签事件推出独立统计显著性，也不能从单个已曝光案例声称泛化。全部重数、逐帧结果、配对效果和门槛保存在 `RESULTS.json`、CSV 与原生记录中；本文稿不因此更新。','']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(lines))
    for path in OUT.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and '.next.' not in path.name:protected[str(path.relative_to(ROOT))]=sha(path)
    (OUT/'FINAL_VERIFICATION.json').write_text(json.dumps(dict(passed=True,native_runs=12,native_robot_frames=5760,
        exact_original_controls=6,exact_causal_prefixes=6,independently_recounted_target_frames=target_count,
        independently_recounted_reimport_events=event_count,expand_same_refinement=expand,input_sha256=protected),indent=2)+'\n')
    print('KNOWN CENSOR FINAL VERIFICATION PASSED',json.dumps(dict(expand=expand,effects=effects,fair_comparisons=comparisons)),flush=True)

if __name__=='__main__':main()
