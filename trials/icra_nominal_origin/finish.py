"""Recount complete native target outputs and publish the bounded causal result."""
from collections import defaultdict
from pathlib import Path
import ast
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
GCE='marked_gaussian_evidence';NOAGE='marked_lineage';ARM=GCE+'_once_initial_negative'
NAMES={GCE:'原 GCE',ARM:'仅第 2 帧取消额外负标量',NOAGE:'No-age（复用）'}

def main():
    assert not (OUT/'FINAL_VERIFICATION.json').exists() and not (OUT/'INTERVENTION_RESULTS.json').exists()
    freeze=json.loads((OUT/'INTERVENTION_FREEZE.json').read_text());assert freeze['passed']
    protected={};rows={};diagnostics={};paths={};timeline=[];new_frames=0;reused_frames=0
    for stage in ['nominal_origin_controls','nominal_origin_event']:
        cfgpath=OUT/'stages'/(stage+'.json');cfg=json.loads(cfgpath.read_text());assert sha(cfgpath)==freeze['configurations'][str(cfgpath.relative_to(ROOT))]
        audit=json.loads((OUT/('audit_'+stage+'.json')).read_text());assert audit['passed']
        for name,h in {**cfg['source_sha256'],**audit['inputs'],**audit['source_sha256']}.items():assert sha(ROOT/name)==h,name;protected[name]=h
        runtime=json.loads((OUT/('runtime_'+stage+'.json')).read_text())
        assert len(runtime)==2 and all(r['returncode']==0 and r['completion_line'] and r['files']==r['expected_files']==1 for r in runtime)
        for u in cfg['units']:
            condition=u['conditions'][0]
            for arm in u['arms']:paths[condition,arm]=OUT/'results'/stage/f"{u['sequence']}_{condition}_{arm}.json.gz"
            for ref in u['references'].values():assert sha(ROOT/ref['path'])==ref['sha256'];protected[ref['path']]=ref['sha256']
            if stage=='nominal_origin_controls':paths[condition,NOAGE]=ROOT/u['references'][NOAGE]['path']
        if stage=='nominal_origin_controls':
            assert len(audit['parity'])==2
            for p in audit['parity']:
                ref=next(u['references'][GCE] for u in cfg['units'] if u['conditions']==[p['condition']])
                with gzip.open(ROOT/ref['path'],'rt') as stream:original=json.load(stream)['runs']
                assert set(p['exact_fields'])==set(original)-{'runtimeSeconds'}
        else:assert len(audit['parity'])==2 and all(len(p['exact_prefix_fields'])==21 for p in audit['parity'])
        rows.update({(r['condition'],r['arm']):r for r in audit['rows']});diagnostics.update({(d['condition'],d['arm']):d for d in audit['diagnostics']})
        with (OUT/('target_frames_'+stage+'.csv')).open(newline='') as stream:timeline+=list(csv.DictReader(stream))
        new_frames+=audit['audited_robot_frames'];reused_frames+=audit['reused_robot_frames']
    lookup={(r['condition'],r['arm'],int(r['frame']),int(r['robot'])):r for r in timeline}
    assert len(paths)==len(rows)==6 and len(lookup)==len(timeline)==2880 and new_frames==1920 and reused_frames==960
    checked=0;events=[]
    for (condition,arm),path in paths.items():
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        run=data['runs'];total=[];local=defaultdict(list);fused=defaultdict(list)
        existence={tuple(r[:4]):r[5] for r in run['localIncrementRecords']}
        for r in run['localGaussianRecords']:local[int(r[0]),int(r[1])].append([*r[:4],existence[tuple(r[:4])],*r[18:32]])
        for r in run['fusionOutputRecords']:fused[int(r[0]),int(r[1])].append(r)
        for t in range(1,241):
            truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T;ti=np.flatnonzero(np.asarray(data['truthIds'][t-1]).ravel()==5).item();xy=truth[ti,:2]
            for n in [1,2]:
                saved=lookup[condition,arm,t,n];est=np.asarray(run['estimates'][n-1+2*(t-1)],float).reshape(-1,4)
                distance=np.array([[math.hypot(a[0]-b[0],a[1]-b[1]) for b in est] for a in truth]).reshape(len(truth),len(est))
                a,b=linear_sum_assignment(np.where(distance<=2,distance,1e6));detected=any(i==ti and distance[i,j]<=2 for i,j in zip(a,b))
                assert detected==(saved['detected']=='True') and len(est)==int(saved['output_count'])
                values=fused[t,n] if data['delivered'][n-1][2-n][t-1] else local[t,n]
                active=[v for v in values if v[4]>.001];near=[v for v in active if math.hypot(v[5]-xy[0],v[6]-xy[1])<=2]
                assert len(active)==int(saved['active_components']) and len(near)==int(saved['near_active_components'])
                assert abs(math.fsum(v[4] for v in near)-float(saved['near_total_r']))<1e-11
                assert max([v[4] for v in near],default=0)==float(saved['near_max_r'])
                total.append(dict(t=t,n=n,detected=detected,near=len(near),all=len(active)));checked+=1
        windows=diagnostics[condition,arm]['target']['windows']
        for name,left,right in [('full',1,240),('before_event',1,1),('after_event',2,240),('original_window',53,122)]:
            selected=[r for r in total if left<=r['t']<=right];w=windows[name]
            assert sum(r['detected'] for r in selected)==w['detected'] and len(selected)==w['robot_frames']
            assert [sum(r['detected'] for r in selected if r['n']==n) for n in [1,2]]==w['by_robot']
            assert max(r['near'] for r in selected)==w['maximum_near_components'] and max(r['all'] for r in selected)==w['maximum_components']
        if arm==ARM:
            e=np.asarray(run['initialNegativeRecords'],float).reshape(-1,18);changed=e[e[:,6]<0]
            assert (e[:,0]==2).all() and len(e)==diagnostics[condition,arm]['event_records'] and len(changed)==diagnostics[condition,arm]['altered_event_records']
            for r in changed:assert abs(math.log(r[5])-math.log1p(-r[5])-math.log(r[4])+math.log1p(-r[4])+r[6])<1e-10
            unchanged=e[e[:,6]==0];assert np.array_equal(unchanged[:,4],unchanged[:,5])
            events.append(dict(condition=condition,recorded_labels=len(e),changed_labels=len(changed),maximum_r_change=float(abs(e[:,5]-e[:,4]).max(initial=0)),records=e.tolist()))
        print('NOMINAL NATIVE TARGET VERIFIED',condition,arm,flush=True)
    for name in ['RUNNER_PATCH.json','AUDITOR_PATCH.json']:
        receipt=json.loads((OUT/name).read_text());source=ROOT/receipt['source'];target=ROOT/receipt['target']
        assert sha(source)==receipt['source_sha256'];text=source.read_text()
        for c in receipt['changes']:assert text.count(c['before'])==1;text=text.replace(c['before'],c['after'])
        assert text==target.read_text() and sha(target)==receipt['target_sha256']
    effects=[]
    for condition in ['reliable','intermittent']:
        old,new=rows[condition,GCE],rows[condition,ARM]
        old_w=diagnostics[condition,GCE]['target']['windows'];new_w=diagnostics[condition,ARM]['target']['windows']
        noage_w=diagnostics[condition,NOAGE]['target']['windows']
        effects.append(dict(condition=condition,ospa_delta=new['ospa']-old['ospa'],ospa_change_percent=100*(new['ospa']/old['ospa']-1),
            wire_byte_change_percent=100*(new['wire_bytes']/old['wire_bytes']-1),
            window_detection_delta=new_w['original_window']['detected']-old_w['original_window']['detected'],
            full_detection_delta=new_w['full']['detected']-old_w['full']['detected'],
            equals_noage_window=new_w['original_window']['detected']==noage_w['original_window']['detected']))
    order=[(c,a) for c in ['reliable','intermittent'] for a in [GCE,ARM,NOAGE]]
    result=dict(passed=True,scope='Single exposed nominal case; one frame-2 negative-scalar intervention under both links; no method selection',
        rows=[rows[k] for k in order],diagnostics=[diagnostics[k] for k in order],effects=effects,events=events,
        native_new_robot_frames=new_frames,native_reused_robot_frames=reused_frames,independent_target_rows=checked)
    (OUT/'INTERVENTION_RESULTS.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'ALL_SCORES.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(result['rows'][0]),lineterminator='\n');writer.writeheader();writer.writerows(result['rows'])
    lines=['# 原 nominal GCE：第 2 帧负增量的因果检验','',
        '本轮返回原 nominal pD=0.9 模型，保留 `v2xt_0001` 的可靠和间歇链路。此前 range 模型的单次干预已经结束，不能替代此处对原 GCE 失效的解释。','',
        '## 最早分化','',
        '六条旧轨迹的完整追踪显示，GCE 和 GS 均在第 2 帧首次与 No-age 出现有效差异；此前局部源输入逐项相同，源标签匹配相同。机器人 1 的匹配标签 `[2,100006]` 的存在率从 No-age 的 0.008679466919197855 降为 GCE 的 0.003401795027527528。差异来自 −0.9419666289521098 的额外负 log-odds；空间差异仅为浮点舍入量级。第 3 帧的局部更新随之分化。该时间顺序本身不是因果证明。','',
        '这个最早事件直接涉及曲率准入的不对称：第一源的局部正增量为 +2.6553273208730146，正门为 0.1750782041549245，但曲率拒绝使保留指数为 0；第二源的局部负增量为 −2.3025850929940463，负门为 0.8181818181818181，曲率放行后保留指数为 0.40909090909090906。额外校正因而只剩负项。本次干预检验取消最早负标量项的影响，没有改变曲率规则或测试接纳被拒正增量。','',
        '## 固定的单次干预','',
        '每个接收到消息的第 2 帧融合处，对所有输出标签取消已接纳的额外负标量项。保留完整 GCE 空间分布、历史项、额外正项、曲率决定和局部后验中已有的负信息。无负项的标签保留原存在率的精确值；第 3 帧起使用完整原 GCE。没有依据目标、标签身份、邻域或结果选择改动对象。','',
        '两条 GCE 控制轨迹重跑后与原有共同记录逐字段精确相同（运行时间除外）。两条干预轨迹的事件前局部输入和消息、事件时的所有非存在率融合记录与控制精确相同。所有四条新轨迹都真实退出成功，并通过全程概率、空间高斯、匹配、发包、MAP 提取和评分检查。','',
        '## 全部结果','',
        'OSPA/GOSPA 为完整 240 帧、两个机器人的均值。目标检出使用最终输出与全部真值的 2 m 门限一对一匹配；原窗口固定为第 53–122 帧。','',
        '| 链路 | 方法 | OSPA ↓ | GOSPA ↓ | 窗口目标检出 | 全段目标检出 | 尝试 payload 字节 | wire 字节 |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for key in order:
        c,arm=key;r=rows[key];w=diagnostics[key]['target']['windows']
        lines.append(f"| {c} | {NAMES[arm]} | {r['ospa']:.6f} | {r['gospa']:.6f} | {w['original_window']['detected']}/140 | {w['full']['detected']}/480 | {int(r['raw_bytes']):,} | {int(r['wire_bytes']):,} |")
    lines+=['','| 链路 | 方法 | 定位平方代价 | 漏检平方代价 | 误检平方代价 | 基数误差 | 窗口附近候选峰值 |',
        '|---|---|---:|---:|---:|---:|---:|']
    for key in order:
        c,arm=key;r=rows[key];w=diagnostics[key]['target']['windows']['original_window']
        lines.append(f"| {c} | {NAMES[arm]} | {r['loc2']:.6f} | {r['miss2']:.6f} | {r['false2']:.6f} | {r['countError']:.6f} | {w['maximum_near_components']} |")
    lines+=['','候选峰值指距目标 2 m 内、存在率大于 0.001 的内部标签数，不等于输出误检数。wire 包含实际 16384 字节填充分块及每帧 256 字节控制成本。','',
        '| 链路 | 单次干预相对 GCE 的 ΔOSPA | 窗口检出次数变化 | 全段检出次数变化 | wire 变化 | 达到 No-age 窗口检出数 |',
        '|---|---:|---:|---:|---:|---|']
    for e in effects:lines.append(f"| {e['condition']} | {e['ospa_delta']:+.6f} | {e['window_detection_delta']:+d} | {e['full_detection_delta']:+d} | {e['wire_byte_change_percent']:+.3f}% | {'是' if e['equals_noage_window'] else '否'} |")
    lines+=['','## 结论边界','']
    if all(e['equals_noage_window'] for e in effects):
        lines+=['这次单帧改动在两种链路下均达到 No-age 的原窗口目标检出数，支持最早负标量状态差可以改变该案例的长期失效路径。整段其他目标、虚警和误差仍须按上表分别判断，不能把一个目标的恢复等同于整个方法获益。']
    else:
        lines+=['这次单帧改动未在两种链路下都达到 No-age 的原窗口目标检出数。因此，取消最早一次额外负标量项尚不足以解释或完全修复原失效；不能由最早分化的时间位置认定它就是充分原因。具体改善和退化均保留在表中。']
    lines+=['','按冻结协议结束本次实验，不延长持续时间或选择其他起始帧。这个已曝光案例的因果检验没有产生新的通用算法，也不能替代完整数据及未参与修改录制的验证。','',
        '复算覆盖 1920 个新机器人帧与 960 个复用 No-age 机器人帧，另独立重数全部 2880 个目标机器人帧及附近候选。完整逐帧关联集中度保存在两份 `target_frames_*.csv` 和 `INTERVENTION_RESULTS.json`。源文件、原生数据和记录哈希见 `FINAL_VERIFICATION.json`。','']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(lines))
    for p in OUT.glob('*.py'):ast.parse(p.read_text())
    for p in OUT.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and '.next.' not in p.name:protected[str(p.relative_to(ROOT))]=sha(p)
    result=dict(passed=True,native_new_runs=4,native_reused_runs=2,native_new_robot_frames=1920,native_reused_robot_frames=960,
        independent_target_rows=checked,exact_original_controls=2,exact_prefix_interventions=2,
        source_sha256=sha(Path(__file__)),input_sha256=protected,
        scope='Complete single-case causal diagnostic; no method promotion, timing search or extension')
    (OUT/'FINAL_VERIFICATION.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('NOMINAL ORIGIN FINAL VERIFICATION PASSED',json.dumps(effects),flush=True)

if __name__=='__main__':main()
