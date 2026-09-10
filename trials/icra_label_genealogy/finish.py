"""Rebuild census summaries and preserve complete diagnostic provenance."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def read_table(name):
    with (OUT/name).open(newline='') as stream:return list(csv.DictReader(stream))


def main():
    destination=OUT/'FINAL_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'REIMPORT_FREEZE.json').read_text())
    census=json.loads((OUT/'CENSUS_RESULTS.json').read_text())
    verification=json.loads((OUT/'CENSUS_VERIFICATION.json').read_text())
    reimport=json.loads((OUT/'REIMPORT_RESULTS.json').read_text())
    rv=json.loads((OUT/'REIMPORT_VERIFICATION.json').read_text())
    execution=json.loads((OUT/'CENSUS_EXECUTION_V2.json').read_text())
    assert all(x['passed'] for x in [census,verification,reimport,rv])
    assert execution['completed'] and execution['returncode']==0
    assert execution['source_sha256']==sha(OUT/'execute_census_v2.py')
    assert execution['log_sha256']==sha(ROOT/execution['log'])
    assert verification['report_sha256']==sha(OUT/'CENSUS_RESULTS.json')
    assert verification['freeze_sha256']==execution['freeze_sha256']==census['freeze_sha256']==sha(OUT/'CENSUS_FREEZE_V2.json')
    assert verification['execution_sha256']==sha(OUT/'CENSUS_EXECUTION_V2.json')
    assert verification['verifier_sha256']==sha(OUT/'verify_census_v2.py')
    assert rv['report_sha256']==sha(OUT/'REIMPORT_RESULTS.json') and rv['freeze_sha256']==sha(OUT/'REIMPORT_FREEZE.json')
    assert rv['verifier_sha256']==sha(OUT/'verify_reimport.py')
    assert census['robot_frames']==verification['robot_frames']==2880
    assert census['target_phase_rows']==verification['target_phase_rows']==5760
    assert reimport['event_rows']==rv['event_rows']==153262 and rv['summary_rows']==24
    assert reimport['events_sha256']==sha(ROOT/reimport['events_path'])
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    repair=json.loads((OUT/'INTEGER_STORAGE_FIX.json').read_text())
    for name,port in repair['ports'].items():
        source=OUT/port['source'];assert sha(source)==port['source_sha256'];text=source.read_text()
        for old,new in port['patches']:assert old in text;text=text.replace(old,new)
        assert (OUT/name).read_text()==text and sha(OUT/name)==port['output_sha256']
    population=read_table('POPULATION.csv');targets=read_table('TARGET_ANCESTRY.csv')
    summaries=[];first_counts=[];age_rows=[];frame53=[]
    for cell in cfg['cells']:
        selected=[r for r in population if (r['condition'],r['backend'])==(cell['condition'],cell['backend'])]
        target=[r for r in targets if (r['condition'],r['backend'])==(cell['condition'],cell['backend']) and r['phase']=='posterior']
        assert len(selected)==len(target)==480
        for window,lo,hi in [('full',1,240),('original_window',53,122)]:
            group=[r for r in selected if lo<=int(r['frame'])<=hi]
            counts={k:sum(int(r[k]) for r in group) for k in ['birth_count','birth_local_retained','birth_local_pruned','inherited_local_pruned',
                'local_pruned','local_lost_at_fusion','remote_only_retained','population_delta']}
            assert counts['population_delta']==counts['birth_count']-counts['local_pruned']-counts['local_lost_at_fusion']+counts['remote_only_retained']
            observations=[r for r in target if lo<=int(r['frame'])<=hi]
            counts.update(mean_retained_labels=math.fsum(int(r['retained_count']) for r in group)/len(group),
                maximum_retained_labels=max(int(r['retained_count']) for r in group),
                maximum_target_near_labels=max(int(r['near_count']) for r in observations),
                target_shared_ancestry_rows=sum(int(r['pairs_with_shared_roots'])>0 for r in observations),
                maximum_target_shared_pairs=max(int(r['pairs_with_shared_roots']) for r in observations))
            summaries.append(dict(condition=cell['condition'],backend=cell['backend'],window=window,**counts))
        frame53.extend([r for r in target if int(r['frame'])==53])
        path=OUT/'results/v2'/f"{cell['condition']}_{cell['backend'].replace(' ','_')}.json.gz"
        with gzip.open(path,'rt') as stream:raw=json.load(stream)
        for pool in raw['pools']:
            cohorts={}
            for label in pool['labels']:
                born=label['label'][0];cohorts[born]=cohorts.get(born,0)+1
            for born,count in sorted(cohorts.items()):
                age_rows.append(dict(condition=cell['condition'],backend=cell['backend'],frame=pool['frame'],robot=pool['robot'],
                    phase=pool['phase'],birth_frame=born,label_age=pool['frame']-born,count=count))
    for condition in ['reliable','intermittent']:
        noage={(int(r['frame']),int(r['robot'])):r for r in population if (r['condition'],r['backend'])==(condition,'No-age')}
        for backend in ['GCE','Guarded Scalar']:
            group=[r for r in population if (r['condition'],r['backend'])==(condition,backend)]
            first={}
            for field in ['predicted_count','local_count','retained_count']:
                row=next((r for r in group if int(r[field])!=int(noage[int(r['frame']),int(r['robot'])][field])),None)
                first[field]=None if row is None else dict(frame=int(row['frame']),robot=int(row['robot']),
                    candidate_count=int(row[field]),noage_count=int(noage[int(row['frame']),int(row['robot'])][field]))
            first_counts.append(dict(condition=condition,backend=backend,first_count_difference=first))
    age_path=OUT/'results/v2/LABEL_COHORTS.csv.gz'
    with gzip.open(age_path,'wt',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(age_rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(age_rows)
    analysis=dict(passed=True,population_summaries=summaries,first_count_differences=first_counts,
        frame53_target_posterior=frame53,birth_cohort_rows=len(age_rows),birth_cohort_path=str(age_path.relative_to(ROOT)),
        birth_cohort_sha256=sha(age_path),reimport_summaries=reimport['summaries'])
    (OUT/'ANALYSIS.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    lines=['# 原 GCE 失效：标签数量与同帧回流账本','',
        '定位到一个明确的循环：本地更新将标签降到删除阈值以下，接收融合却只使用 0.001 的缺标签上界，使该标签从对端重新进入当前后验。这个循环在原失效目标附近大量发生；它与完整漏检的因果关系仍需原生干预验证。','',
        '本轮读取同一个已曝光 nominal 案例的全部六条旧轨迹：GCE、Guarded Scalar、No-age，各自两种链路，240 帧、两个机器人。没有修改跟踪结果、门控或阈值，也没有新方法分数。','',
        '## 每一帧的数量都守恒','',
        '逐帧确认：后验数量变化 = 新出生 − 本地删除 − 融合丢弃的本地标签 + 保留的对端独有标签。全部方法的出生标签、存在率和空间先验完全相同。每条轨迹共有 2,909 个出生假设，第一轮本地更新后全部保留。','',
        '这与原参数一致：出生存在率为 0.01，即使 pD=0.9 且完全漏检，其更新值仍为 0.001009081735620585，略高于 0.001 的删除阈值。这个共同设置本身不能解释方法间差异。','',
        '| 链路 | 方法 | 本地删除的继承标签次数 | 融合丢弃的本地标签次数 | 对端独有标签引入次数 | 平均后验标签数 |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for row in summaries:
        if row['window']=='full':
            lines.append(f"| {row['condition']} | {row['backend']} | {row['inherited_local_pruned']:,} | {row['local_lost_at_fusion']:,} | {row['remote_only_retained']:,} | {row['mean_retained_labels']:.3f} |")
    lines+=['','这些是重复发生的标签事件，不是不同物体或不同虚警的数量。完整逐帧账本见 `POPULATION.csv`；所有出生帧的标签队列见 `ANALYSIS.json` 所指向的压缩 CSV。','',
        '## 同一帧删除后又引入','',
        '完整连接本地更新与接收记录后，确认大部分引入事件是同一标签刚在本地被删掉。下表的“有本地精确值”还要求缺标签项实际参与融合；这些事件均有当前名义观测机会，记录中的 0.001 上界都高于本地实际存在率。','',
        '| 链路 | 方法 | 同帧删除后引入 | 有本地精确值的参与删失项 | 实际存在率均值 | 至少连续两帧的回流事件 | 最长连续回流 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for row in reimport['summaries']:
        if (row['window'],row['scope'])==('full','all'):
            lines.append(f"| {row['condition']} | {row['backend']} | {row['same_frame_deleted']:,} | {row['qualified_censor']:,} | {row['mean_deleted_r']:.9f} | {row['at_least_two_consecutive_returns']:,} | {row['maximum_consecutive_returns']} 帧 |")
    lines+=['','### 原目标窗口：第 53–122 帧、GT 5 的 2 m 邻域','',
        '| 链路 | 方法 | 同帧删除后引入 | 有本地精确值的参与删失项 | 实际存在率均值 |',
        '| --- | --- | ---: | ---: | ---: |']
    for row in reimport['summaries']:
        if (row['window'],row['scope'])==('original_window','target_neighbourhood'):
            value='—' if row['mean_deleted_r'] is None else f"{row['mean_deleted_r']:.9f}"
            lines.append(f"| {row['condition']} | {row['backend']} | {row['same_frame_deleted']:,} | {row['qualified_censor']:,} | {value} |")
    lines+=['','No-age 在整段其他位置也有回流，但在该目标窗口内没有“本地删掉后同帧返回”的事件。该区别比单纯的全局标签总数更具体；仍不能单凭相关性认定它解释了全部 GCE 退化。','',
        '## 出生来源重叠不是早期目标堆积的解释','',
        '每个出生假设是一个来源叶节点，预测和更新继承来源，融合按实际两侧源标签取并集。它只描述出生假设的谱系，不是全部测量历史，也不把共享来源当成同一真实物体的证明。','',
        '| 链路 | 方法 | 全池首次出现出生来源重叠的融合后帧 | 原目标窗口有重叠的机器人帧数 |',
        '| --- | --- | ---: | ---: |']
    for cell in census['cells']:
        c=cell['cell'];event=cell['first']['posterior'];where='无' if event is None else str(event['frame'])
        s=next(s for s in summaries if (s['condition'],s['backend'],s['window'])==(c['condition'],c['backend'],'original_window'))
        lines.append(f"| {c['condition']} | {c['backend']} | {where} | {s['target_shared_ancestry_rows']} / 140 |")
    lines+=['','第 53 帧的目标邻域：','',
        '| 链路 | 方法 | 机器人 | 附近标签 | 最大存在率 | 独立出生根数 | 共享出生根数 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for r in frame53:
        lines.append(f"| {r['condition']} | {r['backend']} | {r['robot']} | {r['near_count']} | {float(r['near_max_r']):.6f} | {r['ancestry_roots']} | {r['repeated_roots']} |")
    lines+=['','这些标签确实来自不同出生假设，不能通过简单的同源去重解释或消除早期堆积。第一次数量分化的各阶段记录保存在 `ANALYSIS.json`；完整来源集合保存在六份谱系输出中。','',
        '## 验证范围','',
        f"独立检查全部 2,880 个机器人帧、5,760 个目标阶段行、{sum(c['checked_predictions'] for c in census['cells']):,} 条预测存在率与均值、{sum(c['checked_predicted_covariances'] for c in census['cells']):,} 条可用预测协方差，以及全部 153,262 条对端独有标签引入事件。生产实现以整数位集合传播谱系，复核使用独立的出生 ID 集合和原生来源记录；所有保存的来源集合、事件字段、连续回流和汇总数均核对。",'',
        '首次结果写入因 NumPy 整数类型失败，保留原代码、退出日志及未完成文件。v2 只在解析标签时转成 Python 整数，数值、来源规则和全部输入没有改变。新版本运行成功后才产生有效结果，独立核验通过。','',
        '本轮是同一工作流程中的独立实现复算，尚不是第三方复现。原生输入、旧评分和论文保持原样。下一项可检验的问题是：融合时使用已知的本地实际低存在率，能否打断回流，并在完整递归中改善检出；必须给 No-age 和 Guarded Scalar 同样的信息后比较。','']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(lines))
    sources=cfg['source_sha256'].copy();sources.update(census['artifacts'])
    sources[reimport['events_path']]=reimport['events_sha256'];sources[str(age_path.relative_to(ROOT))]=sha(age_path)
    sources[execution['log']]=execution['log_sha256']
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in OUT.iterdir() if p.is_file()})
    for name,digest in sources.items():assert sha(ROOT/name)==digest,name
    final=dict(passed=True,protected_files=len(sources),native_runs=0,source_runs=6,robot_frames=2880,target_phase_rows=5760,
        checked_predictions=sum(c['checked_predictions'] for c in census['cells']),
        checked_predicted_covariances=sum(c['checked_predicted_covariances'] for c in census['cells']),
        reimport_events=153262,birth_roots_per_run=2909,source_sha256=sources,verifier_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(final,indent=2,allow_nan=False)+'\n')
    print('GENEALOGY AND REIMPORT ARCHIVE VERIFIED',len(sources),'files',flush=True)


if __name__=='__main__':main()
