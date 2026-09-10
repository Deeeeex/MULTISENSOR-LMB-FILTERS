"""Recount verified events, rebuild every aggregate, and close the fixed screen."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import math
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
RULES=['original','paired_veto','positive_rejected_veto','negative_rejected_veto']
NAMES={'original':'原规则','paired_veto':'双向冲突暂停（主）',
       'positive_rejected_veto':'仅正项被拒时暂停','negative_rejected_veto':'仅负项被拒时暂停'}
METRICS=['ospa','gospa','loc2','miss2','false2','countError','outputCount']


def main():
    destination=OUT/'FINAL_VERIFICATION.json'; assert not destination.exists()
    freeze=OUT/'SCREEN_FREEZE_V2.json'; cfg=json.loads(freeze.read_text())
    report=json.loads((OUT/'SCREEN_RESULTS.json').read_text())
    verification=json.loads((OUT/'SCREEN_VERIFICATION.json').read_text())
    execution=json.loads((OUT/'SCREEN_EXECUTION_V2.json').read_text())
    failed=json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    assert report['passed'] and verification['passed'] and execution['completed'] and execution['returncode']==0
    assert failed['completed'] and failed['returncode']==1
    assert execution['source_sha256']==sha(OUT/'execute_screen_v2.py')
    assert sha(ROOT/execution['log'])==execution['log_sha256']
    assert report['freeze_sha256']==verification['freeze_sha256']==execution['freeze_sha256']==sha(freeze)
    assert verification['report_sha256']==sha(OUT/'SCREEN_RESULTS.json')
    assert verification['verifier_sha256']==sha(OUT/'verify_screen_v2.py')
    assert verification['execution_sha256']==sha(OUT/'SCREEN_EXECUTION_V2.json')
    assert verification['csv_sha256']==sha(OUT/'ALL_SCREEN_SCORES.csv')
    assert report['source_runs']==verification['source_runs']==len(cfg['cells'])==56
    assert report['robot_frames']==verification['alternate_robot_frames']==83584
    assert report['original_parity_robot_frames']==20896 and len(report['rows'])==verification['rows']==224
    assert report['native_runs']==0 and report['fixed_input_only'] and len(report['artifacts'])==112
    assert report['source_sha256']==cfg['source_sha256']
    for name,digest in cfg['source_sha256'].items(): assert sha(ROOT/name)==digest,name
    for name,digest in report['artifacts'].items(): assert sha(ROOT/name)==digest,name
    log=(ROOT/execution['log']).read_text()
    assert log.count('SUBSTITUTION COMPLETE ')==56 and 'SCREEN COMPLETE advance ' in log
    assert 'Traceback' not in log
    # Check the repair contains only its recorded changes; old files stay bound.
    repair=json.loads((OUT/'MISSING_VALUE_CHECK_FIX.json').read_text())
    for name,port in repair['ports'].items():
        p=OUT/port['source']; assert sha(p)==port['source_sha256']; text=p.read_text()
        for old,new in port['patches']: assert old in text; text=text.replace(old,new)
        assert text==(OUT/name).read_text() and sha(OUT/name)==port['output_sha256']
    event_rows=[]; fusion_labels=0
    for i,cell in enumerate(cfg['cells']):
        prefix=OUT/'results'/f"{i:02d}_{cell['sequence']}_{cell['condition']}_{cell['backend'].replace(' ','_')}"
        arrays=np.load(prefix.with_suffix('.npz'))
        original_r=arrays['original__r']; original_mean=arrays['original__mean']
        p=arrays['original__positive_rejected']; n=arrays['original__negative_rejected']
        assert not (p&n).any()
        cell_diagnostic=report['diagnostics'][i]; assert cell_diagnostic['cell']==cell
        for rule in RULES:
            expected={'original':np.zeros(len(p),bool),'paired_veto':p|n,
                      'positive_rejected_veto':p,'negative_rejected_veto':n}[rule]
            mask=arrays[rule+'__trigger']; assert np.array_equal(mask,expected)
            r=arrays[rule+'__r']; mean=arrays[rule+'__mean']; kept=arrays[rule+'__kept']
            assert not kept[mask].any()
            for key in ['r','mean','covariance','log_integral','kept']:
                assert np.array_equal(arrays[rule+'__'+key][~mask],arrays['original__'+key][~mask])
            calculated=dict(fusion_labels=len(r),positive_rejected_events=int(p.sum()),
                negative_rejected_events=int(n.sum()),vetoed_labels=int(mask.sum()),
                existence_increased=int((r>original_r+1e-12).sum()),
                existence_decreased=int((r<original_r-1e-12).sum()),
                mean_absolute_existence_change=float(np.mean(abs(r-original_r))),
                changed_spatial_labels=int((np.max(abs(mean-original_mean),axis=1)>1e-10).sum()))
            expected_diag=next(x for x in cell_diagnostic['rules'] if x['rule']==rule)
            for key,value in calculated.items(): assert value==expected_diag[key],(cell,rule,key)
            event_rows.append(dict(**{k:cell[k] for k in ['dataset','sequence','condition','backend']},rule=rule,**calculated))
        fusion_labels+=len(p)*len(RULES); arrays.close()
    assert fusion_labels==verification['fusion_distributions']
    aggregates=[]
    for dataset in ['v2v_development','v2x_val']:
        for backend in ['GCE','Guarded Scalar']:
            for condition in ['reliable','intermittent']:
                for rule in RULES:
                    group=[r for r in report['rows'] if (r['dataset'],r['backend'],r['condition'],r['rule'])==(dataset,backend,condition,rule)]
                    assert len(group)==(9 if dataset=='v2v_development' else 5)
                    values={k:math.fsum(r[k] for r in group)/len(group) for k in METRICS}
                    old=next(r for r in report['aggregate'] if (r['dataset'],r['backend'],r['rule'])==(dataset,backend,rule))['conditions'][condition]
                    for key,value in values.items(): assert abs(value-old[key])<1e-12
                    events=[r for r in event_rows if (r['dataset'],r['backend'],r['condition'],r['rule'])==(dataset,backend,condition,rule)]
                    counts={k:sum(r[k] for r in events) for k in ['fusion_labels','positive_rejected_events','negative_rejected_events','vetoed_labels','existence_increased','existence_decreased','changed_spatial_labels']}
                    aggregates.append(dict(dataset=dataset,backend=backend,condition=condition,rule=rule,**values,**counts))
    lookup={(r['dataset'],r['backend'],r['condition'],r['rule']):r for r in aggregates}
    gates=[]
    for dataset in ['v2v_development','v2x_val']:
        for condition in ['reliable','intermittent']:
            orig=lookup[dataset,'GCE',condition,'original']['ospa']; candidate=lookup[dataset,'GCE',condition,'paired_veto']['ospa']
            g=dict(dataset=dataset,condition=condition,difference=candidate-orig,relative_percent=100*(candidate/orig-1),passed=candidate<orig)
            for source in [report,verification]:
                old=next(x for x in source['gates'] if (x['dataset'],x['condition'])==(dataset,condition))
                assert g['passed']==old['passed'] and abs(g['difference']-old['difference'])<1e-12
            gates.append(g)
    advance=all(g['passed'] for g in gates)
    assert advance==report['advance_to_recursion']==verification['advance_to_recursion']
    analysis=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),aggregates=aggregates,event_rows=event_rows,
        gates=gates,advance_to_recursion=advance,method='Counts from independently verified saved arrays; math.fsum aggregate reconstruction',
        screen_sha256=sha(OUT/'SCREEN_RESULTS.json'),verification_sha256=sha(OUT/'SCREEN_VERIFICATION.json'))
    (OUT/'SCREEN_ANALYSIS.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    lines=['# 曲率筛选造成的正负不对称：完整固定输入筛查','',
        ('主规则通过全部四项固定输入门槛，允许进入预定完整递归；目前尚无新递归效果结论。' if advance else
         '主规则未通过全部四项固定输入门槛，本规则停止推进；两个方向的控制不另行选作主方法。'),'',
        '本轮检验一条不依赖时刻和目标身份的规则：若曲率检查拒绝了一个有正权重的当前增量，而相反符号的增量仍被接纳，则暂停该标签的全部额外当前校正。继承权重、局部后验及其中的负信息保留，空间回到原池。规则对两个符号方向对称。此前只取消第 2 帧负项的失败结果保持有效；本轮不是该干预的时间调参。','',
        '全部九段 V2V4Real 开发输入与五段已曝光 V2X-Real validation 输入均保留。两个链路条件、GCE 与 Guarded Scalar 各自访问的状态共 56 条源轨迹。以下是每帧替换输出后重新提取集合的结果，没有将替换结果反馈给下一帧。两个后端的状态不同，不能把其分数差直接归因于共同输入上的空间校正。','',
        '## 四项主门槛','', '| 数据 | 链路 | 原 GCE OSPA | 主规则 OSPA | 变化 | 相对变化 | 通过 |',
        '| --- | --- | ---: | ---: | ---: | ---: | --- |']
    for g in gates:
        original=lookup[g['dataset'],'GCE',g['condition'],'original']; candidate=lookup[g['dataset'],'GCE',g['condition'],'paired_veto']
        lines.append(f"| {g['dataset']} | {g['condition']} | {original['ospa']:.9f} | {candidate['ospa']:.9f} | {g['difference']:+.9f} | {g['relative_percent']:+.5f}% | {'是' if g['passed'] else '否'} |")
    for backend in ['GCE','Guarded Scalar']:
        lines+=['',f'## {backend} 访问状态上的全部输出规则','',
            '| 数据 | 链路 | 规则 | OSPA | GOSPA | 定位平方代价 | 漏检平方代价 | 误检平方代价 | 基数误差 |',
            '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
        for r in aggregates:
            if r['backend']==backend:
                lines.append(f"| {r['dataset']} | {r['condition']} | {NAMES[r['rule']]} | {r['ospa']:.9f} | {r['gospa']:.6f} | {r['loc2']:.6f} | {r['miss2']:.6f} | {r['false2']:.6f} | {r['countError']:.6f} |")
    lines+=['','## 触发次数','',
        '下表是融合标签事件的计数，同一物体可能在多个标签和时刻被重复计入。这些次数不代表真实物体数、输出误检数或独立样本量。方向分解的集合误差也不必线性相加。','',
        '| 数据 | 链路 | 源后端 | 全部融合标签 | 正项被拒而负项保留 | 负项被拒而正项保留 | 主规则提高存在率 | 主规则降低存在率 | 主规则改变空间均值 |',
        '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in aggregates:
        if r['rule']=='paired_veto':
            lines.append(f"| {r['dataset']} | {r['condition']} | {r['backend']} | {r['fusion_labels']} | {r['positive_rejected_events']} | {r['negative_rejected_events']} | {r['existence_increased']} | {r['existence_decreased']} | {r['changed_spatial_labels']} |")
    gce_primary=[r for r in aggregates if r['backend']=='GCE' and r['rule']=='paired_veto']
    positive_events=sum(r['positive_rejected_events'] for r in gce_primary)
    negative_events=sum(r['negative_rejected_events'] for r in gce_primary)
    lines+=['','## 机制读数','',
        f'在全部 GCE 输入上，正项被拒、负项保留共 {positive_events:,} 次；负项被拒、正项保留共 {negative_events:,} 次。最早 nominal 分化事件属于前一种，不能据此假定它代表整个语料中的主要方向。']
    negative_miss_worse=all(lookup[d,'GCE',c,'negative_rejected_veto']['miss2']>lookup[d,'GCE',c,'original']['miss2']
                          for d in ['v2v_development','v2x_val'] for c in ['reliable','intermittent'])
    if negative_miss_worse:
        lines+=['','方向控制显示，取消负项被拒后保留下来的正校正，在四个组合上均增加即时漏检代价。这里是固定输入下的直接输出变化，不是整段递归失效的充分原因证明。']
    lines+=['','各方向的具体改善、退化或不变均保留在上表。无论单向控制是否在某个指标上更有利，都不会替换事先固定的对称主规则。']
    lines+=['','## 验证与边界','',
        f'完整检查原结果的 20,896 个机器人帧；四规则共 83,584 个替换输出机器人帧、{fusion_labels:,} 个融合分布。生产计算由局部预测和后验矩构建空间池，独立验证从保存的融合密度减去原传输增量重建，并单独计算触发事件、基数分布、提取集合和 OSPA/GOSPA。全部 224 行序列分数、112 份输出文件及四项门槛均核对。这里的独立计算仍在同一工作流程内完成，不是第三方复现。','',
        '首次执行在首条源轨迹的原规则检查处退出：缺席来源的存在率为 JSON null，转换成 NaN 后普通相等判断会误报。逐字节检查确认当时 3,210 行记录无任何变化，尚无候选分数或完整输出。v2 只将该无改动断言改为逐字节比较；旧源码、冻结清单及失败日志原样保留，公式、事件、门槛和数据名单均未变。','',
        '本轮未重跑原生跟踪，也没有通信、运行时间或未见录制泛化的新结论。全部输入已参与开发。' +
        ('下一阶段必须执行协议预定的 56 条新递归及全部比较，固定输入通过不能替代最终效果。' if advance else
         '依照冻结协议，不更改规则或另选方向继续筛查。本轮没有得到可替换原 GCE 的方法。'),'',
        '逐序列入口：`ALL_SCREEN_SCORES.csv`；全部分布、集合和分数：`SCREEN_RESULTS.json` 的 artifacts；事件和汇总：`SCREEN_ANALYSIS.json`；核验：`SCREEN_VERIFICATION.json`、`FINAL_VERIFICATION.json`。','']
    (OUT/'RESULTS_CN.md').write_text('\n'.join(lines))
    sources=cfg['source_sha256'].copy(); sources.update(report['artifacts']); sources.update(verification['inputs'])
    sources[str((ROOT/execution['log']).relative_to(ROOT))]=sha(ROOT/execution['log'])
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in OUT.iterdir() if p.is_file()})
    for name,digest in sources.items(): assert sha(ROOT/name)==digest,name
    final=dict(passed=True,protected_files=len(sources),source_runs=56,alternate_robot_frames=83584,
        original_parity_robot_frames=20896,fusion_distributions=fusion_labels,new_recursive_runs=0,
        gate_passes=sum(g['passed'] for g in gates),gate_count=4,advance_to_recursion=advance,
        source_sha256=sources,verifier_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(final,indent=2,allow_nan=False)+'\n')
    print('FIXED SCREEN CLOSED',len(sources),'hashes;',final['gate_passes'],'/ 4 gates; advance',advance,flush=True)


if __name__=='__main__':main()
