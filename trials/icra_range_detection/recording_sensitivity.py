"""Descriptive recording/day sensitivity after the complete frozen screen."""
from pathlib import Path
from collections import defaultdict
import hashlib
import json
import math
import re

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
mean=lambda values:math.fsum(values)/len(values)
GCE='marked_gaussian_evidence';PRIMARY=GCE+'_range'
NAMES={GCE:'原始 GCE',GCE+'_constant':'固定拟合概率 GCE','marked_lineage_range':'距离 No-age',
    GCE+'_guarded_scalar_range':'距离 Guarded Scalar','marked_lineage_constant':'固定拟合概率 No-age',
    GCE+'_guarded_scalar_constant':'固定拟合概率 Guarded Scalar'}

def main():
    path=OUT/'SCREEN_SELECTION.json';selection=json.loads(path.read_text());assert selection['passed']
    verification=json.loads((OUT/'SELECTION_VERIFICATION.json').read_text())
    assert verification['passed'] and verification['selection_sha256']==sha(path)
    units={};config_hashes={}
    for suffix in ['preflight','screen']:
        cfgpath=OUT/'stages'/f'range_detection_{suffix}.json';cfg=json.loads(cfgpath.read_text())
        config_hashes[str(cfgpath.relative_to(OUT))]=sha(cfgpath)
        for u in cfg['units']:
            if u['dataset']=='v2x_test_mechanism':continue
            recording=u.get('source_recording',u['recording'])
            day=re.search(r'\d{4}-\d{2}-\d{2}',recording).group()
            assert u['scene'].startswith(recording)
            units[u['dataset'],u['sequence']]=dict(recording=recording,day=day)
    assert len(units)==14
    lookup={(r['dataset'],r['sequence'],r['condition'],r['arm']):r['ospa'] for r in selection['rows']}
    results=[]
    for dataset in ['v2v_development','v2x_val']:
        scenes=sorted(s for d,s in units if d==dataset)
        for reference in NAMES:
            deltas={s:mean([lookup[dataset,s,c,PRIMARY]-lookup[dataset,s,c,reference] for c in ['reliable','intermittent']]) for s in scenes}
            for grouping in ['recording','day']:
                grouped=defaultdict(list)
                for s,delta in deltas.items():grouped[units[dataset,s][grouping]].append(delta)
                values={g:mean(v) for g,v in grouped.items()}
                omitted={g:mean([v for key,v in values.items() if key!=g]) for g in values}
                results.append(dict(dataset=dataset,reference=reference,grouping=grouping,groups=len(values),
                    sequence_macro_delta=mean(list(deltas.values())),group_macro_delta=mean(list(values.values())),
                    per_group_delta=values,leave_one_group_out_delta=omitted,
                    leave_one_out_min=min(omitted.values()),leave_one_out_max=max(omitted.values())))
    destination=OUT/'RECORDING_SENSITIVITY.json';assert not destination.exists()
    destination.write_text(json.dumps(dict(passed=True,scope='Descriptive sensitivity only; frozen sequence-macro selection remains unchanged',
        grouping=[dict(dataset=d,sequence=s,**u) for (d,s),u in units.items()],results=results,
        selection_sha256=sha(path),config_sha256=config_hashes,source_sha256=sha(Path(__file__))),indent=2)+'\n')
    lines=['# 录制与日期分组的敏感性','',
        '本页使用全部已核验的筛选结果，补充等权录制/日期汇总及逐组删除检查，不改变事先声明的序列均值门槛。所有差值均为距离 GCE 减去对照，两种链路先等权平均；负值表示改善。','',
        'V2V 的 9 段对应 6 次录制、3 个日期；V2X 的 5 段对应 5 次录制、3 个日期。V2X 原表的 `recording` 字段保存的是日期，此处使用明确的 `source_recording` 时间戳分组。不同录制或日期并不自动保证统计独立。','',
        '逐组删除范围是删除每一组后重新计算均值所得的最小值和最大值，不是置信区间，也不支持未暴露数据上的泛化声明。','',
        '| 数据 | 分组 | 对照 | 分组均值差（米） | 逐组删除范围（米） |','|---|---|---|---:|---:|']
    for r in results:
        dataset='V2V' if r['dataset']=='v2v_development' else 'V2X';group='录制' if r['grouping']=='recording' else '日期'
        lines.append(f"| {dataset} | {group}（{r['groups']}） | {NAMES[r['reference']]} | {r['group_macro_delta']:+.9f} | [{r['leave_one_out_min']:+.9f}, {r['leave_one_out_max']:+.9f}] |")
    lines+=['','逐段到分组的完整映射和每组差值保存在 `RECORDING_SENSITIVITY.json`。','']
    report=OUT/'RECORDING_SENSITIVITY_CN.md';assert not report.exists();report.write_text('\n'.join(lines))
    (OUT/'RECORDING_REPORT_BUILD.json').write_text(json.dumps(dict(data_sha256=sha(destination),report_sha256=sha(report),source_sha256=sha(Path(__file__))),indent=2)+'\n')
    print('BUILT RECORDING/DAY SENSITIVITY',len(results),'comparisons; original selection unchanged')

if __name__=='__main__':main()
