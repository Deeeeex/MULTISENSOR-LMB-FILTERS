"""Independent stdlib arithmetic; optionally verify every native/source hash."""
from pathlib import Path
import argparse
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence';HISTORY=GCE+'_miss_history';HALF=GCE+'_miss_half'
mean=lambda xs:math.fsum(xs)/len(xs)
close=lambda a,b:math.isclose(a,b,abs_tol=2e-12,rel_tol=2e-12)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--full',action='store_true');args=parser.parse_args()
    result=json.loads((OUT/'SCREEN_SELECTION.json').read_text());assert result['passed']
    rows=result['rows'];assert len(rows)==90
    lookup={(r['dataset'],r['sequence'],r['condition'],r['arm']):r for r in rows}
    assert len(lookup)==90
    for dataset,n in [('v2v_development',9),('v2x_val',5),('v2x_test_mechanism',1)]:
        assert len({r['sequence'] for r in rows if r['dataset']==dataset})==n
        for arm in [GCE,HISTORY,HALF]:
            for condition in ['reliable','intermittent']:
                assert sum(r['dataset']==dataset and r['arm']==arm and r['condition']==condition for r in rows)==n
    for summary in result['summaries']:
        part=[r for r in rows if all(r[key]==summary[key] for key in ['dataset','condition','arm'])]
        assert len(part)==summary['sequences']
        for key,value in summary['sequence_macro'].items():assert close(mean([r[key] for r in part]),value)
        assert sum(r['wire_bytes'] for r in part)==summary['wire_bytes']
    for comparison in result['comparisons']:
        ds,cond=comparison['dataset'],comparison['condition']
        differences={seq:lookup[ds,seq,cond,comparison['candidate']]['ospa']-lookup[ds,seq,cond,comparison['reference']]['ospa'] for seq in comparison['sequence_deltas']}
        for seq,value in differences.items():assert close(value,comparison['sequence_deltas'][seq])
        assert close(mean(list(differences.values())),comparison['mean_delta'])
        assert comparison['improved']==sum(v < -1e-10 for v in differences.values())
        assert comparison['worsened']==sum(v > 1e-10 for v in differences.values())
    delta={}
    for ds in ['v2v_development','v2x_val']:
        h=[r['ospa'] for r in rows if r['dataset']==ds and r['arm']==HISTORY]
        g=[r['ospa'] for r in rows if r['dataset']==ds and r['arm']==GCE]
        delta[ds]=mean(h)-mean(g)
    assert result['advance']==all(value<0 for value in delta.values())==False
    report=json.loads((OUT/'REPORT_BUILD.json').read_text())
    assert sha(OUT/'SCREEN_SELECTION.json')==report['source_sha256']
    assert sha(OUT/'RESULTS_CN.md')==report['report_sha256']
    assert sha(OUT/'build_report.py')==report['builder_sha256']
    files={};native=0;robot_frames=0;parity=0
    if args.full:
        for stage in ['miss_history_preflight','miss_history_screen']:
            cfgpath=OUT/'stages'/(stage+'.json');cfg=json.loads(cfgpath.read_text())
            audit=json.loads((OUT/('audit_'+stage+'.json')).read_text())
            runtime_path=OUT/('runtime_'+stage+'.json');runtime=json.loads(runtime_path.read_text())
            assert audit['passed'] and audit['config_sha256']==sha(cfgpath) and audit['runtime_sha256']==sha(runtime_path)
            assert len(runtime)==len(cfg['units'])
            for row in runtime:assert row['returncode']==0 and row['completion_line'] and row['files']==2*len(cfg['arms'])
            for group in [cfg['source_sha256'],audit['inputs'],audit['auditor_sha256']]:
                for name,value in group.items():
                    if name in files:assert files[name]==value,name
                    files[name]=value
            paths={str(path.relative_to(ROOT)) for path in (OUT/'results'/stage).glob('*.json.gz')}
            expected={str((OUT/'results'/stage/f"{unit['sequence']}_{condition}_{arm}.json.gz").relative_to(ROOT)) for unit in cfg['units'] for condition in cfg['conditions'] for arm in cfg['arms']}
            assert paths==expected and paths<=audit['inputs'].keys()
            native+=len(paths);robot_frames+=audit['audited_robot_frames'];parity+=len(audit['parity'])
        extra=json.loads((OUT/'POSITIVE_MARK_RECHECK.json').read_text());assert extra['passed']
        assert len(extra['checks'])==12 and extra['local_updates']==397296
        assert extra['auditor_sha256']==sha(OUT/'history_audit_v3.py')
        files.update(extra['inputs'])
        for name,value in files.items():assert sha(ROOT/name)==value,name
        assert native==64 and robot_frames==24364 and parity==4
    answer=dict(passed=True,full=args.full,rows=len(rows),summaries=len(result['summaries']),comparisons=len(result['comparisons']),
                advancement_recomputed=result['advance'],two_condition_mean_deltas=delta,
                native_files=native,audited_robot_frames=robot_frames,exact_baseline_recursions=parity,
                protected_files_checked=len(files),selection_sha256=sha(OUT/'SCREEN_SELECTION.json'),verifier_sha256=sha(Path(__file__)))
    if args.full:
        path=OUT/'SCREEN_VERIFICATION.json';assert not path.exists();path.write_text(json.dumps(answer,indent=2)+'\n')
    print('MISS HISTORY SELECTION VERIFIED',json.dumps(answer),flush=True)


if __name__=='__main__':main()
