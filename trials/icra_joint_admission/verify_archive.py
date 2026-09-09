"""Verify frozen inputs, all diagnostic artifacts, reports and exact decision."""
from pathlib import Path
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'FINAL_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    report=json.loads((OUT/'SCREEN_RESULTS.json').read_text())
    verification=json.loads((OUT/'SCREEN_VERIFICATION.json').read_text())
    arithmetic=json.loads((OUT/'WEIGHT_ARITHMETIC_VERIFICATION.json').read_text())
    execution=json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    build=json.loads((OUT/'REPORT_BUILD.json').read_text())
    assert all(x['passed'] for x in [report,verification,arithmetic,build])
    assert execution['completed'] and execution['returncode']==0
    assert execution['source_sha256']==sha(OUT/'execute_screen.py')
    assert report['source_runs']==verification['source_runs']==len(cfg['cells'])==56
    assert report['robot_frames']==verification['alternate_robot_frames']==83584
    assert report['original_parity_robot_frames']*4==report['robot_frames']
    assert report['native_runs']==0 and report['fixed_input_only']
    assert len(report['rows'])==verification['rows']==224
    assert arithmetic['source_runs']==18 and arithmetic['checked_rows']==407972
    assert len(report['artifacts'])==112
    assert report['source_sha256']==cfg['source_sha256']
    assert report['freeze_sha256']==verification['freeze_sha256']==execution['freeze_sha256']==sha(OUT/'SCREEN_FREEZE.json')
    assert verification['report_sha256']==arithmetic['report_sha256']==sha(OUT/'SCREEN_RESULTS.json')
    assert verification['verifier_sha256']==sha(OUT/'verify_screen.py')
    assert verification['execution_sha256']==sha(OUT/'SCREEN_EXECUTION.json')
    assert verification['csv_sha256']==sha(OUT/'ALL_SCREEN_SCORES.csv')
    assert arithmetic['source_sha256']==sha(OUT/'verify_weight_arithmetic.py')
    for key,file in [('report_sha256','RESULTS_CN.md'),('screen_sha256','SCREEN_RESULTS.json'),
                     ('verification_sha256','SCREEN_VERIFICATION.json'),('arithmetic_sha256','WEIGHT_ARITHMETIC_VERIFICATION.json'),
                     ('builder_sha256','build_report.py')]:assert build[key]==sha(OUT/file)
    gates=[]
    for dataset in ['v2v_development','v2x_val']:
        values={}
        for rule in ['original','conditional','joint']:
            group=[r for r in report['rows'] if (r['dataset'],r['backend'],r['rule'])==(dataset,'GCE',rule)]
            assert len(group)==(18 if dataset=='v2v_development' else 10)
            values[rule]=math.fsum(r['ospa'] for r in group)/len(group)
        for reference in ['original','conditional']:
            g=next(g for g in report['gates'] if (g['dataset'],g['reference'])==(dataset,reference))
            assert abs(g['difference']-(values['joint']-values[reference]))<1e-12
            assert g['passed']==(values['joint']<values[reference]);gates.append(g)
    assert report['advance_to_recursion']==verification['advance_to_recursion']==all(g['passed'] for g in gates)
    logs=ROOT/execution['log'];assert sha(logs)==execution['log_sha256']
    log=logs.read_text();assert log.count('SUBSTITUTION COMPLETE ')==56 and 'SCREEN COMPLETE advance False robot frames 83584' in log
    sources=cfg['source_sha256'].copy()
    sources.update(report['artifacts']);sources.update(arithmetic['inputs']);sources.update(verification['inputs'])
    sources[str(logs.relative_to(ROOT))]=sha(logs)
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in OUT.iterdir() if p.is_file()})
    for key,digest in sources.items():assert sha(ROOT/key)==digest,key
    result=dict(passed=True,protected_files=len(sources),source_runs=56,alternate_robot_frames=83584,
        original_parity_robot_frames=20896,fusion_distributions=verification['fusion_distributions'],
        independently_checked_raw_weight_rows=arithmetic['checked_rows'],new_recursive_runs=0,
        gate_passes=sum(g['passed'] for g in gates),gate_count=4,
        advance_to_recursion=report['advance_to_recursion'],source_sha256=sources,
        verifier_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ARCHIVE VERIFIED',len(sources),'protected files;',result['gate_passes'],'/ 4 gates; advance',result['advance_to_recursion'],flush=True)


if __name__=='__main__':main()
