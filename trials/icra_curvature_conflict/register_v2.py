"""Preserve failed v1; fix only strict comparison of unchanged missing fields."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'SCREEN_FREEZE_V2.json'; assert not destination.exists()
    prior=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    failed=json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    assert failed['completed'] and failed['returncode']==1
    assert not (OUT/'SCREEN_RESULTS.json').exists() and not list((OUT/'results').iterdir())
    for name,digest in prior['source_sha256'].items():assert sha(ROOT/name)==digest,name
    ports={}
    replacements={
        'conflict_math.py': [('assert np.array_equal(candidate[~trigger],rec[~trigger])',
                            'assert candidate[~trigger].tobytes() == rec[~trigger].tobytes()')],
        'screen.py': [('from conflict_math import','from conflict_math_v2 import'),
                      ("OUT / 'SCREEN_FREEZE.json'", "OUT / 'SCREEN_FREEZE_V2.json'")],
        'execute_screen.py': [("'SCREEN_EXECUTION.json'","'SCREEN_EXECUTION_V2.json'"),
                              ('screen.log','screen_v2.log'),("'SCREEN_FREEZE.json'","'SCREEN_FREEZE_V2.json'"),
                              ("'screen.py'","'screen_v2.py'")],
        'verify_screen.py': [("'SCREEN_FREEZE.json'","'SCREEN_FREEZE_V2.json'"),
                             ("'SCREEN_EXECUTION.json'","'SCREEN_EXECUTION_V2.json'")]
    }
    for name,patches in replacements.items():
        source=OUT/name; text=source.read_text()
        for old,new in patches: assert old in text; text=text.replace(old,new)
        target=source.with_name(source.stem+'_v2.py'); assert not target.exists(); target.write_text(text)
        ports[target.name]=dict(source=name,source_sha256=sha(source),output_sha256=sha(target),patches=patches)
    repair=dict(reason='Absent-source existence fields are JSON null / NumPy NaN; array_equal falsely rejected byte-identical records.',
        evidence=dict(first_source_rows=3210,missing_rows_per_column={'17':244,'18':244},
                      nonmissing_mismatches=0,entire_record_bytes_identical=True,completed_artifacts=0),
        change='Only the unchanged-row assertion uses exact byte comparison. Formula, event logic, protocol, cohorts and gates are unchanged.',
        ports=ports,failed_execution_sha256=sha(OUT/'SCREEN_EXECUTION.json'))
    (OUT/'MISSING_VALUE_CHECK_FIX.json').write_text(json.dumps(repair,indent=2)+'\n')
    sources=prior['source_sha256'].copy()
    for p in [OUT/'SCREEN_FREEZE.json',OUT/'SCREEN_EXECUTION.json',ROOT/failed['log'],
              OUT/'MISSING_VALUE_CHECK_FIX.json',*OUT.glob('*.py')]: sources[str(p.relative_to(ROOT))]=sha(p)
    for name,digest in sources.items(): assert sha(ROOT/name)==digest,name
    cfg=prior.copy(); cfg.update(created_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sources,
        repair='MISSING_VALUE_CHECK_FIX.json',previous_freeze_sha256=sha(OUT/'SCREEN_FREEZE.json'))
    destination.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('V2 FROZEN',len(cfg['cells']),'source runs;',len(sources),'protected files',flush=True)


if __name__=='__main__':main()
