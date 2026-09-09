"""Keep physical parity strict while identifying two unused legacy diagnostics."""
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
for p in ['icra_fusion_holdout','icra_marked_control']:
    sys.path.insert(0,str(OUT.parent/p))
from analyze_full import baseline

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
rows=[]
for condition in ['reliable','intermittent']:
    old,_,_,oldpath=baseline('development','0000',condition,'marked_lineage')
    oldrun=next(r for r in old['runs'] if r['arm']=='marked_lineage')
    newpath=OUT/'results/association_screen_benchmark_check'/f'0000_{condition}_marked_lineage.json.gz'
    with gzip.open(newpath,'rt') as h:newrun=json.load(h)['runs']
    a=np.asarray(oldrun['iterationRecords'],float).reshape(-1,26)
    b=np.asarray(newrun['iterationRecords'],float).reshape(-1,60)[:,:26]
    changed=~((a==b)|(np.isnan(a)&np.isnan(b)))
    assert np.flatnonzero(changed.any(0)).tolist()==[19,20]
    assert not b[:,19:21].any()
    equal_keys=[]
    for key in oldrun.keys() & newrun.keys():
        if key not in ['iterationRecords','runtimeSeconds']:
            assert oldrun[key]==newrun[key],key
            equal_keys.append(key)
    rows.append(dict(condition=condition,rows=len(a),changed_columns_1based=[20,21],
        changed_values=int(changed.sum()),exact_nonrecord_fields=sorted(equal_keys),
        original_path=str(oldpath.relative_to(ROOT)),original_sha256=sha(oldpath),
        new_path=str(newpath.relative_to(ROOT)),new_sha256=sha(newpath)))

old=OUT/'audit_screen_assessment.py';new=OUT/'audit_screen_assessment_v2.py'
assert not new.exists()
before="""                    assert np.array_equal(np.asarray(run['iterationRecords'])[:, :26], np.asarray(oldrun['iterationRecords']).reshape(-1, 26))
                    parity.append(dict(sequence=seq, condition=condition, arm=arm, exact_fields=keys + ['iterationRecords_first26'], robot_frames=2*T))"""
after="""                    new_records = np.asarray(run['iterationRecords'], float).reshape(-1, 60)[:, :26]
                    old_records = np.asarray(oldrun['iterationRecords'], float).reshape(-1, 26)
                    columns = [i for i in range(26) if i not in [19, 20]]
                    assert np.array_equal(new_records[:, columns], old_records[:, columns], equal_nan=True)
                    assert not new_records[:, 19:21].any()
                    # No-age assigns r0 independently of the direct-evidence ceiling.
                    assert np.array_equal(new_records[:, 6], new_records[:, 7])
                    parity.append(dict(sequence=seq, condition=condition, arm=arm, exact_fields=keys,
                        exact_record_columns_1based=[i+1 for i in columns],
                        unused_diagnostic_columns_1based=[20, 21], robot_frames=2*T))"""
value=old.read_text();assert value.count(before)==1
new.write_text(value.replace(before,after))
result=dict(passed=True,rows=rows,reason='The original reviewer runner already sets directEvidenceCeiling to zero. No-age does not use this ceiling: its existence is r0, and every physical output plus every other recorded field is exactly unchanged.',
    initial_auditor_sha256=sha(old),revised_auditor_sha256=sha(new),
    replacement=dict(before=before,after=after),
    evidence={str(p.relative_to(ROOT)):sha(p) for p in [
        OUT.parent/'icra_marked_iteration/fuseMarkedInputsStable.m',
        OUT.parent/'icra_reviewer_revision/runReviewerReplay.m',OUT/'runScreenedAssociation.m',Path(__file__)]})
destination=OUT/'BENCHMARK_DIAGNOSTIC_PARITY.json';assert not destination.exists()
destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print('NO-AGE PHYSICAL PARITY PROVEN; TWO UNUSED CEILING DIAGNOSTICS IDENTIFIED')
