"""Check the reported bounded mark sums without matrix multiplication.

The screen log contains NumPy matmul runtime warnings in the raw-W audit.
This check neither suppresses those warnings nor changes frozen outputs.
It evaluates every available weighted sum as explicit elementwise terms.
"""
from pathlib import Path
from collections import defaultdict
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.io import loadmat

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'WEIGHT_ARITHMETIC_VERIFICATION.json';assert not destination.exists()
    report_path=OUT/'SCREEN_RESULTS.json';report=json.loads(report_path.read_text());assert report['passed']
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    checks=[];inputs={}
    for cell in cfg['cells']:
        diagnostic=next(d for d in report['diagnostics'] if d['cell']==cell)
        if not diagnostic['raw_weights']['available']:continue
        path=ROOT/cell['path'];assert sha(path)==cell['sha256']
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        run=data['runs'];local=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
        ratios_path=ROOT/cell['ratios_path'];ratios=loadmat(ratios_path)['likelihoodRatios']
        assert sha(ratios_path)==cfg['source_sha256'][cell['ratios_path']]
        groups=defaultdict(list)
        for i,row in enumerate(local):groups[tuple(row[:2].astype(int))].append(i)
        total=0;maximum=0.;error=0.;r_error=0.
        for (t,n),idx in groups.items():
            raw=np.asarray(run['localAssociationWeights'][n-1+2*(t-1)],float)
            ratio=np.asarray(ratios[n-1,t-1],float).ravel()
            assert np.isfinite(ratio).all() and (ratio>0).all()
            marks=np.maximum(0,np.tanh(.5*np.log(ratio)))
            assert np.isfinite(marks).all() and (marks>=0).all() and (marks<=1).all()
            rows=local[idx]
            if not raw.size:assert not rows[:,8:10].any();continue
            W=raw.reshape(len(rows),len(marks)+1)
            assert np.isfinite(W).all() and (W>=0).all()
            W=W/W.sum(1,keepdims=True)
            for row,weights in zip(rows,W):
                terms=weights[1:]*marks
                assert np.isfinite(terms).all() and (terms>=0).all() and (terms<=1).all()
                value=math.fsum(terms);assert math.isfinite(value) and 0<=value<=1+1e-12
                support=min(max(value,0.),1.)*row[7]
                error=max(error,abs(support-row[8]));assert abs(support-row[8])<2e-14
                association=min(max(math.fsum(weights[1:]),0.),1.)*row[7]
                assert abs(association-row[9])<2e-14
                joint=math.fsum(row[5]*weights[1:])*row[7]
                r_error=max(r_error,abs(joint-row[5]*row[9]))
                assert abs(joint-row[5]*row[9])<2e-14
                maximum=max(maximum,value);total+=1
        assert total==diagnostic['raw_weights']['local_rows_with_W']
        checks.append(dict(sequence=cell['sequence'],condition=cell['condition'],backend=cell['backend'],
            checked_rows=total,maximum_bounded_mark_sum=maximum,maximum_mark_error=error,maximum_joint_error=r_error))
        inputs[cell['path']]=sha(path);inputs[cell['ratios_path']]=sha(ratios_path)
    assert len(checks)==18
    result=dict(passed=True,source_runs=len(checks),checked_rows=sum(r['checked_rows'] for r in checks),checks=checks,
        interpretation='All available mark sums and joint association sums are finite and reproduce saved values using elementwise products and math.fsum. The NumPy warning root cause is not established; no frozen source, data, tolerance or decision was changed.',
        report_sha256=sha(report_path),source_sha256=sha(Path(__file__)),inputs=inputs)
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ELEMENTWISE WEIGHT ARITHMETIC VERIFIED',result['checked_rows'],'local rows',flush=True)


if __name__=='__main__':main()
