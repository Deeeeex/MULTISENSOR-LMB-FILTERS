"""Check provenance, recording splits, and independent calibration equations."""
from collections import Counter
from pathlib import Path
import csv
import hashlib
import json
import math

import numpy as np
from scipy.optimize import root

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()


def close(a,b):
    assert math.isclose(a,b,rel_tol=2e-11,abs_tol=2e-11),(a,b)


def metrics(y,p):
    count=len(y)
    return dict(log_loss=math.fsum(-a*math.log(b)-(1-a)*math.log1p(-b) for a,b in zip(y,p))/count,
                brier=math.fsum((b-a)**2 for a,b in zip(y,p))/count)


def main():
    files={};reports={}
    names=['BOX_VISIBILITY_DIAGNOSTIC.json','RAW_SAMPLE_INSPECTION.json','RAW_GEOMETRY_RECHECK.json',
           'RAW_ALIGNMENT_SEQUENCE.json','RANGE_RECALL_CALIBRATION.json']
    scripts=['diagnose_boxes.py','inspect_raw_samples.py','recheck_raw_geometry.py',
             'check_alignment_sequence.py','calibrate_range_recall.py']
    for name,script in zip(names,scripts):
        report=json.loads((OUT/name).read_text());assert report['passed']
        assert report['source_sha256']==sha(OUT/script)
        for path,value in report['inputs'].items():
            if path in files:assert files[path]==value,path
            files[path]=value
        reports[name]=report
    for name,value in files.items():assert sha(ROOT/name)==value,name
    boxes=reports[names[0]]
    assert len(boxes['rows'])==305
    keys={(r['sequence'],r['condition'],r['group'],r['frame'],r['source'],*r['label']) for r in boxes['rows']}
    assert len(keys)==305
    for summary in boxes['summaries']:
        part=[r for r in boxes['rows'] if all(r[k]==summary[k] for k in ['sequence','condition','group'])]
        assert len(part)==summary['queries']
        assert sum(r['center_ray_occluded'] for r in part)==summary['center_ray_occluded']
    assert sha(OUT/'raw_samples.png')==reports[names[1]]['figure_sha256']
    alignment=reports['RAW_ALIGNMENT_SEQUENCE.json']
    assert len(alignment['rows'])==10 and all(r['lidar_equals_true_pose'] for r in alignment['rows'])
    calibration=reports[names[-1]]
    path=OUT/'RANGE_RECALL_ROWS.csv';assert sha(path)==calibration['rows_sha256']
    with path.open() as handle:rows=list(csv.DictReader(handle))
    assert len(rows)==calibration['rows']==36836
    x=np.array([float(r['range_m'])/40 for r in rows])
    y=np.array([int(r['matched_2m']) for r in rows])
    seq=np.array([r['sequence'] for r in rows]);rec=np.array([r['recording'] for r in rows])
    assert ((x>0)&(x<=1)).all()
    assert len(set(seq))==9 and len(set(rec))==6
    predicted=np.full(len(rows),np.nan);constant=np.full(len(rows),np.nan)
    max_parameter_error=0.
    for recording,model in [*calibration['folds'].items(),('FULL',calibration['full_fit'])]:
        mask=np.ones(len(rows),bool) if recording=='FULL' else rec!=recording
        assert set(model['training_sequences'])==set(seq[mask])
        assert model['train_rows']==int(mask.sum())
        counts=Counter(seq[mask]);nseq=len(counts)
        w=np.array([1/(nseq*counts[s]) if enabled else 0 for s,enabled in zip(seq,mask)])
        # Solve the strictly convex first-order conditions directly, rather
        # than calling the producer's L-BFGS objective or its fit function.
        def score(theta):
            z=theta[0]+theta[1]*x
            p=np.array([1/(1+math.exp(-float(v))) for v in z])
            return np.array([np.sum(w*(p-y)),np.sum(w*(p-y)*x)+.002*theta[1]])
        solution=root(score,[1.,-1.],tol=1e-10)
        assert solution.success and np.max(np.abs(score(solution.x)))<1e-10
        assert solution.x[1]<0
        expected=np.array([model['intercept'],model['slope']])
        error=float(np.max(np.abs(solution.x-expected)));assert error<2e-5
        max_parameter_error=max(max_parameter_error,error)
        close(float(np.sum(w*y)),model['constant'])
        if recording!='FULL':
            test=rec==recording
            predicted[test]=[1/(1+math.exp(-model['intercept']-model['slope']*v)) for v in x[test]]
            constant[test]=model['constant']
    assert np.isfinite(predicted).all()
    recomputed={arm:[] for arm in ['nominal','constant','range']}
    for item in calibration['sequence_metrics']:
        mask=seq==item['sequence'];target=y[mask].tolist()
        close(float(y[mask].mean()),item['recall_2m'])
        for arm,values in [('nominal',[.9]*sum(mask)),('constant',constant[mask].tolist()),('range',predicted[mask].tolist())]:
            result=metrics(target,values)
            for key in result:close(result[key],item[arm][key])
            recomputed[arm].append(result)
    for arm,values in recomputed.items():
        for key in ['log_loss','brier']:close(math.fsum(v[key] for v in values)/9,calibration['macro'][arm][key])
    gate=calibration['macro']['range']['log_loss']<calibration['macro']['constant']['log_loss']
    assert gate==calibration['calibration_gate_passed']
    answer=dict(passed=True,input_files_checked=len(files),visibility_queries=305,raw_alignment_frames=10,
                calibration_rows=36836,recording_folds=6,independent_stationary_point_max_parameter_error=max_parameter_error,
                calibration_gate_recomputed=gate,
                report_sha256={name:sha(OUT/name) for name in names},source_sha256=sha(Path(__file__)))
    path=OUT/'DIAGNOSTIC_VERIFICATION.json';assert not path.exists()
    path.write_text(json.dumps(answer,indent=2)+'\n')
    print('VISIBILITY DIAGNOSTIC VERIFIED',json.dumps(answer),flush=True)


if __name__=='__main__':main()
