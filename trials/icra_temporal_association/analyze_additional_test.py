"""Evaluate the fixed five-arm additional cohort and the geometric breakdown."""
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json
import sys

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
TEST=OUT.parent/'icra_association_test'
sys.path.insert(0,str(OUT.parent/'icra_fusion_holdout'))
from analyze_holdout import score
from diagnose_association_v2 import truth_assignment,domain
from identity_metrics import evaluate_identities
from association_aggregates import summarize,METRICS

BASE='marked_gaussian_evidence'
GS=BASE+'_guarded_scalar'
SELECTED=BASE+'_assoc_quality_nis'
CONTROL=GS+'_assoc_quality_nis'
ARMS=['marked_lineage',BASE,GS,SELECTED,CONTROL]
FOCUS=ARMS[1:]
STAGE='association_screen_selected_test'
FINAL_FIELDS=['final_assigned_estimates','label_truth_transition_opportunities','label_truth_switches',
              'truth_label_transition_opportunities','truth_label_switches']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def final_identities(data):
    run=data['runs'];positions=np.asarray(data['positions'])
    counts={c:Counter() for c in [2,12]}
    previous={(n,c):{} for n in [0,1] for c in counts}
    for t in range(len(data['time'])):
        truth=np.asarray(data['truth'][t],float).reshape(4,-1).T
        ids=np.asarray(data['truthIds'][t]).reshape(-1)
        for n in [0,1]:
            raw=np.asarray(run['rawEstimates'][n+2*t],float).reshape(-1,4)
            labels=np.asarray(run['labels'][n+2*t],int).reshape(2,-1).T
            keep=domain(raw,positions[:,:,t]);raw,labels=raw[keep],labels[keep]
            assert np.array_equal(raw,np.asarray(run['estimates'][n+2*t],float).reshape(-1,4))
            for cutoff,stat in counts.items():
                assignment=truth_assignment(raw,truth,ids,cutoff)
                now={tuple(labels[i]):value for i,value in assignment.items()}
                last=previous[n,cutoff]
                stat['final_assigned_estimates']+=len(now)
                for label in now.keys() & last.keys():
                    stat['label_truth_transition_opportunities']+=1
                    stat['label_truth_switches']+=int(now[label]!=last[label])
                current_truth,old_truth={v:k for k,v in now.items()},{v:k for k,v in last.items()}
                for identity in current_truth.keys() & old_truth.keys():
                    stat['truth_label_transition_opportunities']+=1
                    stat['truth_label_switches']+=int(current_truth[identity]!=old_truth[identity])
                previous[n,cutoff]=now
    return {str(c):dict(v) for c,v in counts.items()}


def main():
    destination=OUT/'ADDITIONAL_TEST_ANALYSIS.json';assert not destination.exists()
    audit_path=OUT/('audit_'+STAGE+'.json');audit=json.loads(audit_path.read_text())
    cfgpath=OUT/'stages'/(STAGE+'.json');cfg=json.loads(cfgpath.read_text())
    runtimepath=OUT/('runtime_'+STAGE+'.json');runtime=json.loads(runtimepath.read_text())
    assert audit['passed'] and len(audit['rows'])==140
    assert sha(cfgpath)==audit['config_sha256'] and sha(runtimepath)==audit['runtime_sha256']
    assert cfg['arms']==ARMS and cfg['selected_primary']==SELECTED
    assert len(cfg['units'])==len(runtime)==14 and sum(u['frames'] for u in cfg['units'])==2172
    assert all(r['returncode']==0 and r['completion_line'] and r['files']==10 for r in runtime)
    inputs={str(p.relative_to(ROOT)):sha(p) for p in [audit_path,cfgpath,runtimepath]}
    for group in [cfg['source_sha256'],audit['auditor_sha256'],audit['inputs']]:
        for name,expected in group.items():
            if name in inputs:assert inputs[name]==expected,name
            else:assert sha(ROOT/name)==expected,name
            inputs[name]=expected
    selection_path=OUT/'SCREENED_DEVELOPMENT_SELECTION.json';selection=json.loads(selection_path.read_text())
    cohort_path=TEST/'COHORT_FREEZE.json';cohort=json.loads(cohort_path.read_text())
    assert selection['advance'] and selection['selected']['arm']==SELECTED
    assert sha(selection_path)==cohort['development_selection_sha256']==cfg['selection_sha256']
    assert datetime.fromisoformat(selection['created_utc']) < datetime.fromisoformat(cohort['created_utc'])
    for p in [selection_path,cohort_path,TEST/'PROTOCOL.md']:
        inputs[str(p.relative_to(ROOT))]=sha(p)
    units={u['sequence']:u for u in cfg['units']}
    rows=[];identities=[];final_rows=[];opportunities=[]
    overlap_by_sequence={}
    for row in audit['rows']:
        unit=units[row['sequence']]
        path=OUT/'results'/STAGE/f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
        assert sha(path)==inputs[str(path.relative_to(ROOT))]
        with gzip.open(path,'rt') as handle:data=json.load(handle)
        run=data['runs'];T=len(data['time']);assert T==row['frames']==unit['frames']
        final=final_identities(data)
        final_rows.append(dict(sequence=row['sequence'],condition=row['condition'],arm=row['arm'],cutoffs=final))
        if row['arm'] in FOCUS:
            identity=evaluate_identities(data)
            for cutoff in ['2','12']:
                assert all(identity['cutoffs'][cutoff].get(k,0)==final[cutoff].get(k,0) for k in FINAL_FIELDS)
            identities.append(dict(dataset='v2x_test',sequence=row['sequence'],condition=row['condition'],arm=row['arm'],**identity))
        rows.append(dict(dataset='v2x_test',cohort='additional_test',sequence=row['sequence'],scene=unit['scene'],
            recording=unit['recording'],condition=row['condition'],arm=row['arm'],frames=T,**{k:row[k] for k in METRICS},
            wire_bytes=int(run['totalWireBytes']),raw_payload_bytes=int(sum(run['rawPayloadBytes'])),
            delivered_raw_bytes=int(sum(run['deliveredRawBytes'])),split_branches=len(run['associationSplits'])))
        positions=np.asarray(data['positions'])
        distance2=np.sum((positions[:,1,:]-positions[:,0,:])**2,axis=0)
        overlap=distance2<80**2
        if row['sequence'] in overlap_by_sequence:
            assert overlap_by_sequence[row['sequence']]==overlap.tolist()
        else:overlap_by_sequence[row['sequence']]=overlap.tolist()
        per_frame={k:np.zeros(T) for k in METRICS}
        for t in range(T):
            for n in [0,1]:
                value=score(data['truth'][t],run['estimates'][n+2*t])
                assert np.isclose(value['ospa'],run['ospa'][n][t],atol=1e-8,rtol=1e-9)
                for key in METRICS:per_frame[key][t]+=value[key]
        for stratum,mask in [('overlapping_disks',overlap),('nonoverlapping_disks',~overlap)]:
            count=int(mask.sum())
            sums={k:float(v[mask].sum()) for k,v in per_frame.items()}
            opportunities.append(dict(sequence=row['sequence'],scene=unit['scene'],recording=unit['recording'],
                condition=row['condition'],arm=row['arm'],stratum=stratum,paired_frames=count,robot_frames=2*count,
                metric_sums=sums,means={k:v/(2*count) if count else None for k,v in sums.items()},
                wire_bytes=int(np.asarray(run['wireBytes'])[mask].sum()),
                raw_payload_bytes=int(np.asarray(run['rawPayloadBytes'])[mask].sum()),
                split_branches=sum(bool(mask[int(event[0])-1]) for event in run['associationSplits'])))
        print('ADDITIONAL TEST ANALYZED',row['sequence'],row['condition'],row['arm'],flush=True)
    assert len(rows)==len(final_rows)==140 and len(identities)==112 and len(opportunities)==280
    comparisons=[(SELECTED,BASE),(CONTROL,GS),(SELECTED,CONTROL),(SELECTED,'marked_lineage')]
    aggregates,paired=summarize(rows,identities,FOCUS,comparisons,[('v2x_test',lambda r:True)])
    for item in aggregates:
        part=[r for r in rows if r['arm']==item['arm'] and r['condition']==item['condition']]
        assert len(part)==14 and item['frames']==2172 and item['recording_groups']==5
        item['communication']={k:sum(r[k] for r in part) for k in ['wire_bytes','raw_payload_bytes','delivered_raw_bytes','split_branches']}
        item['final_identity']={}
        for cutoff in ['2','12']:
            counts=Counter()
            for r in final_rows:
                if r['arm']==item['arm'] and r['condition']==item['condition']:counts.update(r['cutoffs'][cutoff])
            item['final_identity'][cutoff]=dict(counts)
    geometry=[]
    for stratum in ['overlapping_disks','nonoverlapping_disks']:
        for condition in ['reliable','intermittent']:
            for arm in ARMS:
                part=[r for r in opportunities if r['arm']==arm and r['condition']==condition and r['stratum']==stratum]
                assert len(part)==14
                count=sum(r['robot_frames'] for r in part)
                available=[r for r in part if r['robot_frames']]
                geometry.append(dict(stratum=stratum,condition=condition,arm=arm,segments_with_frames=len(available),
                    paired_frames=count//2,robot_frames=count,
                    frame_weighted={k:sum(r['metric_sums'][k] for r in part)/count if count else None for k in METRICS},
                    sequence_macro_nonempty={k:float(np.mean([r['means'][k] for r in available])) if available else None for k in METRICS},
                    wire_bytes=sum(r['wire_bytes'] for r in part),raw_payload_bytes=sum(r['raw_payload_bytes'] for r in part),
                    split_branches=sum(r['split_branches'] for r in part)))
    lookup={(r['arm'],r['condition']):r for r in aggregates}
    outcomes={}
    for reference in [BASE,CONTROL,'marked_lineage']:
        differences={c:lookup[SELECTED,c]['sequence_macro']['ospa']-lookup[reference,c]['sequence_macro']['ospa'] for c in ['reliable','intermittent']}
        outcomes[reference]=dict(ospa_difference=differences,improves_both=all(v<0 for v in differences.values()))
    result=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),selected=SELECTED,matched_control=CONTROL,
        unique_segments=14,paired_frames=2172,collection_dates=5,rows=rows,aggregate=aggregates,
        identity_rows=identities,final_identity_rows=final_rows,paired_recording=paired,
        geometry_rows=opportunities,geometry_aggregate=geometry,overlap_by_sequence=overlap_by_sequence,
        comparisons=outcomes,transfer_improves_both_conditions=outcomes[BASE]['improves_both'],inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),OUT/'association_aggregates.py',OUT/'identity_metrics.py',
            OUT/'diagnose_association_v2.py',OUT.parent/'icra_fusion_holdout/analyze_holdout.py']},
        geometry_definition='Primary result uses all 14 segments. Secondary overlap uses platform separation <80 m, before inspecting detections or errors; means over eligible robot-frames.',
        interval_definition='10000 paired collection-date-macro bootstrap samples, seed 8301; five date groups; fixed selected settings and calibration.',
        exposure='This cohort was registered before raw inspection and the selected QN method was frozen before acquisition. No test-based tuning or method selection.')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for name,data_rows in [('additional_test_scores.csv',rows),('additional_test_geometry.csv',geometry)]:
        with (OUT/name).open('w') as handle:
            flat=[]
            for r in data_rows:
                out={}
                for k,v in r.items():
                    if isinstance(v,dict):out.update({k+'_'+kk:vv for kk,vv in v.items()})
                    else:out[k]=v
                flat.append(out)
            names=list(dict.fromkeys(k for r in flat for k in r))
            writer=csv.DictWriter(handle,fieldnames=names,lineterminator='\n');writer.writeheader();writer.writerows(flat)
    print('ADDITIONAL TEST COMPLETE',json.dumps(outcomes,allow_nan=False),flush=True)
    for r in aggregates:print(r['condition'],r['arm'],r['sequence_macro']['ospa'],flush=True)


if __name__=='__main__':main()
