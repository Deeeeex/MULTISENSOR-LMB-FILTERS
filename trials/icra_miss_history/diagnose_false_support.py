"""Post-outcome source-support diagnosis in the four largest screen losses."""
from collections import Counter
from pathlib import Path
import gzip
import hashlib
import json
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_v2v4real import domain

GCE='marked_gaussian_evidence';HISTORY=GCE+'_miss_history'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def read(path):
    with gzip.open(path,'rt') as handle:return json.load(handle)


def selected(data,t,n):
    run=data['runs'];index=n-1+2*(t-1)
    raw=np.asarray(run['rawEstimates'][index],float).reshape(-1,4)
    labels=np.asarray(run['labels'][index],int).reshape(2,-1).T
    assert len(raw)==len(labels)
    keep=domain(raw,np.asarray(data['positions'])[:,:,t-1])
    states=raw[keep];labels=labels[keep]
    assert np.array_equal(states,np.asarray(run['estimates'][index],float).reshape(-1,4))
    truth=np.asarray(data['truth'][t-1],float).reshape(4,-1).T
    cost=np.sum((truth[:,None,:2]-states[None,:,:2])**2,axis=2)
    i,j=linear_sum_assignment(np.minimum(cost,144.))
    good=j[cost[i,j]<144.]
    false=np.ones(len(states),bool);false[good]=False
    assert len(states)-len(good)==np.count_nonzero(false)
    nearest=cost.min(0) if len(truth) else np.full(len(states),np.inf)
    return states,labels,false,nearest


selection=json.loads((OUT/'SCREEN_SELECTION.json').read_text());assert selection['passed'] and not selection['advance']
cfg=json.loads((OUT/'stages/miss_history_screen.json').read_text())
units={u['sequence']:u for u in cfg['units']}
cases=[];inputs={};all_rows=[]
for dataset in ['v2v_development','v2x_val']:
    for condition in ['reliable','intermittent']:
        comparison=next(r for r in selection['comparisons'] if r['dataset']==dataset and r['condition']==condition and r['candidate']==HISTORY and r['reference']==GCE)
        sequence=max(comparison['sequence_deltas'],key=comparison['sequence_deltas'].get)
        assert comparison['sequence_deltas'][sequence]>0
        unit=units[sequence]
        paths=[OUT/'results/miss_history_screen'/f'{sequence}_{condition}_{HISTORY}.json.gz',ROOT/unit['reference_paths'][condition]]
        a,b=map(read,paths)
        if isinstance(b['runs'],list):b['runs']=next(r for r in b['runs'] if r['arm']==GCE)
        for path in paths:inputs[str(path.relative_to(ROOT))]=sha(path)
        run=a['runs'];record=np.asarray(run['iterationRecords'],float).reshape(-1,60)
        index={tuple(row[:4].astype(int)):row for row in record}
        local={tuple(row[:4].astype(int)):row for row in np.asarray(run['localIncrementRecords'],float).reshape(-1,12)}
        history={tuple(row[:4].astype(int)):row for row in np.asarray(run['negativeHistoryRecords'],float).reshape(-1,12)}
        counts=Counter();sources=Counter();qvalues=[];case_rows=[]
        for t in range(1,len(a['time'])+1):
            for n in [1,2]:
                states,labels,false,nearest=selected(a,t,n)
                _,oldlabels,oldfalse,_=selected(b,t,n)
                counts['candidate_false_outputs']+=int(false.sum());counts['original_false_outputs']+=int(oldfalse.sum())
                oldkeys={tuple(label) for label in oldlabels}
                for j in np.flatnonzero(false):
                    label=tuple(labels[j]);new_label=label not in oldkeys
                    counts['candidate_false_labels_absent_from_original_output']+=int(new_label)
                    counts['candidate_false_within_2m_of_a_truth']+=int(nearest[j]<=4)
                    if not new_label:continue
                    key=(t,n,*label)
                    if key not in index:
                        assert not a['delivered'][n-1][2-n][t-1]
                        counts['new_false_on_local_only_frame']+=1
                        continue
                    row=index[key];original=row[31:35].reshape(2,2).astype(int)
                    present=original[:,0]>0;active=row[13:15]>0
                    q=np.zeros(2);delta=np.zeros(2);discount=np.ones(2)
                    for s in np.flatnonzero(present):
                        source=n if s==0 else 3-n
                        lk=(t,source,*original[s]);loc=local[lk];h=history[lk]
                        delta[s]=loc[6];discount[s]=h[8]
                        if active[s] and loc[7] and loc[6]>0:q[s]=loc[5]*loc[9]
                    both=bool((present&active).all())
                    counts['new_false_on_fusion_frame']+=1
                    counts['new_false_both_participating']+=int(both)
                    counts['new_false_without_current_positive_other_source']+=int(not(q>0).any())
                    attenuated=present&active&(delta<0)&(row[52:54]>0)&(discount<1-1e-9)
                    for s in np.flatnonzero(attenuated):
                        other_q=float(q[1-s]);qvalues.append(other_q)
                        sources['attenuated_negative_sources']+=1
                        sources['other_positive_support_zero']+=int(other_q==0)
                        sources['other_positive_support_below_half']+=int(other_q<.5)
                        sources['other_positive_support_at_least_point9']+=int(other_q>=.9)
                    case_rows.append(dict(sequence=sequence,condition=condition,frame=t,robot=n,
                        label=list(map(int,label)),nearest_truth_distance=float(np.sqrt(nearest[j])) if np.isfinite(nearest[j]) else None,
                        existence=float(row[9]),both_participating=both,source_deltas=delta.tolist(),
                        positive_joint_support=q.tolist(),history_discounts=discount.tolist(),
                        attenuated_admitted_negative_sources=attenuated.tolist()))
        T=len(a['time'])
        lookup={(r['sequence'],r['condition'],r['arm']):r for r in selection['rows']}
        for arm,key in [(HISTORY,'candidate_false_outputs'),(GCE,'original_false_outputs')]:
            assert np.isclose(counts[key]*72/(2*T),lookup[sequence,condition,arm]['false2'],atol=1e-10,rtol=0)
        cases.append(dict(dataset=dataset,sequence=sequence,condition=condition,frames=T,
            ospa_delta=comparison['sequence_deltas'][sequence],counts=dict(counts),sources=dict(sources),
            mean_other_positive_support=float(np.mean(qvalues)) if qvalues else None))
        all_rows+=case_rows
        print('FALSE-SUPPORT CASE',json.dumps(cases[-1]),flush=True)
answer=dict(passed=True,selection_sha256=sha(OUT/'SCREEN_SELECTION.json'),cases=cases,rows=all_rows,inputs=inputs,
            source_sha256=sha(Path(__file__)),scope='Post-outcome four largest segment losses; newly output labels are a diagnostic subset, not a causal decomposition of the total false-output change. Positive joint support is posterior existence times current association mass when the local increment is positive.')
destination=OUT/'FALSE_SUPPORT_DIAGNOSTIC.json';assert not destination.exists();destination.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')
print('FALSE SUPPORT DIAGNOSIS COMPLETE',len(all_rows),'new false-label fusion rows',flush=True)
