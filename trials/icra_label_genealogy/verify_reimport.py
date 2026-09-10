"""Check all native remote-only rows and independently count delete/reimport runs."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def key(row):return tuple(map(int,row[:4]))


def main():
    destination=OUT/'REIMPORT_VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'REIMPORT_FREEZE.json').read_text());report=json.loads((OUT/'REIMPORT_RESULTS.json').read_text())
    assert report['passed'] and report['freeze_sha256']==sha(OUT/'REIMPORT_FREEZE.json')
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    assert sha(ROOT/report['events_path'])==report['events_sha256']
    with gzip.open(ROOT/report['events_path'],'rt',newline='') as stream:table=list(csv.DictReader(stream))
    known={}
    for row in table:
        entry=(row['condition'],row['backend'],int(row['frame']),int(row['robot']),int(row['birth_frame']),int(row['birth_location']))
        assert entry not in known;known[entry]=row
    visited=set();all_rows=[]
    with (OUT/'POPULATION.csv').open(newline='') as stream:population=list(csv.DictReader(stream))
    for cell in cfg['cells']:
        with gzip.open(ROOT/cell['path'],'rt') as stream:data=json.load(stream)
        run=data['runs'];updates=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
        inputs={key(r):r for r in updates};remaining={key(r) for r in np.asarray(run['localGaussianRecords'],float).reshape(-1,32)}
        fusion=np.asarray(run['fusionOutputRecords'],float).reshape(-1,19)
        mappings={key(r):r for r in np.asarray(run['fusionSourceRecords'],float).reshape(-1,8)}
        diagnostics={key(r):r for r in np.asarray(run['iterationRecords'],float).reshape(-1,60)}
        candidate_keys={key(r) for r in fusion if r[4]>.001 and key(r) not in remaining}
        pruned_keys={key(r) for r in updates if r[5]<=.001}
        same_frame=candidate_keys&pruned_keys
        cycles={};rebuilt=[]
        for row in sorted(fusion,key=key):
            k=key(row)
            if k not in candidate_keys:continue
            t,n,bt,bl=k;m=mappings[k];d=diagnostics[k]
            assert m[4]==m[5]==0 and m[6]>0 and bool(np.asarray(data['delivered'])[n-1,2-n,t-1])
            ref=(t,3-n,int(m[6]),int(m[7]));assert ref in remaining
            local=inputs.get(k);deleted=k in same_frame
            assert (local is not None)==deleted
            cycle=0
            if deleted:
                last,count=cycles.get((n,bt,bl),(-1,0));cycle=count+1 if last==t-1 else 1
                cycles[n,bt,bl]=(t,cycle)
            exists=bool(d[13]>0);bound=float(d[17]) if exists else None
            if exists:assert bound==.001
            ix=list(np.asarray(data['truthIds'][t-1]).ravel()).index(5)
            truth=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:,ix]
            distance=math.sqrt((row[5]-truth[0])**2+(row[6]-truth[1])**2)
            expected=dict(condition=cell['condition'],backend=cell['backend'],frame=t,robot=n,birth_frame=bt,birth_location=bl,
                remote_birth_frame=int(m[6]),remote_birth_location=int(m[7]),in_current_prediction=deleted,same_frame_deleted=deleted,
                current_local_prior_r=None if local is None else float(local[4]),current_local_posterior_r=None if local is None else float(local[5]),
                current_local_delta=None if local is None else float(local[6]),current_local_opportunity=None if local is None else bool(local[7]),
                current_local_pd=None if local is None else float(local[10]),censor_participates=exists,censor_bound=bound,
                bound_exceeds_local=deleted and exists and local[5]<bound,remote_r=float(inputs[ref][5]),returned_r=float(row[4]),
                ordinary_self_weight=float(d[13]),ordinary_remote_weight=float(d[14]),recency_self_weight=float(d[11]),recency_remote_weight=float(d[12]),
                target_distance=distance,near_target=distance<=2.,consecutive_returns=cycle)
            entry=(cell['condition'],cell['backend'],*k);saved=known[entry];visited.add(entry)
            assert saved.keys()==expected.keys()
            for field,value in expected.items():
                if value is None:assert saved[field]==''
                elif isinstance(value,(bool,np.bool_)):assert saved[field]==str(bool(value))
                elif isinstance(value,(int,float,np.number)):assert abs(float(saved[field])-value)<1e-10,(entry,field)
                else:assert saved[field]==value
            rebuilt.append(expected)
        assert len(rebuilt)==sum(int(r['remote_only_retained']) for r in population if (r['condition'],r['backend'])==(cell['condition'],cell['backend']))
        all_rows.extend(rebuilt);print('INDEPENDENT REIMPORT VERIFIED',cell['condition'],cell['backend'],len(rebuilt),flush=True)
    assert visited==set(known) and len(visited)==report['event_rows']
    for summary in report['summaries']:
        lo,hi=(1,240) if summary['window']=='full' else (53,122)
        selection=[r for r in all_rows if (r['condition'],r['backend'])==(summary['condition'],summary['backend'])
                   and lo<=r['frame']<=hi and (summary['scope']=='all' or r['near_target'])]
        deleted=[r for r in selection if r['same_frame_deleted']];qualified=[r for r in deleted if r['censor_participates']]
        values=[r['current_local_posterior_r'] for r in qualified]
        expected=dict(remote_only_retained=len(selection),same_frame_deleted=len(deleted),absent_from_prediction=len(selection)-len(deleted),
            qualified_censor=len(qualified),censor_exceeds_actual=sum(bool(r['bound_exceeds_local']) for r in qualified),
            qualified_with_opportunity=sum(bool(r['current_local_opportunity']) for r in qualified),minimum_deleted_r=min(values,default=None),
            maximum_deleted_r=max(values,default=None),mean_deleted_r=math.fsum(values)/len(values) if values else None,
            at_least_two_consecutive_returns=sum(r['consecutive_returns']>=2 for r in deleted),maximum_consecutive_returns=max((r['consecutive_returns'] for r in deleted),default=0))
        for field,value in expected.items():
            if value is None:assert summary[field] is None
            elif isinstance(value,float):assert abs(summary[field]-value)<1e-14
            else:assert summary[field]==value
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    result=dict(passed=True,event_rows=len(visited),summary_rows=len(report['summaries']),
        report_sha256=sha(OUT/'REIMPORT_RESULTS.json'),freeze_sha256=sha(OUT/'REIMPORT_FREEZE.json'),
        verifier_sha256=sha(Path(__file__)),method='Independent pruned-local and retained-fusion key intersection, every native event field, consecutive-label timelines and all summary counts')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('REIMPORT VERIFICATION PASSED',len(visited),'events',flush=True)


if __name__=='__main__':main()
