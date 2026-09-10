"""Join remote-only retained outputs to exact current local pruning records."""
from collections import Counter
from datetime import datetime,timezone
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


def keyed(values,width):
    rows=np.asarray(values,float).reshape(-1,width)
    out={tuple(map(int,row[:4])):row for row in rows};assert len(out)==len(rows)
    return out


def main():
    destination=OUT/'REIMPORT_RESULTS.json';assert not destination.exists()
    cfg=json.loads((OUT/'REIMPORT_FREEZE.json').read_text())
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    rows=[];summaries=[]
    for cell in cfg['cells']:
        with gzip.open(ROOT/cell['path'],'rt') as stream:data=json.load(stream)
        run=data['runs'];inc=keyed(run['localIncrementRecords'],12);local=keyed(run['localGaussianRecords'],32)
        sources=keyed(run['fusionSourceRecords'],8);fused=keyed(run['fusionOutputRecords'],19)
        records=keyed(run['iterationRecords'],60)
        prior_cycle={};current_cycle={};last_t=0;cell_rows=[]
        for key,row in sorted(fused.items()):
            t,n,bt,bl=key
            if t!=last_t:prior_cycle=current_cycle if t==last_t+1 else {};current_cycle={};last_t=t
            s=sources[key]
            if row[4]<=.001 or s[4]>0:continue
            assert s[6]>0 and key not in local and np.asarray(data['delivered'])[n-1,2-n,t-1]
            rec=records[key];other=(t,3-n,int(s[6]),int(s[7]))
            assert other in local and other in inc and abs(rec[18]-inc[other][5])<1e-12
            current=inc.get(key);reintroduced=current is not None
            if reintroduced:assert current[5]<=.001
            active=rec[13]>0;bound=float(rec[17]) if active else None
            if active:assert bound==.001
            gt_ids=np.asarray(data['truthIds'][t-1]).ravel();ix=np.flatnonzero(gt_ids==5).item()
            xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,ix]
            cycle=0
            if reintroduced:
                cycle=prior_cycle.get((n,bt,bl),0)+1;current_cycle[n,bt,bl]=cycle
            event=dict(condition=cell['condition'],backend=cell['backend'],frame=t,robot=n,
                birth_frame=bt,birth_location=bl,remote_birth_frame=int(s[6]),remote_birth_location=int(s[7]),
                in_current_prediction=reintroduced,same_frame_deleted=reintroduced,
                current_local_prior_r=None if current is None else float(current[4]),
                current_local_posterior_r=None if current is None else float(current[5]),
                current_local_delta=None if current is None else float(current[6]),
                current_local_opportunity=None if current is None else bool(current[7]),
                current_local_pd=None if current is None else float(current[10]),
                censor_participates=active,censor_bound=bound,
                bound_exceeds_local=bool(reintroduced and active and current[5]<bound),
                remote_r=float(inc[other][5]),returned_r=float(row[4]),
                ordinary_self_weight=float(rec[13]),ordinary_remote_weight=float(rec[14]),
                recency_self_weight=float(rec[11]),recency_remote_weight=float(rec[12]),
                target_distance=math.hypot(*(row[5:7]-xy)),near_target=math.hypot(*(row[5:7]-xy))<=2.,
                consecutive_returns=cycle)
            cell_rows.append(event)
        for window,start,end in [('full',1,240),('original_window',53,122)]:
            for scope in ['all','target_neighbourhood']:
                chosen=[r for r in cell_rows if start<=r['frame']<=end and (scope=='all' or r['near_target'])]
                deleted=[r for r in chosen if r['same_frame_deleted']]
                qualified=[r for r in deleted if r['censor_participates']]
                positives=[r['current_local_posterior_r'] for r in qualified]
                summaries.append(dict(condition=cell['condition'],backend=cell['backend'],window=window,scope=scope,
                    remote_only_retained=len(chosen),same_frame_deleted=len(deleted),absent_from_prediction=len(chosen)-len(deleted),
                    qualified_censor=len(qualified),censor_exceeds_actual=sum(r['bound_exceeds_local'] for r in qualified),
                    qualified_with_opportunity=sum(r['current_local_opportunity'] for r in qualified),
                    minimum_deleted_r=min(positives,default=None),maximum_deleted_r=max(positives,default=None),
                    mean_deleted_r=math.fsum(positives)/len(positives) if positives else None,
                    at_least_two_consecutive_returns=sum(r['consecutive_returns']>=2 for r in deleted),
                    maximum_consecutive_returns=max((r['consecutive_returns'] for r in deleted),default=0)))
        rows.extend(cell_rows)
        print('REIMPORT JOINED',cell['condition'],cell['backend'],len(cell_rows),'remote returns',flush=True)
    table=OUT/'results/v2/REIMPORT_EVENTS.csv.gz';assert not table.exists()
    with gzip.open(table,'wt',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    for name,digest in cfg['source_sha256'].items():assert sha(ROOT/name)==digest,name
    result=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),event_rows=len(rows),
        summaries=summaries,events_path=str(table.relative_to(ROOT)),events_sha256=sha(table),freeze_sha256=sha(OUT/'REIMPORT_FREEZE.json'),
        source_sha256=cfg['source_sha256'],scope='Observed local-deletion / remote-reimport joins; no substituted output')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('REIMPORT CENSUS COMPLETE',len(rows),'events',flush=True)


if __name__=='__main__':main()
