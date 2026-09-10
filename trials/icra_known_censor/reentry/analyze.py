"""Forward native-state ledger for all remote-only arrivals and later returns."""
from collections import Counter,defaultdict
from datetime import datetime,timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def key(row):return tuple(map(int,row[:4]))

def matrix(rows):
    out={key(r):r for r in rows};assert len(out)==len(rows);return out

def main():
    destination=OUT/'RESULTS.json';assert not destination.exists()
    cfg=json.loads((OUT/'FREEZE.json').read_text())
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    summaries=[];artifacts=[];examples=[];count=0;directory=OUT/'results';directory.mkdir()
    for cell in cfg['cells']:
        with gzip.open(ROOT/cell['path'],'rt') as stream:data=json.load(stream)
        run=data['runs'];inc=matrix(run['localIncrementRecords']);local=matrix(run['localGaussianRecords']);fused=matrix(run['fusionOutputRecords'])
        sources=matrix(run['fusionSourceRecords']);records=matrix(run['iterationRecords']);refinements=matrix(run['knownCensorRecords'])
        final={};by_t=defaultdict(list);inc_at=defaultdict(list)
        for k,r in inc.items():inc_at[k[0]].append(k)
        for k,r in fused.items():by_t[k[0]].append(k)
        for t in range(1,241):
            for n in [1,2]:
                if data['delivered'][n-1][2-n][t-1]:
                    for k in by_t[t]:
                        if k[1]==n:final[k]=float(fused[k][4])
                else:
                    for k in inc_at[t]:
                        if k[1]==n and k in local:final[k]=float(inc[k][5])
        last={};events=[]
        for t in range(1,241):
            ids=data['truthIds'][t-1]
            if ids and isinstance(ids[0],list):ids=[x for row in ids for x in row]
            target=ids.index(5);xy=[data['truth'][t-1][0][target],data['truth'][t-1][1][target]]
            for k in sorted(by_t[t]):
                _,n,bt,bl=k;r=fused[k];s=sources[k];rec=records[k]
                if r[4]<=.001 or s[4]>0:continue
                assert k not in local and s[4:6]==[0,0] and s[6]>0
                remote_key=(t,3-n,int(s[6]),int(s[7]));assert remote_key in local and inc[remote_key][5]>.001
                current=inc.get(k);old=last.get((n,bt,bl))
                kind='same_frame_pruned' if current is not None else 'first_local_arrival' if old is None else 'return_after_gap'
                if current is not None:assert current[5]<=.001
                if current is None and old is not None:
                    assert old['final_r'] is None or old['final_r']<=.001
                direct=kind=='return_after_gap' and old['frame']==t-1 and old['refinement_caused_prune'] and old['peer_retained'] and rec[13]>0 and rec[17]==.001 and remote_key[2:]==(bt,bl)
                event=dict(condition=cell['condition'],arm=cell['arm'],frame=t,robot=n,birth_frame=bt,birth_location=bl,
                    remote_birth_frame=int(s[6]),remote_birth_location=int(s[7]),remote_input_r=float(inc[remote_key][5]),kind=kind,returned_r=float(r[4]),
                    near_target=math.hypot(r[5]-xy[0],r[6]-xy[1])<=2.,censor_participates=rec[13]>0,
                    current_censor=float(rec[17]) if rec[13]>0 else None,
                    last_prediction_frame=None if old is None else old['frame'],
                    gap=None if old is None or current is not None else t-old['frame'],
                    last_local_r=None if old is None else old['local_r'],last_local_opportunity=None if old is None else old['opportunity'],
                    last_final_r=None if old is None else old['final_r'],
                    last_refinement_old_r=None if old is None else old['refinement_old_r'],
                    last_refinement_actual_r=None if old is None else old['refinement_actual_r'],
                    last_refinement_caused_prune=False if old is None else old['refinement_caused_prune'],
                    peer_retained_at_last_prediction=False if old is None else old['peer_retained'],
                    peer_r_at_last_prediction=None if old is None else old['peer_r'],direct_one_frame_rebound=bool(direct))
                events.append(event)
                if direct and 53<=t<=122 and event['near_target'] and not any(x['condition']==cell['condition'] and x['arm']==cell['arm'] for x in examples):
                    examples.append(event.copy())
            for k in inc_at[t]:
                _,n,bt,bl=k;row=inc[k];r=final.get(k);e=refinements.get(k);peer=final.get((t,3-n,bt,bl))
                last[n,bt,bl]=dict(frame=t,local_r=float(row[5]),opportunity=bool(row[7]),final_r=r,
                    refinement_old_r=None if e is None else float(e[4]),refinement_actual_r=None if e is None else float(e[5]),
                    refinement_caused_prune=e is not None and e[4]>.001 and e[5]<=.001,
                    peer_retained=peer is not None and peer>.001,peer_r=peer)
        for window,left,right in [('full',1,240),('original_window',53,122)]:
            for scope in ['all','target_neighbourhood']:
                chosen=[e for e in events if left<=e['frame']<=right and (scope=='all' or e['near_target'])];classes=Counter(e['kind'] for e in chosen)
                gaps=Counter(e['gap'] for e in chosen if e['kind']=='return_after_gap')
                summaries.append(dict(condition=cell['condition'],arm=cell['arm'],window=window,scope=scope,remote_only_retained=len(chosen),
                    **{name:classes[name] for name in ['same_frame_pruned','first_local_arrival','return_after_gap']},
                    after_refinement_prune=sum(e['kind']=='return_after_gap' and e['last_refinement_caused_prune'] for e in chosen),
                    direct_one_frame_rebound=sum(e['direct_one_frame_rebound'] for e in chosen),
                    gap_counts={str(k):v for k,v in sorted(gaps.items())}))
        path=directory/(cell['condition']+'_'+cell['arm']+'.csv.gz')
        with gzip.open(path,'wt',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(events[0]),lineterminator='\n');writer.writeheader();writer.writerows(events)
        count+=len(events);artifacts.append(dict(condition=cell['condition'],arm=cell['arm'],path=str(path.relative_to(ROOT)),sha256=sha(path),rows=len(events)))
        print('REENTRY CENSUS',cell['condition'],cell['arm'],len(events),'events',flush=True)
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    destination.write_text(json.dumps(dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),summaries=summaries,artifacts=artifacts,
        examples=examples,event_rows=count,freeze_sha256=sha(OUT/'FREEZE.json')),indent=2,allow_nan=False)+'\n')
    print('REENTRY CENSUS COMPLETE',count,'events',flush=True)

if __name__=='__main__':main()
