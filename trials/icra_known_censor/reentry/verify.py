"""Independent per-label history search for every saved arrival and rebound."""
from bisect import bisect_left
from collections import Counter,defaultdict
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def equal(saved,want):
    if want is None:return saved==''
    if isinstance(want,bool):return saved==str(want)
    if isinstance(want,str):return saved==want
    return float(saved)==want

def main():
    destination=OUT/'VERIFICATION.json';assert not destination.exists()
    cfg=json.loads((OUT/'FREEZE.json').read_text());result=json.loads((OUT/'RESULTS.json').read_text());assert result['passed']
    assert result['freeze_sha256']==sha(OUT/'FREEZE.json')
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    checked=0;count_summaries=0;native_sources={};example_keys=set();direct_events={}
    for cell in cfg['cells']:
        with gzip.open(ROOT/cell['path'],'rt') as stream:data=json.load(stream)
        run=data['runs'];tables={}
        for name in ['localIncrementRecords','localGaussianRecords','fusionOutputRecords','fusionSourceRecords','iterationRecords','knownCensorRecords']:
            rows=run[name];table={tuple(map(int,row[:4])):row for row in rows};assert len(table)==len(rows);tables[name]=table
        inc=tables['localIncrementRecords'];local=tables['localGaussianRecords'];fused=tables['fusionOutputRecords'];sources=tables['fusionSourceRecords']
        rec=tables['iterationRecords'];interventions=tables['knownCensorRecords'];history=defaultdict(list);final={}
        for (t,n,bt,bl),row in inc.items():history[n,bt,bl].append(t)
        for k in history:history[k].sort();assert len(history[k])==len(set(history[k]))
        for (t,n,bt,bl),row in local.items():
            if not data['delivered'][n-1][2-n][t-1]:final[t,n,bt,bl]=inc[t,n,bt,bl][5]
        for k,row in fused.items():
            t,n,_,_=k;assert data['delivered'][n-1][2-n][t-1];final[k]=row[4]
        expected={k for k,row in fused.items() if row[4]>.001 and k not in local}
        artifact=next(a for a in result['artifacts'] if a['condition']==cell['condition'] and a['arm']==cell['arm'])
        path=ROOT/artifact['path'];assert sha(path)==artifact['sha256']
        with gzip.open(path,'rt',newline='') as stream:raw=list(csv.DictReader(stream))
        saved={tuple(int(row[x]) for x in ['frame','robot','birth_frame','birth_location']):row for row in raw}
        assert len(raw)==len(saved)==artifact['rows'] and set(saved)==expected
        verified=[]
        for k in sorted(expected):
            t,n,bt,bl=k;row=fused[k];source=sources[k];s=saved[k]
            assert source[4:6]==[0,0] and source[6]>0
            remote=(t,3-n,int(source[6]),int(source[7]));assert remote in local and inc[remote][5]>.001
            times=history[n,bt,bl];index=bisect_left(times,t)-1;previous=times[index] if index>=0 else None
            old=inc[previous,n,bt,bl] if previous is not None else None
            oldfinal=final.get((previous,n,bt,bl));oldref=interventions.get((previous,n,bt,bl));oldpeer=final.get((previous,3-n,bt,bl))
            present=k in inc
            if present:assert inc[k][5]<=.001
            kind='same_frame_pruned' if present else 'first_local_arrival' if previous is None else 'return_after_gap'
            if kind=='return_after_gap':assert oldfinal is None or oldfinal<=.001
            caused=oldref is not None and oldref[4]>.001 and oldref[5]<=.001
            if oldref is not None:assert oldfinal==oldref[5]
            peer_kept=oldpeer is not None and oldpeer>.001
            rebound=kind=='return_after_gap' and previous==t-1 and caused and peer_kept and rec[k][13]>0 and rec[k][17]==.001 and remote[2:]==(bt,bl)
            ids=data['truthIds'][t-1]
            if ids and isinstance(ids[0],list):ids=[v for group in ids for v in group]
            target=ids.index(5);near=math.hypot(row[5]-data['truth'][t-1][0][target],row[6]-data['truth'][t-1][1][target])<=2
            want=dict(condition=cell['condition'],arm=cell['arm'],frame=t,robot=n,birth_frame=bt,birth_location=bl,
                remote_birth_frame=remote[2],remote_birth_location=remote[3],remote_input_r=inc[remote][5],kind=kind,returned_r=row[4],near_target=near,
                censor_participates=rec[k][13]>0,current_censor=rec[k][17] if rec[k][13]>0 else None,
                last_prediction_frame=previous,gap=None if present or previous is None else t-previous,
                last_local_r=None if old is None else old[5],last_local_opportunity=None if old is None else bool(old[7]),last_final_r=oldfinal,
                last_refinement_old_r=None if oldref is None else oldref[4],last_refinement_actual_r=None if oldref is None else oldref[5],
                last_refinement_caused_prune=caused,peer_retained_at_last_prediction=peer_kept,peer_r_at_last_prediction=oldpeer,direct_one_frame_rebound=rebound)
            assert set(want)==set(s)
            for name,value in want.items():assert equal(s[name],value),(cell['condition'],cell['arm'],k,name,s[name],value)
            verified.append(want)
            if rebound:direct_events[cell['condition'],cell['arm'],*k]=want
        for summary in [s for s in result['summaries'] if s['condition']==cell['condition'] and s['arm']==cell['arm']]:
            left,right=(1,240) if summary['window']=='full' else (53,122)
            chosen=[e for e in verified if left<=e['frame']<=right and (summary['scope']=='all' or e['near_target'])];classes=Counter(e['kind'] for e in chosen)
            assert summary['remote_only_retained']==len(chosen)==sum(classes.values())
            for name in ['same_frame_pruned','first_local_arrival','return_after_gap']:assert summary[name]==classes[name]
            assert summary['after_refinement_prune']==sum(e['kind']=='return_after_gap' and e['last_refinement_caused_prune'] for e in chosen)
            assert summary['direct_one_frame_rebound']==sum(e['direct_one_frame_rebound'] for e in chosen)
            counts=Counter(e['gap'] for e in chosen if e['kind']=='return_after_gap');assert summary['gap_counts']=={str(k):v for k,v in counts.items()}
            count_summaries+=1
        checked+=len(verified);native_sources[cell['path']]=cell['sha256']
        print('REENTRY HISTORY VERIFIED',cell['condition'],cell['arm'],len(verified),'events',flush=True)
    for e in result['examples']:
        k=(e['condition'],e['arm'],e['frame'],e['robot'],e['birth_frame'],e['birth_location'])
        assert k in direct_events and direct_events[k]==e and e['near_target'] and 53<=e['frame']<=122
        example_keys.add(k)
    expected_examples=set()
    for cell in cfg['cells']:
        candidates=[k for k,e in direct_events.items() if k[:2]==(cell['condition'],cell['arm']) and e['near_target'] and 53<=e['frame']<=122]
        if candidates:expected_examples.add(min(candidates))
    assert example_keys==expected_examples,'first qualifying event in the fixed native order'
    assert len(example_keys)==len(result['examples']) and checked==result['event_rows'] and count_summaries==48
    hashes={**cfg['source_sha256'],str(OUT.relative_to(ROOT))+'/FREEZE.json':sha(OUT/'FREEZE.json'),
        str(OUT.relative_to(ROOT))+'/RESULTS.json':sha(OUT/'RESULTS.json')}
    hashes.update({a['path']:a['sha256'] for a in result['artifacts']})
    destination.write_text(json.dumps(dict(passed=True,verified_events=checked,verified_groups=count_summaries,
        verified_examples=len(example_keys),native_changes=False,input_sha256=hashes),indent=2)+'\n')
    print('REENTRY INDEPENDENT VERIFICATION PASSED',checked,'events;',count_summaries,'groups',flush=True)

if __name__=='__main__':main()
