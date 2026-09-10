"""Count all recurrent labels with two independent history traversals."""
from bisect import bisect_left
from collections import Counter,defaultdict
import csv
import gzip

def key(row):return tuple(map(int,row[:4]))

def census(data,event_path,destination):
    run=data['runs'];inc={key(r):r for r in run['localIncrementRecords']};local={key(r) for r in run['localGaussianRecords']}
    fused={key(r):r for r in run['fusionOutputRecords']};sources={key(r):r for r in run['fusionSourceRecords']}
    histories=defaultdict(list)
    for t,n,bt,bl in inc:histories[n,bt,bl].append(t)
    for h in histories.values():h.sort()
    with gzip.open(event_path,'rt',newline='') as stream:events=list(csv.DictReader(stream))
    annotated=[];seen_keys=set()
    for e in events:
        t,n,bt,bl=[int(e[x]) for x in ['frame','robot','birth_frame','birth_location']];k=(t,n,bt,bl)
        assert k not in seen_keys;seen_keys.add(k);h=histories[n,bt,bl];index=bisect_left(h,t)
        previous=h[index-1] if index else None
        if k in inc:kind='same_frame_pruned';assert inc[k][5]<=.001;gap=None
        elif previous is None:kind='first_local_arrival';gap=None
        else:kind='return_after_gap';gap=t-previous
        annotated.append(dict(**e,kind=kind,last_prediction_frame=previous,gap=gap))
    # Separate forward state set, driven by raw fusion and prediction rows.
    expected={};seen=set();inc_at=defaultdict(set);fusion_at=defaultdict(list)
    for t,n,bt,bl in inc:inc_at[t].add((n,bt,bl))
    for k,r in fused.items():fusion_at[k[0]].append((k,r))
    for t in range(1,241):
        for k,r in fusion_at[t]:
            if r[4]<=.001 or k in local:continue
            _,n,bt,bl=k;label=(n,bt,bl)
            assert sources[k][4:6]==[0,0] and sources[k][6]>0
            expected[k]='same_frame_pruned' if label in inc_at[t] else 'return_after_gap' if label in seen else 'first_local_arrival'
        seen.update(inc_at[t])
    assert expected.keys()==seen_keys
    for e in annotated:
        k=tuple(int(e[x]) for x in ['frame','robot','birth_frame','birth_location']);assert expected[k]==e['kind']
    summaries=[]
    for window,left,right in [('full',1,240),('original_window',53,122)]:
        for scope in ['all','target_neighbourhood']:
            selected=[e for e in annotated if left<=int(e['frame'])<=right and (scope=='all' or e['near_target']=='True')]
            counts=Counter(e['kind'] for e in selected);gaps=Counter(e['gap'] for e in selected if e['kind']=='return_after_gap')
            summaries.append(dict(window=window,scope=scope,remote_only_retained=len(selected),
                **{k:counts[k] for k in ['same_frame_pruned','first_local_arrival','return_after_gap']},
                recurrences=counts['same_frame_pruned']+counts['return_after_gap'],gap_counts={str(k):v for k,v in sorted(gaps.items())}))
    assert not destination.exists()
    with gzip.open(destination,'wt',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(annotated[0]),lineterminator='\n');writer.writeheader();writer.writerows(annotated)
    return dict(verified_events=len(annotated),summaries=summaries)
