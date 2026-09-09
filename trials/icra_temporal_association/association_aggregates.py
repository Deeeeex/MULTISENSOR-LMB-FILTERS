"""Common sequence, frame, recording and paired summaries for audited rows."""
from collections import defaultdict
import numpy as np
from identity_metrics import pooled

METRICS=['ospa','gospa','loc2','miss2','false2','countError']
CONDITIONS=['reliable','intermittent']


def summarize(rows, identities, focus, comparisons, scopes):
    aggregates=[]
    for scope,predicate in scopes:
        chosen=[r for r in rows if predicate(r)]
        for condition in CONDITIONS:
            for arm in sorted({r['arm'] for r in chosen}):
                part=[r for r in chosen if r['arm']==arm and r['condition']==condition]
                recordings=defaultdict(list)
                for r in part:
                    recordings[r['recording']].append(r['ospa'])
                value=dict(scope=scope,condition=condition,arm=arm,sequences=len(part),
                    frames=sum(r['frames'] for r in part),recording_groups=len(recordings),
                    sequence_macro={k:float(np.mean([r[k] for r in part])) for k in METRICS},
                    frame_weighted_ospa=float(sum(r['frames']*r['ospa'] for r in part)/sum(r['frames'] for r in part)),
                    recording_macro_ospa=float(np.mean([np.mean(v) for v in recordings.values()])))
                if arm in focus:
                    ids={(r['dataset'],r['sequence']) for r in part}
                    value['identity']=pooled([r for r in identities if (r['dataset'],r['sequence']) in ids and r['arm']==arm and r['condition']==condition])
                    value['communication']={k:sum(r[k] for r in part) for k in ['wire_bytes','raw_payload_bytes','delivered_raw_bytes','split_branches']}
                aggregates.append(value)
    paired=[]
    for scope,predicate in scopes:
        chosen=[r for r in rows if predicate(r)]
        scenes=sorted({r['scene'] for r in chosen})
        recordings=sorted({r['recording'] for r in chosen})
        lookup={(r['scene'],r['condition'],r['arm']):r for r in chosen}
        available={r['arm'] for r in chosen}
        indices=np.random.default_rng(8301).integers(0,len(recordings),size=(10000,len(recordings)))
        for candidate,reference in comparisons:
            if not {candidate,reference}<=available:
                continue
            for condition in CONDITIONS:
                delta={s:lookup[s,condition,candidate]['ospa']-lookup[s,condition,reference]['ospa'] for s in scenes}
                by_rec=np.asarray([np.mean([delta[s] for s in scenes if lookup[s,condition,candidate]['recording']==rec]) for rec in recordings])
                low,high=np.quantile(by_rec[indices].mean(1),[.025,.975])
                paired.append(dict(scope=scope,condition=condition,candidate=candidate,reference=reference,
                    sequence_macro_difference=float(np.mean(list(delta.values()))),
                    recording_macro_difference=float(by_rec.mean()),low=float(low),high=float(high),
                    recording_groups=len(recordings),improved=sum(v < -1e-10 for v in delta.values()),
                    worsened=sum(v > 1e-10 for v in delta.values()),unchanged=sum(abs(v)<=1e-10 for v in delta.values()),
                    sequence_differences=delta))
    return aggregates,paired


def exposed_scopes():
    return [('v2v_all',lambda r:r['dataset']=='v2v'),
            ('v2v_development',lambda r:r['dataset']=='v2v' and r['cohort']=='development'),
            ('v2v_remaining',lambda r:r['dataset']=='v2v' and r['cohort']!='development'),
            ('v2x_val',lambda r:r['dataset']=='v2x_val')]
