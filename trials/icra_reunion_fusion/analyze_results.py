"""Exact independent assignment audit and predeclared descriptive readout.

Only NumPy is needed. At most six estimates/four truths makes exhaustive
rectangular matching inexpensive and independent of MATLAB's Hungarian code.
This recomputation validates metrics, not the research hypothesis or method.
"""
from functools import lru_cache
from itertools import permutations
from pathlib import Path
import argparse
import csv
import gzip
import hashlib
import json

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SCENES = ['split_latebirth', 'churn_departure', 'split_no_new']
ARMS = ['local', 'kla', 'fov', 'lineage', 'mil', 'recent']
SEEDS = [2801, 2802, 2803]
C2 = 144.


@lru_cache(None)
def assignments(small, large):
    assert 0 < small <= large <= 6
    return np.asarray(list(permutations(range(large), small)), dtype=int)


def score(truth, estimates):
    truth = np.asarray(truth, dtype=float).reshape(4, -1).T
    estimates = np.asarray(estimates, dtype=float).reshape(-1, 4)
    n, m = len(truth), len(estimates)
    assert n <= 4 and m <= 6
    count = abs(n-m)
    match_d2 = np.full(n, np.nan)
    if not min(n, m):
        return dict(ospa=0. if n == m else 12., countError=count,
                    matchedSquaredError=0., matchedCount=0,
                    gospa=np.sqrt((n+m)*C2/2), loc2=0., miss2=n*C2/2,
                    false2=m*C2/2, match_d2=match_d2)
    cost = ((truth[:, None, :2] - estimates[None, :, :2])**2).sum(-1)
    reduced = np.minimum(cost if n <= m else cost.T, C2)
    p = assignments(min(n, m), max(n, m))
    totals = reduced[np.arange(min(n, m))[None, :], p].sum(1)
    chosen = p[int(np.argmin(totals))]
    rows, cols = ((np.arange(n), chosen) if n <= m else (chosen, np.arange(m)))
    distances = cost[rows, cols]
    valid = distances < C2
    loc2 = float(distances[valid].sum())
    matched = int(valid.sum())
    miss2, false2 = (n-matched)*C2/2, (m-matched)*C2/2
    match_d2[rows[valid]] = distances[valid]
    return dict(ospa=float(np.sqrt((float(totals.min()) + C2*count)/max(n, m))),
                countError=count, matchedSquaredError=loc2, matchedCount=matched,
                gospa=float(np.sqrt(loc2+miss2+false2)), loc2=loc2,
                miss2=miss2, false2=false2, match_d2=match_d2)


def mean_or_none(values):
    return float(np.mean(values)) if len(values) else None


def inspect_run(data, run):
    N, T = 8, len(data['time'])
    metrics = {k: np.zeros((N, T)) for k in ['gospa', 'loc2', 'miss2', 'false2']}
    matched = np.full((N, 6, T), np.nan)
    audited = 0
    for t in range(T):
        regions = np.asarray(data['truthRegions'][t], dtype=int)-1
        for node in range(N):
            result = score(data['truth'][t], run['estimates'][node+N*t])
            for name in ['ospa', 'countError', 'matchedSquaredError', 'matchedCount']:
                assert np.isclose(run[name][node][t], result[name], atol=1e-9, rtol=1e-10), (
                    data['scene'], data['seed'], run['arm'], node, t, name)
            pre = score(data['truth'][t], run['preEstimates'][node+N*t])
            assert np.isclose(run['preOspa'][node][t], pre['ospa'], atol=1e-9)
            assert run['preCountError'][node][t] == pre['countError']
            for name in metrics:
                metrics[name][node, t] = result[name]
            matched[node, regions, t] = result['match_d2']
            audited += 1
    ospa = np.asarray(run['ospa'])
    components = np.asarray(data['componentCount'])
    reunions = []
    for frame in np.atleast_1d(data['reconnectionFrames']).astype(int):
        first = frame-1
        if first >= T:
            continue
        later = np.flatnonzero(components[first:] > 1)
        end = min(first+10, first+int(later[0]) if len(later) else T, T)
        reunions.append(dict(frame=int(frame), frames=int(end-first),
                             mean_ospa=float(ospa[:, first:end].mean())))
    acquisitions = []
    for region in [3, 4]:
        active = [t for t, ids in enumerate(data['truthRegions']) if region in ids]
        if not active:
            continue
        # Opposite squad is fixed by robot UID, never by changing group names.
        remote = range(4, 8) if region == 3 else range(4)
        for node in remote:
            a, b = active[0], active[-1]+1
            good = matched[node, region-1, a:b] < 4.
            starts = [j for j in range(max(0, len(good)-2)) if good[j:j+3].all()]
            acquisitions.append(dict(region=region, node=node+1,
                                     delay_s=.5*starts[0] if starts else None,
                                     censored=not bool(starts), active_frames=b-a))
    sse = float(np.asarray(run['matchedSquaredError']).sum())
    support = int(np.asarray(run['matchedCount']).sum())
    row = dict(scene=data['scene'], seed=data['seed'], arm=run['arm'],
               ospa=float(ospa.mean()), count_mae=float(np.asarray(run['countError']).mean()),
               worst_node=float(ospa.max(0).mean()), p90=float(np.quantile(ospa, .9)),
               gospa=float(metrics['gospa'].mean()), loc2=float(metrics['loc2'].mean()),
               miss2=float(metrics['miss2'].mean()), false2=float(metrics['false2'].mean()),
               matched_rmse=float(np.sqrt(sse/support)) if support else None,
               matched_support=support, matched_squared_error=sse,
               reunion_ospa=mean_or_none([r['mean_ospa'] for r in reunions]),
               post_departure_false2=float(metrics['false2'][:, 90:].mean()) if T>90 else None,
               pre_ospa=float(np.asarray(run['preOspa']).mean()),
               wire_bytes=run['totalWireBytes'], raw_bytes=sum(run['rawPayloadBytes']),
               delivered_raw_bytes=sum(run['deliveredRawBytes']),
               control_bytes=sum(run['controlBytes']),
               lineage_exclusions=int(np.asarray(run['lineageExclusions']).sum()),
               observable_absences=int(np.asarray(run['observableAbsences']).sum()),
               weight_l1=float(np.asarray(run['weightChangeL1']).sum()),
               runtime_s=run['runtimeSeconds'],
               remote_acquisitions=sum(not a['censored'] for a in acquisitions),
               remote_queries=len(acquisitions),
               acquired_remote_delay_s=mean_or_none([a['delay_s'] for a in acquisitions if not a['censored']]),
               acquisitions=acquisitions, reunions=reunions)
    return row, matched, metrics, audited


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['full', 'smoke'], default='full')
    args = parser.parse_args()
    rows, comparisons, visibility, curves = [], [], [], {}
    audited = 0
    for scene in SCENES:
        scene_curves = []
        for seed in (SEEDS if args.mode == 'full' else SEEDS[:1]):
            path = OUT/'results'/f'{scene}_seed{seed}_{args.mode}.json.gz'
            with gzip.open(path, 'rt') as stream:
                data = json.load(stream)
            assert data['seed']==seed and data['scene']==scene and data['mode']==args.mode
            assert [r['arm'] for r in data['runs']]==ARMS
            expected_T = 120 if args.mode == 'full' else 4
            assert len(data['time'])==expected_T
            assert len({r['totalWireBytes'] for r in data['runs'][1:]}) == 1
            expected_wire = sum(16384*2*(8-c)+128*8 for c in data['componentCount'])
            assert all(r['totalWireBytes']==expected_wire for r in data['runs'][1:])
            traces = {}
            for run in data['runs']:
                row, matched, metrics, count = inspect_run(data, run)
                rows.append(row); traces[run['arm']] = matched; audited += count
            scene_curves.append([np.asarray(r['ospa']).mean(0).tolist() for r in data['runs']])
            for candidate in ['lineage', 'mil', 'recent']:
                for reference in ['kla', 'fov']:
                    left, right = traces[candidate], traces[reference]
                    common = np.isfinite(left) & np.isfinite(right)
                    extra = np.isfinite(left) & ~np.isfinite(right)
                    comparisons.append(dict(scene=scene,seed=seed,candidate=candidate,reference=reference,
                        common_support=int(common.sum()),
                        candidate_common_rmse=float(np.sqrt(left[common].mean())) if common.any() else None,
                        reference_common_rmse=float(np.sqrt(right[common].mean())) if common.any() else None,
                        added_support=int(extra.sum()),
                        added_rmse=float(np.sqrt(left[extra].mean())) if extra.any() else None))
            vis = np.asarray(data['visibility'], dtype=bool)
            for region in [1,2,3,4]:
                active = [t for t, ids in enumerate(data['truthRegions']) if region in ids]
                if not active:
                    continue
                remote = range(4,8) if region in [1,3] else range(4)
                visibility.append(dict(scene=scene,seed=seed,region=region,active_frames=len(active),
                    unobserved_network_frames=int((~vis[:,region-1,active].any(0)).sum()),
                    remote_observed_node_frames=int(vis[list(remote),region-1,:][:,active].sum())))
        curves[scene]=scene_curves
    numeric = ['ospa','count_mae','worst_node','p90','gospa','loc2','miss2','false2',
               'matched_rmse','wire_bytes','raw_bytes','reunion_ospa','post_departure_false2']
    aggregate=[]
    for scene in SCENES:
        for arm in ARMS:
            selected=[r for r in rows if r['scene']==scene and r['arm']==arm]
            a=dict(scene=scene,arm=arm,seeds=len(selected))
            for key in numeric:
                values=[r[key] for r in selected if r[key] is not None]
                a[key]=mean_or_none(values)
            a['ospa_sample_sd']=float(np.std([r['ospa'] for r in selected],ddof=1)) if len(selected)>1 else None
            a['remote_acquisitions']=sum(r['remote_acquisitions'] for r in selected)
            a['remote_queries']=sum(r['remote_queries'] for r in selected)
            aggregate.append(a)
    lookup={(a['scene'],a['arm']):a for a in aggregate}
    gates=[]
    for candidate in ['lineage','mil','recent']:
        checks=[]
        for scene in SCENES[:2]:
            a=lookup[scene,candidate]
            for ref in ['kla','fov']:
                b=lookup[scene,ref]
                checks.extend([
                    dict(scene=scene,reference=ref,metric='ospa_gain',value=1-a['ospa']/b['ospa'],passed=a['ospa']<=.95*b['ospa']),
                    dict(scene=scene,reference=ref,metric='count_mae',value=a['count_mae']-b['count_mae'],passed=a['count_mae']<=b['count_mae']+1e-12),
                    dict(scene=scene,reference=ref,metric='worst_node',value=a['worst_node']/b['worst_node'],passed=a['worst_node']<=1.05*b['worst_node'])])
        for ref in ['kla','fov']:
            a,b=lookup['split_no_new',candidate],lookup['split_no_new',ref]
            checks.append(dict(scene='split_no_new',reference=ref,metric='false_cost',
                               value=a['false2']-b['false2'],passed=a['false2']<=1.05*b['false2']+1e-12))
        gates.append(dict(arm=candidate,screen_passed=all(c['passed'] for c in checks),checks=checks))
    hashes=json.loads((OUT/'source_sha256.json').read_text())
    hash_audit=[]
    for path, reference in hashes.items():
        value=(ROOT/path).read_bytes()
        match=hashlib.sha256(value).hexdigest()==reference['raw_sha256']
        assert match,path
        hash_audit.append(dict(path=path,matched=match))
    output=dict(mode=args.mode,assignment_audit='exhaustive rectangular matching, NumPy',
                audited_node_frames=audited,aggregate=aggregate,runs=rows,gates=gates,
                common_target_comparisons=comparisons,visibility=visibility,curves=curves,
                source_hash_audit=hash_audit)
    dest=OUT/f'summary_{args.mode}.json'
    dest.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    with (OUT/f'metrics_{args.mode}.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(aggregate[0]),lineterminator='\n')
        writer.writeheader(); writer.writerows(aggregate)
    print(f'Audited {audited} node-frames, pre/post OSPA, count, localization and support.')
    for a in aggregate:
        print(f"{a['scene']:18} {a['arm']:8} OSPA={a['ospa']:.4f} count={a['count_mae']:.4f} "
              f"miss2={a['miss2']:.2f} false2={a['false2']:.2f}")
    if args.mode=='full':
        for gate in gates:
            print(gate['arm'],'screen_passed',gate['screen_passed'])


if __name__=='__main__':
    main()
