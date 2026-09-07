"""Audit frozen v3 against the same v1 controls and v2 screening gate."""
from pathlib import Path
import gzip
import hashlib
import json
import numpy as np
from analyze_results import OUT, ROOT, SCENES, SEEDS, inspect_run

EXTENSION_ARMS = ['qualified_exist', 'confirmed_lineage', 'confirmed_exist']


def phase_readout(data, run):
    ospa, count = np.asarray(run['ospa']), np.asarray(run['countError'])
    before, before_count = np.asarray(run['preOspa']), np.asarray(run['preCountError'])
    phases = {'initial_1_34':list(range(34)), 'late_birth_35_76':list(range(34,76)),
              'after_77_120':list(range(76,120)), 'departure_91_120':list(range(90,120))}
    first = int(np.atleast_1d(data['reconnectionFrames'])[0])-1
    phases['first_reunion_10'] = list(range(first,min(first+10,120)))
    return [dict(phase=name, ospa=float(ospa[:,frames].mean()),
                 count_mae=float(count[:,frames].mean()),
                 fusion_delta_ospa=float((ospa-before)[:,frames].mean()),
                 fusion_delta_count=float((count-before_count)[:,frames].mean()))
            for name,frames in phases.items()]


def main():
    baseline=json.loads((OUT/'summary_full.json').read_text())
    rows=[]; common=[]; phases=[]; curves={}; audited=0
    for scene in SCENES:
        curves[scene]=[]
        for seed in SEEDS:
            with gzip.open(OUT/'results'/f'{scene}_seed{seed}_full.json.gz','rt') as stream:
                base=json.load(stream)
            with gzip.open(OUT/'results'/f'{scene}_seed{seed}_confirmation.json.gz','rt') as stream:
                extra=json.load(stream)
            assert extra['truth']==base['truth'] and extra['visibility']==base['visibility']
            assert [r['arm'] for r in extra['runs']]==EXTENSION_ARMS
            base_matches={}
            for r in base['runs']:
                if r['arm'] in ['fov','lineage','recent']:
                    _,base_matches[r['arm']],_,_=inspect_run(base,r)
                phases.extend(dict(scene=scene,seed=seed,arm=r['arm'],**p) for p in phase_readout(base,r))
            curves[scene].append([np.asarray(r['ospa']).mean(0).tolist() for r in extra['runs']])
            for r in extra['runs']:
                assert r['totalWireBytes']==base['runs'][1]['totalWireBytes']
                assert r['deliveredMessages']==base['runs'][1]['deliveredMessages']
                row,matched,_,count=inspect_run(extra,r); rows.append(row); audited+=count
                phases.extend(dict(scene=scene,seed=seed,arm=r['arm'],**p) for p in phase_readout(extra,r))
                for ref,other in base_matches.items():
                    joint=np.isfinite(matched) & np.isfinite(other)
                    common.append(dict(scene=scene,seed=seed,arm=r['arm'],reference=ref,
                        common_support=int(joint.sum()),
                        candidate_sse=float(matched[joint].sum()),reference_sse=float(other[joint].sum())))
    aggregate=[]
    numeric=['ospa','count_mae','worst_node','p90','gospa','loc2','miss2','false2',
             'matched_rmse','wire_bytes','raw_bytes','reunion_ospa','post_departure_false2']
    for scene in SCENES:
        for arm in EXTENSION_ARMS:
            selected=[r for r in rows if r['scene']==scene and r['arm']==arm]
            a=dict(scene=scene,arm=arm)
            for key in numeric: a[key]=float(np.mean([r[key] for r in selected]))
            a['ospa_sample_sd']=float(np.std([r['ospa'] for r in selected],ddof=1))
            a['remote_acquisitions']=sum(r['remote_acquisitions'] for r in selected)
            a['remote_queries']=sum(r['remote_queries'] for r in selected)
            aggregate.append(a)
    lookup={(r['scene'],r['arm']):r for r in baseline['aggregate']+aggregate}
    checks=[]
    for arm in EXTENSION_ARMS:
        arm_checks=[]
        for scene in SCENES[:2]:
            a,b=lookup[scene,arm],lookup[scene,'lineage']
            arm_checks.append(dict(scene=scene,criterion='ospa_ratio_vs_lineage',
                                   value=a['ospa']/b['ospa'],passed=a['ospa']<=1.02*b['ospa']))
            group=[r for r in common if r['scene']==scene and r['arm']==arm and r['reference']=='lineage']
            ratio=np.sqrt(sum(r['candidate_sse'] for r in group)/sum(r['reference_sse'] for r in group))
            arm_checks.append(dict(scene=scene,criterion='common_rmse_ratio_vs_lineage',
                                   value=float(ratio),passed=bool(ratio<=1.02)))
        a,b=lookup['churn_departure',arm],lookup['churn_departure','lineage']
        arm_checks.append(dict(scene='churn_departure',criterion='departure_false_ratio_vs_lineage',
                               value=a['post_departure_false2']/b['post_departure_false2'],
                               passed=a['post_departure_false2']<=.9*b['post_departure_false2']))
        a,b=lookup['split_no_new',arm],lookup['split_no_new','fov']
        arm_checks.append(dict(scene='split_no_new',criterion='false_cost_vs_fov',
                               value=a['false2']-b['false2'],passed=a['false2']<=b['false2']+1e-12))
        checks.append(dict(arm=arm,screen_passed=all(c['passed'] for c in arm_checks),checks=arm_checks))
    hashes=json.loads((OUT/'confirmation_source_sha256.json').read_text())
    for name,want in hashes.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==want,name
    result=dict(audited_extension_node_frames=audited,aggregate=aggregate,runs=rows,
                common_target_comparisons=common,phases=phases,curves=curves,gates=checks,
                source_and_input_hashes_verified=len(hashes))
    (OUT/'summary_confirmation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Audited extension node-frames:',audited)
    for a in aggregate:
        print(a['scene'],a['arm'],'OSPA',round(a['ospa'],4),'count',round(a['count_mae'],4),
              'false2',round(a['false2'],4),'post-departure false2',round(a['post_departure_false2'],4))
    for gate in checks:
        print(gate['arm'],'screen_passed',gate['screen_passed'])
        for check in gate['checks']: print(' ',check)


if __name__=='__main__':
    main()
