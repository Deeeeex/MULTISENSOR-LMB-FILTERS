"""Independent metric audit and descriptive plots; no parameter selection."""
from pathlib import Path
import hashlib
import gzip
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tmp/whales_deps'))
import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
SCENES = ['split_rejoin', 'boundary_churn']
ARMS = ['local', 'tree', 'dynamic', 'stable']
SEEDS = [2701, 2702, 2703]
COLORS = ['#737373', '#2563eb', '#ea580c', '#16a34a']


def score(truth, states):
    states = np.asarray(states, dtype=float).reshape(-1, 4)
    if len(states) == 0:
        return 30., 3., 0., 0
    squared = ((truth[:2, :].T[:, None] - states[None, :, :2])**2).sum(-1)
    rows, cols = linear_sum_assignment(np.minimum(squared, 900))
    selected = squared[rows, cols]
    good = selected < 900
    return (np.sqrt((np.minimum(selected, 900).sum() + 900*abs(3-len(states)))/max(3, len(states))),
            abs(3-len(states)), selected[good].sum(), int(good.sum()))


def recovery(result, run):
    mean = np.asarray(run['ospa']).mean(0)
    components = np.asarray(result['componentCount'])
    events = np.atleast_1d(result['reconnectionFrames']).astype(int)
    records = []
    for frame in events:
        start = frame-1
        later = np.flatnonzero(components[start:] > 1)
        end = start+later[0] if len(later) else len(mean)
        candidates = [t for t in range(start, end-4) if np.all(mean[t:t+5] <= 5)]
        records.append(dict(frame=int(frame), time_s=float(result['time'][start]),
                            available_frames=int(end-start),
                            first10_ospa=float(mean[start:min(start+10, end)].mean()),
                            recovery_delay_s=None if not candidates else .5*(candidates[0]-start),
                            censored=not bool(candidates)))
    return records


def main():
    data = {}
    rows = []
    audited = 0
    geometry = json.loads((OUT/'geometry.json').read_text())
    for scene in SCENES:
        data[scene] = []
        for seed in SEEDS:
            prefix = OUT/'results'/f'{scene}_seed{seed}_full'
            with gzip.open(str(prefix)+'.json.gz','rt',encoding='utf8') as stream:
                result = json.load(stream)
            artifact = loadmat(prefix.with_suffix('.mat'), simplify_cells=True)
            truth = artifact['truth']
            assert truth.shape == (4, 3, 120)
            assert result['mode'] == 'full' and result['seed'] == seed
            assert len(result['time']) == 120
            data[scene].append(result)
            budgets = [r['totalWireBytes'] for r in result['runs'][1:]]
            assert len(set(budgets)) == 1
            for a, run in enumerate(result['runs']):
                assert run['arm'] == ARMS[a]
                ospa = np.asarray(run['ospa'])
                assert ospa.shape == (8, 120)
                assert abs(ospa.mean()-run['meanOspa']) < 1e-12
                for t in range(120):
                    for node in range(8):
                        # MATLAB JSON flattens a non-vector cell array in
                        # column-major order; each element is a set of states.
                        estimated = run['estimates'][node+8*t]
                        values = score(truth[:, :, t], estimated)
                        for metric, value in zip(['ospa', 'countError', 'matchedSquaredError', 'matchedCount'], values):
                            assert np.isclose(run[metric][node][t], value, atol=1e-9, rtol=1e-10), (scene, seed, a, node, t, metric)
                        audited += 1
                events = recovery(result, run)
                support = np.asarray(run['matchedCount']).sum()
                rows.append(dict(scene=scene, seed=seed, arm=run['arm'],
                                 ospa=float(ospa.mean()), p90=float(np.quantile(ospa,.9)),
                                 mean_worst_node=float(ospa.max(0).mean()),
                                 count_mae=float(np.asarray(run['countError']).mean()),
                                 matched_rmse=float(np.sqrt(np.asarray(run['matchedSquaredError']).sum()/support)) if support else None,
                                 matched_support=int(support), possible_target_node_frames=2880,
                                 pre_ospa=float(np.asarray(run['preOspa']).mean()),
                                 wire_bytes=run['totalWireBytes'],
                                 payload_bytes=sum(run['payloadBytes']), control_bytes=sum(run['controlBytes']),
                                 attempted_messages=sum(run['attemptedMessages']),
                                 delivered_messages=sum(run['deliveredMessages']),
                                 pair_edits=sum(run['pairEdits']), edge_edits=sum(run['edgeEdits']),
                                 runtime_s=run['runtimeSeconds'], topology_s=run['topologySeconds'],
                                 recovery=events))
    aggregate = []
    for scene in SCENES:
        for arm in ARMS:
            selected = [r for r in rows if r['scene']==scene and r['arm']==arm]
            aggregate.append(dict(scene=scene, arm=arm,
                                  ospa_mean=float(np.mean([r['ospa'] for r in selected])),
                                  ospa_sample_sd=float(np.std([r['ospa'] for r in selected],ddof=1)),
                                  count_mae=float(np.mean([r['count_mae'] for r in selected])),
                                  p90_mean=float(np.mean([r['p90'] for r in selected])),
                                  worst_node_mean=float(np.mean([r['mean_worst_node'] for r in selected])),
                                  wire_mib=float(np.mean([r['wire_bytes'] for r in selected])/2**20),
                                  raw_payload_mib=float(np.mean([r['payload_bytes'] for r in selected])/2**20),
                                  control_mib=float(np.mean([r['control_bytes'] for r in selected])/2**20),
                                  pair_edits=selected[0]['pair_edits'], edge_edits=selected[0]['edge_edits'],
                                  post_rejoin_ospa=float(np.mean([e['first10_ospa'] for r in selected for e in r['recovery']]))))
    paired = []
    for scene in SCENES:
        for seed in SEEDS:
            values = {r['arm']:r for r in rows if r['scene']==scene and r['seed']==seed}
            paired.append(dict(scene=scene,seed=seed,
                               stable_minus_tree_ospa=values['stable']['ospa']-values['tree']['ospa'],
                               stable_minus_dynamic_ospa=values['stable']['ospa']-values['dynamic']['ospa'],
                               stable_minus_tree_worst_node=values['stable']['mean_worst_node']-values['tree']['mean_worst_node']))
    summary = dict(metric_audit_node_frames=audited, aggregate=aggregate, paired=paired, runs=rows,
                   recovery_window_note='Up to 10 frames, truncated at next physical disconnection; 5-frame recovery window must fit.',
                   topology_and_control_note='Centralized geometry; charged reliable out-of-band control; no distributed consensus implementation.')
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
    fig, axes = plt.subplots(2,2,figsize=(12,7),sharex='col',layout='constrained')
    for col, scene in enumerate(SCENES):
        time = np.asarray(data[scene][0]['time'])
        curves = np.array([[np.asarray(r['ospa']).mean(0) for r in d['runs']] for d in data[scene]])
        for a, arm in enumerate(ARMS):
            axes[0,col].plot(time,curves[:,a].mean(0),color=COLORS[a],label=arm,lw=1.4)
        delta = curves[:,3]-curves[:,1]
        axes[1,col].plot(time,delta.mean(0),color=COLORS[3],lw=1.4)
        axes[1,col].fill_between(time,delta.min(0),delta.max(0),alpha=.18,color=COLORS[3],label='Range across 3 paired seeds')
        axes[1,col].axhline(0,color='#555555',lw=.8)
        for ax in axes[:,col]:
            for idx in np.flatnonzero(np.asarray(data[scene][0]['componentCount'])>1):
                ax.axvspan(time[idx],time[idx]+.5,color='#aaaaaa',alpha=.12,lw=0)
            ax.grid(alpha=.15)
        axes[0,col].set(title=scene.replace('_',' '),ylabel='Mean node OSPA (m)')
        axes[1,col].set(xlabel='Time (s)',ylabel='Stable minus tree OSPA (m)')
    axes[0,0].legend(ncol=4,fontsize=8)
    axes[1,0].legend(fontsize=8)
    fig.suptitle('Matched padded wire budgets; shading = physical separation\nNegative differences favor stable grouping; bands are seed ranges, not confidence intervals')
    fig.savefig(OUT/'tracking_comparison.png',dpi=180)
    fig.savefig(OUT/'tracking_comparison.pdf')
    fig, axes = plt.subplots(2,2,figsize=(11,7),layout='constrained')
    for col, scene in enumerate(SCENES):
        raw=loadmat(OUT/'results'/f'{scene}_seed2701_full.mat',simplify_cells=True)
        p=raw['scene']['positions']; time=raw['scene']['time']; topo=raw['topo']
        for n in range(8):
            axes[0,col].plot(p[0,n],p[1,n],lw=1,color=COLORS[1 if n<4 else 2],alpha=.8)
            axes[0,col].scatter(p[0,n,0],p[1,n,0],s=24,marker='s',color=COLORS[1 if n<4 else 2])
            axes[0,col].annotate(str(n+1),p[:,n,0],fontsize=8)
        for j in range(3):
            axes[0,col].plot(raw['truth'][0,j],raw['truth'][1,j],color='#555555',ls='--',lw=1)
        axes[0,col].set(aspect='equal',title=scene.replace('_',' '),xlabel='x (m)',ylabel='y (m)')
        axes[1,col].step(time,topo['componentCount'],where='post',color='#7c3aed',label='Physical components')
        axes[1,col].set(yticks=[1,2],ylim=(.8,2.2),xlabel='Time (s)',ylabel='Components')
        for ax in axes[:,col]: ax.grid(alpha=.15)
    fig.suptitle('Prescribed mobile-sensor paths; dashed = targets for seed 2701')
    fig.savefig(OUT/'scenarios.png',dpi=180)
    tracked = [OUT/'PROTOCOL.md',OUT/'runGroupTrackingPilot.m',OUT/'buildGroupPilotScene.m',OUT/'buildGroupPilotTopologies.m']
    tracked += [ROOT/p for p in ['lmb/lmbPredictionStep.m','multisensorLmb/updateLmbWithSensorMeasurement.m',
                                'multisensorLmb/fuseLmbPosteriorsByLabel.m','common/computePositionEuclideanOspa.m']]
    manifest = {str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    (OUT/'source_sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    print(json.dumps(dict(audited=audited,aggregate=aggregate,paired=paired),indent=2))


if __name__=='__main__':
    main()
