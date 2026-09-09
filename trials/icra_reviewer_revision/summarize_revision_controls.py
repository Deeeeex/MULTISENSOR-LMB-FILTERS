"""Sequence-paired recursive 2x2 contrasts and frozen fixed-strength control."""
from pathlib import Path
import json
import hashlib
import csv

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
GS = 'marked_gaussian_evidence_guarded_scalar'
GCE = 'marked_gaussian_evidence'
SCALAR = 'marked_asymmetric'
NO_CURVATURE = 'marked_gaussian_evidence_no_curvature'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def checked_report(name):
    path = OUT / name; value = json.loads(path.read_text()); assert value['passed']
    for key, expected in value.get('inputs', {}).items():
        assert sha(ROOT / key) == expected, key
    return value


def interval(values, samples):
    values = np.asarray(values, float)
    low, high = np.percentile(values[samples].mean(axis=1), [2.5, 97.5])
    return dict(mean=float(values.mean()), low=float(low), high=float(high),
                negative_sequences=int((values < -1e-10).sum()), positive_sequences=int((values > 1e-10).sum()))


def main():
    destination = OUT / 'CONTROLS_ANALYSIS.json'; assert not destination.exists()
    legacy = checked_report('LEGACY_RESCORING.json')
    development = checked_report('audit_controls_development.json')
    transfer = checked_report('audit_gs_seen_transfer.json')
    fixed_transfer = checked_report('audit_fixed025_seen_transfer.json')
    selection = json.loads((OUT / 'FIXED_SELECTION.json').read_text()); fixed = selection['selected']['arm']
    cohorts = {}; all_rows = []
    for cohort, additional in [('development', development['rows']), ('seen_transfer', transfer['rows'] + fixed_transfer['rows'])]:
        rows = [r for r in legacy['rows'] if r['cohort'] == cohort] + additional
        names = sorted({r['sequence'] for r in rows}); assert len(names) == (9 if cohort == 'development' else 25)
        lookup = {(r['sequence'], r['condition'], r['arm']):r for r in rows}; assert len(lookup) == len(rows)
        samples = np.random.default_rng(8301).integers(0, len(names), size=(10000, len(names)))
        aggregates = []; contrasts = []; interactions = []
        arms = sorted({r['arm'] for r in rows})
        for condition in ['reliable', 'intermittent']:
            for arm in arms:
                group = [lookup[name, condition, arm] for name in names]
                aggregates.append(dict(condition=condition, arm=arm,
                    **{metric:interval([row[metric] for row in group], samples) for metric in
                       ['ospa','gospa','loc2','miss2','false2','countError','raw_bytes','delivered_raw_bytes','wire_bytes']}))
            for candidate, reference in [(GCE,GS),(GS,SCALAR),(GCE,NO_CURVATURE),(NO_CURVATURE,SCALAR),
                                          (GCE,SCALAR),(GCE,fixed),(fixed,'marked_lineage')]:
                delta = [lookup[name,condition,candidate]['ospa'] - lookup[name,condition,reference]['ospa'] for name in names]
                contrasts.append(dict(condition=condition,candidate=candidate,reference=reference,ospa=interval(delta,samples)))
            interaction = [lookup[name,condition,GCE]['ospa'] - lookup[name,condition,GS]['ospa']
                           - lookup[name,condition,NO_CURVATURE]['ospa'] + lookup[name,condition,SCALAR]['ospa'] for name in names]
            interactions.append(dict(condition=condition,contrast='(GCE-GS)-(no-curvature-Scalar)',ospa=interval(interaction,samples)))
        cohorts[cohort] = dict(sequences=len(names), frames=sum(lookup[name,'reliable',GCE]['frames'] for name in names),
                              rows=rows, aggregate=aggregates, paired=contrasts, interaction=interactions)
        all_rows.extend(dict(row,cohort=cohort) for row in rows)
    files = ['LEGACY_RESCORING.json','audit_controls_development.json','audit_gs_seen_transfer.json',
             'audit_fixed025_seen_transfer.json','FIXED_SELECTION.json']
    result = dict(passed=True, cohorts=cohorts, fixed_selection=selection,
                  sources={name:sha(OUT / name) for name in files}, analyzer_sha256=sha(Path(__file__)),
                  factors={'scalar_without_guard':SCALAR,'scalar_with_guard':GS,'joint_without_guard':NO_CURVATURE,'joint_with_guard':GCE},
                  inference='Complete independent recursion per arm. Bootstrap units are entire sequences, 10000 paired draws, seed 8301. Both cohorts were previously exposed; intervals are descriptive, not unseen-data generalization.',
                  communication='GS and Fixed need full 352-byte Gaussian packets. Native Scalar is 232 bytes. Legacy No-age development retained an unused 8-byte field; native seen-transfer No-age is 208 bytes.')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT / 'controls_sequence_scores.csv').open('w') as stream:
        fields = ['cohort','sequence','condition','arm','frames','ospa','gospa','loc2','miss2','false2','countError','raw_bytes','delivered_raw_bytes','wire_bytes','runtime_s']
        writer = csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(all_rows)
    for cohort, group in cohorts.items():
        for row in group['paired'][:]:
            if row['reference'] in [GS,fixed]:
                print('RECURSIVE CONTROL',cohort,row['condition'],row['candidate'],'minus',row['reference'],row['ospa'],flush=True)
        print('INTERACTION',cohort,group['interaction'],flush=True)


if __name__ == '__main__':
    main()
