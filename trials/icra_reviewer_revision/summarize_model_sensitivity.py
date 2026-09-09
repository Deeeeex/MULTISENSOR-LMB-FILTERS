"""Summarize all registered motion and pD checks only after complete audits."""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

from summarize_new_data import checked_audit, sha, CONDITIONS, METRICS

OUT = Path(__file__).resolve().parent
GCE = 'marked_gaussian_evidence'
ARMS = ['marked_lineage', 'marked_asymmetric', GCE + '_guarded_scalar', GCE]


def summary(values, draws):
    values = np.asarray(values, float)
    result = dict(mean=float(values.mean()), n=len(values), values=values.tolist())
    if draws is not None:
        low, high = np.percentile(values[draws].mean(axis=1), [2.5, 97.5])
        result.update(low=float(low), high=float(high))
    return result


def summarize(rows, expected):
    names = sorted({row['sequence'] for row in rows})
    assert len(names) == expected
    lookup = {(row['sequence'], row['condition'], row['arm']): row for row in rows}
    assert len(rows) == len(lookup) == expected * 2 * 4
    draws = np.random.default_rng(8301).integers(0, expected, (10000, expected)) if expected >= 9 else None
    aggregate, paired = [], []
    for condition in CONDITIONS:
        for arm in ARMS:
            selected = [lookup[name, condition, arm] for name in names]
            aggregate.append(dict(condition=condition, arm=arm,
                **{metric: summary([row[metric] for row in selected], draws) for metric in METRICS}))
            if arm != GCE:
                differences = [lookup[name, condition, GCE]['ospa'] - lookup[name, condition, arm]['ospa'] for name in names]
                paired.append(dict(condition=condition, candidate=GCE, reference=arm, ospa=summary(differences, draws)))
    return dict(sequences=names, frames=sum(lookup[name, 'reliable', GCE]['frames'] for name in names),
                rows=rows, aggregate=aggregate, paired=paired)


def main():
    destination = OUT / 'MODEL_SENSITIVITY_ANALYSIS.json'
    assert not destination.exists()
    controls = json.loads((OUT / 'CONTROLS_ANALYSIS.json').read_text()); assert controls['passed']
    new = json.loads((OUT / 'NEW_DATA_ANALYSIS.json').read_text()); assert new['passed']
    sources = ['CONTROLS_ANALYSIS.json', 'NEW_DATA_ANALYSIS.json']
    uncompensated = {cohort: [row for row in group['rows'] if row['arm'] in ARMS]
                    for cohort, group in controls['cohorts'].items()}
    uncompensated['new_validation'] = [row for row in new['rows'] if row['arm'] in ARMS]
    motion, motion_effect, pd_groups = {}, [], {}
    for cohort, count in [('development', 9), ('seen_transfer', 25), ('new_validation', 3)]:
        name = 'audit_motion_' + cohort + '.json'
        audit = checked_audit(name); sources.append(name)
        assert audit['coordinate_adapter'] == 'planar' and audit['pd'] == .9
        group = summarize(audit['rows'], count); motion[cohort] = group
        before = {(row['sequence'], row['condition'], row['arm']): row for row in uncompensated[cohort]}
        after = {(row['sequence'], row['condition'], row['arm']): row for row in group['rows']}
        draws = np.random.default_rng(8301).integers(0, count, (10000, count)) if count >= 9 else None
        for condition in CONDITIONS:
            for arm in ARMS:
                values = [after[n, condition, arm]['ospa'] - before[n, condition, arm]['ospa'] for n in group['sequences']]
                motion_effect.append(dict(cohort=cohort, condition=condition, arm=arm,
                    contrast='Planar-compensated minus original current-ego CV', ospa=summary(values, draws)))
    for pd, stage in [(.7, 'pd070_development'), (.8, 'pd080_development'), (.95, 'pd095_development')]:
        name = 'audit_' + stage + '.json'
        audit = checked_audit(name); sources.append(name)
        assert audit['pd'] == pd and audit['cohort'] == 'development'
        pd_groups[str(pd)] = summarize(audit['rows'], 9)
    pd_groups['0.9'] = summarize(uncompensated['development'], 9)
    result = dict(passed=True, arms=ARMS, motion=motion, motion_effect=motion_effect, detection_probability=pd_groups,
        source_reports={name: sha(OUT / name) for name in sources}, analyzer_sha256=sha(Path(__file__)),
        interpretation='Matched model sensitivity, without detector, calibration, GCE or parameter reselection. Compensate planar yaw and translation after prediction and previous-scan birth creation. pD values change the assumed local detection probability only, not the measurements.',
        uncertainty='10000 paired whole-sequence percentile resamples, seed 8301, for the old 9/25 sequence cohorts; descriptive and unadjusted. No inferential interval for the three related new segments.',
        communication='Original seen-transfer No-age used 208-byte native packets; new motion runner retains the 216-byte legacy development format. Do not interpret these implementation bytes as a pure motion effect.')
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    output_rows = []
    for cohort, group in motion.items():
        output_rows.extend(dict(row, experiment='motion', cohort=cohort, pd=.9) for row in group['rows'])
    for pd, group in pd_groups.items():
        output_rows.extend(dict(row, experiment='pd', cohort='development', pd=float(pd)) for row in group['rows'])
    with (OUT / 'model_sensitivity_sequence_scores.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0])); writer.writeheader(); writer.writerows(output_rows)
    for name, group in [('motion_' + k, v) for k, v in motion.items()] + [('pd_' + k, v) for k, v in pd_groups.items()]:
        for row in group['paired']:
            print(name, row['condition'], row['reference'], row['ospa'], flush=True)


if __name__ == '__main__':
    main()
