"""Assemble portable paper data from complete, verified experiment summaries."""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / 'source_data'
PRIMARY = 'marked_gaussian_evidence'
METHODS = ['marked_local', 'marked_mil_support', 'marked_tc_ospa2_w5', 'marked_tc_ospa2_w10',
           'marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_score',
           'marked_ceiling_calibrated', 'marked_asymmetric', PRIMARY]
LABELS = dict(zip(METHODS, ['Local', 'MIL-AM', 'TC-5', 'TC-10', 'No-age KLA', 'Recency',
                          'Conservative', 'Capped-score', 'Capped-cal.', 'Scalar', 'GCE']))
LABELS.update({PRIMARY+'_no_curvature': 'No curvature guard', PRIMARY+'_no_history': 'No history switch',
               PRIMARY+'_no_mark': 'No positive score constraint'})
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError', 'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']
SNAPSHOTS = {
    'gaussian_main_summary.json': 'icra_gaussian_evidence/summary_seen_transfer.json',
    'gaussian_development_summary.json': 'icra_gaussian_evidence/summary_development.json',
    'gaussian_main_qa.json': 'icra_gaussian_evidence/CURRENT_QA_seen_transfer.json',
    'gaussian_development_qa.json': 'icra_gaussian_evidence/CURRENT_QA_development.json',
    'gaussian_main_components.json': 'icra_gaussian_components/summary_seen_transfer.json',
    'gaussian_development_components.json': 'icra_gaussian_components/summary_development.json',
    'gaussian_codec_main_summary.json': 'icra_gaussian_zero_codec/summary_seen_transfer.json',
    'gaussian_codec_main_qa.json': 'icra_gaussian_zero_codec/CURRENT_QA_seen_transfer.json',
    'gaussian_codec_development_summary.json': 'icra_gaussian_zero_codec/summary_development.json',
    'gaussian_codec_development_qa.json': 'icra_gaussian_zero_codec/CURRENT_QA_development.json',
    'gaussian_round.json': 'icra_gaussian_evidence/ROUND_FREEZE.json',
    'shared_main_summary.json': 'icra_fusion_holdout/summary_holdout.json',
    'shared_main_qa.json': 'icra_fusion_holdout/CURRENT_QA.json',
    'score_calibration.json': 'icra_ceiling_iteration/calibration.json',
    'score_input_diagnostic.json': 'icra_fusion_holdout/input_shift.json',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interval(values, draws):
    values = np.asarray(values, dtype=float)
    low, high = np.quantile(values[draws].mean(1), [.025, .975])
    return dict(mean=float(values.mean()), sd=float(values.std(ddof=1)), low=float(low), high=float(high), n=len(values))


def main():
    OUT.mkdir(exist_ok=True)
    manifest_path = OUT / 'gaussian_source_manifest.json'
    in_repository = (ROOT / 'trials/icra_gaussian_evidence/CURRENT_QA_seen_transfer.json').exists()
    if in_repository:
        manifest = {}
        for name, relative in SNAPSHOTS.items():
            original = ROOT / 'trials' / relative
            assert original.exists(), original
            shutil.copyfile(original, OUT / name)
            manifest[name] = dict(source='trials/'+relative, sha256=sha(original))
        manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    else:
        manifest = json.loads(manifest_path.read_text())
    assert set(manifest) == set(SNAPSHOTS)
    for name, record in manifest.items():
        assert sha(OUT / name) == record['sha256'], name
    data = {name.removesuffix('.json'): json.loads((OUT / name).read_text()) for name in SNAPSHOTS}
    main_data = data['gaussian_main_summary']
    original = data['shared_main_summary']
    codec = data['gaussian_codec_main_summary']
    components = data['gaussian_main_components']
    assert main_data['primary'] == PRIMARY and main_data['sequences'] == original['sequences'] == 25
    assert main_data['frames'] == original['frames'] == 5601
    for cohort in ['main', 'development']:
        qa = data[f'gaussian_{cohort}_qa']
        assert qa['passed'] and qa['summary_sha256'] == sha(OUT / f'gaussian_{cohort}_summary.json')
        assert all(code == 0 for code in qa['native_matlab_stage_exits'])
        cq = data[f'gaussian_codec_{cohort}_qa']
        cs = data[f'gaussian_codec_{cohort}_summary']
        assert cq['passed'] and cq['exact_whole_trajectory_parity']
        assert cq['summary_sha256'] == sha(OUT / f'gaussian_codec_{cohort}_summary.json')
        assert cs['passed'] and cs['reference_summary_sha256'] == sha(OUT / f'gaussian_{cohort}_summary.json')
        assert all(code == 0 for code in cq['native_matlab_exits'])
    sq = data['shared_main_qa']
    assert sq['passed'] and sq['artifact_sha256']['trials/icra_fusion_holdout/summary_holdout.json'] == sha(OUT / 'shared_main_summary.json')
    assert components['source_summary_sha256'] == sha(OUT / 'gaussian_main_summary.json')
    inputs = {**main_data['inputs'], **codec['inputs']}
    for row in original['inputs']:
        if row['arm'] in METHODS + ['lineage', 'qualified_exist']:
            path = f"trials/icra_fusion_holdout/results_holdout/{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
            inputs[path] = row['sha256']
    if in_repository:
        for path, expected in inputs.items():
            assert sha(ROOT / path) == expected, path
    names = sorted({r['sequence'] for r in main_data['runs']})
    assert len(names) == 25
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in original['runs']}
    for row in main_data['runs']:
        key = row['sequence'], row['condition'], row['arm']
        if key in lookup:
            assert all(np.isclose(row[m], lookup[key][m], atol=1e-9, rtol=1e-12) for m in METRICS)
        lookup[key] = row
    codecs = {(r['sequence'], r['condition']): r for r in codec['rows']}
    rows = []
    for condition in ['reliable', 'intermittent']:
        for name in names:
            for method in METHODS:
                row = dict(lookup[name, condition, method])
                if method == PRIMARY:
                    replay = codecs[name, condition]
                    assert all(np.isclose(replay[m], row[m], atol=1e-10, rtol=0) for m in METRICS[:6])
                    for m in METRICS[-3:]:
                        row['full_gaussian_'+m] = row[m]
                        row[m] = replay[m]
                rows.append(row)
    draws = np.random.default_rng(8301).integers(0, 25, (10000, 25))
    aggregate, paired = [], []
    paired_lookup = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    for condition in ['reliable', 'intermittent']:
        for method in METHODS:
            group = [paired_lookup[name, condition, method] for name in names]
            aggregate.append(dict(condition=condition, arm=method, **{m: interval([r[m] for r in group], draws) for m in METRICS}))
            if method != PRIMARY:
                differences = {m: [paired_lookup[name, condition, PRIMARY][m]-paired_lookup[name, condition, method][m] for name in names] for m in METRICS}
                paired.append(dict(condition=condition, candidate=PRIMARY, reference=method,
                    **{m: interval(values, draws) for m, values in differences.items()},
                    ospa_wins=sum(x < -1e-10 for x in differences['ospa']),
                    ospa_differences=differences['ospa'], sequences=names))
    primary_pairs = [r for r in paired if r['reference'] in ['marked_lineage', 'marked_er']]
    assert len(primary_pairs) == 4 and all(r['ospa']['high'] < 0 for r in primary_pairs)
    scalar_pairs = [r for r in components['comparisons'] if r['reference'] == 'marked_asymmetric']
    assert len(scalar_pairs) == 2 and all(r['ospa']['high'] < 0 for r in scalar_pairs)
    evidence = dict(primary=PRIMARY, methods=METHODS, labels=LABELS, sequences=names, frames=5601,
        aggregate=aggregate, paired=paired, runs=rows, components=components['comparisons'],
        component_sequence_pairs=components['rows'], common=main_data['common_aggregate'],
        diagnostics=main_data['diagnostics'], development=data['gaussian_development_summary']['aggregate'],
        unmarked_controls=[r for r in original['aggregate'] if r['arm'] in ['lineage', 'qualified_exist']],
        codec=data['gaussian_codec_main_qa']['aggregate'], all_real_outcomes_previously_seen=True,
        primary_costs='Native exact-zero codec; full Gaussian encoding retained as the transport control.',
        source_manifest_sha256=sha(manifest_path), source_inputs_sha256=inputs,
        generator_sha256=sha(Path(__file__)))
    (OUT / 'gaussian_paper_evidence.json').write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')
    print('PAPER EVIDENCE COMPLETE: 25 sequences; 11 methods; 20 paired comparisons;', len(inputs), 'source result identities.')


if __name__ == '__main__':
    main()
