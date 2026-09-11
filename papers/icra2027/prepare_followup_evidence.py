"""Import completed follow-up evidence and derive portable manuscript inputs."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / 'source_data'
SNAP = DATA / 'admission_followup'
GCE = 'marked_gaussian_evidence'
GS, FIXED = GCE + '_guarded_scalar', GCE + '_fixedx_000'
CONDITIONS = ['reliable', 'intermittent']
FILES = ['icra_admission_final/' + name for name in [
    'EVALUATION_ANALYSIS.json', 'EXTERNAL_ANALYSIS.json', 'REPLAY_ACCEPTANCE.json',
    'RESULT_FILES_MANIFEST.json', 'SUMMARY_AUDIT.json', 'verify_summary.py', 'RESULTS_CN.md']]
FILES += ['icra_compatible_admission/FINAL_SELECTION.json', 'icra_compatible_admission/DATA_EXPOSURE.json']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(name):
    return json.loads((SNAP / name).read_text())


def derive():
    manifest = json.loads((DATA / 'followup_source_manifest.json').read_text())
    assert set(manifest) == set(FILES)
    for rel, record in manifest.items():
        assert sha(SNAP / rel) == record['sha256'], rel
    verifier = SNAP / 'icra_admission_final/verify_summary.py'
    spec = importlib.util.spec_from_file_location('followup_summary_verifier', verifier)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.main()
    evaluation = read('icra_admission_final/EVALUATION_ANALYSIS.json')
    selected = read('icra_compatible_admission/FINAL_SELECTION.json')
    acceptance = read('icra_admission_final/REPLAY_ACCEPTANCE.json')
    assert acceptance['passed'] and acceptance['all_native_unit_exit_codes_zero']
    assert acceptance['completed_native_result_files'] == 458
    assert acceptance['independently_audited_node_frames'] == 187444
    assert acceptance['result_manifest_sha256'] == sha(SNAP / 'icra_admission_final/RESULT_FILES_MANIFEST.json')
    assert selected['selected_fixed']['eta'] == 0
    assert selected['fixed_strength_grid'] == [0, .05, .1, .125, .25, .5, 1]
    original = json.loads((DATA / 'gaussian_paper_evidence.json').read_text())
    reviewer = json.loads((DATA / 'reviewer_evidence.json').read_text())
    names = original['sequences']
    rows = [r for r in evaluation['rows'] if r['dataset'] == 'v2v' and r['split'] == 'train' and r['sequence'] in names]
    index = {(r['sequence'], r['condition'], r['arm']): r for r in rows}
    assert len(index) == len(rows)
    seen = []
    for condition in CONDITIONS:
        for arm in ['marked_lineage', 'marked_asymmetric', GS, GCE, FIXED]:
            group = [index[n, condition, arm] for n in names]
            assert sum(r['frames'] for r in group) == 5601
            values = {m: {'mean': float(np.mean([r[m] for r in group]))} for m in ['ospa', 'miss2', 'false2']}
            if arm != FIXED:
                source = reviewer['controls']['cohorts']['seen_transfer']['aggregate'] if arm == GS else original['aggregate']
                old = next(r for r in source if r['condition'] == condition and r['arm'] == arm)
                for m in values:
                    assert np.isclose(old[m]['mean'], values[m]['mean'], atol=1e-10, rtol=1e-12)
                old_rows = reviewer['controls']['cohorts']['seen_transfer']['rows'] if arm == GS else original['runs']
                previous = {r['sequence']: r for r in old_rows if r['condition'] == condition and r['arm'] == arm}
                assert set(previous) == set(names)
                for current in group:
                    before = previous[current['sequence']]
                    assert before['frames'] == current['frames']
                    for metric in ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']:
                        assert np.isclose(before[metric], current[metric], atol=1e-10, rtol=1e-12)
            seen.append(dict(condition=condition, arm=arm, **values))
    draws = np.random.default_rng(8301).integers(0, 25, (10000, 25))
    paired = []
    for condition in CONDITIONS:
        d = np.array([index[n, condition, GCE]['ospa']-index[n, condition, FIXED]['ospa'] for n in names])
        low, high = np.percentile(d[draws].mean(1), [2.5, 97.5])
        paired.append(dict(condition=condition, mean=float(d.mean()), low=float(low), high=float(high)))
    arms = ['marked_lineage', 'marked_er', 'marked_asymmetric', GS, GCE+'_no_curvature', FIXED, GCE]
    aggregate = [r for r in evaluation['aggregates'] if r['scope'] == 'all' and r['arm'] in arms]
    assert len(aggregate) == 28
    gce_gs = [r for r in evaluation['paired'] if r['dataset'] == 'v2v' and r['scope'] == 'all'
              and r['candidate'] == GCE and r['reference'] == GS]
    assert len(gce_gs) == 2
    return dict(source_manifest_sha256=sha(DATA/'followup_source_manifest.json'),
                generator_sha256=sha(Path(__file__)), datasets=evaluation['datasets'],
                methods=arms, aggregate=aggregate, seen_25=seen, seen_fixed_paired=paired,
                recording_gce_gs=gce_gs, selected_fixed=selected['selected_fixed'],
                fixed_grid=selected['fixed_strength_grid'], native_result_files=458, native_robot_frames=187444,
                evaluation_rows=854, recomputed_paired_comparisons=180,
                scope='Previously exposed V2V corpus; V2X evaluated after final candidate selection, now exposed for later work. No prospective validation claim.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path)
    args = parser.parse_args()
    if args.source_root:
        manifest = {}
        for rel in FILES:
            source = args.source_root / 'trials' / rel; destination = SNAP / rel
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            manifest[rel] = dict(source='trials/'+rel, sha256=sha(source), bytes=source.stat().st_size)
        (DATA/'followup_source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    evidence = derive()
    (DATA/'followup_evidence.json').write_text(json.dumps(evidence, indent=2)+'\n')
    print('Prepared verified 43-segment V2V / five-segment V2X evidence and the selected fixed-zero control.')


if __name__ == '__main__':
    main()
