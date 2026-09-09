"""Prepare the six missing scenes and freeze both stages before outcomes."""
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import re
import sys

import numpy as np
from scipy.io import loadmat, savemat
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
REVIEW = OUT.parent / 'icra_reviewer_revision'
PRIOR = OUT.parent / 'icra_method_iteration'
AUTHOR = ROOT / 'tmp/external_baselines/DMSTrack'
sys.path.insert(0, str(OUT.parent / 'icra_external_fusion'))
from prepare_v2v4real import in_domain

ARMS = ['marked_lineage', 'marked_er', 'marked_asymmetric',
        'marked_gaussian_evidence_guarded_scalar',
        'marked_gaussian_evidence_no_curvature',
        'marked_gaussian_evidence_fixed_025', 'marked_gaussian_evidence']
SELECTED = [5, 10, 15, 20, 25, 30]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    assert not path.exists(), path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def inventory():
    groups = defaultdict(lambda: defaultdict(lambda: defaultdict(set)))
    paths = {}
    for path in sorted((ROOT / 'tmp/external_baselines/v2v_official').glob('*/directory.json')):
        split = path.parent.name.split('_')[0]
        if split not in ['train', 'test', 'val']:
            continue
        paths[str(path.relative_to(ROOT))] = sha(path)
        for entry in json.loads(path.read_text())['entries']:
            parts = entry['name'].split('/')
            if len(parts) >= 3 and parts[-1].endswith('.yaml') and parts[-2] in ['0', '1']:
                groups[split][parts[-3]][parts[-2]].add(parts[-1])
    assert len(paths) == 12
    rows = []
    for split in ['train', 'test', 'val']:
        for i, (scene, sensors) in enumerate(sorted(groups[split].items())):
            rows.append(dict(split=split, sequence=f'{i:04d}', scene=scene,
                             recording=re.sub(r'_\d+$', '', scene),
                             frames=len(sensors['0'] & sensors['1']),
                             sensor_frames=[len(sensors['0']), len(sensors['1'])]))
    assert [sum(r['split'] == split for r in rows) for split in ['train', 'test', 'val']] == [32, 9, 3]
    unique = {}
    for row in rows:
        if row['scene'] in unique:
            assert row['sequence'] == unique[row['scene']]['sequence'] == '0000'
            assert row['frames'] == unique[row['scene']]['frames'] == 147
        unique[row['scene']] = row
    assert len(unique) == 43 and sum(r['frames'] for r in unique.values()) == 9699
    return dict(published_scenarios=67, release_entries=44, unique_scenes=43,
                unique_paired_frames=9699, rows=rows, inputs=paths,
                duplicate_rule='Keep test/development 0000 once; omit duplicate train 0000.',
                published_scenario_to_release_directory_mapping='unverified')


def main():
    if sys.argv[1:] == ['--register-only']:
        existing = json.loads((OUT / 'INPUT_MANIFEST.json').read_text())
        for name, expected in existing['source_sha256'].items():
            assert sha(ROOT / name) == expected, name
        register(existing['sequences'], existing['source_sha256'])
        return
    assert not sys.argv[1:]
    assert not (OUT / 'INPUT_MANIFEST.json').exists()
    catalog = inventory()
    calpath = OUT.parent / 'icra_ceiling_iteration/calibration.json'
    priorpath = OUT.parent / 'icra_marked_iteration/likelihood_manifest.json'
    cal = json.loads(calpath.read_text())['full_seen_fit']
    prior = json.loads(priorpath.read_text())['full_seen_positive_prior']
    assert cal['training_sequences'] == [f'{i:04d}' for i in range(9)]
    manifest_path = PRIOR / 'transfer_input_manifest.json'
    original = json.loads(manifest_path.read_text())
    sources = {str(p.relative_to(ROOT)): sha(p) for p in [calpath, priorpath, manifest_path]}
    rows = []
    for seq in SELECTED:
        name = f'{seq:04d}'
        old = PRIOR / 'data_transfer' / f'v2v4real_{name}.mat'
        expected = next(r for r in original['sequences'] if r['sequence'] == name)
        assert sha(old) == expected['input_sha256']
        sources[str(old.relative_to(ROOT))] = sha(old)
        data = {k: v for k, v in loadmat(old).items() if not k.startswith('__')}
        T = int(data['T'].item())
        raw_scores = np.empty((2, T), object)
        calibrated = np.empty((2, T), object)
        ratios = np.empty((2, T), object)
        for n, sensor in enumerate(['ego', '1']):
            path = AUTHOR / f'AB3DMOT/data/v2v4real/detection/multi_sensor_differentiable_kalman_filter_Car_train/{sensor}/{name}.txt'
            assert sha(path) == original['source_files'][str(path.relative_to(AUTHOR))]
            sources[str(path.relative_to(ROOT))] = sha(path)
            raw = np.loadtxt(path, delimiter=',', ndmin=2) if path.stat().st_size else np.empty((0, 15))
            assert np.isfinite(raw).all() and raw.shape[1] == 15
            for t in range(T):
                frame = raw[raw[:, 0] == t]
                xy = frame[:, [10, 12]]
                pose = data['positions'][:, :, t].T
                mask = in_domain(xy, pose) & (((xy - pose[n]) ** 2).sum(1) <= 1600)
                assert np.array_equal(xy[mask].T, data['measurements'][n, t]), (name, n, t)
                score = frame[mask, 6].reshape(1, -1)
                bounded = np.clip(score, 1e-6, 1 - 1e-6)
                probability = expit(cal['a'] * (np.log(bounded) - np.log1p(-bounded)) + cal['b'])
                raw_scores[n, t] = score
                calibrated[n, t] = probability
                probability = np.clip(probability, 1e-6, 1 - 1e-6)
                ratios[n, t] = probability / (1 - probability) * (1 - prior) / prior
        data.update(rawScores=raw_scores, calibratedScores=calibrated, likelihoodRatios=ratios)
        dest = OUT / 'data' / f'v2v4real_{name}.mat'
        assert not dest.exists()
        dest.parent.mkdir(exist_ok=True)
        savemat(dest, data, do_compression=True)
        with dest.open('r+b') as handle:
            handle.write(b'MATLAB 5.0 MAT-file, deterministic full-coverage V2V4Real input'.ljust(116, b' '))
        check = loadmat(dest)
        for key, value in loadmat(old).items():
            if key.startswith('__'):
                continue
            if value.dtype == object:
                assert all(np.array_equal(a, b) for a, b in zip(value.flat, check[key].flat)), key
            else:
                assert np.array_equal(value, check[key]), key
        scene = next(r for r in catalog['rows'] if r['split'] == 'train' and r['sequence'] == name)
        assert scene['frames'] == T
        rows.append(dict(**scene, data_path=str(dest.relative_to(ROOT)), input_sha256=sha(dest), radio_seed=8301 + seq))
        sources[str(dest.relative_to(ROOT))] = sha(dest)
        print('PREPARED', name, T, 'complete frames; geometry unchanged', flush=True)
    assert sum(r['frames'] for r in rows) == 1357
    write_json(OUT / 'DATA_INVENTORY.json', catalog)
    write_json(OUT / 'INPUT_MANIFEST.json', dict(sequences=rows, frames=1357, source_sha256=sources,
               calibration='Existing full-nine fit; no refit.', coordinate_parity='All original MAT fields identical.'))
    register(rows, sources)


def register(rows, input_sources):
    assert not list((OUT / 'stages').glob('*.json'))
    # Long raw detection paths collide after MATLAB jsondecode's 63-character
    # field conversion. Preserve every raw hash in the frozen input manifest;
    # the Python launcher and final checker verify them without conversion.
    sources = {name: value for name, value in input_sources.items()
               if not name.startswith('tmp/external_baselines/DMSTrack/AB3DMOT/data/v2v4real/detection/')}
    old_sources = json.loads((OUT.parent / 'icra_gaussian_evidence/source_sha256.json').read_text())
    for name, expected in old_sources.items():
        assert sha(ROOT / name) == expected, name
    protected = list(OUT.glob('*.py')) + list(OUT.glob('*.m')) + [OUT / 'PROTOCOL.md', OUT / 'DATA_INVENTORY.json', OUT / 'INPUT_MANIFEST.json']
    protected += [REVIEW / x for x in ['runReviewerReplay.m', 'checkReviewerEvidence.m', 'fuseReviewerEvidence.m', 'audit_stage.py', 'review_probability_audit.py', 'review_gaussian_audit.py']]
    sources.update(old_sources)
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    reused = [OUT.parent / f'icra_gaussian_evidence/summary_{cohort}.json' for cohort in ['development', 'seen_transfer']]
    reused += [REVIEW / f'audit_{stage}.json' for stage in ['controls_development', 'gs_seen_transfer', 'fixed025_seen_transfer', 'new_validation_primary', 'new_validation_recency']]
    reused += [REVIEW / 'REPLAY_ACCEPTANCE.json', REVIEW / 'RESULT_FILES_MANIFEST.json']
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in reused})
    preflight = dict(sequence='0000', data_path='trials/icra_external_fusion/data/v2v4real_0000.mat',
                     marks_path='trials/icra_ceiling_iteration/data_marks/marks_0000.mat',
                     ratios_path='trials/icra_marked_iteration/data_likelihoods/likelihoods_0000.mat', radio_seed=8301)
    preflight['input_sha256'] = sha(ROOT / preflight['data_path'])
    for key in ['data_path', 'marks_path', 'ratios_path']:
        sources[preflight[key]] = sha(ROOT / preflight[key])
    mapped = [re.sub('[^A-Za-z0-9_]', '_', name)[:63] for name in sources]
    assert len(set(mapped)) == len(mapped), 'MATLAB field-name collision'
    for stage, units, arms, check in [
            ('coverage_preflight', [preflight], ['marked_asymmetric', 'marked_gaussian_evidence'], True),
            ('coverage_remaining_train', rows, ARMS, False)]:
        write_json(OUT / 'stages' / f'{stage}.json', dict(protocol='icra-full-public-coverage-v1', stage=stage,
                   cohort='development' if check else 'previous_train_probes', created_utc=datetime.now(timezone.utc).isoformat(),
                   arms=arms, pd=.9, preflight=check, conditions=['reliable', 'intermittent'], units=units, source_sha256=sources))
    print('REGISTERED: 4 preflight outputs, then 84 full-sequence outputs; all 43 unique scenes covered after reuse.', flush=True)


if __name__ == '__main__':
    main()
