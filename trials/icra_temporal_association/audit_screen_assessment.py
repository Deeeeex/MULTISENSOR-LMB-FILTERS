"""Independent native-stage audit of the two frozen association candidates."""
from pathlib import Path
import argparse
import csv
import json
import re
import sys

import numpy as np
from scipy.io import loadmat

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
for folder in ['icra_gaussian_evidence', 'icra_marked_control', 'icra_fusion_holdout', 'icra_reviewer_revision']:
    sys.path.insert(0, str(OUT.parent / folder))
sys.path.insert(0, str(OUT))
from analyze_control import sha, read, score_run
from analyze_holdout import radio_draws
from review_probability_audit import audit_probability
from audit_observation_run import current_records
from audit_persistent_run import associations as persistent_associations
from audit_observation_run import associations as observation_associations
from audit_observation_domain import audit_domain
from audit_screened_run import associations as screened_associations
from analyze_holdout import audit_existence
from analyze_full import baseline


def packets(run, delivered, T, mode):
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    counts = np.zeros((2, T), int)
    for row in local:
        counts[int(row[1]) - 1, int(row[0]) - 1] += 1
    width = 416 if mode else (352 if 'gaussian_evidence' in run['arm'] else 216)
    expected = 32 + width * counts
    assert np.array_equal(run['packetBytes'], expected)
    assert run['attemptedMessages'] == [2] * T and run['controlBytes'] == [256] * T
    assert np.array_equal(run['deliveredMessages'], delivered.sum((0, 1)))
    assert np.array_equal(run['rawPayloadBytes'], expected.sum(0))
    assert np.array_equal(run['deliveredRawBytes'], (expected * delivered.sum(0)).sum(0))
    assert np.array_equal(run['wireBytes'], np.ceil(expected / 16384).sum(0) * 16384 + 256)
    assert run['totalWireBytes'] == sum(run['wireBytes'])
    # The existing Gaussian auditor checks the unchanged 352-byte base.
    base_view = dict(run, arm=run['arm'].split('_assoc_', 1)[0])
    base_width = 352 if 'gaussian_evidence' in base_view['arm'] else 216
    base_view['packetBytes'] = (32 + base_width * counts).tolist()
    return base_view


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage')
    args = parser.parse_args()
    cfgpath = OUT / 'stages' / (args.stage + '.json')
    runtimepath = OUT / ('runtime_' + args.stage + '.json')
    cfg, runtime = json.loads(cfgpath.read_text()), json.loads(runtimepath.read_text())
    assert len(runtime) == len(cfg['units'])
    for name, expected in cfg['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    source = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in cfg['source_sha256'].items()}
    assert len(source) == len(cfg['source_sha256'])
    rows, diagnostics, parity, hashes = [], [], [], {}
    assert cfg['cohort'].startswith('association_screen_assessment_')
    if not cfg['preflight']:
        assert sha(OUT / 'SCREENED_DEVELOPMENT_SELECTION.json') == cfg['selection_sha256']
    for unit, native in zip(cfg['units'], runtime):
        seq = unit['sequence']
        assert native['sequence'] == seq and native['returncode'] == 0 and native['completion_line']
        assert native['files'] == 2 * len(cfg['arms'])
        log = ROOT / 'RUN/ICRA_TEMPORAL_ASSOCIATION' / args.stage / (seq + '.log')
        content = log.read_text()
        assert 'SCREENED ASSOCIATION CHECK PASSED' in content
        assert f'COMPLETED REVIEW {args.stage} {seq}' in content
        hashes[str(log.relative_to(ROOT))] = sha(log)
        mat = loadmat(ROOT / unit['data_path'])
        T = int(mat['T'].item())
        for condition in cfg['conditions']:
            for arm in cfg['arms']:
                mode = arm.rsplit('_assoc_', 1)[1] if '_assoc_' in arm else ''
                path = OUT / 'results' / args.stage / f'{seq}_{condition}_{arm}.json.gz'
                data = read(path)
                run = data['runs']
                assert data['protocol'] == 'icra-screened-association-v1'
                assert data['stage'] == args.stage and data['cohort'] == cfg['cohort']
                assert data['sourceSha256'] == source and data['inputSha256'] == unit['input_sha256']
                assert data['sequence'] == seq and data['condition'] == condition and run['arm'] == arm
                assert data['pd'] == cfg['pd']
                assert np.array_equal(data['time'], mat['time'].ravel())
                assert np.array_equal(data['positions'], mat['positions'])
                for t in range(T):
                    assert np.array_equal(np.asarray(data['truth'][t]).reshape(4, -1), mat['truth'][0, t])
                    assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(), mat['truthIds'][0, t].ravel())
                delivered = radio_draws(unit['radio_seed'] - 8301, T, condition)
                assert np.array_equal(data['delivered'], delivered)
                base_view = packets(run, delivered, T, mode)
                domain = audit_domain(run, data)
                if arm == 'marked_lineage':
                    short_view = dict(base_view, iterationRecords=np.asarray(run['iterationRecords'], float).reshape(-1, 60)[:, :26].tolist())
                    density = audit_existence(short_view, data, 'marked_lineage')
                else:
                    density = audit_probability(base_view, data)
                frames, direct = current_records(run, mat, bool(mode))
                if mode in ['quality', 'nis', 'quality_nis']:
                    matching = screened_associations(run, data, frames, mode)
                elif mode:
                    assert not any(run[k] for k in ['associationRemoteAbstentions', 'associationSplits', 'associationConflicts'])
                    matching = observation_associations(run, data, frames, mode)
                else:
                    matching = {}
                diagnostics.append(dict(sequence=seq, condition=condition, arm=arm,
                    current_observation=direct, association=matching, density=density, observation_domain=domain))
                value, _ = score_run(data, run)
                rows.append(dict(sequence=seq, condition=condition, arm=arm, frames=T, **value))
                assert mode in ['', 'quality', 'nis', 'quality_nis']
                if cfg['preflight']:
                    assert arm == 'marked_lineage' and seq == '0000'
                    original, _, _, oldpath = baseline('development', seq, condition, arm)
                    oldrun = next(r for r in original['runs'] if r['arm'] == arm)
                    keys = ['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount',
                        'rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages',
                        'deliveredMessages','maximumBernoulliCount']
                    for key in keys:
                        assert run[key] == oldrun[key], (seq, condition, arm, key, 'exact No-age recursion')
                    assert np.array_equal(np.asarray(run['iterationRecords'])[:, :26], np.asarray(oldrun['iterationRecords']).reshape(-1, 26))
                    parity.append(dict(sequence=seq, condition=condition, arm=arm, exact_fields=keys + ['iterationRecords_first26'], robot_frames=2*T))
                    hashes[str(oldpath.relative_to(ROOT))] = sha(oldpath)
                hashes[str(path.relative_to(ROOT))] = sha(path)
                print('AUDITED ASSOCIATION', args.stage, seq, condition, arm, flush=True)
    result = dict(passed=True, stage=args.stage, rows=rows, diagnostics=diagnostics, parity=parity,
        audited_robot_frames=sum(2 * r['frames'] for r in rows), inputs=hashes,
        config_sha256=sha(cfgpath), runtime_sha256=sha(runtimepath),
        auditor_sha256={str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__),
            OUT / 'screen_observation_math.py', OUT / 'screen_persistent_math.py', OUT / 'audit_screened_run.py',
            OUT.parent / 'icra_fusion_holdout/analyze_holdout.py',
            OUT / 'audit_observation_domain.py', OUT / 'persistent_math.py', OUT / 'audit_persistent_run.py', OUT / 'association_math.py',
            OUT / 'audit_observation_run.py', OUT / 'observation_math.py',
            OUT.parent / 'icra_reviewer_revision/review_probability_audit.py',
            OUT.parent / 'icra_reviewer_revision/review_gaussian_audit.py']})
    destination = OUT / ('audit_' + args.stage + '.json')
    assert not destination.exists()
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / ('scores_' + args.stage + '.csv')).open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print('ASSOCIATION STAGE AUDIT PASSED', args.stage, result['audited_robot_frames'], 'robot-frames', flush=True)


if __name__ == '__main__':
    main()
