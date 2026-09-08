"""Independent scoring and matched-support checks for the extra CR control."""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import re
import sys
import time

import numpy as np
from scipy.io import loadmat

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PORT = OUT.parent / 'icra_fusion_holdout'
sys.path.insert(0, str(PORT))
from analyze_holdout import audit_existence, audit_packets, domain, interval, radio_draws, score

ARM = 'marked_conservative'
REFERENCES = ['marked_ceiling_score', 'marked_lineage', 'marked_er']
METRICS = ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError', 'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


def source_check():
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    mapped = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in source.items()}
    assert len(mapped) == len(source)
    return mapped


def score_run(data, run):
    T = len(data['time'])
    poses = np.asarray(data['positions'])
    values = {key: [] for key in METRICS[:6]}
    matched = []
    assert len(run['rawEstimates']) == len(run['estimates']) == len(run['labels']) == 2 * T
    for t in range(T):
        for n in range(2):
            index = n + 2 * t
            raw = np.asarray(run['rawEstimates'][index], float).reshape(-1, 4)
            output = np.asarray(run['estimates'][index], float).reshape(-1, 4)
            labels = np.asarray(run['labels'][index]).reshape(2, -1)
            assert np.isfinite(raw).all() and labels.shape[1] == len(raw)
            assert np.unique(labels, axis=1).shape[1] == len(raw)
            assert np.array_equal(output, raw[domain(raw, poses[:, :, t])])
            value = score(data['truth'][t], output)
            for key in ['ospa', 'countError', 'matchedSquaredError', 'matchedCount']:
                assert np.isclose(value[key], run[key][n][t], atol=1e-8, rtol=1e-9)
            for key in values:
                values[key].append(value[key])
            matched.append(value['match_d2'])
    row = {key: float(np.mean(v)) for key, v in values.items()}
    row.update(raw_bytes=sum(run['rawPayloadBytes']), delivered_raw_bytes=sum(run['deliveredRawBytes']),
               wire_bytes=sum(run['wireBytes']), runtime_s=run['runtimeSeconds'])
    return row, np.concatenate(matched)


def audit_file(path, source, mat, cohort, name, condition, arm):
    data = read(path)
    assert data['protocol'] == 'shared-information-conservative-control-v1'
    assert data['implementation'] == 'marked-conservative-v1' and data['sourceSha256'] == source
    assert data['cohort'] == cohort and data['sequence'] == name and data['condition'] == condition
    assert data['inputSha256'] == sha(mat) and data['runs']['arm'] == arm and not data['smoke']
    original = loadmat(mat)
    T = int(original['T'].item())
    assert np.array_equal(data['time'], original['time'].ravel())
    assert np.array_equal(data['positions'], original['positions'])
    for t in range(T):
        assert np.array_equal(np.asarray(data['truth'][t]).reshape(4, -1), original['truth'][0, t])
        assert np.array_equal(np.asarray(data['truthIds'][t]).ravel(), original['truthIds'][0, t].ravel())
    delivery = radio_draws(int(name), T, condition)
    assert np.array_equal(data['delivered'], delivery)
    run = data['runs']
    audit_packets(run, delivery, T, {})
    diagnostic = audit_existence(run, data, ARM)
    if arm == ARM:
        records = np.asarray(run['iterationRecords'], float).reshape(-1, 26)
        assert np.all(records[:, 19:21] == 0)
        assert np.allclose(records[:, 6], np.minimum(records[:, 7], records[:, 8]), atol=1e-14, rtol=0)
    row, matched = score_run(data, run)
    return data, dict(sequence=name, condition=condition, arm=arm, frames=T, **row), matched, diagnostic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    source = source_check()
    assert args.preflight, 'Full-cohort entry point is added after the preflight evidence is validated.'
    rows = []
    hashes = {}
    for condition in ['reliable', 'intermittent']:
        mat = OUT.parent / 'icra_external_fusion/data/v2v4real_0000.mat'
        for arm in ['marked_er', ARM]:
            path = OUT / 'results_development_check' / f'0000_{condition}_{arm}.json.gz'
            data, row, _, diagnostic = audit_file(path, source, mat, 'development_check', '0000', condition, arm)
            rows.append(dict(**row, diagnostics=diagnostic))
            hashes[str(path.relative_to(ROOT))] = sha(path)
            if arm == 'marked_er':
                original_path = PORT / 'results_development_check' / f'0000_{condition}_marked_er.json.gz'
                original = read(original_path)
                hashes[str(original_path.relative_to(ROOT))] = sha(original_path)
                for key, value in data['runs'].items():
                    if key != 'runtimeSeconds':
                        assert value == original['runs'][key], (condition, 'ER port equivalence', key)
    result = dict(passed=True, new_node_frames=sum(2 * r['frames'] for r in rows),
                  er_exact_node_frames=sum(2 * r['frames'] for r in rows if r['arm'] == 'marked_er'),
                  er_all_output_and_diagnostic_fields_exact_except_runtime=True,
                  conservative_probability_and_same_input_extraction_verified=True,
                  independent_radio_and_packet_accounting=True, rows=rows, source_files=len(source),
                  input_sha256=hashes, auditor_sha256=sha(Path(__file__)))
    assert result['new_node_frames'] == 1176 and result['er_exact_node_frames'] == 588
    (OUT / 'preflight_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('CONTROL PREFLIGHT PASSED:', result['new_node_frames'], 'independently scored node-frames;',
          result['er_exact_node_frames'], 'exact ER node-frames; conservative equation and extraction verified.', flush=True)


if __name__ == '__main__':
    main()
