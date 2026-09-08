"""Check whole saved trajectories exactly and independently count variable bytes."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import sys
import time
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PARENT = OUT.parent / 'icra_gaussian_evidence'
sys.path.insert(0, str(OUT.parent / 'icra_marked_control'))
from analyze_control import read, score_run

PRIMARY = 'marked_gaussian_evidence'
ALL_ARMS = [PRIMARY, PRIMARY+'_no_curvature', PRIMARY+'_no_history', PRIMARY+'_no_mark']
BYTE_KEYS = ['packetBytes', 'rawPayloadBytes', 'deliveredRawBytes', 'wireBytes', 'totalWireBytes']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_check():
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    mapped = {re.sub('[^A-Za-z0-9_]', '_', name)[:63]: value for name, value in source.items()}
    assert len(mapped) == len(source)
    return mapped


def audit_one(cohort, seq, condition, arm, source, reference_summary):
    name = f'{seq:04d}'
    path = OUT / f'results_{cohort}' / f'{name}_{condition}_{arm}.json.gz'
    ref_cohort = 'development' if cohort == 'development_check' else cohort
    original = PARENT / f'results_{ref_cohort}' / f'{name}_{condition}_{arm}.json.gz'
    assert sha(original) == reference_summary['inputs'][str(original.relative_to(ROOT))]
    data, old = read(path), read(original)
    assert data['protocol'] == 'coherent-gaussian-zero-codec-v1'
    assert data['implementation'] == 'gaussian-zero-codec-v1' and data['sourceSha256'] == source
    assert data['cohort'] == cohort and data['sequence'] == name and data['condition'] == condition
    for key in ['time', 'truth', 'truthIds', 'positions', 'delivered', 'inputSha256', 'smoke']:
        assert data[key] == old[key], (seq, condition, arm, key)
    run, prior = data['runs'], old['runs']
    assert set(run) == set(prior) | {'packetGaussianTags'}
    for key in prior:
        if key not in BYTE_KEYS + ['runtimeSeconds']:
            assert run[key] == prior[key], (seq, condition, arm, 'trajectory/diagnostic parity', key)
    records = np.asarray(run['packetGaussianRecords'], float).reshape(-1, 19)
    tags = np.asarray(run['packetGaussianTags'], float).reshape(-1, 5)
    assert np.array_equal(tags[:, :4], records[:, :4])
    assert np.all(tags[:, 4] == np.floor(tags[:, 4])) and np.all((tags[:, 4] >= 0) & (tags[:, 4] <= 65535))
    full = np.any(records[:, 4:] != 0, axis=1)
    tag_values = tags[:, 4].astype(np.uint16)
    assert np.array_equal((tag_values & 32768) > 0, full)
    assert np.all(tag_values[full] == 32768)
    count = np.zeros((2, len(data['time'])), int)
    retained = np.zeros_like(count)
    t, n = records[:, 0].astype(int)-1, records[:, 1].astype(int)-1
    np.add.at(count, (n, t), 1)
    np.add.at(retained, (n[full], t[full]), 1)
    sizes = 32 + 234*count + 120*retained
    assert np.array_equal(prior['packetBytes'], 32+352*count)
    assert np.array_equal(run['packetBytes'], sizes)
    assert np.array_equal(run['rawPayloadBytes'], sizes.sum(0))
    delivered = (sizes*np.asarray(data['delivered']).sum(0)).sum(0)
    wire = (np.ceil(sizes/16384).sum(0)*16384+256).astype(int)
    assert np.array_equal(run['deliveredRawBytes'], delivered)
    assert np.array_equal(run['wireBytes'], wire) and run['totalWireBytes'] == int(wire.sum())
    metrics, _ = score_run(data, run)
    reference = next(r for r in reference_summary['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == arm)
    for key in ['ospa', 'gospa', 'loc2', 'miss2', 'false2', 'countError']:
        assert np.isclose(metrics[key], reference[key], atol=1e-10, rtol=0), key
    return dict(sequence=name, condition=condition, arm=arm, frames=len(data['time']),
                node_frames=2*len(data['time']), objects=len(records), nonzero_vectors=int(full.sum()),
                **metrics, original_raw_bytes=sum(prior['rawPayloadBytes']),
                original_delivered_raw_bytes=sum(prior['deliveredRawBytes']), original_wire_bytes=sum(prior['wireBytes']),
                input_sha256={str(path.relative_to(ROOT)): sha(path), str(original.relative_to(ROOT)): sha(original)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    source = source_check()
    cohort = 'development_check' if args.preflight else args.cohort
    ref = json.loads((PARENT / f'summary_{args.cohort}.json').read_text())
    if args.preflight:
        units, arms = [dict(sequence=0, index=0)], ALL_ARMS
    else:
        frozen = json.loads((OUT / f'CODEC_FREEZE_{args.cohort}.json').read_text())
        assert frozen['reference_summary_sha256'] == sha(PARENT / f'summary_{args.cohort}.json')
        for name, expected in frozen['source_and_evidence_sha256'].items():
            assert sha(ROOT / name) == expected, name
        units, arms = frozen['units'], [PRIMARY]
    rows = []
    for unit in units:
        if not args.preflight:
            while True:
                runtime = OUT / f'runtime_{cohort}.json'
                complete = json.loads(runtime.read_text()) if runtime.exists() else []
                entry = next((r for r in complete if r['sequence'] == unit['sequence']), None)
                if entry is not None:
                    assert entry['returncode'] == 0 and entry['completion_line'] and entry['files'] == 2
                    break
                assert args.watch
                time.sleep(10)
        for condition in ['reliable', 'intermittent']:
            for arm in arms:
                rows.append(audit_one(cohort, unit['sequence'], condition, arm, source, ref))
        print('ZERO CODEC TRAJECTORY AND PACKETS EXACT', cohort, unit['sequence'], flush=True)
    result = dict(protocol='coherent-gaussian-zero-codec-v1', cohort=cohort, passed=True, sequences=len(units),
                  node_frames=sum(r['node_frames'] for r in rows), rows=rows, source_files=len(source),
                  analyzer_sha256=sha(Path(__file__)), source_manifest_sha256=sha(OUT / 'source_sha256.json'),
                  reference_summary_sha256=sha(PARENT / f'summary_{args.cohort}.json'),
                  inputs={p: value for row in rows for p, value in row['input_sha256'].items()},
                  gaussian_coefficients_bit_exact_asserted_in_native_codec=True,
                  signed_zero_note='Native typecast-to-uint64 assertions check all input/output bits. JSON comparisons check numerical equality and native tags; JSON alone does not preserve original signed-zero bits.',
                  full_trajectory_metrics_local_and_fusion_diagnostics_exact=True)
    output = 'preflight_audit.json' if args.preflight else f'summary_{cohort}.json'
    if args.preflight:
        assert result['node_frames'] == 2352
    (OUT / output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('ZERO CODEC COMPLETE EXACT AUDIT', cohort, result['node_frames'], 'node-frames.', flush=True)


if __name__ == '__main__':
    main()
