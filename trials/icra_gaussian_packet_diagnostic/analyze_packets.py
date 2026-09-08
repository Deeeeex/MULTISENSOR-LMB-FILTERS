"""Price three exact omission rules on immutable full development histories."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SOURCE = OUT.parent / 'icra_gaussian_evidence'
sys.path.insert(0, str(SOURCE))
from gaussian_audit import unpack, LOWER
sys.path.insert(0, str(OUT.parent / 'icra_marked_control'))
from analyze_control import read


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'summary_development.json'
    assert not target.exists()
    summary = json.loads((SOURCE / 'summary_development.json').read_text())
    qa = json.loads((SOURCE / 'CURRENT_QA_development.json').read_text())
    assert qa['passed'] and qa['summary_sha256'] == sha(SOURCE / 'summary_development.json')
    assert summary['sequences'] == 9 and summary['primary'] == 'marked_gaussian_evidence'
    inputs, rows = {}, []
    for sequence in range(9):
        for condition in ['reliable', 'intermittent']:
            path = SOURCE / 'results_development' / f'{sequence:04d}_{condition}_marked_gaussian_evidence.json.gz'
            name = str(path.relative_to(ROOT))
            assert sha(path) == summary['inputs'][name]
            inputs[name] = sha(path)
            data = read(path)
            run = data['runs']
            local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
            packet = np.asarray(run['packetGaussianRecords'], float).reshape(-1, 19)
            assert np.array_equal(local[:, :4], packet[:, :4])
            encoded = packet[:, 4:]
            post_j = np.linalg.inv(unpack(local[:, 22:32]))
            post_j = (post_j + post_j.swapaxes(-1, -2))/2
            delta_j = unpack(encoded[:, :10])
            tolerance = 1e-10*np.maximum(1., np.maximum(np.linalg.norm(post_j, 2, axis=(-1, -2)),
                                                       np.linalg.norm(post_j-delta_j, 2, axis=(-1, -2))))
            allowed = np.linalg.eigvalsh(delta_j)[:, 0] >= -tolerance
            scalar_lookup = {tuple(r[:4].astype(int)): r for r in np.asarray(run['localIncrementRecords'], float)}
            scalar = np.asarray([scalar_lookup[tuple(r[:4].astype(int))] for r in local])
            branch_gate = np.where(scalar[:, 6] >= 0, scalar[:, 8], scalar[:, 11])
            masks = {'exact_zero': np.any(encoded != 0, axis=1)}
            masks['curvature'] = masks['exact_zero'] & allowed
            masks['current_gate'] = masks['curvature'] & (branch_gate > 0)
            t, n = local[:, 0].astype(int)-1, local[:, 1].astype(int)-1
            count = np.zeros((2, len(data['time'])), int)
            np.add.at(count, (n, t), 1)
            native = 32+352*count
            assert np.array_equal(native, run['packetBytes'])
            assert np.array_equal(native.sum(0), run['rawPayloadBytes'])
            delivery = np.asarray(data['delivered']).sum(0)
            for rule, mask in {'native': np.ones(len(local), bool), **masks}.items():
                retained = np.zeros_like(count)
                np.add.at(retained, (n[mask], t[mask]), 1)
                sizes = native if rule == 'native' else 32+233*count+120*retained
                rows.append(dict(sequence=f'{sequence:04d}', condition=condition, rule=rule,
                                 objects=len(local), retained=int(mask.sum()),
                                 raw_bytes=int(sizes.sum()), delivered_raw_bytes=int((sizes*delivery).sum()),
                                 wire_bytes=int((np.ceil(sizes/16384)*16384).sum()+256*len(data['time']))))
            print('PACKET PROJECTION', f'{sequence:04d}', condition, len(local),
                  {k: int(v.sum()) for k, v in masks.items()}, flush=True)
    aggregates = []
    for condition in ['reliable', 'intermittent']:
        for rule in ['native', 'exact_zero', 'curvature', 'current_gate']:
            group = [r for r in rows if r['condition'] == condition and r['rule'] == rule]
            aggregates.append(dict(condition=condition, rule=rule,
                **{key: sum(r[key] for r in group) for key in ['objects', 'retained', 'raw_bytes', 'delivered_raw_bytes', 'wire_bytes']}))
    result = dict(protocol='exact-spatial-ratio-omission-diagnostic-v1', sequences=9, rows=rows, aggregate=aggregates,
                  inputs=inputs, source_summary_sha256=sha(SOURCE / 'summary_development.json'),
                  protocol_sha256=sha(OUT / 'PROTOCOL.md'), analyzer_sha256=sha(Path(__file__)),
                  native_packet_roundtrip_executed=False, tracking_rerun=False,
                  interpretation='Projected specified codec lengths only; actual implementation and exact full-state parity still required. No change to original M-GE native results.')
    target.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for r in aggregates:
        print(r, flush=True)
    print('PACKET PROJECTION COMPLETE: 18 immutable saved histories; no new tracking.', flush=True)


if __name__ == '__main__':
    main()
