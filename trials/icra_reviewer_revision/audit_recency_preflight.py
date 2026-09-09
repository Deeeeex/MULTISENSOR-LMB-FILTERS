"""Exact unchanged Recency recursion on a released old development sequence."""
import json
from pathlib import Path
from audit_stage import OUT, ROOT, sha, read, baseline, np, loadmat, score_run, radio_draws, audit_packets


def main():
    stage = 'recency_identity_preflight'
    cfg = json.loads((OUT / 'stages' / f'{stage}.json').read_text())
    for path, expected in cfg['source_sha256'].items():
        assert sha(ROOT / path) == expected
    runtime = json.loads((OUT / f'runtime_{stage}.json').read_text()); assert len(runtime) == 1
    assert runtime[0]['returncode'] == 0 and runtime[0]['completion_line'] and runtime[0]['files'] == 2
    reports = []
    for condition in cfg['conditions']:
        path = OUT / 'results' / stage / f'0000_{condition}_marked_er.json.gz'
        data = read(path); run = data['runs']; old, row, _, old_path = baseline('development', '0000', condition, 'marked_er')
        previous = next(r for r in old['runs'] if r['arm'] == 'marked_er')
        for key in ['time','positions','truth','truthIds','delivered']:
            assert data[key] == old[key]
        keys = ['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount',
                'rawPayloadBytes','deliveredRawBytes','wireBytes','controlBytes','attemptedMessages',
                'deliveredMessages','maximumBernoulliCount']
        for key in keys:
            assert run[key] == previous[key], (condition, key)
        audited, _ = score_run(data, run); audit_packets(run, np.asarray(data['delivered']), len(data['time']))
        reports.append(dict(condition=condition, fields_exact=keys, new_sha256=sha(path), reference_sha256=sha(old_path), ospa=audited['ospa']))
    destination = OUT / 'RECENCY_PREFLIGHT_AUDIT.json'; assert not destination.exists()
    destination.write_text(json.dumps(dict(passed=True, rows=reports, auditor_sha256=sha(Path(__file__))),indent=2)+'\n')
    print('RECENCY FULL RECURSION PARITY PASSED', flush=True)


if __name__ == '__main__':
    main()
