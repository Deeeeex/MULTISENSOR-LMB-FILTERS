"""Rescore unchanged legacy arms used by recursive and sensitivity contrasts."""
import json
from pathlib import Path
from audit_stage import OUT, ROOT, sha, read, score_run, baseline, np

ARMS = ['marked_lineage','marked_er','marked_asymmetric','marked_gaussian_evidence_no_curvature','marked_gaussian_evidence']


def main():
    target = OUT / 'LEGACY_RESCORING.json'; assert not target.exists()
    protected = json.loads((OUT / 'PRE_REVISION_FREEZE.json').read_text())['prior_sources']
    for path, expected in protected.items():
        assert sha(ROOT / path) == expected, path
    rows = []; sources = {}; summaries = {}
    for cohort in ['development', 'seen_transfer']:
        source = OUT.parent / 'icra_gaussian_evidence' / f'summary_{cohort}.json'
        summary = json.loads(source.read_text()); summaries[str(source.relative_to(ROOT))] = sha(source)
        expected = {(r['sequence'], r['condition'], r['arm']):r for r in summary['runs']}
        names = sorted({r['sequence'] for r in summary['runs']})
        for name in names:
            for condition in ['reliable','intermittent']:
                for arm in ARMS:
                    if arm in ['marked_lineage','marked_er']:
                        data, row, _, path = baseline(cohort, name, condition, arm)
                        run = data['runs'] if isinstance(data['runs'], dict) else next(r for r in data['runs'] if r['arm'] == arm)
                    else:
                        suffix = f'/{name}_{condition}_{arm}.json.gz'
                        matches = [key for key in summary['inputs'] if key.endswith(suffix)]
                        assert len(matches) == 1, (cohort, name, arm, matches)
                        path = ROOT / matches[0]
                        data = read(path); run = data['runs']; row, _ = score_run(data, run)
                    key = str(path.relative_to(ROOT)); assert sha(path) == summary['inputs'][key], key
                    for metric in ['ospa','gospa','loc2','miss2','false2','countError','raw_bytes','delivered_raw_bytes','wire_bytes']:
                        assert np.isclose(row[metric], expected[name,condition,arm][metric], rtol=0, atol=1e-10), (cohort,name,condition,arm,metric)
                    rows.append({**row, 'cohort':cohort, 'sequence':name, 'condition':condition,
                                 'arm':arm, 'frames':len(data['time'])})
                    sources[key] = sha(path)
            print('LEGACY INPUTS RESCORED', cohort, name, flush=True)
    result = dict(passed=True, rows=rows, inputs=sources, summaries=summaries,
                  independently_rescored_node_frames=sum(2 * r['frames'] for r in rows),
                  auditor_sha256=sha(Path(__file__)), helper_sha256=sha(OUT.parent / 'icra_marked_control/analyze_control.py'),
                  note='Only runtime is excluded from numerical equality. Original files and original native packet costs retained.')
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ALL LEGACY REFERENCE TRAJECTORIES RESCORED',result['independently_rescored_node_frames'],'node-frames',flush=True)


if __name__ == '__main__':
    main()
