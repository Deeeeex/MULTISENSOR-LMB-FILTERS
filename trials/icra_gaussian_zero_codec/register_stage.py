"""Fix a whole codec-parity stage after its unchanged reference is complete."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import json
from analyze_codec import sha, source_check

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PARENT = OUT.parent / 'icra_gaussian_evidence'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    target = OUT / f'CODEC_FREEZE_{cohort}.json'
    assert not target.exists()
    source_check()
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    assert preflight['passed'] and preflight['node_frames'] == 2352
    assert preflight['analyzer_sha256'] == sha(OUT / 'analyze_codec.py')
    for name, expected in preflight['inputs'].items():
        assert sha(ROOT / name) == expected, name
    summary = json.loads((PARENT / f'summary_{cohort}.json').read_text())
    qa = json.loads((PARENT / f'CURRENT_QA_{cohort}.json').read_text())
    assert summary['sequences'] == (9 if cohort == 'development' else 25)
    assert qa['passed'] and qa['summary_sha256'] == sha(PARENT / f'summary_{cohort}.json')
    assert all(x == 0 for x in qa['native_matlab_stage_exits'])
    for name, expected in summary['inputs'].items():
        assert sha(ROOT / name) == expected, name
    parent = json.loads((PARENT / 'ROUND_FREEZE.json').read_text())
    files = [OUT / n for n in ['PROTOCOL.md', 'source_sha256.json', 'preflight_audit.json', 'analyze_codec.py', 'register_stage.py', 'run_sequences.py']]
    files += [PARENT / n for n in [f'summary_{cohort}.json', f'CURRENT_QA_{cohort}.json', 'ROUND_FREEZE.json']]
    assert not list((OUT / f'results_{cohort}').glob('*.json.gz'))
    result = dict(protocol='bit-preserving-gaussian-zero-codec-stage-v1', cohort=cohort,
                  timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  primary='marked_gaussian_evidence', max_workers=2,
                  units=[u for u in parent['units'] if u['cohort'] == cohort],
                  reference_summary_sha256=sha(PARENT / f'summary_{cohort}.json'),
                  source_and_evidence_sha256={str(p.relative_to(ROOT)): sha(p) for p in files},
                  unit_exit_observed=0, unit_exec_session=98989,
                  preflight_exit_observed=0, preflight_exec_session=47935,
                  numerical_parameters_changed=False, complete_exact_trajectory_parity_required=True)
    target.write_text(json.dumps(result, indent=2)+'\n')
    print('WHOLE ZERO CODEC PARITY STAGE FIXED:', cohort, len(result['units']), 'sequences.')


if __name__ == '__main__':
    main()
