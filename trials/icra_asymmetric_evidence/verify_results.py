"""Verify complete stages, original failed-preflight source, and current artifacts."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_ASYMMETRIC_EVIDENCE'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    initial = json.loads((OUT / 'initial_preflight/source_sha256.json').read_text())
    for name, expected in initial.items():
        path = ROOT / name
        if path.parent == OUT and path.name in ['runAsymmetricReplay.m', 'make_core.py', 'freeze_source.py']:
            path = OUT / 'initial_preflight' / path.name
        assert sha(path) == expected, ('initial source', name)
    source = json.loads((OUT / 'source_sha256.json').read_text())
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    for name, expected in {**source, **frozen['source_and_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    assert preflight['passed'] and preflight['exact_si_node_frames'] == 588
    assert preflight['auditor_sha256'] == sha(OUT / 'analyze_asymmetric.py')
    for name, expected in preflight['input_sha256'].items():
        assert sha(ROOT / name) == expected, name
    assert 'ASYMMETRIC CHECK PASSED' in (LOG / 'unit.log').read_text()
    failed_log = (LOG / 'preflight_initial_shape_failure.log').read_text()
    assert 'assert(isequal(opportunity,predictedPd>0))' in failed_log and 'DONE ASYMMETRIC' not in failed_log
    assert 'COMPLETED ASYMMETRIC indices 0--0 cohort=development_check' in (LOG / 'preflight.log').read_text()
    development = json.loads((OUT / 'summary_development.json').read_text())
    cohorts = ['development']
    if (OUT / 'summary_seen_transfer.json').exists():
        assert development['continuation_gate_passed']
        cohorts.append('seen_transfer')
    if not development['continuation_gate_passed']:
        assert not list((OUT / 'results_seen_transfer').glob('*.json.gz'))
    stages = []
    for cohort in cohorts:
        d = json.loads((OUT / f'summary_{cohort}.json').read_text())
        assert d['round_freeze_sha256'] == sha(OUT / 'ROUND_FREEZE.json')
        assert d['analyzer_sha256'] == sha(OUT / 'analyze_asymmetric.py') and d['primary'] == frozen['primary']
        ledger = json.loads((OUT / f'runtime_{cohort}.json').read_text())
        units = [u for u in frozen['units'] if u['cohort'] == cohort]
        width = 2*len(frozen['arms_by_cohort'][cohort])
        assert len(ledger) == len(units) == d['sequences']
        assert sorted(r['sequence'] for r in ledger) == sorted(u['sequence'] for u in units)
        assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == width for r in ledger)
        outputs = list((OUT / f'results_{cohort}').glob('*.json.gz'))
        assert len(outputs) == width*len(units)
        for name, expected in d['inputs'].items():
            assert sha(ROOT / name) == expected, name
        text = (OUT / f'RESULTS_{cohort}_CN.md').read_text()
        assert all(str(u['sequence']).zfill(4) in text for u in units)
        assert len(json.loads((OUT / f'ablation_comparisons_{cohort}.json').read_text())) == 8
        stages.append(dict(cohort=cohort, sequences=len(units), files=len(outputs),
                           successful_native_matlab_exits=len(ledger),
                           new_node_frames=d['new_node_frames'], rescored_reference_node_frames=d['rescored_baseline_node_frames'],
                           verified_local_update_records=sum(r['local_update_records'] for r in d['diagnostics']),
                           joined_received_source_records=sum(r['joined_source_records'] for r in d['diagnostics']),
                           continuation_gate_passed=d['continuation_gate_passed'],
                           all_four_primary_intervals_below_zero=d['all_four_primary_intervals_below_zero'],
                           summary_sha256=sha(OUT / f'summary_{cohort}.json')))
    files = [p for p in OUT.iterdir() if p.is_file() and p.name != 'CURRENT_QA.json'] + list(LOG.glob('*.log'))
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
                  preceding_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  original_failed_preflight_source_files_verified=len(initial),
                  source_files_verified=len(source), registration_files_verified=len(frozen['source_and_evidence_sha256']),
                  preflight_node_frames=2352, exact_si_preflight_node_frames=588,
                  preflight_native_exits_observed=dict(initial_shape_failure=1, repaired_preflight=0, unit_check=0),
                  manuscript_updated=False, independent_validation_claimed=False, stages=stages,
                  artifact_sha256={str(p.relative_to(ROOT)): sha(p) for p in files})
    (OUT / 'CURRENT_QA.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(source_files=len(source), stages=stages), indent=2))


if __name__ == '__main__':
    main()
