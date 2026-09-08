"""Close completed selective stages with current source and output checks."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_SELECTIVE_INNOVATION'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    source = json.loads((OUT / 'source_sha256.json').read_text())
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    for name, expected in {**source, **frozen['source_and_evidence_sha256']}.items():
        assert sha(ROOT / name) == expected, name
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    assert preflight['passed'] and preflight['exact_cr_node_frames'] == 588
    assert preflight['auditor_sha256'] == sha(OUT / 'analyze_selective.py')
    for name, expected in preflight['input_sha256'].items():
        assert sha(ROOT / name) == expected, name
    assert 'SELECTIVE CHECK PASSED' in (LOG / 'unit.log').read_text()
    development = json.loads((OUT / 'summary_development.json').read_text())
    cohorts = ['development']
    if (OUT / 'summary_seen_transfer.json').exists():
        assert development['continuation_gate_passed']
        cohorts.append('seen_transfer')
    if not development['continuation_gate_passed']:
        assert not list((OUT / 'results_seen_transfer').glob('*.json.gz'))
    stages = []
    for cohort in cohorts:
        report = json.loads((OUT / f'summary_{cohort}.json').read_text())
        assert report['round_freeze_sha256'] == sha(OUT / 'ROUND_FREEZE.json')
        assert report['analyzer_sha256'] == sha(OUT / 'analyze_selective.py')
        assert report['primary'] == frozen['primary']
        ledger = json.loads((OUT / f'runtime_{cohort}.json').read_text())
        units = [u for u in frozen['units'] if u['cohort'] == cohort]
        assert len(ledger) == len(units) == report['sequences']
        assert sorted(r['sequence'] for r in ledger) == sorted(u['sequence'] for u in units)
        assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 8 for r in ledger)
        outputs = list((OUT / f'results_{cohort}').glob('*.json.gz'))
        assert len(outputs) == 8*len(units)
        for name, expected in report['inputs'].items():
            assert sha(ROOT / name) == expected, name
        text = (OUT / f'RESULTS_{cohort}_CN.md').read_text()
        assert all(str(u['sequence']).zfill(4) in text for u in units)
        assert len(json.loads((OUT / f'ablation_comparisons_{cohort}.json').read_text())) == 6
        stages.append(dict(cohort=cohort, sequences=len(units), files=len(outputs),
                           successful_native_matlab_exits=len(ledger),
                           new_node_frames=report['new_node_frames'],
                           rescored_reference_node_frames=report['rescored_baseline_node_frames'],
                           verified_local_update_records=sum(r['local_update_records'] for r in report['diagnostics']),
                           joined_received_source_records=sum(r['joined_source_records'] for r in report['diagnostics']),
                           continuation_gate_passed=report['continuation_gate_passed'],
                           all_four_primary_intervals_below_zero=report['all_four_primary_intervals_below_zero'],
                           summary_sha256=sha(OUT / f'summary_{cohort}.json')))
    files = [p for p in OUT.iterdir() if p.is_file() and p.name != 'CURRENT_QA.json']
    files += list(LOG.glob('*.log'))
    current = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
                   preceding_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                   source_files_verified=len(source), registration_files_verified=len(frozen['source_and_evidence_sha256']),
                   preflight_node_frames=2940, exact_cr_preflight_node_frames=588,
                   preflight_exit_note='The preflight shell used a tee pipeline without pipefail; its shell exit is not claimed as a separately captured MATLAB exit. Complete outputs, completion marker and parity were checked. All full-stage MATLAB exits were captured directly by subprocess.Popen.',
                   manuscript_updated=False, independent_validation_claimed=False, stages=stages,
                   artifact_sha256={str(p.relative_to(ROOT)): sha(p) for p in files})
    (OUT / 'CURRENT_QA.json').write_text(json.dumps(current, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(source_files=current['source_files_verified'], stages=stages), indent=2))


if __name__ == '__main__':
    main()
