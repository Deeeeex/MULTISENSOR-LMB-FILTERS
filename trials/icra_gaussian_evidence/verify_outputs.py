"""Verify immutable sources, complete native exits and all reported input hashes."""
from pathlib import Path
import argparse
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
LOG = ROOT / 'RUN/ICRA_GAUSSIAN_EVIDENCE'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(mapping):
    for name, expected in mapping.items():
        assert sha(ROOT / name) == expected, name
    return len(mapping)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    source = json.loads((OUT / 'source_sha256.json').read_text())
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    summary = json.loads((OUT / f'summary_{cohort}.json').read_text())
    runtime = json.loads((OUT / f'runtime_{cohort}.json').read_text())
    source_count = verify(source)
    registration_count = verify(frozen['source_and_evidence_sha256'])
    verify(preflight['input_sha256'])
    result_count = verify(summary['inputs'])
    assert preflight['passed'] and preflight['independently_scored_node_frames'] == 2940
    assert preflight['exact_as_node_frames'] == 588
    assert preflight['auditor_sha256'] == summary['analyzer_sha256'] == sha(OUT / 'analyze_gaussian.py')
    assert summary['round_freeze_sha256'] == sha(OUT / 'ROUND_FREEZE.json')
    units = [u for u in frozen['units'] if u['cohort'] == cohort]
    assert len(runtime) == len(units) == summary['sequences']
    assert sorted(r['sequence'] for r in runtime) == sorted(u['sequence'] for u in units)
    for r in runtime:
        assert r['returncode'] == 0 and r['completion_line'] and r['files'] == 2*len(frozen['arms_by_cohort'][cohort])
        text = (LOG / f"{cohort}_{r['sequence']:04d}.log").read_text()
        assert f"COMPLETED GAUSSIAN EVIDENCE indices {r['index']}--{r['index']} cohort={cohort}" in text
    assert 'COMPLETED GAUSSIAN EVIDENCE indices 0--0 cohort=development_check' in (LOG / 'preflight.log').read_text()
    assert 'GAUSSIAN EVIDENCE PREFLIGHT AUDIT PASSED' in (LOG / 'preflight_audit_protocol_aligned.log').read_text()
    assert 'ALL GAUSSIAN EVIDENCE STAGE RUNS COMPLETE' in (LOG / f'{cohort}_driver.log').read_text()
    assert 'ALL GAUSSIAN EVIDENCE OUTPUTS AUDITED' in (LOG / f'{cohort}_audit.log').read_text()
    primary = [r for r in summary['paired'] if r['candidate'] == frozen['primary']]
    first = [r for r in primary if r['reference'] in ['marked_lineage', 'marked_er']]
    additional = [r for r in primary if r['reference'] in ['marked_conservative', 'marked_ceiling_score', 'marked_asymmetric']]
    assert len(first) == 4
    if cohort == 'development':
        assert len(additional) == 6
        gate = all(r['ospa']['mean'] < 0 for r in first) and all(r['ospa']['mean'] <= 0 for r in additional)
        assert gate == summary['continuation_gate_passed']
        if not gate:
            assert not list((OUT / 'results_seen_transfer').glob('*.json.gz'))
    diagnostics = summary['diagnostics']
    counts = {key: sum(r[key] for r in diagnostics) for key in ['local_update_records', 'joined_source_records',
              'local_gaussian_records', 'packet_gaussian_records', 'gaussian_source_joins', 'corrected_labels',
              'curvature_rejected_sources', 'aggregate_fallbacks']}
    report_files = [OUT / f'RESULTS_{cohort}_CN.md', OUT / f'ablation_comparisons_{cohort}.json',
                    OUT / 'DERIVATION_CN.md', OUT / 'build_report.py', OUT / 'verify_outputs.py']
    result = dict(passed=True, cohort=cohort, source_files=source_count, registration_files=registration_count,
                  result_files_with_verified_hashes=result_count, native_matlab_stage_exits=[r['returncode'] for r in runtime],
                  native_preflight_exit_observed_by_agent=0, native_preflight_exec_session=28142,
                  native_final_unit_exit_observed_by_agent=0, native_final_unit_exec_session=26360,
                  native_preflight_auditor_exit_observed_by_agent=0, native_preflight_auditor_exec_session=94089,
                  preflight_node_frames=2940, exact_as_node_frames=588,
                  new_node_frames=summary['new_node_frames'], rescored_baseline_node_frames=summary['rescored_baseline_node_frames'],
                  counts=counts, primary=summary['primary'], continuation_gate_passed=summary['continuation_gate_passed'],
                  summary_sha256=sha(OUT / f'summary_{cohort}.json'), round_freeze_sha256=sha(OUT / 'ROUND_FREEZE.json'),
                  report_sha256={str(p.relative_to(ROOT)): sha(p) for p in report_files},
                  scope='Independent NumPy reconstruction within the same workflow; all real trajectories already seen. Native Gaussian 352 B vs scalar 232 B. No equal-byte, independent-test, or third-party-reproduction claim.')
    (OUT / f'CURRENT_QA_{cohort}.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('GAUSSIAN EVIDENCE PROVENANCE VERIFIED:', cohort, source_count, 'source files;', result_count,
          'hashed result files; native exits all zero; continuation', summary['continuation_gate_passed'])


if __name__ == '__main__':
    main()
