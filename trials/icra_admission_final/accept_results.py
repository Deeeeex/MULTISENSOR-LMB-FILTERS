"""Verify every completed native stage and preserve the result-file manifest."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIALS = OUT.parent
STAGES = [
    ('icra_admission_revision', 'admission_v1_preflight'),
    ('icra_admission_revision', 'admission_v1_development_rest'),
    ('icra_projected_admission', 'projected_v1_preflight'),
    ('icra_projected_admission', 'projected_v1_development_rest'),
    ('icra_compatible_admission', 'compatible_v1_preflight'),
    ('icra_compatible_admission', 'compatible_v1_development_rest'),
    ('icra_projected_admission', 'final_v2v_evaluation_v2'),
    ('icra_projected_admission', 'final_v2x_evaluation'),
]


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    result_files = {}; checked = {}; rows = []

    def verify(path, expected=None):
        key = str(path.relative_to(ROOT))
        if key not in checked:
            checked[key] = sha(path)
        if expected is not None:
            assert checked[key] == expected, key
        return checked[key]

    original_path = TRIALS / 'icra_gaussian_evidence/source_sha256.json'
    original_sources = json.loads(original_path.read_text())
    verify(original_path)
    for name, expected in original_sources.items():
        verify(ROOT / name, expected)
    for folder, stage in STAGES:
        base = TRIALS / folder
        cfgpath = base / 'stages' / (stage + '.json')
        runtimepath = base / ('runtime_' + stage + '.json')
        auditpath = base / ('audit_' + stage + '.json')
        report = json.loads(auditpath.read_text()); assert report['passed']
        verify(auditpath); verify(cfgpath, report['config_sha256']); verify(runtimepath, report['runtime_sha256'])
        cfg = json.loads(cfgpath.read_text()); native = json.loads(runtimepath.read_text())
        assert len(cfg['units']) == len(native) == report['sequences']
        assert all(r['returncode'] == 0 and r['completion_line'] and r['files'] == 2 * len(cfg['arms']) for r in native)
        expected_files = len(native) * len(cfg['arms']) * 2
        actual_files = sorted((base / 'results' / stage).glob('*.json.gz'))
        assert len(actual_files) == len(report['rows']) == expected_files
        for source in [cfg['source_sha256'], report['auditor_sha256'], report['inputs']]:
            for key, expected in source.items():
                verify(ROOT / key, expected)
        for path in actual_files:
            key = str(path.relative_to(ROOT))
            assert key not in result_files and key in report['inputs']
            result_files[key] = dict(bytes=path.stat().st_size, sha256=verify(path, report['inputs'][key]))
        for unit in cfg['units']:
            log = ROOT / 'RUN/ICRA_REVIEWER_REVISION' / stage / (unit['sequence'] + '.log')
            assert 'COMPLETED REVIEW ' + stage + ' ' + unit['sequence'] in log.read_text()
            verify(log)
        rows.append(dict(folder=folder, stage=stage, units=len(native), result_files=expected_files,
                         audited_node_frames=report['audited_node_frames'], exact_legacy_parity_entries=len(report['parity'])))
    assert len(result_files) == 458
    assert sum(r['audited_node_frames'] for r in rows) == 187444
    failed = TRIALS / 'icra_projected_admission/runtime_final_v2v_evaluation.json'
    failures = json.loads(failed.read_text())
    assert len(failures) == 34 and all(r['returncode'] != 0 and r['files'] == 0 for r in failures)
    verify(failed)
    for folder, name in [
            ('icra_projected_admission', 'EVALUATION_REGISTRATION_REPAIR.json'),
            ('icra_projected_admission', 'AUDITOR_REPAIR.json'),
            ('icra_compatible_admission', 'DATA_EXPOSURE.json'),
            ('icra_compatible_admission', 'FINAL_SELECTION.json'),
            ('icra_scan_admission', 'SCAN_DIAGNOSTIC.json'),
            ('icra_admission_final', 'EVALUATION_ANALYSIS.json'),
            ('icra_admission_final', 'EXTERNAL_ANALYSIS.json'),
            ('icra_admission_final', 'DETECTION_SUPPORT_DIAGNOSTIC.json'),
            ('icra_v2x_transfer', 'NEW_INPUT_AUDIT.json')]:
        verify(TRIALS / folder / name)
    manifest_path = OUT / 'RESULT_FILES_MANIFEST.json'
    report_path = OUT / 'REPLAY_ACCEPTANCE.json'
    core = dict(passed=True, all_native_unit_exit_codes_zero=True,
        all_expected_results_present_and_independently_audited=True,
        original_protected_sources_unchanged=len(original_sources), stages=rows,
        completed_native_result_files=len(result_files), native_result_bytes=sum(r['bytes'] for r in result_files.values()),
        independently_audited_node_frames=sum(r['audited_node_frames'] for r in rows),
        exact_legacy_parity_entries=sum(r['exact_legacy_parity_entries'] for r in rows),
        preserved_failed_registration_units=34, preserved_failed_registration_output_files=0,
        evaluation_comparison_rows=854, complete_development_admission_candidates=9,
        fixed_strength_grid=[0, .05, .1, .125, .25, .5, 1],
        new_candidate_promoted=False,
        decision='The development-selected spatial projection is worse than original GCE under both radio conditions on all 43 V2V4Real segments and does not improve the external two-condition mean. Keep original GCE as the working baseline.',
        verification_sha256=sha(Path(__file__)), checked_files=checked)
    if args.verify_only:
        assert json.loads(manifest_path.read_text()) == result_files
        previous = json.loads(report_path.read_text())
        for key, expected in core.items():
            assert previous[key] == expected, key
        assert previous['result_manifest_sha256'] == sha(manifest_path)
    else:
        assert not manifest_path.exists() and not report_path.exists()
        manifest_path.write_text(json.dumps(result_files, indent=2, sort_keys=True) + '\n')
        core.update(completed_utc=datetime.now(timezone.utc).isoformat(), result_manifest_sha256=sha(manifest_path))
        report_path.write_text(json.dumps(core, indent=2, allow_nan=False) + '\n')
    print('NATIVE ASSESSMENT VERIFIED:', len(result_files), 'result files;', core['independently_audited_node_frames'], 'robot-frames;', core['original_protected_sources_unchanged'], 'unchanged sources.', flush=True)


if __name__ == '__main__':
    main()
