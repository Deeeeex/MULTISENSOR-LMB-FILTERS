"""Final integrity check of fixed inputs, native exits and retained result files."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def read(name):
    return json.loads((OUT / name).read_text())


def main():
    destination = OUT / 'REPLAY_ACCEPTANCE.json'
    manifest_path = OUT / 'RESULT_FILES_MANIFEST.json'
    assert not destination.exists() and not manifest_path.exists()
    prior = read('PRE_REVISION_FREEZE.json')
    for name, digest in prior['prior_sources'].items():
        assert sha(ROOT / name) == digest, name
    manuscript_changes = []
    for name, digest in prior['frozen_artifacts'].items():
        if name == 'papers/icra2027/main.tex':
            original = subprocess.check_output(['git', 'show', prior['base_commit'] + ':' + name], cwd=ROOT)
            assert hashlib.sha256(original).hexdigest() == digest
            manuscript_changes.append(dict(path=name, original_sha256=digest, current_sha256=sha(ROOT / name),
                                           purpose='Reviewed manuscript identity; manuscript revision is explicitly authorized.'))
        else:
            assert sha(ROOT / name) == digest, name
    new = read('NEW_DATA_FREEZE.json')
    for category in ['protected_sha256', 'raw_file_sha256']:
        for name, digest in new[category].items():
            assert sha(ROOT / name) == digest, name
    detections = read('NEW_DETECTION_MANIFEST.json')
    assert detections['passed'] and detections['truth_used_for_detections'] is False
    assert detections['freeze_sha256'] == sha(OUT / 'NEW_DATA_FREEZE.json')
    for row in detections['sequences']:
        assert sha(ROOT / row['path']) == row['sha256']
    inputs = read('NEW_INPUT_MANIFEST.json')
    assert inputs['passed'] and not inputs['calibration_refitted'] and not inputs['positive_prior_refitted']
    for row in inputs['sequences']:
        assert sha(ROOT / row['data_path']) == row['input_sha256']
        assert sha(ROOT / row['pose_path']) == row['pose_sha256']
    for name, digest in inputs['source_sha256'].items():
        assert sha(ROOT / name) == digest, name
    cancellation = read('CANCELLED_STAGES.json')
    files, stages = {}, []
    for path in sorted((OUT / 'stages').glob('*.json')):
        stage = path.stem
        cfg = json.loads(path.read_text())
        if stage in cancellation:
            assert sha(path) == cancellation[stage]['config_sha256']
            assert not (OUT / ('runtime_' + stage + '.json')).exists()
            assert not (OUT / 'results' / stage).exists()
            continue
        report_name = 'RECENCY_PREFLIGHT_AUDIT.json' if stage == 'recency_identity_preflight' else 'audit_' + stage + '.json'
        report = read(report_name); assert report['passed']
        for name, digest in cfg['source_sha256'].items():
            assert sha(ROOT / name) == digest, (stage, name)
        runtime_path = OUT / ('runtime_' + stage + '.json')
        runtime = json.loads(runtime_path.read_text())
        assert len(runtime) == len(cfg['units'])
        assert all(row['returncode'] == 0 and row['completion_line'] and row['files'] == 2 * len(cfg['arms']) for row in runtime)
        if stage != 'recency_identity_preflight':
            assert report['config_sha256'] == sha(path)
            assert report['runtime_sha256'] == sha(runtime_path)
            for relative, digest in report['inputs'].items():
                candidate = ROOT / relative
                if relative not in files:
                    actual = sha(candidate)
                    assert actual == digest, relative
                    if candidate.suffixes[-2:] == ['.json', '.gz'] and candidate.is_relative_to(OUT / 'results'):
                        files[relative] = dict(sha256=actual, bytes=candidate.stat().st_size, stage=candidate.parent.name)
                else:
                    assert files[relative]['sha256'] == digest
        else:
            assert report['auditor_sha256'] == sha(OUT / 'audit_recency_preflight.py')
            for row in report['rows']:
                result = OUT / 'results' / stage / f"0000_{row['condition']}_marked_er.json.gz"
                assert sha(result) == row['new_sha256']
        for unit in cfg['units']:
            for condition in cfg['conditions']:
                for arm in cfg['arms']:
                    result = OUT / 'results' / stage / f"{unit['sequence']}_{condition}_{arm}.json.gz"
                    relative = str(result.relative_to(ROOT))
                    assert result.is_file()
                    if relative not in files:
                        files[relative] = dict(sha256=sha(result), bytes=result.stat().st_size, stage=stage)
        stages.append(dict(stage=stage, sequences=len(cfg['units']), arms=len(cfg['arms']),
                           files=2 * len(cfg['arms']) * len(cfg['units']), config_sha256=sha(path),
                           runtime_sha256=sha(runtime_path), audit_sha256=sha(OUT / report_name)))
    actual_paths = {str(path.relative_to(ROOT)) for path in (OUT / 'results').rglob('*.json.gz')}
    assert actual_paths == set(files), (actual_paths - set(files), set(files) - actual_paths)
    assert sum(stage['files'] for stage in stages) == len(files) == 750
    manifest_path.write_text(json.dumps(files, indent=2, sort_keys=True) + '\n')
    result = dict(passed=True, checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        base_commit=prior['base_commit'], prior_algorithm_sources_unchanged=len(prior['prior_sources']),
        detector_and_calibration_unchanged=True, raw_new_sensor_files_unchanged=len(new['raw_file_sha256']),
        registered_stages=stages, cancelled_without_execution=cancellation, manuscript_revision=manuscript_changes,
        retained_native_result_files=len(files), retained_native_result_bytes=sum(row['bytes'] for row in files.values()),
        result_manifest_sha256=sha(manifest_path), all_native_unit_exit_codes_zero=True,
        all_expected_results_present_and_independently_audited=True,
        paper_author_approval_or_submission=False, checker_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('REPLAY ACCEPTANCE PASSED:', len(files), 'unchanged retained result files;', len(stages), 'complete native stages')


if __name__ == '__main__':
    main()
