"""Package complete study sources and verify the summary from a fresh directory."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FOLDERS = ['icra_admission_revision', 'icra_projected_admission',
           'icra_compatible_admission', 'icra_admission_final',
           'icra_scan_admission', 'icra_v2x_transfer']
STAGES = ['admission_v1_preflight', 'admission_v1_development_rest',
          'projected_v1_preflight', 'projected_v1_development_rest',
          'compatible_v1_preflight', 'compatible_v1_development_rest',
          'final_v2v_evaluation', 'final_v2v_evaluation_v2', 'final_v2x_evaluation']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert json.loads((HERE / 'SUMMARY_AUDIT.json').read_text())['passed']
    assert json.loads((HERE / 'REPLAY_ACCEPTANCE.json').read_text())['passed']
    files = []
    for name in FOLDERS:
        for path in (ROOT / 'trials' / name).rglob('*'):
            if path.is_file() and not {'results', 'output', '__pycache__', 'new_data', 'pose_inputs'}.intersection(path.parts):
                assert path.suffix not in ['.mat', '.bin', '.pcd', '.pyc']
                files.append(path)
    for stage in STAGES:
        files.extend((ROOT / 'RUN/ICRA_REVIEWER_REVISION' / stage).glob('*.log'))
    for relative in ['trials/icra_full_coverage/COVERAGE_ANALYSIS.json',
                     'trials/icra_full_coverage/DATA_INVENTORY.json',
                     'trials/icra_gaussian_evidence/source_sha256.json']:
        files.append(ROOT / relative)
    assert len(files) == len(set(files))
    manifest = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)}
    output = HERE / 'output'; output.mkdir(exist_ok=True)
    manifest_path = output / 'SOURCE_MANIFEST.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    archive = output / 'icra_admission_assessment.zip'
    top = Path('icra_admission_assessment')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
        for path in sorted(files):
            bundle.write(path, top / path.relative_to(ROOT))
        bundle.write(manifest_path, top / 'SOURCE_MANIFEST.json')
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None
        for key, expected in manifest.items():
            assert hashlib.sha256(bundle.read(str(top / key))).hexdigest() == expected, key
    work = Path(tempfile.mkdtemp(prefix='icra_admission_portable_', dir=ROOT / 'tmp'))
    with zipfile.ZipFile(archive) as bundle:
        bundle.extractall(work)
    command = [sys.executable, 'trials/icra_admission_final/verify_summary.py']
    run = subprocess.run(command, cwd=work / top, capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    assert 'PORTABLE SUMMARY VERIFIED' in run.stdout
    report = dict(passed=True, archive_sha256=sha(archive), manifest_sha256=sha(manifest_path),
                  files=len(manifest), archive_bytes=archive.stat().st_size,
                  fresh_directory=str(work / top), command=command, returncode=run.returncode,
                  stdout=run.stdout.strip(), source_rows=854, paired_comparisons=180,
                  scope='Fresh-directory summary recomputation and exact archive-member hashes; no native MATLAB rerun.')
    (output / 'PORTABLE_REBUILD.json').write_text(json.dumps(report, indent=2) + '\n')
    print('VERIFIED STUDY ARCHIVE', archive, 'files', len(manifest), 'bytes', archive.stat().st_size, flush=True)


if __name__ == '__main__':
    main()
