"""Archive study sources and compact inputs, then verify summaries after extraction."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
TEST=OUT.parent/'icra_association_test'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    accepted=json.loads((OUT/'REPLAY_ACCEPTANCE.json').read_text());assert accepted['passed']
    files=[]
    for base in [OUT,TEST]:
        for path in base.rglob('*'):
            if not path.is_file() or {'results','output','__pycache__'}.intersection(path.relative_to(base).parts):
                continue
            assert path.suffix not in ['.bin','.pcd','.pyc']
            files.append(path)
    files += [p for p in (ROOT/'RUN/ICRA_TEMPORAL_ASSOCIATION').rglob('*') if p.is_file() and p.suffix in ['.log','.txt','.json']]
    files += [OUT.parent/'icra_gaussian_evidence/source_sha256.json']
    assert len(files)==len(set(files))
    manifest={str(p.relative_to(ROOT)):sha(p) for p in sorted(files)}
    output=OUT/'output';output.mkdir(exist_ok=True)
    manifest_path=output/'SOURCE_MANIFEST.json'
    archive=output/'icra_temporal_association_study.zip'
    report_path=output/'PORTABLE_REBUILD.json'
    assert not archive.exists() and not report_path.exists()
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    top=Path('icra_temporal_association_study')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as bundle:
        for path in sorted(files):bundle.write(path,top/path.relative_to(ROOT))
        bundle.write(manifest_path,top/'SOURCE_MANIFEST.json')
    with zipfile.ZipFile(archive) as bundle:
        assert bundle.testzip() is None
        for name,expected in manifest.items():
            assert hashlib.sha256(bundle.read(str(top/name))).hexdigest()==expected,name
    work=Path(tempfile.mkdtemp(prefix='icra_association_portable_',dir=ROOT/'tmp'))
    with zipfile.ZipFile(archive) as bundle:bundle.extractall(work)
    command=[sys.executable,'trials/icra_temporal_association/verify_association_summary.py']
    run=subprocess.run(command,cwd=work/top,capture_output=True,text=True)
    assert run.returncode==0,run.stdout+run.stderr
    assert 'PORTABLE ASSOCIATION SUMMARY VERIFIED' in run.stdout
    report=dict(passed=True,archive_path=str(archive.relative_to(ROOT)),archive_sha256=sha(archive),
        manifest_sha256=sha(manifest_path),files=len(manifest),archive_bytes=archive.stat().st_size,
        fresh_directory=str(work/top),command=command,returncode=run.returncode,stdout=run.stdout.strip(),
        native_valid_results=accepted['valid_native_result_files'],native_results_in_archive=False,
        original_raw_clouds_in_archive=False,detector_checkpoint_in_archive=False,
        scope='Exact archive-member hashes and fresh-directory independent summary recomputation. Includes frozen compact detection/score/pose inputs and original logs. Full native MATLAB re-execution was not repeated by this packaging step.')
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    print('VERIFIED ASSOCIATION SOURCE ARCHIVE',len(manifest),'files;',archive.stat().st_size,'bytes',flush=True)


if __name__=='__main__':main()
