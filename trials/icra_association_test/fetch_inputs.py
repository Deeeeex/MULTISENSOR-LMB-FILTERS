"""Acquire the prospectively fixed test cohort only after the development gate."""
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
STUDY = OUT.parent / 'icra_temporal_association'
CACHE = ROOT / 'tmp/external_baselines/v2x_real'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=12)
    args = parser.parse_args()
    selected_path = STUDY / 'SCREENED_DEVELOPMENT_SELECTION.json'
    selected = json.loads(selected_path.read_text())
    assert selected['passed'] and selected['advance'] and selected['selected']['eligible']
    assert selected['selected']['at_least_one_percent_both_conditions']
    for group in [selected['inputs'], selected['source_sha256']]:
        for name, expected in group.items():
            assert sha(ROOT / name) == expected, name
    prospective_path = STUDY / 'PROSPECTIVE_TEST_COHORT.json'
    prospective = json.loads(prospective_path.read_text())
    for name, expected in prospective['sources'].items():
        assert sha(ROOT / name) == expected, name
    source_path = OUT.parent / 'icra_reviewer_revision/fetch_official_data.py'
    spec = importlib.util.spec_from_file_location('additional_public_archive', source_path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.CACHE = CACHE
    module.SHARED = '429ak8yk8pawnd84xqx5opcsl0p2mvrp'
    module.VANITY = 'https://ucla.box.com/s/' + module.SHARED
    module.ARCHIVES = {'test': ('1668588258866', 5772980847)}
    source = module.Archive('test')
    directory = source.directory()
    assert source.file_id == prospective['file_id'] and source.size == prospective['archive_bytes']
    entries = {e['name']:e for e in directory['entries']}
    assert all(entries[e['name']] == e for e in prospective['files'])
    remaining = sum(e['raw'] for e in prospective['files'] if not (source.cache/'files'/e['name']).exists())
    assert shutil.disk_usage(ROOT).free > remaining + 3*1024**3, 'Require room for raw data plus inference/replay outputs; do not delete existing results.'
    sequences = [dict(r, scene_path=str((source.cache/'files/test'/r['scene']).relative_to(ROOT))) for r in prospective['sequences']]
    cohort = dict(created_utc=datetime.now(timezone.utc).isoformat(), source_url=module.VANITY,
        file_id=source.file_id, archive_bytes=source.size, sequences=sequences, files=prospective['files'],
        protocol_sha256=sha(OUT/'PROTOCOL.md'), directory_sha256=sha(source.cache/'directory.json'),
        prospective_freeze_sha256=sha(prospective_path), development_selection_sha256=sha(selected_path),
        selected_arm=selected['selected']['arm'], excluded=[])
    registration = OUT/'COHORT_FREEZE.json'
    if registration.exists():
        old=json.loads(registration.read_text());cohort['created_utc']=old['created_utc'];assert old==cohort
    else:
        registration.write_text(json.dumps(cohort,indent=2)+'\n')
    destination=OUT/'RAW_INPUT_MANIFEST.json';assert not destination.exists()
    print('ADDITIONAL TEST ACQUISITION FROZEN',len(sequences),'segments',prospective['paired_frames'],'paired frames',flush=True)
    fetched=[];failures=[];pending={};remaining=iter(prospective['files'])
    pool=ThreadPoolExecutor(max_workers=args.workers)
    try:
        for _ in range(args.workers):
            entry=next(remaining,None)
            if entry is not None:pending[pool.submit(source.read_file,entry)]=entry
        while pending:
            done,_=wait(pending,timeout=30,return_when=FIRST_COMPLETED)
            if not done:
                print('WAITING FOR PUBLIC RANGES',len(fetched),'validated;',len(pending),'in flight',flush=True)
                continue
            for future in done:
                entry=pending.pop(future)
                try:
                    fetched.append(future.result())
                except Exception as error:
                    headers=getattr(error,'headers',{}) or {}
                    failure=dict(name=entry['name'],error_type=type(error).__name__,
                        http_status=getattr(error,'code',None),retry_after=headers.get('Retry-After'))
                    failures.append(failure)
                    print('PUBLIC RANGE FAILED',json.dumps(failure),flush=True)
                    break
                if len(fetched)%200==0:
                    print('VALIDATED ADDITIONAL TEST',len(fetched),'/',len(prospective['files']),flush=True)
                entry=next(remaining,None)
                if entry is not None:pending[pool.submit(source.read_file,entry)]=entry
            if failures:
                for future in pending:future.cancel()
                break
    finally:
        pool.shutdown(wait=True,cancel_futures=True)
    if failures:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        receipt=OUT/('FETCH_FAILURE_'+stamp+'.json')
        receipt.write_text(json.dumps(dict(passed=False,failures=failures,validated_files=len(fetched),
            source_sha256=sha(Path(__file__)),cohort_freeze_sha256=sha(registration),workers=args.workers),indent=2)+'\n')
        raise RuntimeError('Public input acquisition incomplete; completed cache files are preserved.')
    validation_path=OUT.parent/'icra_v2x_transfer/RAW_INPUT_MANIFEST.json'
    validation=json.loads(validation_path.read_text());assert validation['passed']
    old_clouds={r['sha256'] for r in validation['files'] if r['name'].endswith('.bin')}
    overlap=[r['name'] for r in fetched if r['name'].endswith('.bin') and r['sha256'] in old_clouds]
    result=dict(passed=not bool(overlap),completed_utc=datetime.now(timezone.utc).isoformat(),
        cohort_freeze_sha256=sha(registration),files=sorted(fetched,key=lambda r:r['name']),
        fetch_sha256=sha(Path(__file__)),reused_downloader_sha256=sha(source_path),
        exposed_validation_manifest_sha256=sha(validation_path),exact_cloud_overlap_with_validation=overlap)
    destination.write_text(json.dumps(result,indent=2)+'\n')
    assert not overlap, 'Keep all files and stop for input-overlap review before inference.'
    print('ADDITIONAL TEST RAW INPUTS COMPLETE',len(fetched),'files; exact old-cloud overlaps',len(overlap),flush=True)


if __name__=='__main__':
    main()
