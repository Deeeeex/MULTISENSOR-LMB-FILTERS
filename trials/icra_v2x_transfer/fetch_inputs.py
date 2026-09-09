"""Download the metadata-selected vehicle pairs using public scoped access."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
CACHE = ROOT / 'tmp/external_baselines/v2x_real'
PUBLIC_SHARE = 'est8t7lxirg85ohkgoueietxd0xqpf36'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive():
    # Isolate module globals: the old frozen V2V4Real downloader is unchanged.
    path = OUT.parent / 'icra_reviewer_revision/fetch_official_data.py'
    spec = importlib.util.spec_from_file_location('v2x_public_archive', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.CACHE = CACHE
    module.SHARED = PUBLIC_SHARE
    module.VANITY = 'https://ucla.box.com/s/' + PUBLIC_SHARE
    module.ARCHIVES = {'val': ('1668892134513', 2389805356)}
    return module.Archive('val')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=12)
    args = parser.parse_args()
    source = archive()
    directory = source.directory()
    records = directory['entries']
    scenes = sorted({Path(e['name']).parts[1] for e in records if e['name'].endswith('.yaml')})
    cohorts = []
    omitted = []
    for name in scenes:
        by_agent = {}
        for e in records:
            p = Path(e['name'])
            if p.suffix == '.yaml' and p.parts[1] == name:
                by_agent.setdefault(p.parts[2], set()).add(p.stem)
        if not all(a in by_agent for a in ['1', '2']):
            omitted.append(dict(scene=name, agents=sorted(by_agent), reason='Both mobile agents 1 and 2 are required.'))
            continue
        stems = sorted(by_agent['1'] & by_agent['2'])
        assert stems == [f'{i:06d}' for i in range(len(stems))]
        cohorts.append(dict(sequence=f'{len(cohorts):04d}', scene=name,
                            scene_path=str((source.cache / 'files/val' / name).relative_to(ROOT)),
                            collection_date=name[:10], frames=len(stems), paired_stems=stems,
                            unpaired_stems={a: sorted(by_agent[a] - set(stems)) for a in ['1', '2']},
                            radio_seed=8301 + len(cohorts)))
    assert len(cohorts) == 5 and sum(s['frames'] for s in cohorts) == 619
    selected_names = {f"val/{s['scene']}/{a}/{stem}.{kind}"
                      for s in cohorts for a in ['1', '2'] for stem in s['paired_stems'] for kind in ['yaml', 'bin']}
    selected = [e for e in records if e['name'] in selected_names]
    assert len(selected) == len(selected_names) == 2476
    registration = OUT / 'COHORT_FREEZE.json'
    metadata = dict(source_url='https://ucla.box.com/s/' + PUBLIC_SHARE,
                    file_id=source.file_id, archive_bytes=source.size,
                    directory_sha256=sha(source.cache / 'directory.json'),
                    protocol_sha256=sha(OUT / 'PROTOCOL.md'),
                    sequences=cohorts, excluded=omitted, files=selected,
                    created_utc=datetime.now(timezone.utc).isoformat())
    if registration.exists():
        old = json.loads(registration.read_text())
        metadata['created_utc'] = old['created_utc']
        assert metadata == old
    else:
        registration.write_text(json.dumps(metadata, indent=2) + '\n')
    print('COHORT FROZEN', len(cohorts), 'scenes', 619, 'paired frames', flush=True)
    fetched = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(source.read_file, e) for e in selected]):
            fetched.append(future.result())
            if len(fetched) % 100 == 0:
                print('FETCHED V2X', len(fetched), '/', len(selected), flush=True)
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(),
                  cohort_freeze_sha256=sha(registration), files=sorted(fetched, key=lambda r: r['name']),
                  fetch_sha256=sha(Path(__file__)),
                  reused_downloader_sha256=sha(OUT.parent / 'icra_reviewer_revision/fetch_official_data.py'))
    path = OUT / 'RAW_INPUT_MANIFEST.json'
    assert not path.exists()
    path.write_text(json.dumps(report, indent=2) + '\n')
    print('V2X RAW INPUTS COMPLETE', len(fetched), 'CRC and SHA checked files', flush=True)


if __name__ == '__main__':
    main()
