"""Bind the post-census reimport question before enumerating the event join."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'REIMPORT_FREEZE.json';assert not destination.exists()
    cfg=json.loads((OUT/'CENSUS_FREEZE_V2.json').read_text())
    result=json.loads((OUT/'CENSUS_RESULTS.json').read_text())
    verified=json.loads((OUT/'CENSUS_VERIFICATION.json').read_text());assert result['passed'] and verified['passed']
    assert verified['report_sha256']==sha(OUT/'CENSUS_RESULTS.json')
    sources=cfg['source_sha256'].copy();sources.update(result['artifacts'])
    for p in [OUT/'REIMPORT_PROTOCOL.md',OUT/'CENSUS_FREEZE_V2.json',OUT/'CENSUS_RESULTS.json',
              OUT/'CENSUS_VERIFICATION.json',OUT/'CENSUS_EXECUTION_V2.json',OUT/'POPULATION.csv',
              OUT/'TARGET_ANCESTRY.csv',*OUT.glob('*.py')]:sources[str(p.relative_to(ROOT))]=sha(p)
    for name,digest in sources.items():assert sha(ROOT/name)==digest,name
    out=dict(protocol='icra-same-frame-reimport-census-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        cells=cfg['cells'],source_sha256=sources)
    destination.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    print('REIMPORT FROZEN',len(sources),'files',flush=True)


if __name__=='__main__':main()
