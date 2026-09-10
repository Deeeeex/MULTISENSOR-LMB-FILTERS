"""Preserve failed serialization and normalize only label integer storage."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'CENSUS_FREEZE_V2.json';assert not destination.exists()
    old=json.loads((OUT/'CENSUS_FREEZE.json').read_text())
    failed=json.loads((OUT/'CENSUS_EXECUTION.json').read_text())
    assert failed['completed'] and failed['returncode']==1 and not (OUT/'CENSUS_RESULTS.json').exists()
    for name,digest in old['source_sha256'].items():assert sha(ROOT/name)==digest,name
    patches={
        'census.py': [('def label(row):return tuple(row[2:4].astype(int))',
                       'def label(row):return tuple(map(int,row[2:4]))'),
                      ("OUT/'CENSUS_FREEZE.json'","OUT/'CENSUS_FREEZE_V2.json'"),
                      ("resultdir=OUT/'results'","resultdir=OUT/'results'/'v2'")],
        'verify_census.py': [("OUT/'CENSUS_FREEZE.json'","OUT/'CENSUS_FREEZE_V2.json'"),
                             ("OUT/'CENSUS_EXECUTION.json'","OUT/'CENSUS_EXECUTION_V2.json'"),
                             ("OUT/'results'/f", "OUT/'results'/'v2'/f")],
        'execute_census.py': [("'CENSUS_EXECUTION.json'","'CENSUS_EXECUTION_V2.json'"),
                              ('/census.log','/census_v2.log'),
                              ("'CENSUS_FREEZE.json'","'CENSUS_FREEZE_V2.json'"),
                              ("'census.py'","'census_v2.py'")]
    }
    ports={}
    for name,replacements in patches.items():
        source=OUT/name;text=source.read_text()
        for before,after in replacements:assert before in text;text=text.replace(before,after)
        target=source.with_name(source.stem+'_v2.py');assert not target.exists();target.write_text(text)
        ports[target.name]=dict(source=name,source_sha256=sha(source),output_sha256=sha(target),patches=replacements)
    partial=OUT/'results/reliable_GCE.json.gz';assert partial.exists()
    repair=dict(reason='Tuple labels retained NumPy int64 values, which the standard JSON encoder cannot serialize.',
        change='Normalize label entries to Python int at parsing; numeric label values, populations, ancestry rules and all native inputs are unchanged.',
        failed_partial_file=str(partial.relative_to(ROOT)),failed_partial_sha256=sha(partial),
        failed_execution_sha256=sha(OUT/'CENSUS_EXECUTION.json'),ports=ports)
    fix=OUT/'INTEGER_STORAGE_FIX.json';fix.write_text(json.dumps(repair,indent=2)+'\n')
    sources=old['source_sha256'].copy()
    for p in [OUT/'CENSUS_FREEZE.json',OUT/'CENSUS_EXECUTION.json',ROOT/failed['log'],fix,partial,*OUT.glob('*.py')]:
        sources[str(p.relative_to(ROOT))]=sha(p)
    for name,digest in sources.items():assert sha(ROOT/name)==digest,name
    cfg=old.copy();cfg.update(created_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sources,
        repair='INTEGER_STORAGE_FIX.json',previous_freeze_sha256=sha(OUT/'CENSUS_FREEZE.json'))
    destination.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('CENSUS V2 FROZEN',len(sources),'protected files',flush=True)


if __name__=='__main__':main()
