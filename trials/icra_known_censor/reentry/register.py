"""Bind the completed case and the complete descriptive event scope."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
CASE=OUT.parent
ROOT=CASE.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'FREEZE.json';assert not destination.exists() and not (OUT/'results').exists()
    final=json.loads((CASE/'FINAL_VERIFICATION.json').read_text());assert final['passed'] and final['native_runs']==12
    result=json.loads((CASE/'RESULTS.json').read_text());assert result['passed'] and len(result['rows'])==12
    for name,h in final['input_sha256'].items():assert sha(ROOT/name)==h,name
    cells=[]
    for row in result['rows']:
        arm=row['arm'];kind='refined' if arm.endswith('_known_censor') else 'controls'
        path=CASE/'results'/('known_censor_'+kind)/f"{row['sequence']}_{row['condition']}_{arm}.json.gz"
        assert path.is_file();cells.append(dict(condition=row['condition'],arm=arm,mode=kind,path=str(path.relative_to(ROOT)),sha256=sha(path)))
    files=[CASE/'FINAL_VERIFICATION.json',CASE/'RESULTS.json',OUT/'PROTOCOL.md',*OUT.glob('*.py')]
    sources={str(p.relative_to(ROOT)):sha(p) for p in files}
    sources.update({c['path']:c['sha256'] for c in cells})
    destination.write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),cells=cells,source_sha256=sources,
        exposed_diagnostic=True,native_changes=False,frames=240,threshold=.001,truth_id=5,range_m=2.,window=[53,122]),indent=2)+'\n')
    print('REENTRY CENSUS FROZEN',len(cells),'complete native trajectories',flush=True)

if __name__=='__main__':main()
