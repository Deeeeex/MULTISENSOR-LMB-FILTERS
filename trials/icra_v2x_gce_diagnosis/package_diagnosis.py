"""Verify native fragmentation counts and reproduce the compact diagnosis."""
from pathlib import Path
import gzip
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
verification=json.loads((OUT/'DIAGNOSIS_VERIFICATION.json').read_text());assert verification['passed'] and verification['full']
fragment=json.loads((OUT/'FRAGMENTATION_DIAGNOSIS.json').read_text());assert fragment['passed']
native_checks=0
for name,expected in fragment['inputs'].items():
    assert sha(ROOT/name)==expected,name
    if not name.endswith('.json.gz'):continue
    with gzip.open(ROOT/name,'rt') as handle:data=json.load(handle)
    records=np.asarray(data['runs']['iterationRecords'],float).reshape(-1,60)
    for row in fragment['rows']:
        if row['arm']!=data['runs']['arm']:continue
        t,n=row['frame'],row['robot']
        ids=np.asarray(data['truthIds'][t-1]).reshape(-1)
        xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,np.flatnonzero(ids==row['truth_id'])[0]]
        z=records[(records[:,0]==t)&(records[:,1]==n)]
        nearby=z[np.sum((z[:,4:6]-xy)**2,axis=1)<=4]
        kept=nearby[nearby[:,9]>.001]
        assert len(nearby)==row['preprune_components'] and len(kept)==row['retained_components']
        assert np.isclose(sum(kept[:,9]),row['sum_r'],atol=1e-13,rtol=0)
        assert np.isclose(max(kept[:,9],default=0.),row['max_r'],atol=1e-13,rtol=0)
        assert int((kept[:,2]>=t-9).sum())==row['born_in_last_ten_frames']
        native_checks+=1
assert native_checks==960
output=OUT/'output';output.mkdir(exist_ok=True)
archive=output/'icra_v2x_gce_diagnosis.zip';receipt=output/'PORTABLE_REBUILD.json'
assert not archive.exists() and not receipt.exists()
files=[p for p in OUT.rglob('*') if p.is_file() and not {'output','__pycache__'}.intersection(p.relative_to(OUT).parts)]
files += [p for p in (ROOT/'RUN/ICRA_V2X_GCE_DIAGNOSIS').rglob('*') if p.is_file()]
files += [ROOT/'trials/icra_selective_innovation/positiveInnovationSupport.m',
          ROOT/'trials/icra_asymmetric_evidence/negativeInnovationSupport.m',
          ROOT/'trials/icra_marked_iteration/likelihood_manifest.json']
assert len(files)==len(set(files))
manifest={str(p.relative_to(ROOT)):sha(p) for p in sorted(files)}
manifest_path=output/'SOURCE_MANIFEST.json';manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
top=Path('icra_v2x_gce_diagnosis')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as bundle:
    for path in sorted(files):bundle.write(path,top/path.relative_to(ROOT))
    bundle.write(manifest_path,top/'SOURCE_MANIFEST.json')
fresh=Path(tempfile.mkdtemp(prefix='icra_gce_diagnosis_',dir=ROOT/'tmp'))
with zipfile.ZipFile(archive) as bundle:
    assert bundle.testzip() is None
    for name,expected in manifest.items():
        assert hashlib.sha256(bundle.read(str(top/name))).hexdigest()==expected,name
    bundle.extractall(fresh)
work=fresh/top
checks=[]
for program in ['verify_diagnosis.py','build_report.py','plot_diagnosis.py']:
    run=subprocess.run([sys.executable,'trials/icra_v2x_gce_diagnosis/'+program],cwd=work,capture_output=True,text=True)
    assert run.returncode==0,run.stdout+run.stderr
    checks.append(dict(program=program,returncode=run.returncode,stdout=run.stdout.strip()))
reproduced={}
for name in ['DIAGNOSIS_CN.md','DIAGNOSIS_REPORT_BUILD.json','figures/figure_source.csv',
             'figures/gce_diagnosis.svg','figures/gce_diagnosis.pdf','figures/gce_diagnosis.png']:
    regenerated=work/'trials/icra_v2x_gce_diagnosis'/name
    assert sha(regenerated)==sha(OUT/name),name
    reproduced[name]=sha(regenerated)
report=dict(passed=True,files=len(manifest),archive_sha256=sha(archive),archive_bytes=archive.stat().st_size,
            source_manifest_sha256=sha(manifest_path),fresh_directory=str(work),checks=checks,
            exactly_reproduced=reproduced,independently_checked_native_fragmentation_rows=native_checks,
            native_trajectories_in_archive=False,raw_clouds_in_archive=False,
            scope='Independent native fragmentation counts, exact archive member hashes, standalone arithmetic verification, and byte-identical fresh regeneration of the Chinese report, source CSV and SVG/PDF/PNG figure. No MATLAB rerun.')
receipt.write_text(json.dumps(report,indent=2)+'\n')
print('DIAGNOSIS PACKAGE VERIFIED',len(manifest),'files',archive.stat().st_size,'bytes;',native_checks,'native fragmentation rows',flush=True)
