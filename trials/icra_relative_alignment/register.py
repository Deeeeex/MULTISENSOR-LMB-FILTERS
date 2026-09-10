"""Freeze the complete raw cohort, estimator and native comparison before use."""
from datetime import datetime,timezone
from pathlib import Path
import ast
import json
import sys
import numpy as np
from scipy.io import loadmat
from common import OUT,ROOT,ARMS,sha,write_new
from fixtures import check

sys.path.insert(0,str(OUT.parent/'icra_reviewer_revision'))
import official_input_adapter as v2v
sys.path.insert(0,str(OUT.parent/'icra_v2x_transfer'))
import input_adapter as v2x

def main():
    assert not (OUT/'FREEZE.json').exists() and not (OUT/'features').exists() and not (OUT/'results').exists()
    for p in OUT.glob('*.py'):ast.parse(p.read_text())
    sources={}
    def bind(path,expected=None):
        path=path if path.is_absolute() else ROOT/path
        value=sha(path);assert expected is None or expected==value,str(path)
        sources[str(path.relative_to(ROOT))]=value
        return value
    vg=OUT.parent/'icra_reviewer_revision/stages/controls_development.json'
    vs=OUT.parent/'icra_projected_admission/stages/final_v2x_evaluation.json'
    va=OUT.parent/'icra_projected_admission/audit_final_v2x_evaluation.json'
    old=json.loads(vg.read_text());external=json.loads(vs.read_text());audit=json.loads(va.read_text());assert audit['passed']
    bind(vs,audit['config_sha256']);bind(va);bind(vg)
    for name,digest in old['source_sha256'].items():bind(ROOT/name,digest)
    posepath=OUT.parent/'icra_reviewer_revision/POSE_INPUTS_development.json'
    pose=json.loads(posepath.read_text());assert pose['passed'];bind(posepath)
    posemap={r['sequence']:r for r in pose['sequences']}
    stablepath=OUT.parent/'icra_marked_iteration/summary_development.json'
    stable=json.loads(stablepath.read_text());bind(stablepath)
    gcepath=OUT.parent/'icra_gaussian_evidence/summary_development.json'
    gce=json.loads(gcepath.read_text());bind(gcepath)
    rawfreeze=OUT.parent/'icra_v2x_transfer/INFERENCE_FREEZE.json'
    vxraw=json.loads(rawfreeze.read_text());bind(rawfreeze)
    vxmap={u['sequence']:u for u in vxraw['sequences']}
    directories={}
    for name in ['test_01','test_02','test_03']:
        p=v2v.CACHE/name/'directory.json';bind(p)
        directories[name]=json.loads(p.read_text())
    units=[];pcd_count=bin_count=0
    for dataset,originals in [('v2v_development',old['units']),('v2x_val',external['units'])]:
        for sourceunit in originals:
            u=dict(sourceunit);name=u['sequence'];vv=dataset=='v2v_development'
            mat=loadmat(ROOT/u['data_path']);T=int(mat['T'].item());u['frames']=T;u['dataset']=dataset
            if vv:
                meta=posemap[name];scene=meta['scene'];u['scene']=scene;u['recording']=scene.rsplit('_',1)[0]
                archive=next(k for k in directories if (v2v.CACHE/k/'files'/scene).is_dir())
                folder=v2v.CACHE/archive/'files'/scene
                stems=sorted(p.stem for p in (folder/'0').glob('*.yaml'));assert len(stems)==T
                entrymap={e['name']:e for e in directories[archive]['entries']}
                u['pose_path']=meta['pose_path'];u['pose_sha256']=meta['pose_sha256']
                gcefile=OUT.parent/'icra_gaussian_evidence/results_development'/f'{name}_reliable_marked_gaussian_evidence.json.gz'
                assert sha(gcefile)==gce['inputs'][str(gcefile.relative_to(ROOT))]
                noage=OUT.parent/'icra_marked_iteration/results_stable'/f'{name}_reliable.json.gz'
                ref=next(r for r in stable['inputs'] if r['sequence']==name and r['condition']=='reliable')
                assert sha(noage)==ref['sha256']
            else:
                meta=vxmap[u['original_sequence']];scene=meta['scene'];folder=ROOT/meta['scene_path'];stems=meta['paired_stems']
                assert len(stems)==T
                gcefile=OUT.parent/'icra_projected_admission/results/final_v2x_evaluation'/f'{name}_reliable_marked_gaussian_evidence.json.gz'
                noage=OUT.parent/'icra_projected_admission/results/final_v2x_evaluation'/f'{name}_reliable_marked_lineage.json.gz'
                for path in [gcefile,noage]:assert sha(path)==audit['inputs'][str(path.relative_to(ROOT))]
            u['input_files']={}
            for field in ['data_path','marks_path','ratios_path','pose_path']:
                if u.get(field):u['input_files'][u[field]]=bind(ROOT/u[field])
            assert u['input_files'][u['data_path']]==u['input_sha256']
            u['references']={arm:dict(path=str(path.relative_to(ROOT)),sha256=sha(path)) for arm,path in zip(ARMS,[gcefile,noage])}
            pairs=[]
            for frame,stem in enumerate(stems,1):
                yaml_paths=[folder/str(n)/f'{stem}.yaml' for n in ([0,1] if vv else [1,2])]
                raw=[v2v.read_yaml(p) for p in yaml_paths]
                matrices=(v2v if vv else v2x).relative_poses(raw)
                assert all(np.isfinite(m).all() for m in matrices)
                assert np.max(np.abs(matrices[1][:2,3]-mat['positions'][:,1,frame-1]))<1e-5
                entries=[]
                for index,yp in enumerate(yaml_paths):
                    expected=pose['source_sha256'][str(yp.relative_to(ROOT))] if vv else vxraw['raw_file_sha256'][str(yp.relative_to(ROOT))]
                    bind(yp,expected)
                    p=yp.with_suffix('.pcd' if vv else '.bin')
                    info=dict(source=index+1,path=str(p.relative_to(ROOT)),transform=matrices[index].tolist())
                    if vv:
                        key=str(p.relative_to(v2v.CACHE/archive/'files'));entry=entrymap[key]
                        info.update(archive=archive,entry=entry);pcd_count+=1
                    else:
                        digest=vxraw['raw_file_sha256'][str(p.relative_to(ROOT))]
                        assert sha(p)==digest;info.update(sha256=digest,bytes=p.stat().st_size);bin_count+=1
                    entries.append(info)
                pairs.append(dict(frame=frame,stem=stem,sources=entries))
            u['raw_pairs']=pairs;u['alignment_path']=str((OUT/'features'/f'{name}_alignment.mat').relative_to(ROOT));units.append(u)
            print('RAW COHORT FROZEN',dataset,name,T,flush=True)
    assert len(units)==14 and sum(u['frames'] for u in units)==2612 and pcd_count==3986 and bin_count==1238
    for p in list(OUT.glob('*.py'))+list(OUT.glob('*.m'))+list(OUT.glob('*.md'))+[OUT/'RUNNER_PORT.json']:
        bind(p)
    for p in [Path(v2v.__file__),Path(v2x.__file__),OUT.parent/'icra_reviewer_revision/fetch_official_data.py',
              OUT.parent/'icra_reviewer_revision/review_probability_audit.py',OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py',
              OUT.parent/'icra_reviewer_revision/audit_stage.py',OUT.parent/'icra_marked_control/analyze_control.py',
              OUT.parent/'icra_visibility_diagnostic/RAW_ALIGNMENT_SEQUENCE.json']:
        bind(p)
    result=dict(protocol='icra-relative-alignment-v1',created_utc=datetime.now(timezone.utc).isoformat(),
                arms=ARMS,conditions=['reliable'],pd=.9,units=units,raw_pcd_files=pcd_count,raw_bin_files=bin_count,
                native_parity_sequences=['0000','v2x_0002'],native_corrected_runs=28,raw_frames=2612,
                fixtures=check(),sources=sources,gates={'v2v_development':'OSPA and GOSPA nonincreasing for each method',
                                                     'v2x_val':'OSPA and GOSPA at least one percent lower for each method'},
                exposure='All fourteen segments previously seen; no new recording or generalization claim')
    write_new(OUT/'FREEZE.json',result)
    print('RELATIVE ALIGNMENT FROZEN',len(sources),'source files;',pcd_count+bin_count,'raw clouds',flush=True)

if __name__=='__main__':main()
