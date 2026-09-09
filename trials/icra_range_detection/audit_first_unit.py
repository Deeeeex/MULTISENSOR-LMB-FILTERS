"""Early implementation check on completed 0000; never a method-selection result."""
import json
import numpy as np
from scipy.io import loadmat
import audit_stage as audit

OUT=audit.OUT;ROOT=audit.ROOT

def main():
    cfgpath=OUT/'stages/range_detection_preflight.json';cfg=json.loads(cfgpath.read_text())
    for name,expected in cfg['source_sha256'].items():assert audit.sha(ROOT/name)==expected,name
    unit=cfg['units'][0];assert unit['sequence']=='0000'
    mat=loadmat(ROOT/unit['data_path']);T=int(mat['T'].item())
    ratios=loadmat(ROOT/unit['ratios_path'])['likelihoodRatios'];results=[];hashes={}
    for condition in cfg['conditions']:
        delivery=audit.radio_draws(unit['radio_seed']-8301,T,condition)
        for arm in cfg['arms']:
            path=OUT/'results'/cfg['stage']/f'0000_{condition}_{arm}.json.gz';data=audit.read(path);run=data['runs']
            assert np.array_equal(data['delivered'],delivery) and data['inputSha256']==unit['input_sha256']
            assert run['rangeDetectionModel']==unit['range_detection_model']
            quality=audit.audit_actual_pd(run,data)
            view=audit.source_view(audit.packets(run,delivery,T,''))
            density=audit.audit_noage(view,data) if audit.base_arm(arm)=='marked_lineage' else audit.audit_probability(view,data)
            frames,direct=audit.current_records(run,mat,False)
            marks=audit.audit_positive_marks(run,mat,ratios);matching=audit.audit_matching(view,data,frames)
            value,_=audit.score_run(data,run);parity=[]
            if run['rangeDetectionMode']=='nominal':
                reference=ROOT/unit['parity_paths'][condition][arm]
                assert audit.sha(reference)==unit['parity_sha256'][condition][arm]
                old=audit.read(reference)['runs']
                if isinstance(old,list):old=next(r for r in old if r['arm']==arm)
                parity=[key for key in old if key in run and key!='runtimeSeconds']
                for key in parity:assert run[key]==old[key],(condition,arm,key)
            results.append(dict(condition=condition,arm=arm,quality=quality,density=density,direct=direct,
                positive_marks=marks,matching=matching,score=value,parity=parity))
            hashes[str(path.relative_to(ROOT))]=audit.sha(path)
            print('FIRST UNIT AUDITED',condition,arm,flush=True)
    destination=OUT/'FIRST_UNIT_CHECK.json';assert not destination.exists()
    destination.write_text(json.dumps(dict(passed=True,results=results,inputs=hashes,config_sha256=audit.sha(cfgpath),
        auditor_sha256=audit.sha(__import__('pathlib').Path(__file__)),meaning='Implementation and parity only; complete mechanism case still required'),indent=2,allow_nan=False)+'\n')
    print('FIRST UNIT CHECK PASSED; no selection performed',flush=True)

if __name__=='__main__':main()
