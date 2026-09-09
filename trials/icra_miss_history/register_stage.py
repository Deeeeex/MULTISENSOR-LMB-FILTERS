"""Freeze the complete miss-history screen before recursive outcomes."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
GCE='marked_gaussian_evidence'
ARMS=[GCE+'_miss_history',GCE+'_miss_half']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('kind',choices=['preflight','screen']);args=p.parse_args()
    stage='miss_history_'+args.kind
    destination=OUT/'stages'/(stage+'.json')
    assert not destination.exists() and not (OUT/'results'/stage).exists()
    prior=OUT.parent/'icra_temporal_association/stages/association_screen_selected_test.json'
    original=json.loads(prior.read_text());sources=original['source_sha256'].copy()
    for name,expected in sources.items():assert sha(ROOT/name)==expected,name
    development=OUT.parent/'icra_reviewer_revision/stages/controls_development.json'
    external=OUT.parent/'icra_projected_admission/stages/final_v2x_evaluation.json'
    inventory_path=OUT.parent/'icra_full_coverage/DATA_INVENTORY.json'
    inventory=json.loads(inventory_path.read_text())
    mapping={(r['split'],r['sequence']):r for r in inventory['rows']}
    dev=[]
    for u in json.loads(development.read_text())['units']:
        scene=mapping['test',u['sequence']]
        dev.append(dict(u,dataset='v2v_development',scene=scene['scene'],recording=scene['recording']))
    ext=[dict(u,dataset='v2x_val') for u in json.loads(external.read_text())['units']]
    test=[dict(u,dataset='v2x_test_mechanism') for u in original['units'] if u['sequence']=='v2xt_0001']
    assert len(dev)==9 and len(ext)==5 and len(test)==1
    units=([dev[0],test[0]] if args.kind=='preflight' else [u for u in dev if u['sequence']!='0000']+ext)
    assert (dev[0]['sequence']=='0000') and len(units)==(2 if args.kind=='preflight' else 13)
    for unit in units:
        assert sha(ROOT/unit['data_path'])==unit['input_sha256']
        for key in ['data_path','marks_path','ratios_path']:
            if key in unit:sources[unit[key]]=sha(ROOT/unit[key])
        unit['reference_paths']={};unit['reference_sha256']={}
        unit['noage_reference_paths']={};unit['noage_reference_sha256']={}
        for condition in ['reliable','intermittent']:
            if unit['dataset']=='v2v_development':
                base=OUT.parent/'icra_temporal_association/results/association_instrumentation_development'
            elif unit['dataset']=='v2x_val':
                base=OUT.parent/'icra_projected_admission/results/final_v2x_evaluation'
            else:base=OUT.parent/'icra_temporal_association/results/association_screen_selected_test'
            reference=base/(unit['sequence']+'_'+condition+'_'+GCE+'.json.gz')
            assert reference.is_file(),reference
            name=str(reference.relative_to(ROOT));unit['reference_paths'][condition]=name
            unit['reference_sha256'][condition]=sha(reference)
            if unit['dataset']=='v2x_test_mechanism':
                noage=base/(unit['sequence']+'_'+condition+'_marked_lineage.json.gz')
                assert noage.is_file()
                unit['noage_reference_paths'][condition]=str(noage.relative_to(ROOT))
                unit['noage_reference_sha256'][condition]=sha(noage)
    protected=[prior,development,external,inventory_path,OUT/'PROTOCOL.md',OUT/'RUNNER_PATCH.json']
    protected+=list(OUT.glob('*.m'))+list(OUT.glob('*.py'))
    sources.update({str(path.relative_to(ROOT)):sha(path) for path in protected})
    fields=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources]
    assert len(fields)==len(set(fields))
    config=dict(protocol='icra-miss-history-v1',stage=stage,cohort='miss_history_'+args.kind,
                created_utc=datetime.now(timezone.utc).isoformat(),pd=.9,preflight=args.kind=='preflight',
                arms=([GCE]+ARMS if args.kind=='preflight' else ARMS),
                conditions=['reliable','intermittent'],units=units,source_sha256=sources,
                exposure='All inputs previously exposed; prospectively frozen method and execution only')
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(config,indent=2,allow_nan=False)+'\n')
    print('REGISTERED',stage,len(units),'units',len(units)*2*len(config['arms']),'outputs',len(sources),'protected files',flush=True)


if __name__=='__main__':main()
