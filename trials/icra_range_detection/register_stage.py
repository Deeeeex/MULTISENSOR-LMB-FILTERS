"""Freeze the declared shared-model screen before any recursive outcomes."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE=['marked_gaussian_evidence','marked_lineage','marked_gaussian_evidence_guarded_scalar']
NEW=[a+s for s in ['_range','_constant'] for a in BASE]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=['preflight','screen']);args=parser.parse_args()
    stage='range_detection_'+args.kind;destination=OUT/'stages'/(stage+'.json')
    assert not destination.exists() and not (OUT/'results'/stage).exists()
    prior=OUT.parent/'icra_miss_history/stages'/('miss_history_'+args.kind+'.json')
    old=json.loads(prior.read_text());sources=old['source_sha256'].copy()
    for name,expected in sources.items():assert sha(ROOT/name)==expected,name
    calibration=OUT.parent/'icra_visibility_diagnostic/RANGE_RECALL_CALIBRATION.json'
    fits=json.loads(calibration.read_text());assert fits['passed']
    units=old['units']
    for unit in units:
        v2v=unit['dataset']=='v2v_development'
        fit=fits['folds'][unit['recording']] if v2v else fits['full_fit']
        if v2v:assert unit['sequence'] not in fit['training_sequences']
        unit['range_detection_model']=fit
        unit['calibration_fold']=unit['recording'] if v2v else 'full_v2v_development_transfer'
        unit['parity_paths']={}
        if args.kind=='preflight':
            for condition in old['conditions']:
                unit['parity_paths'][condition]={BASE[0]:unit['reference_paths'][condition]}
                if v2v:
                    paths={BASE[1]:'icra_temporal_association/results/association_screen_benchmark_check',
                           BASE[2]:'icra_reviewer_revision/results/controls_development'}
                else:paths={a:'icra_temporal_association/results/association_screen_selected_test' for a in BASE[1:]}
                for arm,folder in paths.items():
                    path=OUT.parent/folder/f"{unit['sequence']}_{condition}_{arm}.json.gz"
                    assert path.is_file(),path
                    unit['parity_paths'][condition][arm]=str(path.relative_to(ROOT))
        # Native paths share long prefixes and cannot be flattened into MATLAB
        # struct field names without truncation collisions. Bind them per unit.
        unit['parity_sha256']={condition:{arm:sha(ROOT/name) for arm,name in paths.items()}
            for condition,paths in unit['parity_paths'].items()}
    protected=[prior,calibration,OUT.parent/'icra_visibility_diagnostic/RANGE_RECALL_PROTOCOL.md',
        OUT.parent/'icra_visibility_diagnostic/DIAGNOSTIC_VERIFICATION.json',OUT/'PROTOCOL.md',OUT/'RUNNER_PATCH.json']
    protected+=list(OUT.rglob('*.m'))+list(OUT.glob('*.py'))
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in protected})
    fields=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources]
    assert len(fields)==len(set(fields))
    cfg=dict(protocol='icra-range-detection-v1',stage=stage,cohort=stage,
        created_utc=datetime.now(timezone.utc).isoformat(),pd=.9,preflight=args.kind=='preflight',
        arms=(BASE+NEW if args.kind=='preflight' else NEW),conditions=old['conditions'],units=units,
        source_sha256=sources,exposure='All data previously exposed; frozen common observation model and screen only')
    destination.parent.mkdir(exist_ok=True);destination.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('REGISTERED',stage,len(units)*2*len(cfg['arms']),'outputs',len(sources),'protected files',flush=True)

if __name__=='__main__':main()
