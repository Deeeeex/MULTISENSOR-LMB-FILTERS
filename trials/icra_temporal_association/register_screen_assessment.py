"""Freeze V3 matched controls, complete exposed assessment and additional test."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
REVIEW=OUT.parent/'icra_reviewer_revision'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('dataset',choices=['benchmark_check','controls_development','v2v','v2x','test'])
    args=p.parse_args()
    stage='association_screen_'+('benchmark_check' if args.dataset=='benchmark_check' else 'selected_'+args.dataset)
    destination=OUT/'stages'/(stage+'.json')
    assert not destination.exists() and not (OUT/'results'/stage).exists()
    sources=json.loads((OUT.parent/'icra_gaussian_evidence/source_sha256.json').read_text())
    for name,expected in sources.items():
        assert sha(ROOT/name)==expected,name
    protected=[OUT/'CANDIDATES_V3.md',OUT/'PATH_RESTORATION.md',OUT/'SCREENED_RUNNER_PATCH.json',
        OUT/'SCREENED_AUDIT_MATH_PATCH.json',OUT/'COLUMN_SHAPE_PATCH.json']
    selection_path=OUT/'SCREENED_DEVELOPMENT_SELECTION.json'
    if args.dataset!='benchmark_check':
        selection=json.loads(selection_path.read_text())
        assert selection['passed'] and selection['advance'] and selection['selected']['eligible']
        assert selection['selected']['at_least_one_percent_both_conditions']
        for group in [selection['source_sha256'],selection['inputs']]:
            for name,expected in group.items():
                assert sha(ROOT/name)==expected,name
        selected=selection['selected']['arm']
        mode=selected.split('_assoc_',1)[1]
        assert mode in ['quality','nis','quality_nis']
        control='marked_gaussian_evidence_guarded_scalar_assoc_'+mode
        protected.append(selection_path)
    else:
        selected=control=None
    units=[]
    if args.dataset in ['benchmark_check','controls_development']:
        previous=REVIEW/'stages/controls_development.json'
        units=json.loads(previous.read_text())['units'];protected.append(previous)
        if args.dataset=='benchmark_check':
            units=[u for u in units if u['sequence']=='0000'];arms=['marked_lineage']
            assert len(units)==1
        else:
            arms=[control];assert len(units)==9
    elif args.dataset=='v2v':
        inventory_path=OUT.parent/'icra_full_coverage/DATA_INVENTORY.json'
        inventory=json.loads(inventory_path.read_text())
        mapping={(r['split'],r['sequence']):r for r in inventory['rows']};protected.append(inventory_path)
        for previous,split in [(REVIEW/'stages/gs_seen_transfer.json','train'),
                (OUT.parent/'icra_full_coverage/stages/coverage_remaining_train.json','train'),
                (REVIEW/'stages/new_validation_primary.json','val')]:
            protected.append(previous)
            for row in json.loads(previous.read_text())['units']:
                scene=mapping[split,row['sequence']]
                units.append(dict(row,sequence=split+'_'+row['sequence'],original_sequence=row['sequence'],
                    split=split,scene=scene['scene'],recording=scene['recording']))
        assert len(units)==len({r['scene'] for r in units})==34
        arms=[selected,control]
    else:
        folder=OUT.parent/('icra_association_test' if args.dataset=='test' else 'icra_v2x_transfer')
        manifest_path,audit_path=folder/'NEW_INPUT_MANIFEST.json',folder/'NEW_INPUT_AUDIT.json'
        manifest,audit=json.loads(manifest_path.read_text()),json.loads(audit_path.read_text())
        assert manifest['passed'] and audit['passed'] and audit['manifest_sha256']==sha(manifest_path)
        for name,expected in manifest['source_sha256'].items():
            assert sha(ROOT/name)==expected,name
        freeze_path=folder/'INFERENCE_FREEZE.json';freeze=json.loads(freeze_path.read_text())
        for group in [freeze['protected_sha256'],freeze['raw_file_sha256']]:
            for name,expected in group.items():
                assert sha(ROOT/name)==expected,name
        protected += [manifest_path,audit_path,freeze_path,folder/'DETECTION_MANIFEST.json']
        for row in manifest['sequences']:
            units.append(dict(row,sequence=('v2xt_' if args.dataset=='test' else 'v2x_')+row['sequence'],
                original_sequence=row['sequence'],split='v2x_'+('test' if args.dataset=='test' else 'val'),
                recording=row['collection_date']))
        if args.dataset=='test':
            assert len(units)==14 and sum(r['frames'] for r in units)==2172
            cohort_path=folder/'COHORT_FREEZE.json';cohort=json.loads(cohort_path.read_text())
            assert cohort['development_selection_sha256']==sha(selection_path) and cohort['selected_arm']==selected
            raw_path=folder/'RAW_INPUT_MANIFEST.json';raw=json.loads(raw_path.read_text())
            assert raw['passed'] and not raw['exact_cloud_overlap_with_validation']
            benchmark=OUT/'audit_association_screen_benchmark_check.json'
            check=json.loads(benchmark.read_text());assert check['passed'] and len(check['parity'])==2
            protected += [cohort_path,raw_path,benchmark,folder/'PROTOCOL.md']
            arms=['marked_lineage','marked_gaussian_evidence','marked_gaussian_evidence_guarded_scalar',selected,control]
        else:
            assert len(units)==5 and sum(r['frames'] for r in units)==619
            arms=[selected,control]
    for unit in units:
        assert sha(ROOT/unit['data_path'])==unit['input_sha256']
        protected += [ROOT/unit[k] for k in ['data_path','marks_path','ratios_path'] if k in unit]
    protected += list(OUT.glob('*.py'))+list(OUT.glob('*.m'))
    protected += [REVIEW/name for name in ['runReviewerReplay.m','checkReviewerEvidence.m','fuseReviewerEvidence.m',
        'review_probability_audit.py','review_gaussian_audit.py']]
    protected += [OUT.parent/'icra_fusion_holdout/analyze_holdout.py']
    sources.update({str(p.relative_to(ROOT)):sha(p) for p in protected})
    mapped=[re.sub('[^A-Za-z0-9_]','_',name)[:63] for name in sources]
    assert len(set(mapped))==len(mapped),'MATLAB field-name collision'
    result=dict(protocol='icra-screened-association-v1',stage=stage,cohort='association_screen_assessment_'+args.dataset,
        created_utc=datetime.now(timezone.utc).isoformat(),arms=arms,pd=.9,preflight=args.dataset=='benchmark_check',
        conditions=['reliable','intermittent'],units=units,source_sha256=sources,
        selected_primary=selected,matched_frontend_control=control,
        exposure='Additional prospectively frozen test data' if args.dataset=='test' else 'Previously exposed data')
    if not result['preflight']:
        result['selection_sha256']=sha(selection_path)
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('REGISTERED SCREEN ASSESSMENT',stage,len(units),'segments',len(arms),'arms',len(units)*len(arms)*2,'outputs',flush=True)


if __name__=='__main__':main()
