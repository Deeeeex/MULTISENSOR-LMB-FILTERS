"""Normalize singleton MATLAB event arrays without changing tracking or scores."""
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
STAGES=['association_restored_preflight','association_restored_development_rest',
    'association_selected_controls_development','association_selected_v2v_column','association_selected_v2x',
    'association_screen_preflight','association_screen_rest','association_screen_selected_controls_development',
    'association_screen_selected_v2v','association_screen_selected_v2x']


def in_scope(row,scope):
    if scope=='v2v_all':return row['dataset']=='v2v'
    if scope=='v2v_development':return row['dataset']=='v2v' and row['cohort']=='development'
    if scope=='v2v_remaining':return row['dataset']=='v2v' and row['cohort']!='development'
    assert scope=='v2x_val'
    return row['dataset']=='v2x_val'


def copy_patch(source_name,target_name,changes):
    source,target=OUT/source_name,OUT/target_name;assert not target.exists()
    text=source.read_text()
    for before,after in changes:
        assert before in text,(source_name,before)
        text=text.replace(before,after)
    target.write_text(text)
    return dict(source=source_name,target=target_name,source_sha256=sha(source),target_sha256=sha(target),
                replacements=[dict(before=a,after=b) for a,b in changes])


def main():
    assert not (OUT/'ADDITIONAL_TEST_ANALYSIS.json').exists()
    # MATLAB serializes zero rows as [], a single row as a flat vector,
    # and multiple rows as nested vectors. All events have six fields.
    for raw,count in [([],0),([1,1,2,3,2,1000003],1),([[1,1,2,3,2,1000003],[2,2,2,3,2,2000003]],2)]:
        assert len(np.asarray(raw,float).reshape(-1,6))==count
    counts={};audit_inputs={}
    for stage in STAGES:
        path=OUT/('audit_'+stage+'.json');report=json.loads(path.read_text());assert report['passed']
        audit_inputs[str(path.relative_to(ROOT))]=sha(path)
        for row in report['diagnostics']:
            if '_assoc_' not in row['arm']:continue
            seq=row['sequence'];dataset='v2x_val' if seq.startswith('v2x_') else 'v2v'
            if dataset=='v2v' and not seq.startswith(('train_','val_')):seq='test_'+seq
            key=(dataset,seq,row['condition'],row['arm'])
            if 'split_branches' not in row['association']:
                assert row['arm'].endswith(('_assoc_direct','_assoc_temporal'))
            value=row['association'].get('split_branches',0)
            assert isinstance(value,int) and value>=0
            if key in counts:assert counts[key]==value,key
            counts[key]=value
    corrected=[]
    for source_name,target_name in [('RESTORED_ASSESSMENT.json','RESTORED_ASSESSMENT_V2.json'),
                                    ('SCREENED_ASSESSMENT.json','SCREENED_ASSESSMENT_V2.json')]:
        source,target=OUT/source_name,OUT/target_name;assert not target.exists()
        old=json.loads(source.read_text());assert old['passed'];new=deepcopy(old);changes=[]
        for i,row in enumerate(new['rows']):
            if 'split_branches' not in row:continue
            key=(row['dataset'],row['sequence'],row['condition'],row['arm'])
            count=counts[key] if '_assoc_' in row['arm'] else 0
            if count!=row['split_branches']:
                changes.append(dict(section='rows',index=i,key=list(key),before=row['split_branches'],after=count))
                assert row['split_branches']==6 and count==1,key
                row['split_branches']=count
        for i,item in enumerate(new['aggregate']):
            if 'communication' not in item:continue
            count=sum(r['split_branches'] for r in new['rows'] if in_scope(r,item['scope']) and
                      r['condition']==item['condition'] and r['arm']==item['arm'])
            before=item['communication']['split_branches']
            if count!=before:
                changes.append(dict(section='aggregate',index=i,key=[item['scope'],item['condition'],item['arm']],before=before,after=count))
                item['communication']['split_branches']=count
        restored=deepcopy(new)
        for change in changes:
            if change['section']=='rows':restored['rows'][change['index']]['split_branches']=change['before']
            else:restored['aggregate'][change['index']]['communication']['split_branches']=change['before']
        assert restored==old,'Only event-count fields may differ.'
        new['inputs'].update(audit_inputs);new['inputs'][str(source.relative_to(ROOT))]=sha(source)
        new['source_sha256'][str(Path(__file__).relative_to(ROOT))]=sha(Path(__file__))
        new['event_count_repair']=dict(original_report_sha256=sha(source),changes=changes,
            rule='Read the independently audited number of six-field branch-event rows, including singleton rows.',
            all_other_metrics_and_identity_fields_unchanged=True)
        target.write_text(json.dumps(new,indent=2,allow_nan=False)+'\n')
        corrected.append(dict(source=source_name,target=target_name,source_sha256=sha(source),target_sha256=sha(target),changes=changes))
    patches=[]
    patches.append(copy_patch('analyze_additional_test.py','analyze_additional_test_v2.py',[
        ("    rows=[];identities=[];final_rows=[];opportunities=[]\n",
         "    rows=[];identities=[];final_rows=[];opportunities=[]\n    event_counts={(r['sequence'],r['condition'],r['arm']):r['association'].get('split_branches',0) for r in audit['diagnostics']}\n"),
        ("        run=data['runs'];T=len(data['time']);assert T==row['frames']==unit['frames']\n",
         "        run=data['runs'];T=len(data['time']);assert T==row['frames']==unit['frames']\n        split_events=np.asarray(run['associationSplits'],float).reshape(-1,6)\n        assert len(split_events)==event_counts[row['sequence'],row['condition'],row['arm']]\n        assert all(1<=int(e[0])<=T and int(e[1]) in [1,2] for e in split_events)\n"),
        ("split_branches=len(run['associationSplits'])","split_branches=len(split_events)"),
        ("for event in run['associationSplits']","for event in split_events")]))
    names=[('RESTORED_ASSESSMENT.json','RESTORED_ASSESSMENT_V2.json'),
           ('SCREENED_ASSESSMENT.json','SCREENED_ASSESSMENT_V2.json')]
    patches.append(copy_patch('build_final_association_report.py','build_final_association_report_v2.py',names))
    patches.append(copy_patch('accept_association_results.py','accept_association_results_v2.py',names+[
        ('build_final_association_report.py','build_final_association_report_v2.py')]))
    patches.append(copy_patch('verify_association_summary.py','verify_association_summary_v2.py',names))
    patches.append(copy_patch('package_association_study.py','package_association_study_v2.py',[
        ('verify_association_summary.py','verify_association_summary_v2.py')]))
    receipt=dict(completed_utc=datetime.now(timezone.utc).isoformat(),passed=True,corrected_reports=corrected,
        script_patches=patches,source_sha256=sha(Path(__file__)),audit_inputs=audit_inputs,
        failure_log_sha256=sha(ROOT/'RUN/ICRA_TEMPORAL_ASSOCIATION/additional_test_analysis.log'),
        native_results_changed=False,ospa_gospa_cardinality_identity_communication_bytes_changed=False,
        method_or_selection_changed=False,reason='A one-row MATLAB matrix is a flat six-value JSON vector. The old len counted six fields; the corrected count is one event. Independent native audits already reshape all events to six columns and have the correct counts.')
    (OUT/'EVENT_SHAPE_SUMMARY_REPAIR.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('EVENT SHAPE SUMMARY REPAIRED',sum(len(r['changes']) for r in corrected),'count fields; all other metrics unchanged',flush=True)


if __name__=='__main__':main()
