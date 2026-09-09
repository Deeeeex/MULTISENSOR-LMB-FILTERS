"""Bind final reports and diagnostic additions to the completed frozen experiment."""
from pathlib import Path
import csv
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda name:json.loads((OUT/name).read_text())

def main():
    final=OUT/'FINAL_VERIFICATION.json';assert not final.exists()
    selection=read('SCREEN_SELECTION.json');verified=read('SELECTION_VERIFICATION.json')
    assert selection['passed'] and verified['passed'] and verified['selection_sha256']==sha(OUT/'SCREEN_SELECTION.json')
    pipeline=read('PIPELINE_EXECUTION.json')
    assert pipeline['complete'] and len(pipeline['steps'])==7 and all(r['returncode']==0 for r in pipeline['steps'])
    assert pipeline['orchestrator_sha256']==sha(OUT/'finish_pipeline.py')
    for name,expected in pipeline['source_sha256'].items():assert sha(OUT/name)==expected,name
    hashes=dict(verified['input_sha256']);native_runs=0
    for stage in ['range_detection_preflight','range_detection_screen']:
        cfg=read('stages/'+stage+'.json');audit=read('audit_'+stage+'.json');runtime=read('runtime_'+stage+'.json')
        assert audit['passed'] and len(runtime)==len(cfg['units'])
        assert all(r['returncode']==0 and r['completion_line'] and r['files']==2*len(cfg['arms']) for r in runtime)
        native_runs+=sum(r['files'] for r in runtime)
        for name,expected in audit['auditor_sha256'].items():
            assert sha(ROOT/name)==expected,name
            hashes[name]=expected
    assert native_runs==192
    receipt=read('RUNNER_PATCH.json')
    for kind in ['source','destination','audit_source','audit_destination']:
        assert sha(ROOT/receipt[kind])==receipt[kind+'_sha256']
    repair=read('METADATA_FIELD_REPAIR.json')
    assert sha(OUT/'audit_stage.py')==repair['original_auditor_sha256']
    assert sha(OUT/'audit_stage_v2.py')==repair['replacement_auditor_sha256']
    assert (OUT/'audit_stage.py').read_text().replace(repair['before'],repair['after'])==(OUT/'audit_stage_v2.py').read_text()
    first=read('FIRST_UNIT_CHECK.json');assert first['passed'] and len(first['results'])==18
    assert first['auditor_sha256']==sha(OUT/'audit_first_unit.py')
    for name,expected in first['inputs'].items():assert sha(ROOT/name)==expected
    trace=read('MECHANISM_TRACE.json');assert trace['passed'] and trace['source_sha256']==sha(OUT/'diagnose_mechanism.py')
    assert trace['csv_sha256']==sha(OUT/'MECHANISM_FRAMES.csv')
    frames=list(csv.DictReader((OUT/'MECHANISM_FRAMES.csv').open()));assert len(frames)==trace['rows']==2520
    assert len({(r['condition'],r['arm'],r['frame'],r['robot']) for r in frames})==2520
    for summary in trace['summaries']:
        part=[r for r in frames if (r['condition'],r['arm'],r['phase'])==(summary['condition'],summary['arm'],summary['phase'])]
        missed=[r for r in part if r['output_match']=='False']
        assert len(part)==summary['robot_frames']
        assert sum(r['output_match']=='True' for r in part)==summary['detected']
        assert sum(int(r['near_active_components'])==0 for r in missed)==summary['missed_without_active_component_within_2m']
        assert sum(int(r['near_active_components'])>0 for r in missed)==summary['missed_with_active_component_within_2m']
        assert max(int(r['active_components']) for r in part)==summary['max_active_components']
    for row in selection['mechanism']:
        part=[r for r in frames if (r['condition'],r['arm'])==(row['condition'],row['arm'])]
        assert len(part)==row['total']==140
        assert sum(r['output_match']=='True' for r in part)==row['detected_target_robot_frames_53_122']
    cf=read('SCALAR_CASE_SUBSTITUTION.json');cv=read('CASE_SUBSTITUTION_VERIFICATION.json')
    assert cf['passed'] and cv['passed'] and cf['source_sha256']==sha(OUT/'diagnose_scalar_case.py')
    assert cv['substitution_sha256']==sha(OUT/'SCALAR_CASE_SUBSTITUTION.json')
    assert cv['verifier_sha256']==sha(OUT/'verify_case_substitution.py') and cv['counterfactual_robot_frames']==560
    mass=read('ASSIGNMENT_MASS_DIAGNOSTIC.json');assert mass['passed'] and len(mass['rows'])==6
    assert mass['source_sha256']==sha(OUT/'diagnose_assignment_mass.py')
    assert all(r['mark']==0 and r['conditional_mark_mass']==0 and r['joint_mark_mass']==0 for r in mass['rows'] if r['source']==1)
    for data in [trace,mass]:
        for name,expected in data['inputs'].items():assert sha(ROOT/name)==expected,name
    grouping=read('RECORDING_SENSITIVITY.json');assert grouping['passed'] and len(grouping['results'])==24
    assert grouping['selection_sha256']==sha(OUT/'SCREEN_SELECTION.json')
    assert grouping['source_sha256']==sha(OUT/'recording_sensitivity.py')
    for row in grouping['results']:
        values=row['per_group_delta'];assert len(values)==row['groups']
        assert abs(sum(values.values())/len(values)-row['group_macro_delta'])<1e-12
        for omitted,delta in row['leave_one_group_out_delta'].items():
            chosen=[v for key,v in values.items() if key!=omitted]
            assert abs(sum(chosen)/len(chosen)-delta)<1e-12
    builds=[('REPORT_BUILD.json',{'selection_sha256':'SCREEN_SELECTION.json','verification_sha256':'SELECTION_VERIFICATION.json',
        'builder_sha256':'build_report.py','report_sha256':'RESULTS_CN.md','all_screen_scores_sha256':'ALL_SCREEN_SCORES.csv'}),
        ('MECHANISM_NOTE_BUILD.json',{'trace_sha256':'MECHANISM_TRACE.json','substitution_sha256':'SCALAR_CASE_SUBSTITUTION.json',
            'csv_sha256':'MECHANISM_FRAMES.csv','builder_sha256':'build_mechanism_note.py','note_sha256':'MECHANISM_NOTES_CN.md'}),
        ('ASSIGNMENT_MASS_NOTE_BUILD.json',{'diagnostic_sha256':'ASSIGNMENT_MASS_DIAGNOSTIC.json','note_sha256':'ASSIGNMENT_MASS_NOTES_CN.md'}),
        ('RECORDING_REPORT_BUILD.json',{'data_sha256':'RECORDING_SENSITIVITY.json','report_sha256':'RECORDING_SENSITIVITY_CN.md','source_sha256':'recording_sensitivity.py'})]
    for name,fields in builds:
        data=read(name)
        for field,target in fields.items():assert data[field]==sha(OUT/target),(name,field)
    for path in OUT.rglob('*'):
        if not path.is_file() or 'results' in path.relative_to(OUT).parts or '__pycache__' in path.relative_to(OUT).parts:continue
        hashes[str(path.relative_to(ROOT))]=sha(path)
    for name,expected in hashes.items():assert sha(ROOT/name)==expected,name
    final.write_text(json.dumps(dict(passed=True,protected_files=len(hashes),native_runs=native_runs,
        native_robot_frames=verified['native_robot_frames'],nominal_parity_runs=verified['exact_nominal_parity_runs'],
        screen_advance=selection['advance'],case_trace_rows=2520,independent_substitution_frames=560,
        recording_sensitivity_comparisons=24,source_sha256=sha(Path(__file__)),input_sha256=hashes,
        boundary='Evidence reconstruction of this exposed-data screen; not new generalization evidence or manuscript acceptance'),indent=2)+'\n')
    print('FINAL RANGE ARCHIVE VERIFIED',native_runs,'native runs',len(hashes),'files','advance',selection['advance'])

if __name__=='__main__':main()
