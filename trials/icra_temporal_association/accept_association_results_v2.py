"""Check completed valid stages, immutable inputs and final analysis receipts."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TEST = OUT.parent / 'icra_association_test'
STAGES = [
    'association_instrumentation_development',
    'association_restored_preflight', 'association_restored_development_rest',
    'association_selected_controls_development', 'association_selected_v2v_column',
    'association_selected_v2x', 'association_screen_preflight', 'association_screen_rest',
    'association_screen_selected_controls_development', 'association_screen_selected_v2v',
    'association_screen_selected_v2x', 'association_screen_benchmark_check',
    'association_screen_selected_test']
INVALID_STAGES = ['association_v1_preflight', 'association_v1_development_rest', 'association_v2_preflight']


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for data in iter(lambda: handle.read(4*1024**2), b''):
            digest.update(data)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args(); checked = {}; results = {}; stages = []

    def verify(path, expected=None):
        key = str(path.relative_to(ROOT))
        if key not in checked:
            checked[key] = sha(path)
        if expected is not None:
            assert checked[key] == expected, key
        return checked[key]

    original_path = OUT.parent / 'icra_gaussian_evidence/source_sha256.json'
    original = json.loads(original_path.read_text()); verify(original_path)
    for name, expected in original.items():
        verify(ROOT/name, expected)
    for stage in STAGES:
        cfgpath = OUT/'stages'/(stage+'.json')
        auditpath = OUT/('audit_'+stage+'.json')
        runtimepath = OUT/('runtime_'+stage+'.json')
        cfg, audit, runtime = [json.loads(p.read_text()) for p in [cfgpath,auditpath,runtimepath]]
        assert audit['passed'] and len(cfg['units']) == len(runtime)
        verify(auditpath); verify(cfgpath,audit['config_sha256']); verify(runtimepath,audit['runtime_sha256'])
        expected_count = len(runtime)*len(cfg['arms'])*len(cfg['conditions'])
        files = sorted((OUT/'results'/stage).glob('*.json.gz'))
        assert len(files) == len(audit['rows']) == expected_count
        for unit, native in zip(cfg['units'],runtime):
            assert native['sequence'] == unit['sequence'] and native['returncode'] == 0
            assert native['completion_line'] and native['files'] == len(cfg['arms'])*len(cfg['conditions'])
            log = ROOT/'RUN/ICRA_TEMPORAL_ASSOCIATION'/stage/(unit['sequence']+'.log')
            assert 'COMPLETED REVIEW '+stage+' '+unit['sequence'] in log.read_text()
            verify(log)
        for group in [cfg['source_sha256'],audit['auditor_sha256'],audit['inputs']]:
            for name, expected in group.items():
                verify(ROOT/name, expected)
        for path in files:
            name = str(path.relative_to(ROOT)); assert name in audit['inputs'] and name not in results
            results[name] = dict(bytes=path.stat().st_size,sha256=verify(path,audit['inputs'][name]))
        stages.append(dict(stage=stage,native_units=len(runtime),result_files=expected_count,
            audited_node_frames=sum(2*r['frames'] for r in audit['rows']),exact_parity_entries=len(audit['parity'])))
        print('VALID STAGE VERIFIED',stage,expected_count,'outputs',flush=True)
    assert len(results) == 650
    invalid = {}
    for stage in INVALID_STAGES:
        for path in sorted((OUT/'results'/stage).glob('*.json.gz')):
            invalid[str(path.relative_to(ROOT))] = dict(bytes=path.stat().st_size,sha256=verify(path))
    assert len(invalid) == 50 and not set(invalid).intersection(results)
    for stage,count in [('association_selected_v2v',34),('association_selected_v2v_loaded',1)]:
        path=OUT/('runtime_'+stage+'.json'); rows=json.loads(path.read_text());verify(path)
        assert len(rows)==count and all(r['returncode']!=0 and r['files']==0 for r in rows)
        assert not list((OUT/'results'/stage).glob('*.json.gz'))
    analysis = {}
    for name, count in [('RESTORED_DEVELOPMENT_SELECTION.json',90),
                        ('SCREENED_DEVELOPMENT_SELECTION.json',144),
                        ('RESTORED_ASSESSMENT_V2.json',874), ('SCREENED_ASSESSMENT_V2.json',1066),
                        ('ADDITIONAL_TEST_ANALYSIS.json',140)]:
        path=OUT/name; report=json.loads(path.read_text()); assert report['passed'] and len(report['rows'])==count
        verify(path)
        for field in ['inputs','source_sha256']:
            for key,expected in report.get(field,{}).items():
                verify(ROOT/key,expected)
        analysis[name]=dict(rows=count,sha256=verify(path))
    rawpath=TEST/'RAW_INPUT_MANIFEST.json';raw=json.loads(rawpath.read_text())
    assert raw['passed'] and len(raw['files'])==8688 and not raw['exact_cloud_overlap_with_validation']
    verify(rawpath)
    for row in raw['files']:
        verify(ROOT/row['path'],row['sha256'])
    freeze=json.loads((TEST/'INFERENCE_FREEZE.json').read_text())
    for group in [freeze['protected_sha256'],freeze['raw_file_sha256']]:
        for key,expected in group.items():
            verify(ROOT/key,expected)
    detection=json.loads((TEST/'DETECTION_MANIFEST.json').read_text())
    assert detection['passed'] and not detection['truth_used_for_detections']
    assert len(detection['sequences'])==14 and sum(r['frames'] for r in detection['sequences'])==2172
    for row in detection['sequences']:
        verify(ROOT/row['path'],row['sha256'])
    manifest=json.loads((TEST/'NEW_INPUT_MANIFEST.json').read_text())
    audit=json.loads((TEST/'NEW_INPUT_AUDIT.json').read_text())
    assert manifest['passed'] and audit['passed'] and audit['manifest_sha256']==verify(TEST/'NEW_INPUT_MANIFEST.json')
    for row in manifest['sequences']:
        verify(ROOT/row['data_path'],row['input_sha256']);verify(ROOT/row['pose_path'],row['pose_sha256'])
    selection=json.loads((OUT/'SCREENED_DEVELOPMENT_SELECTION.json').read_text())
    cohort=json.loads((TEST/'COHORT_FREEZE.json').read_text())
    assert cohort['development_selection_sha256']==verify(OUT/'SCREENED_DEVELOPMENT_SELECTION.json')
    assert datetime.fromisoformat(selection['created_utc'])<datetime.fromisoformat(cohort['created_utc'])
    final=json.loads((OUT/'ADDITIONAL_TEST_ANALYSIS.json').read_text())
    renderpath=OUT/'FINAL_RESULTS_RENDER.json';render=json.loads(renderpath.read_text());assert render['passed']
    for key,expected in render['inputs'].items():verify(ROOT/key,expected)
    verify(ROOT/render['output_path'],render['output_sha256'])
    verify(OUT/'build_final_association_report_v2.py',render['generator_sha256']);verify(renderpath)
    for base in [OUT,TEST]:
        for path in sorted(base.glob('*')):
            if path.is_file() and path.suffix in ['.m','.py','.md','.json','.csv'] and path.name not in ['REPLAY_ACCEPTANCE.json','RESULT_FILES_MANIFEST.json']:
                verify(path)
    result_path=OUT/'RESULT_FILES_MANIFEST.json';accept_path=OUT/'REPLAY_ACCEPTANCE.json'
    manifest_report=dict(valid_results=results,invalid_results_preserved=invalid)
    core=dict(passed=True,valid_stages=stages,valid_native_result_files=len(results),
        invalid_native_result_files_preserved=len(invalid),native_result_bytes=sum(r['bytes'] for r in results.values()),
        independently_audited_node_frames=sum(r['audited_node_frames'] for r in stages),
        exact_parity_entries=sum(r['exact_parity_entries'] for r in stages),
        protected_original_sources_unchanged=len(original),failed_zero_output_units_preserved=35,
        analysis=analysis,selected=selection['selected']['arm'],additional_test_segments=14,
        additional_test_paired_frames=2172,additional_test_raw_files=8688,
        selected_before_new_raw_acquisition=True,transfer_improves_both_conditions=final['transfer_improves_both_conditions'],
        verification_sha256=sha(Path(__file__)),checked_files=checked)
    if args.verify_only:
        assert json.loads(result_path.read_text())==manifest_report
        previous=json.loads(accept_path.read_text())
        for key,value in core.items():assert previous[key]==value,key
        assert previous['result_manifest_sha256']==sha(result_path)
    else:
        assert not result_path.exists() and not accept_path.exists()
        result_path.write_text(json.dumps(manifest_report,indent=2,sort_keys=True)+'\n')
        core.update(completed_utc=datetime.now(timezone.utc).isoformat(),result_manifest_sha256=sha(result_path))
        accept_path.write_text(json.dumps(core,indent=2,allow_nan=False)+'\n')
    print('ASSOCIATION ASSESSMENT ACCEPTED',len(results),'native outputs;',core['independently_audited_node_frames'],
          'robot-frames;',len(original),'unchanged original sources',flush=True)


if __name__=='__main__':main()
