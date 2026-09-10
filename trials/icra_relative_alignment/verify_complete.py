"""Close the input-retry provenance chain before the registered final report."""
from pathlib import Path
import json
from common import OUT,ROOT,sha,write_new,frozen

def main():
    cfg=frozen();freeze=sha(OUT/'FREEZE.json');artifacts={};attempts=[]
    path=OUT/'FEATURE_EXECUTION.json'
    while True:
        report=json.loads(path.read_text());artifacts[str(path.relative_to(ROOT))]=sha(path)
        log=ROOT/report['log'];assert sha(log)==report['log_sha256'];artifacts[report['log']]=sha(log)
        assert report['freeze_sha256']==freeze
        attempt=report.get('attempt',1)
        script={1:'execute_features.py',2:'resume_features.py',3:'resume_features_v3.py'}[attempt]
        assert report['source_sha256']==sha(OUT/script)
        artifacts[str((OUT/script).relative_to(ROOT))]=sha(OUT/script)
        assert report['returncode']==(0 if attempt==3 else 1) and report['completed']==(attempt==3)
        attempts.append(dict(attempt=attempt,returncode=report['returncode'],receipt=str(path.relative_to(ROOT))))
        if 'previous_receipt' not in report:break
        path=ROOT/report['previous_receipt'];assert sha(path)==report['previous_receipt_sha256']
    assert [r['attempt'] for r in attempts]==[3,2,1]
    current=json.loads((OUT/'FEATURE_EXECUTION.json').read_text())
    assert current['unchanged_estimator_sha256']==sha(OUT/'alignment_math.py')==cfg['sources'][str((OUT/'alignment_math.py').relative_to(ROOT))]
    assert current['original_preparer_sha256']==sha(OUT/'prepare_features.py')
    assert current['effective_voxel_checker_sha256']==sha(OUT/'independent_voxel_index.py')
    assert current['transport_retry_sha256']==sha(OUT/'prepare_features_retry.py')
    assert current['retry_entry_sha256']==sha(OUT/'prepare_features_index_retry.py')
    repair_path=OUT/'GRID_VERIFIER_REPAIR.json';repair=json.loads(repair_path.read_text())
    assert repair['passed'] and repair['freeze_sha256']==freeze and current['checker_repair_sha256']==sha(repair_path)
    assert repair['grid_edge_cases']==1086 and repair['old_false_failures']==248 and repair['actual_repaired_grid_exact']
    for name,digest in repair['artifacts'].items():assert sha(ROOT/name)==digest,name
    artifacts.update(repair['artifacts'])
    feature=json.loads((OUT/'FEATURES.json').read_text());est=json.loads((OUT/'ESTIMATION.json').read_text())
    assert feature['passed'] and est['passed'] and feature['freeze_sha256']==est['freeze_sha256']==freeze
    assert feature['source_sha256']==sha(OUT/'prepare_features.py')
    assert est['feature_sha256']==sha(OUT/'FEATURES.json') and est['feature_execution_sha256']==sha(OUT/'FEATURE_EXECUTION.json')
    assert len(feature['rows'])==5224 and [r['index'] for r in feature['rows']]==list(range(5224))
    local_files=0
    for row in feature['rows']:
        path=ROOT/row['raw_path']
        if path.exists():assert sha(path)==row['raw_sha256'];local_files+=1
    for stage,count in [('parity',4),('corrected',28)]:
        path=OUT/f'audit_alignment_{stage}.json';audit=json.loads(path.read_text())
        assert audit['passed'] and audit['native_runs']==count and audit['freeze_sha256']==freeze
        assert audit['source_sha256']==sha(OUT/'audit_stage.py')
        for name,digest in audit['artifacts'].items():assert sha(ROOT/name)==digest,name
        artifacts[str(path.relative_to(ROOT))]=sha(path)
    for p in [OUT/'FEATURES.json',OUT/'ESTIMATION.json',repair_path,OUT/'independent_voxel_index.py',
              OUT/'prepare_features_retry.py',OUT/'prepare_features_index_retry.py',OUT/'INPUT_TRANSPORT_NOTE.md',OUT/'GRID_CHECK_NOTE.md',Path(__file__)]:
        artifacts[str(p.relative_to(ROOT))]=sha(p)
    write_new(OUT/'PREPARATION_CHAIN_VERIFICATION.json',dict(passed=True,freeze_sha256=freeze,attempts=attempts,
        retained_existing_raw_files_rehashed=local_files,feature_clouds=5224,native_runs=32,
        producer_grid_unchanged=True,raw_selection_unchanged=True,independent_index_repair_checked=True,
        native_and_statistical_protocol_unchanged=True,artifacts=artifacts,verifier_sha256=sha(Path(__file__))))
    print('COMPLETE PREPARATION AND NATIVE PROVENANCE VERIFIED',32,'runs; all three input attempts preserved',flush=True)

if __name__=='__main__':main()
