"""Add the fixed QN recursions to every completed exposed-release comparison."""
from datetime import datetime, timezone
from pathlib import Path
import csv
import gzip
import hashlib
import json

from association_aggregates import summarize,exposed_scopes,METRICS
from identity_metrics import evaluate_identities

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE='marked_gaussian_evidence'
GS=BASE+'_guarded_scalar'
SELECTED=BASE+'_assoc_quality_nis'
CONTROL=GS+'_assoc_quality_nis'
OLD=BASE+'_assoc_split'
OLDCONTROL=GS+'_assoc_split'
FOCUS=[BASE,GS,OLD,OLDCONTROL,SELECTED,CONTROL]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination=OUT/'SCREENED_ASSESSMENT.json';assert not destination.exists()
    previous_path=OUT/'RESTORED_ASSESSMENT.json';previous=json.loads(previous_path.read_text())
    assert previous['passed'] and len(previous['rows'])==874 and len(previous['identity_rows'])==384
    selection_path=OUT/'SCREENED_DEVELOPMENT_SELECTION.json';selection=json.loads(selection_path.read_text())
    assert selection['passed'] and selection['advance'] and selection['selected']['arm']==SELECTED
    rows,identities=previous['rows'].copy(),previous['identity_rows'].copy()
    inputs={str(p.relative_to(ROOT)):sha(p) for p in [previous_path,selection_path]}

    def verify(group):
        for name,expected in group.items():
            if name in inputs:assert inputs[name]==expected,name
            else:assert sha(ROOT/name)==expected,name
            inputs[name]=expected

    for report in [previous,selection]:
        verify(report['inputs']);verify(report['source_sha256'])
    metadata={(r['dataset'],r['sequence']):r for r in previous['rows']}
    for stage,allowed,count in [
        ('association_screen_preflight',[SELECTED],30),
        ('association_screen_rest',[SELECTED],36),
        ('association_screen_selected_controls_development',[CONTROL],18),
        ('association_screen_selected_v2v',[SELECTED,CONTROL],136),
        ('association_screen_selected_v2x',[SELECTED,CONTROL],20)]:
        audit_path=OUT/('audit_'+stage+'.json');audit=json.loads(audit_path.read_text())
        assert audit['passed'] and len(audit['rows'])==count
        cfgpath=OUT/'stages'/(stage+'.json');runtimepath=OUT/('runtime_'+stage+'.json')
        assert sha(cfgpath)==audit['config_sha256'] and sha(runtimepath)==audit['runtime_sha256']
        cfg,runtime=json.loads(cfgpath.read_text()),json.loads(runtimepath.read_text())
        assert len(cfg['units'])==len(runtime)
        assert all(r['returncode']==0 and r['completion_line'] and r['files']==2*len(cfg['arms']) for r in runtime)
        for group in [cfg['source_sha256'],audit['auditor_sha256'],audit['inputs']]:verify(group)
        for p in [audit_path,cfgpath,runtimepath]:inputs[str(p.relative_to(ROOT))]=sha(p)
        units={u['sequence']:u for u in cfg['units']}
        for row in audit['rows']:
            if row['arm'] not in allowed:continue
            unit=units[row['sequence']]
            if stage.endswith('_v2x'):
                dataset,sequence='v2x_val',row['sequence']
            else:
                dataset='v2v';sequence=unit.get('split','test')+'_'+unit.get('original_sequence',row['sequence'])
            meta=metadata[dataset,sequence];assert meta['frames']==row['frames']
            path=OUT/'results'/stage/f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
            assert sha(path)==inputs[str(path.relative_to(ROOT))]
            with gzip.open(path,'rt') as handle:data=json.load(handle)
            run=data['runs'];identity=evaluate_identities(data)
            identities.append(dict(dataset=dataset,sequence=sequence,condition=row['condition'],arm=row['arm'],**identity))
            rows.append(dict(dataset=dataset,cohort=meta['cohort'],sequence=sequence,scene=meta['scene'],recording=meta['recording'],
                condition=row['condition'],arm=row['arm'],frames=row['frames'],**{k:row[k] for k in METRICS},
                wire_bytes=int(run['totalWireBytes']),raw_payload_bytes=int(sum(run['rawPayloadBytes'])),
                delivered_raw_bytes=int(sum(run['deliveredRawBytes'])),split_branches=len(run['associationSplits'])))
            print('FIXED SCREEN IDENTITY CHECKED',dataset,sequence,row['condition'],row['arm'],flush=True)
    assert len(rows)==1066 and len(identities)==576
    assert len({(r['dataset'],r['sequence'],r['condition'],r['arm']) for r in rows})==len(rows)
    comparisons=[(SELECTED,BASE),(CONTROL,GS),(SELECTED,CONTROL),(SELECTED,OLD),(SELECTED,'marked_lineage')]
    aggregate,paired=summarize(rows,identities,FOCUS,comparisons,exposed_scopes())
    result=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),selected=SELECTED,matched_control=CONTROL,
        rows=rows,aggregate=aggregate,identity_rows=identities,paired_recording=paired,inputs=inputs,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),OUT/'association_aggregates.py',
            OUT/'identity_metrics.py',OUT/'diagnose_association_v2.py']},
        interval_definition=previous['interval_definition'],
        exposure='Complete exposed-release assessment of the method fixed on nine development segments. Additional frozen test results are reported separately.')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'screened_assessment_scores.csv').open('w') as handle:
        names=list(dict.fromkeys(k for row in rows for k in row))
        writer=csv.DictWriter(handle,fieldnames=names,lineterminator='\n');writer.writeheader();writer.writerows(rows)
    print('FIXED SCREEN ASSESSMENT COMPLETE',len(rows),'rows',flush=True)
    for r in aggregate:
        if r['scope'] in ['v2v_all','v2x_val'] and r['arm'] in [BASE,GS,SELECTED,CONTROL]:
            print(r['scope'],r['condition'],r['arm'],r['sequence_macro']['ospa'],r['recording_macro_ospa'],flush=True)


if __name__=='__main__':main()
