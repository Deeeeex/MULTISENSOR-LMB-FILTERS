"""Recompute the frozen selection with stdlib arithmetic and native OSPA arrays."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
GCE='marked_gaussian_evidence';PRIMARY=GCE+'_range'
REFS=[GCE,GCE+'_constant','marked_lineage_range',GCE+'_guarded_scalar_range']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
mean=lambda v:math.fsum(v)/len(v)

def main():
    selection_path=OUT/'SCREEN_SELECTION.json';selection=json.loads(selection_path.read_text())
    calibration_path=OUT.parent/'icra_visibility_diagnostic/RANGE_RECALL_CALIBRATION.json'
    calibration=json.loads(calibration_path.read_text())
    sources={};rows=[];native_frames=0;nominal_parity=0;configs=[]
    for stage in ['range_detection_preflight','range_detection_screen']:
        cfgpath=OUT/'stages'/(stage+'.json');cfg=json.loads(cfgpath.read_text());configs.append(cfg)
        auditpath=OUT/('audit_'+stage+'.json');audit=json.loads(auditpath.read_text())
        assert audit['passed'] and selection['inputs'][str(auditpath.relative_to(ROOT))]==sha(auditpath)
        assert audit['config_sha256']==sha(cfgpath)
        assert audit['runtime_sha256']==sha(OUT/('runtime_'+stage+'.json'))
        assert audit['audited_robot_frames']==sum(2*r['frames'] for r in audit['rows'] if not r['reused'])
        for group in [cfg['source_sha256'],audit['inputs']]:
            for name,expected in group.items():
                assert name not in sources or sources[name]==expected
                sources[name]=expected
        assert cfg['pd']==.9
        expected_count=2 if cfg['preflight'] else 13
        assert len(cfg['units'])==expected_count and len(cfg['arms'])==(9 if cfg['preflight'] else 6)
        for unit in cfg['units']:
            if unit['dataset']=='v2v_development':
                expected=calibration['folds'][unit['recording']]
                assert unit['sequence'] not in expected['training_sequences']
            else:expected=calibration['full_fit']
            assert unit['range_detection_model']==expected
            for condition,paths in unit['parity_paths'].items():
                for arm,name in paths.items():assert sha(ROOT/name)==unit['parity_sha256'][condition][arm]
        nominal_parity+=len(audit['parity'])
        csvpath=OUT/('scores_'+stage+'.csv')
        csvrows=list(csv.DictReader(csvpath.open()))
        assert len(csvrows)==len(audit['rows'])
        by_seq={u['sequence']:u for u in cfg['units']}
        for row,csvrow in zip(audit['rows'],csvrows):
            assert all(str(value)==csvrow[key] for key,value in row.items())
            if row['reused']:
                path=ROOT/by_seq[row['sequence']]['reference_paths'][row['condition']]
            else:
                path=OUT/'results'/stage/f"{row['sequence']}_{row['condition']}_{row['arm']}.json.gz"
                native_frames+=2*row['frames']
            with gzip.open(path,'rt') as handle:data=json.load(handle)
            run=data['runs']
            if isinstance(run,list):run=next(r for r in run if r['arm']==row['arm'])
            values=[x for sensor in run['ospa'] for x in sensor]
            assert len(values)==2*row['frames'] and abs(mean(values)-row['ospa'])<1e-12
            assert row['wire_bytes']==sum(run['wireBytes']) and row['raw_bytes']==sum(run['rawPayloadBytes'])
            assert row['delivered_raw_bytes']==sum(run['deliveredRawBytes'])
            rows.append(dict(row,ospa=mean(values)))
    assert nominal_parity==12
    assert len({(r['dataset'],r['sequence'],r['condition'],r['arm']) for r in rows})==len(rows)
    assert len(rows)==len(selection['rows'])==218
    for left,right in zip(rows,selection['rows']):
        assert left.keys()==right.keys()
        for key,value in left.items():
            if key=='ospa':assert abs(value-right[key])<1e-12
            else:assert value==right[key]
    gates=[]
    for dataset,count in [('v2v_development',9),('v2x_val',5)]:
        chosen=[r for r in rows if r['dataset']==dataset]
        lookup={(r['sequence'],r['condition'],r['arm']):r['ospa'] for r in chosen}
        seqs=sorted({r['sequence'] for r in chosen});assert len(seqs)==count
        for reference in REFS:
            deltas=[mean([lookup[s,c,PRIMARY]-lookup[s,c,reference] for c in ['reliable','intermittent']]) for s in seqs]
            delta=mean(deltas);old=next(g for g in selection['gates'] if g['dataset']==dataset and g['reference']==reference)
            assert abs(delta-old['mean_delta'])<1e-12 and (delta<0)==old['passes']
            gates.append(dict(dataset=dataset,reference=reference,mean_delta=delta,passes=delta<0))
    assert len(selection['gates'])==8 and selection['advance']==all(g['passes'] for g in gates)
    for name,expected in sources.items():assert sha(ROOT/name)==expected,name
    report=dict(passed=True,protected_files=len(sources),audited_native_outputs=192,
        native_robot_frames=native_frames,score_rows=len(rows),exact_nominal_parity_runs=nominal_parity,
        gates=gates,advance=selection['advance'],selection_sha256=sha(selection_path),
        calibration_sha256=sha(calibration_path),verifier_sha256=sha(Path(__file__)),input_sha256=sources)
    destination=OUT/'SELECTION_VERIFICATION.json';assert not destination.exists()
    destination.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('RANGE SELECTION VERIFIED',native_frames,'robot-frames',len(sources),'protected files','advance',report['advance'])

if __name__=='__main__':main()
