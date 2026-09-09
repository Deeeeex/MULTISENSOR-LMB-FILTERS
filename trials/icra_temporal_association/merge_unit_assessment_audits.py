"""Accept all unit audits only after the complete native stage has exited."""
from pathlib import Path
import argparse
import csv
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as handle:
        for data in iter(lambda:handle.read(4*1024**2),b''):digest.update(data)
    return digest.hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage');args=parser.parse_args()
    cfgpath=OUT/'stages'/(args.stage+'.json');runtimepath=OUT/('runtime_'+args.stage+'.json')
    cfg,runtime=[json.loads(p.read_text()) for p in [cfgpath,runtimepath]]
    assert len(runtime)==len(cfg['units']) and cfg['conditions']==['reliable','intermittent']
    checked={}
    def verify(name,expected):
        if name not in checked:checked[name]=sha(ROOT/name)
        assert checked[name]==expected,name
    for name,expected in cfg['source_sha256'].items():verify(name,expected)
    receiptpath=OUT/'UNIT_AUDIT_SCHEDULING_PATCH.json';receipt=json.loads(receiptpath.read_text())
    for key,name in [('source_sha256','audit_screen_assessment_v2.py'),('target_sha256','audit_screen_assessment_unit.py'),('generator_sha256','make_unit_assessment_auditor.py')]:
        verify(str((OUT/name).relative_to(ROOT)),receipt[key])
    inputs={};auditors={};rows=[];diagnostics=[];parity=[];unit_audits=[]
    def include(mapping,name,value):
        if name in mapping:assert mapping[name]==value,name
        mapping[name]=value
    for unit,native in zip(cfg['units'],runtime):
        seq=unit['sequence'];assert native['sequence']==seq and native['returncode']==0 and native['completion_line']
        assert native['files']==len(cfg['arms'])*len(cfg['conditions'])
        path=OUT/'partial_audits'/args.stage/(seq+'.json');report=json.loads(path.read_text())
        assert report['passed'] and report['partial_unit_audit'] and report['execution_unit']==seq
        assert report['stage']==args.stage and report['config_sha256']==sha(cfgpath)
        assert report['native_unit']==native
        snapshot=report['runtime_snapshot']
        assert [r for r in snapshot if r['sequence']==seq]==[native]
        assert hashlib.sha256((json.dumps(snapshot,indent=2)+'\n').encode()).hexdigest()==report['runtime_sha256']
        expected_keys=[(seq,c,a) for c in cfg['conditions'] for a in cfg['arms']]
        assert [(r['sequence'],r['condition'],r['arm']) for r in report['rows']]==expected_keys
        assert len(report['diagnostics'])==len(report['rows'])==native['files']
        assert all(r['frames']==unit['frames'] for r in report['rows'])
        assert report['audited_robot_frames']==sum(2*r['frames'] for r in report['rows'])
        for group,mapping in [('inputs',inputs),('auditor_sha256',auditors)]:
            for name,value in report[group].items():verify(name,value);include(mapping,name,value)
        for _,condition,arm in expected_keys:
            result=OUT/'results'/args.stage/f'{seq}_{condition}_{arm}.json.gz'
            assert str(result.relative_to(ROOT)) in report['inputs']
        name=str(path.relative_to(ROOT));include(inputs,name,sha(path))
        unit_audits.append(name);rows.extend(report['rows']);diagnostics.extend(report['diagnostics']);parity.extend(report['parity'])
    assert len(list((OUT/'results'/args.stage).glob('*.json.gz')))==len(rows)
    assert len(rows)==len(cfg['units'])*len(cfg['conditions'])*len(cfg['arms'])
    for path in [Path(__file__),OUT/'make_unit_assessment_auditor.py',OUT/'audit_screen_assessment_v2.py',receiptpath]:
        include(auditors,str(path.relative_to(ROOT)),sha(path))
    result=dict(passed=True,stage=args.stage,rows=rows,diagnostics=diagnostics,parity=parity,
        audited_robot_frames=sum(2*r['frames'] for r in rows),inputs=inputs,config_sha256=sha(cfgpath),
        runtime_sha256=sha(runtimepath),auditor_sha256=auditors,unit_audits=unit_audits,
        scheduling='The unchanged v2 audit body executed per completed native unit. Each unit output and auditor hash was rechecked here; all native exits and all expected outputs are required before final acceptance.')
    destination=OUT/('audit_'+args.stage+'.json');assert not destination.exists()
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/('scores_'+args.stage+'.csv')).open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    print('ASSOCIATION STAGE AUDIT PASSED',args.stage,result['audited_robot_frames'],'robot-frames',flush=True)


if __name__=='__main__':main()
