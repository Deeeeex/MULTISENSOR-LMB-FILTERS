"""Classify every matched-reference reversal of the already fixed root label."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import numpy as np
from scipy.special import expit
from independent_math import source as independent_source
from direction_math import source_state

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
LABEL=(3,100004)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def spans(rows,field):
    result=[];start=None;last=None
    for row in rows:
        if row[field]:
            if start is None:start=row['frame']
            last=row['frame']
        elif start is not None:result.append([start,last]);start=None
    if start is not None:result.append([start,last])
    return result

def main():
    destination=OUT/'REFERENCE_TRACE.json';assert not destination.exists()
    cfg=json.loads((OUT/'SCREEN_FREEZE.json').read_text())
    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name
    old_path=OUT.parent/'icra_peer_detection/MOTIVATING_TRACE.csv'
    with old_path.open(newline='') as stream:old=list(csv.DictReader(stream))
    lookup={(r['condition'],int(r['frame']),int(r['robot'])):r for r in old if r['mode']=='original' and r['backend']=='GCE'}
    noage={(r['condition'],int(r['frame']),int(r['robot'])):r for r in old if r['mode']=='original' and r['backend']=='No-age'}
    rows=[];groups=[];source_checks=[]
    for condition,info in cfg['reference_inputs'].items():
        path=ROOT/info['path'];assert sha(path)==info['sha256']
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        s=source_state(data['runs']);ind=independent_source(data['runs'],False);rec=s['records']
        for name in ['minus_i','ref_z','d0','d1']:
            assert np.allclose(s[name],ind[name],atol=1e-8,rtol=0),(condition,name)
        for name in ['negative_reversal','positive_reversal']:assert np.array_equal(s[name],ind[name])
        source_checks.append(dict(condition=condition,fusion_distributions=len(rec)))
        index={tuple(map(int,r[:4])):i for i,r in enumerate(rec)}
        for robot in [1,2]:
            current=[]
            for t in range(1,241):
                actual=lookup[condition,t,robot];control=noage[condition,t,robot];i=index.get((t,robot,*LABEL))
                row=dict(condition=condition,robot=robot,frame=t,fused_record=i is not None,
                    near_target=actual['root_near_target']=='True',actual_r=None if actual['root_r']=='' else float(actual['root_r']),
                    noage_r=None if control['root_r']=='' else float(control['root_r']),
                    actual_extracted=actual['root_extracted']=='True',noage_extracted=control['root_extracted']=='True',
                    reference_r=None,base_r=None,base_direction=None,actual_direction=None,extra_scalar=None,
                    integral_change=None,negative_reversal=False,positive_reversal=False)
                if i is not None:
                    assert rec[i,9]==row['actual_r']
                    row.update(reference_r=float(expit(s['ref_z'][i])),base_r=float(expit(s['base_z'][i])),
                        base_direction=float(s['d0'][i]),actual_direction=float(s['d1'][i]),extra_scalar=float(s['extra_scalar'][i]),
                        integral_change=float(rec[i,56]-rec[i,10]),negative_reversal=bool(s['negative_reversal'][i]),positive_reversal=bool(s['positive_reversal'][i]))
                current.append(row)
            rows.extend(current)
            groups.append(dict(condition=condition,robot=robot,frames=240,
                negative_reversals=sum(r['negative_reversal'] for r in current),positive_reversals=sum(r['positive_reversal'] for r in current),
                negative_spans=spans(current,'negative_reversal'),positive_spans=spans(current,'positive_reversal')))
        print('REFERENCE DIRECTIONS VERIFIED',condition,480,'root robot frames',flush=True)
    assert len(rows)==960
    table=OUT/'REFERENCE_TRACE.csv'
    with table.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    destination.write_text(json.dumps(dict(passed=True,diagnostic_only=True,root_label=list(LABEL),robot_frames=len(rows),groups=groups,
        scope='Original states only; all frames and both links; not an intervention or selection input',
        freeze_sha256=sha(OUT/'SCREEN_FREEZE.json'),source_sha256=sha(Path(__file__)),table_sha256=sha(table),
        source_table_sha256=sha(old_path),native_inputs=cfg['reference_inputs'],source_checks=source_checks),indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
