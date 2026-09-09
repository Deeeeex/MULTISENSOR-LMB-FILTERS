"""Describe every registered event using audited native logs, without new interventions."""
from pathlib import Path
import gzip
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'EVENT_DETAILS.json';assert not destination.exists()
    audit=json.loads((OUT/'audit_recursion_interventions.json').read_text());assert audit['passed']
    details=[];protected={}
    for row in audit['rows']:
        arm=row['arm'];path=OUT/'results/recursion_interventions'/f"{row['sequence']}_intermittent_{arm}.json.gz"
        name=str(path.relative_to(ROOT));assert sha(path)==audit['inputs'][name];protected[name]=sha(path)
        with gzip.open(path,'rt') as stream:data=json.load(stream)
        run=data['runs'];records=np.asarray(run['interventionRecords'],float).reshape(-1,54)
        assert np.isfinite(records).all() and (records[:,0]==3).all()
        target=np.flatnonzero(np.asarray(data['truthIds'][2]).ravel()==5).item();xy=np.asarray(data['truth'][2],float).reshape(4,-1)[:2,target]
        near=np.sqrt(np.sum((records[:,8:10]-xy)**2,axis=1))<=2
        mode=run['interventionMode']
        if mode=='existence':assert np.array_equal(records[:,16:20],records[:,8:12]) and np.array_equal(records[:,40:50],records[:,20:30])
        if mode=='spatial':assert np.array_equal(records[:,7],records[:,5])
        details.append(dict(arm=arm,mode=mode,event_rows=len(records),robot_rows=[int((records[:,1]==n).sum()) for n in [1,2]],
            maximum_candidate_existence_difference=float(abs(records[:,6]-records[:,5]).max()),
            maximum_applied_existence_difference=float(abs(records[:,7]-records[:,5]).max()),
            maximum_applied_position_shift_m=float(np.sqrt(np.sum((records[:,16:18]-records[:,8:10])**2,axis=1)).max()),
            maximum_applied_velocity_shift_mps=float(np.sqrt(np.sum((records[:,18:20]-records[:,10:12])**2,axis=1)).max()),
            maximum_applied_covariance_element_difference=float(abs(records[:,40:50]-records[:,20:30]).max()),
            target_near_rows=[dict(robot=int(r[1]),label=[int(x) for x in r[2:4]],old_r=float(r[5]),candidate_r=float(r[6]),applied_r=float(r[7])) for r in records[near]]))
    # All three events are computed from their exactly identical GS prefix.
    assert len({r['maximum_candidate_existence_difference'] for r in details})==1
    result=dict(passed=True,scope='Descriptive audit of all three already fixed interventions; no new native work',
        events=details,inputs=protected,source_sha256=sha(Path(__file__)))
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('ALL EVENT DETAILS CHECKED',json.dumps(details,allow_nan=False),flush=True)

if __name__=='__main__':main()
