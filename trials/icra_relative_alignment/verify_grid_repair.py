"""Verify the actual failing raw cloud and both sides of every grid boundary."""
from pathlib import Path
import json
import zlib
import numpy as np
from common import OUT,ROOT,sha,write_new,frozen
from prepare_features import parse_cloud
import alignment_math as method
import independent_math as previous
import independent_voxel_index as repair

def main():
    frozen();diagnostic=json.loads((OUT/'GRID_CHECK_DIAGNOSTIC.json').read_text());info=diagnostic['source']
    path=OUT/'features/diagnostic_failed_cloud.pcd';raw=path.read_bytes()
    assert sha(path)==diagnostic['raw_sha256'] and len(raw)==info['entry']['raw'] and zlib.crc32(raw)==info['entry']['crc']
    points=method.project(parse_cloud(raw,True),np.array(info['transform']))
    expected=method.occupancy(points);old=previous.occupancy(points);fixed=repair.occupancy(points)
    assert not np.array_equal(old,expected) and np.array_equal(fixed,expected)
    assert np.argwhere(old!=expected).tolist()==diagnostic['old_mismatch_cells']
    checked=failures=0;examples=[]
    for axis,edges in [(0,np.arange(-50,50.5,.5)),(1,np.arange(-40,40.5,.5))]:
        for boundary in edges:
            for position in [np.nextafter(boundary,-np.inf),boundary,np.nextafter(boundary,np.inf)]:
                point=np.array([[.125,.125,0.]])
                point[0,axis]=position;reference=method.occupancy(point)
                assert np.array_equal(repair.occupancy(point),reference),(axis,float(boundary),float(position))
                mismatch=not np.array_equal(previous.occupancy(point),reference);failures+=mismatch;checked+=1
                if mismatch and len(examples)<12:examples.append(dict(axis=axis,boundary=float(boundary),position=float(position),position_hex=float(position).hex()))
    assert checked==1086 and failures>0
    protected=[Path(__file__),OUT/'independent_voxel_index.py',OUT/'GRID_CHECK_DIAGNOSTIC.json',path,
               ROOT/diagnostic['range_path'],OUT/'prepare_features_index_retry.py',OUT/'resume_features_v3.py']
    write_new(OUT/'GRID_VERIFIER_REPAIR.json',dict(passed=True,actual_source=info['path'],actual_raw_sha256=sha(path),
        actual_mismatch_cells=diagnostic['old_mismatch_cells'],actual_repaired_grid_exact=True,
        grid_edge_cases=checked,old_false_failures=failures,examples=examples,
        method_sha256=sha(OUT/'alignment_math.py'),original_preparer_sha256=sha(OUT/'prepare_features.py'),
        native_runner_sha256=sha(OUT/'runAlignmentReplay.m'),freeze_sha256=sha(OUT/'FREEZE.json'),
        artifacts={str(p.relative_to(ROOT)):sha(p) for p in protected},
        scope='Independent voxel checker only: floor(2*x)+integer_origin. No producer grid, correction, input, tracking, or statistical gate changes.'))
    print('VOXEL CHECKER REPAIR VERIFIED',checked,'boundary cases;',failures,'old false failures; actual raw grid now exact',flush=True)

if __name__=='__main__':main()
