"""Independent explicit sums for raw counts and coarse registration evidence."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.signal import correlate2d

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_v2x_transfer'))
import input_adapter as adapter

sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
inputs={}


def bind(path,expected=None):
    value=sha(path)
    assert expected is None or value==expected,path
    inputs[str(path.relative_to(ROOT))]=value


def project(points,transform):
    assert np.isfinite(points).all() and np.isfinite(transform).all()
    assert np.max(np.abs(points))<1e5
    values=np.column_stack([points[:,0]*transform[i,0]+points[:,1]*transform[i,1]+points[:,2]*transform[i,2]+transform[i,3] for i in range(3)])
    assert np.isfinite(values).all()
    return values


def count_box(points,box):
    x,y,z=(points-box[:3]).T
    cosine,sine=np.cos(box[6]),np.sin(box[6])
    u=x*cosine+y*sine
    v=-x*sine+y*cosine
    keep=(np.abs(u)<=box[5]/2)&(np.abs(v)<=box[4]/2)&(np.abs(z)<=box[3]/2)
    return int(keep.sum())


def grid(points,zlow,zhigh):
    # Fixed display/diagnosis grid. No tracker input is replaced.
    selected=points[(points[:,2]>=zlow)&(points[:,2]<zhigh)]
    hist,_,_=np.histogram2d(selected[:,0],selected[:,1],bins=[np.arange(-50,50.5,.5),np.arange(-40,40.5,.5)])
    return hist>0


def main():
    source=OUT/'RAW_SAMPLE_INSPECTION.json'
    bind(source)
    saved=json.loads(source.read_text())
    assert saved['passed']
    for name,value in saved['inputs'].items():bind(ROOT/name,value)
    rows=[]
    for sample in saved['samples']:
        name,frame=sample['sequence'],sample['frame']
        folder=OUT.parent/('icra_association_test' if name.startswith('v2xt') else 'icra_v2x_transfer')
        freeze=json.loads((folder/'INFERENCE_FREEZE.json').read_text())
        sequence=next(row for row in freeze['sequences'] if row['sequence']==name[-4:])
        scene=ROOT/sequence['scene_path'];stem=sequence['paired_stems'][frame-1]
        raw=[adapter.read_yaml(scene/str(n)/f'{stem}.yaml') for n in [1,2]]
        transforms=adapter.relative_poses(raw)
        points=[];counts=[]
        for index in range(2):
            assert np.array_equal(raw[index]['lidar_pose'],raw[index]['true_ego_pose'])
            path=scene/str(index+1)/f'{stem}.bin'
            bind(path,freeze['raw_file_sha256'][str(path.relative_to(ROOT))])
            native=adapter.read_points(path)
            transformed=project(native[:,:3].astype(float),transforms[index]);points.append(transformed)
            prepared=project(adapter.old.prepare_points(native.copy(),0)[:,:3].astype(float),transforms[index])
            observed=[count_box(transformed,np.asarray(sample['diagnostic_box'])),count_box(prepared,np.asarray(sample['diagnostic_box']))]
            reference=sample['sources'][index]
            assert observed==[reference['raw_points_in_diagnostic_box'],reference['prepared_points_in_diagnostic_box']]
            counts.append(observed)
        bands=[]
        for low,high in [(-5.,3.),(-3.,-1.5),(-1.5,3.)]:
            a,b=[grid(p,low,high) for p in points]
            # Cross correlation gives the translation applied to source 2.
            corr=correlate2d(a.astype(float),b.astype(float),mode='full')
            center=np.array(b.shape)-1
            region=corr[center[0]-20:center[0]+21,center[1]-20:center[1]+21]
            maximum=float(region.max())
            shifts=np.argwhere(region==maximum)-20
            baseline=float(corr[tuple(center)])
            bands.append(dict(common_z_band=[low,high],occupied_voxels=[int(a.sum()),int(b.sum())],
                              native_overlap_voxels=baseline,maximum_overlap_voxels=maximum,
                              maximizing_translation_of_source2_m=(shifts*.5).tolist(),
                              overlap_gain_ratio=maximum/baseline if baseline else None))
        row=dict(sequence=name,frame=frame,exact_box_counts=counts,
                 lidar_pose_equals_true_ego_pose=True,coarse_alignment=bands)
        rows.append(row)
        print('RAW GEOMETRY RECHECK',json.dumps(row),flush=True)
    answer=dict(passed=True,rows=rows,inputs=inputs,source_sha256=sha(Path(__file__)),
                numerical_check='All four source point clouds and box counts recomputed with explicit finite coordinate sums. Initial BLAS warnings did not change these counts; warning cause is unconfirmed.',
                alignment_scope='Coarse occupancy overlap on a fixed 0.5 m grid and +/-10 m translation window in three fixed height bands. This is not an estimated correction or evidence of a uniform pose error.')
    path=OUT/'RAW_GEOMETRY_RECHECK.json';assert not path.exists()
    path.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
