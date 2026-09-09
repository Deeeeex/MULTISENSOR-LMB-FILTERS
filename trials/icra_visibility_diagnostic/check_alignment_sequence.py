"""Five fixed, evenly spaced raw alignment checks per diagnosed V2X scene."""
from pathlib import Path
import hashlib
import json
import zlib

import numpy as np
from scipy.signal import fftconvolve

from recheck_raw_geometry import adapter,project,grid

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    rows=[];inputs={}
    cases=[('v2x_0002','icra_v2x_transfer',[1,36,71,106,140]),
           ('v2xt_0001','icra_association_test',[1,60,120,180,240])]
    for name,directory,frames in cases:
        folder=OUT.parent/directory
        freeze_path=folder/'INFERENCE_FREEZE.json'
        manifest_path=folder/'RAW_INPUT_MANIFEST.json'
        for path in [freeze_path,manifest_path]:inputs[str(path.relative_to(ROOT))]=sha(path)
        freeze=json.loads(freeze_path.read_text())
        raw_manifest=json.loads(manifest_path.read_text())
        raw_lookup={row['path']:row for row in raw_manifest['files']}
        sequence=next(row for row in freeze['sequences'] if row['sequence']==name[-4:])
        scene=ROOT/sequence['scene_path']
        for frame in frames:
            stem=sequence['paired_stems'][frame-1]
            yaml_paths=[scene/str(n)/f'{stem}.yaml' for n in [1,2]]
            bin_paths=[scene/str(n)/f'{stem}.bin' for n in [1,2]]
            for path in yaml_paths+bin_paths:
                relative=str(path.relative_to(ROOT));content=path.read_bytes()
                digest=hashlib.sha256(content).hexdigest()
                assert digest==freeze['raw_file_sha256'][relative]==raw_lookup[relative]['sha256']
                assert zlib.crc32(content)&0xffffffff==raw_lookup[relative]['crc32']
                inputs[relative]=digest
            raw=[adapter.read_yaml(path) for path in yaml_paths]
            transforms=adapter.relative_poses(raw)
            clouds=[project(adapter.read_points(path)[:,:3].astype(float),transform) for path,transform in zip(bin_paths,transforms)]
            a,b=[grid(p,-1.5,3.) for p in clouds]
            correlation=fftconvolve(a.astype(float),b[::-1,::-1].astype(float),mode='full')
            assert np.max(np.abs(correlation-np.rint(correlation)))<1e-8
            correlation=np.rint(correlation)
            center=np.array(b.shape)-1
            region=correlation[center[0]-20:center[0]+21,center[1]-20:center[1]+21]
            native=int((a&b).sum());assert correlation[tuple(center)]==native
            maximum=int(region.max());shifts=np.argwhere(region==maximum)-20
            # Direct overlap at every maximizing shift verifies FFT orientation.
            for sx,sy in shifts:
                ax=slice(max(0,sx),min(a.shape[0],a.shape[0]+sx))
                ay=slice(max(0,sy),min(a.shape[1],a.shape[1]+sy))
                bx=slice(max(0,-sx),min(b.shape[0],b.shape[0]-sx))
                by=slice(max(0,-sy),min(b.shape[1],b.shape[1]-sy))
                assert int((a[ax,ay]&b[bx,by]).sum())==maximum
            row=dict(sequence=name,frame=frame,native_overlap_voxels=native,maximum_overlap_voxels=maximum,
                     maximizing_source2_translation_m=(shifts*.5).tolist(),overlap_gain_ratio=maximum/native,
                     source2_position=transforms[1][:3,3].tolist(),
                     lidar_equals_true_pose=all(np.array_equal(r['lidar_pose'],r['true_ego_pose']) for r in raw))
            rows.append(row);print('ALIGNMENT SEQUENCE',json.dumps(row),flush=True)
    for path in [OUT/'recheck_raw_geometry.py',Path(adapter.__file__),Path(adapter.old.__file__)]:
        inputs[str(path.relative_to(ROOT))]=sha(path)
    answer=dict(passed=True,rows=rows,inputs=inputs,source_sha256=sha(Path(__file__)),
                scope='Five predetermined evenly spaced frames in each scene, 0.5 m occupancy grid, common z in [-1.5,3) m, translation +/-10 m. Translation is a post-hoc alignment diagnostic; no poses or outputs are corrected.')
    path=OUT/'RAW_ALIGNMENT_SEQUENCE.json';assert not path.exists()
    path.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
