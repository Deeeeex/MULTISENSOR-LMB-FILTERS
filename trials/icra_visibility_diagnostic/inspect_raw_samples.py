"""Inspect raw returns and annotations at one fixed true and false case."""
from pathlib import Path
import hashlib
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(OUT.parent / 'icra_v2x_transfer'))
import input_adapter as adapter
from diagnose_boxes import footprint

sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
inputs = {}


def bind(path, expected=None):
    digest = sha(path)
    assert expected is None or expected == digest, path
    inputs[str(path.relative_to(ROOT))] = digest


def count_in_box(points, box):
    rotation, half, _ = footprint(box)
    planar = (points[:, :2] - box[:2]) @ rotation
    return int(((np.abs(planar) <= half).all(axis=1) & (np.abs(points[:, 2] - box[2]) <= box[3]/2)).sum())


def main():
    source = OUT / 'BOX_VISIBILITY_DIAGNOSTIC.json'
    bind(source)
    diagnostic = json.loads(source.read_text())
    true = next(row for row in diagnostic['rows'] if row['sequence']=='v2xt_0001' and row['condition']=='reliable' and row['frame']==53)
    false = [row for row in diagnostic['rows'] if row['sequence']=='v2x_0002' and row['condition']=='reliable']
    assert all(row['label']==[33,200005] and row['source']==1 for row in false)
    false = false[len(false)//2]
    samples = [true, false]
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    records = []
    for row_index, sample in enumerate(samples):
        name, frame = sample['sequence'], sample['frame']
        folder = OUT.parent / ('icra_association_test' if name.startswith('v2xt') else 'icra_v2x_transfer')
        freeze_path = folder / 'INFERENCE_FREEZE.json'
        bind(freeze_path)
        freeze = json.loads(freeze_path.read_text())
        sequence = next(row for row in freeze['sequences'] if row['sequence']==name[-4:])
        scene = ROOT / sequence['scene_path']
        stem = sequence['paired_stems'][frame-1]
        paths = [scene / str(n) / f'{stem}.yaml' for n in [1, 2]]
        for path in paths:
            bind(path, freeze['raw_file_sha256'][str(path.relative_to(ROOT))])
        raw = [adapter.read_yaml(path) for path in paths]
        transforms = adapter.relative_poses(raw)
        world_to_ego = np.linalg.inv(adapter.pose_matrix(raw[0]['lidar_pose']))
        archive_path = folder / 'detections' / f'detections_{name[-4:]}.npz'
        bind(archive_path)
        archive = np.load(archive_path, allow_pickle=False)
        boxes = [archive[f'{frame-1:06d}_{n}_boxes'] for n in [0, 1]]
        scores = [archive[f'{frame-1:06d}_{n}_scores'] for n in [0, 1]]
        query = np.asarray(sample['predicted_xy'])
        target_box = None
        if name.startswith('v2xt'):
            gt_boxes, gt_ids = adapter.labels_for_pair(raw, transforms)
            assert np.count_nonzero(gt_ids==5)==1
            target_box = gt_boxes[gt_ids==5][0]
            box_source = 'Canonical first-source annotation ID 5, diagnostic only.'
        else:
            positive_source = 1
            nearest = np.argmin(np.sum((boxes[positive_source][:, :2]-query)**2, axis=1))
            target_box = boxes[positive_source][nearest]
            assert np.linalg.norm(target_box[:2]-query)<2
            box_source = 'Closest positive-source detector box, not an annotated target.'
        sample_record = dict(sequence=name, frame=frame, predicted_xy=query.tolist(),
                             diagnostic_box=target_box.tolist(), box_source=box_source, sources=[])
        for source_index in [0, 1]:
            point_path = scene / str(source_index+1) / f'{stem}.bin'
            bind(point_path, freeze['raw_file_sha256'][str(point_path.relative_to(ROOT))])
            raw_points = adapter.read_points(point_path)
            transform = transforms[source_index]
            points = raw_points[:, :3] @ transform[:3, :3].T + transform[:3, 3]
            prepared = adapter.old.prepare_points(raw_points.copy(), 0)
            prepared = prepared[:, :3] @ transform[:3, :3].T + transform[:3, 3]
            annotations = []
            for label, obj in raw[source_index]['vehicles'].items():
                center = (world_to_ego @ np.r_[np.asarray(obj['location'])+obj['center'], 1])[:3]
                annotations.append(dict(id=int(label), type=obj['obj_type'], xyz=center.tolist(),
                                        planar_distance=float(np.linalg.norm(center[:2]-query))))
            annotations.sort(key=lambda item:item['planar_distance'])
            nearest_box = int(np.argmin(np.linalg.norm(boxes[source_index][:,:2]-query, axis=1)))
            source_record = dict(source=source_index+1, source_xyz=transform[:3,3].tolist(),
                                 raw_points=len(points), prepared_points=len(prepared),
                                 raw_points_in_diagnostic_box=count_in_box(points,target_box),
                                 prepared_points_in_diagnostic_box=count_in_box(prepared,target_box),
                                 nearest_annotations=annotations[:5],
                                 closest_detection_distance=float(np.linalg.norm(boxes[source_index][nearest_box,:2]-query)),
                                 closest_detection_box=boxes[source_index][nearest_box].tolist(),
                                 closest_detection_score=float(scores[source_index][nearest_box]))
            sample_record['sources'].append(source_record)
            axis = axes[row_index, source_index]
            # Fixed spatial crop for display only; counts above use all points.
            if row_index == 0:
                xmin,xmax,ymin,ymax = -5, 48, -14, 23
            else:
                xmin,xmax,ymin,ymax = -13, 15, -10, 10
            display = points[(points[:,0]>=xmin)&(points[:,0]<=xmax)&(points[:,1]>=ymin)&(points[:,1]<=ymax)]
            axis.scatter(display[:,0],display[:,1],c=display[:,2],vmin=-2.5,vmax=.5,s=.25,cmap='Greys',alpha=.5,rasterized=True)
            for box in boxes[source_index]:
                axis.add_patch(Polygon(footprint(box)[2],closed=True,fill=False,edgecolor='#227a46',linewidth=.8))
            axis.add_patch(Polygon(footprint(target_box)[2],closed=True,fill=False,edgecolor='#ce3450',linewidth=2))
            origin=transform[:2,3]
            axis.plot([origin[0],query[0]],[origin[1],query[1]],color='#2058c0',linewidth=1)
            axis.scatter(*origin,marker='^',c='#2058c0',s=40)
            axis.scatter(*query,marker='x',c='#ce3450',s=55)
            axis.set(xlim=(xmin,xmax),ylim=(ymin,ymax),aspect='equal',xlabel='ego-1 x (m)',ylabel='ego-1 y (m)',
                     title=f'{name} t={frame}, source {source_index+1}: {source_record["prepared_points_in_diagnostic_box"]} returns in red box')
            axis.grid(alpha=.15)
        records.append(sample_record)
        print('RAW SAMPLE',json.dumps(sample_record),flush=True)
    fig.suptitle('Raw LiDAR and existing detections at the fixed diagnostic locations\nGreen: detector footprints; red: diagnostic box; blue: source-to-query ray',fontsize=13)
    figure_path=OUT/'raw_samples.png'
    assert not figure_path.exists()
    fig.savefig(figure_path,dpi=160)
    plt.close(fig)
    for path in [Path(adapter.__file__),Path(adapter.old.__file__),OUT/'diagnose_boxes.py']:
        bind(path)
    answer=dict(passed=True,samples=records,inputs=inputs,source_sha256=sha(Path(__file__)),figure_sha256=sha(figure_path),
                scope='Two declared samples; counts inside the stated diagnostic boxes are raw signal checks, not visibility labels or a method input.')
    destination=OUT/'RAW_SAMPLE_INSPECTION.json'
    assert not destination.exists()
    destination.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
