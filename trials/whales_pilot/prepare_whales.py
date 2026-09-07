"""Convert official metadata to auditable 2-D inputs; no tracking is run."""
from pathlib import Path
import collections
import hashlib
import itertools
import json
import pickle
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tmp/whales_deps'))
import numpy as np
from scipy.io import savemat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent


class NumpyOnlyUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        allowed = {('numpy.core.multiarray', '_reconstruct'),
                   ('numpy', 'ndarray'), ('numpy', 'dtype'), ('_codecs', 'encode')}
        if (module, name) not in allowed:
            raise pickle.UnpicklingError(f'Unexpected global: {module}.{name}')
        return super().find_class(module, name)


def partitions(items):
    if not items:
        yield []
        return
    first = items[0]
    for pair in itertools.combinations(items[1:], 2):
        group = (first,) + pair
        for rest in partitions(tuple(i for i in items if i not in group)):
            yield [group] + rest


def main():
    source = ROOT / 'tmp/whales/whales_infos_val.pkl'
    with source.open('rb') as f:
        data = NumpyOnlyUnpickler(f).load()
    scenes = collections.defaultdict(list)
    for entry in data['infos']:
        scenes[entry['sample_info']['scene_token']].append(entry)
    candidates = []
    inventory = []
    for scene, entries in sorted(scenes.items()):
        frames = sorted({e['sample_info']['frame'] for e in entries})
        vehicles = sorted({e['sample_info']['vehicle_index'] for e in entries
                           if e['veh_or_rsu'] == 'vehicle'})
        inventory.append(dict(scene=scene, frames=len(frames), vehicles=len(vehicles)))
        if len(vehicles) >= 9 and len(frames) >= 80:
            candidates.append(scene)
    assert candidates
    scene = candidates[0]
    assert scene == '2024-02-26-07-10-14'
    entries = scenes[scene]
    base = sorted([e for e in entries if e['sample_info']['vehicle_index'] == 0],
                  key=lambda e: e['timestamp'])
    timestamps = np.array([e['timestamp'] for e in base])
    assert len(base) == 99 and np.allclose(np.diff(timestamps), 0.5)
    original_vehicle_count = base[0]['sample_info']['vehicle_num']
    sensor_xy = np.array([e['sample_info']['veh_locations'][:9] for e in base])[:, :, :2]
    sensor_v = np.array([e['sample_info']['veh_speeds'][:9] for e in base])[:, :, :2]
    yaw = np.deg2rad(np.array([e['sample_info']['veh_rotations'][:9] for e in base])[:, :, 1])
    target_ids = sorted(a['index'] for a in base[0]['sample_info']['sample_annotation']
                        if a['index'] >= original_vehicle_count)
    target_xy, target_v = [], []
    identity_checks = []
    for e in base:
        si = e['sample_info']
        assert si['vehicle_num'] == original_vehicle_count
        ann = {a['index']: a for a in si['sample_annotation']}
        assert len(ann) == len(si['sample_annotation'])
        assert all(i in ann for i in target_ids)
        target_xy.append([ann[i]['location'][:2] for i in target_ids])
        target_v.append([np.asarray(ann[i]['velocity'])[:2] for i in target_ids])
        identity_checks.append(sorted(ann))
    assert all(ids == identity_checks[0] for ids in identity_checks)
    target_xy, target_v = np.array(target_xy), np.array(target_v)
    assert np.isfinite(sensor_xy).all() and np.isfinite(target_xy).all()
    assert np.isfinite(sensor_v).all() and np.isfinite(target_v).all()
    by_frame = collections.defaultdict(list)
    for e in entries:
        by_frame[e['sample_info']['frame']].append(e)
    for e in base:
        peers = by_frame[e['sample_info']['frame']]
        assert len(peers) == original_vehicle_count + 1
        assert all(p['timestamp'] == e['timestamp'] for p in peers)
        assert all(np.allclose(p['sample_info']['veh_locations'],
                               e['sample_info']['veh_locations']) for p in peers)
    # Check source slot interpretation against corresponding object centers.
    for e in base:
        si = e['sample_info']
        ann = {a['index']: a for a in si['sample_annotation']}
        for i in range(9):
            assert np.linalg.norm(np.array(ann[i]['location'])[:2] -
                                  np.array(si['veh_locations'][i])[:2]) < 1.0
    initial = sensor_xy[0]
    def cost(groups):
        return sum(float(np.sum((initial[a]-initial[b])**2))
                   for g in groups for a,b in itertools.combinations(g,2))
    groups = min(partitions(tuple(range(9))), key=lambda g: (cost(g),g))
    order = [i for g in groups for i in g]
    sensor_xy, sensor_v, yaw = sensor_xy[:, order], sensor_v[:, order], yaw[:, order]
    distance = np.linalg.norm(sensor_xy[:, :, None] - sensor_xy[:, None, :], axis=-1)
    within_max = np.array([max(distance[t, 3*g:3*g+3, 3*g:3*g+3].max()
                              for g in range(3)) for t in range(len(base))])
    # With three members, a distance-symmetric directed cycle needs all pairs.
    local_ok = within_max <= 100.0
    output = dict(scene=scene, timestamps=timestamps.tolist(),
                  sensor_slots=order, target_ids=target_ids,
                  sensor_xy=sensor_xy.tolist(), sensor_v=sensor_v.tolist(),
                  sensor_yaw=yaw.tolist(), target_xy=target_xy.tolist(),
                  target_v=target_v.tolist(), groups_original_slots=groups,
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  archive_sha256=hashlib.sha256((ROOT/'tmp/whales/whales_meta.tar.zst').read_bytes()).hexdigest(),
                  inventory=inventory, local_cycle_feasible_frames=int(local_ok.sum()),
                  synchronization_checked=True, finite_states_checked=True,
                  constant_annotation_indices_checked=True,
                  sensor_slot_annotation_correspondence_checked=True,
                  maximum_within_group_distance_m=within_max.tolist())
    (OUT/'converted_scene.json').write_text(json.dumps(output,indent=2),encoding='utf8')
    savemat(OUT/'converted_scene.mat',dict(
        positions=np.transpose(sensor_xy,(2,1,0)),
        velocities=np.transpose(sensor_v,(2,1,0)),
        headings=yaw.T, timestamps=timestamps,
        targetPositions=np.transpose(target_xy,(2,1,0)),
        targetVelocities=np.transpose(target_v,(2,1,0)),
        sensorSlots=np.array(order), targetIds=np.array(target_ids)))
    fig, axes = plt.subplots(1,2,figsize=(12,5),layout='constrained')
    colors=['#2563eb','#ea580c','#16a34a']
    for i in range(9):
        xy=sensor_xy[:,i]
        axes[0].plot(xy[:,0],xy[:,1],color=colors[i//3],lw=1.5)
        axes[0].scatter(*xy[0],color=colors[i//3],marker='s',s=35)
        axes[0].annotate(str(order[i]),xy[0],xytext=(4,4),textcoords='offset points',fontsize=8)
    for i in range(len(target_ids)):
        axes[0].plot(target_xy[:,i,0],target_xy[:,i,1],color='#777777',alpha=.55,lw=1,ls='--')
    axes[0].set(aspect='equal',xlabel='World x (m)',ylabel='World y (m)',
                title='Recorded vehicle paths; fixed initial groups')
    for g in range(3):
        diam=distance[:,3*g:3*g+3,3*g:3*g+3].max(axis=(1,2))
        axes[1].plot(timestamps,diam,color=colors[g],label=f'Group {g+1}')
    axes[1].axhline(100,color='black',ls='--',label='Assumed radio range: 100 m')
    axes[1].set(xlabel='Dataset time (s)',ylabel='Group diameter (m)',
                title='Fixed-group cycle applicability')
    axes[1].legend(fontsize=8)
    for ax in axes: ax.grid(alpha=.2)
    fig.savefig(OUT/'scene_preflight.png',dpi=180)
    fig.savefig(OUT/'scene_preflight.pdf')
    print(json.dumps({k:output[k] for k in ['scene','groups_original_slots','target_ids',
                       'local_cycle_feasible_frames','source_sha256']},indent=2))
    print('First local-cycle failure (1-based frame):',np.flatnonzero(~local_ok)[:1]+1)


if __name__ == '__main__':
    main()
