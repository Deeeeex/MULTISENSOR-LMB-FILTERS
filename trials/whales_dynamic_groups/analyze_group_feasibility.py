"""Current-geometry feasibility audit, not a tracking or distributed protocol.

Enumerate every unlabeled partition of nine sensors into three triples.
Use the original 100 m radio assumption and all 99 frames without retuning.
Only sensor coordinates enter decisions; target states are never read.
"""
from pathlib import Path
import collections
import hashlib
import itertools
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tmp/whales_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent


def partitions(items):
    if not items:
        yield ()
        return
    for pair in itertools.combinations(items[1:], 2):
        group = (items[0],) + pair
        for rest in partitions(tuple(i for i in items if i not in group)):
            yield (group,) + rest


def components(adjacency):
    unseen = set(range(len(adjacency)))
    groups = []
    while unseen:
        todo = [min(unseen)]
        group = set(todo)
        unseen.difference_update(group)
        while todo:
            neighbors = set(np.flatnonzero(adjacency[todo.pop()])) & unseen
            unseen.difference_update(neighbors)
            group.update(neighbors)
            todo.extend(sorted(neighbors))
        groups.append(tuple(sorted(group)))
    return tuple(groups)


def architecture_feasible(groups, adjacency):
    if not all(adjacency[a, b] for g in groups
               for a, b in itertools.combinations(g, 2)):
        return False
    # Every tree on three groups is a star. Its center must have distinct
    # receiving nodes for its two neighbors (the V240 gateway contract).
    for center in range(3):
        left, right = [i for i in range(3) if i != center]
        a = [i for i in groups[center] if any(adjacency[i, j] for j in groups[left])]
        b = [i for i in groups[center] if any(adjacency[i, j] for j in groups[right])]
        if any(i != j for i in a for j in b):
            return True
    return False


def align_groups(groups, previous):
    # Relabeling a whole group is free: minimize actual member moves.
    candidates = [(9 - sum(len(set(g) & set(p)) for g, p in zip(order, previous)), order)
                  for order in itertools.permutations(groups)]
    return min(candidates)


def forest(adjacency, distances):
    # Deterministic minimum-distance spanning forest; undirected links are
    # implemented with two directed posterior transmissions.
    parent = list(range(len(adjacency)))
    def find(i):
        while parent[i] != i:
            i = parent[i]
        return i
    result = np.zeros_like(adjacency)
    edges = sorted((float(distances[i, j]), i, j)
                   for i, j in itertools.combinations(range(len(adjacency)), 2)
                   if adjacency[i, j])
    for _, i, j in edges:
        a, b = find(i), find(j)
        if a != b:
            parent[a] = b
            result[i, j] = result[j, i] = True
    return result


def main():
    source = ROOT / 'trials/whales_pilot/converted_scene.json'
    data = json.loads(source.read_text(encoding='utf8'))
    xy = np.asarray(data['sensor_xy'])
    assert xy.shape == (99, 9, 2)
    all_groups = list(partitions(tuple(range(9))))
    assert len(all_groups) == 280 and len(set(all_groups)) == 280
    initial = ((0, 1, 2), (3, 4, 5), (6, 7, 8))
    incumbent = initial
    frames = []
    previous_components = None
    previous_forest = np.zeros((9, 9), dtype=bool)
    for t, p in enumerate(xy):
        distances = np.linalg.norm(p[:, None] - p[None, :], axis=-1)
        physical = (distances <= 100) & ~np.eye(9, dtype=bool)
        comp = components(physical)
        feasible = [g for g in all_groups if architecture_feasible(g, physical)]
        local_count = sum(all(physical[a, b] for group in g
                              for a, b in itertools.combinations(group, 2)) for g in all_groups)
        # Keep the previous grouping whenever feasible. Upon failure use only
        # present geometry, then minimal member changes, then compactness.
        selected = None
        moves = 0
        if feasible:
            candidates = []
            for g in feasible:
                changes, aligned = align_groups(g, incumbent)
                compactness = sum(distances[a, b]**2 for group in g
                                  for a, b in itertools.combinations(group, 2))
                candidates.append((changes, float(compactness), aligned))
            moves, _, selected = min(candidates)
            incumbent = selected
        selected_forest = forest(physical, distances)
        assert components(selected_forest) == comp
        assert not np.any(selected_forest & ~physical)
        assert int(selected_forest.sum()) == 2 * (9 - len(comp))
        low_degree = np.flatnonzero(physical.sum(axis=1) < 2).tolist()
        if feasible:
            assert len(comp) == 1 and not low_degree
        frames.append(dict(
            frame=t+1, time_s=data['timestamps'][t],
            components=[[data['sensor_slots'][i] for i in g] for g in comp],
            component_sizes=sorted([len(g) for g in comp], reverse=True),
            component_membership_changed=previous_components is not None and comp != previous_components,
            physical_connected=len(comp) == 1,
            fixed_feasible=architecture_feasible(initial, physical),
            local_triple_partitions=local_count,
            feasible_triple_partitions=len(feasible),
            selected_groups_by_input_index=selected,
            selected_groups_original_slots=None if selected is None else
                [[data['sensor_slots'][i] for i in g] for g in selected],
            member_moves=moves,
            nodes_degree_below_two_original_slots=[data['sensor_slots'][i] for i in low_degree],
            forest_directed_messages=int(selected_forest.sum()),
            forest_undirected_edge_edits=int(np.logical_xor(selected_forest, previous_forest).sum()//2)
                if t else 0))
        previous_components, previous_forest = comp, selected_forest
    baseline = json.loads((ROOT / 'trials/whales_pilot/routing_preflight.json').read_text())
    assert [f['fixed_feasible'] for f in frames] == baseline['sparse_v242']['passed']
    assert [f['physical_connected'] for f in frames] == baseline['physicalConnected']
    counts = collections.Counter(','.join(map(str, f['component_sizes'])) for f in frames)
    summary = dict(
        frame_count=len(frames), partitions_per_frame=len(all_groups),
        fixed_feasible_frames=sum(f['fixed_feasible'] for f in frames),
        dynamic_triples_feasible_frames=sum(f['feasible_triple_partitions'] > 0 for f in frames),
        physical_connected_frames=sum(f['physical_connected'] for f in frames),
        rescued_frames=[f['frame'] for f in frames if not f['fixed_feasible'] and f['feasible_triple_partitions']],
        connected_but_no_triple_partition_frames=[f['frame'] for f in frames
                                                 if f['physical_connected'] and not f['feasible_triple_partitions']],
        component_size_patterns=dict(counts),
        component_change_frames=[f['frame'] for f in frames if f['component_membership_changed']],
        regroup_events=[dict(frame=f['frame'], moves=f['member_moves'], groups=f['selected_groups_original_slots'])
                        for f in frames if f['member_moves']],
        member_moves_total=sum(f['member_moves'] for f in frames),
        forest_mean_directed_messages=float(np.mean([f['forest_directed_messages'] for f in frames])))
    result = dict(contract='whales-dynamic-groups-geometry-v1',
                  scene=data['scene'], radio_range_m=100,
                  input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  uses_target_states=False, uses_future_geometry=False,
                  centralized_geometry_audit=True, tracking_executed=False,
                  summary=summary, frames=frames)
    (OUT / 'feasibility.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, layout='constrained')
    times = data['timestamps']
    for key, label, color in [('fixed_feasible', 'Fixed triples', '#dc2626'),
                              ('feasible_triple_partitions', 'Any dynamic triples', '#2563eb'),
                              ('physical_connected', 'Physical graph connected', '#16a34a')]:
        axes[0].step(times, [bool(f[key]) for f in frames], where='post', label=label, color=color, alpha=.8)
    axes[0].set(yticks=[0, 1], yticklabels=['No', 'Yes'], ylabel='Global backbone feasible', ylim=(-.1, 1.3))
    axes[0].legend(loc='upper right')
    axes[1].step(times, [len(f['components']) for f in frames], where='post', color='#7c3aed')
    axes[1].set(xlabel='Dataset time (s)', ylabel='Physical components', yticks=[1, 2, 3, 4])
    fig.suptitle('WHALES: exact 3 x 3 regrouping audit at assumed 100 m radio range')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.savefig(OUT / 'feasibility.png', dpi=180)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
