"""Check input conversions against the pinned source and raw paired labels."""
from datetime import datetime, timezone
import ast
import json

from input_adapter import *


def main():
    destination = OUT / 'ADAPTER_CHECK.json'; assert not destination.exists()
    author_path = AUTHOR / 'opencood/utils/transformation_utils.py'
    tree = ast.parse(author_path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'x_to_world')
    namespace = {'np': np}
    # Execute only the reviewed pure NumPy pose function, without module imports.
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(author_path), 'exec'), namespace)
    cohort = json.loads((OUT / 'COHORT_FREEZE.json').read_text())
    raw_manifest = json.loads((OUT / 'RAW_INPUT_MANIFEST.json').read_text())
    raw_hashes = {r['path']: r['sha256'] for r in raw_manifest['files']}
    rows = []; error = 0.; total_points = 0; shared_distances = []
    for sequence in cohort['sequences']:
        scene = ROOT / sequence['scene_path']; labels_total = 0
        for stem in sequence['paired_stems']:
            raw = []
            for agent in [1, 2]:
                yaml_path = scene / str(agent) / f'{stem}.yaml'
                bin_path = scene / str(agent) / f'{stem}.bin'
                for path in [yaml_path, bin_path]:
                    assert sha(path) == raw_hashes[str(path.relative_to(ROOT))]
                item = read_yaml(yaml_path); raw.append(item)
                assert item['infra'] is False
                matrix = pose_matrix(item['lidar_pose'])
                author_matrix = namespace['x_to_world'](item['lidar_pose'])
                error = max(error, float(np.max(np.abs(matrix - author_matrix))))
                assert error < 1e-14
                assert np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), rtol=0, atol=1e-14)
                points = read_points(bin_path); total_points += len(points)
                assert np.all(points[:, 3] == 0)
            transforms = relative_poses(raw)
            assert np.allclose(transforms[0], np.eye(4), rtol=0, atol=1e-12)
            back = np.linalg.inv(pose_matrix(raw[1]['lidar_pose'])) @ pose_matrix(raw[0]['lidar_pose'])
            assert np.allclose(transforms[1] @ back, np.eye(4), rtol=0, atol=1e-12)
            for label in raw[0]['vehicles'].keys() & raw[1]['vehicles'].keys():
                one, two = raw[0]['vehicles'][label], raw[1]['vehicles'][label]
                if one['obj_type'] in VEHICLE_TYPES:
                    assert one['obj_type'] == two['obj_type']
                    a = np.asarray(one['location']) + one['center']
                    b = np.asarray(two['location']) + two['center']
                    shared_distances.append(float(np.linalg.norm(a - b)))
            boxes, labels = labels_for_pair(raw, transforms)
            assert len(set(labels)) == len(labels) and np.isfinite(boxes).all()
            labels_total += len(labels)
        rows.append(dict(sequence=sequence['sequence'], frames=sequence['frames'], vehicle_label_rows=labels_total))
        print('V2X ADAPTER CHECK', sequence['sequence'], sequence['frames'], 'paired frames', flush=True)
    report = dict(passed=True, completed_utc=datetime.now(timezone.utc).isoformat(), sequences=rows,
                  point_rows_read=total_points, max_pose_difference_from_author=error,
                  author_pose_sha256=sha(author_path), adapter_sha256=sha(OUT / 'input_adapter.py'),
                  checker_sha256=sha(Path(__file__)), raw_manifest_sha256=sha(OUT / 'RAW_INPUT_MANIFEST.json'),
                  shared_annotation_centers=dict(count=len(shared_distances), median_m=float(np.median(shared_distances)),
                      maximum_m=float(np.max(shared_distances)), duplicate_rule='First occurrence, agent 1 before agent 2, as in author BasePostprocessor.generate_gt_bbx.'),
                  checks='All input hashes; official pose parity; rotations and relative inverses; binary shape, finite xyz and zero intensity; shared object center discrepancies recorded with class agreement; unique projected IDs; no tracking outcomes.')
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('V2X ADAPTER PASSED', sum(r['frames'] for r in rows), 'paired frames', flush=True)


if __name__ == '__main__':
    main()
