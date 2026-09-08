"""Compare all declared components on complete native trajectory pairs."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PARENT = OUT.parent / 'icra_gaussian_evidence'
sys.path.insert(0, str(OUT.parent / 'icra_marked_control'))
from analyze_control import METRICS, interval, read, score_run

PRIMARY = 'marked_gaussian_evidence'
REFERENCES = [PRIMARY + suffix for suffix in ['_no_curvature', '_no_history', '_no_mark']] + ['marked_asymmetric']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    target = OUT / f'summary_{cohort}.json'
    assert not target.exists(), 'Preserve the previous analysis.'
    summary = json.loads((PARENT / f'summary_{cohort}.json').read_text())
    qa = json.loads((PARENT / f'CURRENT_QA_{cohort}.json').read_text())
    assert qa['passed'] and qa['summary_sha256'] == sha(PARENT / f'summary_{cohort}.json')
    assert all(code == 0 for code in qa['native_matlab_stage_exits'])
    frozen = json.loads((PARENT / 'ROUND_FREEZE.json').read_text())
    names = [f"{u['sequence']:04d}" for u in frozen['units'] if u['cohort'] == cohort]
    assert len(names) == summary['sequences'] == (9 if cohort == 'development' else 25)
    assert REFERENCES == frozen['arms'][1:] + frozen['component_controls']
    lookup = {(r['sequence'], r['condition'], r['arm']): r for r in summary['runs']}
    rows, inputs = [], {}
    node_frames = 0
    for name in names:
        for condition in ['reliable', 'intermittent']:
            scores, matched = {}, {}
            original = None
            for arm in [PRIMARY] + REFERENCES:
                folder = (OUT.parent / 'icra_asymmetric_evidence') if cohort == 'development' and arm == 'marked_asymmetric' else PARENT
                path = folder / f'results_{cohort}' / f'{name}_{condition}_{arm}.json.gz'
                key = str(path.relative_to(ROOT))
                assert sha(path) == summary['inputs'][key], key
                inputs[key] = sha(path)
                data = read(path)
                if original is None:
                    original = data
                else:
                    for field in ['time', 'positions', 'truth', 'truthIds', 'delivered']:
                        assert data[field] == original[field], (key, field)
                score, matches = score_run(data, data['runs'])
                for metric in METRICS:
                    assert np.isclose(score[metric], lookup[name, condition, arm][metric], atol=1e-9, rtol=1e-12), (key, metric)
                scores[arm], matched[arm] = score, matches
                node_frames += 2 * len(data['time'])
            for reference in REFERENCES:
                both = np.isfinite(matched[PRIMARY]) & np.isfinite(matched[reference])
                support = int(both.sum())
                assert support > 0, (name, condition, reference)
                left = float(matched[PRIMARY][both].sum())
                right = float(matched[reference][both].sum())
                rows.append(dict(sequence=name, condition=condition, candidate=PRIMARY, reference=reference,
                    **{metric: scores[PRIMARY][metric] - scores[reference][metric] for metric in METRICS},
                    support=support, candidate_sse=left, reference_sse=right,
                    candidate_rmse=float(np.sqrt(left / support)), reference_rmse=float(np.sqrt(right / support))))
            print('FULL COMPONENT TRAJECTORIES RESCORED', cohort, name, condition, flush=True)
    samples = np.random.default_rng(8301).integers(0, len(names), (10000, len(names)))
    comparisons = []
    for condition in ['reliable', 'intermittent']:
        for reference in REFERENCES:
            group = [r for r in rows if r['condition'] == condition and r['reference'] == reference]
            assert len(group) == len(names)
            support = sum(r['support'] for r in group)
            comparisons.append(dict(condition=condition, candidate=PRIMARY, reference=reference,
                **{metric: interval([r[metric] for r in group], samples) for metric in METRICS},
                ospa_wins=sum(r['ospa'] < -1e-10 for r in group),
                common_support=support,
                candidate_common_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group) / support)),
                reference_common_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group) / support)),
                sequence_common_rmse_difference=interval([r['candidate_rmse'] - r['reference_rmse'] for r in group], samples)))
    result = dict(cohort=cohort, sequences=len(names), comparisons=comparisons, rows=rows, inputs=inputs,
        independently_rescored_node_frames=node_frames, original_packet_costs=True,
        source_summary_sha256=sha(PARENT / f'summary_{cohort}.json'),
        original_qa_sha256=sha(PARENT / f'CURRENT_QA_{cohort}.json'),
        analyzer_sha256=sha(Path(__file__)), protocol_sha256=sha(OUT / 'PROTOCOL.md'),
        interpretation='Full closed-loop pairs on previously seen real data; common support depends on the reference. Descriptive sequence bootstrap only, with no independent samples or new method selection.')
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (OUT / f'paired_sequences_{cohort}.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = [f'# 完整轨迹组件比较：{len(names)} 段', '',
        '主方法减组件对照，负差值有利。所有真实序列均已见；95% 区间为',
        '10000 次序列重采样的描述性区间。共同真值定位支持量随对照改变。', '',
        '| 链路 | 对照 | ΔOSPA [95% 区间] | 胜出段数 | 共同真值 | 主方法 RMSE | 对照 RMSE |',
        '| --- | --- | --- | --- | --- | --- | --- |']
    for r in comparisons:
        v = r['ospa']
        lines.append(f"| {r['condition']} | {r['reference']} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['ospa_wins']}/{len(names)} | {r['common_support']} | {r['candidate_common_rmse']:.6f} | {r['reference_common_rmse']:.6f} |")
    lines += ['', '表中的 M-AE 是完整重新递推的标量对照。逐序列差值和共同匹配误差',
              '保存在同目录 CSV/JSON；区间包含零时，不声称该组件的精度贡献已证实。',
              '字节差来自原始 352 B 高斯与 232 B 标量包；压缩费用另见原生编码复跑。', '']
    (OUT / f'RESULTS_{cohort}_CN.md').write_text('\n'.join(lines))
    print('FULL COMPONENT COMPARISONS COMPLETE', cohort, len(rows), 'pairs;', node_frames, 'rescored node-frames.', flush=True)


if __name__ == '__main__':
    main()
