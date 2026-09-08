"""Report actual codec costs only after all native exits and exact parity checks."""
from pathlib import Path
import argparse
import json
from analyze_codec import sha, source_check

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PARENT = OUT.parent / 'icra_gaussian_evidence'
LOG = ROOT / 'RUN/ICRA_GAUSSIAN_ZERO_CODEC'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cohort', choices=['development', 'seen_transfer'], default='development')
    cohort = parser.parse_args().cohort
    source = source_check()
    data = json.loads((OUT / f'summary_{cohort}.json').read_text())
    frozen = json.loads((OUT / f'CODEC_FREEZE_{cohort}.json').read_text())
    original = json.loads((PARENT / f'summary_{cohort}.json').read_text())
    runtime = json.loads((OUT / f'runtime_{cohort}.json').read_text())
    n = 9 if cohort == 'development' else 25
    assert data['passed'] and data['sequences'] == len(runtime) == n
    assert data['reference_summary_sha256'] == frozen['reference_summary_sha256'] == sha(PARENT / f'summary_{cohort}.json')
    assert data['analyzer_sha256'] == sha(OUT / 'analyze_codec.py')
    assert sorted(r['sequence'] for r in runtime) == sorted(u['sequence'] for u in frozen['units'])
    for name, expected in {**frozen['source_and_evidence_sha256'], **data['inputs']}.items():
        assert sha(ROOT / name) == expected, name
    for r in runtime:
        assert r['returncode'] == 0 and r['completion_line'] and r['files'] == 2
        assert f"COMPLETED GAUSSIAN ZERO CODEC indices {r['index']}--{r['index']} cohort={cohort}" in (LOG / f"{cohort}_{r['sequence']:04d}.log").read_text()
    assert 'ALL ZERO CODEC STAGE RUNS COMPLETE' in (LOG / f'{cohort}_driver.log').read_text()
    assert 'ZERO CODEC COMPLETE EXACT AUDIT' in (LOG / f'{cohort}_audit.log').read_text()
    conditions = [('reliable', '可靠'), ('intermittent', '间歇')]
    keys = ['raw_bytes', 'delivered_raw_bytes', 'wire_bytes']
    aggregate = []
    lines = [f'# 零向量编码：{n} 段完整轨迹复现', '',
             '全部主方法轨迹、存在/空间诊断和精度指标与原 352 B 版本逐值一致。',
             '编码器的逐包断言保留每个 double 的 64 位，包括正负零；独立 Python',
             '复算实际标签、可变包长、投递和分片费用。以下费用来自真实编码复跑。', '',
             '## 原生费用', '', '表中为全部序列的总 MiB，包含原规则的控制开销。', '',
             '| 链路 | 版本 | 原始 MiB | 投递 MiB | 分片及控制 MiB |',
             '| --- | --- | --- | --- | --- |']
    for condition, label in conditions:
        rows = [r for r in data['rows'] if r['condition'] == condition]
        assert len(rows) == n
        new = {key: sum(r[key] for r in rows) for key in keys}
        old = {key: sum(r['original_'+key] for r in rows) for key in keys}
        counts = {key: sum(r[key] for r in rows) for key in ['objects', 'nonzero_vectors']}
        savings = {key: 1-new[key]/old[key] for key in keys}
        aggregate.append(dict(condition=condition, native_zero_codec=new, native_original=old, savings=savings, **counts))
        for title, values in [('原 352 B', old), ('精确零向量省略', new)]:
            lines.append(f"| {label} | {title} | {values['raw_bytes']/2**20:.3f} | {values['delivered_raw_bytes']/2**20:.3f} | {values['wire_bytes']/2**20:.3f} |")
        lines.append(f"| {label} | 节约 | {savings['raw_bytes']:.2%} | {savings['delivered_raw_bytes']:.2%} | {savings['wire_bytes']:.2%} |")
    lines += ['', '## 相对原参照的费用', '',
              '将同一主方法的新原生费用与原实验完整参照比较。开发期旧 No-age/ER',
              '携带未使用标量，其记录不等于原生 208 B 包；25 段阶段为原生包。', '',
              '| 链路 | 参照 | 原始字节差 | 投递字节差 | 分片总字节差 |',
              '| --- | --- | --- | --- | --- |']
    comparisons = []
    for row in aggregate:
        condition = row['condition']
        for arm in ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_score', 'marked_asymmetric']:
            reference = next(r for r in original['aggregate'] if r['condition'] == condition and r['arm'] == arm)
            values = {key: row['native_zero_codec'][key]/(n*reference[key]['mean'])-1 for key in keys}
            comparisons.append(dict(condition=condition, reference=arm, **values))
            lines.append(f"| {dict(conditions)[condition]} | {arm} | {values['raw_bytes']:+.2%} | {values['delivered_raw_bytes']:+.2%} | {values['wire_bytes']:+.2%} |")
    lines += ['', '## 验证与解释', '',
              f"{2*n} 个结果文件、{n} 个 MATLAB 进程全部正常退出，复算 {data['node_frames']} 个节点—帧。",
              '这是一组已保存轨迹的精确复现，不是新增独立样本，不能据此缩窄精度区间。',
              '复跑唯一变化为原生包编码和新增编码诊断。旧头部和 232 B 基础载荷透传；',
              '每对象用 2 B 标记，仅非零空间比值继续传 120 B。标记同时保存零的符号。',
              '因此单包长度为 32+234n+120k，不使用数值阈值或有损量化。', '',
              'JSON 本身不能证明原始正负零位模式，位级还原由原生编码器逐包断言',
              '和独立构造的混合正负零单位夹具验证。Python 检查数值、标签及字节关系。',
              '正式预检前的 NaN 夹具比较、标签记录类型与提前启动问题完整保存在',
              'PREFLIGHT_FIX.md 和 initial_preflight；失败预检未产生跟踪输出。', '',
              '原丢包轨迹和分片模型保持，尚未测量真实网络的时延或字节数对丢包率的',
              '影响。所有真实轨迹已经参与方法开发；不产生独立验证或第三方复现结论。', '']
    report = OUT / f'RESULTS_{cohort}_CN.md'
    report.write_text('\n'.join(lines))
    result = dict(passed=True, cohort=cohort, sequences=n, source_files=len(source), node_frames=data['node_frames'],
                  native_matlab_exits=[r['returncode'] for r in runtime], aggregate=aggregate,
                  original_reference_byte_comparisons=comparisons, exact_whole_trajectory_parity=True,
                  summary_sha256=sha(OUT / f'summary_{cohort}.json'), stage_sha256=sha(OUT / f'CODEC_FREEZE_{cohort}.json'),
                  source_manifest_sha256=sha(OUT / 'source_sha256.json'), report_sha256=sha(report),
                  verifier_sha256=sha(Path(__file__)), unit_exit_observed=0, unit_exec_session=98989,
                  preflight_exit_observed=0, preflight_exec_session=47935,
                  preflight_auditor_exit_observed=0, preflight_auditor_exec_session=76273)
    (OUT / f'CURRENT_QA_{cohort}.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('ZERO CODEC NATIVE COSTS AND FULL PARITY VERIFIED:', cohort, data['node_frames'], 'node-frames.', flush=True)
    for r in aggregate:
        print(r, flush=True)


if __name__ == '__main__':
    main()
