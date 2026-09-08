"""Record the complete negative-evidence diagnostic and its limited conclusion."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def main():
    d = json.loads((OUT / 'summary_negative.json').read_text())
    for name, expected in d['input_sha256'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    lines = ['# 负向本帧证据的固定输入诊断', '',
             '全部九序列、两种链路、两套已保存历史上的负证据规则均已完成。',
             '额外负证据降低误报，但增加漏检；OSPA 均值略有改善，所有新增负证据',
             '规则的配对区间仍包含零。这个诊断不证明完整递推会改善。', '',
             '下一步值得检验的是“当前可见且漏检分支支持”的负证据，原因是它对应',
             '明确的非观测似然；不是因为某个序列最好。后续若执行完整递推，应预先',
             '固定该规则、保留去掉它及其他组件的消融，并完成全部序列。', '',
             'negative_all 放行全部当前负增量；negative_miss 按漏检关联质量和名义',
             '漏检似然的对比量加权，系数为 (1−检测关联质量)×0.9/1.1。实际遮挡',
             '没有被名义可见性模型完全捕捉，漏检增加是必须保留的风险。', '',
             '| 链路 | 固定输入历史 | 标量规则 | ΔOSPA [95% 区间] m | Δ漏检 m² | Δ误报 m² |',
             '| --- | --- | --- | --- | --- | --- |']
    for r in d['paired']:
        v = r['ospa']
        lines.append(f"| {r['condition']} | {r['backend']} | {r['rule']} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {r['miss2']['mean']:+.6f} | {r['false2']['mean']:+.6f} |")
    lines += ['', '差值相对保持该历史的原始规则。区间为按九个序列重采样的 10000 次',
              '描述性百分位区间。原规则的所有节点—帧重新评分均与原结果一致。',
              '两套历史各自包含不发生融合的帧，这些帧保持原输出；发生融合时保持',
              '空间位置和候选标签，按替换后的存在值重新做原 MAP 基数提取。', '',
              f"核对 {d['original_node_frames']} 个原节点—帧，计算 {d['rule_node_frames']} 个规则节点—帧；",
              '完整均值、区间、所有逐序列结果及输入散列保存在 summary_negative.json。',
              '没有重跑跟踪器、调参、修改原轨迹或更新论文。', '']
    (OUT / 'README_CN.md').write_text('\n'.join(lines))
    print('Full negative-evidence diagnostic report written; all source and result hashes verified.')


if __name__ == '__main__':
    main()
