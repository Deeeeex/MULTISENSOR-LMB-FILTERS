"""Build the complete marked-development report from the immutable audit."""
from pathlib import Path
import json
import numpy as np

OUT = Path(__file__).resolve().parent
NAMES = {
    'marked_lineage': 'M-No-age', 'marked_er': 'M-ER',
    'marked_ceiling_association': 'M-ECR-A', 'marked_ceiling_score': 'M-ECR-S',
    'marked_ceiling_calibrated': 'M-ECR-C', 'lineage': 'No-age',
    'qualified_exist': 'ER', 'ceiling_score': 'ECR-S', 'ceiling_calibrated': 'ECR-C',
}


def main():
    result = json.loads((OUT / 'summary_development.json').read_text())
    manifest = json.loads((OUT / 'likelihood_manifest.json').read_text())
    assert result['audited_node_frames'] == 39860 and result['sequences'] == 9
    lines = ['# 同信息检测分数对照：完整九序列开发结果', '',
             '共享标记似然更新后，M-ECR-S 相对 M-No-age 的平均收益缩小到约 1–2%。',
             '可靠链路的配对区间仍跨零；本轮只能选择后续验证方法，不能单独支撑新的论文主结论。',
             '全部九序列与两种链路均保留，没有按结果选择子集。', '',
             '## 观测模型及同信息比较', '',
             '原始检测分数 s 通过此前冻结的逐序列交叉拟合模型 p(s)=sigmoid(a logit(s)+b)',
             '得到经验目标性概率。每一折只使用另外八个序列的同权平均正例比例 pi，',
             '定义似然比 ell(s)=odds(p(s))/odds(pi)。每个检测关联列仅乘一次 ell(s)，',
             '漏检分支不变；归一化后的测量条件高斯分量不重复乘分数。', '',
             '这来自 Bayes odds 恒等式；把幅度或分数用于 LMB 关联已有先例。',
             '本控制不宣称该观测模型是新方法。分数与位置条件独立、跨场景分数稳定和',
             '标注所代表的检测类别都是近似假设。校准标签是裁剪区域内 12 m 的二维一对一',
             '指派结果，不是原数据集三维检测标签。名义 p_D、杂波强度、出生规则及动态模型未调。', '',
             '五个新方法共享相同的局部标记似然更新：M-No-age、M-ER、M-ECR-A/S/C。',
             'A/S/C 只改变正时效增益上限所用的支持量，分别使用 1、原分数、交叉拟合概率。',
             '最终存在概率仍为 r=min(rER,max(r0,c))；c 是经验支持量，不是统计置信上界。',
             '单次融合在相同输入下保持空间池不变；闭环中存在概率改变仍会影响后续关联与轨迹。', '',
             '## 全部开发结果', '',
             'OSPA 为各序列等权均值，单位 m；漏检和虚假项为 GOSPA 的平方代价 m²。', '',
             '| 链路 | 方法 | OSPA ↓ | 漏检代价 ↓ | 虚假代价 ↓ |',
             '| --- | --- | --- | --- | --- |']
    for row in result['aggregate']:
        if row['arm'] in NAMES:
            lines.append(f"| {'可靠' if row['condition']=='reliable' else '间歇'} | {NAMES[row['arm']]} | {row['ospa']['mean']:.6f} | {row['miss2']['mean']:.5f} | {row['false2']['mean']:.5f} |")
    lines += ['', '相同标记似然基线是主要开发参照。候选减参照，负值为改善；以序列为',
              '单位的 10000 次 bootstrap，种子 8301，95% 百分位区间，未做多重比较校正。', '',
              '| 链路 | 候选 | 参照 | OSPA 差 [95% 区间] | 胜出序列 |',
              '| --- | --- | --- | --- | --- |']
    for row in result['paired']:
        if row['candidate'] not in ['marked_ceiling_association', 'marked_ceiling_score', 'marked_ceiling_calibrated']:
            continue
        if row['reference'] not in ['marked_lineage', 'marked_er']:
            continue
        v = row['ospa']
        lines.append(f"| {'可靠' if row['condition']=='reliable' else '间歇'} | {NAMES[row['candidate']]} | {NAMES[row['reference']]} | {v['mean']:+.6f} [{v['low']:+.6f}, {v['high']:+.6f}] | {row['ospa_wins']}/9 |")
    lines += ['', '使用双方共同匹配到的真值实例检查定位，避免比较各自幸存目标导致支持集偏差。', '',
              '| 链路 | 候选 | 参照 | 共同实例 | 候选 RMSE m | 参照 RMSE m |',
              '| --- | --- | --- | --- | --- | --- |']
    for condition in ['reliable', 'intermittent']:
        for reference in ['marked_lineage', 'marked_er']:
            group = [r for r in result['common_target'] if r['condition'] == condition and
                     r['candidate'] == 'marked_ceiling_score' and r['reference'] == reference]
            support = sum(r['support'] for r in group)
            a = np.sqrt(sum(r['candidate_sse'] for r in group) / support)
            b = np.sqrt(sum(r['reference_sse'] for r in group) / support)
            lines.append(f"| {'可靠' if condition=='reliable' else '间歇'} | M-ECR-S | {NAMES[reference]} | {support} | {a:.6f} | {b:.6f} |")
    lines += ['', '## 核验与运行记录', '',
              f"独立重算全部 {result['audited_node_frames']} 个节点—帧，并验证 {result['source_hashes_verified']} 个源文件哈希。",
              '检查包括每折分数似然映射、真值/坐标/无线输入一致性、逐帧 OSPA 与计数、',
              '解析存在概率、正负时效修正、同输入反事实及共同定位支持。',
              '分析器可复跑到新的输出目录；不要覆盖已用于后续方法冻结的摘要文件。', '',
              '最初两次 MATLAB 进程发生原生崩溃，其中一次管道表面返回 0，另一次直接记录到',
              '子进程返回 -9。二者都缺少完整结束标志，因此均未记作完整实验。保存原日志和',
              '三个已完成文件，随后采用保持原始诊断结构的等价局部更新适配器，并内联同一融合规则。',
              '原生崩溃根因未证实。新旧适配器的固定输入状态、关联权重及全部五种融合记录逐位一致；',
              f"稳定运行与首轮完成输出的 {result['first_attempt_exact_output_parity_node_frames']} 个节点—帧逐位一致（仅运行时间不比较）。",
              '稳定版本随后完整完成九序列，主摘要仅使用这套完整输出。', '',
              '本开发轮所有五个新方法为了诊断携带一个额外支持量，包长为 216 B/object + 32 B；',
              'M-No-age/M-ER 实际并不需要该字段。不能据此宣称相对它们没有通信开销。',
              '后续共同入口已在已见序列核验原生 208 B 控制包与 216 B 候选包，输出状态保持一致。', '',
              '## 后续固定验证', '',
              '按完整开发队列，两种链路均值最低的 M-ECR-S 被固定为后续主方法。',
              '全部 25 个此前未用于融合方法选择的训练序列、5601 帧和 16 个同输入比较臂',
              '都在首次回放前登记，见相邻 icra_fusion_holdout/METHOD_FREEZE.json。',
              '冻结的全开发集正例比例为 ' + f"{manifest['full_seen_positive_prior']:.12f}" + '。',
              '公开检测器在该训练划分训练过，预留的是融合选择结果，不是独立检测器测试集。',
              '只有完整验证与同信息参照的配对证据足够强，才据此修改论文主结论。', '',
              '```sh',
              './tmp/external_baselines/venv/bin/python trials/icra_marked_iteration/analyze_marked.py --stable --output-dir tmp/marked_reaudit',
              '```', '']
    (OUT / 'README_CN.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()
