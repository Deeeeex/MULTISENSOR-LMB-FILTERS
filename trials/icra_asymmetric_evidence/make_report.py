"""Adapt the complete previous report to branch-specific evidence and controls."""
from pathlib import Path

OUT = Path(__file__).resolve().parent


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    s = (OUT.parent / 'icra_selective_innovation/build_report.py').read_text()
    s = s.replace("'marked_selective': 'M-SI（主）'", "'marked_selective': '关闭负证据 M-SI'")
    s = s.replace("'marked_selective_signed': '放行正负增量'", "'marked_selective_signed': '正负共用分数门控 SI'")
    s = replace(s, "'marked_joint_evidence': '前轮 M-JE'",
                "'marked_asymmetric': 'M-AE（主）', 'marked_asymmetric_no_history': '去除历史保守项',\n         'marked_asymmetric_no_mark': '去除分数约束', 'marked_joint_evidence': '前轮 M-JE'")
    s = s.replace("primary == 'marked_selective'", "primary == 'marked_asymmetric'")
    s = replace(s, "        for ref in f['arms'][1:]:",
                "        for ref in ['marked_asymmetric_no_history', 'marked_asymmetric_no_mark']+f['component_controls']:")
    begin, end = s.index("    lines = [f'# 选择性本帧增量"), s.index("             '## 全部同信息方法'", s.index('    lines ='))
    s = s[:begin]+"""    lines = [f'# 正负分支支持的本帧证据：{n} 个' + ('开发序列' if cohort == 'development' else '已见迁移序列'), '', decision, '',
             '前一轮正向选择性增量的可靠链路结果有改善，但未通过间歇链路门槛。',
             '本轮把正证据与负证据分开：检测分数支持正向增量，当前可见且漏检',
             '分支占比高时支持负向增量。全部真实序列已参与开发，不能称为未见验证。', '',
             '## 固定方法', '',
             'δ=logit(r后验)−logit(r预测)。正向支持 g+=sum W·max(0,(L−1)/(L+1))；',
             '负向支持 g−=(1−检测关联质量)·pD/(2−pD)。pD 是原模型本次计算的',
             '名义检测概率，当前可见时 0.9，否则为 0。分数似然比和所有关联步骤',
             '均沿用此前实现。g−的后半项是漏检似然 1−pD 的有界负向对比量。', '',
             '原时效修正为负时 β=q，否则 β=b；1e−12 对数几率容差处理数值相等。',
             '仅在至少两个合格来源都表示同一标签时，于保守继承项上加入',
             'sum(1−β)·[g+ max(δ,0)+g− min(δ,0)]。门控在无当前观测机会和融合后',
             '清零，并在下一次本地更新重算。空间融合、出生、匹配、提取等均保持。', '',
             '组件对照包括去除历史保守项、去除正向分数约束、关闭负证据的原 M-SI，',
             '以及正负增量都使用正向分数门控的前轮 signed SI。后两种在开发阶段',
             '复用既有完整输出，只有新主方法通过门槛后才在迁移阶段作为组件对照运行。',
             '门控是有界启发式；真实遮挡、相关检测及近似关联仍可能使负证据失真。', '',
"""+s[end:]
    s = replace(s, "for arm in f['primary_references']+f['additional_references']+f['arms']:",
                "for arm in list(dict.fromkeys(f['primary_references']+f['additional_references']+f['component_controls']+f['arms_by_cohort'][cohort])):")
    s = s.replace("['no_age', 'ER', 'CR', 'candidate']", "['no_age', 'ER', 'CR', 'no_negative', 'candidate']")
    s = replace(s, "f\"本阶段 {n} 序列、{d['frames']} 帧、{8*n} 个新文件均有成功的实际 MATLAB\"",
                "f\"本阶段 {n} 序列、{d['frames']} 帧、{2*n*len(f['arms_by_cohort'][cohort])} 个新文件均有成功的实际 MATLAB\"")
    s = s.replace('2352', '2352')
    s = s.replace('预检额外重算 2940 个节点—帧；其中 588 个 CR 节点—帧的轨迹、指标、',
                  '预检额外重算 2352 个节点—帧；其中 588 个 SI 节点—帧的轨迹、指标、')
    s = s.replace('包长和原有 26 项诊断与原版精确相同（耗时除外）。',
                  '包长、原有 35 项融合诊断和 10 项本地诊断与原版精确相同（耗时除外）。')
    s = s.replace('新包为 224 B/Bernoulli + 32 B 包头；CR 为原生 208 B/Bernoulli。旧开发',
                  '新包为 232 B/Bernoulli + 32 B 包头；SI 组件对照保留 224 B 原生包。旧开发')
    s = replace(s, "              '检测器训练划分、相关路线、二维相对坐标、近似关联与仿真链路限制继续',",
                "              '第一次预检在首个空帧遇到空数组形状断言。修复仅统一空数组形状；',\n              '没有已完成轨迹被替换，原源码、散列与失败日志均保留，成功预检才用于',\n              '完整阶段注册。详细边界见 PREFLIGHT_EMPTY_SHAPE_FIX.md。', '',\n              '检测器训练划分、相关路线、二维相对坐标、近似关联与仿真链路限制继续',")
    s = s.replace('Selective complete result written:', 'Asymmetric complete result written:')
    (OUT / 'build_report.py').write_text(s)
    print('Generated complete asymmetric report with four fixed component comparisons.')


if __name__ == '__main__':
    main()
