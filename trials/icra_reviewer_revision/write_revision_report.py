"""Write the reviewer ledger from complete, checked experiment summaries."""
from pathlib import Path
import json

OUT = Path(__file__).resolve().parent
PAPER = OUT.parents[1] / 'papers/icra2027'
GCE = 'marked_gaussian_evidence'
GS, FIXED = GCE + '_guarded_scalar', GCE + '_fixed_025'
ARMS = ['marked_lineage', 'marked_asymmetric', GS, GCE]
NAMES = dict(zip(ARMS, ['No-age KLA', 'Scalar', 'Guarded Scalar', 'GCE']))
NAMES.update({'marked_er': 'Recency', FIXED: 'Fixed Ratio (0.25)', GCE + '_no_curvature': 'Joint w/o curvature'})
CONDITIONS = ['reliable', 'intermittent']


def read(name):
    return json.loads((OUT / name).read_text())


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |', '| ' + ' | '.join(['---'] * len(headers)) + ' |'] +
                     ['| ' + ' | '.join(str(value) for value in row) + ' |' for row in rows])


def interval(stat):
    return f"{stat['mean']:.5f} [{stat['low']:.5f}, {stat['high']:.5f}]" if 'low' in stat else f"{stat['mean']:.5f}"


def main():
    controls, new, sensitivity = [read(name) for name in
        ['CONTROLS_ANALYSIS.json', 'NEW_DATA_ANALYSIS.json', 'MODEL_SENSITIVITY_ANALYSIS.json']]
    assert all(value['passed'] for value in [controls, new, sensitivity])
    correlation, selection = read('correlation_control.json'), read('FIXED_SELECTION.json')
    audits = [read('audit_' + name + '.json') for name in [
        'controls_development', 'gs_seen_transfer', 'fixed025_seen_transfer',
        'new_validation_primary', 'new_validation_recency', 'motion_development',
        'motion_seen_transfer', 'motion_new_validation', 'pd070_development', 'pd080_development', 'pd095_development']]
    assert all(value['passed'] for value in audits)
    runs = sum(len(value['rows']) for value in audits)
    frames = sum(value['audited_node_frames'] for value in audits)
    lines = [
        '# ICRA 审稿修改与实验复核', '',
        '审稿对象为 `9479f0d3` 的八页稿件。原意见保存在研究仓库的 '
        '`trials/icra_reviewer_revision/SOURCE_REVIEW_CN.md`。本轮先登记实验定义、已有数据暴露范围与算法/输入标识，'
        '再运行新增对照。GCE 主算法、门控、校准与检测器权重保持不变。', '',
        table(['意见', '处理', '结论边界'], [
            ['R1 独立数据', '下载官方 validation，固定设置后处理 3 段、748 对帧，完成 7 方法双通信评估',
             '仅 1 个原始记录此前缺席；该记录的主评估出现退化。稳定泛化与完整检测器训练独立性仍未建立'],
            ['R2 校正/曲率混杂', '新增自身完整递归的 Guarded Scalar，补齐 2×2 与配对交互量',
             'GCE 相对 GS 的均值优势较小；25 段的两个交互区间均包含零'],
            ['R3 准入新意与相关性', '开发集选择固定强度后评估；新增已知相关高斯覆盖率控制',
             '自适应准入优于测试的固定强度；PSD 不能保证统计可信度'],
            ['R4 运动/pD 依赖', '真实位姿补偿全部 34 旧段与 3 新段；9 段上完成全部 4 个 pD 设定',
             '并列报告相对排序及不利结果，不按新结果重选主配置'],
            ['R5 理论和通信表述', '重写摘要、贡献、归一化解释、结果与讨论；补列相对 No-age 的 raw 开销',
             '区分已有结构、可积性与经验收益；不推断实测无线时延、容量或能耗'],
        ]), '',
        f'本轮完整新增评估共 **{runs} 份序列×方法×通信条件轨迹**；保存输出经另一套 NumPy '
        f'实现复算了 **{frames:,} 个机器人—帧**的分布、集合抽取或评分。另有原实现/坐标恒等变换预检与 '
        'Recency 适配一致性预检。这里的“独立复算”指不同实现路径，不表示第三方评审或外部独立复现。', '',
        '## R1：新增数据及其限制', '',
        '新增数据来自 [UCLA 官方 validation 存档](https://ucla.app.box.com/v/UCLA-MobilityLab-V2V4REAL/file/1619910191481)。'
        '检查了全部 train/test/val 压缩包目录；新段名称与文件大小/CRC 候选均不与旧档案重合。'
        '但片段不等于独立原始记录，更不能据此声称新路线。', '',
        table(['本轮序列', '原始片段', '成对帧', '原始记录此前出现'],
              [[row['sequence'], row['scene'], row['frames'], '是' if row['original_recording_present_in_old_data'] else '否'] for row in new['sequences']]), '',
        '所有方法复用同一组新检测。采用原发布检查点和后处理；CPU 体素化与 MPS 浮点推理并非原发布 GPU 输出的逐比特复制。'
        '十二个旧传感器帧的检测数一致，匹配中心最大偏差 0.007771 m、分数最大偏差 0.003084；'
        '体素参考与原后处理几何检查通过。权重配置指定 train/test 为训练/验证，'
        '但该配置不能证明检查点的全部历史。新标签只用于几何转换与评分，不用于重新拟合校准或选择方法。', '',
    ]
    for group_name, title in [('all_new_segments', '全部新增片段，按片段等权'), ('new_recording', '唯一此前缺席的原始记录')]:
        group = new['groups'][group_name]
        lookup = {(row['condition'], row['arm']): row['ospa'] for row in group['aggregate']}
        arms = ['marked_lineage', 'marked_er', 'marked_asymmetric', GS, GCE + '_no_curvature', FIXED, GCE]
        lines += ['### ' + title, '', table(['方法', '可靠 OSPA', '间歇 OSPA'],
                    [[NAMES[arm]] + [f'{lookup[c, arm]:.6f}' for c in CONDITIONS] for arm in arms]), '']
    lines += ['全部 3 段的较好均值不能掩盖唯一新记录上的反例。正文同时展示两组结果；'
              '不对来自两个记录的三个片段给出泛化置信区间。', '', '### 每个新增片段的全部结果', '']
    lines += [table(['序列', '方法', '可靠 OSPA', '间歇 OSPA'],
              [[name, NAMES[arm]] + [f"{next(row['ospa'] for row in new['rows'] if row['sequence']==name and row['condition']==condition and row['arm']==arm):.6f}" for condition in CONDITIONS]
               for name in ['0000', '0001', '0002'] for arm in arms]), '',
        '来源：`NEW_DATA_FREEZE.json`、`NEW_INPUT_AUDIT.json`、`NEW_DATA_ANALYSIS.json`、'
        '`OFFICIAL_ADAPTER_PREFLIGHT.json`。新数据输入、旧九段校准和 Fixed Ratio 选择均在新跟踪之前固定。', '',
        '## R2：自己的递归，而非 GCE 输入上的一次替换', '',
        'GS 在自己的输入上重新计算资格、正负准入、源曲率与聚合 fallback，保留原空间池 p0/I0，'
        '将得到的后验反馈到自己的下一次预测和关联。它需要 GCE 的完整 352 B Gaussian ratio 包；'
        '原生 Scalar 为 232 B，不把两者记成同等通信成本。', '',
    ]
    for cohort, title in [('seen_transfer', '25 个已使用序列'), ('development', '9 个开发序列')]:
        group = controls['cohorts'][cohort]
        lookup = {(row['condition'], row['arm']): row['ospa']['mean'] for row in group['aggregate']}
        lines += ['### ' + title, '', table(['校正', '曲率', '可靠 OSPA', '间歇 OSPA'], [
            ['Scalar', '关'] + [f'{lookup[c, "marked_asymmetric"]:.6f}' for c in CONDITIONS],
            ['Scalar', '开'] + [f'{lookup[c, GS]:.6f}' for c in CONDITIONS],
            ['Joint', '关'] + [f'{lookup[c, GCE + "_no_curvature"]:.6f}' for c in CONDITIONS],
            ['Joint', '开'] + [f'{lookup[c, GCE]:.6f}' for c in CONDITIONS],
        ]), '', table(['配对差值（负值有利于前者）', '通信', '均值 [95% 区间]'],
            [[NAMES.get(row['candidate'], row['candidate']) + ' − ' + NAMES.get(row['reference'], row['reference']),
              row['condition'], interval(row['ospa'])] for row in group['paired'] if row['reference'] in [GS, 'marked_asymmetric']]), '',
            table(['交互：(GCE−GS)−(无曲率 Joint−Scalar)', '均值 [95% 区间]'],
                  [[row['condition'], interval(row['ospa'])] for row in group['interaction']]), '']
    lines += ['25 段间歇条件下 GCE−GS 的区间上界约为 −0.000063 m，接近零；九段开发子集的对应区间包含零。'
              '不把未做多重性校正的名义区间解读为稳定泛化或联合归一化的单独因果贡献。'
              '此前固定 GCE 输入的 2×2 替换与完整递归估计不同，全部四个替换仍保留在附带数据中。', '',
              '来源：`CONTROLS_ANALYSIS.json`、`LEGACY_RESCORING.json` 与对应 `audit_*.json`；'
              '配对重采样单位为完整序列，10,000 次，随机种子 8301。', '',
              '## R3：固定强度与已知相关控制', '',
              'Fixed Ratio 保留资格、当前机会、历史基底与曲率保护，将两种符号的恢复强度替换为常数。'
              '仅按旧九段两个通信条件均值，从下列既定网格选择 0.25，之后没有用新数据重选。', '',
              table(['强度', '可靠 OSPA', '间歇 OSPA', '选型均值'],
                    [[row['lambda'], f"{row['sequence_macro_ospa_by_condition']['reliable']:.6f}",
                      f"{row['sequence_macro_ospa_by_condition']['intermittent']:.6f}", f"{row['selection_mean_ospa']:.6f}"] for row in selection['candidates']]), '',
              table(['数据', '通信', 'GCE−Fixed Ratio，均值 [95% 区间]'],
                    [[cohort, row['condition'], interval(row['ospa'])] for cohort, group in controls['cohorts'].items()
                     for row in group['paired'] if row['reference'] == FIXED]), '',
              '相关控制是条件于目标存在的二维空间高斯特例：先验方差 25、单源测量方差 1，'
              '五个相关系数各 10,000 个样本。GCE 采用 unit admission，全部 PSD 检验通过；'
              '它不是对真实跟踪器自适应门控覆盖率的估计。', '',
              table(['相关系数', '方法', '位置 RMSE', 'NEES/维数', '95% 覆盖率'],
                    [[row['rho'], row['arm'], f"{row['position_rmse']:.6f}", f"{row['mean_nees_per_dimension']:.6f}",
                      f"{100*row['coverage_95']:.2f}%"] for row in correlation['rows']]), '',
              '在相关系数 0.9 处，GCE 的覆盖率为 80.55%，已知相关 oracle 为 95.19%。'
              '误差、联合协方差及覆盖率均由独立矩阵路径复算，见 `CORRELATION_AUDIT_V2.json`。'
              '先前审计器的 BLAS 浮点标志警告保留在记录中；改用等价 einsum 路径后核对通过，样本未重采或更换。', '',
              '## R4：运动坐标与检测概率', '',
              '绝对位姿对应全部 9+25 个旧序列；与发布的相对车辆变换误差小于 1e-5，'
              '标签 ID 集合与二维真值位置也与发布适配逐帧一致。变换仅使用原始位姿，'
              '在预测和上一帧检测创建的 birth 后，对位置、速度及完整协方差施加平面 yaw/translation 变换。'
              '恒等、静止/运动世界点、逆变换与协方差旋转检查通过。它不是完整六自由度运动补偿。', '',
              table(['数据', '方法', '补偿后可靠 OSPA', '补偿后间歇 OSPA'],
                    [[cohort, NAMES[arm]] + [f"{next(row['ospa']['mean'] for row in group['aggregate'] if row['condition']==c and row['arm']==arm):.6f}" for c in CONDITIONS]
                     for cohort, group in sensitivity['motion'].items() for arm in ARMS]), '',
              '### 唯一新记录：坐标模型会改变排序', '',
              table(['方法', '原可靠', '原间歇', '补偿后可靠', '补偿后间歇'],
                    [[NAMES[arm]] + [f"{next(row['ospa'] for row in rows if row['sequence']=='0000' and row['condition']==c and row['arm']==arm):.6f}"
                                     for rows in [new['rows'], sensitivity['motion']['new_validation']['rows']] for c in CONDITIONS]
                     for arm in ARMS]), '',
    ]
    for condition in CONDITIONS:
        lines += ['### pD 敏感性：' + condition, '', table(['pD'] + [NAMES[a] for a in ARMS],
            [[pd] + [f"{next(row['ospa']['mean'] for row in sensitivity['detection_probability'][pd]['aggregate'] if row['condition']==condition and row['arm']==arm):.6f}" for arm in ARMS]
             for pd in ['0.7', '0.8', '0.9', '0.95']]), '']
    lines += ['全部敏感性结果使用同一检测、校准与 GCE 设置。补偿后新记录上的 GCE 优于 No-age，'
              '仍略逊于 Scalar；不据此替换原主配置。pD 仅改变模型中的检测概率，不合成漏检、不重新筛检测，'
              '仍不能替代遮挡相关的可见性模型。来源：`MODEL_SENSITIVITY_ANALYSIS.json` 及全部 motion/pD 审计。', '',
              '## R5：论文修改位置', '',
              '- 摘要、引言与相关工作：新意集中于近似关联条件下的选择性准入；明确 prior/likelihood 分离已有。',
              '- 方法：命题改为 Bernoulli 归一化恒等式，保留推导；明确不提供误差排序或协方差一致性保证。',
              '- 实验与结果：新增完整递归 GS、固定强度、真实新段、运动补偿、pD 与相关控制，保留所有不利结果。',
              '- Table I/II：增加 GS、Fixed Ratio 与完整校正×曲率解释；Table IV 改为新增数据及唯一新记录。',
              '- Fig. 3：展示全部 pD 设置和已知相关覆盖率；原逐序列与断链阶段图仍附在源码中。',
              '- 通信：相对 No-age 的 raw 额外成本为 17.3%/21.5%，分片额外成本为 4.1%/7.0%；'
              '前者由未舍入原始数值计算，故与审稿意见按表中两位小数计算的 17.2%/21.6% 略有差别。',
              '- 讨论：保留独立数据数量、检测器历史、平面模型、未知相关及模拟无线的限制。', '',
              '## 可复核范围与尚未解决的问题', '',
              '结果后验与轨迹保存在本地 `trials/icra_reviewer_revision/results/`，未删除或覆盖。'
              '因其达到数 GiB，这些完整轨迹不写入 Git/论文 ZIP；提交的审计与结果文件清单记录每份输出的 SHA-256。'
              '论文包包含逐序列结果、来源快照、协议和统计/图表生成代码，可在没有 MATLAB 原始轨迹的目录重建图表与 PDF。'
              '完整跟踪重放还需要研究仓库、MATLAB 与公开输入数据。', '',
              'R1 的实验缺口得到实际补充，但“稳定跨记录泛化”和完整感知系统的独立验证仍未建立。'
              'R2 的必要递归对照已完成，结果不支持把全部收益解释为联合归一化的单独作用。'
              '这些是保留的研究限制，不以措辞修改冒充解决。未提交会议，也不把自动检查等同于作者认可或可接收性判断。', '',
              '制品验收以 `output/qa/artifact_qa.json`、`output/qa/visual_review.md`、'
              '`output/qa/portable_rebuild.json` 为准；目标保持正文完整七页，Ack/Ref 单独一页。', '',
    ]
    document = '\n'.join(lines)
    (OUT / 'REVISION_LEDGER_CN.md').write_text(document)
    (PAPER / 'REVIEW_REVISION_CN.md').write_text(document)
    overview = controls['cohorts']['seen_transfer']['aggregate']
    lookup = {(row['condition'], row['arm']): row['ospa']['mean'] for row in overview}
    concise_table = table(['方法', '可靠 OSPA（m）', '间歇 OSPA（m）'],
                         [[NAMES[arm]] + [f'{lookup[c, arm]:.3f}' for c in CONDITIONS]
                          for arm in ['marked_lineage', 'marked_asymmetric', GS, FIXED, GCE]])
    readme = '''# GCE 论文与可复核源码

**Guarded Current-Evidence Fusion for Cooperative Multitarget Tracking**

版面约束为正文完整七页，Ack/Ref 单独一页；四幅正文矢量图、四张表和三十条参考文献。
本版按审稿意见补充实际实验，并据结果收紧结论。未提交会议。

## 阅读入口

- 论文：`output/pdf/icra2027_draft.pdf`
- 可移植源码包：`output/icra2027_review_source.zip`
- 审稿修改、全部新数据与敏感性结果：`REVIEW_REVISION_CN.md`
- 新增实验数值：`source_data/reviewer_evidence.json`
- 来源快照和协议：`source_data/reviewer_revision/`
- 原有 25 段数值：`source_data/gaussian_paper_evidence.json`
- 固定输入与断链阶段：`source_data/mechanism_analysis.json`

## 本轮结果应如何理解

以下比较使用已参与方法开发的 25 个完整序列，不能当作冻结后的独立测试。

''' + concise_table + '''

Guarded Scalar 使用与 GCE 相同的曲率保护，并运行自己的完整递归。
GCE 对它的均值优势较小；校正×曲率交互的两个区间均包含零。
固定强度 0.25 仅由旧九段开发数据选定，随后应用于其余数据。

新增官方 validation 有三个成对片段、748 帧，来自两个原始记录。
仅 409 帧的那个记录此前缺席；GCE 在其原主配置评估中反而逊于 No-age 和 Scalar。
三个片段的均值优势不能证明稳定跨记录泛化。真实位姿补偿和全部 pD 敏感性结果
也同时保留，未按新结果重选 GCE。相关高斯控制则显示，即使全部 PSD 检验通过，
名义 95% 区域仍可明显欠覆盖。

通信图比较精度与字节开销。相对完整高斯包，编码节省分片字节 11.3%/10.8%；
相对 No-age，仍增加 raw 字节 17.3%/21.5% 和分片字节 4.1%/7.0%。
这是规定记账模型下的字节变化，未测量实际无线时延、容量或能耗。

## 图表与来源

- Fig. 1：`figures/intro.svg`；生图母版、提示词及可编辑复刻记录在 `intro_design/`。
- Fig. 2：`figures/overview.svg`，保留连续场景、两条融合分支及递归反馈。
- Fig. 3：`figures/gaussian_robustness.svg`，包含全部四个 pD、两种通信及五个相关系数。
- Fig. 4：`figures/gaussian_communication.svg`，保留相对 No-age 的额外成本。
- Table I/II：完整基线与校正×曲率控制；Table III/IV：通信成本与新增数据。
- 附带矢量图：`gaussian_paired`、`gaussian_components`、`gaussian_sequence_differences`、`gaussian_phases`。

每幅图有 SVG、矢量 PDF 和 PNG。旧逐序列图保留全部点；固定输入的四个替换
和全部断链阶段仍在源码数据内。它们与新增完整递归 GS 回答的问题不同。

## 构建

需要 Tectonic、NumPy、Matplotlib、Pillow、PyMuPDF 和 pypdf。在本目录执行：

```sh
python3 build.py --regenerate
```

脚本会探测可用 Python，也可用 `PAPER_PLOT_PYTHON`、`PAPER_PDF_PYTHON` 指定。
省略 `--regenerate` 可复用已经生成的图表。命令重建数值、表格和图形，编译 PDF，
核对完整七页正文与第八页 Ack/Ref、字体、浮动图表、引用/DOI 和来源数值，再生成 ZIP。

解压后在 `icra2027` 目录运行相同命令即可重建图表和论文，无需原始 MATLAB 轨迹。
完整跟踪重放另依赖研究仓库的 `trials/icra_reviewer_revision/`、MATLAB 与公开原始数据。
数 GiB 的完整后验轨迹保留在本地，不写入 Git 或论文 ZIP；随包清单记录其完整 SHA-256。
原算法、检测器和校准保持情况见 `source_data/reviewer_revision/REPLAY_ACCEPTANCE.json`。

## 验收边界

- 自动检查：`output/qa/artifact_qa.json`
- 逐页视觉检查：`output/qa/visual_review.md`
- 独立目录重建：`output/qa/portable_rebuild.json`
- ZIP 文件清单：`output/review_manifest.json`
- 文献来源：`LITERATURE_SCOPE.md`

这些是代码/制品检查与另一实现路径的数值复算，不表示第三方评审、作者批准或投稿完成。
稳定泛化、完整检测器训练独立性、真实跟踪协方差一致性及实测无线性能仍未建立。

官方 `ieeeconf.cls` 与 `IEEEtran.bst` 未修改，作者栏留空，AI 使用在 Ack 中披露。
页数约束依据此前核对的 [ICRA 2027 官方 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)，
最终文件状态以匹配 PDF 标识的验收记录为准。
'''
    (PAPER / 'README_CN.md').write_text(readme)
    print('Wrote review ledger:', runs, 'complete experiment trajectories,', frames, 'rescored node-frames.')


if __name__ == '__main__':
    main()
