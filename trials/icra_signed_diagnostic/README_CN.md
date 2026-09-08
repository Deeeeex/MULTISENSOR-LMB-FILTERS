# 负向本帧证据的固定输入诊断

全部九序列、两种链路、两套已保存历史上的负证据规则均已完成。
额外负证据降低误报，但增加漏检；OSPA 均值略有改善，所有新增负证据
规则的配对区间仍包含零。这个诊断不证明完整递推会改善。

下一步值得检验的是“当前可见且漏检分支支持”的负证据，原因是它对应
明确的非观测似然；不是因为某个序列最好。后续若执行完整递推，应预先
固定该规则、保留去掉它及其他组件的消融，并完成全部序列。

negative_all 放行全部当前负增量；negative_miss 按漏检关联质量和名义
漏检似然的对比量加权，系数为 (1−检测关联质量)×0.9/1.1。实际遮挡
没有被名义可见性模型完全捕捉，漏检增加是必须保留的风险。

| 链路 | 固定输入历史 | 标量规则 | ΔOSPA [95% 区间] m | Δ漏检 m² | Δ误报 m² |
| --- | --- | --- | --- | --- | --- |
| reliable | marked_selective | positive | +0.000000 [+0.000000, +0.000000] | +0.000000 | +0.000000 |
| reliable | marked_selective | mark_signed | -0.001890 [-0.005494, +0.000656] | +0.070251 | -0.062515 |
| reliable | marked_selective | negative_all | -0.011499 [-0.043357, +0.018755] | +2.665362 | -2.773123 |
| reliable | marked_selective | negative_miss | -0.005314 [-0.025302, +0.015613] | +1.933551 | -1.803799 |
| reliable | marked_selective_signed | positive | +0.002181 [+0.000000, +0.006542] | +0.000000 | +0.052632 |
| reliable | marked_selective_signed | mark_signed | +0.000000 [+0.000000, +0.000000] | +0.000000 | +0.000000 |
| reliable | marked_selective_signed | negative_all | -0.003046 [-0.036252, +0.027637] | +2.779424 | -2.453750 |
| reliable | marked_selective_signed | negative_miss | -0.003297 [-0.021741, +0.014052] | +1.944463 | -1.565129 |
| intermittent | marked_selective | positive | +0.000000 [+0.000000, +0.000000] | +0.000000 | +0.000000 |
| intermittent | marked_selective | mark_signed | -0.001131 [-0.004767, +0.001374] | +0.065025 | -0.115205 |
| intermittent | marked_selective | negative_all | -0.013717 [-0.040031, +0.005433] | +1.710673 | -2.181393 |
| intermittent | marked_selective | negative_miss | -0.012300 [-0.038570, +0.005581] | +1.216553 | -1.518546 |
| intermittent | marked_selective_signed | positive | +0.001042 [-0.001239, +0.004555] | -0.108931 | +0.026316 |
| intermittent | marked_selective_signed | mark_signed | +0.000000 [+0.000000, +0.000000] | +0.000000 | +0.000000 |
| intermittent | marked_selective_signed | negative_all | -0.003369 [-0.014855, +0.006975] | +1.720331 | -2.083623 |
| intermittent | marked_selective_signed | negative_miss | -0.011283 [-0.037397, +0.006358] | +1.192637 | -1.559567 |

差值相对保持该历史的原始规则。区间为按九个序列重采样的 10000 次
描述性百分位区间。原规则的所有节点—帧重新评分均与原结果一致。
两套历史各自包含不发生融合的帧，这些帧保持原输出；发生融合时保持
空间位置和候选标签，按替换后的存在值重新做原 MAP 基数提取。

核对 15944 个原节点—帧，计算 79720 个规则节点—帧；
完整均值、区间、所有逐序列结果及输入散列保存在 summary_negative.json。
没有重跑跟踪器、调参、修改原轨迹或更新论文。
