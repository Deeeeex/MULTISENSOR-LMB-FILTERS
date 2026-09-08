# 当前高斯证据的完整分布校正

本目录是独立保存的研究候选，延续 M-AE 的标量证据规则，并把本地
预测/后验高斯比值用于空间分布及存在归一化。主方法和三项消融在
任何跟踪之前固定，详细方法及继续条件见 [PROTOCOL.md](PROTOCOL.md)，
数学解释及边界见 [DERIVATION_CN.md](DERIVATION_CN.md)。

新增摘要为 352 B/Bernoulli + 32 B 包头，其中新增 15 个 double 是
精度矩阵差的下三角、信息向量差和对数密度常数差。M-AE 对照保留
232 B 原生摘要。两者共用本地观测、分数模型和关联设置，但传输的
统计量与通信费用不同。

## 核查路径

1. `checkGaussianEvidence.m`：集中式单 Bernoulli 高斯特例，单源/零门控，
   曲率拒绝、非可积回退和实际包往返。首次单源夹具舍入问题完整记录在
   [UNIT_NUMERICAL_NOTE.md](UNIT_NUMERICAL_NOTE.md)。
2. `source_sha256.json`：单位夹具通过后、跟踪之前固定的 1347 个源文件。
3. `preflight_audit.json`：两种链路、五个方法的 2940 个节点—帧；
   原 M-AE 的 588 个节点—帧与原结果逐值一致。
4. `ROUND_FREEZE.json`：成功预检后固定完整九段和条件性 25 段执行。
5. `gaussian_audit.py`：从记录的本地预测/后验均值与协方差独立构造
   自然参数，不把生产程序给出的比值当作正确答案。核对每个原来源、
   时间、原始标签连接以及最终完整分布。
6. `analyze_gaussian.py`：本地存在增量、正负门控、来源资格、时效权重、
   可积性、通信、提取和 OSPA/GOSPA 统一复算；旧参照按已保存输出复用。

所有真实轨迹均已参与此前方法开发。本轮结果不能解释为独立测试。
通过开发继续条件只允许扩大该候选，不自动意味着达到论文主结果标准。

## 执行与结果

完整阶段不覆盖既有输出。首次运行顺序为源冻结、MATLAB 预检、Python
预检、注册，再执行阶段；生成脚本只用于构建新一轮，不应重写已固定源。

```sh
./tmp/external_baselines/venv/bin/python trials/icra_gaussian_evidence/run_sequences.py --cohort development
./tmp/external_baselines/venv/bin/python trials/icra_gaussian_evidence/analyze_gaussian.py --watch --cohort development
./tmp/external_baselines/venv/bin/python trials/icra_gaussian_evidence/build_report.py --cohort development
./tmp/external_baselines/venv/bin/python trials/icra_gaussian_evidence/verify_outputs.py --cohort development
```

仅当完整九段主方法通过固定条件，调度器才允许 `--cohort seen_transfer`。
全部细项位于 `summary_<cohort>.json`，中文结果位于
`RESULTS_<cohort>_CN.md`，组件配对位于 `ablation_comparisons_<cohort>.json`，
最终散列与原生进程退出核查位于 `CURRENT_QA_<cohort>.json`。
对应日志位于仓库 `RUN/ICRA_GAUSSIAN_EVIDENCE/`。

单位夹具通过、预检通过、完整开发通过和可作论文主结果是不同状态。
论文文件在本候选阶段保持原有版本。
