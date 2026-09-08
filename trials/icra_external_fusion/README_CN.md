# 外部融合基线与 V2V4Real 真实检测回放

本目录保存 2026-09-08 的补充实验。结论和完整对比见
[实验报告](../../papers/icra2027/EXTERNAL_DATASET_RESULTS_CN.md)。
原有 `common/`、`lmb/`、`multisensorLmb/`、case-study 源码和结果没有修改。

## 已完成的实验

- TC-OSPA²：复用原有 60 个配对输入，在相同 Local 输出上增加窗口 5/10，
  共 120 次新运行。作者的运动状态融合函数保持原样；适配器与作者入口的
  18 个多节点输出逐元素一致。独立重算 115,200 个新增节点帧。
- V2V4Real：全部 9 个发布的评估序列、1,993 帧、2 辆车，6 个方案和 2 种通信
  条件，共 108 个序列/条件/方案组合；独立重算 47,832 个节点帧。
- MIL-AM：公共作者稿中的独立标签增广指派与 common/exclusive 子空间 MIL；
  数值检查覆盖未匹配槽位、存在概率不参与匹配、解析对称 KL、算术矩、
  恒等情况和输入交换。它不是经完整 TAES 2022 原文/代码核验的复现。
- 真实输入、结果、逐序列表、散列记录和运行日志已保存。烟雾测试输出不进入
  汇总；中止的两次预检日志单独保留。

## 数据入口与来源

`data/v2v4real_0000.mat` 至 `0008.mat` 是可直接用于 MATLAB 回放的紧凑输入，
总计约 1 MB。包含真实检测、相对传感器位置、时间以及独立评分用的真值。
跟踪器只接收量测/模型字段；真值不参与出生、关联和运动估计。
`prepare_v2v4real.py` 将发布的检测与标签转换为平面坐标，不生成合成量测。

作者依赖固定为：

- [TC 作者库](https://github.com/AdelaideAuto-IDLab/Distributed-limitedFoV-MOT)，
  `b6b20ec30b7854dcee6f4a718237d82d96ac7c2a`。
- [DMSTrack 作者库](https://github.com/eddyhkchiu/DMSTrack)，
  `d3b9949499c8e68ea33060873bd1cb95b6d4d323`；检测与标签已包含在该库中。
- [DMSTrack 数据说明](https://github.com/eddyhkchiu/DMSTrack/blob/main/docs/DATA.md)
  指向的 `no_fusion_keep_all.zip`。`fetch_v2v_transforms.py` 使用 HTTP Range
  只取 ZIP 目录和 3,986 个变换矩阵，逐个核验 CRC32、尺寸和 SHA-256，
  不需要下载 6.3 GB 特征/点云归档。

外部作者源码位于被忽略的 `tmp/external_baselines/`，保持原样，未复制进论文源码包。
请遵守作者库和数据原有研究使用条件。相对位姿矩阵完整清单保存在
`v2v4real_transform_manifest.json`；输入清单记录其散列与原始检测/标签散列。

## 最小复现

从仓库根目录执行。需要 MATLAB R2024a 或兼容版本；Python 需要 NumPy、SciPy。
本机解释器为 `tmp/external_baselines/venv/bin/python`。重新生成结果会覆盖对应
结果文件，建议在独立 checkout 中复跑。

```sh
python3 trials/icra_external_fusion/fetch_dependencies.py
python3 trials/icra_external_fusion/fetch_v2v_transforms.py
python3 trials/icra_external_fusion/prepare_v2v4real.py

/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_external_fusion'); checkTcAdapter; checkGaussianMil; checkReplayQuality;"
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_external_fusion'); runExternalCaseStudies(2901,2920);" 2>&1 | tee RUN/ICRA_EXTERNAL/tc_replay.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_external_fusion'); runV2v4realReplay(0,8,false);" 2>&1 | tee RUN/ICRA_EXTERNAL/v2v_replay.log

python3 trials/icra_external_fusion/analyze_case_studies.py
python3 trials/icra_external_fusion/analyze_v2v4real.py
```

日志查看：`tail -f RUN/ICRA_EXTERNAL/v2v_replay.log`。
已提交的输入可直接回放；前三条命令用于从发布源重新核验/转换输入。
不要重新冻结散列来掩盖源码改变：运行和分析应核验已经保存的协议与源码清单。

## 结果与检查记录

- `summary_case_studies.json`、`case_study_runs.csv`：外部 TC 和复用的对照数据。
- `summary_v2v4real.json`、`v2v4real_runs.csv`：全部真实序列、条件、方案、配对
  差值、序列 bootstrap 区间、共同匹配支持以及消息字节。
- `results/*_validation_tc.json.gz`、`results_v2v/*_reliable.json.gz` /
  `*_intermittent.json.gz`：逐帧输出。真实数据结果标识为
  `consistent-domain-and-absence-v2`。
- `RUN/ICRA_EXTERNAL/`：TC 和真实回放运行日志、独立评分日志、适配器检查日志。
- `PROTOCOL.md`、`V2V4REAL_PROTOCOL.md`：方法、通信、评分、统计单位及预检修订。
- `replay_source_sha256.json`：1,182 个源码/协议文件；原有 1,169 个源码未变。

这些是同一工作流程中的实现与指标交叉检查，不是第三方独立复现。
