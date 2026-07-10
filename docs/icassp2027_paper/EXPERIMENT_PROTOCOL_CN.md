# ICASSP Fusion-Sufficient Moment Exchange 实验协议

状态：**在 confirmatory 结果产生前冻结**

协议版本：`fusion-sufficient-moment-exchange-v1`

## 1. 研究问题与唯一变量

本实验检验一个严格限定的问题：对于当前实现的单轮 projected Gaussian KLA-LMB receiver，将逐标签 canonical moment projection 从接收端前移到发送端，是否在保持 fusion output 完全一致的同时减少应用层报文字节？

实验只含两臂：

1. `Periodic full posterior`：每个静态有向边、每个时刻发送 full GM-LMB message；
2. `Periodic light posterior on static topology`：在发送前逐标签投影为一个 Gaussian，再经相同 wire codec 发送 moment message。

唯一改变的变量是 sender 是否在编码前执行 canonical moment projection。两臂共享随机种子、测量、传感器/目标轨迹、静态 4+4 topology、发送 schedule、active-label threshold、Metropolis fusion weights、existence weights 与预生成 delivery uniforms。当前 loss draw 与 payload size 无关。

两臂均显式关闭 dynamic topology、link gate、stale cache、heartbeat、mixed payload、covariance inflation、mode-aware weights、initial/label-change/stale heavy override。第一臂固定 `alwaysHeavy`，第二臂固定 `alwaysLight`；这里 event type 只标识 full 或 projected wire payload，不改变发送时机。

## 2. 冻结批次与运行规则

Confirmatory batch 固定为 50 个 paired trials：

- `numberOfTrials=50`
- `baseSeed=81`
- 实际 seeds：`82:131`
- 每个 trial 的 simulation length：100
- bootstrap RNG seed：`20270710`
- bootstrap resamples：10,000

唯一允许的预运行 smoke 是 `(true,5,1000)`，即 seeds `1001:1005`。Smoke 只检验执行、schema 和 gate，不可与 confirmatory 结果合并，也不得覆盖 N50 文件。除这两种调用外，paper-specific runner 拒绝执行。

在启动 N50 前，runner、codec、projection、snapshot comparator、artifact writer/validator 和本协议必须先 commit 并 push。产物记录该 implementation commit。任何算法、配置、指标定义、codec schema、随机种子或分析方法的修改都会使已有 batch 失效；修改后必须形成新 commit 并从头重跑，不得选择性续用旧 trials。

`runExperiment=true` 还有一个不可绕过的 provenance gate：所有仓库内 `.m` runtime/test 文件与本协议必须已包含在当前 `HEAD` 且无 staged、unstaged 或 untracked 改动；当前 `HEAD` 必须与配置的 upstream commit 完全相同。Dry-run `runExperiment=false` 不执行该 gate，便于在提交前检查冻结配置。因而任何未 commit/push 的 runner 修改都不能启动 N5/N50。

## 3. 字节口径

唯一 byte oracle 是 versioned LMB application-layer codec 产生的 `uint8` 数组长度：

- `attemptedPayloadBytes`：delivery draw 之前已经编码并尝试发送的应用层字节；
- `deliveredPayloadBytes`：delivery 成功且被 receiver 解码的应用层字节；
- 旧字段 `payloadBytes` 仅为 delivered compatibility alias，不进入论文主指标。

口径明确排除 MAC/PHY framing、IP/transport headers、MTU fragmentation、retransmission 和 packet-size-dependent loss。主通信指标是 attempted bytes，delivered bytes 作为接收信息量单独报告。

对 seed `s`，预先定义：

```text
rho_s = 100 * (B_full,s - B_moment,s) / B_full,s
```

其中 `B` 是该 seed 汇总的 attempted application-layer bytes。主数字是 50 个 `rho_s` 的算术平均，而不是 aggregate byte ratio。95% 区间固定为：从 50 个 paired `rho_s` 中有放回抽取 50 个值，计算均值，重复 10,000 次，取 bootstrap means 的 2.5/97.5 percentiles。不得根据观察结果改成 BCa、basic interval 或任意 pass-count gate。

## 4. 输出等价与因果隔离 gate

Runner 必须开启 `capturePosteriorSnapshots=true`，逐 sensor、逐 time 保存只含 sorted labels、`r`、`mu`、`Sigma` 的紧凑快照。每个 seed 以 full arm 为 baseline 比较 projected arm，预先固定以下硬 gate：

- missing snapshot count = 0；
- label-set mismatch count = 0；
- missing label count = 0；
- max `|delta r|` = 0；
- max `|delta mu|` = 0；
- max `|delta Sigma|` = 0；
- `exactMatch=true`。
- 每臂每 trial 的 comparator `snapshotCount=S*T=8*100=800`。

这里 0 是 binary64 字段的 exact gate，不是事后选择的 tolerance。若任何一项非零，batch 失败，应先诊断实现，不能把容差调大后继续宣称 exact equivalence。

每个 seed 还必须满足：

- full/moment attempted masks 完全相同；
- full/moment delivered masks 完全相同；
- 每个 mask 都必须是 logical `8×8×100` 数组；
- delivered bytes 不大于 attempted bytes；
- attempted-byte reduction 严格大于 0。

所有 attempted/delivered byte 输入必须是 finite、非负、整数；两臂每个 seed 的 attempted bytes 都必须大于 0。

Tracking diagnostics 同时保存每 seed 的 mean local E-OSPA、consensus OSPA、consensus position disagreement 与 consensus cardinality dispersion。它们用于证明两臂实际执行结果一致，不替代字段级 snapshot gate。

## 5. 结构化产物与验证

N50 的固定 stem 是：

`GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131`

一次完成的运行原子发布三个同 stem 产物：

- 本地忽略的 `.mat`：完整 simulator summary 与 frozen config；
- `.csv`：逐 seed 的 attempted/delivered bytes、paired reductions、tracking/consensus metrics、输出残差与 mask flags；
- `.md`：配置、commit、wire/evidence schema、aggregate、bootstrap interval、MAT/CSV SHA-256 与 regeneration command。

Writer 先在目标目录生成临时 sibling files，关闭并调用 validator；只有三者全部通过才 rename 为最终文件。Markdown 只记录稳定的 repo-relative `RUN/GA/<basename>`，不记录某台机器的 worktree 绝对路径。

三次 rename 或最终验证中任一步失败时，writer 必须删除本次已经发布的 final files，避免 partial bundle 阻塞同一冻结批次重跑。N5 smoke 的 `.csv/.md` 是 local-only ignored artifacts；N50 `.csv/.md` 保持可跟踪。

Validator 必须重新载入 `.mat`，逐行重算 CSV、重算所有 aggregate 和 bootstrap interval，核对 seeds/config/schema/commit、exact residual gates、mask shape/equality 与 SHA-256。Writer 与 validator 共用 deterministic Markdown renderer；validator 用重算结果机械重建完整 report，并与文件逐字节比较，而不只检查隐藏 machine block。测试会分别篡改一个 CSV cell 和 human-readable 主数字，validator 均必须拒绝。论文数字与图只能读取通过该 validator 的 artifact，不得手工抄写或在绘图脚本硬编码。

## 6. 执行入口

只查看冻结配置，不运行仿真：

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); [~,~,~,~,c]=runFusionSufficientMomentExchangeConfirmatory(false); disp(c);"
```

唯一 smoke：

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); runFusionSufficientMomentExchangeConfirmatory(true,5,1000);"
```

Confirmatory N50（仅在实现 commit/push 与 N5 gate 通过后启动）：

```bash
octave --quiet --eval "setPath; addpath('RUN/GA'); runFusionSufficientMomentExchangeConfirmatory(true,50,81);"
```
