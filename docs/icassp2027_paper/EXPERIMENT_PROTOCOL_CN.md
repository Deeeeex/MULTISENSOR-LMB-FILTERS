# ICASSP Fusion-Sufficient Moment Exchange 实验协议

状态：**在 confirmatory 结果产生前冻结**

协议版本：`fusion-sufficient-moment-exchange-v2`

## 0. v2 runtime amendment（confirmatory 前）

本修订发生在任何 confirmatory seed `82:131` 启动之前，也没有观察这些 seeds 的 tracking、通信或等价性结果。唯一动机是 v1 串行 smoke 的运行时诊断：一次 `T=100` full arm 约需 262.6 s，照原串行路径完成 N50 约需 6 h。该 runtime 数字是 development evidence，不进入论文结论；已有 v1 串行 N5 产物作废，正式 smoke 必须用本 v2 路径重新运行。

v2 不改变算法、两臂、seeds、`T=100`、随机数生成、每 seed 的预生成 `linkUniforms`、codec、指标、bootstrap 或 evidence CSV 语义，只把彼此独立的 paired seed trials 分配到独立 `octave-cli` subprocess。执行环境冻结为 Octave `11.1.0`；本机为 10 logical cores / 16 GiB，观测单 worker 约 110 MiB 且占用一个 CPU core，因此预注册并固定 `maxWorkers=6`。该上限不得按运行时负载自适应改变；它只是完成同一 Monte Carlo batch 的执行配置，不是方法或论文贡献。

## 1. 研究问题与唯一变量

本实验检验一个严格限定的问题：对于当前实现的单轮 projected Gaussian KLA-LMB receiver，将逐标签 canonical moment projection 从接收端前移到发送端，是否在保持 fusion output 完全一致的同时减少应用层报文字节？

实验只含两臂：

1. `Periodic full GM fusion message`：每个静态有向边、每个时刻发送 full GM fusion message；
2. `Periodic moment message on static topology`：在发送前逐标签投影为一个 Gaussian，再经相同 wire codec 发送 moment message。

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
- immutable batch identity：`confirmatory-primary-seeds82-131-v2`

唯一允许的预运行 smoke 是 `(true,5,1000)`，即 seeds `1001:1005`，identity 为 `smoke-seeds1001-1005-v2`。Smoke 只检验执行、schema 和 gate，不可与 confirmatory 结果合并，也不得覆盖 N50 文件。当前 paper-specific runner 故意只允许该 smoke 与 primary confirmatory `82:131`；fallback seeds 不在当前实现的允许列表中。

在启动 N50 前，runner、codec、projection、snapshot comparator、artifact writer/validator 和本协议必须先 commit 并 push。产物记录该 implementation commit。在 reservation acquisition 之前，任何算法、配置、指标定义、codec schema、随机种子或分析方法的修改都要求重新冻结 commit。Confirmatory reservation 一旦取得，`82:131` 即永久 burn：无论 worker 是否真正完成，都不得删除 ledger/worker directory 后重跑这些 seeds。

`runExperiment=true` 还有一个不可绕过的 provenance gate：所有仓库内 `.m` runtime/test 文件、v2 launcher `.sh` 与本协议必须已包含在当前 `HEAD` 且无 staged、unstaged 或 untracked 改动；当前 `HEAD` 必须与配置的 upstream commit 完全相同。Dry-run `runExperiment=false` 不执行该 gate，便于在提交前检查冻结配置。因而任何未 commit/push 的 runner 修改都不能启动 N5/N50。

主 runner 先在 worker directory 外的 ignored `.fusion_sufficient_attempts/<batchIdentity>.reserved` 通过 OS atomic `mkdir` 获取稳定 reservation；这是 seed-burn 的线性化点，即使误删 worker directory，该 reservation 仍阻止第二主进程重跑。取得 reservation 后才创建 worker directory，并以 temporary file + POSIX no-clobber hard link 发布 `batch_plan.mat`。Reservation 同时生成一次性 256-bit launcher capability；稳定 receipt 只保存其 SHA-256，明文 capability 以权限 0600 留在 worker directory。Launcher 必须原子消费该 capability 后才能写结构化 lifecycle claim，未取得 claim 的第二 launcher 不得写 FAILED tombstone。每个 seed 由一个 `octave-cli` subprocess 走相同 generic two-arm runner，保留完整 single-seed summary、完整 frozen config、config/source/summary hashes、commit、schema、Octave version 与 exact-gate 状态；stdout/stderr 仅进入该 seed 的独立 log。Launcher 固定最多同时运行 6 个 worker，只在控制台报告 seed start/finish/exit，禁止交错 simulator stdout。每个 subprocess 的 PID 必须在 `$!` 返回后立即登记到 signal trap 的 active set；INT/TERM 会终止并 wait 已登记的全部子进程。

从 reservation acquisition 到 success/failed terminal receipt 必须始终使用同一个专用 worktree；禁止复制 checkout、移动 attempt ledger 或在另一个 worktree 以相同 identity 启动。这是避免跨 worktree transient-source/ledger TOCTOU 的执行约束。

只有所有 worker exit 0 且恰好收齐预期 MAT/log 后，launcher 才生成一次性 256-bit completion token，发布绑定 launcher capability hash 的 completion authorization 与 ordered exit ledger，再同时持有明文 completion token 和先前已原子消费的明文 launcher capability 调用 success publisher；两项明文均不写入稳定 ledger。Publisher 必须重新哈希 launcher capability，并与 reservation receipt、结构化 lifecycle claim、exit ledger 三处的 `launcher_capability_sha256` 逐一匹配；因此 launcher 崩溃后仅凭磁盘 ledger 与补造的 completion token 不能封口成功。随后才发布同时存于 worker directory 和稳定 attempt ledger 的 `batch_success.receipt`。Receipt 绑定 schema、batch identity、config/source/plan/exit-ledger SHA-256、launcher capability/completion-token SHA-256、ordered seeds、maxWorkers、每 seed `exit=0` 与 MAT/log SHA-256。任一 worker 非零、launcher INT/TERM 或 ownership 后的 worker-stage 错误都会等待/终止当前 wave 并发布不可覆盖的双位置 `batch_failed.tombstone`。FAILED/BURNED 即永久拒绝，即使事后补齐 MAT 也不能 assemble。

Assembler 只接受 `COMPLETE_WORKERS`：success receipt 两份相同、FAILED 不存在、结构化 lifecycle claim 与 reservation 中的 launcher capability hash 一致，且 plan、completion authorization、exit ledger、每个 artifact 的 hashes 全部复算匹配。随后按 seed 升序组合最终最小完整 summary，并从 trial fields 重新计算 equivalence aggregates，不信任 worker aggregate。若主进程在 COMPLETE_WORKERS 后、assemble 或三文件发布前中断，允许只恢复 assembly/publish，绝不重算 worker；临时 writer/assembler 失败不会把完整 worker seal 改写成 FAILED。若 confirmatory execution 已 FAILED/BURNED，必须保留 tombstone/ledger，新 commit 先做预注册 amendment，才可显式启用未观察的 `132:181` 与新 identity；当前 runner 不提供该 fallback。

`COMPLETE_WORKERS` 后的 evidence publication 有独立线性化点：publisher 必须在稳定 attempt directory 通过 atomic `mkdir publish.claim` 取得 claim，并写入 token hash、PID 与 hostname。已有 claim（包括进程崩溃留下的 stale claim）或存在未封口 final artifact 时一律机械拒绝自动接管，只能人工审计；不得按 PID 是否存活自动抢占。成功 publisher 用 POSIX no-clobber hard links 发布三件套，并在每次 link 后核对 source/final 是同一 device+inode；rollback 只能删除仍属于本次 temporary source inode 的 final。三件套再次通过 validator 后，才写不可覆盖的 `evidence_published.receipt`，绑定 claim 与 MAT/CSV/MD hashes。三次 link 是带所有权校验的 transactional publication，不是跨文件的单一原子操作；若 final 已出现但 PUBLISHED seal 未完成，claim 必须保留供人工审计。

所有 synthetic/fault-injection 路径使用独立 `*-test-v2` schema、`TEST_` artifact stem、`test-` identity 与只在 `test_*` 调用栈有效的显式 authorization；生产 ownership/worker/assembler/writer/validator/publisher 均拒绝这些对象。测试产物不能作为论文 evidence。

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

v2 smoke 的 local-only stem 是：

`GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_V2_N5_SEEDS1001_1005`

一次完成的运行以 transactional rollback 发布三个同 stem 产物（不是跨三个文件的文件系统原子事务）：

- 本地忽略的 `.mat`：完整 simulator summary 与 frozen config；
- `.csv`：逐 seed 的 attempted/delivered bytes、paired reductions、tracking/consensus metrics、输出残差与 mask flags；
- `.md`：配置、commit、wire/evidence schema、Octave/maxWorkers 执行 provenance（明确非论文贡献）、config/source hashes、aggregate、bootstrap interval、MAT/CSV SHA-256 与 regeneration command。

Writer 先在目标目录生成临时 sibling files，关闭并调用 validator；只有三者全部通过才在独占 publish claim 下用 POSIX no-clobber hard links 发布为最终文件。Markdown 只记录稳定的 repo-relative `RUN/GA/<basename>`，不记录某台机器的 worktree 绝对路径。

三次 link 或最终验证中任一步失败时，writer 只能删除经 inode 核验仍属于本次 publisher 的 final files，绝不能删除并发替换的外部文件。若 worker seal 已是 COMPLETE_WORKERS 且没有未封口 final/stale claim，随后可从同一 receipt 仅恢复 assembly/publish；不得重跑任何 seed。N5 smoke 的 `.csv/.md` 是 local-only ignored artifacts；N50 `.csv/.md` 保持可跟踪。在 N5/N50 evidence commit 完成前，禁止执行 `git clean -fdX` 或任何会删除 ignored attempt/worker ledger 的清理命令。

Validator 必须重新载入 `.mat`，逐行重算 CSV、重算所有 aggregate 和 bootstrap interval，核对 seeds/config/schema/commit、exact residual gates、mask shape/equality 与 SHA-256。Writer 与 validator 共用 deterministic Markdown renderer；validator 用重算结果机械重建完整 report，并与文件逐字节比较，而不只检查隐藏 machine block。测试会分别篡改一个 CSV cell 和 human-readable 主数字，validator 均必须拒绝。论文数字与图只能读取通过该 validator 的 artifact，不得手工抄写或在绘图脚本硬编码。

## 6. 执行入口

只查看冻结配置，不运行仿真：

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); [~,~,~,~,c]=runFusionSufficientMomentExchangeConfirmatory(false); disp(c);"
```

唯一 smoke：

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); runFusionSufficientMomentExchangeConfirmatory(true,5,1000);"
```

Confirmatory N50（仅在实现 commit/push 与 N5 gate 通过后启动）：

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); runFusionSufficientMomentExchangeConfirmatory(true,50,81);"
```
