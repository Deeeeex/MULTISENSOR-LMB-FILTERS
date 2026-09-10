# 传递本帧裁剪信息：完整递归因果检验

在已曝光的 v2xt_0001 全 240 帧上，六条新控制与上一轮接收端本地修正逐字段精确一致（运行时间除外）。六条通信干预使用同一份当前裁剪标量协议，分别覆盖 GCE、Guarded Scalar、No-age 与两条链路。另列六条已经审计的原始轨迹。

12 条新轨迹全部正常退出，5760 个机器人帧完成概率、高斯、匹配、预测与裁剪平衡、MAP 提取和评分核对。每包裁剪 trailer 的 uint8 字节独立按 little-endian float64 解码，并从发送方本帧更新记录完整重建；每个使用的远端标量均来自当帧实际送达的消息。消息头 32 字节、每个条目 32 字节全部计入尝试、送达及 16384 字节块填充成本。

下表窗口仍是第 53–122 帧、目标 ID 5；检出按所有真值的 2 m 一对一匹配计算。复现次数包括同帧裁剪后返回和跨预测空档返回，避免把一帧延迟误当作消除。它们是相关标签事件数，不是独立物理目标数。

| 链路 | 方法 | OSPA ↓ | GOSPA ↓ | 窗口检出 /140 | 全段检出 /480 | 窗口标签复现 | wire 字节 |
|---|---|---:|---:|---:|---:|---:|---:|
| reliable | GCE 原始 | 8.750049 | 25.352520 | 0 | 68 | 13,040 | 18,853,888 |
| reliable | GCE + 本地实际值 | 8.768466 | 25.357382 | 0 | 67 | 9,504 | 18,313,216 |
| reliable | GCE + 当前裁剪通信 | 8.749328 | 25.287200 | 0 | 70 | 1,787 | 11,431,936 |
| reliable | Guarded Scalar 原始 | 8.938267 | 25.664306 | 0 | 0 | 15,628 | 20,279,296 |
| reliable | Guarded Scalar + 本地实际值 | 8.837588 | 25.460762 | 0 | 53 | 9,591 | 19,886,080 |
| reliable | Guarded Scalar + 当前裁剪通信 | 8.923682 | 25.599581 | 0 | 2 | 3,240 | 12,087,296 |
| reliable | No-age 原始 | 8.574881 | 24.954896 | 140 | 206 | 0 | 11,382,784 |
| reliable | No-age + 本地实际值 | 8.578222 | 24.958522 | 140 | 206 | 0 | 11,317,248 |
| reliable | No-age + 当前裁剪通信 | 8.294486 | 24.297930 | 140 | 402 | 0 | 7,925,760 |
| intermittent | GCE 原始 | 8.653982 | 25.055787 | 18 | 87 | 6,368 | 24,424,448 |
| intermittent | GCE + 本地实际值 | 8.665815 | 25.059744 | 18 | 85 | 5,353 | 24,473,600 |
| intermittent | GCE + 当前裁剪通信 | 8.670273 | 25.030705 | 23 | 94 | 1,815 | 12,267,520 |
| intermittent | Guarded Scalar 原始 | 8.797806 | 25.308160 | 18 | 19 | 8,238 | 25,178,112 |
| intermittent | Guarded Scalar + 本地实际值 | 8.716755 | 25.149331 | 16 | 66 | 5,780 | 24,506,368 |
| intermittent | Guarded Scalar + 当前裁剪通信 | 8.715747 | 25.113014 | 22 | 78 | 1,832 | 12,169,216 |
| intermittent | No-age 原始 | 8.264497 | 24.136504 | 115 | 382 | 0 | 9,367,552 |
| intermittent | No-age + 本地实际值 | 8.266762 | 24.131558 | 115 | 382 | 0 | 9,383,936 |
| intermittent | No-age + 当前裁剪通信 | 8.255886 | 24.100178 | 115 | 382 | 0 | 7,942,144 |

| 链路 | 方法 | OSPA 改善 % | GOSPA 改善 % | 窗口检出变化 | 全段检出变化 | 复现变化 | wire 变化 % |
|---|---|---:|---:|---:|---:|---:|---:|
| reliable | GCE | +0.218 | +0.277 | +0 | +3 | -7717 | -37.575 |
| reliable | Guarded Scalar | -0.974 | -0.545 | +0 | -51 | -6351 | -39.217 |
| reliable | No-age | +3.308 | +2.647 | +0 | +196 | +0 | -29.967 |
| intermittent | GCE | -0.051 | +0.116 | +5 | +9 | -3538 | -49.874 |
| intermittent | Guarded Scalar | +0.012 | +0.144 | +6 | +12 | -3948 | -50.343 |
| intermittent | No-age | +0.132 | +0.130 | +0 | +0 | +0 | -15.364 |

上述变化均相对于本轮精确复现的接收端本地修正控制。

| 链路 | 方法 | 尝试 payload | 送达 payload | 尝试 trailer | 送达 trailer | 发送裁剪条目 |
|---|---|---:|---:|---:|---:|---:|
| reliable | GCE + 当前裁剪通信 | 6,787,040 | 6,787,040 | 282,208 | 282,208 | 8,339 |
| reliable | Guarded Scalar + 当前裁剪通信 | 7,508,000 | 7,508,000 | 361,472 | 361,472 | 10,816 |
| reliable | No-age + 当前裁剪通信 | 3,151,512 | 3,151,512 | 180,192 | 180,192 | 5,151 |
| intermittent | GCE + 当前裁剪通信 | 7,989,728 | 6,015,584 | 303,584 | 256,160 | 9,007 |
| intermittent | Guarded Scalar + 当前裁剪通信 | 8,091,680 | 6,094,528 | 324,576 | 276,672 | 9,663 |
| intermittent | No-age + 当前裁剪通信 | 3,253,240 | 2,559,904 | 186,880 | 150,176 | 5,360 |

公平比较：
- reliable：GCE − 同信息 No-age 的 OSPA 差为 +0.454842，GOSPA 差为 +0.989271。负值有利于 GCE。
- intermittent：GCE − 同信息 No-age 的 OSPA 差为 +0.414387，GOSPA 差为 +0.930526。负值有利于 GCE。

预先冻结的扩大验证门槛：
- reliable：ospa_improves_at_least_one_percent=未通过；gospa_improves_at_least_one_percent=未通过；beats_shared_noage_ospa=未通过；beats_shared_noage_gospa=未通过；window_detection_improves=未通过；full_detection_nonworsening=通过；recurrences_decrease=通过；wire_within_cap=通过。
- intermittent：ospa_improves_at_least_one_percent=未通过；gospa_improves_at_least_one_percent=未通过；beats_shared_noage_ospa=未通过；beats_shared_noage_gospa=未通过；window_detection_improves=通过；full_detection_nonworsening=通过；recurrences_decrease=通过；wire_within_cap=通过。

门槛未全部通过，本次当前裁剪标量通信规则按协议关闭；不调整阈值、延长保留期或改选方法补救。

这是一项已经曝光的单案例因果诊断，不是独立方法验证，也不能据此声称泛化或统计显著性。即使标签复现减少，也须同时查看最终检出、完整误差和通信成本。主论文与共享生产融合实现未改动。全部原始/本地/通信三组结果和 GOSPA 分解保存于 RESULTS.json、ALL_SCORES.csv 及逐帧记录中。
