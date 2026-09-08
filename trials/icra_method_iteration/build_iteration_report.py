"""Build the iteration report from the complete, audited result summaries."""
from pathlib import Path
import csv
import hashlib
import json
import math

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
NAMES = {
    "local": "Local", "lineage": "无时效", "qualified_exist": "原 ER",
    "innovation_recency": "IR 增量版", "conservative_recency": "CR 严格约束版",
    "confirmation_recency": "CGR 确认门控版", "mil_support": "MIL-AM",
    "tc_ospa2_w5": "TC-OSPA² w=5", "tc_ospa2_w10": "TC-OSPA² w=10",
}
ARMS = list(NAMES)
CANDIDATES = ["conservative_recency", "confirmation_recency"]
CONDITIONS = {"reliable": "可靠链路", "intermittent": "间歇链路"}
SCENES = {"split_latebirth": "分裂后新生", "churn_departure": "成员变化与离场", "split_no_new": "无新目标控制"}


def read(name):
    return json.loads((OUT / name).read_text())


def table(headers, rows):
    return "\n".join(["| " + " | ".join(headers) + " |",
                      "| " + " | ".join(["---"] * len(headers)) + " |"] +
                     ["| " + " | ".join(str(v) for v in row) + " |" for row in rows])


def fmt(value):
    return f"{value:.4f}"


def ci(row, key="ospa"):
    r = row[key]
    return f"{r['mean']:+.4f} [{r['low']:+.4f}, {r['high']:+.4f}]"


def main():
    dev = {"ir": read("summary_development.json"), "cr": read("summary_conservative.json"),
           "cgr": read("summary_confirmed.json")}
    cases = {"cr": read("summary_cases.json"), "cgr": read("summary_cases_confirmed.json")}
    transfer = read("summary_transfer.json")
    original = json.loads((OUT.parent / "icra_external_fusion/summary_v2v4real.json").read_text())
    assert all(d["sequences"] == 9 for d in dev.values())
    assert dev["ir"]["audited_node_frames"] == 15944
    assert dev["ir"]["original_er_bitwise_parity_node_frames"] == 7972
    assert all(dev[k]["audited_node_frames"] == 7972 for k in ["cr", "cgr"])
    assert all(d["audited_node_frames"] == 57600 for d in cases.values())
    assert transfer["audited_node_frames"] == 48128 and transfer["primary_reserved_frames"] == 1357

    # Verify every saved source snapshot without regenerating a freeze manifest.
    manifests = {}
    for path in sorted(OUT.glob("source_sha256*.json")):
        data = json.loads(path.read_text())
        for name, digest in data.items():
            assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
        manifests[path.name] = len(data)

    real_rows = []
    lookup = {(r["sequence"], r["condition"], r["arm"]): r for r in original["runs"]}
    for d in dev.values():
        for r in d["runs"]:
            lookup[r["sequence"], r["condition"], r["arm"]] = r
    for r in lookup.values():
        real_rows.append(dict(cohort="development", **r))
    for r in transfer["runs"]:
        real_rows.append(dict(cohort="reserved" if r["role"] == "reserved" else "overlap_control", **r))
    real_rows.sort(key=lambda r: (r["cohort"], r["sequence"], r["condition"], ARMS.index(r["arm"])))
    fields = ["cohort", "sequence", "condition", "arm", "frames", "ospa", "gospa", "loc2",
              "miss2", "false2", "countError", "matched_sse", "matched_support", "raw_bytes", "wire_bytes"]
    with (OUT / "report_real_runs.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader(); writer.writerows(real_rows)

    aggregates = {}
    for condition in CONDITIONS:
        for arm in ARMS:
            rows = [r for d in dev.values() for r in d["aggregate"]
                    if r["condition"] == condition and r["arm"] == arm]
            assert rows, (condition, arm)
            aggregates["development", condition, arm] = rows[0]
    for r in transfer["aggregate"]:
        aggregates[r["cohort"], r["condition"], r["arm"]] = r

    primary_table = []
    for arm in ARMS:
        values = [fmt(aggregates[c, condition, arm]["ospa"]["mean"])
                  if (c, condition, arm) in aggregates else "未进入预留检验"
                  for c in ["development", "reserved"] for condition in CONDITIONS]
        primary_table.append([NAMES[arm]] + values)

    paired = []
    for key, d in dev.items():
        for r in d["paired"]:
            paired.append(dict(cohort="development", candidate=d["candidate"], **r))
    paired.extend(transfer["paired"])
    main_pairs = [r for r in paired if r["cohort"] in ["development", "reserved"]
                  and r["candidate"] in CANDIDATES and r["reference"] in ["lineage", "qualified_exist"]]
    pair_table = [["开发" if r["cohort"] == "development" else "预留", CONDITIONS[r["condition"]],
                   NAMES[r["candidate"]], NAMES[r["reference"]], ci(r), f"{r['ospa_wins']}/{r['ospa']['n']}"]
                  for r in main_pairs]
    errors = []
    for cohort in ["development", "reserved"]:
        for condition in CONDITIONS:
            for arm in ["lineage", "qualified_exist"] + CANDIDATES:
                r = aggregates[cohort, condition, arm]
                errors.append(["开发" if cohort == "development" else "预留", CONDITIONS[condition],
                               NAMES[arm]] + [fmt(r[k]["mean"]) for k in ["gospa", "loc2", "miss2", "false2", "countError"]])
    case_table = []
    for scene in SCENES:
        for arm in ["lineage", "qualified_exist"] + CANDIDATES:
            rows = [r for d in cases.values() for r in d["aggregate"] if r["scene"] == scene and r["arm"] == arm]
            r = rows[0]
            case_table.append([SCENES[scene], NAMES[arm]] + [fmt(r[k]["mean"]) for k in ["ospa", "miss2", "false2", "reunion_ospa", "remote_acquisitions"]])
    case_pairs = []
    for key, d in cases.items():
        for r in d["paired"]:
            if r["reference"] in ["lineage", "qualified_exist"]:
                case_pairs.append(dict(candidate=CANDIDATES[0 if key == "cr" else 1], **r))
    case_pair_table = [[SCENES[r["scene"]], NAMES[r["candidate"]], NAMES[r["reference"]], ci(r),
                        f"{r['ospa_wins']}/20"] for r in case_pairs]

    per_sequence = []
    for cohort, seqs in [("development", range(9)), ("reserved", [5, 10, 15, 20, 25, 30]), ("overlap_control", [0])]:
        for seq in seqs:
            for condition in CONDITIONS:
                group = {r["arm"]: r for r in real_rows if r["cohort"] == cohort and r["sequence"] == f"{seq:04d}" and r["condition"] == condition}
                per_sequence.append([{"development": "开发", "reserved": "预留", "overlap_control": "重叠控制"}[cohort],
                                     f"{seq:04d}", CONDITIONS[condition], group["lineage"]["frames"]] +
                                    [fmt(group[a]["ospa"]) if a in group else "—" for a in ARMS])

    common_table = []
    for condition in CONDITIONS:
        for candidate in CANDIDATES:
            for reference in ["lineage", "qualified_exist"]:
                rows = [r for r in transfer["common_target"] if r["role"] == "reserved" and r["condition"] == condition
                        and r["candidate"] == candidate and r["reference"] == reference]
                support = sum(r["support"] for r in rows)
                common_table.append([CONDITIONS[condition], NAMES[candidate], NAMES[reference], support,
                                     fmt(math.sqrt(sum(r["candidate_sse"] for r in rows) / support)),
                                     fmt(math.sqrt(sum(r["reference_sse"] for r in rows) / support))])

    source_names = ["summary_development.json", "summary_conservative.json", "summary_confirmed.json",
                    "summary_cases.json", "summary_cases_confirmed.json", "summary_transfer.json"]
    report = dict(source_sha256={n: hashlib.sha256((OUT / n).read_bytes()).hexdigest() for n in source_names},
                  manifests=manifests, audited_new_node_frames=195216,
                  aggregate=[dict(cohort=c, **r) for (c, _, _), r in aggregates.items() if "cohort" not in r] + transfer["aggregate"],
                  paired=paired, cases={k: d["aggregate"] for k, d in cases.items()}, case_paired=case_pairs,
                  case_runs={k: d["runs"] for k, d in cases.items()}, real_runs=real_rows)
    (OUT / "report_data.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    blocks = [
        "# 方法迭代记录：限制时效造成的正向存在概率放大\n\n2026-09-08。三版规则、完整负结果与预留检验均保留；本报告由已通过独立指标重算的结果生成。",
        "## 本轮结论\n\n" + (OUT / "report_conclusion.md").read_text().strip(),
        "## 1. 改了什么，以及为什么\n\n原 ER 对整个后验的存在对数几率赋予时效权重。整个后验包含历史传入信息、先验和本轮量测；直接感知机会较新，并不等于一个正向存在声明已经得到可靠的新量测支持。\n\n先测试 IR：只让本轮本地更新增量承担时效修正。开发结果未修复问题，因此没有把最初的机制猜想写成已证实原因。随后测试 CR 的非正向约束，再测试 CGR 的局部确认例外。三次假设、检查和范围修订分别记录在 [PROTOCOL.md](PROTOCOL.md)、[AMENDMENT_CR.md](AMENDMENT_CR.md)、[AMENDMENT_CGR.md](AMENDMENT_CGR.md)。\n\nCR 与 CGR 沿用原有检测、出生、运动、匹配、裁剪、剪枝、MAP 基数提取、通信日程和年龄核；不搜索衰减常数、门限或序列子集。",
        r"""## 2. 固定输入的形式与边界

对一个已经对齐的 Bernoulli 标签，$\mathcal E$ 是原逻辑判定可参与存在融合的源，$b_j$ 是普通权重在该集合上的归一化，$a_j$ 是原空间权重。沿用

$$f_j=0.25+0.75e^{-\mathrm{age}_j/5\mathrm{s}},\qquad q_j=\frac{b_jf_j}{\sum_k b_kf_k},$$

无有效本地时间戳的已表示源使用 $f_j=0.25$；缺标签删失输入沿用 $f_j=1$，其存在上界和资格仍由原有可见域逻辑提供，本地增量为零。定义

$$p_a(x)=\eta_a^{-1}\prod_jp_j(x)^{a_j},\quad z_0=\sum_{j\in\mathcal E}b_j\operatorname{logit}(r_j)+\log\eta_a,\quad z_E=\sum_{j\in\mathcal E}q_j\operatorname{logit}(r_j)+\log\eta_a.$$

$r_0=\sigma(z_0)$ 和 $r_E=\sigma(z_E)$ 都来自**同一组当前递归输入**。它们与另行递归运行的无时效/ER 基线不是同一个反事实。

**IR：本地增量。** 每次本地更新重新计算 $\delta_j=\operatorname{logit}(r_j^+)-\operatorname{logit}(r_j^-)$，删失输入取零，

$$r_{IR}=\sigma\!\left(z_0+\sum_j(q_j-b_j)\delta_j\right).$$

这是近似 LMB 更新后的有效增量，不是经校准且跨源独立的似然比。新增一个 float64/目标；实际 4 维高斯包从 208 变成 216 字节/目标，另有 32 字节包头。

**CR：严格约束。**

$$r_{CR}=\min(r_0,r_E),\qquad p_{CR}=p_a.$$

这可从原 ER 的固定输入目标导出：最小化 $\sum_jq_jD_B(r\|r_j)+r\sum_ja_jD(p\|p_j)$，另加 $r\le r_0$。空间最优解仍为 $p_a$；标量目标导数为 $\operatorname{logit}(r)-z_E$，二阶导数为 $1/[r(1-r)]>0$，所以约束最优解是上述最小值。新增约束是设计选择，不是 Bayes 模型强制要求。

**CGR：确认门控。** 存在一个可参与的已表示源，同时满足 $q_j>b_j+10^{-12}$、$r_j\ge0.5$，以及连续两次本地更新的检测关联概率质量均至少 0.5，才允许 $r_E>r_0$；否则使用 CR。所有负向修正保留。

两次本地更新均须有直接感知机会和非空量测。漏检、无机会、空量测或标签本地更新中断都会清空连续计数。计数和上次更新时间留在本地；当前确认位使用原来已经分配的包字段，接收到的确认位不能推进本地计数。融合后恢复接收端自己的当前确认位。这里没有永久、可转传的确认凭证；持续误检仍然可能通过门控。两次/0.5 关联标准来自此前确认消融，未在预留数据上拟合。

三版只在固定当前输入下保持空间密度相同；递归时存在概率会影响后续关联、剪枝和提取。$r_{CR}\le r_0$ 不能推出递归 OSPA、误检数或漏检数必然下降。CR/CGR 无新增包字段或可调衰减参数。
""",
        "## 3. 数据与统计单位\n\n- 开发集：此前已看过的 9 个 V2V4Real 发布评估序列，1,993 帧、两车、两种链路。它们已参与方法诊断，不能称作本轮独立测试集。\n- 预留算法迁移集：先按 ID 每五个取一个，选定 train 的 0000/0005/0010/0015/0020/0025/0030。运行前发现 0000 真值文件与开发 0000 完全相同，因此保留为单独重叠控制；主要结果使用其余六个序列，共 1,357 帧。未用跟踪输出剔除或替换序列。对全部 32 个 train 和 9 个 val 做过几何指纹检查，没有发现其他精确帧重复；这不能证明道路/行程独立。\n- 检测器是在原 train 划分上训练的。预留仅指本轮融合方法未使用这些序列的跟踪结果，不代表新的检测测试基准。\n- 模拟机制集：原有三类事件，每类 20 个固定种子，共 60 个缓存输入。CR/CGR 各运行一次，旧对照直接复用已冻结的配对结果。\n- 真实回放仍为平面坐标、相对自车 CV 近似、固定名义检测概率与仿真通信。没有新增 3D 跟踪基准或完整 DMSTrack 方法复现。MIL-AM 和 TC 的共同后端适配边界沿用[外部实验报告](../../papers/icra2027/EXTERNAL_DATASET_RESULTS_CN.md)。\n\nOSPA 使用 $p=2,c=12$ m；GOSPA 使用 $p=2,c=12$ m、$\\alpha=2$。误检/漏检/定位分量是平方代价，单位 m²。先平均每序列所有节点帧，再对序列等权平均；模拟按种子等权。配对差值为候选减参照，负值更好。95% 区间是按序列或种子的 10,000 次 percentile bootstrap，固定种子 8301，属描述性区间；未做多重比较校正，不把相关帧当独立样本。",
        "## 4. 真实数据：所有方案的平均 OSPA（m）\n\n" + table(["方案", "开发/可靠 n=9", "开发/间歇 n=9", "预留/可靠 n=6", "预留/间歇 n=6"], primary_table),
        "![完整方法比较](figures/method_iteration.png)\n\n图中所有浅色点是完整序列或种子单位，粗点/横线是均值和描述性 95% 区间；面板 d 为预留集误检/漏检平方代价相对无时效的变化。",
        "## 5. 与自身对照的配对差值\n\n胜出序列数仅为逐序列 OSPA 严格下降的计数；不作显著性检验。\n\n" + table(["数据", "链路", "候选", "参照", "ΔOSPA [95% 区间] (m)", "胜出"], pair_table),
        "## 6. 误差分解：改善的来源与代价\n\n" + table(["数据", "链路", "方案", "GOSPA m", "定位 m²", "漏检 m²", "误检 m²", "基数绝对误差"], errors),
        "预留集的定位另在两方案共同匹配到的真值实例上比较，避免只看各自幸存轨迹的 RMSE。此表是共同目标实例的 pooled RMSE，不是序列宏平均；支持量随比较而变化。\n\n" + table(["链路", "候选", "参照", "共同实例数", "候选 RMSE m", "参照 RMSE m"], common_table),
        "## 7. 原有机制场景：不能隐去的退步\n\n重连 OSPA 为原协议的重连后十帧窗口；远端发现数是每次试验平均成功数量，无新目标控制中应为零。\n\n" + table(["场景 n=20", "方案", "OSPA m", "漏检 m²", "误检 m²", "重连 OSPA m", "远端发现数"], case_table) + "\n\n" + table(["场景", "候选", "参照", "ΔOSPA [95% 区间] (m)", "胜出"], case_pair_table),
        "## 8. 核验与复现\n\n全部新增完整结果合计 **195,216 个节点帧**，使用 Python/SciPy 从保存的状态独立重算指标；小规模指派另与穷举解对照。它是同一工作流程中的实现交叉检查，不是第三方独立复现。\n\n解析检查覆盖固定输入最优解、空间密度不变、单源/等龄/零增量恒等、删失与未观测先验排除、包内容、确认位不由远端计数推进，以及缺失感知的计数重置。加速后的原 ER 在全部 7,972 个真实开发节点帧与旧状态输出逐元素完全一致；已完成慢版与快版的 IR/ER 共 4,824、CR 共 2,412 个节点帧也完全一致。\n\n密集序列的 Hungarian 搜索热点做了试验目录内的等价向量化，保留行优先零元素和 tie break，35 个矩阵用例的匹配和代价与旧实现完全一致。保留被中止的慢版和性能诊断，均不混入完整结果。运行时修订见 [AMENDMENT_RUNTIME.md](AMENDMENT_RUNTIME.md)；本轮不据此声称方法本身获得计算优势。\n\n完整预留数据/方法的 1,222 文件散列在跟踪前冻结，含两版完整真实开发汇总。原有 1,182 个源码/协议散列未变。全部保存的源快照再次核验通过。\n\n可直接重算审计与本报告（从仓库根目录，Python 需 NumPy/SciPy/Matplotlib；本机使用 `tmp/external_baselines/venv/bin/python`）：\n\n```sh\npython3 trials/icra_external_fusion/fetch_dependencies.py\npython3 trials/icra_method_iteration/analyze_development.py --candidate ir --accelerated\npython3 trials/icra_method_iteration/analyze_development.py --candidate cr --accelerated\npython3 trials/icra_method_iteration/analyze_development.py --candidate cgr --accelerated\npython3 trials/icra_method_iteration/analyze_cases.py --candidate cr\npython3 trials/icra_method_iteration/analyze_cases.py --candidate cgr\npython3 trials/icra_method_iteration/analyze_transfer.py\npython3 trials/icra_method_iteration/build_iteration_report.py\npython3 trials/icra_method_iteration/plot_iteration.py\n```\n\n紧凑输入和完整结果均纳入版本管理。已保存的真实输入位于原目录 `../icra_external_fusion/data/` 及本目录 `data_transfer/`；60 个配对模拟输入来自原目录的已跟踪 MAT 缓存。完整跟踪重跑入口和日志说明见 [REPRODUCE.md](REPRODUCE.md)。冻结清单不应因源码差异而重新生成；`make_*.py`/`freeze_sources.py` 是本次开发构建记录，不属于复现入口。\n\n结果文件：`summary_*.json` 保留全部聚合、配对差值与输入散列；`report_real_runs.csv` 收齐真实数据所有方案；`report_data.json` 是图表数据；`results_*/*.json.gz` 保存逐帧输出。当前 CGR 模拟日志的 START 行沿用生成器中的 CR 文本，实际结果 `arm=confirmation_recency` 已核验。\n\n本报告对应新方法试验；主论文 PDF 仍对应此前冻结的 ER 实验版本。",
        "## 9. 每一个真实序列的结果\n\n包括被排除出主要汇总的重叠控制，开发编号与预留编号属于不同发布划分。全部数值为 OSPA（m）。\n\n" + table(["数据", "编号", "链路", "帧数"] + [NAMES[a] for a in ARMS], per_sequence),
    ]
    (OUT / "README_CN.md").write_text("\n\n".join(blocks) + "\n")
    print("REPORT BUILT: 195216 audited node-frames;", len(real_rows), "real run rows; all", len(manifests), "source snapshots unchanged.")


if __name__ == "__main__":
    main()
