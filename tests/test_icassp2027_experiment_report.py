from __future__ import annotations

import csv
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "docs" / "icassp2027_paper" / "EXPERIMENT_RESULTS_CN.md"
CSV_PATH = (
    ROOT
    / "RUN"
    / "GA"
    / "GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv"
)


ROW_PATTERN = re.compile(
    r"^\|\s*(?P<seed>\d+)\s*\|\s*"
    r"(?P<full_attempted>\d+\.\d{3})\s*\|\s*"
    r"(?P<moment_attempted>\d+\.\d{3})\s*\|\s*"
    r"(?P<attempted_reduction>\d+\.\d{3})\s*\|\s*"
    r"(?P<full_delivered>\d+\.\d{3})\s*\|\s*"
    r"(?P<moment_delivered>\d+\.\d{3})\s*\|\s*"
    r"(?P<delivered_reduction>\d+\.\d{3})\s*\|\s*"
    r"(?P<comparisons>[\d,]+)\s*\|\s*(?P<exact>Yes)\s*\|$"
)


def test_report_is_bound_to_frozen_csv() -> None:
    text = REPORT.read_text()
    parsed_rows = {}
    for line in text.splitlines():
        match = ROW_PATTERN.match(line)
        if match:
            parsed_rows[int(match.group("seed"))] = match.groupdict()

    with CSV_PATH.open(newline="") as handle:
        csv_rows = list(csv.DictReader(handle))

    assert len(csv_rows) == 50
    assert sorted(parsed_rows) == list(range(82, 132))

    for row in csv_rows:
        seed = int(row["seed"])
        shown = parsed_rows[seed]
        assert shown["full_attempted"] == f"{float(row['full_attempted_bytes']) / 1e6:.3f}"
        assert shown["moment_attempted"] == f"{float(row['moment_attempted_bytes']) / 1e6:.3f}"
        assert shown["attempted_reduction"] == f"{float(row['attempted_reduction_percent']):.3f}"
        assert shown["full_delivered"] == f"{float(row['full_delivered_bytes']) / 1e6:.3f}"
        assert shown["moment_delivered"] == f"{float(row['moment_delivered_bytes']) / 1e6:.3f}"
        assert shown["delivered_reduction"] == f"{float(row['delivered_reduction_percent']):.3f}"
        assert int(shown["comparisons"].replace(",", "")) == int(row["comparison_count"])
        assert shown["exact"] == "Yes"
        assert int(row["exact_match"]) == 1

    for marker in (
        "Receiver-Induced Moment Exchange for Distributed LMB Fusion",
        "58.277264%",
        "57.923222%, 58.636095%",
        "1,119,037 retained matched label instances",
        "self `1/3`、四邻居各 `1/6`",
        "raw pre-retention equality 由命题给出",
    ):
        assert marker in text


def test_report_carries_receiver_first_story_and_boundaries() -> None:
    text = REPORT.read_text()

    for marker in (
        "## 0. 论文主叙事：由接收端反推消息接口",
        "### 0.1 背景：发送内容与接收端实际使用的信息不匹配",
        "### 0.2 方法设计：把首个不可逆映射移到发送端",
        "### 0.3 设计思想：从“少发字段”升级为“可执行证书”",
        r"\mathcal F_{\omega,\mathcal R}",
        r"\mathcal P\circ\mathcal T=\mathcal P",
        "Receiver-first",
        "executed receiver-output equivalence",
        "不声称 full GM 与 moment message 表示相同的 mixture density",
        "不推断 radio energy、latency、airtime、rate optimality",
    ):
        assert marker in text
