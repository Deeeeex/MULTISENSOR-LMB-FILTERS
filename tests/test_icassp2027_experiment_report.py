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


def test_report_carries_plain_language_story_and_boundaries() -> None:
    text = REPORT.read_text()
    plain_section = text.split("## 0. 面向非专业听众的汇报主线", 1)[1].split(
        "## 1. 一屏总览", 1
    )[0]

    for marker in (
        "## 0. 面向非专业听众的汇报主线",
        "### 0.1 这项工作要解决什么问题",
        "### 0.2 我们的方法是怎么想到的",
        "### 0.3 我们如何确认结果可信",
        "### 0.4 实验结果说明了什么",
        "### 0.5 汇报时必须主动说明的边界",
        "### 0.6 一分钟口头汇报版本",
        "先把十页材料寄给对方，对方收到后再按固定模板整理成一页摘要",
        "系统实际编码并准备发送的数据量平均减少 **58.28%**",
        "超过 111 万条保留目标记录逐项完全一致",
        "不能直接换算成无线能耗或时延收益",
        "以下为技术复核区",
    ):
        assert marker in text

    assert "可执行证书" not in text
    assert "executable certificate" not in plain_section
    assert r"\mathcal" not in plain_section
    assert "`" not in plain_section
    assert "应用层" not in plain_section
    for acronym in ("LMB", "GM", "KLA"):
        assert acronym not in plain_section
