from __future__ import annotations

import csv
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "docs" / "icassp2027_paper" / "EXPERIMENT_RESULTS_CN.md"
BIB_PATH = ROOT / "docs" / "icassp2027_paper" / "refs.bib"
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
    plain_section = text.split("## 研究背景、核心思路与主要结论", 1)[1].split(
        "## 1. 一屏总览", 1
    )[0]

    for marker in (
        "## 研究背景、核心思路与主要结论",
        "### 研究问题",
        "本文评估的原型软件采用一个由作者实现的特定接收端",
        "先根据目标编号对齐不同来源",
        "不代表实际部署的多传感器系统或其他算法通常都这样工作",
        "### 相关工作与本文区别",
        "### 方法设计思路",
        "### 结果可信性",
        "### 主要实验结果",
        "### 结论的适用范围",
        "### 核心内容概述",
        "先把十页材料寄给对方，对方收到后再按固定模板整理成一页摘要",
        "相同的核心摘要 + 相同的后续处理 = 相同的接收结果",
        "可以按照摘要把所有详细描述分组",
        "这并不表示两份详细描述本身完全相同",
        "调整发送时机",
        "选择部分候选信息",
        "直接交换紧凑的目标状态",
        "归纳多个候选分布",
        "重新设计多节点融合规则",
        "本文固定现有融合方式，只重新设计输入消息",
        "系统实际编码并准备发送的数据量平均减少 **58.28%**",
        "超过 111 万条保留目标记录逐项完全一致",
        "不能直接换算成无线能耗或时延收益",
    ):
        assert marker in text

    assert "可执行证书" not in text
    assert "executable certificate" not in plain_section
    assert r"\mathcal" not in plain_section
    assert "`" not in plain_section
    assert "应用层" not in plain_section
    for acronym in ("LMB", "GM", "KLA"):
        assert acronym not in plain_section
    for meta_term in ("汇报", "非专业", "听众", "阅读建议", "技术复核区"):
        assert meta_term not in text
    for overgeneralization in (
        "我们检查现有系统后发现",
        "每个节点过去会",
        "可以把原来的过程理解为",
        "现有接收端",
    ):
        assert overgeneralization not in plain_section


def test_plain_related_work_is_bound_to_paper_bibliography() -> None:
    bib = BIB_PATH.read_text()

    for key in (
        "Shen2022EventTriggeredLMB",
        "Li2023EventTriggeredConsensusLMB",
        "Li2026EventTriggeredMdeltaGLMB",
        "Li2019PartialConsensusGMPHD",
        "Xue2026StateCovProjection",
        "Runnalls2007GMR",
        "Battistelli2014KLA",
        "Wang2017MBGCI",
        "Gao2020MIL",
        "Wei2024AALMB",
    ):
        assert f"{{{key}," in bib
