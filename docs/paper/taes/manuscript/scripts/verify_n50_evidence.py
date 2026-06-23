#!/usr/bin/env python3
"""Independent checks for the paper-facing N50 evidence package.

This verifier intentionally recomputes what it can from raw per-trial artifacts:

* network disagreement means, confidence intervals, paired reductions, wins, and
  sign-test p-values from the per-trial Markdown table;
* local E-OSPA/RMSE/CardErr means and paired reductions when the validation
  report contains per-trial local tracking rows;
* runtime means/stds/relative costs from the trial log;
* generated manuscript fragments against the tracked report hash.

Archived reports that predate the per-trial local table remain supported. For
those reports, local E-OSPA/RMSE/CardErr are explicitly marked summary-traced.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
OUT = ROOT / "generated"
REPORT = REPO / "RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md"
LOG = REPO / "RUN/AA/AA_NEIGHBORHOOD_LABEL_BARYCENTER_N50_SEED1_20260622_174817.log"
EVIDENCE_JSON = OUT / "n50_evidence.json"

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
RUNTIME_LABEL = {
    "Tuned spatial-KLA AA": "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA": "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA": "Neighborhood reference-only label-consensus spatial-KLA AA",
}
EXPECTED_TRIALS = 50


@dataclass(frozen=True)
class SummaryStats:
    mean: float
    std: float
    ci_low: float
    ci_high: float
    n: int


@dataclass(frozen=True)
class PairedStats:
    reduction_mean: float
    reduction_std: float
    ci_low: float
    ci_high: float
    reduction_pct: float
    wins: int
    n: int
    p_value: float


def read_text(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"Missing required artifact: {path}")
    return path.read_text(encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def markdown_table_after(text: str, heading: str) -> list[list[str]]:
    idx = text.find(heading)
    if idx < 0:
        raise ValueError(f"Missing heading: {heading}")
    lines = text[idx:].splitlines()
    table_lines: list[str] = []
    in_table = False
    for line in lines[1:]:
        if line.startswith("|"):
            table_lines.append(line)
            in_table = True
        elif in_table:
            break
    if len(table_lines) < 3:
        raise ValueError(f"No Markdown table after {heading}")
    rows: list[list[str]] = []
    for line in table_lines:
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if all(set(part) <= {":", "-"} for part in parts):
            continue
        rows.append(parts)
    return rows


def markdown_table_after_optional(text: str, heading: str) -> list[list[str]] | None:
    if text.find(heading) < 0:
        return None
    return markdown_table_after(text, heading)


def parse_network_trials(report_text: str) -> dict[str, dict[str, list[float]]]:
    rows = markdown_table_after(report_text, "## Per-Trial Network Disagreement Metrics")
    expected = ["Trial", "Seed", "Arm", *NETWORK_METRICS]
    if rows[0] != expected:
        raise ValueError(f"Unexpected network trial header: {rows[0]}")
    values = {arm: {metric: [] for metric in NETWORK_METRICS} for arm in ARM_ORDER}
    for row in rows[1:]:
        arm = row[2]
        if arm not in values:
            raise ValueError(f"Unexpected arm in per-trial network table: {arm}")
        for metric, value in zip(NETWORK_METRICS, row[3:]):
            values[arm][metric].append(float(value))
    for arm in ARM_ORDER:
        for metric in NETWORK_METRICS:
            if len(values[arm][metric]) != EXPECTED_TRIALS:
                raise ValueError(
                    f"Expected {EXPECTED_TRIALS} rows for {arm}/{metric}, found {len(values[arm][metric])}"
                )
    return values


def parse_local_trials(report_text: str) -> dict[str, dict[str, list[float]]] | None:
    rows = markdown_table_after_optional(report_text, "## Per-Trial Local Tracking Metrics")
    if rows is None:
        return None
    expected = ["Trial", "Seed", "Arm", *LOCAL_METRICS]
    if rows[0] != expected:
        raise ValueError(f"Unexpected local trial header: {rows[0]}")
    values = {arm: {metric: [] for metric in LOCAL_METRICS} for arm in ARM_ORDER}
    for row in rows[1:]:
        arm = row[2]
        if arm not in values:
            raise ValueError(f"Unexpected arm in per-trial local table: {arm}")
        for metric, value in zip(LOCAL_METRICS, row[3:]):
            values[arm][metric].append(float(value))
    for arm in ARM_ORDER:
        for metric in LOCAL_METRICS:
            if len(values[arm][metric]) != EXPECTED_TRIALS:
                raise ValueError(
                    f"Expected {EXPECTED_TRIALS} rows for {arm}/{metric}, found {len(values[arm][metric])}"
                )
    return values


def parse_network_summary(report_text: str) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(report_text, "## Network Disagreement Metrics")
    if rows[0] != ["Arm", *NETWORK_METRICS]:
        raise ValueError(f"Unexpected network summary header: {rows[0]}")
    summary = {row[0]: {metric: float(value) for metric, value in zip(NETWORK_METRICS, row[1:])} for row in rows[1:]}
    require_arms(summary, "network summary")
    return summary


def parse_local_summary(report_text: str) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(report_text, "## Local Tracking Metrics")
    if rows[0] != ["Arm", *LOCAL_METRICS]:
        raise ValueError(f"Unexpected local summary header: {rows[0]}")
    summary = {row[0]: {metric: float(value) for metric, value in zip(LOCAL_METRICS, row[1:])} for row in rows[1:]}
    require_arms(summary, "local summary")
    return summary


def require_arms(table: dict[str, object], context: str) -> None:
    missing = [arm for arm in ARM_ORDER if arm not in table]
    if missing:
        raise ValueError(f"Missing expected arms in {context}: {missing}")


def parse_runtime_summary(report_text: str) -> dict[str, dict[str, str]]:
    rows = markdown_table_after(report_text, "## Runtime")
    expected = ["Arm", "Filter runtime (s)", "Runtime/step (s)", "Relative to Tuned spatial-KLA AA", "N"]
    if rows[0] != expected:
        raise ValueError(f"Unexpected runtime header: {rows[0]}")
    return {
        row[0]: {
            "runtime": row[1],
            "runtime_step": row[2],
            "relative": row[3],
            "n": row[4],
        }
        for row in rows[1:]
    }


def parse_log_runtimes(log_text: str) -> dict[str, list[float]]:
    runtimes = {arm: [] for arm in ARM_ORDER}
    current_arm: str | None = None
    for line in log_text.splitlines():
        arm_match = re.search(r"Arm\s+\d+/\d+:\s+(.+)$", line)
        if arm_match:
            current_arm = arm_match.group(1).strip()
            continue
        runtime_match = re.search(r"Filter runtime:\s+([0-9.]+)\s+s", line)
        if runtime_match:
            if current_arm not in runtimes:
                raise ValueError(f"Runtime without expected arm context: {line}")
            runtimes[current_arm].append(float(runtime_match.group(1)))
            current_arm = None
    for arm in ARM_ORDER:
        if len(runtimes[arm]) != EXPECTED_TRIALS:
            raise ValueError(f"Expected {EXPECTED_TRIALS} runtime rows for {arm}, found {len(runtimes[arm])}")
    return runtimes


def summarize(values: list[float]) -> SummaryStats:
    if not values:
        raise ValueError("Cannot summarize an empty vector")
    n = len(values)
    mean = statistics.fmean(values)
    std = statistics.stdev(values) if n > 1 else 0.0
    half_width = 1.96 * std / math.sqrt(n) if n > 1 else 0.0
    return SummaryStats(mean=mean, std=std, ci_low=mean - half_width, ci_high=mean + half_width, n=n)


def sign_test_pvalue(deltas: list[float]) -> float:
    nonzero = [delta for delta in deltas if delta != 0]
    n = len(nonzero)
    if n == 0:
        return 1.0
    wins = sum(delta > 0 for delta in nonzero)
    tail = min(wins, n - wins)
    prob = sum(math.comb(n, k) for k in range(0, tail + 1)) / (2**n)
    return min(1.0, 2.0 * prob)


def paired_stats(baseline: list[float], candidate: list[float]) -> PairedStats:
    if len(baseline) != len(candidate):
        raise ValueError("Paired vectors must have the same length")
    deltas = [b - c for b, c in zip(baseline, candidate)]
    delta_stats = summarize(deltas)
    baseline_mean = summarize(baseline).mean
    pct = 100.0 * delta_stats.mean / baseline_mean if abs(baseline_mean) > 0 else float("nan")
    return PairedStats(
        reduction_mean=delta_stats.mean,
        reduction_std=delta_stats.std,
        ci_low=delta_stats.ci_low,
        ci_high=delta_stats.ci_high,
        reduction_pct=pct,
        wins=sum(delta > 0 for delta in deltas),
        n=len(deltas),
        p_value=sign_test_pvalue(deltas),
    )


def parse_mean_std(value: str) -> tuple[float, float]:
    match = re.fullmatch(r"\s*([0-9.]+)\s*\+/-\s*([0-9.]+)\s*", value)
    if not match:
        raise ValueError(f"Could not parse mean/std: {value}")
    return float(match.group(1)), float(match.group(2))


def assert_close(label: str, observed: float, expected: float, tol: float = 5e-4) -> None:
    if abs(observed - expected) > tol:
        raise AssertionError(f"{label}: observed {observed:.9f}, expected {expected:.9f}, tol {tol}")


def verify_network(report_text: str) -> dict[str, object]:
    raw = parse_network_trials(report_text)
    summary = parse_network_summary(report_text)
    verification: dict[str, object] = {"means": {}, "paired": {}}
    for arm in ARM_ORDER:
        arm_stats = {}
        for metric in NETWORK_METRICS:
            stats = summarize(raw[arm][metric])
            assert_close(f"network mean {arm}/{metric}", stats.mean, summary[arm][metric], tol=5e-7)
            arm_stats[metric] = stats.__dict__
        verification["means"][arm] = arm_stats
    baseline = ARM_ORDER[0]
    for arm in ARM_ORDER[1:]:
        arm_paired = {}
        for metric in NETWORK_METRICS:
            stats = paired_stats(raw[baseline][metric], raw[arm][metric])
            arm_paired[metric] = stats.__dict__
        verification["paired"][arm] = arm_paired
    return verification


def verify_local(report_text: str) -> dict[str, object]:
    raw = parse_local_trials(report_text)
    summary = parse_local_summary(report_text)
    if raw is None:
        return {
            "status": "summary_traced",
            "reason": "The archived report does not include per-trial local tracking rows.",
            "summary": summary,
        }

    verification: dict[str, object] = {"status": "independent", "means": {}, "paired": {}}
    for arm in ARM_ORDER:
        arm_stats = {}
        for metric in LOCAL_METRICS:
            stats = summarize(raw[arm][metric])
            assert_close(f"local mean {arm}/{metric}", stats.mean, summary[arm][metric], tol=1e-6)
            arm_stats[metric] = stats.__dict__
        verification["means"][arm] = arm_stats

    baseline = ARM_ORDER[0]
    for arm in ARM_ORDER[1:]:
        arm_paired = {}
        for metric in LOCAL_METRICS:
            stats = paired_stats(raw[baseline][metric], raw[arm][metric])
            arm_paired[metric] = stats.__dict__
        verification["paired"][arm] = arm_paired
    return verification


def verify_runtime(report_text: str, log_text: str) -> dict[str, object]:
    report_runtime = parse_runtime_summary(report_text)
    raw = parse_log_runtimes(log_text)
    baseline_values = raw[ARM_ORDER[0]]
    verification: dict[str, object] = {}
    for arm in ARM_ORDER:
        stats = summarize(raw[arm])
        report_mean, report_std = parse_mean_std(report_runtime[arm]["runtime"])
        assert_close(f"runtime mean {arm}", stats.mean, report_mean, tol=1e-3)
        assert_close(f"runtime std {arm}", stats.std, report_std, tol=1e-3)
        if arm == ARM_ORDER[0]:
            ratio_mean = 1.0
        else:
            ratios = [value / base for value, base in zip(raw[arm], baseline_values)]
            ratio_mean = summarize(ratios).mean
        report_ratio = float(report_runtime[arm]["relative"].rstrip("x"))
        assert_close(f"runtime ratio {arm}", ratio_mean, report_ratio, tol=5e-4)
        verification[arm] = {
            "mean": stats.mean,
            "std": stats.std,
            "n": stats.n,
            "relative": ratio_mean,
        }
    return verification


def verify_generated_hash() -> dict[str, str]:
    if not EVIDENCE_JSON.exists():
        raise FileNotFoundError(f"Run extract_n50_evidence.py first: {EVIDENCE_JSON}")
    data = json.loads(EVIDENCE_JSON.read_text(encoding="utf-8"))
    expected_hash = sha256(REPORT)
    if data.get("source_sha256") != expected_hash:
        raise AssertionError(
            f"Generated evidence hash mismatch: {data.get('source_sha256')} != {expected_hash}"
        )
    return {
        "source_report": data.get("source_report", ""),
        "source_sha256": expected_hash,
    }


def write_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "n50_verification.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    network = payload["network"]
    local = payload["local"]
    runtime = payload["runtime"]
    local_is_independent = local["status"] == "independent"
    if local_is_independent:
        local_note = "independently recomputed from per-trial local tracking rows."
    else:
        local_note = (
            "summary-traced only because the archived validation report does not "
            "include per-trial local metric rows."
        )
    lines = [
        "# N50 Evidence Verification Report\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/verify_n50_evidence.py`.\n\n",
        f"- Source report: `{payload['source']['source_report']}`\n",
        f"- Source SHA256: `{payload['source']['source_sha256']}`\n",
        "- Network metrics: independently recomputed from the per-trial Markdown table.\n",
        "- Runtime metrics: independently recomputed from the trial log.\n",
        f"- Local metrics: {local_note}\n\n",
        "## Verified Checks\n\n",
        "- Network disagreement means match the report summary to within `5e-7`.\n",
        "- Runtime means/stds match the report summary to within `1e-3` seconds.\n",
        "- Runtime relative costs match the report summary to within `5e-4`.\n",
        "- Generated manuscript evidence JSON references the current report SHA256.\n\n",
        "## Recomputed Network Highlights\n\n",
    ]
    full_network = network["means"]["Neighborhood label-barycenter spatial-KLA AA"]
    base_network = network["means"]["Tuned spatial-KLA AA"]
    paired_full = network["paired"]["Neighborhood label-barycenter spatial-KLA AA"]
    lines.extend(
        [
            f"- Network OSPA mean: full `{full_network['OSPA']['mean']:.6f}` vs tuned `{base_network['OSPA']['mean']:.6f}`.\n",
            f"- Network OSPA paired reduction: `{paired_full['OSPA']['reduction_pct']:.2f}%`, wins `{paired_full['OSPA']['wins']}/{paired_full['OSPA']['n']}`.\n",
            f"- Localization disagreement paired reduction: `{paired_full['Loc. disag.']['reduction_pct']:.2f}%`, wins `{paired_full['Loc. disag.']['wins']}/{paired_full['Loc. disag.']['n']}`.\n\n",
        ]
    )
    if local_is_independent:
        local_means = local["means"]["Neighborhood label-barycenter spatial-KLA AA"]
        local_base = local["means"]["Tuned spatial-KLA AA"]
        local_paired = local["paired"]["Neighborhood label-barycenter spatial-KLA AA"]
        lines.extend(
            [
                "## Recomputed Local Highlights\n\n",
                f"- Local E-OSPA mean: full `{local_means['E-OSPA']['mean']:.6f}` vs tuned `{local_base['E-OSPA']['mean']:.6f}`.\n",
                f"- RMSE paired reduction: `{local_paired['RMSE']['reduction_pct']:.2f}%`, wins `{local_paired['RMSE']['wins']}/{local_paired['RMSE']['n']}`.\n",
                f"- CardErr paired reduction: `{local_paired['CardErr']['reduction_pct']:.2f}%`, wins `{local_paired['CardErr']['wins']}/{local_paired['CardErr']['n']}`.\n\n",
            ]
        )
    else:
        lines.extend(
            [
                "## Local Metric Boundary\n\n",
                f"- {local['reason']}\n\n",
            ]
        )
    lines.extend(
        [
            "## Recomputed Runtime Highlights\n\n",
            f"- Full runtime: `{runtime['Neighborhood label-barycenter spatial-KLA AA']['mean']:.3f}` s, relative `{runtime['Neighborhood label-barycenter spatial-KLA AA']['relative']:.3f}x`.\n",
            f"- Reference-only runtime: `{runtime['Neighborhood reference-only label-consensus spatial-KLA AA']['mean']:.3f}` s, relative `{runtime['Neighborhood reference-only label-consensus spatial-KLA AA']['relative']:.3f}x`.\n",
        ]
    )
    (OUT / "N50_VERIFICATION_REPORT.md").write_text("".join(lines), encoding="utf-8")


def main() -> None:
    report_text = read_text(REPORT)
    log_text = read_text(LOG)
    payload: dict[str, object] = {
        "source": verify_generated_hash(),
        "network": verify_network(report_text),
        "local": verify_local(report_text),
        "runtime": verify_runtime(report_text, log_text),
    }
    write_outputs(payload)


if __name__ == "__main__":
    main()
