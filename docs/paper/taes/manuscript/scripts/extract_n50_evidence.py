#!/usr/bin/env python3
"""Extract manuscript-ready N50 evidence from the AA validation report.

The manuscript tables should not be maintained by hand. This script parses the
Markdown validation report, verifies that all paper-facing arms and metrics are
present, and writes small generated LaTeX fragments plus a traceable manifest.
"""

from __future__ import annotations

import csv
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from evidence_sources import REPO, ROOT, evidence_path

REPORT = evidence_path("n50_aa_report")
OUT = ROOT / "generated"

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
]
ARM_LABEL = {
    "Tuned spatial-KLA AA": "Fixed spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA": "Neighborhood reference-only",
    "Neighborhood label-barycenter spatial-KLA AA": "Neighborhood label-barycenter",
}
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
PAIRED_METRICS = [
    "Network OSPA",
    "Loc. disagreement",
    "Card. dispersion",
    "E-OSPA",
    "RMSE",
    "Card. error",
]
PAIRED_SOURCE = {
    "Network OSPA": ("network", "OSPA"),
    "Loc. disagreement": ("network", "Loc. disag."),
    "Card. dispersion": ("network", "Card. disp."),
    "E-OSPA": ("local", "E-OSPA"),
    "RMSE": ("local", "RMSE"),
    "Card. error": ("local", "CardErr"),
}
FIG_LABEL = {
    "Network OSPA": "Network OSPA",
    "Loc. disagreement": "Loc. disag.",
    "E-OSPA": "E-OSPA",
    "RMSE": "RMSE",
    "Card. error": "Card. error",
}


@dataclass(frozen=True)
class PairedResult:
    reduction_mean: float
    reduction_std: float
    ci_low: float
    ci_high: float
    reduction_pct: float
    wins: str
    p_value: str


def read_report() -> str:
    if not REPORT.exists():
        raise FileNotFoundError(f"Missing validation report: {REPORT}")
    return REPORT.read_text(encoding="utf-8")


def report_sha256() -> str:
    return hashlib.sha256(REPORT.read_bytes()).hexdigest()


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
        raise ValueError(f"No Markdown table after heading: {heading}")
    rows: list[list[str]] = []
    for line in table_lines:
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if all(set(part) <= {":", "-"} for part in parts):
            continue
        rows.append(parts)
    return rows


def parse_simple_metric_table(text: str, heading: str, metrics: list[str]) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(text, heading)
    header = rows[0]
    expected = ["Arm", *metrics]
    if header != expected:
        raise ValueError(f"Unexpected header under {heading}: {header}")
    parsed: dict[str, dict[str, float]] = {}
    for row in rows[1:]:
        arm = row[0]
        parsed[arm] = {metric: float(value) for metric, value in zip(metrics, row[1:])}
    require_arms(parsed, heading)
    return parsed


def parse_paired_tables(text: str) -> dict[tuple[str, str], PairedResult]:
    parsed: dict[tuple[str, str], PairedResult] = {}
    for heading, group in [
        ("## Paired Improvements Relative to Tuned spatial-KLA AA", "network"),
        ("## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA", "local"),
    ]:
        rows = markdown_table_after(text, heading)
        header = rows[0]
        expected = ["Arm", "Metric", "Paired reduction", "95% CI", "Reduction", "Wins", "Sign-test p"]
        if header != expected:
            raise ValueError(f"Unexpected paired header under {heading}: {header}")
        for row in rows[1:]:
            arm, metric, mean_std, ci, pct, wins, p_value = row
            mean, std = parse_mean_std(mean_std)
            ci_low, ci_high = parse_ci(ci)
            parsed[(arm, metric)] = PairedResult(
                reduction_mean=mean,
                reduction_std=std,
                ci_low=ci_low,
                ci_high=ci_high,
                reduction_pct=float(pct.rstrip("%")),
                wins=wins,
                p_value=p_value,
            )
    for label in ARM_ORDER[1:]:
        for source_group, metric in PAIRED_SOURCE.values():
            key = (label, metric)
            if key not in parsed:
                raise ValueError(f"Missing paired {source_group} metric: {label} / {metric}")
    return parsed


def parse_runtime_table(text: str) -> dict[str, dict[str, str]]:
    rows = markdown_table_after(text, "## Runtime")
    header = rows[0]
    expected = ["Arm", "Filter runtime (s)", "Runtime/step (s)", "Relative to Tuned spatial-KLA AA", "N"]
    if header != expected:
        raise ValueError(f"Unexpected runtime header: {header}")
    parsed = {
        row[0]: {
            "runtime": row[1],
            "runtime_step": row[2],
            "relative": row[3],
            "n": row[4],
        }
        for row in rows[1:]
    }
    require_arms(parsed, "runtime")
    return parsed


def parse_mean_std(value: str) -> tuple[float, float]:
    match = re.fullmatch(r"\s*([0-9.]+)\s*\+/-\s*([0-9.]+)\s*", value)
    if not match:
        raise ValueError(f"Could not parse mean/std: {value}")
    return float(match.group(1)), float(match.group(2))


def parse_ci(value: str) -> tuple[float, float]:
    match = re.fullmatch(r"\s*\[([-0-9.]+),\s*([-0-9.]+)\]\s*", value)
    if not match:
        raise ValueError(f"Could not parse CI: {value}")
    return float(match.group(1)), float(match.group(2))


def require_arms(parsed: dict[str, object], context: str) -> None:
    missing = [arm for arm in ARM_ORDER if arm not in parsed]
    if missing:
        raise ValueError(f"Missing arms in {context}: {missing}")


def tex_num(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def tex_pct(value: float) -> str:
    return f"{value:.2f}\\%"


def tex_ci(result: PairedResult) -> str:
    return f"[{result.ci_low:.3f}, {result.ci_high:.3f}]"


def tex_pvalue(value: str) -> str:
    numeric = float(value)
    if numeric < 1e-3:
        return "$<10^{-3}$"
    return f"{numeric:.3f}"


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def generated_header() -> str:
    rel = REPORT.relative_to(REPO)
    return (
        "% Auto-generated by scripts/extract_n50_evidence.py.\n"
        f"% Source: {rel}\n"
        "% Do not edit by hand; rerun ./build.sh from the manuscript directory.\n"
    )


def write_mean_rows(network: dict[str, dict[str, float]], local: dict[str, dict[str, float]]) -> None:
    rows = [generated_header()]
    metric_values = {
        metric: {arm: network[arm][metric] for arm in ARM_ORDER}
        for metric in NETWORK_METRICS
    }
    metric_values.update(
        {
            metric: {arm: local[arm][metric] for arm in ARM_ORDER}
            for metric in LOCAL_METRICS
        }
    )
    best_by_metric = {
        metric: min(values.values())
        for metric, values in metric_values.items()
    }
    display_tolerance = 0.5 * 10**-3
    for arm in ARM_ORDER:
        metrics = NETWORK_METRICS + LOCAL_METRICS
        values = [network[arm][m] for m in NETWORK_METRICS] + [local[arm][m] for m in LOCAL_METRICS]
        label = ARM_LABEL[arm]
        row_values = []
        for metric, value in zip(metrics, values):
            cell = tex_num(value)
            if value <= best_by_metric[metric] + display_tolerance:
                cell = f"\\textbf{{{cell}}}"
            row_values.append(cell)
        rows.append(f"{label} & {' & '.join(row_values)}\\\\\n")
    rows.append("\\bottomrule\n")
    write_text(OUT / "n50_mean_rows.tex", "".join(rows))


def write_paired_rows(paired: dict[tuple[str, str], PairedResult]) -> None:
    rows = [generated_header()]
    for label in PAIRED_METRICS:
        _, metric = PAIRED_SOURCE[label]
        full = paired[(ARM_ORDER[2], metric)]
        ref = paired[(ARM_ORDER[1], metric)]
        rows.append(
            f"{label} & {tex_pct(full.reduction_pct)} {tex_ci(full)} & {full.wins} & {tex_pvalue(full.p_value)} & "
            f"{tex_pct(ref.reduction_pct)} {tex_ci(ref)} & {ref.wins} & {tex_pvalue(ref.p_value)}\\\\\n"
        )
    rows.append("\\bottomrule\n")
    write_text(OUT / "n50_paired_rows.tex", "".join(rows))


def write_runtime_rows(runtime: dict[str, dict[str, str]]) -> None:
    rows = [generated_header()]
    for arm in ARM_ORDER:
        mean, std = parse_mean_std(runtime[arm]["runtime"])
        rows.append(f"{ARM_LABEL[arm]} & {mean:.3f} $\\pm$ {std:.3f} & {runtime[arm]['relative']}\\\\\n")
    rows.append("\\bottomrule\n")
    write_text(OUT / "n50_runtime_rows.tex", "".join(rows))


def write_reduction_bars(paired: dict[tuple[str, str], PairedResult]) -> None:
    rows = [generated_header(), "\\begin{picture}(38.0,9.8)\n"]
    rows.append("\\put(0.0,9.35){\\makebox(38.0,0.35){\\tablefont Paired reduction over fixed spatial-KLA AA}}\n")
    rows.append("\\put(26.9,8.82){\\color{black}\\rule{0.80pc}{0.06in}}\n")
    rows.append("\\put(27.9,8.78){\\tablefont Full}\n")
    rows.append("\\put(31.4,8.82){\\color[gray]{0.58}\\rule{0.80pc}{0.06in}}\n")
    rows.append("\\put(32.4,8.78){\\tablefont Ref.-only}\n")
    axis_x = 9.2
    axis_y = 0.95
    scale_w = 24.0
    grid_h = 7.65
    bar_h = "0.08in"
    for tick in [0, 25, 50, 75, 100]:
        x = axis_x + scale_w * tick / 100.0
        rows.append(f"\\put({x:.2f},{axis_y:.2f}){{\\color[gray]{{0.82}}\\line(0,1){{{grid_h:.2f}}}}}\n")
        rows.append(f"\\put({x - 0.25:.2f},0.45){{\\tablefont {tick}\\%}}\n")
    y_values = [8.05, 6.52, 4.99, 3.46, 1.93]
    for label, y in zip(PAIRED_METRICS[:2] + PAIRED_METRICS[3:], y_values):
        _, metric = PAIRED_SOURCE[label]
        full = paired[(ARM_ORDER[2], metric)].reduction_pct
        ref = paired[(ARM_ORDER[1], metric)].reduction_pct
        full_w = scale_w * full / 100.0
        ref_w = scale_w * ref / 100.0
        fig_label = FIG_LABEL[label]
        rows.extend(
            [
                f"\\put(0.3,{y + 0.02:.2f}){{\\tablefont {fig_label}}}\n",
                f"\\put({axis_x:.2f},{y + 0.22:.2f}){{\\color{{black}}\\rule{{{full_w:.2f}pc}}{{{bar_h}}}}}\n",
                f"\\put({axis_x:.2f},{y - 0.43:.2f}){{\\color[gray]{{0.58}}\\rule{{{ref_w:.2f}pc}}{{{bar_h}}}}}\n",
            ]
        )
    rows.extend(
        [
            f"\\put({axis_x:.2f},{axis_y:.2f}){{\\line(1,0){{{scale_w:.2f}}}}}\n",
            "\\end{picture}\n",
        ]
    )
    write_text(OUT / "n50_reduction_bars.tex", "".join(rows))


def write_json(
    network: dict[str, dict[str, float]],
    local: dict[str, dict[str, float]],
    paired: dict[tuple[str, str], PairedResult],
    runtime: dict[str, dict[str, str]],
) -> None:
    payload = {
        "source_report": str(REPORT.relative_to(REPO)),
        "source_sha256": report_sha256(),
        "network": network,
        "local": local,
        "paired": {
            f"{arm}::{metric}": result.__dict__
            for (arm, metric), result in sorted(paired.items())
        },
        "runtime": runtime,
    }
    write_text(OUT / "n50_evidence.json", json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_manifest(
    network: dict[str, dict[str, float]],
    local: dict[str, dict[str, float]],
    paired: dict[tuple[str, str], PairedResult],
    runtime: dict[str, dict[str, str]],
) -> None:
    full_rmse = paired[(ARM_ORDER[2], "RMSE")]
    ref_rmse = paired[(ARM_ORDER[1], "RMSE")]
    lines = [
        "# Generated N50 Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_n50_evidence.py`.\n\n",
        f"- Source report: `{REPORT.relative_to(REPO)}`\n",
        f"- Source SHA256: `{report_sha256()}`\n",
        "- Trial count: 50 paired trials\n",
        "- Manuscript fragments: `n50_mean_rows.tex`, `n50_paired_rows.tex`, `n50_runtime_rows.tex`, `n50_reduction_bars.tex`\n",
        "- Machine-readable summary: `n50_evidence.json`\n\n",
        "## Key Paper-Facing Checks\n\n",
        f"- Full method network OSPA: `{network[ARM_ORDER[2]]['OSPA']:.6f}` vs fixed baseline `{network[ARM_ORDER[0]]['OSPA']:.6f}`.\n",
        f"- Full method local E-OSPA: `{local[ARM_ORDER[2]]['E-OSPA']:.6f}` vs fixed baseline `{local[ARM_ORDER[0]]['E-OSPA']:.6f}`.\n",
        f"- Full method RMSE reduction: `{full_rmse.reduction_pct:.2f}%`, CI `[{full_rmse.ci_low:.6f}, {full_rmse.ci_high:.6f}]`, wins `{full_rmse.wins}`, sign-test p `{full_rmse.p_value}`.\n",
        f"- Reference-only RMSE reduction: `{ref_rmse.reduction_pct:.2f}%`, CI `[{ref_rmse.ci_low:.6f}, {ref_rmse.ci_high:.6f}]`, wins `{ref_rmse.wins}`, sign-test p `{ref_rmse.p_value}`.\n",
        f"- Runtime overhead: full `{runtime[ARM_ORDER[2]]['relative']}`, reference-only `{runtime[ARM_ORDER[1]]['relative']}` relative to the fixed baseline.\n",
    ]
    write_text(OUT / "N50_EVIDENCE_MANIFEST.md", "".join(lines))


def write_csv(network: dict[str, dict[str, float]], local: dict[str, dict[str, float]]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "n50_mean_metrics.csv").open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh, lineterminator="\n")
        writer.writerow(["arm", *NETWORK_METRICS, *LOCAL_METRICS])
        for arm in ARM_ORDER:
            writer.writerow([ARM_LABEL[arm], *[network[arm][m] for m in NETWORK_METRICS], *[local[arm][m] for m in LOCAL_METRICS]])


def main() -> None:
    text = read_report()
    network = parse_simple_metric_table(text, "## Network Disagreement Metrics", NETWORK_METRICS)
    local = parse_simple_metric_table(text, "## Local Tracking Metrics", LOCAL_METRICS)
    paired = parse_paired_tables(text)
    runtime = parse_runtime_table(text)
    OUT.mkdir(parents=True, exist_ok=True)
    write_mean_rows(network, local)
    write_paired_rows(paired)
    write_runtime_rows(runtime)
    write_reduction_bars(paired)
    write_json(network, local, paired, runtime)
    write_manifest(network, local, paired, runtime)
    write_csv(network, local)


if __name__ == "__main__":
    main()
