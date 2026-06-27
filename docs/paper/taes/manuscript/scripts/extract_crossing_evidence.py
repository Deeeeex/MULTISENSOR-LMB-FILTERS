#!/usr/bin/env python3
"""Extract optional maneuver/crossing AA evidence for the TAES package.

The crossing run is a fixed-parameter assignment-stability stress check. Its
paper-facing metric is the pre-specified crossing window, not the whole-run
average. If `evidence_sources.json` contains `crossing_n50_report`, this script
parses the report into traceable response-ready artifacts. If the key is absent,
stale crossing artifacts are removed.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path

from evidence_sources import REPO, ROOT, optional_evidence_path


OUT = ROOT / "generated"
REPORT = optional_evidence_path("crossing_n50_report")

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
FULL_ARM = "Neighborhood label-barycenter spatial-KLA AA"
REF_ARM = "Neighborhood reference-only label-consensus spatial-KLA AA"
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
CROSSING_WINDOW = [9, 17]
CROSSING_OUTPUTS = [
    OUT / "crossing_n50_evidence.json",
    OUT / "CROSSING_N50_MANIFEST.md",
    OUT / "crossing_n50_section.tex",
    OUT / "crossing_n50_summary_sentence.tex",
]


def read_report(report: Path) -> str:
    if not report.exists():
        raise FileNotFoundError(f"Missing crossing report: {report}")
    return report.read_text(encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_scalar_line(text: str, key: str) -> str:
    match = re.search(rf"^- {re.escape(key)}: (.+)$", text, flags=re.MULTILINE)
    if not match:
        raise ValueError(f"Could not parse run-config field `{key}`.")
    return match.group(1).strip()


def parse_int_prefix(value: str) -> int:
    match = re.match(r"(\d+)", value)
    if not match:
        raise ValueError(f"Could not parse integer prefix from `{value}`.")
    return int(match.group(1))


def parse_int_list(value: str) -> list[int]:
    return [int(float(item)) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", value)]


def parse_float_list(value: str) -> list[float]:
    return [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", value)]


def parse_run_config(text: str) -> dict[str, object]:
    return {
        "trials": int(parse_scalar_line(text, "Trials")),
        "base_seed": parse_int_prefix(parse_scalar_line(text, "baseSeed")),
        "trial_seeds": parse_int_list(parse_scalar_line(text, "trialSeeds")),
        "scenario_label": parse_scalar_line(text, "scenarioLabel"),
        "target_scenario_mode": parse_scalar_line(text, "targetScenarioMode"),
        "neighbor_map_mode": parse_scalar_line(text, "neighborMapMode"),
        "sensor_comm_range": float(parse_scalar_line(text, "sensorCommRange")),
        "sensor_fov_enabled": int(parse_scalar_line(text, "sensorFovEnabled")),
        "sensor_fov_half_angle_deg": float(parse_scalar_line(text, "sensorFovHalfAngleDeg")),
        "target_formation_life_span": int(parse_scalar_line(text, "targetFormationLifeSpan")),
        "target_formation_count": int(parse_scalar_line(text, "targetFormationCount")),
        "target_formation_staggered_births": int(parse_scalar_line(text, "targetFormationStaggeredBirths")),
        "crossing_window": parse_int_list(parse_scalar_line(text, "crossingWindow")),
        "existence_threshold": float(parse_scalar_line(text, "existenceThreshold")),
        "p_drop_levels": parse_float_list(parse_scalar_line(text, "pDropLevels")),
        "p_drop_level_counts": parse_int_list(parse_scalar_line(text, "pDropLevelCounts")),
    }


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


def parse_metric_table(text: str, heading: str, metrics: list[str]) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(text, heading)
    expected = ["Arm", *metrics]
    if rows[0] != expected:
        raise ValueError(f"Unexpected header under {heading}: {rows[0]}")
    parsed = {row[0]: {metric: float(value) for metric, value in zip(metrics, row[1:])} for row in rows[1:]}
    missing = [arm for arm in ARM_ORDER if arm not in parsed]
    if missing:
        raise ValueError(f"Missing crossing arms: {missing}")
    return parsed


def parse_stats_table(text: str, heading: str, metrics: list[str]) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(text, heading)
    expected = ["Arm", "Metric", "Mean +/- Std", "95% CI", "N"]
    if rows[0] != expected:
        raise ValueError(f"Unexpected stats header under {heading}: {rows[0]}")
    parsed: dict[str, dict[str, float]] = {arm: {} for arm in ARM_ORDER}
    for arm, metric, mean_std, _ci, _n in rows[1:]:
        if arm in parsed and metric in metrics:
            parsed[arm][metric] = float(mean_std.split("+/-", 1)[0].strip())
    missing = [
        f"{arm}/{metric}"
        for arm in ARM_ORDER
        for metric in metrics
        if metric not in parsed.get(arm, {})
    ]
    if missing:
        raise ValueError(f"Missing crossing-window stats: {missing}")
    return parsed


def parse_paired_table(text: str, heading: str) -> dict[str, dict[str, dict[str, str]]]:
    rows = markdown_table_after(text, heading)
    expected = ["Arm", "Metric", "Paired reduction", "95% CI", "Reduction", "Wins", "Sign-test p"]
    if rows[0] != expected:
        raise ValueError(f"Unexpected paired header under {heading}: {rows[0]}")
    parsed: dict[str, dict[str, dict[str, str]]] = {arm: {} for arm in ARM_ORDER[1:]}
    for row in rows[1:]:
        arm, metric, paired_reduction, ci, reduction, wins, p_value = row
        if arm in parsed:
            parsed[arm][metric] = {
                "paired_reduction": paired_reduction,
                "ci": ci,
                "reduction": reduction,
                "wins": wins,
                "p_value": p_value,
            }
    return parsed


def reduction_value(value: str) -> float:
    match = re.search(r"([-+]?\d+(?:\.\d+)?)\s*%", value)
    if not match:
        return math.nan
    return float(match.group(1))


def classify_evidence_tier(trials: int) -> str:
    if trials >= 50:
        return "paper_grade"
    if trials >= 5:
        return "multi_trial_smoke"
    return "n1_smoke"


def classify_result(payload: dict[str, object]) -> str:
    paired = payload["scenario_window"]["paired_local"]
    full_rmse = reduction_value(paired[FULL_ARM]["RMSE"]["reduction"])
    ref_rmse = reduction_value(paired[REF_ARM]["RMSE"]["reduction"])
    full_eospa = reduction_value(paired[FULL_ARM]["E-OSPA"]["reduction"])
    ref_eospa = reduction_value(paired[REF_ARM]["E-OSPA"]["reduction"])
    if full_rmse > 0 and full_rmse > ref_rmse and full_eospa > 0 and full_eospa > ref_eospa:
        return "crossing_window_supports_full_barycenter_separation"
    if full_rmse > 0 and full_rmse > ref_rmse:
        return "crossing_window_supports_rmse_barycenter_separation"
    if full_rmse > 0:
        return "crossing_window_supports_full_but_reference_gap_is_weak"
    return "crossing_window_mixed_or_negative_boundary_case"


def build_payload(report: Path) -> dict[str, object]:
    report = report.resolve()
    text = read_report(report)
    config = parse_run_config(text)
    payload = {
        "source_report": str(report.relative_to(REPO)),
        "source_sha256": sha256(report),
        "config": config,
        "whole_run": {
            "network": parse_metric_table(text, "## Network Disagreement Metrics", NETWORK_METRICS),
            "local": parse_metric_table(text, "## Local Tracking Metrics", LOCAL_METRICS),
            "paired_network": parse_paired_table(text, "## Paired Improvements Relative to Tuned spatial-KLA AA"),
            "paired_local": parse_paired_table(text, "## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA"),
        },
        "scenario_window": {
            "range": config["crossing_window"],
            "network": parse_stats_table(text, "### Scenario-Window Network Summary", NETWORK_METRICS),
            "local": parse_stats_table(text, "### Scenario-Window Local Summary", LOCAL_METRICS),
            "paired_network": parse_paired_table(
                text, "### Scenario-Window Network Improvements Relative to Tuned spatial-KLA AA"
            ),
            "paired_local": parse_paired_table(
                text, "### Scenario-Window Local Improvements Relative to Tuned spatial-KLA AA"
            ),
        },
    }
    payload["evidence_tier"] = classify_evidence_tier(int(config["trials"]))
    payload["interpretation_class"] = classify_result(payload)
    return payload


def tex_escape(value: object) -> str:
    text = str(value)
    for source, replacement in {"&": r"\&", "%": r"\%", "_": r"\_"}.items():
        text = text.replace(source, replacement)
    return text


def tex_ci(value: str) -> str:
    match = re.fullmatch(r"\[([-0-9.]+),\s*([-0-9.]+)\]", value.strip())
    if not match:
        return value
    return f"[{float(match.group(1)):.3f}, {float(match.group(2)):.3f}]"


def tex_pvalue(value: str) -> str:
    try:
        numeric = float(value)
    except ValueError:
        return value
    if numeric < 1e-3:
        return r"$<10^{-3}$"
    return f"{numeric:.3f}"


def metric_row(metric: str, payload: dict[str, object], paired_key: str) -> str:
    full = payload["scenario_window"][paired_key][FULL_ARM][metric]
    ref = payload["scenario_window"][paired_key][REF_ARM][metric]
    values = [
        metric,
        f"{tex_escape(full['reduction'])} {tex_ci(full['ci'])}",
        f"{full['wins']}, {tex_pvalue(full['p_value'])}",
        f"{tex_escape(ref['reduction'])} {tex_ci(ref['ci'])}",
        f"{ref['wins']}, {tex_pvalue(ref['p_value'])}",
    ]
    return " & ".join(values) + r"\\"


def write_fragment(payload: dict[str, object]) -> None:
    config = payload["config"]
    window = payload["scenario_window"]
    rows = "\n".join(
        [
            metric_row("OSPA", payload, "paired_network"),
            metric_row("E-OSPA", payload, "paired_local"),
            metric_row("RMSE", payload, "paired_local"),
            metric_row("CardErr", payload, "paired_local"),
        ]
    )
    full_local = window["paired_local"][FULL_ARM]
    ref_local = window["paired_local"][REF_ARM]
    section = rf"""% Auto-generated by scripts/extract_crossing_evidence.py.
% Source: {payload['source_report']}
% Response-ready fragment. It is not imported by main.tex unless explicitly enabled.
\begin{{table*}}[t]
\caption{{Maneuver/crossing assignment-stability N50 check. Reductions are computed only on the fixed crossing window {window['range']} and are relative to the fixed spatial-KLA AA baseline. Whole-run averages are preserved in the JSON but should not be used to hide crossing-window assignment behavior.}}
\label{{tab:crossing-window}}
\centering
\tablefont
\begin{{tabular}}{{lcccc}}
\toprule
Window metric & Full red. [CI] & Full wins/$p$ & Ref. red. [CI] & Ref. wins/$p$\\
\midrule
{rows}
\bottomrule
\end{{tabular}}
\end{{table*}}

The maneuver/crossing check uses the same fixed AA arms, base seed {config['base_seed']}, {config['trials']} paired trials, and crossing window {window['range']}. The full operator changes crossing-window E-OSPA by {tex_escape(full_local['E-OSPA']['reduction'])} and RMSE by {tex_escape(full_local['RMSE']['reduction'])}; the reference-only arm changes RMSE by {tex_escape(ref_local['RMSE']['reduction'])}. This fragment is assignment-stability boundary evidence and should remain separate from whole-run formation-family claims unless the manuscript explicitly imports it as a supplement or reviewer-response table.
"""
    (OUT / "crossing_n50_section.tex").write_text(section, encoding="utf-8")

    summary = (
        "% Auto-generated by scripts/extract_crossing_evidence.py.\n"
        f"The fixed maneuver/crossing N50 check evaluates window {window['range']} rather than whole-run averages. "
        f"In that window, full label-barycenter changes E-OSPA by {tex_escape(full_local['E-OSPA']['reduction'])} "
        f"and RMSE by {tex_escape(full_local['RMSE']['reduction'])}, while reference-only RMSE changes by "
        f"{tex_escape(ref_local['RMSE']['reduction'])}.\n"
    )
    (OUT / "crossing_n50_summary_sentence.tex").write_text(summary, encoding="utf-8")


def write_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "crossing_n50_evidence.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    config = payload["config"]
    window = payload["scenario_window"]
    full_local = window["paired_local"][FULL_ARM]
    ref_local = window["paired_local"][REF_ARM]
    lines = [
        "# Maneuver/Crossing AA Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_crossing_evidence.py`.\n\n",
        "This optional evidence path checks assignment-stability behavior under a fixed target crossing scenario. It is not a tuning loop and it must be interpreted through the crossing-window metrics.\n\n",
        f"- Source report: `{payload['source_report']}`\n",
        f"- Source SHA-256: `{payload['source_sha256']}`\n",
        f"- Trials: `{config['trials']}`\n",
        f"- Base seed: `{config['base_seed']}`\n",
        f"- Trial seeds: `{config['trial_seeds']}`\n",
        f"- Target scenario mode: `{config['target_scenario_mode']}`\n",
        f"- Crossing window: `{window['range']}`\n",
        f"- Evidence tier: `{payload['evidence_tier']}`\n",
        f"- Interpretation: `{payload['interpretation_class']}`\n",
        "- Generated manuscript fragment: `generated/crossing_n50_section.tex` (response-ready; not imported by `main.tex` by default).\n",
        "- Generated manuscript sentence: `generated/crossing_n50_summary_sentence.tex`.\n\n",
        "## Key Crossing-Window Checks\n\n",
        f"- Crossing-window E-OSPA reduction: full `{full_local['E-OSPA']['reduction']}`, reference-only `{ref_local['E-OSPA']['reduction']}`.\n",
        f"- Crossing-window RMSE reduction: full `{full_local['RMSE']['reduction']}`, reference-only `{ref_local['RMSE']['reduction']}`.\n",
        f"- Crossing-window CardErr reduction: full `{full_local['CardErr']['reduction']}`, reference-only `{ref_local['CardErr']['reduction']}`.\n",
    ]
    (OUT / "CROSSING_N50_MANIFEST.md").write_text("".join(lines), encoding="utf-8")
    write_fragment(payload)


def remove_outputs() -> None:
    for path in CROSSING_OUTPUTS:
        path.unlink(missing_ok=True)


def main() -> None:
    if REPORT is None:
        remove_outputs()
        return
    write_outputs(build_payload(REPORT))


if __name__ == "__main__":
    main()
