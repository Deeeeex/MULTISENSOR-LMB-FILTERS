#!/usr/bin/env python3
"""Extract optional scenario-family AA evidence for the TAES package.

Scenario-family runs test whether the fixed AA operator still behaves
sensibly when the communication topology or sensing geometry changes. They are
not parameter-selection runs. If `evidence_sources.json` contains one or more
scenario report keys, this script parses them into a traceable manifest and a
response-ready LaTeX fragment. If no scenario keys are configured, stale
scenario artifacts are removed so the package stays reproducible.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from evidence_sources import REPO, ROOT, load_sources


OUT = ROOT / "generated"

SCENARIOS = [
    {
        "key": "scenario_topology_ring_report",
        "name": "topology-ring",
        "expected_label": "topology-ring-formation",
    },
    {
        "key": "scenario_partial_fov35_report",
        "name": "partial-fov35",
        "expected_label": "partial-fov35-formation",
    },
    {
        "key": "scenario_full_topology_report",
        "name": "full-topology",
        "expected_label": "full-topology-formation",
    },
]

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
FULL_ARM = "Neighborhood label-barycenter spatial-KLA AA"
REF_ARM = "Neighborhood reference-only label-consensus spatial-KLA AA"
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
SCENARIO_OUTPUTS = [
    OUT / "scenario_family_evidence.json",
    OUT / "SCENARIO_FAMILY_MANIFEST.md",
    OUT / "scenario_family_section.tex",
    OUT / "scenario_family_summary_sentence.tex",
]


def read_report(report: Path) -> str:
    if not report.exists():
        raise FileNotFoundError(f"Missing scenario-family report: {report}")
    return report.read_text(encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_scalar_line(text: str, key: str) -> str:
    match = re.search(rf"^- {re.escape(key)}: (.+)$", text, flags=re.MULTILINE)
    if not match:
        raise ValueError(f"Could not parse run-config field `{key}`.")
    return match.group(1).strip()


def parse_float_list(value: str) -> list[float]:
    return [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", value)]


def parse_int_list(value: str) -> list[int]:
    return [int(float(item)) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", value)]


def parse_int_prefix(value: str) -> int:
    match = re.match(r"(\d+)", value)
    if not match:
        raise ValueError(f"Could not parse integer prefix from `{value}`.")
    return int(match.group(1))


def parse_run_config(text: str) -> dict[str, object]:
    return {
        "trials": int(parse_scalar_line(text, "Trials")),
        "base_seed": parse_int_prefix(parse_scalar_line(text, "baseSeed")),
        "trial_seeds": parse_int_list(parse_scalar_line(text, "trialSeeds")),
        "lmb_parallel_update_mode": parse_scalar_line(text, "lmbParallelUpdateMode"),
        "scenario_label": parse_scalar_line(text, "scenarioLabel"),
        "neighbor_map_mode": parse_scalar_line(text, "neighborMapMode"),
        "sensor_comm_range": float(parse_scalar_line(text, "sensorCommRange")),
        "fusion_weighting": parse_scalar_line(text, "fusionWeighting"),
        "leader_sensor": int(parse_scalar_line(text, "leaderSensor")),
        "sensor_fov_enabled": int(parse_scalar_line(text, "sensorFovEnabled")),
        "sensor_fov_half_angle_deg": float(parse_scalar_line(text, "sensorFovHalfAngleDeg")),
        "sensor_fov_range": float(parse_scalar_line(text, "sensorFovRange")),
        "sensor_motion_type": parse_scalar_line(text, "sensorMotionType"),
        "sensor_motion_process_noise_std": float(parse_scalar_line(text, "sensorMotionProcessNoiseStd")),
        "target_formation_life_span": int(parse_scalar_line(text, "targetFormationLifeSpan")),
        "existence_threshold": float(parse_scalar_line(text, "existenceThreshold")),
        "link_model": parse_scalar_line(text, "linkModel"),
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
        raise ValueError(f"Missing scenario-family arms: {missing}")
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
        raise ValueError(f"Could not parse reduction percentage: {value}")
    return float(match.group(1))


def ci_lower(value: str) -> float:
    match = re.fullmatch(r"\[([-0-9.]+),\s*([-0-9.]+)\]", value.strip())
    if not match:
        raise ValueError(f"Could not parse CI: {value}")
    return float(match.group(1))


def classify_evidence_tier(trials: int) -> str:
    if trials >= 50:
        return "paper_grade"
    if trials >= 5:
        return "multi_trial_smoke"
    return "n1_smoke"


def classify_result(payload: dict[str, object]) -> str:
    paired_network = payload["paired_network"][FULL_ARM]
    paired_local = payload["paired_local"][FULL_ARM]
    ospa_ok = reduction_value(paired_network["OSPA"]["reduction"]) > 0.0
    loc_ok = reduction_value(paired_network["Loc. disag."]["reduction"]) > 0.0
    eospa_ok = reduction_value(paired_local["E-OSPA"]["reduction"]) > 0.0
    rmse_ok = reduction_value(paired_local["RMSE"]["reduction"]) > 0.0
    card_disp_ok = ci_lower(paired_network["Card. disp."]["ci"]) >= 0.0
    card_err_ok = ci_lower(paired_local["CardErr"]["ci"]) >= 0.0
    if ospa_ok and loc_ok and eospa_ok and rmse_ok and card_disp_ok and card_err_ok:
        return "supports_spatial_and_cardinality_mechanisms"
    if ospa_ok and loc_ok and eospa_ok and rmse_ok:
        return "supports_spatial_mechanism_with_cardinality_boundary"
    if ospa_ok and loc_ok:
        return "supports_network_agreement_but_local_effects_are_mixed"
    return "mixed_or_negative_boundary_case"


def configured_reports() -> list[tuple[dict[str, str], Path]]:
    sources = load_sources()
    reports: list[tuple[dict[str, str], Path]] = []
    for scenario in SCENARIOS:
        key = scenario["key"]
        if key not in sources:
            continue
        report = REPO / sources[key]
        if not report.exists():
            raise FileNotFoundError(f"Evidence source `{key}` does not exist: {report}")
        reports.append((scenario, report))
    return reports


def build_scenario_payload(scenario: dict[str, str], report: Path) -> dict[str, object]:
    text = read_report(report)
    payload = {
        "name": scenario["name"],
        "source_key": scenario["key"],
        "expected_label": scenario["expected_label"],
        "source_report": str(report.relative_to(REPO)),
        "source_sha256": sha256(report),
        "config": parse_run_config(text),
        "network": parse_metric_table(text, "## Network Disagreement Metrics", NETWORK_METRICS),
        "local": parse_metric_table(text, "## Local Tracking Metrics", LOCAL_METRICS),
        "paired_network": parse_paired_table(text, "## Paired Improvements Relative to Tuned spatial-KLA AA"),
        "paired_local": parse_paired_table(text, "## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA"),
    }
    config = payload["config"]
    payload["evidence_tier"] = classify_evidence_tier(int(config["trials"]))
    payload["scenario_label_matches_expected"] = config["scenario_label"] == scenario["expected_label"]
    payload["interpretation_class"] = classify_result(payload)
    return payload


def build_payload(reports: list[tuple[dict[str, str], Path]]) -> dict[str, object]:
    scenarios = [build_scenario_payload(scenario, report) for scenario, report in reports]
    return {
        "generated_by": "docs/paper/taes/manuscript/scripts/extract_scenario_family_evidence.py",
        "purpose": "Optional scenario-family evidence for topology and sensing-geometry robustness boundaries.",
        "configured_keys": [scenario["key"] for scenario, _report in reports],
        "scenario_count": len(scenarios),
        "paper_grade_count": sum(1 for item in scenarios if item["evidence_tier"] == "paper_grade"),
        "smoke_count": sum(1 for item in scenarios if item["evidence_tier"] != "paper_grade"),
        "scenarios": scenarios,
    }


def tex_escape(value: object) -> str:
    text = str(value)
    replacements = {
        "&": r"\&",
        "%": r"\%",
        "_": r"\_",
        "#": r"\#",
    }
    for source, replacement in replacements.items():
        text = text.replace(source, replacement)
    return text


def scenario_row(payload: dict[str, object]) -> str:
    config = payload["config"]
    paired_network = payload["paired_network"][FULL_ARM]
    paired_local = payload["paired_local"][FULL_ARM]
    fields = [
        tex_escape(payload["name"]),
        tex_escape(payload["evidence_tier"]),
        f"{config['trials']} / {config['base_seed']}",
        tex_escape(config["neighbor_map_mode"]),
        f"{float(config['sensor_fov_half_angle_deg']):.0f}",
        tex_escape(paired_network["OSPA"]["reduction"]),
        tex_escape(paired_network["Loc. disag."]["reduction"]),
        tex_escape(paired_local["RMSE"]["reduction"]),
        tex_escape(paired_network["Card. disp."]["reduction"]),
    ]
    return " & ".join(fields) + r"\\"


def count_noun(count: int, singular: str, plural: str) -> str:
    noun = singular if count == 1 else plural
    return f"{count} {noun}"


def write_fragment(payload: dict[str, object]) -> None:
    rows = "\n".join(scenario_row(item) for item in payload["scenarios"])
    paper_grade_phrase = count_noun(
        payload["paper_grade_count"],
        "paper-grade N50-or-larger check",
        "paper-grade N50-or-larger checks",
    )
    smoke_phrase = count_noun(
        payload["smoke_count"],
        "smoke-tier check",
        "smoke-tier checks",
    )
    section = rf"""% Auto-generated by scripts/extract_scenario_family_evidence.py.
% Response-ready fragment. It is not imported by main.tex unless explicitly enabled.
\begin{{table*}}[t]
\caption{{Scenario-family AA checks under fixed method parameters. Reductions are relative to tuned spatial-KLA AA. These runs vary topology or sensing geometry without per-scenario threshold, barycenter-weight, or label-rule search; smoke tiers are recorded as boundary probes, not paper-grade statistical evidence.}}
\label{{tab:scenario-family-aa}}
\centering
\tablefont
\begin{{tabular}}{{llrllllll}}
\toprule
Scenario & Tier & Trials/seed & Topology & FOV & OSPA & Loc. & RMSE & Card. disp.\\
\midrule
{rows}
\bottomrule
\end{{tabular}}
\end{{table*}}

The scenario-family parser records {payload['scenario_count']} configured checks. The tier split is {paper_grade_phrase} and {smoke_phrase}. These artifacts are intended for supplement or reviewer-response use unless the corresponding tier is upgraded to a paper-grade run.
"""
    (OUT / "scenario_family_section.tex").write_text(section, encoding="utf-8")


def write_summary_sentence(payload: dict[str, object]) -> None:
    paper_grade = [
        item
        for item in payload["scenarios"]
        if item.get("evidence_tier") == "paper_grade"
    ]
    if not paper_grade:
        sentence = (
            "% Auto-generated by scripts/extract_scenario_family_evidence.py.\n"
            "The configured scenario-family checks currently remain smoke-tier boundary probes and are not used as paper-grade robustness claims.\n"
        )
        (OUT / "scenario_family_summary_sentence.tex").write_text(sentence, encoding="utf-8")
        return

    clauses = []
    for item in paper_grade:
        config = item["config"]
        paired_network = item["paired_network"][FULL_ARM]
        paired_local = item["paired_local"][FULL_ARM]
        ref_local = item["paired_local"][REF_ARM]
        scenario_name = str(item["name"]).replace("-", " ")
        clauses.append(
            f"{tex_escape(scenario_name)} N50 "
            f"(base seed {config['base_seed']}, topology {tex_escape(config['neighbor_map_mode'])}, "
            f"FOV {float(config['sensor_fov_half_angle_deg']):.0f} deg) reduces network OSPA by "
            f"{tex_escape(paired_network['OSPA']['reduction'])}, local E-OSPA by "
            f"{tex_escape(paired_local['E-OSPA']['reduction'])}, and RMSE by "
            f"{tex_escape(paired_local['RMSE']['reduction'])}, whereas reference-only reduces RMSE by "
            f"{tex_escape(ref_local['RMSE']['reduction'])}"
        )
    if len(clauses) == 1:
        lead = "A fixed-parameter paper-grade scenario-family check also supports the mechanism boundary: "
    else:
        lead = "Fixed-parameter paper-grade scenario-family checks also support the mechanism boundary: "
    sentence = "% Auto-generated by scripts/extract_scenario_family_evidence.py.\n" + lead + "; ".join(clauses) + ".\n"
    (OUT / "scenario_family_summary_sentence.tex").write_text(sentence, encoding="utf-8")


def write_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "scenario_family_evidence.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# Scenario-Family AA Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_scenario_family_evidence.py`.\n\n",
        "This optional evidence path checks topology and sensing-geometry variants under fixed method parameters. It is not a tuning loop; smoke-tier outcomes must be interpreted as boundary probes.\n\n",
        f"- Configured scenario reports: `{payload['scenario_count']}`\n",
        f"- Paper-grade scenarios: `{payload['paper_grade_count']}`\n",
        f"- Smoke-tier scenarios: `{payload['smoke_count']}`\n",
        "- Generated manuscript fragment: `generated/scenario_family_section.tex` (response-ready; not imported by `main.tex` by default).\n",
        "- Generated manuscript sentence: `generated/scenario_family_summary_sentence.tex`.\n\n",
        "## Configured Scenarios\n\n",
        "| Scenario | Source report | SHA-256 | Trials | Base seed | Topology | FOV half-angle | Tier | Interpretation |\n",
        "| --- | --- | --- | ---: | ---: | --- | ---: | --- | --- |\n",
    ]
    for item in payload["scenarios"]:
        config = item["config"]
        lines.append(
            f"| `{item['name']}` | `{item['source_report']}` | `{item['source_sha256']}` | "
            f"{config['trials']} | {config['base_seed']} | `{config['neighbor_map_mode']}` | "
            f"{float(config['sensor_fov_half_angle_deg']):.3f} | `{item['evidence_tier']}` | "
            f"`{item['interpretation_class']}` |\n"
        )
    lines.append("\n## Mechanism Checks\n\n")
    for item in payload["scenarios"]:
        paired_network = item["paired_network"][FULL_ARM]
        paired_local = item["paired_local"][FULL_ARM]
        ref_local = item["paired_local"][REF_ARM]
        lines.append(
            f"- `{item['name']}`: full OSPA `{paired_network['OSPA']['reduction']}`, "
            f"full Loc. disagreement `{paired_network['Loc. disag.']['reduction']}`, "
            f"full RMSE `{paired_local['RMSE']['reduction']}`, reference-only RMSE `{ref_local['RMSE']['reduction']}`.\n"
        )
    (OUT / "SCENARIO_FAMILY_MANIFEST.md").write_text("".join(lines), encoding="utf-8")
    write_fragment(payload)
    write_summary_sentence(payload)


def remove_outputs() -> None:
    for path in SCENARIO_OUTPUTS:
        if path.exists():
            path.unlink()


def main() -> None:
    reports = configured_reports()
    if not reports:
        remove_outputs()
        return
    write_outputs(build_payload(reports))


if __name__ == "__main__":
    main()
