#!/usr/bin/env python3
"""Extract optional harsh packet-loss stress evidence for the TAES package.

The harsh-loss N50 run is a scenario-family stress check, not a parameter
selection loop and not a hard submission gate. If `evidence_sources.json`
contains `stress_harsh_n50_report`, this script parses the report and writes a
traceable manifest plus a response-ready LaTeX fragment. If the key is absent,
the build stays reproducible and any stale stress artifacts are removed.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from evidence_sources import REPO, ROOT, optional_evidence_path


OUT = ROOT / "generated"
REPORT = optional_evidence_path("stress_harsh_n50_report")

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
FULL_ARM = "Neighborhood label-barycenter spatial-KLA AA"
REF_ARM = "Neighborhood reference-only label-consensus spatial-KLA AA"
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
STRESS_OUTPUTS = [
    OUT / "stress_harsh_evidence.json",
    OUT / "STRESS_HARSH_MANIFEST.md",
    OUT / "stress_harsh_section.tex",
    OUT / "stress_harsh_summary_sentence.tex",
]


def read_report(report: Path) -> str:
    if not report.exists():
        raise FileNotFoundError(f"Missing harsh stress report: {report}")
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


def parse_run_config(text: str) -> dict[str, object]:
    base_seed = parse_scalar_line(text, "baseSeed")
    base_seed_match = re.match(r"(\d+)", base_seed)
    if not base_seed_match:
        raise ValueError(f"Could not parse base seed: {base_seed}")
    return {
        "trials": int(parse_scalar_line(text, "Trials")),
        "base_seed": int(base_seed_match.group(1)),
        "trial_seeds": parse_int_list(parse_scalar_line(text, "trialSeeds")),
        "existence_threshold": float(parse_scalar_line(text, "existenceThreshold")),
        "sensor_comm_range": float(parse_scalar_line(text, "sensorCommRange")),
        "fusion_weighting": parse_scalar_line(text, "fusionWeighting"),
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
        raise ValueError(f"Missing harsh stress arms: {missing}")
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


def classify_result(payload: dict[str, object]) -> str:
    paired_network = payload["paired_network"][FULL_ARM]
    paired_local = payload["paired_local"][FULL_ARM]
    network_ok = reduction_value(paired_network["OSPA"]["reduction"]) > 0.0
    eospa_ok = reduction_value(paired_local["E-OSPA"]["reduction"]) > 0.0
    rmse_ok = reduction_value(paired_local["RMSE"]["reduction"]) > 0.0
    card_ok = reduction_value(paired_local["CardErr"]["reduction"]) >= 0.0
    if network_ok and eospa_ok and rmse_ok and card_ok:
        return "stress_supports_full_local_and_network_claims"
    if network_ok and eospa_ok and card_ok and not rmse_ok:
        return "stress_supports_agreement_and_cardinality_but_rmse_is_mixed"
    if network_ok:
        return "stress_supports_network_agreement_but_local_effects_are_mixed"
    return "stress_boundary_or_negative_case"


def build_payload(report: Path) -> dict[str, object]:
    text = read_report(report)
    payload = {
        "source_report": str(report.relative_to(REPO)),
        "source_sha256": sha256(report),
        "config": parse_run_config(text),
        "network": parse_metric_table(text, "## Network Disagreement Metrics", NETWORK_METRICS),
        "local": parse_metric_table(text, "## Local Tracking Metrics", LOCAL_METRICS),
        "paired_network": parse_paired_table(text, "## Paired Improvements Relative to Tuned spatial-KLA AA"),
        "paired_local": parse_paired_table(text, "## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA"),
    }
    payload["interpretation_class"] = classify_result(payload)
    return payload


def tex_escape(value: str) -> str:
    return value.replace("%", r"\%")


def tex_ci(value: str) -> str:
    match = re.fullmatch(r"\[([-0-9.]+),\s*([-0-9.]+)\]", value.strip())
    if not match:
        raise ValueError(f"Could not parse CI: {value}")
    return f"[{float(match.group(1)):.3f}, {float(match.group(2)):.3f}]"


def tex_pvalue(value: str) -> str:
    numeric = float(value)
    if numeric < 1e-3:
        return r"$<10^{-3}$"
    return f"{numeric:.3f}"


def tex_pvalue_expr(value: str) -> str:
    numeric = float(value)
    if numeric < 1e-3:
        return r"$p<10^{-3}$"
    return f"$p={numeric:.3f}$"


def metric_row(metric: str, payload: dict[str, object], paired_key: str) -> str:
    full = payload[paired_key][FULL_ARM][metric]
    ref = payload[paired_key][REF_ARM][metric]
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
    full_network = payload["paired_network"][FULL_ARM]
    full_local = payload["paired_local"][FULL_ARM]
    ref_local = payload["paired_local"][REF_ARM]
    rows = "\n".join(
        [
            metric_row("OSPA", payload, "paired_network"),
            metric_row("E-OSPA", payload, "paired_local"),
            metric_row("RMSE", payload, "paired_local"),
            metric_row("CardErr", payload, "paired_local"),
        ]
    )
    section = rf"""% Auto-generated by scripts/extract_stress_evidence.py.
% Source: {payload['source_report']}
% Response-ready fragment. It is not imported by main.tex unless explicitly enabled.
\begin{{table*}}[t]
\caption{{Harsh packet-loss N50 stress check. Reductions are relative to the fixed spatial-KLA AA baseline, reported under the validation label Tuned spatial-KLA AA, under fixed method parameters and the packet-loss profile $p_\mathrm{{drop}}\in{config['p_drop_levels']}$ with sensor counts {config['p_drop_level_counts']}. Bracketed intervals are 95\% confidence intervals for absolute paired reductions.}}
\label{{tab:harsh-stress}}
\centering
\tablefont
\begin{{tabular}}{{lcccc}}
\toprule
Metric & Full red. [CI] & Full wins/$p$ & Ref. red. [CI] & Ref. wins/$p$\\
\midrule
{rows}
\bottomrule
\end{{tabular}}
\end{{table*}}

The harsh packet-loss stress run uses the fixed design selected before observing the stress N50 result: base seed {config['base_seed']}, {config['trials']} paired trials, existence threshold {config['existence_threshold']:.2f}, and no per-scenario search over projection cutoff, barycenter weights, label rules, or thresholds. Under this profile, the full operator changes network OSPA by {tex_escape(full_network['OSPA']['reduction'])}, local E-OSPA by {tex_escape(full_local['E-OSPA']['reduction'])}, and local RMSE by {tex_escape(full_local['RMSE']['reduction'])} relative to the fixed spatial-KLA AA baseline. The RMSE mechanism-control comparison is full {full_local['RMSE']['wins']} with {tex_pvalue_expr(full_local['RMSE']['p_value'])} versus reference-only {ref_local['RMSE']['wins']} with {tex_pvalue_expr(ref_local['RMSE']['p_value'])}. This fragment should be interpreted as scenario-family stress evidence rather than as a tuning target.
"""
    (OUT / "stress_harsh_section.tex").write_text(section, encoding="utf-8")
    summary = (
        "% Auto-generated by scripts/extract_stress_evidence.py.\n"
        "As a packet-loss severity check, the same fixed parameters were evaluated under "
        f"$p_\\mathrm{{drop}}\\in{config['p_drop_levels']}$ with sensor counts "
        f"{config['p_drop_level_counts']}. "
        "The full operator keeps the mechanism separation in this harsher setting, reducing "
        f"network OSPA by {tex_escape(full_network['OSPA']['reduction'])}, "
        f"local E-OSPA by {tex_escape(full_local['E-OSPA']['reduction'])}, "
        f"and RMSE by {tex_escape(full_local['RMSE']['reduction'])}, "
        f"while the reference-only arm reduces RMSE by {tex_escape(ref_local['RMSE']['reduction'])}.\n"
    )
    (OUT / "stress_harsh_summary_sentence.tex").write_text(summary, encoding="utf-8")


def write_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stress_harsh_evidence.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    config = payload["config"]
    network = payload["network"]
    local = payload["local"]
    paired_network = payload["paired_network"][FULL_ARM]
    paired_local = payload["paired_local"][FULL_ARM]
    lines = [
        "# Harsh Packet-Loss Stress Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_stress_evidence.py`.\n\n",
        "This is an optional scenario-family stress check. It is deliberately not a parameter-selection gate; mixed or negative outcomes should be treated as boundary evidence.\n\n",
        f"- Source report: `{payload['source_report']}`\n",
        f"- Source SHA256: `{payload['source_sha256']}`\n",
        f"- Trials: `{config['trials']}`\n",
        f"- Base seed: `{config['base_seed']}`\n",
        f"- Trial seeds: `{config['trial_seeds']}`\n",
        f"- Packet-loss levels/counts: `{config['p_drop_levels']}` / `{config['p_drop_level_counts']}`\n",
        f"- Interpretation class: `{payload['interpretation_class']}`\n",
        "- Generated manuscript fragment: `generated/stress_harsh_section.tex` (response-ready; not imported by `main.tex` by default).\n",
        "- Generated manuscript sentence: `generated/stress_harsh_summary_sentence.tex` (imported by `main.tex` when stress evidence is configured).\n\n",
        "## Key Stress Checks\n\n",
        f"- Network OSPA: full `{network[FULL_ARM]['OSPA']:.6f}` vs tuned `{network[ARM_ORDER[0]]['OSPA']:.6f}`; paired reduction `{paired_network['OSPA']['reduction']}`, wins `{paired_network['OSPA']['wins']}`.\n",
        f"- Local E-OSPA: full `{local[FULL_ARM]['E-OSPA']:.6f}` vs tuned `{local[ARM_ORDER[0]]['E-OSPA']:.6f}`; paired reduction `{paired_local['E-OSPA']['reduction']}`, wins `{paired_local['E-OSPA']['wins']}`.\n",
        f"- Local RMSE: full `{local[FULL_ARM]['RMSE']:.6f}` vs tuned `{local[ARM_ORDER[0]]['RMSE']:.6f}`; paired reduction `{paired_local['RMSE']['reduction']}`, wins `{paired_local['RMSE']['wins']}`.\n",
        f"- Reference-only RMSE reduction is `{payload['paired_local'][REF_ARM]['RMSE']['reduction']}` with wins `{payload['paired_local'][REF_ARM]['RMSE']['wins']}`.\n",
    ]
    (OUT / "STRESS_HARSH_MANIFEST.md").write_text("".join(lines), encoding="utf-8")
    write_fragment(payload)


def remove_outputs() -> None:
    for path in STRESS_OUTPUTS:
        if path.exists():
            path.unlink()


def main() -> None:
    if REPORT is None:
        remove_outputs()
        return
    write_outputs(build_payload(REPORT))


if __name__ == "__main__":
    main()
