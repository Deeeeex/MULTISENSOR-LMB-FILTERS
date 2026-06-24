#!/usr/bin/env python3
"""Extract held-out base-seed evidence for the TAES evidence gate.

The paper-facing main experiment remains the N50 base-seed-1 validation. This
script records a tracked N5 base-seed-11 run that uses the same three AA arms as
the manuscript. It is deliberately labeled as sanity evidence, not as a
submission-grade held-out validation.

When `evidence_sources.json` later includes `heldout_n50_report`, this script
also writes a paper-grade held-out N50 manifest. The optional path keeps the
build reproducible while the long N50 run is still in flight.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from evidence_sources import REPO, ROOT, evidence_path, optional_evidence_path

OUT = ROOT / "generated"
SANITY_REPORT = evidence_path("heldout_sanity_report")
HELDOUT_N50_REPORT = optional_evidence_path("heldout_n50_report")

ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]


def read_report(report: Path) -> str:
    if not report.exists():
        raise FileNotFoundError(f"Missing held-out report: {report}")
    return report.read_text(encoding="utf-8")


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
        raise ValueError(f"No Markdown table after heading: {heading}")
    rows: list[list[str]] = []
    for line in table_lines:
        parts = [part.strip() for part in line.strip().strip("|").split("|")]
        if all(set(part) <= {":", "-"} for part in parts):
            continue
        rows.append(parts)
    return rows


def parse_run_config(text: str) -> dict[str, object]:
    trials = re.search(r"^- Trials: (\d+)$", text, flags=re.MULTILINE)
    base_seed = re.search(r"^- baseSeed: (\d+) \(fixed=1\)$", text, flags=re.MULTILINE)
    trial_seeds = re.search(r"^- trialSeeds: \[([^\]]+)\]$", text, flags=re.MULTILINE)
    if not (trials and base_seed and trial_seeds):
        raise ValueError("Could not parse held-out sanity run config.")
    seeds = [int(value) for value in trial_seeds.group(1).split()]
    return {
        "trials": int(trials.group(1)),
        "base_seed": int(base_seed.group(1)),
        "trial_seeds": seeds,
    }


def parse_metric_table(text: str, heading: str, metrics: list[str]) -> dict[str, dict[str, float]]:
    rows = markdown_table_after(text, heading)
    expected = ["Arm", *metrics]
    if rows[0] != expected:
        raise ValueError(f"Unexpected header under {heading}: {rows[0]}")
    parsed = {row[0]: {metric: float(value) for metric, value in zip(metrics, row[1:])} for row in rows[1:]}
    missing = [arm for arm in ARM_ORDER if arm not in parsed]
    if missing:
        raise ValueError(f"Missing held-out arms: {missing}")
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


def build_payload(report: Path) -> dict[str, object]:
    text = read_report(report)
    return {
        "source_report": str(report.relative_to(REPO)),
        "source_sha256": sha256(report),
        "config": parse_run_config(text),
        "network": parse_metric_table(text, "## Network Disagreement Metrics", NETWORK_METRICS),
        "local": parse_metric_table(text, "## Local Tracking Metrics", LOCAL_METRICS),
        "paired_network": parse_paired_table(text, "## Paired Improvements Relative to Tuned spatial-KLA AA"),
        "paired_local": parse_paired_table(text, "## Paired Local-Metric Improvements Relative to Tuned spatial-KLA AA"),
    }


def write_sanity_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "heldout_sanity_evidence.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    config = payload["config"]
    network = payload["network"]
    local = payload["local"]
    paired_network = payload["paired_network"]["Neighborhood label-barycenter spatial-KLA AA"]
    paired_local = payload["paired_local"]["Neighborhood label-barycenter spatial-KLA AA"]
    lines = [
        "# Held-Out Sanity Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_heldout_sanity_evidence.py`.\n\n",
        "This is a tracked small-sample base-seed sanity check. It is useful for detecting obvious seed-1 overfitting, but it is not a substitute for a paper-grade held-out N50 validation.\n\n",
        f"- Source report: `{payload['source_report']}`\n",
        f"- Source SHA256: `{payload['source_sha256']}`\n",
        f"- Trials: `{config['trials']}`\n",
        f"- Base seed: `{config['base_seed']}`\n",
        f"- Trial seeds: `{config['trial_seeds']}`\n",
        "- Arm set: tuned spatial-KLA AA, neighborhood label-barycenter AA, and neighborhood reference-only AA.\n\n",
        "## Key Sanity Checks\n\n",
        f"- Network OSPA: full `{network['Neighborhood label-barycenter spatial-KLA AA']['OSPA']:.6f}` vs tuned `{network['Tuned spatial-KLA AA']['OSPA']:.6f}`; paired reduction `{paired_network['OSPA']['reduction']}`, wins `{paired_network['OSPA']['wins']}`.\n",
        f"- Local E-OSPA: full `{local['Neighborhood label-barycenter spatial-KLA AA']['E-OSPA']:.6f}` vs tuned `{local['Tuned spatial-KLA AA']['E-OSPA']:.6f}`; paired reduction `{paired_local['E-OSPA']['reduction']}`, wins `{paired_local['E-OSPA']['wins']}`.\n",
        f"- RMSE: full `{local['Neighborhood label-barycenter spatial-KLA AA']['RMSE']:.6f}` vs tuned `{local['Tuned spatial-KLA AA']['RMSE']:.6f}`; paired reduction `{paired_local['RMSE']['reduction']}`, wins `{paired_local['RMSE']['wins']}`.\n",
        f"- Reference-only RMSE reduction is `{payload['paired_local']['Neighborhood reference-only label-consensus spatial-KLA AA']['RMSE']['reduction']}` with wins `{payload['paired_local']['Neighborhood reference-only label-consensus spatial-KLA AA']['RMSE']['wins']}`, preserving the barycenter-vs-label-copying separation in this small run.\n",
    ]
    (OUT / "HELDOUT_SANITY_MANIFEST.md").write_text("".join(lines), encoding="utf-8")


def write_n50_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "heldout_n50_evidence.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    config = payload["config"]
    network = payload["network"]
    local = payload["local"]
    paired_network = payload["paired_network"]["Neighborhood label-barycenter spatial-KLA AA"]
    paired_local = payload["paired_local"]["Neighborhood label-barycenter spatial-KLA AA"]
    lines = [
        "# Held-Out N50 Evidence Manifest\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/extract_heldout_sanity_evidence.py`.\n\n",
        "This is a paper-grade held-out base-seed validation using the same three AA arms as the main N50 evidence. It is intended to test whether the label-barycenter mechanism survives a non-development base seed.\n\n",
        f"- Source report: `{payload['source_report']}`\n",
        f"- Source SHA256: `{payload['source_sha256']}`\n",
        f"- Trials: `{config['trials']}`\n",
        f"- Base seed: `{config['base_seed']}`\n",
        f"- Trial seeds: `{config['trial_seeds']}`\n",
        "- Arm set: tuned spatial-KLA AA, neighborhood label-barycenter AA, and neighborhood reference-only AA.\n\n",
        "## Key Held-Out Checks\n\n",
        f"- Network OSPA: full `{network['Neighborhood label-barycenter spatial-KLA AA']['OSPA']:.6f}` vs tuned `{network['Tuned spatial-KLA AA']['OSPA']:.6f}`; paired reduction `{paired_network['OSPA']['reduction']}`, wins `{paired_network['OSPA']['wins']}`.\n",
        f"- Local E-OSPA: full `{local['Neighborhood label-barycenter spatial-KLA AA']['E-OSPA']:.6f}` vs tuned `{local['Tuned spatial-KLA AA']['E-OSPA']:.6f}`; paired reduction `{paired_local['E-OSPA']['reduction']}`, wins `{paired_local['E-OSPA']['wins']}`.\n",
        f"- RMSE: full `{local['Neighborhood label-barycenter spatial-KLA AA']['RMSE']:.6f}` vs tuned `{local['Tuned spatial-KLA AA']['RMSE']:.6f}`; paired reduction `{paired_local['RMSE']['reduction']}`, wins `{paired_local['RMSE']['wins']}`.\n",
        f"- Reference-only RMSE reduction is `{payload['paired_local']['Neighborhood reference-only label-consensus spatial-KLA AA']['RMSE']['reduction']}` with wins `{payload['paired_local']['Neighborhood reference-only label-consensus spatial-KLA AA']['RMSE']['wins']}`, preserving the barycenter-vs-label-copying separation check.\n",
    ]
    (OUT / "HELDOUT_N50_MANIFEST.md").write_text("".join(lines), encoding="utf-8")
    write_n50_manuscript_fragment(payload)


def manuscript_name(arm: str) -> str:
    if arm == "Tuned spatial-KLA AA":
        return "Tuned spatial-KLA AA"
    if arm == "Neighborhood label-barycenter spatial-KLA AA":
        return "Neighborhood label-barycenter"
    if arm == "Neighborhood reference-only label-consensus spatial-KLA AA":
        return "Neighborhood reference-only"
    return arm


def tex_escape(value: str) -> str:
    return value.replace("%", r"\%")


def mean_row(arm: str, payload: dict[str, object]) -> str:
    network = payload["network"][arm]
    local = payload["local"][arm]
    values = [
        manuscript_name(arm),
        f"{network['OSPA']:.6f}",
        f"{network['Loc. disag.']:.6f}",
        f"{network['Card. disp.']:.6f}",
        f"{local['E-OSPA']:.6f}",
        f"{local['RMSE']:.6f}",
        f"{local['CardErr']:.6f}",
    ]
    return " & ".join(values) + r"\\"


def reduction_row(metric: str, payload: dict[str, object], paired_key: str) -> str:
    full = payload[paired_key]["Neighborhood label-barycenter spatial-KLA AA"][metric]
    ref = payload[paired_key]["Neighborhood reference-only label-consensus spatial-KLA AA"][metric]
    values = [
        metric,
        tex_escape(full["reduction"]),
        full["wins"],
        full["p_value"],
        tex_escape(ref["reduction"]),
        ref["wins"],
        ref["p_value"],
    ]
    return " & ".join(values) + r"\\"


def write_n50_manuscript_fragment(payload: dict[str, object]) -> None:
    config = payload["config"]
    full_network = payload["paired_network"]["Neighborhood label-barycenter spatial-KLA AA"]
    full_local = payload["paired_local"]["Neighborhood label-barycenter spatial-KLA AA"]
    ref_local = payload["paired_local"]["Neighborhood reference-only label-consensus spatial-KLA AA"]

    rows = "\n".join(mean_row(arm, payload) for arm in ARM_ORDER)
    paired_rows = "\n".join(
        [
            reduction_row("OSPA", payload, "paired_network"),
            reduction_row("E-OSPA", payload, "paired_local"),
            reduction_row("RMSE", payload, "paired_local"),
        ]
    )
    section = rf"""\begin{{table*}}[t]
\caption{{Held-out base-seed N50 robustness check. The run uses base seed {config['base_seed']} and the same fixed three-arm protocol as the main validation. Lower values are better.}}
\label{{tab:heldout}}
\centering
\tablefont
\begin{{tabular}}{{lcccccc}}
\toprule
Method & Net OSPA & Loc. disag. & Card. disp. & E-OSPA & RMSE & CardErr\\
\midrule
{rows}
\bottomrule
\end{{tabular}}

\vspace{{0.6em}}
\begin{{tabular}}{{lcccccc}}
\toprule
Metric & Full reduction & Full wins & Full $p$ & Ref.-only reduction & Ref.-only wins & Ref. $p$\\
\midrule
{paired_rows}
\bottomrule
\end{{tabular}}
\end{{table*}}

The held-out base-seed run repeats the mechanism test without changing the method parameters. On this run, the full operator reduces network OSPA by {tex_escape(full_network['OSPA']['reduction'])}, local E-OSPA by {tex_escape(full_local['E-OSPA']['reduction'])}, and local RMSE by {tex_escape(full_local['RMSE']['reduction'])} relative to tuned spatial-KLA AA. The reference-only RMSE reduction is {tex_escape(ref_local['RMSE']['reduction'])}, so the held-out evidence preserves the same separation between label-set copying and matched posterior barycentering.
"""
    (OUT / "heldout_n50_section.tex").write_text(section, encoding="utf-8")


def remove_n50_outputs() -> None:
    for path in [
        OUT / "heldout_n50_evidence.json",
        OUT / "HELDOUT_N50_MANIFEST.md",
        OUT / "heldout_n50_section.tex",
    ]:
        if path.exists():
            path.unlink()


def main() -> None:
    write_sanity_outputs(build_payload(SANITY_REPORT))
    if HELDOUT_N50_REPORT is not None:
        write_n50_outputs(build_payload(HELDOUT_N50_REPORT))
    else:
        remove_n50_outputs()


if __name__ == "__main__":
    main()
