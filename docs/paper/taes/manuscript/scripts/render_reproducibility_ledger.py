#!/usr/bin/env python3
"""Render a compact, manuscript-facing reproducibility ledger.

The main results already come from generated tables. This script adds a small
audit table that tells reviewers which artifact supports each evidence role and
which comparisons are paired versus contextual. It keeps that provenance in the
build pipeline instead of maintaining the text by hand.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from evidence_sources import REPO, ROOT, evidence_path


OUT = ROOT / "generated"
ROWS_TEX = OUT / "reproducibility_ledger_rows.tex"
TABLE_TEX = OUT / "reproducibility_ledger_table.tex"
LEDGER_JSON = OUT / "reproducibility_ledger.json"
MANIFEST_MD = OUT / "REPRODUCIBILITY_LEDGER_MANIFEST.md"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_report_config(path: Path) -> dict[str, object]:
    text = path.read_text(encoding="utf-8")
    config: dict[str, object] = {}
    patterns = {
        "trials": r"- Trials:\s*(\d+)",
        "base_seed": r"- baseSeed:\s*(\d+)",
        "trial_seeds": r"- trialSeeds:\s*\[([^\]]+)\]",
        "p_drop_levels": r"- pDropLevels:\s*\[([^\]]+)\]",
        "p_drop_level_counts": r"- pDropLevelCounts:\s*\[([^\]]+)\]",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if not match:
            continue
        value = match.group(1)
        if key in {"trials", "base_seed"}:
            config[key] = int(value)
        elif key in {"trial_seeds", "p_drop_level_counts"}:
            config[key] = [int(item) for item in value.split()]
        else:
            config[key] = [float(item) for item in value.split()]
    required = {"trials", "base_seed", "trial_seeds", "p_drop_levels", "p_drop_level_counts"}
    missing = sorted(required - set(config))
    if missing:
        raise ValueError(f"Missing report config fields in {path}: {missing}")
    return config


def seed_range(config: dict[str, object]) -> str:
    seeds = config["trial_seeds"]
    if not isinstance(seeds, list) or not seeds:
        raise ValueError("trial_seeds must be a nonempty list")
    return f"{seeds[0]}--{seeds[-1]}"


def profile(config: dict[str, object]) -> str:
    levels = config["p_drop_levels"]
    counts = config["p_drop_level_counts"]
    if not isinstance(levels, list) or not isinstance(counts, list):
        raise ValueError("packet-loss profile fields must be lists")
    level_text = "/".join(f"{float(value):g}" for value in levels)
    count_text = "/".join(str(int(value)) for value in counts)
    return f"{level_text}; counts {count_text}"


def rel(path: Path) -> str:
    return path.relative_to(REPO).as_posix()


def tex_escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
    }
    return "".join(replacements.get(char, char) for char in value)


def evidence_row(name: str, source_path: Path, config: dict[str, object], role: str) -> dict[str, object]:
    return {
        "name": name,
        "source": rel(source_path),
        "sha256": sha256(source_path),
        "protocol": (
            f"base seed {config['base_seed']}; {config['trials']} trials; "
            f"seeds {seed_range(config)}; packet loss {profile(config)}"
        ),
        "role": role,
    }


def build_rows() -> list[dict[str, object]]:
    n50_report = evidence_path("n50_aa_report")
    heldout_report = evidence_path("heldout_n50_report")
    ga_report = evidence_path("reference_ga_report")
    stress_report = evidence_path("stress_harsh_n50_report")

    n50_evidence = read_json(OUT / "n50_evidence.json")
    heldout_evidence = read_json(OUT / "heldout_n50_evidence.json")
    reference_evidence = read_json(OUT / "reference_baseline_evidence.json")
    stress_evidence = read_json(OUT / "stress_harsh_evidence.json")
    scenario_evidence = read_json(OUT / "scenario_family_evidence.json")
    crossing_path = OUT / "crossing_n50_evidence.json"
    crossing_evidence = read_json(crossing_path) if crossing_path.exists() else None
    verification = read_json(OUT / "n50_verification.json")

    if n50_evidence.get("source_sha256") != sha256(n50_report):
        raise ValueError("N50 evidence JSON does not match the configured AA report hash")
    if heldout_evidence.get("source_sha256") != sha256(heldout_report):
        raise ValueError("Held-out evidence JSON does not match the configured report hash")
    if reference_evidence.get("ga_sha256") != sha256(ga_report):
        raise ValueError("Reference evidence JSON does not match the configured GA report hash")
    if stress_evidence.get("source_sha256") != sha256(stress_report):
        raise ValueError("Harsh-stress evidence JSON does not match the configured report hash")
    for scenario in scenario_evidence.get("scenarios", []):
        if not isinstance(scenario, dict):
            raise ValueError("Scenario-family evidence payload contains a non-object scenario entry")
        source_report = scenario.get("source_report")
        source_sha256 = scenario.get("source_sha256")
        if not source_report or not source_sha256:
            raise ValueError("Scenario-family evidence entry is missing source_report or source_sha256")
        source_path = REPO / str(source_report)
        if sha256(source_path) != source_sha256:
            raise ValueError(f"Scenario-family evidence JSON hash does not match {source_report}")
    if verification.get("local", {}).get("status") != "independent":
        raise ValueError("N50 local verifier is not marked independent")
    if crossing_evidence is not None:
        source_report = crossing_evidence.get("source_report")
        source_sha256 = crossing_evidence.get("source_sha256")
        if not source_report or not source_sha256:
            raise ValueError("Crossing evidence payload is missing source_report or source_sha256")
        crossing_source_path = REPO / str(source_report)
        if sha256(crossing_source_path) != source_sha256:
            raise ValueError(f"Crossing evidence JSON hash does not match {source_report}")

    scenario_count = int(scenario_evidence.get("scenario_count", 0))
    paper_grade_count = int(scenario_evidence.get("paper_grade_count", 0))
    smoke_count = int(scenario_evidence.get("smoke_count", max(0, scenario_count - paper_grade_count)))
    configured_keys = scenario_evidence.get("configured_keys", [])
    if not isinstance(configured_keys, list):
        raise ValueError("Scenario-family configured_keys must be a list")
    if scenario_count == 0:
        scenario_role = "Scenario-family evidence path is enabled, but no topology/FOV/full-neighborhood checks are currently configured."
    elif smoke_count == 0:
        scenario_role = (
            "Response-ready boundary evidence; all configured topology/FOV/full-neighborhood checks are "
            "N50-or-larger paper-grade fixed-parameter runs."
        )
    else:
        scenario_role = (
            f"Response-ready boundary evidence; {paper_grade_count} paper-grade and "
            f"{smoke_count} smoke-tier check(s), with smoke tiers explicitly labeled as boundary probes."
        )

    rows = [
        evidence_row(
            "Primary AA N50",
            n50_report,
            parse_report_config(n50_report),
            "Paired AA evidence for Tables I, III, IV, and Fig. 3; paired confidence intervals, wins, and sign-test values use the same per-trial packet-loss realizations.",
        ),
        evidence_row(
            "Held-out AA N50",
            heldout_report,
            parse_report_config(heldout_report),
            "Base-seed robustness check; generated after method selection and not used for parameter search.",
        ),
        evidence_row(
            "Contextual GA N50",
            ga_report,
            parse_report_config(ga_report),
            "Reference rows from the tracked GA validation path; same tiered profile, but not part of the paired AA sign tests.",
        ),
        evidence_row(
            "Harsh-loss AA N50",
            stress_report,
            parse_report_config(stress_report),
            "Fixed-design packet-loss stress evidence; summarized in the discussion and kept separate from parameter selection.",
        ),
        {
            "name": "Scenario-family boundary checks",
            "source": rel(OUT / "scenario_family_evidence.json"),
            "sha256": sha256(OUT / "scenario_family_evidence.json"),
            "protocol": (
                f"{scenario_count} configured topology/FOV/full-neighborhood checks; "
                f"{paper_grade_count} paper-grade; keys {', '.join(str(key) for key in configured_keys)}"
            ),
            "role": scenario_role,
        },
    ]
    if crossing_evidence is not None:
        source_path = REPO / str(crossing_evidence["source_report"])
        config = crossing_evidence.get("config", {})
        if not isinstance(config, dict):
            config = parse_report_config(source_path)
        window = crossing_evidence.get("scenario_window", {}).get("range", config.get("crossing_window", "unknown"))
        rows.append(
            {
                "name": "Maneuver-crossing AA N50",
                "source": rel(source_path),
                "sha256": sha256(source_path),
                "protocol": (
                    f"base seed {config.get('base_seed')}; {config.get('trials')} trials; "
                    f"seeds {seed_range(config)}; crossing window {window}; packet loss {profile(config)}"
                ),
                "role": "Response-ready assignment-stability boundary evidence; interpreted through crossing-window metrics, not whole-run averages.",
            }
        )
    rows.append(
        {
            "name": "Independent verifier",
            "source": rel(OUT / "n50_verification.json"),
            "sha256": sha256(OUT / "n50_verification.json"),
            "protocol": "Recomputes network disagreement, runtime, and local E-OSPA/RMSE/CardErr from per-trial rows/logs.",
            "role": "Build gate for report-driven numbers before PDF compilation.",
        }
    )
    return rows


def write_outputs(rows: list[dict[str, object]]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tex_lines = [
        "% Auto-generated by scripts/render_reproducibility_ledger.py.\n",
        "% Do not edit by hand; rerun ./build.sh from the manuscript directory.\n",
    ]
    for row in rows:
        tex_lines.append(
            f"{tex_escape(str(row['name']))} & "
            f"{tex_escape(str(row['protocol']))} & "
            f"{tex_escape(str(row['role']))}\\tabularnewline\n"
        )
    ROWS_TEX.write_text("".join(tex_lines), encoding="utf-8")

    table_lines = [
        "% Auto-generated by scripts/render_reproducibility_ledger.py.\n",
        "% Do not edit by hand; rerun ./build.sh from the manuscript directory.\n",
        "\\begin{table*}[t]\n",
        "\\caption{Reproducibility ledger for the manuscript-facing experimental evidence.}\n",
        "\\label{tab:ledger}\n",
        "\\centering\n",
        "\\tablefont\n",
        "\\begin{tabular}{>{\\raggedright\\arraybackslash}p{1.05in}>{\\raggedright\\arraybackslash}p{2.45in}>{\\raggedright\\arraybackslash}p{2.75in}}\n",
        "\\toprule\n",
        "Artifact & Protocol & Manuscript role\\tabularnewline\n",
        "\\midrule\n",
        *tex_lines[2:],
        "\\bottomrule\n",
        "\\end{tabular}\n",
        "\\end{table*}\n",
    ]
    TABLE_TEX.write_text("".join(table_lines), encoding="utf-8")

    LEDGER_JSON.write_text(json.dumps({"rows": rows}, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# Reproducibility Ledger Manifest\n\n",
        "This manifest is generated by `scripts/render_reproducibility_ledger.py`.\n\n",
        "| Artifact | Source | SHA-256 |\n",
        "| --- | --- | --- |\n",
    ]
    for row in rows:
        lines.append(f"| {row['name']} | `{row['source']}` | `{row['sha256']}` |\n")
    MANIFEST_MD.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    write_outputs(build_rows())


if __name__ == "__main__":
    main()
