#!/usr/bin/env python3
"""Machine-checkable TAES submission-readiness snapshot.

This script is intentionally stricter about mechanical consistency than about
research completeness. It fails the build for missing files, broken citations,
or broken references, but it records author metadata, held-out validation, and
local-metric verifier closure as pending gates while the draft is still moving.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
OUT = ROOT / "generated"

MAIN_TEX = ROOT / "main.tex"
MAIN_PDF = ROOT / "main.pdf"
BIB = ROOT / "references.bib"
COVER_LETTER = ROOT / "COVER_LETTER_AND_METADATA_DRAFT.md"
REQUIREMENTS_DOC = REPO / "docs" / "TAES_SUBMISSION_REQUIREMENTS_CN.md"
REGULAR_TEMPLATE = REPO / "docs" / "paper" / "taes" / "template_regular" / "IEEE_TAES_orig-research" / "TAES_template.tex"
TEMPLATE_ZIP = REPO / "docs" / "paper" / "taes" / "TAES_Template.zip"
VERIFICATION_JSON = OUT / "n50_verification.json"
HELDOUT_SANITY_JSON = OUT / "heldout_sanity_evidence.json"
HELDOUT_N50_JSON = OUT / "heldout_n50_evidence.json"
HELDOUT_N50_MANIFEST = OUT / "HELDOUT_N50_MANIFEST.md"
HELDOUT_N50_FRAGMENT = OUT / "heldout_n50_section.tex"
BUNDLE_MANIFEST_JSON = OUT / "submission_bundle_manifest.json"
BUNDLE_MANIFEST_MD = OUT / "SUBMISSION_BUNDLE_MANIFEST.md"
READINESS_JSON = OUT / "submission_readiness.json"
READINESS_MD = OUT / "SUBMISSION_READINESS_REPORT.md"

HELDOUT_BASE_SEED = 11
ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
FULL_ARM = ARM_ORDER[1]
REF_ONLY_ARM = ARM_ORDER[2]
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]


@dataclass
class Check:
    gate: str
    status: str
    detail: str

    def as_dict(self) -> dict[str, str]:
        return {"gate": self.gate, "status": self.status, "detail": self.detail}


def read_text(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(path)
    return path.read_text(encoding="utf-8")


def cite_keys(tex: str) -> set[str]:
    keys: set[str] = set()
    for match in re.finditer(r"\\cite(?:\[[^\]]*\])*{([^}]+)}", tex):
        keys.update(key.strip() for key in match.group(1).split(",") if key.strip())
    return keys


def bib_keys(bib: str) -> set[str]:
    return set(re.findall(r"@\w+\s*{\s*([^,\s]+)", bib))


def labels(tex: str) -> set[str]:
    return set(re.findall(r"\\label{([^}]+)}", tex))


def refs(tex: str) -> set[str]:
    return set(re.findall(r"\\(?:ref|eqref|pageref){([^}]+)}", tex))


def command_body(tex: str, name: str) -> str:
    match = re.search(rf"\\{name}{{([^}}]+)}}", tex, flags=re.DOTALL)
    return match.group(1).strip() if match else ""


def environment_body(tex: str, name: str) -> str:
    match = re.search(rf"\\begin{{{name}}}(.+?)\\end{{{name}}}", tex, flags=re.DOTALL)
    return match.group(1).strip() if match else ""


def placeholder_hits(tex: str) -> list[str]:
    hits: list[str] = []
    literal_patterns = [
        "FIRST AUTHOR",
        "SECOND AUTHOR",
        "THIRD AUTHOR",
        "AUTHOR ET AL.",
        "TAES.2026.Doi Number",
        "Draft",
        "XX",
    ]
    for pattern in literal_patterns:
        if pattern in tex:
            hits.append(pattern)
    bracket_patterns = [
        r"\[Funding Agency\]",
        r"\[Grant Number\]",
        r"\[Institution\]",
        r"\[City\]",
        r"\[Country\]",
        r"\[author@example\.com\]",
        r"\[repository DOI/URL\]",
    ]
    for pattern in bracket_patterns:
        for value in re.findall(pattern, tex):
            if value not in hits:
                hits.append(value)
    return sorted(hits)


def all_bib_entries_have_doi(bib: str) -> tuple[bool, list[str]]:
    entries = re.split(r"\n(?=@\w+\s*{)", bib.strip())
    missing: list[str] = []
    for entry in entries:
        if not entry.strip():
            continue
        key_match = re.match(r"@\w+\s*{\s*([^,\s]+)", entry)
        if not key_match:
            continue
        key = key_match.group(1)
        if not re.search(r"\bdoi\s*=", entry, flags=re.IGNORECASE):
            missing.append(key)
    return len(missing) == 0, missing


def safe_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def reduction_percent(value: object) -> float | None:
    match = re.search(r"([-+]?\d+(?:\.\d+)?)\s*%", str(value))
    if not match:
        return None
    return float(match.group(1))


def missing_metric_entries(
    payload: dict[str, object],
    table_name: str,
    arms: list[str],
    metrics: list[str],
) -> list[str]:
    table = payload.get(table_name, {})
    missing: list[str] = []
    if not isinstance(table, dict):
        return [table_name]
    for arm in arms:
        row = table.get(arm)
        if not isinstance(row, dict):
            missing.append(f"{table_name}/{arm}")
            continue
        for metric in metrics:
            if metric not in row:
                missing.append(f"{table_name}/{arm}/{metric}")
    return missing


def missing_paired_entries(
    payload: dict[str, object],
    table_name: str,
    metrics: list[str],
) -> list[str]:
    table = payload.get(table_name, {})
    missing: list[str] = []
    if not isinstance(table, dict):
        return [table_name]
    for arm in [FULL_ARM, REF_ONLY_ARM]:
        arm_payload = table.get(arm)
        if not isinstance(arm_payload, dict):
            missing.append(f"{table_name}/{arm}")
            continue
        for metric in metrics:
            metric_payload = arm_payload.get(metric)
            if not isinstance(metric_payload, dict):
                missing.append(f"{table_name}/{arm}/{metric}")
                continue
            for field in ["paired_reduction", "ci", "reduction", "wins", "p_value"]:
                if field not in metric_payload:
                    missing.append(f"{table_name}/{arm}/{metric}/{field}")
    return missing


def file_checks() -> list[Check]:
    checks: list[Check] = []
    required = [
        MAIN_TEX,
        MAIN_PDF,
        BIB,
        COVER_LETTER,
        ROOT / "IEEEtaes.cls",
        ROOT / "IEEEtaes.bst",
        REQUIREMENTS_DOC,
        REGULAR_TEMPLATE,
        TEMPLATE_ZIP,
        VERIFICATION_JSON,
        OUT / "N50_EVIDENCE_MANIFEST.md",
        OUT / "REFERENCE_BASELINE_MANIFEST.md",
        OUT / "HELDOUT_SANITY_MANIFEST.md",
        OUT / "SUBMISSION_BUNDLE_MANIFEST.md",
        ROOT / "scripts" / "create_submission_bundle.py",
    ]
    for path in required:
        status = "pass" if path.exists() else "error"
        detail = str(path.relative_to(REPO)) if path.exists() else f"missing {path.relative_to(REPO)}"
        checks.append(Check("required artifact", status, detail))
    return checks


def manuscript_checks(tex: str, bib: str) -> list[Check]:
    checks: list[Check] = []
    checks.append(
        Check(
            "TAES class",
            "pass" if "\\documentclass{IEEEtaes}" in tex else "error",
            "`main.tex` uses `IEEEtaes`." if "\\documentclass{IEEEtaes}" in tex else "`main.tex` does not use `IEEEtaes`.",
        )
    )

    title = command_body(tex, "title")
    abstract = environment_body(tex, "abstract")
    keywords = environment_body(tex, "IEEEkeywords")

    title_abstract = f"{title}\n{abstract}".lower()
    discouraged_words = sorted(set(re.findall(r"\b(?:new|novel)\b", title_abstract)))
    checks.append(
        Check(
            "TAES title and abstract wording",
            "pass" if not discouraged_words else "warning",
            "Title and abstract avoid discouraged novelty words such as `new` and `novel`."
            if not discouraged_words
            else f"Title/abstract contain discouraged novelty words: {', '.join(discouraged_words)}",
        )
    )

    abstract_bad_tokens = [
        token
        for token in [r"\cite", r"\footnote", r"\begin{equation}", r"\["]
        if token in abstract
    ]
    paragraph_count = len([part for part in re.split(r"\n\s*\n", abstract) if part.strip()])
    abstract_ok = bool(abstract) and not abstract_bad_tokens and paragraph_count == 1
    checks.append(
        Check(
            "TAES abstract format",
            "pass" if abstract_ok else "warning",
            "Abstract is one paragraph and contains no citations, footnotes, or displayed equations."
            if abstract_ok
            else "Abstract should be one paragraph without citations, footnotes, or displayed equations.",
        )
    )

    keyword_items = [item.strip().lower() for item in keywords.split(",") if item.strip()]
    keyword_ok = len(keyword_items) >= 3 and keyword_items == sorted(keyword_items)
    checks.append(
        Check(
            "TAES keyword format",
            "pass" if keyword_ok else "warning",
            "Keywords are comma-separated, include at least three phrases, and are alphabetized."
            if keyword_ok
            else "Keywords should be comma-separated, include at least three phrases, and be alphabetized.",
        )
    )

    cited = cite_keys(tex)
    available = bib_keys(bib)
    missing_cites = sorted(cited - available)
    checks.append(
        Check(
            "citation keys",
            "pass" if not missing_cites else "error",
            f"{len(cited)} cited keys all exist in `references.bib`."
            if not missing_cites
            else f"missing BibTeX keys: {', '.join(missing_cites)}",
        )
    )

    missing_refs = sorted(refs(tex) - labels(tex))
    checks.append(
        Check(
            "cross references",
            "pass" if not missing_refs else "error",
            f"{len(refs(tex))} cross-references resolve to manuscript labels."
            if not missing_refs
            else f"missing labels: {', '.join(missing_refs)}",
        )
    )

    doi_ok, missing_doi = all_bib_entries_have_doi(bib)
    checks.append(
        Check(
            "BibTeX DOI fields",
            "pass" if doi_ok else "warning",
            "All BibTeX entries contain DOI fields."
            if doi_ok
            else f"BibTeX entries without DOI: {', '.join(missing_doi)}",
        )
    )

    placeholders = placeholder_hits(tex)
    checks.append(
        Check(
            "submission metadata placeholders",
            "pending" if placeholders else "pass",
            f"{len(placeholders)} placeholder tokens remain: {', '.join(placeholders)}"
            if placeholders
            else "No obvious placeholder tokens detected.",
        )
    )

    disclosure_ok = "OpenAI Codex" in tex and "authors remain responsible" in tex.lower()
    checks.append(
        Check(
            "AI-assistance disclosure",
            "pass" if disclosure_ok else "warning",
            "Acknowledgment includes a conservative OpenAI Codex disclosure."
            if disclosure_ok
            else "AI-assistance disclosure was not detected.",
        )
    )

    alignment_markers = [
        "active output tracks after the upstream AA existence update and thresholding",
        "does not replace the Bernoulli existence consumer",
        "label-and-moment projection on the active estimate",
        "matched moment coordinates",
        "upstream AA existence consumer",
    ]
    missing_alignment = [marker for marker in alignment_markers if marker not in tex]
    checks.append(
        Check(
            "implementation-alignment wording",
            "pass" if not missing_alignment else "warning",
            "Manuscript explicitly separates active-track label/moment projection from the upstream AA existence consumer."
            if not missing_alignment
            else "Projection/existence boundary wording is incomplete; missing markers: "
            + "; ".join(missing_alignment),
        )
    )
    return checks


def cover_letter_checks() -> list[Check]:
    if not COVER_LETTER.exists():
        return [Check("cover letter and portal metadata draft", "warning", "`COVER_LETTER_AND_METADATA_DRAFT.md` is missing.")]
    text = read_text(COVER_LETTER)
    required_markers = [
        "Regular Paper",
        "Target Tracking and Multi-Sensor Systems",
        "original work",
        "not under consideration elsewhere",
        "OpenAI Codex",
        "ORCID",
        "repository DOI/URL",
    ]
    missing = [marker for marker in required_markers if marker not in text]
    return [
        Check(
            "cover letter and portal metadata draft",
            "pass" if not missing else "warning",
            "Cover-letter draft and portal metadata checklist exist with manuscript type, technical area, originality, AI disclosure, ORCID, and repository placeholders."
            if not missing
            else "Cover-letter draft exists but is missing markers: " + "; ".join(missing),
        )
    ]


def bundle_checks() -> list[Check]:
    if not BUNDLE_MANIFEST_JSON.exists():
        return [Check("submission source bundle", "warning", "`generated/submission_bundle_manifest.json` is missing.")]
    payload = json.loads(read_text(BUNDLE_MANIFEST_JSON))
    bundle_rel = payload.get("bundle", "")
    bundle_path = REPO / bundle_rel if bundle_rel else Path()
    file_count = int(payload.get("file_count", 0))
    checksum = str(payload.get("bundle_sha256", ""))
    ok = bool(bundle_rel) and bundle_path.exists() and file_count >= 10 and len(checksum) == 64
    return [
        Check(
            "submission source bundle",
            "pass" if ok else "warning",
            f"Clean source bundle exists at `{bundle_rel}` with {file_count} files and SHA-256 `{checksum}`."
            if ok
            else "Submission source bundle manifest exists but the bundle path, file count, or checksum is incomplete.",
        )
    ]


def pdf_checks() -> list[Check]:
    checks: list[Check] = []
    if not MAIN_PDF.exists():
        return [Check("PDF render", "error", "`main.pdf` is missing.")]
    reader = PdfReader(str(MAIN_PDF))
    page_count = len(reader.pages)
    checks.append(
        Check(
            "PDF page count",
            "pass" if page_count <= 10 else "warning",
            f"`main.pdf` has {page_count} TAES-template pages; Regular Paper overlength charges start at 10 printed pages.",
        )
    )
    text = "\n".join(page.extract_text() or "" for page in reader.pages)
    bad_tokens = [token for token in ["??", "undefined references", "Citation"] if token in text]
    checks.append(
        Check(
            "PDF extracted text",
            "pass" if not bad_tokens else "warning",
            "No obvious unresolved-reference tokens found in extracted PDF text."
            if not bad_tokens
            else f"possible unresolved tokens in extracted PDF text: {', '.join(bad_tokens)}",
        )
    )
    return checks


def evidence_checks() -> list[Check]:
    checks: list[Check] = []
    if not VERIFICATION_JSON.exists():
        return [Check("N50 verifier", "error", "`generated/n50_verification.json` is missing.")]
    payload = json.loads(read_text(VERIFICATION_JSON))
    local_status = payload.get("local", {}).get("status", "missing")
    checks.append(
        Check(
            "N50 network verifier",
            "pass" if "network" in payload else "error",
            "Network disagreement metrics are recomputed from per-trial report rows."
            if "network" in payload
            else "Network verifier payload is missing.",
        )
    )
    checks.append(
        Check(
            "N50 runtime verifier",
            "pass" if "runtime" in payload else "error",
            "Runtime metrics are recomputed from the trial log."
            if "runtime" in payload
            else "Runtime verifier payload is missing.",
        )
    )
    checks.append(
        Check(
            "N50 local-metric verifier",
            "pass" if local_status == "independent" else "pending",
            "Local E-OSPA/RMSE/CardErr are independently recomputed from per-trial local rows."
            if local_status == "independent"
            else f"Local metrics are still `{local_status}`; switch to the rerun report with per-trial local rows when it finishes.",
        )
    )

    if HELDOUT_N50_JSON.exists():
        heldout = json.loads(read_text(HELDOUT_N50_JSON))
        checks.extend(heldout_n50_checks(heldout))
    elif HELDOUT_SANITY_JSON.exists():
        heldout = json.loads(read_text(HELDOUT_SANITY_JSON))
        config = heldout.get("config", {})
        checks.append(
            Check(
                "held-out scenario evidence",
                "warning",
                "Tracked N5 base-seed sanity evidence exists "
                f"(base seed {config.get('base_seed')}, {config.get('trials')} trials), "
                "but a paper-grade held-out N50 or packet-loss-family validation is still needed.",
            )
        )
    else:
        checks.append(
            Check(
                "held-out scenario evidence",
                "pending",
                "No generated held-out base-seed or packet-loss-family evidence artifact detected.",
            )
        )
    return checks


def heldout_n50_checks(heldout: dict[str, object]) -> list[Check]:
    checks: list[Check] = []
    config = heldout.get("config", {})
    if not isinstance(config, dict):
        config = {}
    trials = safe_int(config.get("trials"))
    base_seed = safe_int(config.get("base_seed"), -1)
    trial_seeds = config.get("trial_seeds", [])
    trial_seed_count = len(trial_seeds) if isinstance(trial_seeds, list) else 0
    protocol_ok = trials >= 50 and base_seed == HELDOUT_BASE_SEED and trial_seed_count >= trials
    checks.append(
        Check(
            "held-out N50 protocol",
            "pass" if protocol_ok else "warning",
            f"Held-out run uses base seed {base_seed}, {trials} trials, and {trial_seed_count} parsed trial seeds."
            if protocol_ok
            else "Paper-grade held-out validation should use "
            f"base seed {HELDOUT_BASE_SEED}, at least 50 trials, and a parsed trial-seed list matching the trial count; "
            f"found base seed {base_seed}, {trials} trials, {trial_seed_count} trial seeds.",
        )
    )

    artifacts_ok = HELDOUT_N50_MANIFEST.exists() and HELDOUT_N50_FRAGMENT.exists()
    checks.append(
        Check(
            "held-out N50 generated artifacts",
            "pass" if artifacts_ok else "error",
            "`HELDOUT_N50_MANIFEST.md` and `heldout_n50_section.tex` exist."
            if artifacts_ok
            else "Held-out N50 JSON exists but the generated manifest or manuscript fragment is missing.",
        )
    )

    missing_mean = []
    missing_mean.extend(missing_metric_entries(heldout, "network", ARM_ORDER, NETWORK_METRICS))
    missing_mean.extend(missing_metric_entries(heldout, "local", ARM_ORDER, LOCAL_METRICS))
    checks.append(
        Check(
            "held-out N50 mean metric coverage",
            "pass" if not missing_mean else "error",
            "Held-out N50 means cover all three arms and all network/local manuscript metrics."
            if not missing_mean
            else "Held-out N50 mean metric payload is incomplete: " + "; ".join(missing_mean),
        )
    )

    missing_paired = []
    missing_paired.extend(missing_paired_entries(heldout, "paired_network", NETWORK_METRICS))
    missing_paired.extend(missing_paired_entries(heldout, "paired_local", LOCAL_METRICS))
    checks.append(
        Check(
            "held-out N50 paired metric coverage",
            "pass" if not missing_paired else "error",
            "Held-out N50 paired reductions include CI, wins, and sign-test p-values for full and reference-only arms."
            if not missing_paired
            else "Held-out N50 paired payload is incomplete: " + "; ".join(missing_paired),
        )
    )

    local = heldout.get("local", {})
    paired_local = heldout.get("paired_local", {})
    full_rmse_pct = None
    ref_rmse_pct = None
    tuned_rmse = full_rmse = ref_rmse = None
    if isinstance(local, dict):
        tuned_rmse = local.get(ARM_ORDER[0], {}).get("RMSE") if isinstance(local.get(ARM_ORDER[0]), dict) else None
        full_rmse = local.get(FULL_ARM, {}).get("RMSE") if isinstance(local.get(FULL_ARM), dict) else None
        ref_rmse = local.get(REF_ONLY_ARM, {}).get("RMSE") if isinstance(local.get(REF_ONLY_ARM), dict) else None
    if isinstance(paired_local, dict):
        full_payload = paired_local.get(FULL_ARM, {})
        ref_payload = paired_local.get(REF_ONLY_ARM, {})
        if isinstance(full_payload, dict) and isinstance(full_payload.get("RMSE"), dict):
            full_rmse_pct = reduction_percent(full_payload["RMSE"].get("reduction"))
        if isinstance(ref_payload, dict) and isinstance(ref_payload.get("RMSE"), dict):
            ref_rmse_pct = reduction_percent(ref_payload["RMSE"].get("reduction"))
    try:
        means_support = float(full_rmse) < float(tuned_rmse) and float(full_rmse) < float(ref_rmse)
    except (TypeError, ValueError):
        means_support = False
    reduction_support = (
        full_rmse_pct is not None
        and ref_rmse_pct is not None
        and full_rmse_pct > 0
        and full_rmse_pct > ref_rmse_pct
    )
    checks.append(
        Check(
            "held-out N50 barycenter-vs-reference separation",
            "pass" if means_support and reduction_support else "warning",
            "Held-out N50 supports the mechanism separation: full barycenter RMSE is below tuned and reference-only, "
            f"with RMSE reduction {full_rmse_pct:.2f}% vs reference-only {ref_rmse_pct:.2f}%."
            if means_support and reduction_support
            else "Held-out N50 exists, but the RMSE mechanism-separation check is weak or missing "
            f"(tuned={tuned_rmse}, full={full_rmse}, ref-only={ref_rmse}, "
            f"full reduction={full_rmse_pct}, ref-only reduction={ref_rmse_pct}).",
        )
    )

    scenario_status = "pass" if protocol_ok else "warning"
    checks.append(
        Check(
            "held-out scenario evidence",
            scenario_status,
            "Paper-grade held-out base-seed N50 evidence exists and strict structure checks are recorded above."
            if scenario_status == "pass"
            else "Held-out evidence artifact exists, but the protocol does not yet match the planned base-seed-11 N50 validation.",
        )
    )
    return checks


def summarize(checks: list[Check]) -> str:
    statuses = {check.status for check in checks}
    if "error" in statuses:
        return "error"
    if "pending" in statuses:
        return "draft_with_pending_gates"
    if "warning" in statuses:
        return "candidate_with_warnings"
    return "submission_candidate"


def write_outputs(checks: list[Check]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    overall = summarize(checks)
    payload = {
        "overall_status": overall,
        "checks": [check.as_dict() for check in checks],
    }
    READINESS_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    by_status: dict[str, list[Check]] = {"error": [], "pending": [], "warning": [], "pass": []}
    for check in checks:
        by_status.setdefault(check.status, []).append(check)

    lines = [
        "# TAES Submission Readiness Report\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/check_submission_readiness.py`.\n\n",
        f"- Overall status: `{overall}`\n",
        f"- Manuscript PDF: `{MAIN_PDF.relative_to(REPO)}`\n",
        f"- Machine-readable report: `{READINESS_JSON.relative_to(REPO)}`\n\n",
    ]
    for status in ["error", "pending", "warning", "pass"]:
        entries = by_status.get(status, [])
        if not entries:
            continue
        title = {
            "error": "Hard Errors",
            "pending": "Pending Gates",
            "warning": "Warnings",
            "pass": "Passed Checks",
        }[status]
        lines.append(f"## {title}\n\n")
        for entry in entries:
            lines.append(f"- `{entry.gate}`: {entry.detail}\n")
        lines.append("\n")
    READINESS_MD.write_text("".join(lines), encoding="utf-8")

    if by_status["error"]:
        raise SystemExit(1)


def main() -> None:
    tex = read_text(MAIN_TEX) if MAIN_TEX.exists() else ""
    bib = read_text(BIB) if BIB.exists() else ""
    checks = []
    checks.extend(file_checks())
    checks.extend(manuscript_checks(tex, bib))
    checks.extend(cover_letter_checks())
    checks.extend(pdf_checks())
    checks.extend(evidence_checks())
    checks.extend(bundle_checks())
    write_outputs(checks)


if __name__ == "__main__":
    main()
