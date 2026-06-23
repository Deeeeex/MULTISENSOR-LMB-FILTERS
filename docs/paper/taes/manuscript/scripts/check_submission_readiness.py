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
VERIFICATION_JSON = OUT / "n50_verification.json"
HELDOUT_SANITY_JSON = OUT / "heldout_sanity_evidence.json"
READINESS_JSON = OUT / "submission_readiness.json"
READINESS_MD = OUT / "SUBMISSION_READINESS_REPORT.md"


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


def file_checks() -> list[Check]:
    checks: list[Check] = []
    required = [
        MAIN_TEX,
        MAIN_PDF,
        BIB,
        ROOT / "IEEEtaes.cls",
        ROOT / "IEEEtaes.bst",
        VERIFICATION_JSON,
        OUT / "N50_EVIDENCE_MANIFEST.md",
        OUT / "REFERENCE_BASELINE_MANIFEST.md",
        OUT / "HELDOUT_SANITY_MANIFEST.md",
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
    return checks


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

    if HELDOUT_SANITY_JSON.exists():
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
    checks.extend(pdf_checks())
    checks.extend(evidence_checks())
    write_outputs(checks)


if __name__ == "__main__":
    main()
