#!/usr/bin/env python3
"""Machine-checkable TAES submission-readiness snapshot.

This script is intentionally stricter about mechanical consistency than about
research completeness. It fails the build for missing files, broken citations,
or broken references, but it records author metadata, held-out validation, and
local-metric verifier closure as pending gates while the draft is still moving.
It reports both portal-level readiness and content-level readiness because
author/funding/repository metadata may remain provisional during internal paper
readiness review.
"""

from __future__ import annotations

import hashlib
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
SUBMISSION_INDEX = ROOT / "SUBMISSION_PACKAGE_INDEX.md"
SUPPLEMENTARY_EVIDENCE_PACKAGE = ROOT / "SUPPLEMENTARY_EVIDENCE_PACKAGE.md"
EVIDENCE_SOURCES = ROOT / "evidence_sources.json"
REQUIREMENTS_DOC = REPO / "docs" / "TAES_SUBMISSION_REQUIREMENTS_CN.md"
REGULAR_TEMPLATE = REPO / "docs" / "paper" / "taes" / "template_regular" / "IEEE_TAES_orig-research" / "TAES_template.tex"
TEMPLATE_ZIP = REPO / "docs" / "paper" / "taes" / "TAES_Template.zip"
VERIFICATION_JSON = OUT / "n50_verification.json"
HELDOUT_SANITY_JSON = OUT / "heldout_sanity_evidence.json"
HELDOUT_N50_JSON = OUT / "heldout_n50_evidence.json"
HELDOUT_N50_MANIFEST = OUT / "HELDOUT_N50_MANIFEST.md"
HELDOUT_N50_FRAGMENT = OUT / "heldout_n50_section.tex"
STRESS_HARSH_JSON = OUT / "stress_harsh_evidence.json"
STRESS_HARSH_MANIFEST = OUT / "STRESS_HARSH_MANIFEST.md"
STRESS_HARSH_FRAGMENT = OUT / "stress_harsh_section.tex"
STRESS_HARSH_SUMMARY = OUT / "stress_harsh_summary_sentence.tex"
SCENARIO_FAMILY_JSON = OUT / "scenario_family_evidence.json"
SCENARIO_FAMILY_MANIFEST = OUT / "SCENARIO_FAMILY_MANIFEST.md"
SCENARIO_FAMILY_FRAGMENT = OUT / "scenario_family_section.tex"
SCENARIO_FAMILY_SUMMARY = OUT / "scenario_family_summary_sentence.tex"
REPRO_LEDGER_JSON = OUT / "reproducibility_ledger.json"
REPRO_LEDGER_MANIFEST = OUT / "REPRODUCIBILITY_LEDGER_MANIFEST.md"
REPRO_LEDGER_ROWS = OUT / "reproducibility_ledger_rows.tex"
REPRO_LEDGER_TABLE = OUT / "reproducibility_ledger_table.tex"
PDF_VISUAL_QA_JSON = OUT / "pdf_visual_qa.json"
PDF_VISUAL_QA_MANIFEST = OUT / "PDF_VISUAL_QA_MANIFEST.md"
BIB_DOI_JSON = OUT / "bibtex_doi_verification.json"
BIB_DOI_MANIFEST = OUT / "BIBTEX_DOI_VERIFICATION.md"
BUNDLE_MANIFEST_JSON = OUT / "submission_bundle_manifest.json"
BUNDLE_MANIFEST_MD = OUT / "SUBMISSION_BUNDLE_MANIFEST.md"
READINESS_JSON = OUT / "submission_readiness.json"
READINESS_MD = OUT / "SUBMISSION_READINESS_REPORT.md"

HELDOUT_BASE_SEED = 11
STRESS_BASE_SEED = 21
STRESS_P_DROP_LEVELS = [0.2, 0.35, 0.5, 0.7]
STRESS_P_DROP_LEVEL_COUNTS = [1, 3, 2, 2]
SCENARIO_FAMILY_KEYS = [
    "scenario_topology_ring_report",
    "scenario_partial_fov35_report",
    "scenario_full_topology_report",
]
ARM_ORDER = [
    "Tuned spatial-KLA AA",
    "Neighborhood label-barycenter spatial-KLA AA",
    "Neighborhood reference-only label-consensus spatial-KLA AA",
]
FULL_ARM = ARM_ORDER[1]
REF_ONLY_ARM = ARM_ORDER[2]
NETWORK_METRICS = ["OSPA", "Loc. disag.", "Card. disp."]
LOCAL_METRICS = ["E-OSPA", "RMSE", "CardErr"]
METADATA_PLACEHOLDER_GATE = "submission metadata placeholders"
BUNDLE_REQUIRED_PATHS = [
    "build.sh",
    "SUBMISSION_PACKAGE_INDEX.md",
    "SUPPLEMENTARY_EVIDENCE_PACKAGE.md",
    "scripts/check_submission_readiness.py",
    "scripts/create_submission_bundle.py",
    "scripts/evidence_sources.py",
    "scripts/extract_heldout_sanity_evidence.py",
    "scripts/extract_n50_evidence.py",
    "scripts/extract_reference_baselines.py",
    "scripts/extract_scenario_family_evidence.py",
    "scripts/extract_stress_evidence.py",
    "scripts/render_figures.py",
    "scripts/render_pdf_visual_qa.py",
    "scripts/render_reproducibility_ledger.py",
    "scripts/verify_bibtex_dois.py",
    "scripts/verify_n50_evidence.py",
]
BUNDLE_BUILD_FALLBACK_MARKERS = [
    "TAES_EVIDENCE_MODE",
    "bundled",
    "Raw evidence sources unavailable; compiling from bundled generated fragments.",
    "Skipping source-bundle and readiness regeneration in bundled-fragment mode.",
]
REQUIREMENTS_DOC_MARKERS = [
    "2026-06-24 Live Source Refresh",
    "https://ieee.atyponrex.com/journal/taes",
    "Regular Paper",
    "200 USD",
    "two-column",
    "single-spaced",
    "10-point font",
    "AI-generated content",
    "Acknowledgments",
    "ORCID",
    "single-blind",
    "Letters category",
    "TAES neither encourages nor discourages",
    "Target Tracking and Multi-Sensor Systems",
    "data fusion",
    "decentralized/distributed estimation",
    "Tectonic",
]


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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def generated_tex_fragments() -> str:
    if not OUT.exists():
        return ""
    return "\n".join(read_text(path) for path in sorted(OUT.glob("*.tex")))


def command_body(tex: str, name: str) -> str:
    match = re.search(rf"\\{name}{{([^}}]+)}}", tex, flags=re.DOTALL)
    return match.group(1).strip() if match else ""


def normalized_spaces(value: str) -> str:
    return re.sub(r"\s+", " ", value).strip()


def markboth_right(tex: str) -> str:
    match = re.search(r"\\markboth\s*{[^{}]*}\s*{([^{}]+)}", tex, flags=re.DOTALL)
    return normalized_spaces(match.group(1)) if match else ""


def markdown_table_value(text: str, field: str) -> str:
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|"):
            continue
        cells = [cell.strip() for cell in stripped.strip("|").split("|")]
        if len(cells) >= 2 and normalized_spaces(cells[0]).lower() == field.lower():
            return normalized_spaces(cells[1])
    return ""


def environment_body(tex: str, name: str) -> str:
    match = re.search(rf"\\begin{{{name}}}(.+?)\\end{{{name}}}", tex, flags=re.DOTALL)
    return match.group(1).strip() if match else ""


def paper_facing_body(tex: str) -> str:
    match = re.search(r"\\begin{abstract}(.+?)\\section\*{ACKNOWLEDGMENT}", tex, flags=re.DOTALL)
    return match.group(1).strip() if match else tex


def placeholder_hits(*texts: str) -> list[str]:
    combined = "\n".join(texts)
    hits: list[str] = []
    literal_patterns = [
        "FIRST AUTHOR",
        "SECOND AUTHOR",
        "THIRD AUTHOR",
        "First Author",
        "Second Author",
        "Third Author",
        "AUTHOR ET AL.",
        "TAES.2026.Doi Number",
        "Draft",
        "XX",
        "Institution, City, Country",
        "Month 00",
    ]
    for pattern in literal_patterns:
        if pattern in combined and pattern not in hits:
            hits.append(pattern)
    bracket_patterns = [
        r"\[Corresponding Author Name\]",
        r"\[Email\]",
        r"\[FIRST AUTHOR\]",
        r"\[Funding Agency\]",
        r"\[Grant Number\]",
        r"\[Institution\]",
        r"\[City\]",
        r"\[Country\]",
        r"\[All authors\]",
        r"\[preprint URL\]",
        r"\[none / URL\]",
        r"\[none / disclosure\]",
        r"\[optional\]",
        r"\[author@example\.com\]",
        r"\[repository DOI/URL\]",
    ]
    for pattern in bracket_patterns:
        for value in re.findall(pattern, combined):
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


def float_list_matches(values: object, expected: list[float], tol: float = 1e-9) -> bool:
    if not isinstance(values, list) or len(values) != len(expected):
        return False
    try:
        return all(abs(float(value) - target) <= tol for value, target in zip(values, expected))
    except (TypeError, ValueError):
        return False


def int_list_matches(values: object, expected: list[int]) -> bool:
    if not isinstance(values, list) or len(values) != len(expected):
        return False
    try:
        return [int(value) for value in values] == expected
    except (TypeError, ValueError):
        return False


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
        SUBMISSION_INDEX,
        SUPPLEMENTARY_EVIDENCE_PACKAGE,
        ROOT / "IEEEtaes.cls",
        ROOT / "IEEEtaes.bst",
        REQUIREMENTS_DOC,
        REGULAR_TEMPLATE,
        TEMPLATE_ZIP,
        VERIFICATION_JSON,
        OUT / "N50_EVIDENCE_MANIFEST.md",
        OUT / "REFERENCE_BASELINE_MANIFEST.md",
        OUT / "HELDOUT_SANITY_MANIFEST.md",
        OUT / "REPRODUCIBILITY_LEDGER_MANIFEST.md",
        OUT / "reproducibility_ledger.json",
        OUT / "reproducibility_ledger_rows.tex",
        OUT / "reproducibility_ledger_table.tex",
        OUT / "pdf_visual_qa.json",
        OUT / "PDF_VISUAL_QA_MANIFEST.md",
        OUT / "bibtex_doi_verification.json",
        OUT / "BIBTEX_DOI_VERIFICATION.md",
        OUT / "SUBMISSION_BUNDLE_MANIFEST.md",
        ROOT / "scripts" / "create_submission_bundle.py",
        ROOT / "scripts" / "extract_scenario_family_evidence.py",
        ROOT / "scripts" / "extract_stress_evidence.py",
        ROOT / "scripts" / "render_pdf_visual_qa.py",
        ROOT / "scripts" / "render_reproducibility_ledger.py",
        ROOT / "scripts" / "verify_bibtex_dois.py",
    ]
    for path in required:
        status = "pass" if path.exists() else "error"
        detail = str(path.relative_to(REPO)) if path.exists() else f"missing {path.relative_to(REPO)}"
        checks.append(Check("required artifact", status, detail))
    return checks


def requirements_doc_checks() -> list[Check]:
    if not REQUIREMENTS_DOC.exists():
        return [Check("TAES requirements live-source refresh", "error", "`docs/TAES_SUBMISSION_REQUIREMENTS_CN.md` is missing.")]
    text = read_text(REQUIREMENTS_DOC)
    missing = [marker for marker in REQUIREMENTS_DOC_MARKERS if marker not in text]
    stale_patterns = [
        "本机未安装 TeX",
        "尚未本地编译验证",
        "2026-06-24 Online Spot-Check",
    ]
    stale = [pattern for pattern in stale_patterns if pattern in text]
    ok = not missing and not stale
    return [
        Check(
            "TAES requirements live-source refresh",
            "pass" if ok else "warning",
            "TAES requirements document records the 2026-06-24 live official-page refresh, portal/type/page-charge/AI/ORCID/preprint/technical-area markers, and current local TeX verification status."
            if ok
            else "TAES requirements document is missing live-source markers or contains stale caveats: "
            + "missing="
            + (", ".join(missing) if missing else "none")
            + "; stale="
            + (", ".join(stale) if stale else "none"),
        )
    ]


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

    uncited = sorted(available - cited)
    checks.append(
        Check(
            "uncited BibTeX entries",
            "pass" if not uncited else "warning",
            "All BibTeX entries are cited in the manuscript."
            if not uncited
            else f"BibTeX entries are present but uncited: {', '.join(uncited)}",
        )
    )

    tex_with_generated_labels = tex + "\n" + generated_tex_fragments()
    missing_refs = sorted(refs(tex) - labels(tex_with_generated_labels))
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

    cover_letter_text = read_text(COVER_LETTER) if COVER_LETTER.exists() else ""
    placeholders = placeholder_hits(tex, cover_letter_text)
    checks.append(
        Check(
            "submission metadata placeholders",
            "pending" if placeholders else "pass",
            f"{len(placeholders)} manuscript or cover-letter metadata placeholder tokens remain: {', '.join(placeholders)}"
            if placeholders
            else "No obvious manuscript or cover-letter metadata placeholder tokens detected.",
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
        "does not estimate new Bernoulli existence probabilities",
        "passes the surviving active-track existence scores through",
        "rewrites only the label and moment fields",
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

    algorithm_markers = [
        r"\textbf{Input:}",
        r"\textbf{1 Reference:}",
        r"\textbf{2 Match:}",
        r"\textbf{3 Project:}",
        r"\textbf{4 Existence:}",
        r"\textbf{5 Iterate:}",
        "rewrite labels/moments only",
        "no global label dictionary is read",
    ]
    missing_algorithm_markers = [marker for marker in algorithm_markers if marker not in tex]
    checks.append(
        Check(
            "method algorithm-box markers",
            "pass" if not missing_algorithm_markers else "warning",
            "Method section preserves the compact stepwise implementation box for reference, matching, projection, existence pass-through, and iteration."
            if not missing_algorithm_markers
            else "Method algorithm box is missing markers: " + "; ".join(missing_algorithm_markers),
        )
    )

    body = paper_facing_body(tex)
    story_markers = [
        "component-correspondence failure",
        "Scalar AA/KLA weights decide how much probability mass to trust",
        "residual correspondence failure",
        "not another scalar-weight search",
        "A held-out 50-trial replication",
        "full-versus-reference separation",
        "held-out 50-trial replication",
        "reference-only ablation",
    ]
    missing_story_markers = [marker for marker in story_markers if marker not in body]
    checks.append(
        Check(
            "first-page narrative markers",
            "pass" if not missing_story_markers else "warning",
            "Abstract and introduction preserve the correspondence-failure framing, scalar-weight boundary, held-out replication, and reference-only mechanism test."
            if not missing_story_markers
            else "First-page narrative is missing markers: " + "; ".join(missing_story_markers),
        )
    )

    related_work_markers = [
        "not another density-pooling rule",
        "correspondence map",
        "targeted output-space projection",
        "orthogonal to those weighting choices",
        "graph-local reference label set",
        "matched moment barycenters",
    ]
    missing_related_markers = [marker for marker in related_work_markers if marker not in body]
    checks.append(
        Check(
            "related-work positioning markers",
            "pass" if not missing_related_markers else "warning",
            "Related Work keeps the method positioned as a correspondence/projection layer rather than another AA/KLA weighting rule."
            if not missing_related_markers
            else "Related Work positioning is missing markers: " + "; ".join(missing_related_markers),
        )
    )

    theory_scope_markers = [
        r"\begin{assumption}[Analysis scope]",
        "after the upstream AA existence consumer has selected active tracks",
        "hold births and deaths outside the projection layer",
        "close target crossings or cardinality mismatches",
        "not assume",
    ]
    missing_theory_scope = [marker for marker in theory_scope_markers if marker not in tex]
    checks.append(
        Check(
            "structural-property scope markers",
            "pass" if not missing_theory_scope else "warning",
            "Structural Properties state the analysis scope before the propositions, including active-track conditioning and excluded lifecycle/crossing cases."
            if not missing_theory_scope
            else "Structural-property scope wording is incomplete; missing markers: "
            + "; ".join(missing_theory_scope),
        )
    )

    internal_patterns = [
        r"\bTODO\b",
        r"\bTBD\b",
        r"before submission",
        r"will be provided",
        r"to be completed",
        r"not yet",
        r"placeholder",
        r"camera-ready",
    ]
    internal_hits = sorted(
        {
            pattern
            for pattern in internal_patterns
            if re.search(pattern, body, flags=re.IGNORECASE)
        }
    )
    checks.append(
        Check(
            "paper-facing wording hygiene",
            "pass" if not internal_hits else "warning",
            "Paper-facing abstract/body text contains no internal-status tokens such as TODO, TBD, before-submission notes, or placeholder language."
            if not internal_hits
            else "Paper-facing abstract/body text contains internal-status patterns: " + "; ".join(internal_hits),
        )
    )

    discussion_interpretation_markers = [
        "support a specific interpretation",
        "target-wise AA weight routing is held fixed",
        "reference-only projection reduces network disagreement",
        "Adding matched moment barycenters creates the spatial tracking separation",
        "complementary to AA/KLA weighting",
        "not as a replacement for density pooling",
    ]
    missing_discussion_interpretation = [
        marker for marker in discussion_interpretation_markers if marker not in body
    ]
    checks.append(
        Check(
            "discussion interpretation markers",
            "pass" if not missing_discussion_interpretation else "warning",
            "Discussion opens by interpreting the ablation and boundary evidence as a correspondence/projection mechanism rather than a density-pooling replacement."
            if not missing_discussion_interpretation
            else "Discussion interpretation wording is incomplete; missing markers: "
            + "; ".join(missing_discussion_interpretation),
        )
    )

    stress_boundary_markers = [
        "stress check broadens packet-loss severity",
        "preserves the same formation-family assumptions",
        "should not be read as a substitute",
        "sparse-topology",
        "partial-field-of-view",
        "maneuvering-target",
        "covariance-consistency",
    ]
    missing_stress_boundary = [marker for marker in stress_boundary_markers if marker not in body]
    checks.append(
        Check(
            "stress generalization boundary wording",
            "pass" if not missing_stress_boundary else "warning",
            "Discussion preserves the boundary that harsh-loss, topology-ring, and partial-FOV checks do not substitute for maneuvering-target or covariance-consistency validation."
            if not missing_stress_boundary
            else "Stress/generalization boundary wording is incomplete; missing markers: "
            + "; ".join(missing_stress_boundary),
        )
    )

    failure_mode_markers = [
        "assignment ambiguity",
        "cardinalities lack a neighborhood majority",
        "plausible but wrong correspondence",
        "assignment-margin",
        "fall back to reference-only or upstream AA output",
        "isolate the label-map and moment-barycenter mechanisms",
    ]
    missing_failure_mode = [marker for marker in failure_mode_markers if marker not in body]
    checks.append(
        Check(
            "projection failure-mode boundary wording",
            "pass" if not missing_failure_mode else "warning",
            "Discussion names the main assignment-ambiguity failure mode and ties recursive deployment to explicit correspondence guards."
            if not missing_failure_mode
            else "Projection failure-mode boundary wording is incomplete; missing markers: "
            + "; ".join(missing_failure_mode),
        )
    )
    return checks


def doi_resolver_checks(bib: str) -> list[Check]:
    artifacts = [BIB_DOI_JSON, BIB_DOI_MANIFEST]
    missing = [path.relative_to(REPO).as_posix() for path in artifacts if not path.exists()]
    if missing:
        return [
            Check(
                "BibTeX DOI resolver verification",
                "error",
                "Generated DOI resolver verification artifacts are missing: " + ", ".join(missing),
            )
        ]

    payload = json.loads(read_text(BIB_DOI_JSON))
    bib_sha = hashlib.sha256(bib.encode("utf-8")).hexdigest()
    available = bib_keys(bib)
    status = str(payload.get("status", "missing"))
    payload_sha = str(payload.get("bib_sha256", ""))
    entry_count = safe_int(payload.get("entry_count"), -1)
    resolved_count = safe_int(payload.get("resolved_count"), -1)
    missing_doi = payload.get("missing_doi", [])
    unresolved = payload.get("unresolved", [])
    used_cache = bool(payload.get("used_cache", False))

    ok = (
        status == "pass"
        and payload_sha == bib_sha
        and entry_count == len(available)
        and resolved_count == len(available)
        and not missing_doi
        and not unresolved
    )
    cache_note = " A same-BibTeX cached resolver pass was reused." if used_cache else ""
    return [
        Check(
            "BibTeX DOI resolver verification",
            "pass" if ok else "error",
            f"DOI resolver verification covers {resolved_count}/{entry_count} BibTeX entries with current BibTeX SHA-256 `{payload_sha}`.{cache_note}"
            if ok
            else "DOI resolver verification is incomplete or stale: "
            f"status={status}, bib_sha_matches={payload_sha == bib_sha}, "
            f"entry_count={entry_count}, expected={len(available)}, resolved_count={resolved_count}, "
            f"missing_doi={missing_doi}, unresolved={unresolved}.",
        )
    ]


def cover_letter_checks(tex: str) -> list[Check]:
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
    positioning_markers = [
        "distributed aerospace and surveillance systems",
        "another scalar-weight search",
        "reference-only ablation",
        "fixed-design and report-driven",
        "active-output label-and-moment layer",
        "assignment ambiguity",
    ]
    missing_positioning = [marker for marker in positioning_markers if marker not in text]
    title = normalized_spaces(command_body(tex, "title"))
    running_head = markboth_right(tex)
    portal_title = markdown_table_value(text, "Title")
    portal_running_head = markdown_table_value(text, "Running head")
    sync_checks = {
        "cover-letter title sentence": bool(title) and f'titled "{title}"' in text,
        "portal title": bool(title) and portal_title == title,
        "portal running head": bool(running_head) and portal_running_head == running_head,
        "portal journal": markdown_table_value(text, "Journal") == "IEEE Transactions on Aerospace and Electronic Systems",
        "portal manuscript type": markdown_table_value(text, "Manuscript type") == "Regular Paper",
        "portal technical area": markdown_table_value(text, "Technical area") == "Target Tracking and Multi-Sensor Systems",
        "cover-letter originality": "original work" in text and "not under consideration elsewhere" in text,
        "simulation-data statement": "simulated multi-target tracking data" in text and "simulated multi-target tracking data" in tex,
        "AI disclosure": "OpenAI Codex" in text and "OpenAI Codex" in tex and "authors remain responsible" in tex.lower(),
    }
    failed_sync = [name for name, ok in sync_checks.items() if not ok]
    return [
        Check(
            "cover letter and portal metadata draft",
            "pass" if not missing else "warning",
            "Cover-letter draft and portal metadata checklist exist with manuscript type, technical area, originality, AI disclosure, ORCID, and repository placeholders."
            if not missing
            else "Cover-letter draft exists but is missing markers: " + "; ".join(missing),
        ),
        Check(
            "cover letter and portal metadata source synchronization",
            "pass" if not failed_sync else "warning",
            "Cover-letter text and portal metadata table match `main.tex` title, running head, manuscript type, technical area, simulated-data statement, and AI disclosure."
            if not failed_sync
            else "Cover-letter or portal metadata drift detected: " + "; ".join(failed_sync),
        ),
        Check(
            "cover letter paper-positioning markers",
            "pass" if not missing_positioning else "warning",
            "Cover letter states the TAES fit, scalar-weight boundary, reference-only ablation, fixed-design evidence chain, and projection limitations."
            if not missing_positioning
            else "Cover-letter paper-positioning wording is incomplete; missing markers: "
            + "; ".join(missing_positioning),
        ),
    ]


def submission_package_index_checks() -> list[Check]:
    if not SUBMISSION_INDEX.exists():
        return [Check("submission package index", "warning", "`SUBMISSION_PACKAGE_INDEX.md` is missing.")]
    text = read_text(SUBMISSION_INDEX)
    required_markers = [
        "Final Upload Set",
        "Internal QA Artifacts",
        "Metadata Placeholders",
        "Final Rebuild Sequence",
        "main.pdf",
        "tmp/submission_bundle/taes_label_barycenter_submission_source.zip",
        "COVER_LETTER_AND_METADATA_DRAFT.md",
        "SUPPLEMENTARY_EVIDENCE_PACKAGE.md",
        "generated/SUBMISSION_READINESS_REPORT.md",
        "generated/SUBMISSION_BUNDLE_MANIFEST.md",
        "generated/REPRODUCIBILITY_LEDGER_MANIFEST.md",
        "cover letter and portal metadata source synchronization",
    ]
    missing = [marker for marker in required_markers if marker not in text]
    return [
        Check(
            "submission package index",
            "pass" if not missing else "warning",
            "Submission package index maps final uploads, internal QA artifacts, metadata placeholders, and the final rebuild sequence."
            if not missing
            else "Submission package index exists but is missing markers: " + "; ".join(missing),
        )
    ]


def reproducibility_ledger_checks() -> list[Check]:
    artifacts = [REPRO_LEDGER_JSON, REPRO_LEDGER_MANIFEST, REPRO_LEDGER_ROWS, REPRO_LEDGER_TABLE]
    missing = [path.relative_to(REPO).as_posix() for path in artifacts if not path.exists()]
    if missing:
        return [
            Check(
                "reproducibility ledger",
                "error",
                "Generated reproducibility ledger artifacts are missing: " + ", ".join(missing),
            )
        ]

    payload = json.loads(read_text(REPRO_LEDGER_JSON))
    rows = payload.get("rows", [])
    names = {str(row.get("name", "")) for row in rows if isinstance(row, dict)}
    required_names = {
        "Primary AA N50",
        "Held-out AA N50",
        "Contextual GA N50",
        "Harsh-loss AA N50",
        "Scenario-family boundary checks",
        "Independent verifier",
    }
    missing_names = sorted(required_names - names)
    rows_tex = read_text(REPRO_LEDGER_ROWS)
    table_tex = read_text(REPRO_LEDGER_TABLE)
    marker_ok = all(name in rows_tex and name in table_tex for name in required_names)
    return [
        Check(
            "reproducibility ledger",
            "pass" if not missing_names and marker_ok else "error",
            "Generated reproducibility ledger covers primary paired AA evidence, held-out robustness, harsh-loss stress evidence, scenario-family boundary checks, contextual GA rows, and the independent verifier."
            if not missing_names and marker_ok
            else "Generated reproducibility ledger is incomplete; missing names: "
            + (", ".join(missing_names) if missing_names else "none")
            + f"; rows_tex_marker_ok={marker_ok}.",
        )
    ]


def supplementary_evidence_package_checks() -> list[Check]:
    if not SUPPLEMENTARY_EVIDENCE_PACKAGE.exists():
        return [
            Check(
                "supplementary evidence package",
                "warning",
                "`SUPPLEMENTARY_EVIDENCE_PACKAGE.md` is missing.",
            )
        ]
    text = read_text(SUPPLEMENTARY_EVIDENCE_PACKAGE)
    required_markers = [
        "Candidate Supplementary Material",
        "Response-ready evidence",
        "Boundary control",
        "generated/heldout_n50_section.tex",
        "generated/stress_harsh_section.tex",
        "generated/scenario_family_section.tex",
        "generated/reproducibility_ledger_table.tex",
        "not a tuning loop",
        "do not substitute for target-maneuver, covariance-consistency, or recursive-online validation",
        "generated fragments should not be edited directly",
    ]
    missing = [marker for marker in required_markers if marker not in text]
    return [
        Check(
            "supplementary evidence package",
            "pass" if not missing else "warning",
            "Supplementary/response evidence package maps generated robustness fragments, provenance material, and interpretation boundaries."
            if not missing
            else "Supplementary evidence package is missing markers: " + "; ".join(missing),
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
    files = payload.get("files", [])
    manifest_entries = {
        str(entry.get("path", "")): entry
        for entry in files
        if isinstance(entry, dict) and entry.get("path")
    }
    bundled_paths = set(manifest_entries)
    missing_required = [path for path in BUNDLE_REQUIRED_PATHS if path not in bundled_paths]

    bundle_hash_ok = bundle_path.exists() and len(checksum) == 64 and sha256_file(bundle_path) == checksum
    ok = bool(bundle_rel) and bundle_path.exists() and file_count >= 10 and bundle_hash_ok

    missing_local: list[str] = []
    stale_entries: list[str] = []
    malformed_sha: list[str] = []
    for rel, entry in manifest_entries.items():
        local_path = ROOT / rel
        expected_sha = str(entry.get("sha256", ""))
        if not local_path.exists():
            missing_local.append(rel)
        elif len(expected_sha) != 64:
            malformed_sha.append(rel)
        elif sha256_file(local_path) != expected_sha:
            stale_entries.append(rel)
    freshness_ok = not missing_local and not stale_entries and not malformed_sha

    build_script = read_text(ROOT / "build.sh") if (ROOT / "build.sh").exists() else ""
    missing_fallback_markers = [marker for marker in BUNDLE_BUILD_FALLBACK_MARKERS if marker not in build_script]
    return [
        Check(
            "submission source bundle",
            "pass" if ok and not missing_required else "warning",
            f"Clean source bundle exists at `{bundle_rel}` with {file_count} files and SHA-256 `{checksum}`."
            if ok and not missing_required
            else "Submission source bundle manifest exists, but the bundle path, file count, checksum, or required reproducibility scripts are incomplete. Missing required paths: "
            + (", ".join(missing_required) if missing_required else "none")
            + ".",
        ),
        Check(
            "submission source bundle reproducibility scripts",
            "pass" if not missing_required else "warning",
            "Source bundle includes `build.sh` and the manuscript evidence/render/readiness scripts needed to regenerate generated fragments."
            if not missing_required
            else "Source bundle is missing required reproducibility scripts: " + ", ".join(missing_required),
        ),
        Check(
            "submission source bundle freshness",
            "pass" if freshness_ok and file_count == len(manifest_entries) else "error",
            "Source-bundle manifest hashes match the current manuscript, generated fragments, figures, and reproducibility scripts."
            if freshness_ok and file_count == len(manifest_entries)
            else "Source-bundle manifest is stale or malformed: "
            f"file_count={file_count}, manifest_entries={len(manifest_entries)}, "
            "missing local files="
            + (", ".join(missing_local) if missing_local else "none")
            + "; stale entries="
            + (", ".join(stale_entries) if stale_entries else "none")
            + "; malformed hashes="
            + (", ".join(malformed_sha) if malformed_sha else "none")
            + ". Re-run `./build.sh` before submission.",
        ),
        Check(
            "submission source bundle fallback build mode",
            "pass" if not missing_fallback_markers else "error",
            "`build.sh` supports `TAES_EVIDENCE_MODE=bundled` so an extracted source bundle can compile from packaged generated fragments when raw `RUN/` evidence is unavailable."
            if not missing_fallback_markers
            else "`build.sh` is missing bundled-fragment fallback markers: " + "; ".join(missing_fallback_markers),
        ),
    ]


def pdf_checks() -> list[Check]:
    checks: list[Check] = []
    if not MAIN_PDF.exists():
        return [Check("PDF render", "error", "`main.pdf` is missing.")]
    reader = PdfReader(str(MAIN_PDF))
    page_count = len(reader.pages)
    page_budget_ok = page_count < 10
    checks.append(
        Check(
            "PDF page count",
            "pass" if page_budget_ok else "warning",
            f"`main.pdf` has {page_count} TAES-template pages, below the 10-page Regular Paper overlength-charge threshold."
            if page_budget_ok
            else f"`main.pdf` has {page_count} TAES-template pages; Regular Paper overlength charges start at 10 printed pages. Compress or explicitly accept overlength charges before submission.",
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


def pdf_visual_qa_checks() -> list[Check]:
    artifacts = [PDF_VISUAL_QA_JSON, PDF_VISUAL_QA_MANIFEST]
    missing = [path.relative_to(REPO).as_posix() for path in artifacts if not path.exists()]
    if missing:
        return [
            Check(
                "PDF visual QA render",
                "warning",
                "Generated PDF visual QA artifacts are missing: " + ", ".join(missing),
            )
        ]
    payload = json.loads(read_text(PDF_VISUAL_QA_JSON))
    status = str(payload.get("status", "missing"))
    pages = payload.get("pages", [])
    if not isinstance(pages, list):
        pages = []
    labels = {str(page.get("label", "")) for page in pages if isinstance(page, dict)}
    required_labels = {
        "title-abstract",
        "method",
        "main-results",
        "heldout-runtime",
        "discussion-conclusion",
        "references",
    }
    missing_labels = sorted(required_labels - labels)
    bad_pages = [
        str(page.get("label", page.get("page", "unknown")))
        for page in pages
        if isinstance(page, dict) and page.get("status") != "pass"
    ]
    ok = status == "pass" and not missing_labels and not bad_pages
    return [
        Check(
            "PDF visual QA render",
            "pass" if ok else "warning",
            "Representative PDF pages were rendered to `tmp/pdf_visual_qa/` and passed dimension/nonblank checks."
            if ok
            else "PDF visual QA render is incomplete: "
            f"status={status}, missing labels={', '.join(missing_labels) if missing_labels else 'none'}, "
            f"non-pass pages={', '.join(bad_pages) if bad_pages else 'none'}.",
        )
    ]


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
    checks.extend(stress_harsh_checks())
    checks.extend(scenario_family_checks())
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


def stress_harsh_checks() -> list[Check]:
    sources = json.loads(read_text(EVIDENCE_SOURCES)) if EVIDENCE_SOURCES.exists() else {}
    configured = "stress_harsh_n50_report" in sources
    artifacts = [STRESS_HARSH_JSON, STRESS_HARSH_MANIFEST, STRESS_HARSH_FRAGMENT, STRESS_HARSH_SUMMARY]
    existing_artifacts = [path for path in artifacts if path.exists()]

    if not configured and not existing_artifacts:
        return [
            Check(
                "optional harsh-stress evidence path",
                "pass",
                "Harsh packet-loss N50 evidence is not configured in `evidence_sources.json`; the non-blocking parser path is present and no stale stress artifacts are included.",
            )
        ]

    if configured and not STRESS_HARSH_JSON.exists():
        return [
            Check(
                "optional harsh-stress evidence path",
                "warning",
                "`stress_harsh_n50_report` is configured, but `generated/stress_harsh_evidence.json` is missing. Re-run `./build.sh` after the report is available.",
            )
        ]

    if not configured and existing_artifacts:
        return [
            Check(
                "optional harsh-stress evidence path",
                "warning",
                "Stress artifacts exist even though `stress_harsh_n50_report` is not configured; remove stale files or re-run `./build.sh`.",
            )
        ]

    stress = json.loads(read_text(STRESS_HARSH_JSON))
    checks: list[Check] = []
    config = stress.get("config", {})
    if not isinstance(config, dict):
        config = {}

    trials = safe_int(config.get("trials"))
    base_seed = safe_int(config.get("base_seed"), -1)
    trial_seeds = config.get("trial_seeds", [])
    trial_seed_count = len(trial_seeds) if isinstance(trial_seeds, list) else 0
    protocol_ok = trials >= 50 and base_seed == STRESS_BASE_SEED and trial_seed_count >= trials
    profile_ok = float_list_matches(config.get("p_drop_levels"), STRESS_P_DROP_LEVELS) and int_list_matches(
        config.get("p_drop_level_counts"), STRESS_P_DROP_LEVEL_COUNTS
    )
    checks.append(
        Check(
            "harsh-stress N50 protocol",
            "pass" if protocol_ok and profile_ok else "warning",
            f"Harsh-stress run uses base seed {base_seed}, {trials} trials, {trial_seed_count} parsed trial seeds, and packet-loss profile {config.get('p_drop_levels')} / {config.get('p_drop_level_counts')}."
            if protocol_ok and profile_ok
            else "Harsh-stress evidence should use "
            f"base seed {STRESS_BASE_SEED}, at least 50 trials, matching trial seeds, and packet-loss profile "
            f"{STRESS_P_DROP_LEVELS} / {STRESS_P_DROP_LEVEL_COUNTS}; found base seed {base_seed}, "
            f"{trials} trials, {trial_seed_count} trial seeds, and profile "
            f"{config.get('p_drop_levels')} / {config.get('p_drop_level_counts')}.",
        )
    )

    artifacts_ok = all(path.exists() for path in artifacts)
    checks.append(
            Check(
                "harsh-stress generated artifacts",
                "pass" if artifacts_ok else "error",
                "`STRESS_HARSH_MANIFEST.md`, `stress_harsh_evidence.json`, `stress_harsh_section.tex`, and `stress_harsh_summary_sentence.tex` exist."
                if artifacts_ok
                else "Harsh-stress evidence JSON exists, but the generated manifest, response-ready fragment, or manuscript summary sentence is missing.",
            )
        )

    missing_mean = []
    missing_mean.extend(missing_metric_entries(stress, "network", ARM_ORDER, NETWORK_METRICS))
    missing_mean.extend(missing_metric_entries(stress, "local", ARM_ORDER, LOCAL_METRICS))
    checks.append(
        Check(
            "harsh-stress mean metric coverage",
            "pass" if not missing_mean else "error",
            "Harsh-stress means cover all three arms and all network/local manuscript metrics."
            if not missing_mean
            else "Harsh-stress mean metric payload is incomplete: " + "; ".join(missing_mean),
        )
    )

    missing_paired = []
    missing_paired.extend(missing_paired_entries(stress, "paired_network", NETWORK_METRICS))
    missing_paired.extend(missing_paired_entries(stress, "paired_local", LOCAL_METRICS))
    checks.append(
        Check(
            "harsh-stress paired metric coverage",
            "pass" if not missing_paired else "error",
            "Harsh-stress paired reductions include CI, wins, and sign-test p-values for full and reference-only arms."
            if not missing_paired
            else "Harsh-stress paired payload is incomplete: " + "; ".join(missing_paired),
        )
    )

    checks.append(
        Check(
            "optional harsh-stress evidence path",
            "pass" if protocol_ok and profile_ok else "warning",
            "Harsh-stress evidence is configured and parsed; interpretation is recorded as "
            f"`{stress.get('interpretation_class', 'missing')}`. This gate checks protocol and coverage, not that every metric improves.",
        )
    )
    return checks


def scenario_family_checks() -> list[Check]:
    sources = json.loads(read_text(EVIDENCE_SOURCES)) if EVIDENCE_SOURCES.exists() else {}
    configured_keys = [key for key in SCENARIO_FAMILY_KEYS if key in sources]
    artifacts = [SCENARIO_FAMILY_JSON, SCENARIO_FAMILY_MANIFEST, SCENARIO_FAMILY_FRAGMENT, SCENARIO_FAMILY_SUMMARY]
    existing_artifacts = [path for path in artifacts if path.exists()]

    if not configured_keys and not existing_artifacts:
        return [
            Check(
                "optional scenario-family evidence path",
                "pass",
                "No topology/FOV scenario-family evidence is configured in `evidence_sources.json`; the non-blocking parser path is present and no stale scenario artifacts are included.",
            )
        ]

    if configured_keys and not SCENARIO_FAMILY_JSON.exists():
        return [
            Check(
                "optional scenario-family evidence path",
                "warning",
                "Scenario-family evidence keys are configured, but `generated/scenario_family_evidence.json` is missing. Re-run `./build.sh` after the source reports are available.",
            )
        ]

    if not configured_keys and existing_artifacts:
        return [
            Check(
                "optional scenario-family evidence path",
                "warning",
                "Scenario-family artifacts exist even though no scenario-family evidence key is configured; remove stale files or re-run `./build.sh`.",
            )
        ]

    payload = json.loads(read_text(SCENARIO_FAMILY_JSON))
    scenarios = payload.get("scenarios", [])
    if not isinstance(scenarios, list):
        scenarios = []

    checks: list[Check] = []
    artifacts_ok = all(path.exists() for path in artifacts)
    checks.append(
        Check(
            "scenario-family generated artifacts",
            "pass" if artifacts_ok else "error",
            "`SCENARIO_FAMILY_MANIFEST.md`, `scenario_family_evidence.json`, `scenario_family_section.tex`, and `scenario_family_summary_sentence.tex` exist."
            if artifacts_ok
            else "Scenario-family evidence JSON exists, but the generated manifest, response-ready fragment, or summary sentence is missing.",
        )
    )

    parsed_keys = {
        str(item.get("source_key", ""))
        for item in scenarios
        if isinstance(item, dict) and item.get("source_key")
    }
    missing_configured = sorted(set(configured_keys) - parsed_keys)
    count_ok = len(scenarios) >= len(configured_keys) and not missing_configured
    checks.append(
        Check(
            "scenario-family configured source coverage",
            "pass" if count_ok else "error",
            f"Parsed {len(scenarios)} scenario-family reports for configured keys: {', '.join(configured_keys)}."
            if count_ok
            else "Scenario-family payload does not cover every configured source key; missing: "
            + (", ".join(missing_configured) if missing_configured else "count mismatch"),
        )
    )

    missing_mean: list[str] = []
    missing_paired: list[str] = []
    missing_metadata: list[str] = []
    nonmatching_labels: list[str] = []
    source_hash_errors: list[str] = []
    tiers: list[str] = []
    for index, scenario in enumerate(scenarios):
        if not isinstance(scenario, dict):
            missing_metadata.append(f"scenario[{index}]")
            continue
        name = str(scenario.get("name", f"scenario[{index}]"))
        source_key = str(scenario.get("source_key", ""))
        source_report = str(scenario.get("source_report", ""))
        source_sha256 = str(scenario.get("source_sha256", ""))
        configured_source = sources.get(source_key) if source_key else None
        if not source_key or not source_report or not source_sha256:
            source_hash_errors.append(f"{name}/missing_source_fields")
        elif configured_source != source_report:
            source_hash_errors.append(
                f"{name}/source_report_mismatch configured={configured_source} payload={source_report}"
            )
        else:
            source_path = REPO / source_report
            if not source_path.exists():
                source_hash_errors.append(f"{name}/missing_source_report={source_report}")
            elif sha256_file(source_path) != source_sha256:
                source_hash_errors.append(f"{name}/stale_source_sha256={source_report}")
        config = scenario.get("config", {})
        if not isinstance(config, dict):
            missing_metadata.append(f"{name}/config")
            config = {}
        for field in [
            "trials",
            "base_seed",
            "trial_seeds",
            "scenario_label",
            "neighbor_map_mode",
            "sensor_fov_half_angle_deg",
            "p_drop_levels",
            "p_drop_level_counts",
        ]:
            if field not in config:
                missing_metadata.append(f"{name}/config/{field}")
        tier = str(scenario.get("evidence_tier", "missing"))
        tiers.append(tier)
        if scenario.get("scenario_label_matches_expected") is False:
            nonmatching_labels.append(name)
        mean_missing = []
        mean_missing.extend(missing_metric_entries(scenario, "network", ARM_ORDER, NETWORK_METRICS))
        mean_missing.extend(missing_metric_entries(scenario, "local", ARM_ORDER, LOCAL_METRICS))
        missing_mean.extend(f"{name}/{entry}" for entry in mean_missing)
        paired_missing = []
        paired_missing.extend(missing_paired_entries(scenario, "paired_network", NETWORK_METRICS))
        paired_missing.extend(missing_paired_entries(scenario, "paired_local", LOCAL_METRICS))
        missing_paired.extend(f"{name}/{entry}" for entry in paired_missing)

    checks.append(
        Check(
            "scenario-family source hash freshness",
            "pass" if not source_hash_errors else "error",
            "Scenario-family payload source reports and SHA-256 digests match the configured raw evidence reports."
            if not source_hash_errors
            else "Scenario-family generated evidence is stale or mismatched: " + "; ".join(source_hash_errors),
        )
    )

    checks.append(
        Check(
            "scenario-family metadata coverage",
            "pass" if not missing_metadata and not nonmatching_labels else "warning",
            "Scenario-family payload records scenario labels, topology/FOV controls, seeds, packet-loss profile, and expected-label matches."
            if not missing_metadata and not nonmatching_labels
            else "Scenario-family metadata is incomplete or labels do not match the configured scenario contract: missing="
            + (", ".join(missing_metadata) if missing_metadata else "none")
            + "; nonmatching labels="
            + (", ".join(nonmatching_labels) if nonmatching_labels else "none"),
        )
    )

    checks.append(
        Check(
            "scenario-family mean metric coverage",
            "pass" if not missing_mean else "error",
            "Scenario-family means cover all three arms and all network/local manuscript metrics for each configured scenario."
            if not missing_mean
            else "Scenario-family mean metric payload is incomplete: " + "; ".join(missing_mean),
        )
    )

    checks.append(
        Check(
            "scenario-family paired metric coverage",
            "pass" if not missing_paired else "error",
            "Scenario-family paired reductions include CI, wins, and sign-test p-values for full and reference-only arms."
            if not missing_paired
            else "Scenario-family paired payload is incomplete: " + "; ".join(missing_paired),
        )
    )

    has_paper_grade = "paper_grade" in tiers
    has_smoke = any(tier != "paper_grade" for tier in tiers)
    if has_paper_grade and not has_smoke:
        tier_detail = "All configured scenario-family checks are paper-grade N50-or-larger fixed-parameter evidence."
    elif has_paper_grade:
        tier_detail = "At least one configured scenario-family check is paper-grade N50-or-larger evidence; smoke tiers remain explicitly labeled."
    else:
        tier_detail = (
            "Configured scenario-family checks are currently smoke-tier evidence only; "
            "keep them in supplement/response planning until upgraded to N50-or-larger runs."
        )
    checks.append(
        Check(
            "scenario-family evidence tier",
            "pass" if has_paper_grade else "warning",
            tier_detail,
        )
    )

    checks.append(
        Check(
            "optional scenario-family evidence path",
            "pass",
            "Scenario-family evidence is configured and parsed. This gate checks protocol, coverage, and evidence-tier labeling, not that every metric improves.",
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


def is_metadata_placeholder_check(check: Check) -> bool:
    return check.gate == METADATA_PLACEHOLDER_GATE


def summarize_content(checks: list[Check]) -> str:
    non_metadata_checks = [check for check in checks if not is_metadata_placeholder_check(check)]
    non_metadata_statuses = {check.status for check in non_metadata_checks}
    metadata_pending = any(
        is_metadata_placeholder_check(check) and check.status == "pending" for check in checks
    )
    if "error" in non_metadata_statuses:
        return "error"
    if "pending" in non_metadata_statuses:
        return "draft_with_pending_gates"
    if "warning" in non_metadata_statuses:
        return "candidate_with_warnings"
    if metadata_pending:
        return "content_ready_metadata_pending"
    return "submission_candidate"


def write_outputs(checks: list[Check]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    portal_status = summarize(checks)
    content_status = summarize_content(checks)
    blocking_gates = [
        check.as_dict()
        for check in checks
        if check.status in {"error", "pending"} and not is_metadata_placeholder_check(check)
    ]
    metadata_gates = [check.as_dict() for check in checks if is_metadata_placeholder_check(check)]
    payload = {
        "overall_status": portal_status,
        "portal_status": portal_status,
        "content_status": content_status,
        "metadata_policy": (
            "Manuscript front-matter and cover-letter/portal placeholders are ignored only "
            "for content-readiness review; they remain blocking for actual portal submission."
        ),
        "metadata_placeholders_allowed_for_content_readiness": True,
        "blocking_gates_after_metadata_allowance": blocking_gates,
        "metadata_gates": metadata_gates,
        "checks": [check.as_dict() for check in checks],
    }
    READINESS_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    by_status: dict[str, list[Check]] = {"error": [], "pending": [], "warning": [], "pass": []}
    for check in checks:
        by_status.setdefault(check.status, []).append(check)

    lines = [
        "# TAES Submission Readiness Report\n\n",
        "Generated by `docs/paper/taes/manuscript/scripts/check_submission_readiness.py`.\n\n",
        f"- Overall status: `{portal_status}`\n",
        f"- Portal status: `{portal_status}`\n",
        f"- Content status with metadata placeholders allowed: `{content_status}`\n",
        f"- Manuscript PDF: `{MAIN_PDF.relative_to(REPO)}`\n",
        f"- Machine-readable report: `{READINESS_JSON.relative_to(REPO)}`\n",
        "- Metadata policy: manuscript front-matter and cover-letter/portal placeholders are "
        "ignored only for content-readiness review; they remain blocking for actual portal submission.\n\n",
    ]
    if blocking_gates:
        lines.append("## Blocking Gates After Metadata Allowance\n\n")
        for entry in blocking_gates:
            lines.append(f"- `{entry['gate']}` ({entry['status']}): {entry['detail']}\n")
        lines.append("\n")
    else:
        lines.append("## Blocking Gates After Metadata Allowance\n\n")
        lines.append("- None. All non-metadata mechanical, evidence, citation, PDF, and source-bundle gates pass.\n\n")
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
    checks.extend(requirements_doc_checks())
    checks.extend(manuscript_checks(tex, bib))
    checks.extend(doi_resolver_checks(bib))
    checks.extend(cover_letter_checks(tex))
    checks.extend(submission_package_index_checks())
    checks.extend(supplementary_evidence_package_checks())
    checks.extend(reproducibility_ledger_checks())
    checks.extend(pdf_checks())
    checks.extend(pdf_visual_qa_checks())
    checks.extend(evidence_checks())
    checks.extend(bundle_checks())
    write_outputs(checks)


if __name__ == "__main__":
    main()
