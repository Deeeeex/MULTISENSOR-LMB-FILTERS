#!/usr/bin/env python3
"""Create a deterministic TAES manuscript source bundle.

The bundle is meant for final submission hygiene: it contains the compiled PDF,
the LaTeX sources needed to rebuild it, the generated evidence manifests that
explain where the paper-facing numbers came from, and the local scripts needed
to regenerate those fragments from the tracked source reports. Runtime logs,
temporary rendered pages, and experiment outputs are intentionally excluded.
"""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
OUT = ROOT / "generated"
TMP = ROOT / "tmp" / "submission_bundle"
BUNDLE = TMP / "taes_label_barycenter_submission_source.zip"
MANIFEST_JSON = OUT / "submission_bundle_manifest.json"
MANIFEST_MD = OUT / "SUBMISSION_BUNDLE_MANIFEST.md"

ZIP_TIMESTAMP = (2026, 1, 1, 0, 0, 0)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def add_if_exists(paths: list[Path], rel: str) -> None:
    path = ROOT / rel
    if path.exists():
        paths.append(path)


def collect_files() -> list[Path]:
    files: list[Path] = []
    for rel in [
        "build.sh",
        "main.tex",
        "main.pdf",
        "references.bib",
        "IEEEtaes.cls",
        "IEEEtaes.bst",
        "README.md",
        "SUBMISSION_PACKAGE_INDEX.md",
        "FINAL_METADATA_CLOSURE_CHECKLIST.md",
        "SUPPLEMENTARY_EVIDENCE_PACKAGE.md",
        "SUPPLEMENTARY_README_DRAFT.md",
        "REVIEWER_RISK_REGISTER.md",
        "CLAIM_EVIDENCE_BOUNDARY_MAP.md",
        "READINESS_AUDIT_CN.md",
        "COVER_LETTER_AND_METADATA_DRAFT.md",
        "evidence_sources.json",
    ]:
        add_if_exists(files, rel)

    for path in sorted((ROOT / "scripts").glob("*.py")):
        files.append(path)

    for directory, patterns in [
        (ROOT / "figures", ["*.svg"]),
        (OUT, ["*.tex", "*.csv", "*.json", "*.md"]),
    ]:
        for pattern in patterns:
            for path in sorted(directory.glob(pattern)):
                if path.name.startswith("submission_bundle_manifest"):
                    continue
                if path.name == "SUBMISSION_BUNDLE_MANIFEST.md":
                    continue
                if path.name == "pdf_visual_qa.json":
                    continue
                if path.name == "PDF_VISUAL_QA_MANIFEST.md":
                    continue
                if path.name == "submission_readiness.json":
                    continue
                if path.name == "SUBMISSION_READINESS_REPORT.md":
                    continue
                files.append(path)

    unique = sorted(set(files), key=lambda item: item.relative_to(ROOT).as_posix())
    missing = [path for path in unique if not path.exists()]
    if missing:
        raise FileNotFoundError(", ".join(str(path) for path in missing))
    return unique


def write_zip(files: list[Path]) -> None:
    TMP.mkdir(parents=True, exist_ok=True)
    BUNDLE.unlink(missing_ok=True)
    with zipfile.ZipFile(BUNDLE, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            rel = path.relative_to(ROOT).as_posix()
            info = zipfile.ZipInfo(rel, ZIP_TIMESTAMP)
            info.compress_type = zipfile.ZIP_DEFLATED
            mode = 0o755 if rel == "build.sh" else 0o644
            info.external_attr = mode << 16
            archive.writestr(info, path.read_bytes())


def write_manifest(files: list[Path]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    entries = [
        {
            "path": path.relative_to(ROOT).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path in files
    ]
    payload = {
        "bundle": BUNDLE.relative_to(REPO).as_posix(),
        "bundle_sha256": sha256(BUNDLE),
        "file_count": len(entries),
        "files": entries,
        "purpose": "TAES manuscript source bundle with PDF, LaTeX sources, generation scripts, generated fragments, and evidence manifests.",
    }
    MANIFEST_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# TAES Submission Bundle Manifest\n\n",
        f"- Bundle: `{payload['bundle']}`\n",
        f"- Bundle SHA-256: `{payload['bundle_sha256']}`\n",
        f"- Files: {payload['file_count']}\n\n",
        "## Included Files\n\n",
        "| Path | Size | SHA-256 |\n",
        "| --- | ---: | --- |\n",
    ]
    for entry in entries:
        lines.append(f"| `{entry['path']}` | {entry['size_bytes']} | `{entry['sha256']}` |\n")
    MANIFEST_MD.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    files = collect_files()
    write_zip(files)
    write_manifest(files)


if __name__ == "__main__":
    main()
