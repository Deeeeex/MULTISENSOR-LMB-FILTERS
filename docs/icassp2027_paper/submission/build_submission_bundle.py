#!/usr/bin/env python3
"""Build and optionally verify the minimal ICASSP 2027 submission bundle."""

from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


PAPER_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PAPER_DIR.parents[1]
UPLOAD_DIR = PAPER_DIR / "submission" / "upload_files"
ARCHIVE_PATH = UPLOAD_DIR / "manuscript_source.zip"
UPLOAD_PDF = UPLOAD_DIR / "manuscript.pdf"
CHECKSUM_PATH = UPLOAD_DIR / "SHA256SUMS"

SOURCE_FILES = (
    "main.tex",
    "refs.bib",
    "IEEEbib.bst",
    "spconf.sty",
    "sections/01_introduction.tex",
    "sections/02_related_work.tex",
    "sections/03_method.tex",
    "sections/04_experiments.tex",
    "sections/05_discussion.tex",
    "sections/05_conclusion.tex",
    "figures/payload_graph_schematic.pdf",
    "figures/heldout_tradeoff.pdf",
)
GENERATED_MEMBERS = ("BUILD.md", "SOURCE_MANIFEST.sha256")
FIXED_ZIP_TIME = (1980, 1, 1, 0, 0, 0)

BUILD_INSTRUCTIONS = """# ICASSP 2027 manuscript source

Compile from this directory with:

    tectonic --keep-logs --keep-intermediates main.tex

The archive intentionally contains only the manuscript source, bibliography,
conference style, and the two PDF figures used by `main.tex`. Generated TeX
intermediates, PNG previews, experiment artifacts, and internal review notes
are excluded.
"""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def archive_member(name: str, data: bytes) -> tuple[zipfile.ZipInfo, bytes]:
    info = zipfile.ZipInfo(name, FIXED_ZIP_TIME)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    info.create_system = 3
    return info, data


def source_payloads() -> dict[str, bytes]:
    payloads: dict[str, bytes] = {}
    for relative in SOURCE_FILES:
        path = PAPER_DIR / relative
        if not path.is_file():
            raise FileNotFoundError(f"required submission source is missing: {path}")
        payloads[relative] = path.read_bytes()
    return payloads


def build_bundle() -> None:
    payloads = source_payloads()
    manifest = "".join(
        f"{sha256_bytes(payloads[name])}  {name}\n" for name in sorted(payloads)
    ).encode("utf-8")

    archive_payloads = {
        **payloads,
        "BUILD.md": BUILD_INSTRUCTIONS.encode("utf-8"),
        "SOURCE_MANIFEST.sha256": manifest,
    }

    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    temporary_archive = ARCHIVE_PATH.with_suffix(".zip.tmp")
    with zipfile.ZipFile(
        temporary_archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9
    ) as archive:
        for name in sorted(archive_payloads):
            info, data = archive_member(name, archive_payloads[name])
            archive.writestr(info, data, compresslevel=9)
    temporary_archive.replace(ARCHIVE_PATH)

    canonical_pdf = PAPER_DIR / "main.pdf"
    if not canonical_pdf.is_file():
        raise FileNotFoundError(f"canonical manuscript PDF is missing: {canonical_pdf}")
    shutil.copyfile(canonical_pdf, UPLOAD_PDF)

    checksums = (
        f"{sha256_file(UPLOAD_PDF)}  {UPLOAD_PDF.name}\n"
        f"{sha256_file(ARCHIVE_PATH)}  {ARCHIVE_PATH.name}\n"
    )
    CHECKSUM_PATH.write_text(checksums, encoding="utf-8")


def parse_source_manifest(text: str) -> dict[str, str]:
    entries: dict[str, str] = {}
    for line in text.splitlines():
        digest, name = line.split("  ", maxsplit=1)
        entries[name] = digest
    return entries


def verify_bundle() -> None:
    expected_members = set(SOURCE_FILES) | set(GENERATED_MEMBERS)
    with tempfile.TemporaryDirectory(prefix="icassp2027-source-") as temporary:
        extract_dir = Path(temporary)
        with zipfile.ZipFile(ARCHIVE_PATH) as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise AssertionError("submission archive contains duplicate members")
            if set(names) != expected_members:
                missing = sorted(expected_members - set(names))
                unexpected = sorted(set(names) - expected_members)
                raise AssertionError(
                    f"submission archive closure mismatch; missing={missing}, "
                    f"unexpected={unexpected}"
                )
            for name in names:
                member_path = Path(name)
                if member_path.is_absolute() or ".." in member_path.parts:
                    raise AssertionError(f"unsafe archive member: {name}")
            archive.extractall(extract_dir)

        manifest = parse_source_manifest(
            (extract_dir / "SOURCE_MANIFEST.sha256").read_text(encoding="utf-8")
        )
        if set(manifest) != set(SOURCE_FILES):
            raise AssertionError("source manifest does not match the declared source closure")
        for relative, expected_digest in manifest.items():
            actual_digest = sha256_file(extract_dir / relative)
            if actual_digest != expected_digest:
                raise AssertionError(f"source hash mismatch: {relative}")

        tectonic = shutil.which("tectonic")
        if tectonic is None:
            raise RuntimeError("tectonic is required for isolated bundle verification")
        subprocess.run(
            [tectonic, "--keep-logs", "--keep-intermediates", "main.tex"],
            cwd=extract_dir,
            check=True,
        )

        checker = REPO_ROOT / "tests" / "check_icassp2027_pdf.py"
        subprocess.run(
            [
                sys.executable,
                str(checker),
                "--pdf",
                str(extract_dir / "main.pdf"),
                "--render-dir",
                str(extract_dir / "render"),
                "--require-submission-declarations",
            ],
            cwd=REPO_ROOT,
            check=True,
        )

    listed_checksums = CHECKSUM_PATH.read_text(encoding="utf-8").splitlines()
    expected_checksums = {
        f"{sha256_file(UPLOAD_PDF)}  {UPLOAD_PDF.name}",
        f"{sha256_file(ARCHIVE_PATH)}  {ARCHIVE_PATH.name}",
    }
    if set(listed_checksums) != expected_checksums:
        raise AssertionError("upload checksum file is stale")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--verify",
        action="store_true",
        help="extract, compile, and run the strict PDF gate after packaging",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    build_bundle()
    if args.verify:
        verify_bundle()
    print(f"submission PDF: {UPLOAD_PDF}")
    print(f"source archive: {ARCHIVE_PATH}")
    print(f"checksums: {CHECKSUM_PATH}")


if __name__ == "__main__":
    main()
