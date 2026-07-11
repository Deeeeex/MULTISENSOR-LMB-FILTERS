from __future__ import annotations

import hashlib
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PAPER_DIR = ROOT / "docs" / "icassp2027_paper"
UPLOAD_DIR = PAPER_DIR / "submission" / "upload_files"

SOURCE_FILES = {
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
}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def test_submission_bundle_is_minimal_and_hash_bound() -> None:
    archive_path = UPLOAD_DIR / "manuscript_source.zip"
    upload_pdf = UPLOAD_DIR / "manuscript.pdf"
    checksums_path = UPLOAD_DIR / "SHA256SUMS"

    assert archive_path.is_file()
    assert upload_pdf.read_bytes() == (PAPER_DIR / "main.pdf").read_bytes()

    with zipfile.ZipFile(archive_path) as archive:
        names = archive.namelist()
        assert len(names) == len(set(names))
        assert set(names) == SOURCE_FILES | {"BUILD.md", "SOURCE_MANIFEST.sha256"}
        assert all(not Path(name).is_absolute() and ".." not in Path(name).parts for name in names)

        manifest = {}
        for line in archive.read("SOURCE_MANIFEST.sha256").decode("utf-8").splitlines():
            digest, name = line.split("  ", maxsplit=1)
            manifest[name] = digest
        assert set(manifest) == SOURCE_FILES
        for name, digest in manifest.items():
            archived = archive.read(name)
            assert sha256(archived) == digest
            assert archived == (PAPER_DIR / name).read_bytes()

    listed = set(checksums_path.read_text(encoding="utf-8").splitlines())
    expected = {
        f"{sha256(upload_pdf.read_bytes())}  manuscript.pdf",
        f"{sha256(archive_path.read_bytes())}  manuscript_source.zip",
    }
    assert listed == expected
