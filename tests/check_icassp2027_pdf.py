#!/usr/bin/env python3
"""Check and render the ICASSP manuscript PDF for final visual QA."""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

import pypdfium2 as pdfium
from pypdf import PdfReader


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PDF = ROOT / "docs" / "icassp2027_paper" / "main.pdf"
DEFAULT_RENDER_DIR = ROOT / "tmp" / "icassp2027_render"


def normalize(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


def collect_base_fonts(reader: PdfReader) -> set[str]:
    fonts: set[str] = set()

    def visit_resources(resources: object) -> None:
        if resources is None:
            return
        resources = resources.get_object()
        font_resources = resources.get("/Font")
        if font_resources:
            for font in font_resources.get_object().values():
                fonts.add(str(font.get_object().get("/BaseFont")))
        xobjects = resources.get("/XObject")
        if xobjects:
            for xobject in xobjects.get_object().values():
                visit_resources(xobject.get_object().get("/Resources"))

    for page in reader.pages:
        visit_resources(page.get("/Resources"))
    return fonts


def assert_link_borders_hidden(reader: PdfReader) -> None:
    link_count = 0
    for page in reader.pages:
        annotations = page.get("/Annots")
        if not annotations:
            continue
        for annotation_ref in annotations:
            annotation = annotation_ref.get_object()
            if str(annotation.get("/Subtype")) != "/Link":
                continue
            link_count += 1
            border_style = annotation.get("/BS")
            if border_style is not None:
                width = float(border_style.get_object().get("/W", 1))
                assert width == 0.0, "visible PDF link border detected"
                continue
            border = annotation.get("/Border")
            assert border is not None and len(border) >= 3, (
                "PDF link relies on the visible default border"
            )
            assert float(border[2]) == 0.0, "visible PDF link border detected"
    assert link_count > 0, "expected hyperref link annotations"


def check_pdf(pdf_path: Path) -> list[str]:
    reader = PdfReader(pdf_path)
    assert len(reader.pages) == 5, f"expected exactly 5 pages, found {len(reader.pages)}"
    assert_link_borders_hidden(reader)

    pages = [normalize(page.extract_text() or "") for page in reader.pages]
    body = " ".join(pages[:4])
    references = pages[4]
    full_text = " ".join(pages)

    assert "RECEIVER-INDUCED MOMENT EXCHANGE FOR DISTRIBUTED LMB FUSION" in pages[0]
    assert "Anonymous ICASSP Submission" not in full_text
    author_positions = [
        pages[0].index("Jinhao Chen"),
        pages[0].index("Hao Lang"),
        pages[0].index("Tianyu Wo"),
    ]
    assert author_positions == sorted(author_positions), "author order is incorrect"
    affiliation_positions = [
        pages[0].index("Beihang University"),
        pages[0].index("Shanghai Jiao Tong University"),
        pages[0].index("Chinese Aeronautical Radio Electronics Research Institute"),
    ]
    assert affiliation_positions == sorted(affiliation_positions), (
        "affiliations are not ordered from the first author's institution"
    )
    for heading in (
        "1. INTRODUCTION",
        "2. RELATED WORK",
        "3. RECEIVER-INDUCED MOMENT EXCHANGE",
        "4. EXPERIMENTS",
        "5. DISCUSSION",
        "6. CONCLUSION",
    ):
        assert heading in body, f"body heading missing: {heading}"

    assert "7. REFERENCES" not in body, "references leaked into pages 1--4"
    assert "7. REFERENCES" in references, "page 5 is not the references page"
    for heading in (
        "1. INTRODUCTION",
        "2. RELATED WORK",
        "3. RECEIVER-INDUCED MOMENT EXCHANGE",
        "4. EXPERIMENTS",
        "5. DISCUSSION",
        "6. CONCLUSION",
    ):
        assert heading not in references, f"body section leaked onto page 5: {heading}"

    for page_number, text in enumerate(pages[:4], start=1):
        assert len(text) >= 500, f"body page {page_number} is unexpectedly sparse"

    assert "Fig. 1" in pages[1], "concept figure is not on body page 2"
    assert "Fig. 2" in pages[2], "confirmatory figure is not near Experiments"
    assert "4. EXPERIMENTS" in pages[2]
    assert "Confirmatory means" in pages[3], "results table is not on body page 4"
    assert "5. DISCUSSION" in pages[3]
    assert "6. CONCLUSION" in pages[3]
    assert "COMPLIANCE WITH ETHICAL STANDARDS" in pages[3]
    assert "ACKNOWLEDGMENT" in pages[3].upper()

    fonts = collect_base_fonts(reader)
    for required in (
        "TeXGyreTermes-Regular",
        "TeXGyreTermes-Bold",
        "TeXGyreTermes-Italic",
    ):
        assert any(required in font for font in fonts), f"required font missing: {required}"
    assert not any("LMRoman" in font for font in fonts), "Latin Modern fallback detected"

    lowered = full_text.lower()
    for prohibited in (
        "light posterior",
        "held-out",
        "dynamic topology",
        "effective graph",
        "full-posterior-equivalent",
        "metropolis weights",
        "projected kla-lmb receiver",
    ):
        assert prohibited not in lowered, f"stale story phrase remains: {prohibited}"
    assert "??" not in full_text, "unresolved LaTeX reference remains"
    assert "58.28%" in full_text
    assert "1,119,037" in full_text
    assert "retained post-step" in lowered
    assert "admissible" in lowered
    assert "fixed symmetric degree-based" in lowered
    return pages


def render_pdf(pdf_path: Path, output_dir: Path) -> None:
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    document = pdfium.PdfDocument(pdf_path)
    assert len(document) == 5
    for page_index in range(len(document)):
        page = document[page_index]
        image = page.render(scale=2.0).to_pil()
        image.save(output_dir / f"page-{page_index + 1:02d}.png")
        page.close()
    document.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pdf", type=Path, default=DEFAULT_PDF)
    parser.add_argument("--render-dir", type=Path, default=DEFAULT_RENDER_DIR)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pdf_path = args.pdf.resolve()
    render_dir = args.render_dir.resolve()
    pages = check_pdf(pdf_path)
    render_pdf(pdf_path, render_dir)
    print(f"validated {pdf_path}: {len(pages)} pages")
    print(f"rendered visual QA pages to {render_dir}")


if __name__ == "__main__":
    main()
