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


def check_pdf(pdf_path: Path) -> list[str]:
    reader = PdfReader(pdf_path)
    assert len(reader.pages) == 5, f"expected exactly 5 pages, found {len(reader.pages)}"

    pages = [normalize(page.extract_text() or "") for page in reader.pages]
    body = " ".join(pages[:4])
    references = pages[4]
    full_text = " ".join(pages)

    assert "FUSION-SUFFICIENT MOMENT EXCHANGE" in pages[0]
    for heading in (
        "1. INTRODUCTION",
        "2. RELATED WORK",
        "3. FUSION-SUFFICIENT MOMENT EXCHANGE",
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
        "3. FUSION-SUFFICIENT MOMENT EXCHANGE",
        "4. EXPERIMENTS",
        "5. DISCUSSION",
        "6. CONCLUSION",
    ):
        assert heading not in references, f"body section leaked onto page 5: {heading}"

    for page_number, text in enumerate(pages[:4], start=1):
        assert len(text) >= 500, f"body page {page_number} is unexpectedly sparse"

    assert "Fig. 1" in pages[1], "concept figure is not on body page 2"
    assert "Fig. 2" in pages[2], "confirmatory figure is not near Experiments"
    assert "Table 1." in pages[2], "results table is not inside the Experiments page"
    assert "4. EXPERIMENTS" in pages[2]
    assert "5. DISCUSSION" in pages[3]
    assert "6. CONCLUSION" in pages[3]

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
    ):
        assert prohibited not in lowered, f"stale story phrase remains: {prohibited}"
    assert "??" not in full_text, "unresolved LaTeX reference remains"
    assert "58.28%" in full_text
    assert "1,119,037" in full_text
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
