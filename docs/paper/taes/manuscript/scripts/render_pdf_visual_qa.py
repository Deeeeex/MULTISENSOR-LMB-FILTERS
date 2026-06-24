#!/usr/bin/env python3
"""Render representative TAES PDF pages and record a visual QA manifest."""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
OUT = ROOT / "generated"
TMP = ROOT / "tmp" / "pdf_visual_qa"
MAIN_PDF = ROOT / "main.pdf"
QA_JSON = OUT / "pdf_visual_qa.json"
QA_MD = OUT / "PDF_VISUAL_QA_MANIFEST.md"
DENSITY_DPI = 180

PAGE_PLAN = [
    ("title-abstract", 1),
    ("method", 3),
    ("main-results", 6),
    ("heldout-conclusion", 7),
    ("references", "last"),
]


def rel(path: Path) -> str:
    return path.relative_to(REPO).as_posix()


def magick_version(magick: str) -> str:
    try:
        output = subprocess.check_output([magick, "-version"], text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        return exc.output.strip().splitlines()[0] if exc.output else "unknown"
    return output.strip().splitlines()[0] if output.strip() else "unknown"


def planned_pages(page_count: int) -> list[tuple[str, int]]:
    pages: list[tuple[str, int]] = []
    seen: set[int] = set()
    for label, raw_page in PAGE_PLAN:
        page = page_count if raw_page == "last" else int(raw_page)
        if page < 1 or page > page_count or page in seen:
            continue
        seen.add(page)
        pages.append((label, page))
    return pages


def identify_image(magick: str, path: Path) -> dict[str, float | int]:
    output = subprocess.check_output(
        [magick, "identify", "-format", "%w %h %[fx:mean] %[fx:standard_deviation]", str(path)],
        text=True,
    )
    width, height, mean, stddev = output.strip().split()
    return {
        "width": int(width),
        "height": int(height),
        "mean_luma": float(mean),
        "stddev_luma": float(stddev),
        "size_bytes": path.stat().st_size,
    }


def render_page(magick: str, label: str, page: int) -> dict[str, object]:
    output = TMP / f"main_p{page}_{label}.png"
    subprocess.check_call(
        [
            magick,
            "-density",
            str(DENSITY_DPI),
            f"{MAIN_PDF}[{page - 1}]",
            "-background",
            "white",
            "-alpha",
            "remove",
            str(output),
        ]
    )
    stats = identify_image(magick, output)
    checks = {
        "dimension_ok": stats["width"] >= 1000 and stats["height"] >= 1000,
        "nonblank_ok": stats["stddev_luma"] >= 0.01 and stats["size_bytes"] >= 10_000,
    }
    status = "pass" if all(checks.values()) else "warning"
    return {
        "label": label,
        "page": page,
        "image": rel(output),
        "status": status,
        "checks": checks,
        **stats,
    }


def write_outputs(payload: dict[str, object]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    QA_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# PDF Visual QA Manifest\n\n",
        f"- PDF: `{payload.get('pdf')}`\n",
        f"- Page count: {payload.get('page_count')}\n",
        f"- Renderer: `{payload.get('renderer')}`\n",
        f"- Density: {payload.get('density_dpi')} dpi\n",
        f"- Status: `{payload.get('status')}`\n\n",
    ]
    if payload.get("status") == "skipped_missing_renderer":
        lines.append("ImageMagick `magick` was not found; visual page rendering was skipped.\n")
    else:
        lines.extend(
            [
                "## Rendered Pages\n\n",
                "| Label | Page | Image | Dimensions | Mean luma | Stddev luma | Status |\n",
                "| --- | ---: | --- | ---: | ---: | ---: | --- |\n",
            ]
        )
        for page in payload.get("pages", []):
            if not isinstance(page, dict):
                continue
            lines.append(
                f"| {page['label']} | {page['page']} | `{page['image']}` | "
                f"{page['width']}x{page['height']} | {page['mean_luma']:.4f} | "
                f"{page['stddev_luma']:.4f} | `{page['status']}` |\n"
            )
    QA_MD.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    if not MAIN_PDF.exists():
        raise FileNotFoundError(MAIN_PDF)

    reader = PdfReader(str(MAIN_PDF))
    page_count = len(reader.pages)
    magick = shutil.which("magick")
    base_payload: dict[str, object] = {
        "pdf": rel(MAIN_PDF),
        "page_count": page_count,
        "density_dpi": DENSITY_DPI,
    }
    if not magick:
        write_outputs(
            {
                **base_payload,
                "renderer": "missing",
                "status": "skipped_missing_renderer",
                "pages": [],
            }
        )
        return

    TMP.mkdir(parents=True, exist_ok=True)
    pages = [render_page(magick, label, page) for label, page in planned_pages(page_count)]
    status = "pass" if pages and all(page["status"] == "pass" for page in pages) else "warning"
    write_outputs(
        {
            **base_payload,
            "renderer": magick_version(magick),
            "status": status,
            "pages": pages,
        }
    )


if __name__ == "__main__":
    main()
