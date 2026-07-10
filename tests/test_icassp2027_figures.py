from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

from pypdf import PdfReader


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "docs" / "icassp2027_paper" / "scripts" / "generate_figures.py"
EVIDENCE = (
    ROOT
    / "RUN"
    / "GA"
    / "GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv"
)
OUTPUT_NAMES = (
    "payload_graph_schematic.pdf",
    "payload_graph_schematic.png",
    "heldout_tradeoff.pdf",
    "heldout_tradeoff.png",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(output_dir: Path) -> dict:
    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--evidence",
            str(EVIDENCE),
            "--output-dir",
            str(output_dir),
        ],
        cwd=ROOT,
        check=True,
        env={"SOURCE_DATE_EPOCH": "0"},
    )
    return json.loads((output_dir / "figure_manifest.json").read_text())


def extract_pdf_text(path: Path) -> str:
    return "\n".join(page.extract_text() or "" for page in PdfReader(path).pages)


def test_figures_are_evidence_driven_and_deterministic(tmp_path: Path) -> None:
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    first_manifest = generate(first_dir)
    second_manifest = generate(second_dir)

    assert first_manifest == second_manifest
    assert first_manifest["schema"] == "icassp2027-figure-manifest-v1"
    assert first_manifest["evidence"]["path"] == str(EVIDENCE.relative_to(ROOT))
    assert first_manifest["evidence"]["sha256"] == sha256(EVIDENCE)
    assert first_manifest["evidence"]["row_count"] == 50
    assert first_manifest["evidence"]["seed_interval"] == [82, 131]
    assert first_manifest["generator_sha256"] == sha256(SCRIPT)
    assert first_manifest["summary"]["all_exact_match"] is True
    assert first_manifest["summary"]["all_masks_equal"] is True
    assert first_manifest["summary"]["max_state_residual"] == 0.0
    assert first_manifest["layout_qa"]["minimum_source_font_points"] >= 7.6
    assert first_manifest["layout_qa"]["minimum_estimated_final_font_points"] >= 7.0

    for name in OUTPUT_NAMES:
        first = first_dir / name
        second = second_dir / name
        assert first.stat().st_size > 10_000
        assert sha256(first) == sha256(second)
        assert first_manifest["outputs"][name]["sha256"] == sha256(first)

    pdf_text = "\n".join(
        extract_pdf_text(first_dir / name)
        for name in OUTPUT_NAMES
        if name.endswith(".pdf")
    )
    normalized_pdf_text = " ".join(pdf_text.split())
    for prohibited in (
        "dynamic topology",
        "graph sparsification",
        "full-dynamic",
        "light-dynamic",
        "held-out",
        "lambda",
    ):
        assert prohibited not in pdf_text.lower()
    assert "Receiver-side projection" in normalized_pdf_text
    assert "Sender-side projection" in normalized_pdf_text
    assert "Fusion-sufficient moment" in normalized_pdf_text
    assert "58.28%" in normalized_pdf_text
