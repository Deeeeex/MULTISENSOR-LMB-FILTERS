#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

can_regenerate_evidence() {
  python3 - <<'PY'
import json
import sys
from pathlib import Path

root = Path.cwd()
repo = root.parents[3] if len(root.parents) > 3 else root
manifest = root / "evidence_sources.json"
if not manifest.exists():
    print(f"missing {manifest}", file=sys.stderr)
    raise SystemExit(1)
sources = json.loads(manifest.read_text(encoding="utf-8"))
missing = []
for key, rel in sources.items():
    path = repo / rel
    if not path.exists():
        missing.append(f"{key}: {path}")
if missing:
    print("raw evidence sources are not available:", file=sys.stderr)
    for item in missing:
        print(f"  - {item}", file=sys.stderr)
    raise SystemExit(1)
PY
}

evidence_mode="${TAES_EVIDENCE_MODE:-auto}"
if [[ "$evidence_mode" == "refresh" ]]; then
  can_regenerate_evidence
  regenerate_evidence=1
elif [[ "$evidence_mode" == "bundled" ]]; then
  regenerate_evidence=0
elif can_regenerate_evidence; then
  regenerate_evidence=1
else
  regenerate_evidence=0
fi

if [[ "$regenerate_evidence" == "1" ]]; then
  python3 scripts/extract_n50_evidence.py
  python3 scripts/extract_reference_baselines.py
  python3 scripts/extract_heldout_sanity_evidence.py
  python3 scripts/extract_stress_evidence.py
  python3 scripts/verify_n50_evidence.py
  python3 scripts/render_figures.py
  python3 scripts/render_reproducibility_ledger.py
else
  echo "Raw evidence sources unavailable; compiling from bundled generated fragments."
fi

if command -v tectonic >/dev/null 2>&1; then
  tectonic -X compile main.tex
elif command -v latexmk >/dev/null 2>&1; then
  latexmk -pdf -interaction=nonstopmode main.tex
elif command -v pdflatex >/dev/null 2>&1; then
  pdflatex -interaction=nonstopmode main.tex
  bibtex main
  pdflatex -interaction=nonstopmode main.tex
  pdflatex -interaction=nonstopmode main.tex
else
  echo "No LaTeX engine found. Install tectonic, latexmk, or pdflatex." >&2
  exit 1
fi

if [[ "$regenerate_evidence" == "1" ]]; then
  python3 scripts/create_submission_bundle.py
  python3 scripts/check_submission_readiness.py
else
  echo "Skipping source-bundle and readiness regeneration in bundled-fragment mode."
fi
