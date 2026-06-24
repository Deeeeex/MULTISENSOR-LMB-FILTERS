#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

python3 scripts/extract_n50_evidence.py
python3 scripts/extract_reference_baselines.py
python3 scripts/extract_heldout_sanity_evidence.py
python3 scripts/extract_stress_evidence.py
python3 scripts/verify_n50_evidence.py
python3 scripts/render_figures.py

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

python3 scripts/create_submission_bundle.py
python3 scripts/check_submission_readiness.py
