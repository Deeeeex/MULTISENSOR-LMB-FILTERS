# Paper Manuscript Notes

Local manuscript entry points for this repository:

- main source: `manuscript.tex`
- bibliography: `paper-refs.bib`
- section files: `sections/*.tex`
- validated table-row fragments: `sections/generated/*.tex`
- working derivation notes: `notes/*.tex`
- adopted manuscript figures: `figs/Figure_1.pdf`, `figs/paper-figure2.pdf`, `figs/paper-figure3.pdf`, `figs/paper-figure-scenario.pdf`, `figs/paper-figure4.pdf`, and `figs/paper-figure7.pdf`
- submission support files: `submission/highlights.txt`, `submission/cover_letter_information_fusion.md`, `submission/pre_submission_checklist.md`, declaration drafts under `submission/declarations/`, and the upload-ready split files under `submission/upload_files/`

Current template choice:

- Elsevier `cas-sc` single-column class

Notes:

- The original Elsevier sample files are kept unchanged in this directory.
- The manuscript uses the adopted local figure assets inside `figs/` rather than the original draft paths under `docs/paper/figures/`.
- Legacy or diagnostic graphics retained in `figs/` but not used by the manuscript are prefixed with `unused-`.
- Appendix C now contains the theory-side interpretation distilled from the working derivation note, while the full evolving derivation remains in `notes/working_adaptive_weight_derivation.tex`.
- Submission declarations are included from `sections/declarations.tex`; author, funding, competing-interest, CRediT, and data/code availability placeholders must be finalized before submission.
- Highlights are kept as a separate upload file rather than embedded in `manuscript.tex`.
- Competing-interest disclosure is kept as a separate submission declaration draft; the manuscript declaration section keeps CRediT, funding, data availability, and generative AI disclosure before the reference list.

Local compilation:

- compiler: `tectonic`
- main command:

```bash
./build.sh
```

- direct command:

```bash
tectonic --keep-logs --keep-intermediates manuscript.tex
```

Generated output:

- `manuscript.pdf`
- `manuscript.log`
- `manuscript.aux`
- `manuscript.bbl`
