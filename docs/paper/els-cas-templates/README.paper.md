# Paper Manuscript Notes

Local manuscript entry points for this repository:

- main source: `manuscript.tex`
- bibliography: `paper-refs.bib`
- section files: `sections/*.tex`
- migrated paper figures: `figs/paper-figure1.png` through `figs/paper-figure6.pdf`

Current template choice:

- Elsevier `cas-sc` single-column class

Notes:

- The original Elsevier sample files are kept unchanged in this directory.
- The manuscript uses the migrated local figure assets inside `figs/` rather than the original draft paths under `docs/paper/figures/`.
- Figure 1 and Figure 2 are stored both as `.svg` source and `.png` fallback assets for direct `\includegraphics` use.

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
