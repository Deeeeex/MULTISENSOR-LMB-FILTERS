# Integrated mechanism overview

The user selected `generated_reference.png` on 2026-09-08 as the visual
reference for the paper's main figure. `imagegen_prompt.md` retains its
design prompt. The reference depicts qualitative robot poses and densities.

`../make_main_figure_integrated.py` retains that continuous composition,
robot scene, source paths, qualification junction, two fusion branches,
combined posterior, and recursive feedback. The scientific content has
been updated to guarded current-evidence fusion. It is not a trace of the
old recency equations in the reference image.

The final 181 × 85.15 mm SVG has editable text and vector geometry, with no
embedded raster images or panel cards. Arial Narrow is used for compact
labels and STIX for mathematics. The following paths hold its outputs:

- `../figures/overview.svg`, `.pdf`, `.png`: master, manuscript image, preview.
- `figure.tex`: inclusion and caption.
- `vector_qa.json`: text bounds, collision checks, and live SVG text count.
- `../source_data/overview_schematic.json`: reference identity and method mapping.

The diagram shows the conservative inherited existence base, current
existence and Gaussian increments, association/score or miss admission,
and the source curvature test. The same admitted weight enters both
spatial and existence corrections. The integral is recomputed from the
corrected spatial product and contributes to existence normalization.
Qualified absence supplies only the existence base and disables current
ratio admission. The combined posterior returns to the next prediction.

The four robots illustrate the mechanism; the empirical experiment has
two vehicles. No performance measurements are represented by the density
glyphs. Regenerate from the parent directory with:

```sh
python3 make_main_figure_integrated.py
```

The standard `build.py --regenerate` command includes this figure.
