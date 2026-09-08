# Integrated mechanism overview

The user selected `generated_reference.png` on 2026-09-08 as the visual
reference for the paper's main figure. `imagegen_prompt.md` preserves the
prompt used with OpenAI's built-in image-generation tool. The reference
contains illustrative robot poses and density glyphs, not experimental data.

`../make_main_figure_integrated.py` reconstructs the selected image with
editable text, paths, facets, curves, and geometric shapes. It retains the
continuous layout, robot scene, orange/teal source evidence, two fusion
branches, combined posterior, and bottom feedback loop. The final canvas is
181 × 85.15 mm. There are no a/b/c panel divisions or embedded raster images.

Outputs:

- `../figures/overview.svg`: editable vector master.
- `../figures/overview.pdf`: vector file used in the paper.
- `../figures/overview.png`: 300 dpi preview.
- `figure.tex`: Fig. 1 inclusion and caption.
- `vector_qa.json`: text-bound and overlap checks, editable-text count.
- `../source_data/overview_schematic.json`: source identity and corrections.

Scientific corrections to the generated reference:

1. Include `+ log eta_a` in the existence equation.
2. Connect spatial overlap to the product operation and existence update.
3. Send a qualified visible-absence censor to existence weighting only,
   with age factor equal to one.
4. Return the combined Bernoulli posterior to the next local prediction.
5. Send detection/miss information to the measurement update.
6. Label the age curve by its factor `f`, with a floor of 0.25.

Small layout adjustments separate labels, widen the qualification junction
and posterior box, and keep both fusion equations readable at final size.
The SVG is a vector reconstruction, rather than a pixel-identical raster trace.
It can be regenerated with the same Python/matplotlib environment as the other
paper figures; Arial Narrow is used for compact labels and STIX for mathematics.

```sh
python3 make_main_figure_integrated.py
```

Run that command from the parent paper directory. The standard
`build.py --regenerate` command also recreates this figure.
