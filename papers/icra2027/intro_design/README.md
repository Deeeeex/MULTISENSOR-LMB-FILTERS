# Illustrated Intro figure

The figure explains one claim: in an idealized common-prior, equal-weight
case, admitted local update ratios restore bounded fractions of the current
likelihood while retaining one inherited prior. The final normalizer couples
spatial density and target existence. This is a conceptual illustration;
the cars, road, sensing wedges, and history trail are not recorded experiment
images, estimated confidence regions, or quantitative results.

## ICRA visual references

The first pages of these primary papers were inspected on 2026-09-09:

- [DMSTrack, ICRA 2024](https://arxiv.org/pdf/2309.14655), Fig. 1:
  a compact side-by-side comparison makes the changed inference component
  explicit. [Author venue record](https://research.nvidia.com/labs/twn/publication/icra_2024_dmstrack/).
- [CoopDETR, ICRA 2025](https://arxiv.org/pdf/2502.19313), Fig. 1:
  a traffic scene leads into observations, objects, and an abstract relation
  graph. The scene supplies concrete task context before technical detail.

These are design observations from two relevant papers, not a claim that
ICRA has one prescribed visual style. No source artwork is reproduced here.
The resulting composition combines a larger cooperative-driving scene with
a subordinate comparison of posterior pooling and GCE.

## Generation and refinement

`concept_v1.png` was generated with the built-in image tool. The second
version refined flat fills and added the GCE-to-existence output arrow.
The selected `concept_v3.png` enlarged the small labels for a single-column
figure. The complete prompts are in `prompts.md` and `prompt_v3.md`.

The selected master uses a 1385 by 1136 canvas, corresponding to the paper's
89 by 73 mm figure. Its three current vehicles, three faded history poses,
street geometry, source-specific sensing wedges and rays, two method boxes,
admission arrow, shield, and coupled outputs retain their source coordinates
in the vector reconstruction. All 14 annotations and mathematical expressions
are typeset as live Arial Narrow/STIX text for legibility and editing.
Font rasterization and small vector-fitting differences mean the SVG is
not pixel-identical to the generated bitmap.

## Editable reconstruction

`trace_master.py` makes the frozen geometric source `vector_scene.json`.
The original generated images remain unchanged. Only annotation regions
in a temporary tracing input are cleared with their surrounding background;
the two labels close to cars use masks bounded by the vehicle silhouettes
to preserve the roofs.
The input receives 0.35-pixel antialias smoothing and 2x Lanczos upsampling.
The fitted vector paths are transformed back to the original coordinates.
All path geometry is retained in `outline_reference.svg` and the JSON source.
The trace has no embedded raster image. Optional authoring dependencies are
Pillow, NumPy, vtracer 0.6.15, and svgpathtools 1.8.0.

Regular paper regeneration does not need the tracer. `make_intro_figure.py`
uses the frozen paths and native typography to export `figures/intro.svg`,
the vector PDF, and a PNG preview using the existing Python/matplotlib setup.
The SVG uses named path groups and editable text; it does not embed the
generated raster. Text bounds and overlap checks run during export.

The vector-fidelity audit checks every exported path against its frozen
source coordinates and fill, checks the generated-master hash, inventories
all live labels, and compares the appearance against the selected bitmap.
The latter is a descriptive image comparison, not evidence of tracking
performance. The paper retains its seven-page body and eighth-page
acknowledgment/reference section.
