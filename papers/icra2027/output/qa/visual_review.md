# Visual review of the Fig. 3 line-plot refinement

Reviewed on 2026-09-09. PDF SHA-256: `d912ef666c3466c6089680a3975f680e0b69132ce14230bb8e8722780fecab62`.
Baseline manuscript: commit `8ff9e0874d781fe493dbf67ff7fb907a59a62a11`,
PDF SHA-256 `eb0e7563637f827773dee012c60b95b2bbe8f50788465f87a6014240d9614634`.

## Verified scope

This edit changes Fig. 3 styling and clarifies that the nominal-coverage
reference is a horizontal gray dashed line. The 47 measured points, all
axis ranges, all original figure evidence fields, the full reviewer input,
scalar facts and generated tables are unchanged. No smoothing, jitter,
resampling, uncertainty band or new experiment is introduced.

Pages 1-5, 7 and 8 are pixel-identical to the previously reviewed manuscript
at 1.5x RGB resolution. Page-6 changes are confined to the Fig. 3 and caption
area, ending above 300 pt; the remaining page content is pixel-identical.

## Figure and page inspection

- Fig. 3 retains the 181 by 70 mm canvas and both matched pD axes. The
  paper scales it to its 177.8 mm text width; the smallest source text is
  7.3 pt, remaining above 7 pt at that compiled size.
- Blue, rose and neutral-gray baselines use hollow, distinct marker shapes
  and different dashed patterns. GCE retains the paper's teal identity
  with a modestly heavier solid line and filled circles.
- The two tracking panels use one shared legend. Correlation curves have
  direct labels; the GCE endpoint annotation is read from the saved
  0.9-correlation row and displays 80.55%. Its label leader anchors to the
  saved 0.75-correlation measurement.
- The oracle label was repositioned below its own curve after detecting
  a clash with the descending GCE curve in the first styling draft.
  The final color and grayscale renders show no text/curve collision,
  clipped label, missing data setting or displaced measurement.
- The complete compiled page 6 was inspected at 1.5x. Panel headings,
  tick labels, direct labels, caption and neighboring communication plot
  are legible without overlap.
- All statistical limitations remain explicit in the unchanged results:
  ranking reversals, small recursive effects and covariance undercoverage
  have not been hidden by the visual refinement.

## Public design references

The [Nature Methods controlled-perturbation figure](https://www.nature.com/articles/s41592-025-02630-5/figures/4)
was inspected for aligned small multiples and consistent color/line-style
encodings. [Paul Tol's qualitative color guidance](https://sronpersonalpages.nl/~pault/)
informed the blue/rose/teal/neutral palette; the exact paper palette is an
adaptation, not a certification of every color-vision condition.
[Nature's figure specifications](https://research-figure-guide.nature.com/figures/preparing-figures-our-specifications/)
informed editable typography and final-size inspection. The figure does
not reproduce the source paper's scientific data or image content.

## Artifact checks

The automated artifact checks pass: eight US Letter pages, seven complete
body pages, one Ack/References page, four manuscript figures and four
tables, 23 embedded fonts, no Type 3 fonts, no annotations, 30 citations
and 26 verified DOI identifiers. The source SVG has live text and no
embedded raster; every plotted setting and all text bounds are checked.
Page 7 ends at 733.929/718.836 pt in its left/right columns. The official
class, bibliography style and manuscript body font remain unchanged.

This is a visual artifact revision, not a new native tracking experiment
or an independent scientific replication. The paper source archive is
verified separately by `portable_rebuild.json`.
