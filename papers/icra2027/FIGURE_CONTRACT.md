# Figures for the eight-page GCE manuscript

## Fig. 3 line-plot refinement

The claim remains that relative accuracy depends on modeled detection
probability and that PSD admission alone does not calibrate uncertainty
under correlated observations. Use a quantitative triptych: reliable and
intermittent tracking are paired comparisons with identical axes; the
correlation control is the distinct uncertainty diagnostic. Preserve all
47 measured points, the sequence averaging, the exact input hashes and
the existing 181 by 70 mm canvas. Do not smooth, jitter, resample, insert
uncertainty bands or select favorable settings during this visual edit.

Follow the aligned small-multiple and line-style approach in Nature
Methods' [controlled-perturbation figure](https://www.nature.com/articles/s41592-025-02630-5/figures/4),
and use [Paul Tol's qualitative palette guidance](https://sronpersonalpages.nl/~pault/)
to balance blue, rose, teal and neutral gray. Preserve the manuscript's
teal GCE identity. Distinct marker shapes and selected dashed lines provide
redundant identification; increase stroke and marker weight at print size.
Use one shared legend for the tracking pair and direct labels for the
three correlation curves. Label the nominal 95% line without implying
that it is an estimated interval. Keep the baseline traces fully visible.
Retain editable SVG/PDF text, 300 dpi PNG, at least 7 pt at compiled size,
label collision checks and a before/after source-value identity check.

## Reviewer revision: model sensitivity and uncertainty

Use the existing Python/Matplotlib backend and live-text SVG/PDF exports.
Replace the main-text sequence scatter and phase plot with one 181 mm wide,
three-panel quantitative figure: (a) modeled detection probability versus
sequence-macro OSPA under reliable links, (b) the same under intermittent
links, and (c) empirical coverage of nominal 95% spatial regions versus
known cross-source correlation. The first two panels include all four
registered pD values and all four recursive arms (No-age, Scalar, Guarded
Scalar, GCE), use identical vertical scales, and identify the nine old
development sequences as the averaging units. Curves join evaluated
settings only; no interpolation model or frame-level uncertainty is claimed.
The third panel includes all five correlation settings and all three
registered Gaussian controls, with 10,000 samples at each setting and an
explicit nominal-coverage reference. It is conditional on target existence
and uses unit admission, not the real tracker's adaptive gates. Every PSD
test passes in this control; the plot must retain its coverage failure.
Use teal for GCE, blue for Scalar, rose for Guarded Scalar, gray for
No-age, and distinct markers as redundant encodings. Labels remain at least
7 pt at final size. Check every label's canvas bounds and pairwise text
collision, and retain source values, dimensions, plotted point counts,
lineage hashes, SVG, PDF and 300 dpi PNG. The previous scatter and phase
figures remain in the companion evidence with their full data.

The new-data table reports both the mean over all three newly acquired
segments and the single recording absent from the earlier archives. Do
not label three segments as three independent routes or conceal the
single-recording reversal with the aggregate mean. Motion compensation
is reported as a separate matched sensitivity, without selecting a new
primary configuration after observing outcomes. The main manuscript
retains the illustrated introduction and editable mechanism overview.

## Retained introduction and companion diagnostics

The new introductory figure must explain why a posterior weight couples
inherited information and current sensing. Use an 89 mm single-column
illustration with two sensing vehicles, a target, shared history, and
two compact expressions: geometric pooling tempers current likelihoods;
admitted posterior-to-prediction ratios restore bounded fractions.
The final branch explicitly connects the corrected spatial density to
existence through its integral. This is a schematic-led composite, with
one continuous reading path and no empirical data, confidence regions,
or claimed independent priors. The shared-prior interpretation is labeled
as an idealized case; exact agreement with central Bayes requires exact
local likelihoods and unit gates. Keep live Arial-family text at least 7 pt,
gray history, blue/amber source identities, and teal admitted correction.
The selected generated master is replicated on the same 1385 by 1136 canvas:
all non-text geometric paths and their source coordinates are retained,
with live text and formulas rebuilt for editing. The street scene, sensing
wedges, history trail, method comparison, and coupled output line remain
explicit. Export SVG, PDF, PNG, the selected master and prompts, source
specification, geometric-fidelity audit, and text-bound checks using the
established Python backend. The scene is conceptual, not an experimental
capture. See intro_design/README.md for inspected ICRA references and
the generation, refinement, and vector reconstruction record.

The companion phase plot follows ANALYSIS_PROTOCOL.md: all 25 sequences in
each before/outage/after phase and both conditions. Plot GCE-minus-control
mean OSPA with complete-sequence paired percentile intervals. No frame
is an independent replicate. Display all phases and both No-age KLA and
Scalar references, including intervals that cross zero. Use one
single-column quantitative grid, 89 mm wide, with two condition panels.
The fixed-input 2 by 2 experiment remains in the companion table to expose all
four spatial/normalizer combinations without suggesting independent
recursive trajectories. The final paper has seven complete body pages
and a dedicated eighth page for Acknowledgment and References, using the
unchanged official template.

Use the existing Python/matplotlib workflow, with editable text SVG, embedded
TrueType vector PDF, and 300 dpi PNG previews. White backgrounds, one teal
method accent, slate reference marks, restrained blue for the second link
condition, and readable Arial type at the 181 mm final width are required.
All numerical data come from the complete portable experiment summaries.

| Figure | Conclusion and evidence | Composition | Required checks |
| --- | --- | --- | --- |
| Integrated overview | The same admitted current ratio modifies spatial density and existence normalization; conservative support and feedback remain explicit. | One continuous schematic, retaining the approved robot scene, source paths, junction, two mathematical branches, and feedback loop. | No panel cards, invented empirical values, spatial-invariance claim, or implicit censor spatial density. Update the equations to the actual Gaussian rule. |
| Sequence gains across link conditions, companion view | GCE lowers OSPA under both link conditions in 20 of 25 sequences against No-age KLA and 14 of 25 against Scalar; the scatter also shows heterogeneous gains and sign reversals. | Quantitative grid with two scatter panels, 181 by 83 mm. A point joins the reliable and intermittent gain for the same complete sequence. No-age KLA represents the inherited posterior pool; Scalar is the original unguarded existence-only reference; Guarded Scalar now appears in the revised manuscript tables. | All 25 sequences appear in each selected comparison, giving 50 two-coordinate points and 100 measured gains. Gain is reference minus GCE. Within each panel both axes have equal scale; ranges may differ between references for legibility. Zero lines, the equal-gain diagonal, and shape-coded outcome categories remain legible in grayscale. No jitter, selective point removal, fitted trend, or causal interpretation. |
| Components, companion view | The three single-component removals have distinct effects; Scalar is a separate reference. | Keep all 200 sequence points in an accompanying editable figure; Table II carries the component comparison in the manuscript, with full GCE last. | Retain every interval crossing zero and all outliers; distinguish the Scalar reference from the three one-component ablations. |
| Communication and accuracy | Exact zero omission moves GCE to a lower payload at identical accuracy. | One single-column panel, 89 by 59 mm, comparing all eight method/condition means in a common payload--OSPA coordinate system. Circle/diamond shapes identify link conditions; color and direct labels identify methods, and open markers identify the full GCE encoding. | Use actual per-sequence means; display the full-to-encoded horizontal move. Include all four specified formats without clipping. Table III reports raw and fragmented costs, while all eleven methods remain in source data. |

The main diagram is qualitative. Statistical units are complete sequences;
no frame is an independent replicate. The corpus has informed development.
The companion sequence scatter shows observations and deterministic sign counts, without confidence
intervals or an independence claim between its two coordinates. All six
geometric-reference comparisons and their 10000-resample paired percentile
intervals remain in the companion view and complete evidence data. The
companion scatter's two references are chosen for their methodological roles,
not by ranking their observed gains. No observation is excluded from a
displayed comparison.

Before delivery, check text bounds and collisions, live SVG text and absence
of embedded bitmap elements, embedded PDF fonts, and the figures at their
final physical sizes. Inspect the compiled paper pages as well as individual
previews, including grayscale rendering and final-size type. All floats must
precede References. The full paper must fit the official eight-page limit,
including references and the acknowledgment, without altering the template.
