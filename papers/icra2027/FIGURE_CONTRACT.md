# Figure contract

Target: ICRA 2027, official ieeeconf, US letter, two columns. Export every
figure as editable-text SVG, vector PDF, and a 300 dpi PNG preview.
Quantitative figures use the existing Python/NumPy/matplotlib workflow.
For the main schematic, the user requested image generation first, followed
by an editable SVG reconstruction. Its generated reference is a design source,
not experimental data. The final schematic contains vector geometry and text.
Use Arial/Helvetica, 7--8 pt labels at final size, 9 pt panel labels;
single column 88.9 mm, double column 181 mm. Preserve SVG text and embed
PDF TrueType fonts. Colorblind-safe blue/orange/green/purple plus line styles.
The integrated overview may use Arial Narrow for compact annotations.

0. overview: one integrated, full-width mechanism figure, about 181 x 85 mm.
   A single continuous layout connects mobile robot sensing and encounters,
   local LMB prediction/update, observation-history qualification, existence
   and spatial weighting, the spatial-overlap coupling, and posterior feedback.
   No a/b/c partitions or three-column panel framing. Direct sensor age updates
   on detections and misses; untouched priors stop at qualification in the
   depicted informed-input case; observable absence feeds existence only.
   The diagram is illustrative. Its age curve has floor rho=0.25, and a
   recent-miss hypothesis lies in the current FoV. Preserve the generated
   design and exact prompt under main_figure_integrated/, with any scientific
   or layout corrections recorded separately from quantitative results.

1. `scene`: schematic-led composite. Claim: prescribed robot motion creates
   separated sensing histories before physical reunion. Show split and churn
   path snapshots, radio MST edges, FoV circles, fixed candidate regions, and
   component-count timelines. All positions come from saved input geometry;
   no imagery or validated robot controller is implied. Source the first
   validation seed by index, not best performance. Double-column, 60--65 mm.
2. `mechanism`: analytic composite. Claim: direct opportunity age and spatial
   density carry different information. The causal metadata flow is specified
   in the method text; show an analytically specified two-Gaussian example
   of age weights changing existence with fixed spatial weights. Label the
   example illustrative, not experimental. Single-column, about 48 mm.
3. `outcomes`: quantitative grid. Compare the existence-only rule to lineage
   and both-block recency in paired OSPA differences, false-target squared
   costs and common-target localization. Absolute OSPA appears in Table I. Show all
   20 paired episode points where space permits and paired mean-difference
   bootstrap intervals. No fabricated values, seed omissions or frame-level
   significance. Double-column, about 80 mm, to retain readable labels.
4. `time`: quantitative grid. Claim from validation: contact
   transitions and target departure expose distinct fusion behaviors. Plot
   mean OSPA and false-cost trajectories for the main ablations, with event
   frames; component counts appear in `scene`. Curves are unsmoothed means;
   episode-level uncertainty appears in `outcomes`, never as 8x120 independent
   samples. Double-column, about 51 mm.

Statistics: n=20 independent random seeds per scenario; paired truth,
measurements, layout offsets and link uniforms across methods. Mean and
95% percentile bootstrap interval (10,000 resamples; seed 8301) on episode
summaries, descriptive without multiplicity correction. Development n=3
is separately identified and never pooled into validation. GOSPA miss/false
panels are squared costs (m^2); common-target RMSE conditions on shared
assignment support. CSV/JSON source data and scripts accompany every plot.

QA: inspect every PNG at final size and every compiled PDF page. Check
clipping, readable legends, zero-inclusive error-cost axes, exact units,
labels and citation claims; parse SVG for live text and PDF for embedded
fonts. Editing layout may not select trials, alter values, smooth curves,
or hide an adverse arm. Tables retain complete arm coverage even if the
plots focus on the preregistered comparisons.

5. `v2v4real`: quantitative grid, existing Python/matplotlib backend.
   Claim: the real-detection replay has heterogeneous sequence outcomes;
   an existence-age gain over its own no-age ablation does not transfer
   consistently. Two aligned heatmaps show ER minus each comparator's OSPA
   for all nine sequences under reliable and intermittent links. Include
   no-age, MIL-AM, and both TC windows without outcome selection. Center the
   diverging color scale at zero and display the actual signed values; use
   one symmetric color range across both panels. Negative favors ER.
   Source: the independently rescored complete 18-case replay summary.
   Double-column, about 181 x 65 mm; vector SVG/PDF and 300 dpi PNG.
   The main table reports absolute errors and sequence SD; paired sequence
   bootstrap intervals are in the prose and source data. No cell is an
   independent significance test. Labels 0000--0008 denote released
   evaluation sequences, not nine independently sampled driving routes.
