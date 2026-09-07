# Figure contract

Target: ICRA 2027, official ieeeconf, US letter, two columns. Export every
figure as editable-text SVG, vector PDF, and a 300 dpi PNG preview, using
the existing Python/NumPy analysis workflow with matplotlib only. No R,
AI-generated image, screenshot tracing, or external illustrative data.
Use Arial/Helvetica, 7--8 pt labels at final size, 9 pt panel labels;
single column 88.9 mm, double column 181 mm. Preserve SVG text and embed
PDF TrueType fonts. Colorblind-safe blue/orange/green/purple plus line styles.

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
