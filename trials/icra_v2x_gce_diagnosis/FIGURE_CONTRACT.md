# Diagnostic figure contract

Conclusion: extra negative scalar evidence can suppress accurately localized, persistently single-sensor targets; positive admission also helps other targets, so the observed losses do not justify deleting every current-evidence term.

The existing analysis workflow and inputs are Python. Use Python/matplotlib for drawing and export. No other plotting backend or fabricated values.

Quantitative grid with a probability-trajectory hero panel. Panel a gives the exact additive OSPA decomposition on all 14 additional segments, separately for both links. Panel b shows the longest strict No-age-only interval in the reliable additional cohort, with the current-label probability around the transition. Panel c shows the actual admitted positive and negative scalar log-odds terms in that interval. Panel d retains an improving segment as a counterexample, using the target already selected by the complete trace analysis.

Export at 183 mm width, approximately 148 mm height: editable SVG, embedded-font PDF, and a 300 dpi PNG preview. Retain full source CSVs, script, input/output hashes and Python package versions. Use readable sans-serif text and a consistent No-age/GCE palette. Figures are for this diagnostic report; no submission-readiness claim.

Panel a uses sequence-equal arithmetic means, 14 segments from five collection dates, with one frozen radio realization per condition. No uncertainty interval or significance claim is encoded. Panels b-d are post-outcome illustrative native traces, not independent replicates. Time axes are cropped to the transitions and the complete frame series remain in the source data. Report MAP extraction as observed, without drawing an invented 0.5 extraction threshold. A matched detection denotes a center within 2 m of the frozen annotation, not proof of physical visibility from the other sensor.
