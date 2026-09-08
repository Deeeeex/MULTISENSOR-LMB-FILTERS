# Common-input execution port before any holdout tracking

The new runner combines the frozen original and marked local updates with
their matching fusion controls. It carries both local-only streams, No-age,
ER, MIL-AM, TC-5/10, unmarked ECR-C, and marked ECR-A/S/C. Total: 16 arms.
Save each complete sequence/condition/arm immediately as a separate compressed
file, keeping the longest 1244-frame sequence resumable and individual files
small. The cohort audit requires every registered arm and both conditions.
These are a fixed comparison menu; the primary method and contrasts will be
specified after development audit in METHOD_FREEZE.json, before holdout.

Before that file exists, run only development_check mode on seen inputs.
Compare outputs against original unmarked six arms, ECR-C, and all five
marked development arms wherever available. Recompute all new output metrics
independently. Preserve density, birth, local mark mapping, radio seeds,
absence eligibility, matching, crop and output extraction. TC receives the
local-only history from the corresponding marked/unmarked backend and uses
the existing author kinematic implementation and history packets.

All density controls send their actual required original 26-scalar Gaussian
packet (208 B/object + 32 B header). ECR arms add their one support scalar
(216 B/object + 32 B header). This removes the unused support field from
marked No-age/ER development instrumentation; it cannot alter their density
or recency rule. Record per-source packet byte lengths each frame and verify
raw/delivered/padded/control bytes directly. No favorable baseline wire cost
is inferred by retrospectively editing a reported output.

The local mark-only likelihood update is a shared empirical observation
model, not a new external algorithm reproduction. Source provenance, proper
native-protocol scope and correlation/calibration limitations from the prior
round remain applicable.
