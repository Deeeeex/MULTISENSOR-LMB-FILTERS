# Final admission assessment

This summary is specified before inspecting either external tracking scores
or the remaining V2V4Real candidate scores. The three development families
retain their original registrations and every tested outcome. Read the
third-family `FINAL_SELECTION.json` before reading any evaluation scores.
That file fixes the candidate and constant-strength reference using the
complete, audited recursions of the original nine development sequences.

Combine the original seven-method, 43-scene V2V4Real results with the
selected candidate and the newly selected constant strength. Reuse complete
matched trajectories; run only missing method/scene/condition combinations.
Keep the original GCE and the earlier selected projected candidate visible.
The 43-scene aggregate is a full-release characterization, including all
development inputs. Report the nine development scenes and remaining 34
separately, without claiming the latter are independent of the development
recordings or detector training.

For V2X-Real, retain all five metadata-selected vehicle-pair segments and
619 paired frames in the primary external aggregate. The two 40 m sensing
disks never overlap in two segments; the remaining three have overlapping
disks throughout. This division uses only the already frozen vehicle-pose
range extrema and the unchanged sensing radius, before tracking scores are
inspected. Report this geometry division as a secondary diagnostic, with
the five-segment aggregate first. Equal weights apply to segments. Also
report collection-date means (three dates), per-segment differences and
wins/ties/losses. Do not attach an inferential interval to three dates or
describe the five segments as five independently sampled recordings.

For V2V4Real, report equal-segment, frame-weighted and equal-recording OSPA;
recording-level paired descriptive intervals use 10,000 whole-recording
resamples, seed 8301. Pair within a recording, average its segment
differences, and resample the 17 recording means. Preserve missed-target,
false-target and localization squared GOSPA costs. A development winner is
not automatically an improvement on evaluation data. No new selection may
be described as held-out after its evaluation scores have been inspected.

All contributing native stages must have zero exit codes, expected result
counts, matching input/source hashes and independently passed full-trace
audits. The source package will carry the aggregate rows, registrations,
input/audit receipts and analysis code; public raw point clouds and large
native trajectory archives remain in the research checkout.

Execution record: after this protocol and the third-family selection rule
were written, a routine log tail exposed two reliable-link train_0014
scores before the final selection report was emitted. All third-family
native development runs had already completed. The candidate set and
development-only rule are unchanged. `icra_compatible_admission/`
`DATA_EXPOSURE.json` records the exact exposure. V2X tracking scores remain
uninspected at this point.
