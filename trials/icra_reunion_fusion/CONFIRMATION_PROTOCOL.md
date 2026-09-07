# Local confirmation ablation v3

Registered 2026-09-08 after v2 analysis and before executing v3. The seven
false estimates in the no-new control all use candidate label (35,3), around
frames 88--91. Saved v2 outputs do not establish which local observation
created this hypothesis. V3 therefore records local association mass and
confirmation as well as testing a bounded mechanism change.

The hypothesis is that excluding never-observed priors should require an
actual positive local confirmation, rather than observation opportunity
alone. A confirmation is created by two consecutive updates at the SAME
robot with predicted detection opportunity and detection-association mass
at least 0.5. Empty measurements always reset the streak; stale diagnostics
in an empty-measurement update cannot count as a detection. Streaks are
local, are not transmitted, and reset when the label is absent. The one-bit
confirmation persists with a retained label and is OR-propagated only from
actual inputs. It is not a new observation and does not refresh direct age.

For an unconfirmed label use ordinary FoV-aware participation. Once any
input has positive confirmation, enable the existing observation-lineage
restriction, retaining legitimate negative evidence. All output thresholds,
pruning, FoV censors, 0.25 age floor and 5 s decay remain frozen.

Arms: `qualified_exist` is an exact v2 replay with extra diagnostics;
`confirmed_lineage` gates lineage restriction and uses ordinary weights;
`confirmed_exist` adds the frozen existence-only recency to that gate.
The two-hit rule is conventional confirmation, not a novelty claim. This
ablation changes qualification semantics, not the filter or motion stack.

All nine exact v1 caches are reused. Every v3 arm serializes the new bit as
one double (8 bytes per represented label) in the same padded packet; raw
bytes are reported. All other inputs and communication schedules are equal.
V2 sources/results and failed screen remain immutable.

Use the unchanged v2 development criteria relative to lineage: at most 2%
OSPA regression on either event scene, at least 10% lower departure false
cost, at most 2% common-target RMSE regression, and zero no-new false cost.
No further confirmation/age/output tuning follows these outputs. Even a
development pass only warrants a separately frozen new-seed experiment.
