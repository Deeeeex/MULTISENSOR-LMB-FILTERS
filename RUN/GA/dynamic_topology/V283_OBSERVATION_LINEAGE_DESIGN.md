# V283: distinguish untouched priors from weak historical evidence

Status: implemented; analyze only after V282 completes and its saved metric
prefix matches. No new tracking policy or filter run.

The current FoV-aware missing-label rule examines absent labels, not every
present low-existence label. That code fact alone does not establish the
cause of undercount. A node outside its current view can still hold valuable
historical information received earlier. Excluding every such input would
discard the very long-range tracking information the network is meant to
preserve.

V283 uses all 40 already opened reference steps, without selecting a favorable
anchor. It propagates one offline Boolean per retained label: has any local
observation opportunity (positive component-mean pD) entered this state through
local update or an actually used fusion input? Both detections and missed
detections count; active observable-absence censoring also counts. A pruned
state loses the flag. This is an opportunity lineage, not measurement-level
provenance or a proposed deployed metadata field.

Report the weight and negative log-odds contribution from never-informed
inputs, separately from inputs lacking a current observation opportunity.
Also count pools with weighted log odds below zero despite an input r>=0.9,
and how often those contain a never-informed input. The descriptive 0.5/0.9
levels are not output thresholds or method parameters.

Decision: if never-informed inputs remain substantial, investigate prior
versus evidence separation. If they fade but weak pools persist, a simple
birth-prior mask cannot be the main solution; investigate weak-history and
fresh-evidence attenuation. Neither outcome directly establishes an improved
tracking method, and the shared powered-GM approximation remains unchanged.

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeObservationLineageV283('RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat');"
```
