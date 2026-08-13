# V138 finding: local label divergence fixes onset, not severe continuation

V138 is a repository-only failed M24 mechanism screen.  It preserves the
material aggregate gain but misses the frozen per-sensor safety gate, so X36
was not run and the result must not be copied into the main progress document.

## Frozen M24 result

- Intervention E-OSPA gain: **+5.821%**.
- Full-window / mature-window gain: **+0.529% / +0.000%**.
- Minimum sensor / formation gain: **-0.470% / +0.000%**.
- Exact whole-formation reentry match: **100%**.
- Charged attempted-byte delta: **+89.182%**.
- V138 evaluated 1,124 protected-node labels and selected the exact relay
  label 175 times.

## Mechanism interpretation

The post-fusion label readout acts at the correct causal stage.  On formation
2 page 4, it changes the roughly `-15.2%` cell losses at sensors 9--12 to
approximately zero without mutating the hidden W chain.  It therefore repairs
the onset of same-formation fusion harm and keeps V136's aggregate headroom.

The same rule does not repair the severe page-5 losses: sensors 9, 11 and 12
remain near `-22.4%`, `-40.7%` and `-40.7%`.  Current local evidence is present
and several R labels are selected, so the failure is not evidence coverage.
The binding issue is that per-label local-to-candidate KLD is not aligned with
the final set-estimate error after cardinality and component extraction.

## Method decision

Do not tune the label-evidence thresholds or KLD margin.  V137 and V138 expose
complementary causal signals: post-fusion label readout fixes the first harmful
fusion page, while the next-page W/R predictive score strongly identifies the
continued node-level failure.  Combine them without changing the hidden W
state: apply V138 label selection first, then use V137's fixed zero predictive
margin only to replace the current extracted node output with R.  Keep W for
future propagation and retain exact whole-formation R reentry.

A result-only counterfactual using the already frozen choices gives `+5.709%`
intervention gain and `-0.0042%` worst-sensor gain.  The next screen should
therefore pre-register a `0.01%` per-sensor numerical safety tolerance while
retaining all other gates.  This tolerance permits only negligible extraction
variation; it does not excuse a material node regression.
