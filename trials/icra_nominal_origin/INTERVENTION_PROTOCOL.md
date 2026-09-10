# One-time removal of the earliest nominal extra negative scalar update

The completed six-trajectory trace and independent target/first-event check
precede this registration. In both links, GCE and GS first differ from
No-age at frame 2 on the same local source distributions and source-label
matching. At robot 1, the meaningful existence difference is one label:
0.008679466919197855 (No-age) versus 0.003401795027527528 (GCE). Its extra
negative scalar increment is -0.9419666289521098 log odds; inherited odds
and selective-history changes agree, and spatial differences are at
roundoff scale. Local updates first differ at frame 3. These timings do
not establish that this first negative increment causes the later failure.

Run one declared intervention on `v2xt_0001`, nominal pD=0.9, under both
reliable and intermittent links. Use ordinary GCE throughout. At every
received fusion at frame 2 only, remove the accepted extra negative scalar
term from all output labels:

    r_event = sigmoid(sum(beta * local_logit)
                      + sum(kept * max(local_increment, 0))
                      + actual_spatial_log_integral).

Keep the complete actual GCE mean/covariance, history term, accepted
positive scalar term, source curvature decisions and all other metadata.
Keep the ordinary local negative evidence already inside each posterior.
At every other frame use unmodified GCE. No truth, target ID, label ID,
neighborhood, score or future outcome determines which labels change.
Frame 2 is fixed from the completed first-divergence trace.

Log old/new existence and all scalar terms. Reconstruct the original GCE
existence before changing it and require exact agreement, so a numerical
round trip cannot silently change unaffected labels. When the accepted
negative term is zero, preserve the original existence exactly. Verify
all means, covariances and metadata remain exact at the event.

First rerun GCE under both links with the new recorder and require exact
original common-field parity excluding runtime. Freeze those two control
trajectories and the two intervention trajectories before any new native
outcome. Run the two link units with separate logs and exit receipts.
Reuse both complete audited No-age reference trajectories. Require full
source, observation, probability, Gaussian, matching, packet, extraction
and metric audits, plus exact GCE prefix parity before the event.

Report all six trajectories, full 240-frame OSPA/GOSPA and error parts,
native bytes, full target robot frames and the existing 53--122 window,
candidate counts and association concentration. Report each link separately.
If this single early change is insufficient, end this exact experiment;
do not enlarge its duration or search another start frame after outcomes.
A repair would support a causal role for the initial scalar state in this
already exposed case, not establish a new method or generalization. The
closed continuous miss-history, range-model, joint-admission and previous
range-case interventions remain closed.
