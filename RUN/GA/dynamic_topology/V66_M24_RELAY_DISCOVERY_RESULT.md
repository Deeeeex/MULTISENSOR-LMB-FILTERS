# V66 M24 relay current-only discovery result

The fresh linear-relay source protocol completed all 25 registered states on
`m24-formation-fov-relay`, seed `1301`, without reading a tracking outcome.
Every state remained below the unchanged V65 network-risk event budget of
`1%`, so neither V65 nor V66 authorized an intervention.

| Rank | Time | Network rescue risk | Decision |
|--:|--:|--:|:--|
| 1 | 44 | `0.839%` | reference fallback |
| 2 | 52 | `0.618%` | reference fallback |
| 3 | 128 | `0.566%` | reference fallback |

The other 22 states range from `0.021%` to `0.404%`.  Eligible time count is
zero, selected time is `NaN`, and tracking-outcome reads remain zero.  The
frozen stopping rule therefore forbids a paired tracking run; lowering the
event threshold after this scan would turn a cross-scene falsification test
into post-hoc candidate creation.

## Method decision

This negative result exposes an action-space boundary rather than a numerical
threshold defect.  V65/V66 can only withhold incoming cross-formation bundles
when those bundles suppress receiver-supported labels.  That action matches
conflict-dominated radial events and can be screened away in the weak convoy
case, but a linear relay is designed around sequential handover: its important
cross-formation opportunity is normally to transport sender-supported labels
toward a receiver that has not yet acquired them.  A suppression-only method
has no positive routing action with which to exploit that opportunity.

The next method should therefore treat cross-formation information value as
signed and label conditioned:

1. a **quarantine** term identifies a sender whose KLA contribution lowers a
   locally measurement-supported label;
2. a **transport** term identifies a sender-supported label that can reduce a
   downstream receiver's current existence or localization uncertainty;
3. the action generator may suppress a harmful input or redirect one fixed-
   budget gateway message toward a useful sender-receiver pair;
4. the same deterministic projection preserves physical reachability, message
   budget, useful-label retention, and rolling connectivity.

Before implementing that action, a source-only diagnostic should measure the
negative rescue mass and positive transport opportunity on the same 25 relay
states.  This distinguishes an actual lack of dynamic headroom from the more
specific hypothesis that the existing action has the wrong sign.

Generated evidence:
`RUN/GA/dynamic_topology/evidence/tracking_aligned_v66/relay_scene_discovery/INFLUENCE_AWARE_DECISION_BREADTH_V66_RELAY_DISCOVERY.md`.

Evidence boundary: fresh M24 scene-seed development discovery; no tracking,
held-out, validation, or learned-model claim.
