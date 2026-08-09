# V66 M24 relay current-only discovery freeze

## Purpose

The first fresh test of V66 asks whether the observable gate can identify a
valuable intervention outside the radial-surround geometry.  The linear-relay
scene represents sequential target handover along a sensor chain and therefore
tests a different information-flow pattern from radial closure and parallel
convoy motion.

## Frozen source protocol

- Preset: `m24-formation-fov-relay`
- Seed: `1301`
- Scale: 24 sensors in 4 formations
- Registered snapshot times: `40:4:136` (25 states)
- Reference: `formation-h3-fixed-ccw-reference-v1`
- V66 robust decision-exposure threshold: `0.05` (unchanged)
- Tracking outcome scoring: disabled
- Model training: disabled
- Validation claim: disabled

The cache generator removes explicit target truth before the reference filter
runs.  Discovery may read only the current posterior, current physical links,
the registered KLA weights, and past topology required by the three-round
influence calculation.  It must not read E-OSPA, cardinality error, future
measurements, or any paired tracking outcome.

## Selection rule

Every registered time is evaluated by the unchanged V65 safety and useful-
information guards followed by the frozen V66 decision-breadth gate.  If one or
more states pass, discovery freezes at most one event by:

1. largest normalized robust decision exposure;
2. then largest V65 network-risk fraction;
3. then earliest time.

If no state passes, no tracking experiment is authorized and the negative
current-only result is retained.  If one state passes, its source record must
be committed before any paired tracking result is opened.

Evidence boundary: fresh scene-seed development discovery; source-only and
pre-outcome.
