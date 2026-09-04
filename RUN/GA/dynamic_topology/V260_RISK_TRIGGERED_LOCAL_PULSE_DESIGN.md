# V260 risk-triggered formation-local pulse

## Why the action changes

The completed V259 trace separates two failure modes.  Its conditional
localization tail selects formation 4 from `t=57`, one page before the opened
`t=58--73` event, and keeps it first on every event page.  The earlier V258
probe restored only one local input on one page; that action was too narrow
for a formation-wide covariance tail and produced a delayed sign change.

V260 therefore keeps the V242 `N+2(F-1)` graph as the fallback and changes
only the localization action.  On a strong localization trigger it restores
the complete set of currently omitted V240 local-residual inputs in the one
selected formation.  The bundle contains four or five messages in the M24
case.  No other formation is modified.

## Causal trigger and pulse

For formation `f`, let `L_f` be the V259 upper-tail conditional localization
risk and let `m_L` be its median across formations.  A formation is eligible
when

`L_f >= 0.25` and `L_f / max(m_L, eps) >= 2.5`.

The largest eligible formation is selected.  A pulse is applied immediately,
then the following page is a mandatory V242 cooldown.  This realizes a
period-2 action without fixing an outcome-selected calendar phase.  If the
risk disappears, the policy stays on V242.  The selector uses current LMB
posteriors and past routes only; it reads no truth or future measurement.

Two pulse strengths have a distinct interpretation:

- `0.10` is period-2 mass matched to the V240 residual weight `0.05`;
- `0.20` is the bounded strong pulse identified by the earlier contraction
  analysis, leaving at least `0.10` self weight on every pulsed receiver.

This is not a parameter sweep.  It is a two-point mechanism decision between
equal average KLA mass and a stronger intermittent update.

## Short continuation gate

Both candidates start from the captured V242 pre-topology state at `t=57` and
run causally through `t=73`.  The recorded V242 trajectory supplies the
paired reference; it is not rerun.  A candidate is useful only if it repairs
formation-4 event RMSE without degrading its E-OSPA, keeps network E-OSPA,
RMSE and consistency within a two-percent regression cap, keeps every
formation within a three-percent cap, and retains positive projected byte
saving versus the static route.

Passing authorizes a full causal M24 arm.  It does not establish a full-run,
multi-seed, X36 or paper claim.  The current policy also assumes a centralized
formation-risk synopsis; a distributed synopsis and its bytes remain a later
deployment requirement.
