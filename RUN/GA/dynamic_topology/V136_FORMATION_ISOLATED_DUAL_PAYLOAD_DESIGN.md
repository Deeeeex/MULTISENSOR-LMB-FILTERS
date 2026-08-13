# V136 formation-isolated dual-payload upper bound

V135 proves exact safety but confines the working-state benefit to the
cross-input gateways.  Its all-formation M24 gain reaches `+5.207%` on the
third protected page, yet only three sensors retain any full-window gain.
The next question is therefore spatial rather than temporal: can the improved
working state spread inside its own formation without contaminating another
formation?

V136 maintains two causal states at every node:

- `W` is the estimator used for tracking.  On intra-formation edges it is
  fused from other `W` states, while selected cross-formation relay inputs are
  withheld according to the frozen V134 action.
- `R` follows the full-posterior static reference route.  It is the only state
  sent on cross-formation edges and the exact fallback when a formation exits
  protection.

An intra-formation link carries a compound `W + R` payload under one shared
attempt and delivery realization.  A cross-formation link carries only `R`.
All extra payload bytes are charged.  This is deliberately an estimation
mechanism upper bound, not a communication-saving method: maintaining exact
`R` while transmitting `W` needs additional representation.  If both M24 and
X36 show at least five-percent intervention gain with nonnegative full,
mature, sensor and formation gains and exact whole-formation reentry, the next
research task is to encode or multiplex the two states within a useful budget.
If either scale fails, the branch closes before any compression work.

Below-gate outcomes remain repository-only under the reporting policy.
