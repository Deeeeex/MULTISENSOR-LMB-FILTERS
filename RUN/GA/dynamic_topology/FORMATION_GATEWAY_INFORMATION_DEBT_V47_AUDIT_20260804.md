# V47 gateway information-debt audit

## Decision

**V47 is rejected before the full M24/X36 tracking screen.**  Its runtime
implementation is reproducible and its nominal payload-message budget is
exact, but the current method does not close the communication-accounting
boundary and the first same-budget M24 structural diagnostic is unfavorable.

## Evidence

| Check | Result | Interpretation |
|:--|:--|:--|
| Real-filter paired smoke | reference `144`, candidate `90` messages | exact 37.5% reduction in posterior-message attempts over 8 steps |
| Repeated smoke identity | SHA-256 `a34606a32ec46e6b396372e0dd3444c286b2741974454818a1c414d2dfaf84f0` | deterministic paired execution |
| Short M24 structural screen | V47 `0.814672`, synchronized B4 `0.784350` | V47 is worse under the post-hoc fixed-route mean-square diagnostic |
| Rolling formation safety | pass, no fallback | attempted route is structurally feasible in the audited window |
| Adversarial service-horizon tamper | rejected | independent registered-policy replay closes the self-reported-horizon bypass |
| Distributed information boundary | fail | the scheduler reads all-node current posterior summaries and a global ACK ledger without charging their exchange |

The mean-square values are development-only diagnostics evaluated on the
realized route sequence.  Because the adaptive route depends on prior packet
deliveries, they are not a closed-loop mean-square certificate and are not
reported as tracking accuracy.

## Root cause

The scheduler is causal in time but not communication-self-contained.  It
does not read truth, future outcomes, or current packet-loss draws.  However,
the implementation supplies one callback with every node's current LMB
posterior, from which it constructs moment summaries, and with the global past
delivery mask.  A physical distributed network would need a controller and a
control channel to collect those summaries and acknowledgments and distribute
the selected schedule.  The nominal 37.5% saving excludes that traffic.

This is a method claim failure, not merely a missing diagnostic field.  Adding
`centralized` to the prose would make the implementation assumption explicit,
but would not establish an end-to-end communication saving.  The cost must be
implemented and charged, or the policy must be redesigned around information
that every decision maker can reproduce without an unmodeled exchange.

## Runtime verifier repair

The original V47 verifier validated the callback's schedule certificate and
hash only for internal consistency.  A callback could coherently alter the
service horizon, recompute the hash, and make an unsafe rolling window appear
immature.  The repaired verifier independently calls the registered V47
builder from the raw sanitized context and compares the complete deterministic
action and evidence object, excluding only wall-clock timing.  It therefore
recomputes the causal base projection, reference route, quota, service horizon,
selected edges, weights, rolling fallback, and all hashes rather than trusting
the callback's copies.

The regression suite now includes a coherently re-hashed horizon tamper in
addition to selected-UID, causal-flag, projection-hash, and route-hash tamper
cases.

## Successor requirements

A successor method must satisfy all of the following before any new tracking
matrix is opened:

1. expose either a strictly common-information scheduling contract or an
   explicit control-plane protocol;
2. include control summaries, ACKs, and schedule dissemination in attempted
   and delivered communication cost;
3. preserve independent action replay, physical support, row-stochastic
   weights, and rolling formation connectivity;
4. beat synchronized B4 in a same-total-byte structural or short closed-loop
   development screen before opening X36; and
5. retain D12 only as a diagnostic, with M24 development and X36 scale transfer
   required for the main claim.

## Claim boundary

This audit establishes a negative development decision and a repaired runtime
evidence boundary.  It does not establish tracking degradation, because the
full paired tracking outcome was not scored.  It does not establish that every
centralized scheduler is invalid; it establishes that V47's uncharged control
inputs make its present end-to-end communication claim incomplete.
