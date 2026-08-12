# V129 design: attenuated light payload on protected edges

## Decision

V126 proves that V105's useful network-level gain is compatible with strict
local safety when a safer network-informed state is available.  V127 and V128
show that replacing the working state with a same-step or independent local
posterior does not supply that information.  V129 therefore changes the
protected input itself instead of maintaining a second state chain.

On every V105-protected cross-formation edge, the sender transmits all active
labels after moment-compressing each Bernoulli spatial density to one Gaussian.
The receiver marks that input as light and applies a frozen reliability factor
of 0.5 before normalizing the KLA weights.  Unprotected edges retain the full
mixture payload and original weight.  The 0.5 factor is a single midpoint
chosen before opening V129, not a parameter sweep over tracking outcomes.

## Hypothesis

Control-only protection obtains aggregate gain by removing a harmful input,
but it can accumulate local information debt.  Full restoration reintroduces
the harmful mixture and creates a switch shock.  The light input is a
continuous middle action: it preserves label existence and the first two
spatial moments while reducing both payload size and influence.  If the debt
comes from missing coarse network information rather than missing multimodal
detail, this action can retain V105's mean gain and remove local regressions.

The experiment uses the existing strict V126 gate: at least 5% mean and mature
gain, nonnegative sensor/formation/formation-time safety, nonnegative F6 peer
terminal gain, nonnegative consensus gains, and nonnegative fully accounted
attempted-byte saving.  Failure remains a repository experiment record only.
