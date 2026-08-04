# V48 contraction-phase development gate

## Decision

The all-receiver contraction-guided phase scheduler is **not advanced as the
primary runtime method**.  It remains useful as a graph-only upper-bound probe
and as a possible schedule teacher.  No posterior, tracking, total-byte, or
end-to-end communication claim is authorized.

## Evidence boundary

- Policy inputs: current repaired route, current link-success probabilities,
  formation membership, and immutable physical sensor identifiers.
- Forbidden inputs: posterior values or summaries, measurements, truth,
  delivery acknowledgments, realized packet draws, and future pages.
- Comparator: the best of all four synchronized pulse phases, not phase 1.
- Budget: every frozen candidate uses exactly \(5N\) posterior-message
  opportunities per four steps, versus \(8N\) for the full reference.
- Control: M24 and X36 incremental lower bounds are 2,482 and 3,859 bytes per
  window.  Tree construction, loss, latency, commit, and shared byte budgets
  are not implemented, so these are not deployable cost results.

## Seed-41 multistyle headroom

| Scale | Style | Best synchronized \(\rho_4\) | Searched \(\rho_4\) | Improvement |
|:--|:--|--:|--:|--:|
| M24 | radial | 0.899619325 | 0.888832780 | 1.199% |
| M24 | convoy | 0.972606384 | 0.956955767 | 1.609% |
| M24 | relay | 0.973748153 | 0.957177407 | 1.702% |
| M24 | crossing | 0.972473337 | 0.960536520 | 1.227% |
| X36 | radial | 0.952840622 | 0.951348673 | 0.157% |
| X36 | convoy | 0.988574954 | 0.980396091 | 0.827% |
| X36 | relay | 0.990002421 | 0.982180883 | 0.790% |
| X36 | crossing | 0.989502209 | 0.982931860 | 0.664% |

M24 median headroom is about 1.42%; X36 median headroom is about 0.73%, and
all four X36 cases fall below the 1% materiality threshold.

The multistyle run hash was
`eaffa3b5b7ec32b108029ce80f06591135c519c9ae318acc99243d69a007641a`.
That run began before the final materiality-field and control-contract edits;
the optimizer objective and unconstrained numerical path were unchanged, but
the artifact is classified as exploratory rather than frozen evidence.

## Lagged-window falsification

The lagged probe plans from \(t-1\), executes the actual repaired route on
\(t:t+3\), and separates two comparators:

- causal synchronized: the pulse phase chosen from \(t-1\);
- posthoc synchronized: the best pulse phase after viewing all four executed
  route and reliability pages, used only as a diagnostic lower bound.

For radial seed 41, M24 improves by 1.200% and X36 by 0.157% against both
comparators.  Neither repaired route changes within the tested window, so the
X36 deficit is not a frozen-route artifact in this check.  The current lagged
artifact hash is
`dd1d7f01ae527bb71fdcc7991197393547366abdc3319138d6b941465c991f83`.

## Interpretation

The scheduler spreads every receiver's residual opportunity across four
phases.  On M24 this sometimes reduces the exact mean-square propagation
factor materially.  On X36, however, the best possible graph-only adjustment
is already small before adding control traffic or packet-loss failure modes.
Implementing a centralized tree, per-receiver pulls, and distributed fallback
would add complexity without enough demonstrated structural margin.

The default optimizer therefore requires at least 1% improvement over the
best synchronized pulse before executing a non-synchronized phase vector.
The headroom probe alone disables this gate to measure the ceiling.

## Next screen

The successor must preserve the main synchronized mixing event.  At the burst
page it keeps all local residual edges and one directed formation cycle, while
allowing only selected edges on the opposite cross-formation cycle to move by
one step.  Before using any posterior-derived bid, enumerate this restricted
action set over M24 and X36 route pages and require:

1. physical support and exact \(5N\) four-step residual opportunity count;
2. a mandatory directed formation cycle on the burst page;
3. attempted rolling-B4 connectivity under current-page repair;
4. material, nonnegative contraction headroom on both scales; and
5. deterministic physical-UID permutation invariance.

Only if this graph-only screen passes should a local bid, charged two-phase
commit, shared byte cap, or real-filter experiment be implemented.
