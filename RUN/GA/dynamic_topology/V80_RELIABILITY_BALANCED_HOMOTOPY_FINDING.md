# V80 reliability-balanced homotopy finding

V80 finds genuine X36 recovery headroom but rejects the same global weight
family as a scale-universal solution.

| Case | Best alpha | Next centered energy | Change from alpha zero | Contracts from pulse |
|:--|--:|--:|--:|:--:|
| M24 historical | 0.00 | 0.00557678 | 0 | no |
| M24 aligned | 0.00 | 0.00421286 | 0 | no |
| X36 historical | 0.02 | 0.00143671 | -0.715% | yes |
| X36 aligned | 0.05 | 0.00145866 | -1.148% | yes |

The M24 response is monotone in the wrong direction.  Even the smallest
positive balancing point increases centered energy relative to ordinary
reference recovery, and stronger balancing rapidly injects spatial
heterogeneity.  Thus M24 cannot be repaired by a scene-independent global
trust or balancing scalar.

X36 behaves differently.  Small positive corrections reduce the dominant
existence-centered term before spatial forcing becomes material.  The useful
interval is narrow: historical and aligned routes prefer `0.02` and `0.05`,
while corrections of `0.20` or larger already lose the desired margin.  This
is the first recovery-weight evidence in the current V75--V80 line that
strictly improves the X36 source mechanism over fixed reference recovery.

The correct conclusion is not to fit a scale-specific alpha.  V80 supplies an
optional global action that a causal, posterior-aware recovery controller can
select on X36 and reject on M24.  The missing complementary action should be
intervention-local: after a candidate pulse changes a small number of residual
senders at weight `0.05`, restore the incumbent sender with a fractional
residual trust and return the remaining weight to self.  This changes only the
affected receiver rows, controls the affine forcing term directly, and leaves
the dominant `0.70` route untouched.

V81 should therefore use one shared recovery action bank on both scales:

1. ordinary reference recovery;
2. the frozen small global balancing points from V80;
3. intervention-local soft-return residual trusts.

At each of two recovery rounds, the controller may simulate these causal
one-step actions from the current virtual posterior and choose the minimum
V77 centered energy, with reference as an exact fallback.  A source mechanism
pass requires the selected energy to be non-increasing in both rounds for all
historical and aligned M24/X36 routes.  Only then should closed-loop tracking
be opened.

V80 remains opened-anchor, source-only evidence.  It reads no prediction, new
measurement, future link, truth, packet draw, route execution, tracking
outcome, or model training.
