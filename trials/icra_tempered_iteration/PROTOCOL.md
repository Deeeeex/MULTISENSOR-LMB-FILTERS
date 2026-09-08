# Evidence-tempered positive recency: development protocol

This round starts after the complete M-ECR-S experiment and its additional
M-CR control. The original 25-sequence primary failed three of its four paired
interval checks. It is not relabelled as a success, and the better secondary
M-ECR-C mean is not promoted retrospectively. All nine original development
sequences and all 25 former fusion-reserved sequences are now seen outcomes.

## Mechanistic reason for this round

On the complete 25 sequences, the absolute support ceiling constrained 96,566
of 97,338 positive-age label events in the reliable condition and 68,110 of
68,702 in the intermittent condition. The ceiling admitted more existence
than CR in only 1,240 and 911 events, respectively. These are repeated
recursive label events, not independent statistical units. M-ECR-S reduces
false-target cost relative to ER while increasing missed-target cost. A raw
single-update association support and an accumulated posterior probability
have different roles; making the former an absolute cap nearly always rejects
positive recency when the posterior is already high.

The new hypothesis is that current evidence should control the *fraction of
positive log-odds correction*, retaining graded positive recency without an
absolute probability cap. This is a design hypothesis, not a proven diagnosis
of all observed errors. No age constant, threshold, score fit, backend,
association, birth, pruning, radio, crop, or extraction setting is tuned.

## Fixed rule

Use the existing eligible-source weights b, recency weights q, spatial
normalizer eta, and current local association support:

    z0 = sum_j b_j logit(r_j) + log(eta)
    Delta = sum_j (q_j - b_j) logit(r_j)
    c_j = sum_m W_jm v_jm, with the missed branch assigned value zero
    c = max c_j over represented, eligible sources with q_j > b_j + 1e-12
        and r_j >= 0.5; zero for an empty set
    zT = z0 + min(Delta, 0) + c max(Delta, 0)
    rT = sigmoid(zT)

Support is zero unless its source had a direct local opportunity in the
current update; received metadata cannot refresh it. Spatial fusion is the
same normalized geometric density on the same current inputs. For Delta <= 0
the implementation returns the existing ER scalar exactly. Thus CR <= T <= ER;
c=0 gives CR, c=1 gives ER, and one source or equal ages gives No-age. These
pointwise properties do not guarantee recursive cardinality or risk dominance.

For a fixed c and positive Delta this also minimizes the Bernoulli objective
whose source weights are (1-c)b+cq, with the same spatial term. For negative
Delta the original q is retained. This conditional objective has strictly
positive scalar second derivative 1/[r(1-r)]. The gate depends on the inputs;
this is not a Bayes proof, a statistical confidence bound, or a consensus
convergence result. Confidence-weighted fusion and marked likelihoods are
established ideas; this experiment does not claim otherwise.

## Primary, ablations, and data use

The primary is `marked_tempered_calibrated` (T-C): v is the already fitted
calibrated detection probability. This is chosen from the rule's interpretation
before any T trajectory is produced. `marked_tempered_score` (T-S) and
`marked_tempered_association` (T-A, v=1) are fixed ablations; neither can replace
the primary on the basis of this round's results. All three share the exact
same calibrated mark-likelihood local update with their No-age, ER, CR and
ECR references. No score parameters are fitted in this round.

Stage 1 runs all nine original development sequences, both radio conditions,
all three T arms. Existing complete reference trajectories are reused and
independently scored. Preflight additionally reruns ER on development 0000
to verify exact output and diagnostic parity, apart from runtime.

Stage 2 runs the same fixed three arms on all 25 former reserved sequences
only if T-C improves the stage-1 macro OSPA means against both M-No-age and
M-ER in both radio conditions and is no worse than M-CR in both conditions.
This continuation rule is set before preflight. Failure is reported and stops
this candidate's expansion; a secondary arm's favorable result cannot satisfy
the continuation rule. Stage 2 is labelled `seen_transfer`, never a new holdout.

The statistic remains sequence-macro 2D position OSPA (p=2, cutoff 12 m), with
paired 10,000-sequence bootstrap percentile intervals and the existing random
number setting. Report missed and false GOSPA squared costs, common-truth
localization support, all sequences, and actual packet bytes. A T packet uses
216 B/Bernoulli plus a 32 B header; original density references use their native
208 B encoding. Timing from concurrent processes is not a controlled benchmark.

Even favorable stage-2 results are development evidence after this round's
outcome-dependent method design. New prospective evidence is needed before
presenting the revised method as independently validated. The detector-trained
split and potentially related driving routes remain material limitations.

## Provenance and checks

No previous scientific source or result is overwritten. `source_sha256.json`
extends the previous immutable source list and includes this protocol, the new
runner/generator/rule, fixtures, and the complete preceding summary artifacts.
After preflight, `ROUND_FREEZE.json` fixes the successful parity evidence and
the full development driver before execution. A real successful MATLAB exit,
completion marker, and exact file count are required for each sequence.

Checks cover the endpoints and nontrivial interpolation, monotonicity in c,
unchanged spatial density and negative correction, single/equal-age identity,
excluded and stale sources, visible-absence censoring, current support reset,
and the packet field. Independent Python rescoring verifies every node-frame,
radio draw, byte total, analytic scalar, and same-input extraction.
