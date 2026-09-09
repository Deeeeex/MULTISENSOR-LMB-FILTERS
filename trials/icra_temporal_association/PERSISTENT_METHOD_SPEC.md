# Executed persistent-conflict and branch rules

This specification describes the already frozen R and S implementations.
The result-selection gate is in `PATH_RESTORATION.md`. It makes no claim
that either candidate has passed that gate or the subsequent assessment.

## Current observation evidence

For a local Bernoulli component, normalize its executed local association
weights over the missed-detection branch and all current detections. Write
the resulting weights as $w_0,w_1,\ldots,w_M$. With
$a=\sum_{j=1}^M w_j>0$, let $\gamma_j=w_j/a$. The transmitted quantities
include

$$
\bar z=\sum_j\gamma_j z_j,\qquad
S=Q+\sum_j\gamma_j(z_j-\bar z)(z_j-\bar z)^\top,
\qquad q=r\,a\max_j\gamma_j.
$$

The complete eight-scalar message is association mass, two coordinates of
$\bar z$, three distinct covariance entries, peak conditional weight, and
conditional entropy. The original Gaussian-evidence packet has 352 bytes
per component; this message adds 64 bytes. The executed total is therefore
32 header bytes plus 416 bytes per component. Empty or unavailable current
evidence is encoded as eight zeros. The current-observation test requires
$q\geq 0.5$ at both endpoints.

For each qualified received pair, compute

$$
d_t=(\bar z_{i,t}-\bar z_{j,t})^\top
       (S_{i,t}+S_{j,t})^{-1}(\bar z_{i,t}-\bar z_{j,t}).
$$

For the same ordered pair of original source labels, use qualified samples
from the current frame and the preceding two frames that this receiver
actually received. If there are $k$ such samples, the normalized discrepancy
is their sum divided by the nominal 99% chi-square quantile with $2k$
degrees of freedom. The three constants are
9.21034037197618, 13.2767041359876, and 16.8118938297709. Each discrepancy
is calculated within its own frame; no cross-frame coordinate subtraction
or future observation enters this test. The thresholds are decision-rule
constants; temporal dependence is not modeled as a calibrated false-alarm
probability.

## Persistent identity conflict

Only currently shared, exactly equal source-label keys can enter the
conflict set. Entry requires qualified current evidence, $k\geq2$, and
normalized discrepancy greater than one. The same evidence requirements
with discrepancy at most one clear the conflict. An unavailable sample
does not clear it. A key is removed when it is absent from both current
inputs. Conflict state is local to each receiving vehicle.

If no currently shared label is blocked, the original Gaussian matching
function executes with its original arithmetic. Otherwise, consistent
shared labels remain locked. Conflicting shared pairs are forbidden;
every other free pair retains the original four-dimensional symmetric
Gaussian KL cost. The assignment uses the unchanged cost of 50 for each
unmatched endpoint. Neither the direct observations nor their historical
discrepancy replace this free-pair cost in R or S.

## Abstention and branch retention

R drops an unmatched incoming component whose original key still collides
with a local key. For a blocked local component without another accepted
remote match, the remote source abstains through the existing source-weight
transfer. It contributes no synthetic absence to that component.

S adds one rule. If the same unmatched collision currently meets the entry
test and its original birth-location field $b$ is below $10^6$, assign its
incoming component the key

$$
(t_{\mathrm{birth}},\;10^9+10^6 n+b),
$$

where $n\in\{1,2\}$ is the receiver. Retain it only if this key is absent
from all local components and all other retained incoming components.
The local source initially abstains for this branch. Its existence, mean,
and covariance equal the received single-source posterior. The conflicting
original local component follows R's abstention rule. Prediction, local
measurement update, subsequent fusion, and extraction then proceed through
the unchanged recursive filter.

Generated branch identifiers exceed $10^9$ and fail the original-key
condition for further splitting. Consequently, for any original key the
rule provides at most one deterministic branch key per receiver, two
branch keys across this two-vehicle replay. A pruned branch can be
recreated under the same key; no new identifier counter is advanced.
The birth-time field is an inherited label identifier. The creation event
has its own logged current frame.

All decisions depend on local posteriors and current or previously received
messages. Ground-truth object identifiers enter the independent evaluation
after the complete recursive trajectory. The native logs expose accepted
pairs, persistent conflicts, reopenings, omitted incoming components,
source-specific abstentions, and every branch creation. The audit checks
these against an independent assignment optimum, observation summaries,
causal history, and the unchanged Gaussian fusion equation.
