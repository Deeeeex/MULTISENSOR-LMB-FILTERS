# Method: Adaptive KLA Fusion

## Paper-Ready Method Draft

This section describes the adaptive fusion rule used in the present distributed GA-LMB implementation. The starting point is a conservative Kullback-Leibler-average or geometric-average fusion strategy for neighboring LMB posteriors under unknown cross-correlations \cite{Battistelli2014KLA,Hlinka2014ICI}. The main methodological question is not how to redesign the underlying labeled-RFS filter family, but how to allocate fusion weights so that they reflect posterior quality and realized communication quality in a communication-constrained sensor network.

### 1. Baseline GA-LMB Fusion

At node $s$, let $\mathcal{N}_s$ denote the local communication neighborhood and let $\pi_k^{(j)}$ be the measurement-updated LMB posterior provided by node $j \in \mathcal{N}_s$. The baseline distributed fusion target is the weighted geometric average

$$
\bar{\pi}_k^{(s)}(X) \propto \prod_{j \in \mathcal{N}_s} \left(\pi_k^{(j)}(X)\right)^{\omega_{k,s}^{(j)}},
\qquad
\sum_{j \in \mathcal{N}_s} \omega_{k,s}^{(j)} = 1,
\qquad
\omega_{k,s}^{(j)} \ge 0.
$$

In the current GA-LMB implementation, each Bernoulli component is fused after moment matching. For the $i$th Bernoulli component at node $j$, let $(r_{k,i}^{(j)}, p_{k,i}^{(j)})$ denote its existence probability and spatial density. After moment projection, the Gaussian approximation of $p_{k,i}^{(j)}$ is

$$
p_{k,i}^{(j)}(x) \approx \mathcal{N}\!\left(x;\nu_{k,i}^{(j)}, T_{k,i}^{(j)}\right).
$$

Using spatial fusion weights $\omega_{k,s}^{x,(j)}$, the fused spatial density is obtained in canonical form:

$$
K_{k,i}^{(s)} = \sum_{j \in \mathcal{N}_s} \omega_{k,s}^{x,(j)} \left(T_{k,i}^{(j)}\right)^{-1},
\qquad
h_{k,i}^{(s)} = \sum_{j \in \mathcal{N}_s} \omega_{k,s}^{x,(j)} \left(T_{k,i}^{(j)}\right)^{-1}\nu_{k,i}^{(j)},
$$

$$
\Sigma_{k,i}^{(s)} = \left(K_{k,i}^{(s)}\right)^{-1},
\qquad
\mu_{k,i}^{(s)} = \Sigma_{k,i}^{(s)} h_{k,i}^{(s)}.
$$

Using existence fusion weights $\omega_{k,s}^{r,(j)}$, the Bernoulli existence probability is fused by the corresponding weighted geometric rule:

$$
r_{k,i}^{(s)} =
\frac{\eta_{k,i}^{(s)} \prod_{j \in \mathcal{N}_s} \left(r_{k,i}^{(j)}\right)^{\omega_{k,s}^{r,(j)}}}
{\eta_{k,i}^{(s)} \prod_{j \in \mathcal{N}_s} \left(r_{k,i}^{(j)}\right)^{\omega_{k,s}^{r,(j)}} +
\prod_{j \in \mathcal{N}_s} \left(1-r_{k,i}^{(j)}\right)^{\omega_{k,s}^{r,(j)}}},
$$

where $\eta_{k,i}^{(s)}$ is the Gaussian normalization term induced by the fused spatial density. If all weights are fixed a priori, the above rule reduces to ordinary fixed-weight GA fusion. The method proposed here instead adapts these weights over time.

### 2. Factorized Adaptive Weight Backbone

The adaptive design begins with a shared quality backbone. For node $s$ and neighbor $j \in \mathcal{N}_s$, define the raw score

$$
\tilde{\omega}_{k,s}^{(j)} =
m_k^{(j)}
\cdot
q_{\mathrm{cov},k}^{(j)}
\cdot
q_{\mathrm{link},k}^{(j)}
\cdot
q_{\mathrm{exist},k}^{(j)},
$$

where $m_k^{(j)} \in \{0,1\}$ is the availability mask, $q_{\mathrm{cov},k}^{(j)}$ is a covariance-quality score, $q_{\mathrm{link},k}^{(j)}$ is a realized link-quality score, and $q_{\mathrm{exist},k}^{(j)}$ is an existence-confidence score. The normalized weight is

$$
\bar{\omega}_{k,s}^{(j)} =
\frac{\tilde{\omega}_{k,s}^{(j)}}{\sum_{u \in \mathcal{N}_s}\tilde{\omega}_{k,s}^{(u)}}.
$$

This factorization is intentionally narrow. It keeps only the three factors with the strongest current evidence:

- posterior concentration through covariance,
- communication reliability through delivered-versus-dropped packets,
- existence decisiveness through Bernoulli existence probabilities.

Other terms, such as NIS penalties, history scores, freshness scores, or ambiguity-aware corrections, remain optional extensions rather than components of the claimed core method.

### 3. Covariance And Link-Quality Terms

The covariance term is designed to reward concentrated local posteriors. For node $j$, let $T_{k,i}^{(j)}$ be the moment-matched covariance of the $i$th Bernoulli component. The code uses

$$
q_{\mathrm{cov},k}^{(j)}
=
\frac{1}{\epsilon + \frac{1}{M_k}\sum_{i=1}^{M_k}\operatorname{tr}\!\left(T_{k,i}^{(j)}\right)},
$$

so that a node with smaller posterior spread receives a larger score.

The link-quality term is derived from realized communication outcomes rather than nominal topology. Let $d_k^{(j)}$ and $\ell_k^{(j)}$ denote the counts of delivered and dropped measurements, respectively. The code uses

$$
q_{\mathrm{link},k}^{(j)}
=
\frac{d_k^{(j)}}{d_k^{(j)} + \ell_k^{(j)}},
$$

which directly reflects how reliably that node's information is reaching its neighbors. This term is particularly important under the tiered heterogeneous packet-loss model, where nodes with similar sensing models can still have systematically different effective utility because their measurements are delivered with different reliability.

### 4. Existence-Confidence Weighting

The main additional method contribution beyond covariance and link quality is the existence-confidence term. Covariance measures spatial concentration, but it does not indicate whether a node is decisive about the existence of a target. Likewise, link statistics indicate whether packets were delivered, but they do not indicate whether the delivered posterior is reliable in cardinality terms.

To capture this missing dimension, the method computes a certainty score from Bernoulli existence probabilities. For the $i$th Bernoulli component at node $j$,

$$
c_{k,i}^{(j)} = \left|2r_{k,i}^{(j)} - 1\right|.
$$

This quantity is large when $r_{k,i}^{(j)}$ is close to $0$ or $1$, and small when $r_{k,i}^{(j)}$ is close to the ambiguous region around $0.5$. The per-node weighted existence confidence is then defined as

$$
\bar{c}_{k}^{(j)} =
\frac{\sum_{i=1}^{M_k} r_{k,i}^{(j)} c_{k,i}^{(j)}}
{\epsilon + \sum_{i=1}^{M_k} r_{k,i}^{(j)}}.
$$

The final existence-confidence score is mapped into a bounded positive weight factor:

$$
q_{\mathrm{exist},k}^{(j)}
=
\lambda_{\min} + (1-\lambda_{\min})\left(\bar{c}_{k}^{(j)}\right)^{p_e},
$$

where $\lambda_{\min} \in (0,1]$ and $p_e > 0$ are tunable parameters. This design gives more influence to nodes whose posteriors are decisive about object existence while preventing low-confidence nodes from being assigned zero weight. Empirically, this term is the key factor that improves cardinality-related consensus beyond a covariance-and-link-only baseline.

### 5. Decoupled Spatial And Existence Scores

The current best implementation does not force the same adaptive score to govern spatial fusion and existence fusion. Instead, it constructs branch-specific dedicated scores and blends them with the shared backbone. The spatial-dedicated score is

$$
\tilde{\omega}_{k,s,\mathrm{sp}}^{(j)} =
m_k^{(j)}
\cdot
\left(q_{\mathrm{cov},k}^{(j)}\right)^{\alpha_x}
\cdot
\left(q_{\mathrm{link},k}^{(j)}\right)^{\beta_x},
$$

while the existence-dedicated score is

$$
\tilde{\omega}_{k,s,\mathrm{ex}}^{(j)} =
m_k^{(j)}
\cdot
\left(q_{\mathrm{link},k}^{(j)}\right)^{\beta_r}
\cdot
\left(q_{\mathrm{exist},k}^{(j)}\right)^{\alpha_r}.
$$

The final branch scores are obtained through geometric interpolation:

$$
\tilde{\omega}_{k,s}^{x,(j)}
=
\left(\tilde{\omega}_{k,s}^{(j)}\right)^{1-\eta_x}
\left(\tilde{\omega}_{k,s,\mathrm{sp}}^{(j)}\right)^{\eta_x},
$$

$$
\tilde{\omega}_{k,s}^{r,(j)}
=
\left(\tilde{\omega}_{k,s}^{(j)}\right)^{1-\eta_r}
\left(\tilde{\omega}_{k,s,\mathrm{ex}}^{(j)}\right)^{\eta_r},
$$

with $0 \le \eta_x,\eta_r \le 1$. In implementation form, these products are realized multiplicatively as

$$
\tilde{\omega}_{k,s}^{x,(j)}
=
\left(\tilde{\omega}_{k,s}^{(j)}\right)^{1-\eta_x}
\cdot
\left(\tilde{\omega}_{k,s,\mathrm{sp}}^{(j)}\right)^{\eta_x},
\qquad
\tilde{\omega}_{k,s}^{r,(j)}
=
\left(\tilde{\omega}_{k,s}^{(j)}\right)^{1-\eta_r}
\cdot
\left(\tilde{\omega}_{k,s,\mathrm{ex}}^{(j)}\right)^{\eta_r}.
$$

The instantaneous branch-normalized weights are then

$$
\bar{\omega}_{k,s}^{x,(j)}
=
\frac{\tilde{\omega}_{k,s}^{x,(j)}}{\sum_{u \in \mathcal{N}_s}\tilde{\omega}_{k,s}^{x,(u)}},
\qquad
\bar{\omega}_{k,s}^{r,(j)}
=
\frac{\tilde{\omega}_{k,s}^{r,(j)}}{\sum_{u \in \mathcal{N}_s}\tilde{\omega}_{k,s}^{r,(u)}}.
$$

The point of this decoupling is not to create two unrelated methods. It is to acknowledge that spatial consensus and existence consensus respond differently to weight perturbations. It also determines how the method uses the information-geometric FID-FIA idea of Cao and Zhao \cite{CaoZhao2025InfoGeometryFusion}: their Fisher-information accumulation score is useful for heterogeneous sensing informativeness and is empirically strong for cardinality decisions, but a scalar FID-FIA weight can trade away spatial RMSE. In the log-utility view developed in the appendix, scalar FID-FIA is a constrained case that forces the same Fisher-separability utility into both the Gaussian spatial barycenter and the Bernoulli existence pool. The present method relaxes that constraint: the communication-aware covariance/link-quality branch remains in charge of spatial fusion, while FID-FIA is reserved for the existence branch.

### 6. FID-FIA Existence Refinement

The final method adds an information-geometric refinement only to the existence branch. Each local Bernoulli component is moment-projected, existence-weighted target pairs are formed, and a pairwise Fisher-information-distance accumulation score is computed using the sensor's detection probability and measurement covariance. The score is normalized over the local neighborhood and mapped to a bounded modulation factor:

$$
q_{\mathrm{fid},k,s}^{(j)}
=
\rho_{\min}
+ (1-\rho_{\min})\left(\hat{a}_{k,s}^{(j)}\right)^{p_f}.
$$

Only the existence score is modified:

$$
\tilde{\omega}_{k,s}^{r,(j)}
\leftarrow
\tilde{\omega}_{k,s}^{r,(j)}
\left(q_{\mathrm{fid},k,s}^{(j)}\right)^{\gamma_f}.
$$

The spatial score is left unchanged. For positive $q_{\mathrm{fid},k,s}^{(j)}$, this is equivalent to adding $\gamma_f \log q_{\mathrm{fid},k,s}^{(j)}$ to the existence-branch utility and adding nothing to the spatial utility; the zero-floor setting in the main experiment is the boundary case that permits a zero instantaneous existence weight. This is the key difference from using Cao-Zhao FID-FIA as a whole-posterior scalar baseline: the paper borrows its information-geometric cardinality strength, but branch decoupling prevents that signal from overriding the spatial/RMSE-oriented weight path.

### 7. Weak Structure-Aware Refinement

After branch decoupling, the method applies a weak graph-aware correction. Let $\xi_{k,s}^{x,(j)}$ and $\xi_{k,s}^{r,(j)}$ denote spatial and existence structure priors, respectively. The code constructs these priors from two ingredients:

- neighborhood-overlap similarity in the local communication graph,
- communication-reliability information derived from link-loss statistics.

The branch scores are then modulated as

$$
\tilde{\omega}_{k,s}^{x,(j)} \leftarrow \tilde{\omega}_{k,s}^{x,(j)}
\cdot
\left(\xi_{k,s}^{x,(j)}\right)^{\gamma_x},
\qquad
\tilde{\omega}_{k,s}^{r,(j)} \leftarrow \tilde{\omega}_{k,s}^{r,(j)}
\cdot
\left(\xi_{k,s}^{r,(j)}\right)^{\gamma_r},
$$

where $\gamma_x,\gamma_r \ge 0$ are structure-strength parameters. The current best configuration intentionally keeps $\gamma_r \ll \gamma_x$. In other words, topology is not allowed to dominate the core weight assignment, and the existence branch receives only a very weak structural perturbation. This design reflects the empirical fact that stronger topology-driven corrections can easily damage cardinality agreement even when they help spatial consensus.

The implementation also contains an optional posterior-structure-consistency mode that derives structure scores from pairwise disagreement among neighboring posteriors. However, the present best-performing main-line configuration keeps this option off and uses the weaker static local-structure prior instead.

### 8. Temporal Stabilization And Minimum-Weight Safeguard

The raw adaptive weights can fluctuate sharply over time, especially when packet delivery changes abruptly. To improve stability, the normalized weights are smoothed by an exponential moving average:

$$
\omega_{k,s}^{x,(j)}
=
\operatorname{Normalize}\!\left(
\alpha_x \omega_{k-1,s}^{x,(j)} + (1-\alpha_x)\bar{\omega}_{k,s}^{x,(j)}
\right),
$$

$$
\omega_{k,s}^{r,(j)}
=
\operatorname{Normalize}\!\left(
\alpha_r \omega_{k-1,s}^{r,(j)} + (1-\alpha_r)\bar{\omega}_{k,s}^{r,(j)}
\right),
$$

where $\bar{\omega}_{k,s}^{x,(j)}$ and $\bar{\omega}_{k,s}^{r,(j)}$ denote the branch-normalized instantaneous weights and $\alpha_x,\alpha_r \in [0,1)$ are smoothing factors. A minimum-weight safeguard is then enforced on available neighbors before renormalization so that no active node is completely discarded by transient score fluctuations.

This stabilization step is not presented as a novelty claim by itself. Its purpose is pragmatic: preserve the intended quality ordering while preventing unstable weight collapse in a time-varying communication setting.

### 9. Optional Consistency And Extension Modules

The implementation retains several optional extensions:

- `innovationPenalty`, an NIS-based consistency penalty,
- `historyScore`, a temporal-stability term,
- `freshnessScore`, a recency-oriented term,
- `cardinalityConsensusScore` and ambiguity-related extensions,
- posterior-structure-consistency scoring.

These modules are intentionally not part of the claimed core method. The main reason is not that they are useless in every setting, but that their current evidence is weaker, more coupled, or less stable than the evidence for the main three-factor backbone plus the retained branch-specific refinements.

Among them, the most relevant is the NIS-based consistency term. Innovation consistency is still useful, but it is treated as a penalty rather than an additional monotonic reward. This choice avoids double counting, because innovation-based scores are structurally coupled with covariance through the innovation covariance. Accordingly, the current paper positions NIS as a secondary consistency module and leaves its detailed analysis to ablation or appendix discussion rather than the main method claim.

## Method Positioning Notes

- Present the method as an adaptive weight-allocation scheme for distributed GA-LMB fusion, not as a new RFS filter family.
- Keep the main narrative on the covariance, link-quality, and existence-confidence backbone, followed by branch decoupling and an FID-FIA cue restricted to the existence-weight path.
- Treat EMA smoothing as a stabilization device, not a headline contribution.
- Treat NIS, history, freshness, and stronger structure priors as optional modules or appendix material.

## Citation Keys Used Here

- `Battistelli2014KLA`
- `Hlinka2014ICI`
- `Vo2014LRFS`
- `Reuter2014LMB`
- `Vo2019MSGLMB`
- `BarShalom2001Estimation`

## Source Files

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/gaLmbTrackMerging.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `multisensorLmb/runDistributedLmbFilter.m`
- `docs/ADAPTIVE_FUSION_WEIGHTS_CN.md`
- `docs/COMMUNICATION_TIERED_DROP_UPDATE_CN.md`
- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`
