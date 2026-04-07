# Problem Formulation

## Paper-Ready Problem-Formulation Draft

Consider a distributed sensor network with sensor index set $\mathcal{V}=\{1,\ldots,S\}$ operating over discrete time $k \in \{1,\ldots,T\}$. The multi-object state at time $k$ is modeled as a labeled random finite set $X_k$, and each sensor $s \in \mathcal{V}$ acquires a local measurement set $Z_k^{(s)}$. Following the standard labeled-RFS formulation, each node maintains an LMB approximation to the local multi-object posterior, whose Bernoulli components jointly encode target existence, kinematic state, and identity \cite{Vo2014LRFS,Reuter2014LMB,Vo2019MSGLMB}. The objective of the distributed fusion layer is to combine neighboring local posteriors into a consensus-quality posterior without assuming that cross-correlations among nodes are known.

### 1. Dynamic And Measurement Model

Let $\mathcal{N}_s \subseteq \mathcal{V}$ denote the communication neighborhood of sensor $s$, including the node itself. In the present implementation, $\mathcal{N}_s$ is induced by a communication graph derived from sensor geometry and communication range, and each node performs local fusion only over the posteriors available inside this neighborhood. The single-sensor motion and measurement models follow the standard multi-sensor LMB setting, so after local prediction and data association, node $j \in \mathcal{N}_s$ produces a measurement-updated LMB posterior

$$
\pi_{k}^{(j)} = \left\{ \left(r_{k,i}^{(j)}, p_{k,i}^{(j)}(\cdot,\ell_i)\right) \right\}_{i=1}^{M_k},
$$

where $r_{k,i}^{(j)} \in [0,1]$ is the Bernoulli existence probability and $p_{k,i}^{(j)}$ is the associated spatial density for label $\ell_i$. In the code, each spatial density is represented by a Gaussian mixture and then moment-matched when GA fusion is performed.

### 2. Communication-Constrained Observation Delivery

The fusion problem is made harder by communication constraints. Before local measurements are used by a neighboring fusion node, they pass through a communication layer with three possible effects:

1. a global or per-step bandwidth budget that limits how many measurements can be delivered,
2. a stochastic packet-drop model on each sensor link,
3. optional node-level outages.

Accordingly, the posterior that reaches a neighbor is influenced not only by the underlying sensing quality, but also by whether measurements were actually delivered. Let $m_k^{(j)} \in \{0,1\}$ denote the availability indicator of node $j$ at time $k$, and let $\rho_k^{(j)} \in [0,1]$ denote the realized link-quality statistic derived from delivered-versus-dropped packets. In the tiered packet-loss experiments used in this paper, the link model is heterogeneous across sensors, so $\rho_k^{(j)}$ varies persistently across nodes even when the historical average drop rate is preserved.

### 3. Distributed KLA Or GA Fusion Objective

Because the cross-correlation structure among neighboring LMB posteriors is unknown, exact Bayesian fusion is generally unavailable. We therefore adopt a conservative Kullback-Leibler-average or geometric-average fusion rule. At node $s$, the target fused density over the neighborhood $\mathcal{N}_s$ is written abstractly as

$$
\bar{\pi}_k^{(s)}(X) \propto \prod_{j \in \mathcal{N}_s} \left(\pi_k^{(j)}(X)\right)^{\omega_{k,s}^{(j)}},
\qquad
\sum_{j \in \mathcal{N}_s} \omega_{k,s}^{(j)} = 1,
\qquad
\omega_{k,s}^{(j)} \ge 0,
$$

where $\omega_{k,s}^{(j)}$ is the fusion weight assigned by node $s$ to neighbor $j$ at time $k$ \cite{Battistelli2014KLA,Hlinka2014ICI}. With fixed-weight fusion, $\omega_{k,s}^{(j)}$ is often chosen as a uniform or Metropolis weight. The central problem studied here is instead how to choose these weights adaptively so that they reflect both posterior quality and communication quality.

In the present GA-LMB implementation, the spatial and existence parts of each Bernoulli component are fused separately after moment matching. For the $i$th Bernoulli component, the Gaussian spatial density is fused by geometric averaging in canonical form, while the Bernoulli existence probability is fused by the corresponding weighted product rule. This separation makes it natural to ask whether the same scalar weight should govern both spatial fusion and existence fusion.

### 4. Adaptive Weight-Allocation Objective

The adaptive weighting design in this paper starts from a shared quality backbone. For node $s$ and neighbor $j \in \mathcal{N}_s$, define the unnormalized score

$$
\tilde{\omega}_{k,s}^{(j)}
=
m_k^{(j)}
\cdot
q_{\mathrm{cov},k}^{(j)}
\cdot
q_{\mathrm{link},k}^{(j)}
\cdot
q_{\mathrm{exist},k}^{(j)},
$$

where:

- $m_k^{(j)}$ is the availability mask,
- $q_{\mathrm{cov},k}^{(j)}$ is a covariance-quality score derived from the inverse of the moment-matched posterior covariance trace,
- $q_{\mathrm{link},k}^{(j)}$ is a realized link-quality score computed from delivered and dropped packets,
- $q_{\mathrm{exist},k}^{(j)}$ is an existence-confidence score derived from the decisiveness of Bernoulli existence probabilities.

The normalized fusion weight is then obtained as

$$
\omega_{k,s}^{(j)} =
\frac{\tilde{\omega}_{k,s}^{(j)}}{\sum_{u \in \mathcal{N}_s}\tilde{\omega}_{k,s}^{(u)}}.
$$

In the code, the covariance score favors posteriors whose moment-matched covariance is more concentrated, the link score favors nodes whose measurements were more reliably delivered, and the existence-confidence score favors nodes whose Bernoulli existence probabilities are further away from the ambiguous region around $0.5$. After normalization, the weights are additionally regularized by exponential moving average smoothing and a minimum-weight safeguard to prevent abrupt collapse of one node's contribution.

### 5. Decoupled Spatial And Existence Weighting

The current best implementation does not force the spatial and existence branches to share exactly the same dynamics. Instead, it starts from the shared backbone above and then forms two branch-specific scores:

$$
\tilde{\omega}_{k,s}^{x,(j)}
=
\operatorname{blend}\!\left(\tilde{\omega}_{k,s}^{(j)}, \tilde{\omega}_{k,s,\mathrm{sp}}^{(j)}, \eta_x\right),
$$

$$
\tilde{\omega}_{k,s}^{r,(j)}
=
\operatorname{blend}\!\left(\tilde{\omega}_{k,s}^{(j)}, \tilde{\omega}_{k,s,\mathrm{ex}}^{(j)}, \eta_r\right),
$$

where $\tilde{\omega}_{k,s,\mathrm{sp}}^{(j)}$ places more emphasis on covariance and spatial reliability, and $\tilde{\omega}_{k,s,\mathrm{ex}}^{(j)}$ places more emphasis on existence confidence. This decoupling is motivated by the empirical fact that spatial consistency and existence consistency are not equally sensitive to weight perturbations: a weight adjustment that helps position consensus may still damage cardinality agreement if applied too strongly to Bernoulli existence fusion.

### 6. Weak Structure-Aware Refinement

After branch decoupling, the method applies only a weak graph-aware correction:

$$
\tilde{\omega}_{k,s}^{x,(j)} \leftarrow \tilde{\omega}_{k,s}^{x,(j)}
\cdot
\left(\xi_{k,s}^{x,(j)}\right)^{\gamma_x},
\qquad
\tilde{\omega}_{k,s}^{r,(j)} \leftarrow \tilde{\omega}_{k,s}^{r,(j)}
\cdot
\left(\xi_{k,s}^{r,(j)}\right)^{\gamma_r},
$$

where $\xi_{k,s}^{x,(j)}$ and $\xi_{k,s}^{r,(j)}$ are local structure priors derived from neighborhood overlap and communication reliability, and the exponents satisfy $\gamma_r \ll \gamma_x$ in the current best configuration. In other words, topology is not treated as the primary source of fusion weights; it acts only as a mild prior layered on top of posterior-quality and communication-quality signals, especially on the existence branch.

### 7. Evaluation Target

The aim of the above formulation is not only to improve each node's local tracking output, but also to improve agreement among nodes. The experiments therefore evaluate two complementary aspects:

- local tracking quality, measured by local E-OSPA and local RMSE,
- inter-sensor consensus quality, measured by consensus OSPA, consensus RMSE, and consensus cardinality disagreement.

This separation is important for the present problem. In a distributed network with unknown cross-correlations and communication losses, an adaptive fusion rule can be valuable even when local metrics improve only mildly, provided that it substantially improves the consistency of the fused multi-sensor picture across nodes.

## Notation And Framing Notes

- Use `sensor` or `node` consistently; avoid switching terminology inside the same paragraph.
- Keep the problem formulation centered on unknown cross-correlation, communication-constrained delivery, and adaptive consensus-quality fusion.
- Introduce topology only after the shared weight backbone, so structure-aware refinement reads as a weak extension rather than the main method.

## Citation Keys Used Here

- `Vo2014LRFS`
- `Reuter2014LMB`
- `Vo2019MSGLMB`
- `Battistelli2014KLA`
- `Hlinka2014ICI`
