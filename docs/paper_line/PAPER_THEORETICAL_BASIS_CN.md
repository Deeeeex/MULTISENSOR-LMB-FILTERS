# 这篇 Paper 的理论依据整理

本文档尝试用相对通俗的方式，把这篇 paper 的理论背景、方法设计的理论依据，以及“哪些部分是严格可解释的，哪些部分只是合理建模选择”完整讲清楚。

这份文档的目标不是替代论文附录，而是把附录里的理论内容重新组织成一条更容易读懂的主线。

可配合阅读的原始材料：

- [appendix_c.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/sections/appendix_c.tex)
- [working_adaptive_weight_derivation.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/notes/working_adaptive_weight_derivation.tex)
- [05_method_adaptive_kla.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/05_method_adaptive_kla.md)
- [04_problem_formulation.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/04_problem_formulation.md)

## 1. 先说结论：这篇 paper 的理论核心是什么

这篇 paper 的理论核心不是“我们发明了一种全新的多目标滤波器”，而是下面这句话：

**在 unknown cross-correlation 的分布式多目标跟踪中，KLA/GA 融合本身仍然是合理的；真正需要改进的是 fusion weight allocation。我们的方法可以被解释为：在 KLA 融合框架内，用一个有理论解释的自适应权重分配机制，去决定每个邻居后验该被信任多少。**

换句话说，理论上这篇 paper 做的是两件事：

1. 保留已有的 KLA-based distributed LMB fusion 框架。
2. 给当前的 adaptive weights 找到一个尽可能严谨的 variational / optimization interpretation。

## 2. 理论背景一：为什么最好先从 RFS 讲起

如果只从传统状态空间模型出发，单目标跟踪通常把状态写成一个固定维度向量，比如

$$
x_k \in \mathbb{R}^d.
$$

但多目标跟踪的困难在于：一个时刻不仅每个目标的状态会变，**目标数量本身也会变**。这时如果还坚持用固定维向量描述系统状态，建模会非常别扭，因为你首先就不知道当前维度应该是多少。

RFS 的核心思想就是：把“多目标状态”直接看成一个随机集合。也就是说，在时刻 $k$，系统状态更自然地写成

$$
X_k=\{x_{k,1},\dots,x_{k,n_k}\},
$$

这里 $n_k$ 本身也是随机变量。这样一来，RFS 能同时表达两件事：

- 当前有多少个目标；
- 每个目标的状态是什么。

所以，从理论上讲，RFS 不是一个额外装饰，而是把多目标问题“定义对”的第一步。它解决的是：**多目标状态本来就不是固定维对象，而应该是随机有限集合。**

## 3. 理论背景二：为什么进一步要用 labeled RFS 和 LMB

只用无标签 RFS，还只能表达“现在有几个目标、它们在哪”；但在实际跟踪和多节点融合里，我们还希望区分“哪个目标是谁”，也就是要保留 track identity。于是更自然的表示就变成了 labeled RFS：

$$
X_k=\{(x_{k,1},\ell_1),\dots,(x_{k,n_k},\ell_{n_k})\}.
$$

这里每个状态都带一个标签 $\ell$。这样做的好处是：

- 它可以同时表示“有几个目标”。
- 它可以表示“每个目标的位置 / 速度等状态”。
- 它还可以保留“目标身份标签”，避免不同节点融合后目标身份混乱。

在 labeled RFS 家族里，LMB 是一种兼顾表达能力和计算可 tractable 的形式。它把每个目标写成一个 Bernoulli 分量，所以很自然地同时保留：

- 该目标存在的概率；
- 该目标的空间状态密度；
- 该目标的标签。

在当前 paper 里，每个节点维护的是一个 LMB 形式的局部后验：

$$
\pi_k^{(j)}=\left\{\left(r_{k,i}^{(j)},\,p_{k,i}^{(j)}(\cdot,\ell_i)\right)\right\}_{i=1}^{M_k}.
$$

这里：

- $j$ 是节点编号。
- $i$ 是第 $i$ 个 Bernoulli 分量。
- $r_{k,i}^{(j)} \in [0,1]$ 是该目标存在的概率。
- $p_{k,i}^{(j)}(\cdot,\ell_i)$ 是该目标的空间状态密度。

这个表示形式非常关键，因为它天然把每个目标拆成两部分：

- `existence`：目标存不存在。
- `spatial density`：如果存在，它的位置分布是什么。

也正因为 LMB 本身就是“存在性 + 空间密度”的组合，后面我们把 spatial branch 和 existence branch 分开加权，理论上才说得通。

## 4. 理论背景三：为什么要用 KLA / geometric-average fusion

在分布式融合里，一个最大的问题是：不同节点的局部后验之间通常存在未知相关性。也就是说，我们知道它们不是相互独立的，但又不知道相关性到底有多强。

这时，如果直接做“精确贝叶斯融合”，往往会重复计算信息，导致过度自信。KLA 或 geometric-average fusion 的理论价值就在这里：它给出了一个**在未知交叉相关下更保守的融合规则**。

KLA 融合的形式是：

$$
\bar{\pi}_k^{(s)}(X) \propto \prod_{j \in \mathcal{N}_s}\left(\pi_k^{(j)}(X)\right)^{\omega_{k,s}^{(j)}},
\qquad
\sum_{j \in \mathcal{N}_s}\omega_{k,s}^{(j)}=1,\quad \omega_{k,s}^{(j)}\ge 0.
$$

这表示在节点 $s$ 处，把邻域内各节点的后验按权重做几何平均。

更重要的是，这个公式不是拍脑袋来的。它有一个标准的变分解释：

$$
\bar{\pi}
=
\arg\min_{\pi}
\sum_j \omega_j\, D_{\mathrm{KL}}(\pi \,\|\, \pi_j).
$$

它的含义是：

- 在所有候选融合后验里，
- 选一个对所有局部后验“加权平均意义下最接近”的后验，
- 距离度量使用的是 reverse KL。

所以，**KLA 本身已经是有理论基础的**。这篇 paper 并不是要重新证明 KLA 对不对，而是要回答另一个更具体的问题：

**这些权重 $\omega_j$ 到底应该怎么选？**

## 5. 为什么固定权重在这个问题里不够

如果所有节点都一样、通信也一样，那么固定权重、均匀权重、Metropolis 权重都可以作为一个简单基线。

但当前 paper 的问题设置不是这样。这里至少有两层异质性：

1. 节点的局部后验质量不同。
2. 节点的信息送到邻居时，通信质量也不同。

因此，一个节点“值不值得信”至少取决于两件事：

- 它本地估计是否好。
- 它的信息是否真的可靠到达了邻居。

而固定权重只编码了拓扑结构，基本没有编码这两件事。这就是为什么 paper 的理论重点会落在 `adaptive weight allocation`，而不是落在“换一个滤波器框架”上。

## 6. 方法的第一层理论依据：三因子权重 backbone

当前方法的共享 backbone 是：

$$
\tilde{\omega}_{k,s}^{(j)}
=
m_k^{(j)}
\cdot
q_{\mathrm{cov},k}^{(j)}
\cdot
q_{\mathrm{link},k}^{(j)}
\cdot
q_{\mathrm{exist},k}^{(j)}.
$$

归一化后得到：

$$
\omega_{k,s}^{(j)}
=
\frac{\tilde{\omega}_{k,s}^{(j)}}{\sum_{u\in\mathcal{N}_s}\tilde{\omega}_{k,s}^{(u)}}.
$$

这个式子可以直接理解成一句话：

**一个邻居该拿多少融合权重，取决于它是否可用、后验是否集中、链路是否可靠、以及它在目标存在性上是否足够果断。**

下面分别看三个因子的理论动机。

### 6.1 covariance term：后验越集中，说明空间信息越可靠

代码里的 covariance score 形式是：

$$
q_{\mathrm{cov},k}^{(j)}
=
\frac{1}{\epsilon + \frac{1}{M_k}\sum_{i=1}^{M_k}\operatorname{tr}\!\left(T_{k,i}^{(j)}\right)}.
$$

这里 $T_{k,i}^{(j)}$ 是第 $j$ 个节点第 $i$ 个目标分量的 moment-matched covariance。

直觉上很好理解：

- covariance trace 越小，说明分布越集中；
- 分布越集中，说明这个节点对目标位置越“有把握”；
- 因此权重应该更大。

这个形式虽然是工程上简化后的 inverse-trace score，但它不是完全随意的。如果在各向同性近似下

$$
T_{k,i}^{(j)}=\sigma_j^2 I,
$$

那么

$$
q_{\mathrm{cov},k}^{(j)}=\frac{1}{\epsilon + d\sigma_j^2}
$$

和下面这些量的排序是一致的：

- 方差越小；
- $\log\det T$ 越小；
- 信息矩阵行列式越大；
- 精度越高。

所以，`inverse trace` 可以看成是一个计算更简单、但和标准信息量排序基本一致的 proxy。

### 6.2 link-quality term：信息有没有可靠送达，本身就是“质量”的一部分

链路质量分数写成：

$$
q_{\mathrm{link},k}^{(j)}
=
\frac{d_k^{(j)}}{d_k^{(j)}+\ell_k^{(j)}},
$$

其中：

- $d_k^{(j)}$ 是送达的测量数，
- $\ell_k^{(j)}$ 是丢失的测量数。

它表达的是 realized communication quality，而不是名义拓扑。

这一步的理论意义在于：  
在通信约束下，一个节点是否值得信，不仅取决于它“本来能不能测好”，还取决于它的信息“实际上有没有可靠送出来”。

这也是为什么 paper 的主线强调 `communication-aware`。理论上，链路质量不是额外小修小补，而是这个问题定义里必须出现的变量。

### 6.3 existence-confidence term：covariance 不能表达“是否真的相信目标存在”

这是当前方法里最值得强调的第三个理论动机。

只看 covariance 会有一个问题：  
某个节点的位置分布也许很集中，但它对“目标到底存不存在”可能依然很犹豫。

所以，paper 用 Bernoulli existence probability 构造一个 decisiveness score。先定义单个分量的 certainty：

$$
c_{k,i}^{(j)}=\left|2r_{k,i}^{(j)}-1\right|.
$$

它的含义是：

- 当 $r$ 接近 $0$ 或 $1$ 时，$c$ 大，表示这个节点很果断；
- 当 $r$ 接近 $0.5$ 时，$c$ 小，表示这个节点在“有 / 无目标”上很模糊。

然后把一个节点所有 Bernoulli 分量汇总成：

$$
\bar c_k^{(j)}
=
\frac{\sum_i r_{k,i}^{(j)}\,|2r_{k,i}^{(j)}-1|}
{\epsilon+\sum_i r_{k,i}^{(j)}}.
$$

最后再映射到一个正的权重因子：

$$
q_{\mathrm{exist},k}^{(j)}
=
\lambda_{\min} + (1-\lambda_{\min})\left(\bar c_k^{(j)}\right)^{p_e}.
$$

它的理论动机可以概括成一句话：

**covariance 描述的是“位置是否集中”，existence confidence 描述的是“对目标存在性是否果断”，这两个维度不是一回事。**

这也是为什么实验里 `existence confidence` 往往主要改善 cardinality-related consensus，而不是简单重复 covariance 已经提供的信息。

## 7. 这个乘积权重为什么不是纯经验规则

很多人看到

$$
\tilde{\omega}_j = q_{\mathrm{cov},j}q_{\mathrm{link},j}q_{\mathrm{exist},j}
$$

第一反应会是：“这不就是把几个 score 相乘吗？”

如果只停在这个层面，确实容易显得经验化。但 Appendix C 给出了更强的解释。

先定义 log-utility：

$$
u_{0,j}
=
\log q_{\mathrm{cov},k}^{(j)}
+
\log q_{\mathrm{link},k}^{(j)}
+
\log q_{\mathrm{exist},k}^{(j)}.
$$

再考虑 simplex 上的优化问题：

$$
\max_{w \in \Delta_{k,s}}
\left\{
\sum_j w_j u_{0,j} + \tau H(w)
\right\},
\qquad
H(w)=-\sum_j w_j\log w_j.
$$

这里 $\Delta_{k,s}$ 是权重 simplex，也就是：

$$
\Delta_{k,s}
=
\left\{
w:\; w_j\ge 0,\ \sum_j w_j=1
\right\}.
$$

这个优化问题的解是：

$$
w_j^\star
=
\frac{\exp(u_{0,j}/\tau)}{\sum_u \exp(u_{0,u}/\tau)}
=
\frac{\left(q_{\mathrm{cov},j}q_{\mathrm{link},j}q_{\mathrm{exist},j}\right)^{1/\tau}}
{\sum_u \left(q_{\mathrm{cov},u}q_{\mathrm{link},u}q_{\mathrm{exist},u}\right)^{1/\tau}}.
$$

当 $\tau=1$ 时，恰好就是当前实现里的归一化乘积形式。

这说明：

- 当前乘积打分不是“毫无来源的经验组合”；
- 它可以被解释为一个**带熵正则的 trust allocation 问题**；
- 每个节点的 log score 是它的 utility；
- 熵项 $H(w)$ 防止所有权重塌缩到单一节点。

通俗说，就是：

**我们不是随便相乘，而是在问：如果要在所有邻居之间分配一份有限信任，并且既想偏向高质量节点，又不想让权重过于极端，那么最自然的解是什么？答案就是这种 softmax / normalized-product 形式。**

## 8. 为什么 spatial branch 和 existence branch 可以分开加权

这一步不是纯经验操作，它和 LMB 的代数结构直接相关。

### 8.1 spatial branch 的理论依据

在当前实现里，空间密度先做 Gaussian moment matching：

$$
p_{k,i}^{(j)}(x)\approx \mathcal N\!\left(x;\nu_{k,i}^{(j)},T_{k,i}^{(j)}\right).
$$

之后 spatial fusion 的 canonical form 是：

$$
K_i=\sum_j w_{x,j}T_{i,j}^{-1},
\qquad
h_i=\sum_j w_{x,j}T_{i,j}^{-1}\nu_{i,j},
$$

$$
\Sigma_i=K_i^{-1},
\qquad
\mu_i=\Sigma_i h_i.
$$

这个结果并不只是“看起来像个合理平均”，而是有精确解释：

$$
\mathcal N(\mu_i,\Sigma_i)
=
\arg\min_{q\in\mathcal G}
\sum_j w_{x,j}\,
D_{\mathrm{KL}}\!\left(q \,\|\, \mathcal N(\nu_{i,j},T_{i,j})\right).
$$

也就是说，在 Gaussian family 内，它就是加权 KLA barycenter。

### 8.2 existence branch 的理论依据

existence fusion 的实现形式是：

$$
r_i
=
\frac{
\eta_i(w_x)\prod_j r_{i,j}^{w_{r,j}}
}{
\eta_i(w_x)\prod_j r_{i,j}^{w_{r,j}}
+
\prod_j (1-r_{i,j})^{w_{r,j}}
}.
$$

其中 $\eta_i(w_x)$ 是 spatial branch 诱导出来的 overlap normalization term。

这个公式还可以改写成更容易理解的 log-odds 形式：

$$
\log\frac{r_i}{1-r_i}
=
\log \eta_i(w_x)
+
\sum_j w_{r,j}\log\frac{r_{i,j}}{1-r_{i,j}}.
$$

这句话的含义非常清楚：

- 存在概率的融合，本质上是在做 weighted logit pooling；
- 只是这个 pooling 还额外受到 spatial overlap 的修正。

所以，existence branch 本身就是一套与 spatial branch 不完全相同的代数结构。  
既然两个分支的融合公式不同，那么它们使用完全相同的权重，并不是理论上必须的。

## 9. 为什么“branch decoupling”在理论上是成立的

更进一步，Appendix C 还给出了一个很关键的桥梁：  
Bernoulli-RFS 的 KL divergence，本来就会拆成存在性项和空间项。

设一个 Bernoulli RFS 密度写成 $\beta=(r,p)$，另一个写成 $\beta_j=(r_j,p_j)$，那么：

$$
D_{\mathrm{KL}}(\beta \,\|\, \beta_j)
=
d_{\mathrm{Ber}}(r\,\|\,r_j)
+
r\,D_{\mathrm{KL}}(p\,\|\,p_j),
$$

其中

$$
d_{\mathrm{Ber}}(r\,\|\,r_j)
=
(1-r)\log\frac{1-r}{1-r_j}
+
r\log\frac{r}{r_j}.
$$

这个分解非常重要，因为它说明：

- 存在性误差本来就是一项；
- 空间密度误差是另一项；
- 而且空间项前面还乘了一个 existence level $r$。

于是，可以构造一个 per-Bernoulli surrogate objective：

$$
\mathcal J_i(r,p;w_x,w_r)
=
\sum_j w_{r,j}\, d_{\mathrm{Ber}}(r\,\|\,r_{i,j})
+
r\sum_j w_{x,j}\, D_{\mathrm{KL}}(p\,\|\,p_{i,j}).
$$

这个目标函数的最优解，恰好给出：

$$
p_i^\star(x)
=
\frac{1}{\eta_i(w_x)}
\prod_j p_{i,j}(x)^{w_{x,j}},
$$

以及

$$
\log\frac{r_i^\star}{1-r_i^\star}
=
\log \eta_i(w_x)
+
\sum_j w_{r,j}\log\frac{r_{i,j}}{1-r_{i,j}}.
$$

也就是说：

- spatial geometric fusion 不是随意写的；
- existence logit pooling 也不是随意写的；
- 两者共同构成了一个 decoupled surrogate objective 的精确最优解。

这就是 branch-decoupled 理论上最重要的依据。

当然，这里有边界条件，paper 也必须诚实说明：

- 这个推导依赖 label alignment；
- 依赖 moment-compatible Gaussian projection；
- 它不是在说“所有 unconstrained LMB fusion 问题的唯一最优解都长这样”。

但对于当前实现而言，这个理论支撑已经足够强。

## 10. 为什么 branch-specific score 的混合形式也有理论解释

在代码里，branch-specific score 不是平地起高楼，而是从 shared backbone 平滑过渡出来的。

先定义两个专用分数：

$$
s_{x,j}
=
\left(q_{\mathrm{cov},j}\right)^{\alpha_x}
\cdot
\left(q_{\mathrm{link},j}\right)^{\beta_x},
$$

$$
s_{r,j}
=
\left(q_{\mathrm{link},j}\right)^{\beta_r}
\cdot
\left(q_{\mathrm{exist},j}\right)^{\alpha_r}.
$$

然后和共享 backbone

$$
s_{0,j}
=
q_{\mathrm{cov},j}
\cdot
q_{\mathrm{link},j}
\cdot
q_{\mathrm{exist},j}
$$

做几何插值：

$$
\tilde s_{x,j}=s_{0,j}^{1-\eta_x}s_{x,j}^{\eta_x},
\qquad
\tilde s_{r,j}=s_{0,j}^{1-\eta_r}s_{r,j}^{\eta_r}.
$$

这一步看起来像经验 blending，但它其实等价于 log-utility space 里的凸组合。因为

$$
\log\tilde s_{x,j}
=
(1-\eta_x)\log s_{0,j}+\eta_x\log s_{x,j},
$$

$$
\log\tilde s_{r,j}
=
(1-\eta_r)\log s_{0,j}+\eta_r\log s_{r,j}.
$$

所以更准确的解释不是“在两个分数之间随便折中”，而是：

**在 shared utility 和 branch-specific utility 之间做连续插值。**

这就让 branch decoupling 成为一个可解释的“渐进偏离”，而不是完全换了一套规则。

## 11. 为什么 weak structure-aware refinement 只能是“弱修正”

当前方法在 decoupling 之后，还会乘一个结构先验：

$$
\tilde s_{x,j}\leftarrow \tilde s_{x,j}\left(\xi_{x,j}\right)^{\gamma_x},
\qquad
\tilde s_{r,j}\leftarrow \tilde s_{r,j}\left(\xi_{r,j}\right)^{\gamma_r}.
$$

其中 $\xi_{x,j},\xi_{r,j}$ 是结构先验，$\gamma_x,\gamma_r$ 是强度参数。

这个形式也能解释成 log-utility 的额外项：

$$
\hat u_{x,j}'=\hat u_{x,j}+\gamma_x\log\xi_{x,j},
\qquad
\hat u_{r,j}'=\hat u_{r,j}+\gamma_r\log\xi_{r,j}.
$$

进一步，它等价于一个“向图先验靠拢”的 KL 正则化问题：

$$
w_x^\star
=
\arg\max_{w\in\Delta}
\left\{
\sum_j w_j\hat u_{x,j}
-\tau_x D_{\mathrm{KL}}(w\,\|\,p_x)
\right\},
$$

$$
w_r^\star
=
\arg\max_{w\in\Delta}
\left\{
\sum_j w_j\hat u_{r,j}
-\tau_r D_{\mathrm{KL}}(w\,\|\,p_r)
\right\},
$$

其中先验满足

$$
p_{x,j}\propto \xi_{x,j}^{\gamma_x/\tau_x},
\qquad
p_{r,j}\propto \xi_{r,j}^{\gamma_r/\tau_r}.
$$

这说明结构项的理论角色应该是：

- 它是一个 prior；
- 它只是在原有 utility 上施加轻量偏置；
- 它不是主导信号。

这也正好解释了为什么当前 paper 必须强调 `weak structure-aware`，而不是把 topology 说成主导贡献：

- 如果把结构项用得太强，理论解释就从“quality-aware weighting + prior regularization”变成了“topology-driven override”；
- 而实验也表明，尤其在 existence branch 上，结构项过强很容易损伤 cardinality agreement。

所以 `\gamma_r \ll \gamma_x` 并不是偶然调参，而是和理论解释一致的设计选择。

## 12. 为什么 smoothing 和 minimum-weight safeguard 不是主理论贡献

当前实现里还有：

- exponential moving average smoothing；
- minimum-weight safeguard。

例如平滑项形式是：

$$
\omega_{k,s}^{x,(j)}
=
\operatorname{Normalize}
\left(
\alpha_x\omega_{k-1,s}^{x,(j)}
+
(1-\alpha_x)\bar\omega_{k,s}^{x,(j)}
\right).
$$

它们当然是合理的，但更准确的说法是：

- 这是为了稳定时间序列上的权重波动；
- 是 pragmatic stabilization；
- 不是这篇 paper 最应强调的理论创新。

从理论上讲，它更像是“在一个时变最优点附近做平滑追踪”，而不是一个新的融合原理。

## 13. 哪些部分是严格可解释的，哪些部分是建模选择

这点在写 paper 时非常重要。下面最好明确分开。

### 13.1 严格或相对严格可解释的部分

- KLA / geometric-average fusion 的变分解释。
- 归一化乘积权重作为熵正则 simplex 分配的解释。
- Gaussian spatial fusion 作为 Gaussian family 内的 KLA barycenter。
- existence fusion 的 logit pooling 解释。
- Bernoulli-RFS KL 分解。
- decoupled surrogate objective 对 spatial / existence 更新的解释。
- structure-aware refinement 作为 toward-prior 的 KL regularization。

### 13.2 合理但仍属于建模选择的部分

- 用 inverse trace 来代表 covariance quality。
- 用 $|2r-1|$ 来代表 existence decisiveness。
- existence-confidence 的具体幂次和下界映射。
- branch-specific power、blend strength、structure strength 的数值设置。
- EMA 平滑和 minimum-weight safeguard。

这并不意味着这些部分“不可靠”，而是意味着它们更适合写成：

- `motivated by theory`
- `consistent with the variational interpretation`
- `supported by experiments`

而不应该写成：

- `universally optimal`

## 14. 理论依据如何支撑 paper 的主结论

把上面所有内容串起来，这篇 paper 的理论依据最终支持的是以下几条主结论。

### 14.1 为什么 paper 不需要 claim 新滤波器

因为理论上最稳的说法是：

- LMB / KLA 作为基础框架已经足够合理；
- 本文的贡献在于 adaptive weight allocation。

### 14.2 为什么三因子 backbone 是主方法

因为：

- `covariance` 对应 posterior concentration；
- `link quality` 对应 realized communication reliability；
- `existence confidence` 对应 posterior decisiveness on target existence。

这三个量共同构成了“应该信任谁”的最核心三维解释。

### 14.3 为什么 existence confidence 是关键补充

因为如果没有它：

- covariance 只能解释空间信息；
- link quality 只能解释通信可靠性；
- 但 cardinality / existence 上的 decisiveness 没有被表达。

而 Bernoulli-RFS 的理论分解恰好说明 existence branch 是一个独立且重要的对象。

### 14.4 为什么 decoupling 和 weak structure-aware 是 refinement

因为：

- decoupling 来自 LMB / Bernoulli 结构本身；
- structure term 更像 prior regularization；
- 所以它们理论上应该被放在 backbone 之后，而不是喧宾夺主。

## 15. 最适合放进论文里的通俗总结

如果要把这套理论依据压缩成论文里最自然的一段话，比较合适的说法是：

**在 unknown cross-correlation 的 distributed LMB fusion 中，KLA 提供了保守而合理的融合框架。本文不改变这一框架，而是把问题聚焦到权重分配：一个节点的融合权重应同时反映其后验空间集中度、实际链路可靠性以及对目标存在性的决断程度。该三因子乘积形式可以解释为 simplex 上带熵正则的效用分配问题；而 LMB/Bernoulli 结构本身又支持将 spatial branch 与 existence branch 进行温和解耦。最后，结构感知项只作为图先验的弱正则化出现，而不是替代质量驱动权重的主导机制。**

## 16. 当前最稳的理论定位

截至当前材料，这篇 paper 最稳的理论定位可以概括为：

- 它不是一个“新滤波家族”的理论。
- 它是一个“在 KLA-based distributed LMB fusion 内，针对 adaptive weight allocation 的理论化解释”。
- 它最强的理论支撑来自：
  - KLA 的变分基础，
  - Bernoulli/LMB 的分解结构，
  - 熵正则 simplex 优化，
  - 以及图先验 KL 正则化解释。
- 它最合适的 claim 是：
  - `theory-motivated and experimentally supported`
  - 而不是 `fully optimal in complete generality`

这也是当前 paper 叙事上最安全、也最有说服力的位置。
