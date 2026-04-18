# 理论依据梳理

## 1. 这篇 paper 到底在试图解决什么问题？

**在 unknown cross-correlation 的分布式多目标跟踪里，KLA-based LMB fusion 本身是合理的，但固定融合权重在异构通信条件下太僵硬，所以我们需要一个理论上说得通的 adaptive weight allocation 机制。**



## 2. 第一层背景：为什么最好先从 RFS 讲起

单目标跟踪通常把状态写成固定维向量，但多目标问题里，目标数目本身会变，所以系统状态更自然地写成一个随机有限集合：

$$
X_k=\{x_{k,1},\dots,x_{k,n_k}\},
$$

其中 $n_k$ 本身也是随机的。

FISST/RFS 可以看成是把**贝叶斯滤波**从“固定维向量状态”推广到“随机有限集合状态”

这就是 RFS 的核心直觉：它同时表示“现在有几个目标”和“每个目标的状态是什么”。所以 RFS 的作用不是把问题讲复杂，而是先把多目标问题定义对。

## 3. 第二层背景：为什么进一步要到 labeled RFS / LMB

只用 RFS 还不够，因为跟踪里我们还要区分目标身份。于是状态进一步写成带标签的集合：

$$
X_k=\{(x_{k,1},\ell_1),\dots,(x_{k,n_k},\ell_{n_k})\}.
$$

在这个基础上，LMB 的好处是：一个节点的局部后验，不只是给出目标位置，而是同时给出：

- 目标是否存在；
- 目标状态分布；
- 目标标签。

在形式上，每个节点维护的后验可以写成：

$$
\pi_k^{(j)}=\left\{\left(r_{k,i}^{(j)},\,p_{k,i}^{(j)}(\cdot,\ell_i)\right)\right\}_{i=1}^{M_k}.
$$

这里：

- $r_{k,i}^{(j)}$ 是目标存在概率；
- $p_{k,i}^{(j)}$ 是该目标的空间状态密度。

> 埋下一个伏笔：
>
> **因为 LMB 本来就同时包含 existence 和 spatial 两部分，所以后面我们把 existence branch 和 spatial branch 分开赋予融合权重，是有结构基础的。**

## 4. 第三层背景：为什么分布式融合要用 KLA

在分布式融合里，核心难点是：不同节点的后验通常存在未知相关性。

这意味着：

- 你不能假装它们独立；
- 但你也不知道它们到底相关到什么程度。

这时直接做精确贝叶斯融合容易重复计数信息，导致过度自信。KLA 或 geometric-average fusion 的意义就在这里：它提供了一个更保守的融合规则。

KLA 的形式是：

$$
\bar{\pi}_k^{(s)}(X)\propto \prod_{j\in\mathcal{N}_s}\left(\pi_k^{(j)}(X)\right)^{\omega_{k,s}^{(j)}},
\qquad
\sum_j \omega_{k,s}^{(j)}=1,\ \omega_{k,s}^{(j)}\ge 0.
$$

这不是经验公式，它有标准的变分解释：

$$
\bar{\pi}
=
\arg\min_{\pi}\sum_j \omega_j D_{\mathrm{KL}}(\pi\,\|\,\pi_j).
$$

也就是说，融合后的后验，是“对所有局部后验加权平均最接近”的那个后验。

## 5. 核心矛盾：为什么 fixed weights 不够

如果所有节点都一样、通信也一样，那么固定权重、均匀权重、Metropolis 权重都可以接受。

但本文的场景不是这样。这里至少有两层异质性：

1. 节点局部后验质量不同。
2. 节点信息送到邻居时，链路质量不同。

因此，一个节点是否值得被赋予更大权重，不应该只由拓扑决定，而至少应该由三件事共同决定：

- 它的后验是否更集中；
- 它的信息是否更可靠送达；
- 它对目标存在性是否更果断。

所以，paper 的理论重点就自然落在：

**从 fixed-weight fusion，转到 adaptive weight allocation。**

## 6. 方法的主理论依据：三因子 adaptive backbone

这篇 paper 最核心的公式，其实就是下面这个：

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
\frac{\tilde{\omega}_{k,s}^{(j)}}{\sum_u \tilde{\omega}_{k,s}^{(u)}}.
$$

也可以总结成：

**一个邻居该拿多少融合权重，取决于它是否可用、后验是否集中、链路是否可靠、以及它在 existence 上是否足够果断。**

这也是整篇 paper 的 backbone。

### 6.1 covariance 为什么合理

covariance score 写成：

$$
q_{\mathrm{cov},k}^{(j)}
=
\frac{1}{\epsilon + \frac{1}{M_k}\sum_i \operatorname{tr}(T_{k,i}^{(j)})}.
$$

它的直觉很简单：

- covariance 越小，说明分布越集中；
- 分布越集中，说明位置估计越可靠；
- 越可靠，就应该给更大权重。

**它本质上是在用后验集中度近似表达“这个节点的空间信息质量”。**

### 6.2 link quality 为什么必须有

链路质量写成：

$$
q_{\mathrm{link},k}^{(j)}
=
\frac{d_k^{(j)}}{d_k^{(j)}+\ell_k^{(j)}}.
$$

它表达的是 realized communication quality，而不是静态拓扑。

**在通信受限场景下，节点的信息价值不只取决于它本地能不能估得准，还取决于它的信息有没有被可靠送到。**

所以 link quality 不是附加项，而是这个问题定义本身的一部分。

### 6.3 existence confidence 为什么是关键第三项

用

$$
c_{k,i}^{(j)}=\left|2r_{k,i}^{(j)}-1\right|
$$

来衡量单个 Bernoulli 分量的 decisiveness。

然后聚合成节点级别的 existence confidence：

$$
\bar c_k^{(j)}
=
\frac{\sum_i r_{k,i}^{(j)}|2r_{k,i}^{(j)}-1|}
{\epsilon+\sum_i r_{k,i}^{(j)}}.
$$

最终得到：

$$
q_{\mathrm{exist},k}^{(j)}
=
\lambda_{\min} + (1-\lambda_{\min})\left(\bar c_k^{(j)}\right)^{p_e}.
$$

这一项的理论动机：

**covariance 只描述“位置准不准”，但不描述“你到底是否相信目标存在”；而 distributed fusion 里，existence / cardinality 一样重要。**

所以 `existence confidence` 不是小修小补，它补的是一个此前缺失的维度。

## 7. 为什么这个乘积形式不是纯经验拼接

很多人会质疑：  
“把三个 score 相乘，再归一化，这是不是有点经验？”

这时候就要给出本文最重要的 variational interpretation。

先定义每个节点的 log-utility：

$$
u_{0,j}
=
\log q_{\mathrm{cov},k}^{(j)}
+
\log q_{\mathrm{link},k}^{(j)}
+
\log q_{\mathrm{exist},k}^{(j)}.
$$

然后在 simplex 上考虑：

$$
\max_{w\in\Delta}
\left\{
\sum_j w_j u_{0,j} + \tau H(w)
\right\},
\qquad
H(w)=-\sum_j w_j\log w_j.
$$

它的解是：

$$
w_j^\star
=
\frac{\exp(u_{0,j}/\tau)}{\sum_u \exp(u_{0,u}/\tau)}.
$$

当 $\tau=1$ 时，就回到当前实现的 normalized-product 形式。

**我们不是随便把三个因子相乘，而是在做一个“带熵正则的信任分配问题”：高 utility 的节点拿更多权重，但熵项阻止权重塌缩到单个节点。**

## 8. 为什么 spatial branch 和 existence branch 可以分开加权

核心原因不是“我们觉得这么做好像更灵活”，而是：

**LMB / Bernoulli 的结构本来就把 existence 和 spatial 拆成了两类对象。**

更具体地说，Bernoulli-RFS 的 KL divergence 可以分解成：

$$
D_{\mathrm{KL}}(\beta\,\|\,\beta_j)
=
d_{\mathrm{Ber}}(r\,\|\,r_j)
+
r\,D_{\mathrm{KL}}(p\,\|\,p_j).
$$

这意味着：

- 一部分误差来自 existence mismatch；
- 另一部分误差来自 spatial mismatch；
- 两者不是同一种东西。

因此，允许 spatial branch 和 existence branch 使用不同的权重，不是毫无依据的自由发挥，而是和 Bernoulli/LMB 的结构相匹配。

即**branch decoupling 的理论依据来自 Bernoulli-RFS 的分解结构。**

## 9. 为什么当前的 decoupling 是“温和解耦”而不是彻底改写

代码里不是直接给两个分支定义完全独立的规则，而是从 shared backbone 平滑偏离：

$$
\tilde s_{x,j}=s_{0,j}^{1-\eta_x}s_{x,j}^{\eta_x},
\qquad
\tilde s_{r,j}=s_{0,j}^{1-\eta_r}s_{r,j}^{\eta_r}.
$$

这等价于在 log-utility 空间做插值：

$$
\log \tilde s_{x,j}
=
(1-\eta_x)\log s_{0,j}+\eta_x\log s_{x,j},
$$

$$
\log \tilde s_{r,j}
=
(1-\eta_r)\log s_{0,j}+\eta_r\log s_{r,j}.
$$

**decoupling 不是推翻 backbone，而是在 backbone 的基础上，对两个分支做温和偏置。**

这就保证了整个方法仍然围绕同一个主逻辑，而不是变成两个彼此无关的子方法。

## 10. 为什么 structure-aware 必须讲成“弱修正”

当前方法在 decoupling 之后，会再乘一个结构先验：

$$
\tilde s_{x,j}\leftarrow \tilde s_{x,j}\left(\xi_{x,j}\right)^{\gamma_x},
\qquad
\tilde s_{r,j}\leftarrow \tilde s_{r,j}\left(\xi_{r,j}\right)^{\gamma_r}.
$$

这一步可以解释成：在原有 utility 上，再加一个图先验：

$$
\hat u_{x,j}'=\hat u_{x,j}+\gamma_x\log\xi_{x,j},
\qquad
\hat u_{r,j}'=\hat u_{r,j}+\gamma_r\log\xi_{r,j}.
$$

也可以进一步看成一个 toward-prior 的 KL regularization。

- 它是 prior，不是主导信号；
- 它是 refinement，不是 backbone；
- 它只能弱用，尤其在 existence branch 上更要弱用。

因此 `weak structure-aware` 这个说法非常重要，因为它准确反映了当前理论位置：

**质量驱动的 adaptive weighting 是主体，structure-aware 只是附加偏置。**

## 一页版提纲

如果要压成一页汇报提纲，最短可以只保留下面 6 句：

1. 我们研究的是 unknown cross-correlation 下的 distributed LMB fusion。
2. KLA 是合理基线，因为它提供保守的 geometric-average fusion。
3. 真正的问题不是换滤波器，而是 fixed weights 在异构通信下不够用。
4. 因此我们把权重设计成 `covariance + link quality + existence confidence` 三因子 backbone。
5. 这个 backbone 可以解释为 entropy-regularized simplex allocation，而 existence / spatial 解耦则来自 Bernoulli/LMB 结构。
6. 结构感知项只是弱修正，不是主导机制，所以整个方法最稳的理论定位是 `theory-motivated adaptive weight allocation within KLA-based distributed LMB fusion`。
