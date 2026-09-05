import {execFileSync} from 'node:child_process';

// One-time precise update, based on canonical document revision 1200.
// The opening paragraph and SVG are handled separately; no resource is replaced.
// Applied on 2026-09-05 through revision 1216. This is a change record, not an
// idempotent sync command: replaced block IDs are no longer current.
const doc='HcFFdtKIRovhHLxKrx5jVpiBpJh';
const p=(lead,text)=>`<p><b>${lead}</b>${text}</p>`;
const updates=[
['doxjpHe1aH5mVmhtrCzYKreYFvf',p('已实现：少量连接组成基础通路。','每个编队内部用一条有向环传递信息，编队之间用一棵树连接，每条树边安排正、反方向各一条网关消息。N 个传感器、F 个编队时，每步安排 N+2(F−1) 条消息，M24 为 30 条、X36 为 46 条；完整两输入参考分别为 48 和 72 条。这是所选结构的精确计数，不是所有通信图的理论最小值。')],
['doxjpcsDIpbsDnQpxWWYthrXkKh',p('动态性用在必要的修复上。','每轮只看当前物理条件和上一轮编队树。旧树仍能实现时保留；失效时先尽量少换编队边，再比较可靠性和距离。保留的是编队级连接结构，传感器网关端点仍按当前物理条件实现，不是锁死所有端点。这样控制不必要的结构变化，但保证的只是物理可行时的计划连通，不能保证丢包后仍连通。')],
['doxjp7M5KHCyLTkbfpcClxsGgVb',p('验证中：消息没到以后怎样融合。','当前基线对意外缺失输入重新归一化。例：自身、主要邻居、弱邻居原本占 0.25、0.70、0.05；主要邻居丢包后，弱邻居会从 0.05 放大到 0.1667。V278 固定稀疏路由，只把缺失权重归还自身，使自身为 0.95、弱邻居仍为 0.05。它不增加消息，但可能放慢信息传播，后验大小也可能变化，所以目前只是待验证规则，不是已确认更优的方法。')],
['doxjpXzr89BqowyQhFdPfZCdRMe',p('哪些不变，哪些有意改变。','消息继续携带完整目标后验，融合底座继续使用保留混合分量的 LMB-KLA 近似，并按视域区分缺失标签是否能作为否定证据。三组路由实验共享场景、测量和随机数，但稀疏化有意减少输入，把主动省略的权重归还自身；V278 则只改变意外缺失输入的处理。不能笼统写成所有组的权重和消息机会完全相同。GNN 暂不属于当前主方法。')],
['doxjpBe4c81OtMcfbpe6HTmBGwy',p('为什么优先保住目标存在信息。','已存结果显示，最小骨干中数量项占平方 E-OSPA 的约 98%–99.6%。若各时刻估计目标数不变，即使定位误差理想地降到零，M24 平均 E-OSPA 最多再降 0.279 m、X36 至多 1.311 m。因此下一步要区分信息未到达与融合后存在概率下降，并用 E-OSPA、目标数、条件 RMSE、一致性、通信量和最弱编队共同判断。当前证据仍是单种子开发结果。')],
['doxjpOKpzZq84b90GvJ0wwn9l4g',p('目前卡点。','X36 最小骨干的目标数量绝对误差由 18.456 增至 18.597；数量项占平方 E-OSPA 的 98.074%–99.337%。保持各时刻估计目标数不变时，仅改善定位最多还能降低 1.311 m 平均 E-OSPA。因此重点是让目标存在信息到达并保留，而不只是让图更连通或位置 RMSE 更低。V278 正在验证缺失输入后的自身权重回填，尚无最终结果。')],
['doxjphJNgFkAAxjIMO18lGW0d1b',p('因果性边界。','在线策略只读取当前物理图、当前链路可靠性、当前节点位置和上一轮编队树，不利用未来掉线、真值或最终误差选边。旧编队树仍可实现时保留，但物理网关端点仍按当前条件实现。当前控制器使用网络范围的几何信息；获取与分发这些信息的控制流量未计入后验字节，也尚未证明为完全分散式路由实现。')],
['doxjpzRnm76HT8nkZxeghputirc',p('与 Adaptive-KLA 及学习模块的关系。','Adaptive-KLA 主要研究既定输入条件下的融合权重。当前主线先研究稀疏路由如何维持跨编队输入，再单独检验消息缺失后的接收处理；两项作用分开对照。GNN 和未来数步价值预测不属于当前已实现主方法，只有确定性主线形成可重复收益后再考虑引入。')],
['doxjp9nFxQfuzQ2r7er0MHluTrd',p('当前状态与停止条件。','M24 与 X36 的固定树、完整修复、最小骨干三臂全程开发比较都已完成。最小骨干保留通信优势，但 X36 的目标数和焦点一致性仍有退化。当前只运行 V278 缺失权重回填，原定联合判据不变；若无价值，先定位目标存在信息在哪个环节丢失，不开展参数扫描或新的网关评分分支。多种子和独立布局验证仍待完成。')]
];
const rows=[
['当前物理图','从当前位置、距离和链路可靠性确定可用连接。','不读取真值、未来轨迹或未来丢包。'],
['因果编队树（已实现）','旧编队树仍可实现时保留；不可行时少换树边，再比较通信质量。','保留编队结构；网关端点按当前物理条件实现。'],
['稀疏骨干（已实现）','编队内有向环＋编队树每条边正反向各一条网关消息。','计划消息数 N+2(F−1)；物理可行时计划图强连通。'],
['统一融合底座','完整目标后验；保留混合分量的 LMB-KLA 近似与视域感知缺失标签处理。','不是任意 GM 幂的精确闭式解；三组路由共用这一底座。'],
['实际缺失输入（验证中）','基线重新归一化；V278 固定稀疏图，把缺失权重回填自身。','与主动省略残余边区分；不预设回填更优，不增加消息。'],
['配对评价','固定树、完整修复、稀疏骨干，以及单独的接收规则对照。','联合看目标集合、条件定位、数量、一致性、最弱编队和后验字节。'],
['当前优化重点','让目标存在信息及时到达并在融合后保留。','定位单独改善的空间已很小；GNN 暂不进入主方法。']
];
updates.push(['doxjpGhgg81ayv0gFzrCqKitCme',`<table><thead><tr><th>组成与状态</th><th>设计含义</th><th>边界或验证方式</th></tr></thead><tbody>${rows.map(row=>`<tr>${row.map(x=>`<td>${x}</td>`).join('')}</tr>`).join('')}</tbody></table>`]);
for(const [id,content] of updates){
  const r=JSON.parse(execFileSync('lark-cli',['docs','+update','--as','user',
    '--doc',doc,'--command','block_replace','--block-id',id,'--content',content],
    {encoding:'utf8',maxBuffer:4*1024*1024}));
  if(!r.ok || r.data?.result!=='success') throw new Error(JSON.stringify(r));
  console.log(`Updated ${id}; revision ${r.data.document?.revision_id}`);
}
