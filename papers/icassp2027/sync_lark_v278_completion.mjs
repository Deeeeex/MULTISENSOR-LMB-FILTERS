import {execFileSync} from 'node:child_process';

// One-time, block-local change record against document revision 1217.
// Replaced IDs expire; this is not an idempotent synchronization command.
// Applied on 2026-09-05 through revision 1231; key results were read back.
const doc='HcFFdtKIRovhHLxKrx5jVpiBpJh';
const p=(lead,body)=>`<p><b>${lead}</b>${body}</p>`;
const updates=[
['doxjp4JIpLwvU7RAlDTV8NHeLRg',p('关键结果。','最小骨干在 M24/X36 上分别减少 10.041%/21.830% 的后验字节。新完成的 X36 自身权重回填 V278，相对静态路由使四项网络均值同时改善：E-OSPA 下降 0.542%、条件 RMSE 下降 45.124%、焦点一致性改善 0.096%、后验字节减少 21.535%。但集合误差与一致性收益仍小，尚未形成显著、稳定的跨尺度提升。')],
['doxjpuYC7CKgvcegn3U5Frp1Qdv',p('目前卡点。','V278 相对原稀疏骨干改善 E-OSPA 和一致性，却使条件 RMSE 退化 4.836%，未通过原定后续实验判据。其目标数量误差 18.518 仍高于静态路由的 18.456。原稀疏骨干的误差分解显示，单纯改善定位最多只能再降 1.311 m 平均 E-OSPA。下一步要定位目标信息在观测、实际接收、融合和输出哪个环节损失。')],
['doxjptpVwyjOS6SX2Odk1eVBQJg',p('当前主线。','用较少的消息维持跨编队的信息通路，并让接收节点在消息缺失时合理融合。设计分成两层：已实现的稀疏因果路由决定“谁给谁发”，接收规则处理“原定输入没到时怎么办”。自身权重回填已完成 X36 单种子对照，呈现精度与一致性的取舍，尚未成为跨尺度验证过的替代规则。下图是结构示意，不代表实际仿真规模。')],
['doxjpnYrvRtxgRGV9P7mACT695d',p('已测试：消息没到以后怎样融合。','基线对意外缺失输入重新归一化。例：自身、主要邻居、弱邻居原本占 0.25、0.70、0.05；主要邻居丢包后，弱邻居会被放大到 0.1667。V278 固定稀疏路由，把缺失权重归自身，使自身为 0.95、弱邻居仍为 0.05；该规则也适用于空邻居输入，不改变逐标签缺失处理。X36 单种子结果改善集合误差与一致性，却使条件 RMSE 退化，不作为统一替代规则。')],
['doxjpKyGDu1tu0WBUJbsygAPqje',p('主线方向。','通信受限下的稀疏动态 LMB-KLA：因果树修复维持基础信息通路，投递感知处理控制缺失输入对融合的影响。V278 已完成单因素对照，尚不能统一取代基线。下一步围绕目标存在信息的恢复与保留定位损失环节，不扩展网关评分或权重参数扫描。')],
['doxjp4PCuR6u4WNaSwIQ4AINFJg',p('最新单因素对照。','V278 的 X36、seed 1301、160 步实验已完成，静态与 V242 基线直接复用。它相对 V242 的 E-OSPA 改善 0.175%、焦点一致性改善 0.865%，但条件 RMSE 退化 4.836%，最受影响编队 RMSE 退化 37.361%。没有通过原定 1% 网络 RMSE 容忍度，不启动该规则的 M24 扩展；其四项均值相对静态同向改善仍保留在当前最优表中。')],
['doxjpNi1aETssxtTQZudH3X93Kb',p('当前状态与停止条件。','M24/X36 三臂全程比较、X36 自身权重回填对照均已完成。回填未通过原定后续判据，不做参数扫描。观测与传播分析显示，全局无人可见的目标—时刻仅约 2.7%，而跨编队信息在 X36 传播更慢。接下来区分局部更新、实际接收、融合和输出的损失，再决定下一项方法改动；多种子与独立布局验证仍待完成。')],
['doxjpLLaouOpYfBfk5kIDJRsGHe',p('阶段结论。','X36 原稀疏骨干仍保持最低后验字节和条件 RMSE；自身回填使四项网络均值相对静态同时改善，但整体 E-OSPA 与一致性收益很小，数量误差和局部尾部仍未解决。当前没有在所有指标占优且经过跨尺度验证的方法。')],
['doxjpEeSbEvL82PSMDBdD7Rq91e',p('表格解读。','M24 稀疏骨干在四项网络均值相对静态同向改善，但局部尾部退化。X36 原稀疏骨干的后验字节和条件 RMSE 最低，完整修复的 E-OSPA 和焦点分歧最低；新增自身回填在较低通信量下实现四项网络均值相对静态同向改善，但收益仍小且未过后续判据。这些方法并列保留，不把纠正前代码或其他场景的历史数值混入排名。')],
['doxjp39AwRKDHJ4i9BclifwhqDf','<p>动态路由主开发场景。M24 与 X36 的固定树、完整因果修复、稀疏骨干三臂均已完成；X36 自身回填也已完成。全部仍为单种子开发结果，局部尾部及跨尺度稳定收益待解决。</p>'],
['doxjpcHQ0dyLJ5CRttXabcTtNHH','<p>实际缺失输入（单种子已测试）</p>'],
['doxjpYiGxaAa8HbXIYc1bLf4swh','<p>不增加消息；X36 集合误差与一致性改善，但条件 RMSE 退化，未过原定后续判据。</p>']
];
const headers=['类别 / 方法','配对条件','E-OSPA：候选 / 固定','条件 RMSE：候选 / 固定','焦点一致性：候选 / 固定','尝试后验字节：候选 / 固定','最弱局部表现','证据状态'];
const rows=[
['M24：V242 稀疏骨干','temporal-coupled formation-braid；seed 1301；160 步','122.462 / 125.478；下降 2.403%','12.183 / 22.640；下降 46.190%','131.664 / 133.599；改善 1.449%','36.676 / 40.769 MB；减少 10.041%','最弱编队 E 改善 0.272%；RMSE 退化 24.085%','四项网络均值同向改善；局部尾部未解决'],
['X36：V242 稀疏骨干','同系列 X36；seed 1301；160 步','132.192 / 132.680；下降 0.368%','19.329 / 36.925；下降 47.655%','140.489 / 139.407；退化 0.776%','60.090 / 76.871 MB；减少 21.830%','最弱编队 E 退化 0.818%；RMSE 改善 4.444%','最低后验字节 / 条件 RMSE；目标数误差 18.597 / 18.456'],
['X36：完整因果修复','同场景、同随机数、160 步','131.795 / 132.680；下降 0.667%','19.586 / 36.925；下降 46.958%','139.004 / 139.407；改善 0.289%','81.259 / 76.871 MB；增加 5.708%','最弱编队 E 退化 1.079%；RMSE 改善 8.389%','最低 E-OSPA / 焦点分歧；通信增加'],
['X36：稀疏骨干＋自身回填 V278','同场景、同随机数；只改缺失输入处理','131.961 / 132.680；下降 0.542%','20.263 / 36.925；下降 45.124%','139.273 / 139.407；改善 0.096%','60.317 / 76.871 MB；减少 21.535%','最弱编队 E 退化 1.155%；RMSE 退化 4.079%','四项均值相对静态同向；目标数误差 18.518 / 18.456；相对 V242 的 RMSE 退化 4.836%，未过后续判据']
];
const table=`<table><thead><tr>${headers.map(x=>`<th>${x}</th>`).join('')}</tr></thead><tbody>${rows.map(row=>`<tr>${row.map(x=>`<td>${x}</td>`).join('')}</tr>`).join('')}</tbody></table>`;
updates.push(['doxjp3AXUo0Fi0PZXBBypwgK95f',table]);
for(const [id,content] of updates){
  const r=JSON.parse(execFileSync('lark-cli',['docs','+update','--as','user','--doc',doc,
    '--command','block_replace','--block-id',id,'--content',content],{encoding:'utf8',maxBuffer:4*1024*1024}));
  if(!r.ok || r.data?.result!=='success') throw new Error(JSON.stringify(r));
  console.log(`Updated ${id}; revision ${r.data.document.revision_id}`);
}
const detail=p('观测与传播的区别。','M24/X36 中，全网络无人可见的目标—时刻占比为 2.773%/2.734%；平均每个节点直接可见约 4 个目标，全局则有 16/24 个目标。假设可见就能完美检测、消息到达后永久保留信息，稀疏骨干在 8 步内具有观测来源路径的目标比例为 81.261%/63.183%，完整修复为 87.337%/68.388%。规模增大会加重传播时延，但这些是理想通信机会，不是实际标签召回率，尚不能直接归因于融合。');
const r=JSON.parse(execFileSync('lark-cli',['docs','+update','--as','user','--doc',doc,
  '--command','block_insert_after','--block-id','doxjpnKuexd0hU8R5zgxunhaZfg','--content',detail],
  {encoding:'utf8',maxBuffer:4*1024*1024}));
if(!r.ok || r.data?.result!=='success') throw new Error(JSON.stringify(r));
console.log(`Inserted V280 interpretation; revision ${r.data.document.revision_id}`);
