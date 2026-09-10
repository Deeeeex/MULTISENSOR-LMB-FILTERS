"""Publish the complete verified descriptive census without changing the gate."""
from pathlib import Path
import csv
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parent.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'RESULTS_CN.md';assert not destination.exists()
    check=json.loads((OUT/'VERIFICATION.json').read_text());assert check['passed']
    for name,h in check['input_sha256'].items():assert sha(ROOT/name)==h,name
    result=json.loads((OUT/'RESULTS.json').read_text());case=json.loads((OUT.parent/'RESULTS.json').read_text())
    assert case['passed'] and not case['expand_same_refinement']
    names={'marked_gaussian_evidence':'GCE','marked_gaussian_evidence_guarded_scalar':'Guarded Scalar','marked_lineage':'No-age'}
    suffix='_known_censor';summaries=result['summaries']
    lines=['# 为什么即时回流减少后，候选池仍然拥挤','',
        '本记录继续描述已经完成的十二条轨迹，不修改算法或输出。已知本地存在率规则未通过原先固定的扩大验证门槛，结论保持不变。以下分类覆盖三种方法、原始及干预、两种链路、全部 240 帧；目标窗口仍为 GT 5、2 m、第 53–122 帧。','',
        '“首次到达”指该接收端此前没有预测过这个精确标签；“间隔后重返”指当前本地预测中没有它，但此前预测过。直接一帧反弹还要求：上帧正是已知值替换使它从本来可保留变为删除，对端仍保留同一标签；下一帧本端无该预测，只能再次使用 0.001 缺失项，接收同一对端标签后又保留。每一步都连接实际记录。','']
    for window,scope,title in [('original_window','target_neighbourhood','原目标窗口'),('full','all','完整轨迹、全部标签')]:
        lines+=['## '+title,'','| 链路 | 方法 | 远端单侧带入总数 | 同帧删除后返回 | 首次到达 | 间隔后重返 | 前次被替换值删除 | 直接一帧反弹 |',
            '|---|---|---:|---:|---:|---:|---:|---:|']
        for r in [r for r in summaries if r['window']==window and r['scope']==scope]:
            name=names[r['arm'].removesuffix(suffix)]+(' + 已知值' if r['arm'].endswith(suffix) else ' 原始')
            lines.append(f"| {r['condition']} | {name} | {r['remote_only_retained']:,} | {r['same_frame_pruned']:,} | {r['first_local_arrival']:,} | {r['return_after_gap']:,} | {r['after_refinement_prune']:,} | {r['direct_one_frame_rebound']:,} |")
    lines+=['','前次被替换值删除与直接一帧反弹都是“间隔后重返”的子集。全部间隔长度保存在 `RESULTS.json` 的 `gap_counts`；没有选择更有利的时间跨度。','',
        '## 固定顺序的原生例子','',
        '以下为各干预方法／链路在原窗口里首个满足全部直接反弹条件的事件，不代表独立样本或典型程度。','']
    for e in result['examples']:
        name=names[e['arm'].removesuffix(suffix)]
        lines.append(f"- {e['condition']} / {name}，机器人 {e['robot']}，标签 [{e['birth_frame']},{e['birth_location']}]：第 {e['last_prediction_frame']} 帧，本地实际存在率 {e['last_local_r']:.12g}；原融合会输出 {e['last_refinement_old_r']:.12g}，替换后为 {e['last_refinement_actual_r']:.12g}，低于或等于 0.001 而删除。同帧对端仍保留该标签，存在率 {e['peer_r_at_last_prediction']:.12g}。第 {e['frame']} 帧本端没有该预测，对端传来 {e['remote_input_r']:.12g}，本端再次使用 0.001 缺失项并输出 {e['returned_r']:.12g}，重新超过阈值。")
    lines+=['','## 可以和不能得出的结论','']
    gce=[r for r in summaries if r['arm']=='marked_gaussian_evidence_known_censor' and r['window']=='original_window' and r['scope']=='target_neighbourhood']
    if any(r['direct_one_frame_rebound'] for r in gce):
        lines+=['原生记录证实：本地已知值的使用范围只覆盖本帧仍有预测的标签。一旦替换值使本地标签被真正删除，而对端继续保留，下一帧该标签便可能重新落回 0.001 缺失项规则。这条实际跨帧路径解释了为何减少同帧回流，不等于阻止远端重新带入。各方法发生多少次、是否覆盖两个链路，以上表为准。']
    else:lines+=['未在 GCE 原目标窗口内确认所定义的直接一帧反弹链；不能用这条具体路径解释该窗口的持续失效。其他首次到达和间隔后重返按完整计数保留。']
    lines+=['','这些仍是相关标签事件，不是物理目标数、误检输出数或独立统计证据。这里识别的是该次干预的作用范围及剩余实际路径，没有证明它们构成原 GCE 与 No-age 全部性能差异的充分原因。没有延长缓存、另调阈值、重新运行算法或改变原实验门槛。','',
        f"生产按时间前向追踪；独立验证按标签历史二分定位前次预测，核对全部 {check['verified_events']:,} 个事件、48 个分组及保存例子的每个操作数。这种独立计算仍在同一工作流程内完成。",'']
    destination.write_text('\n'.join(lines))
    with (OUT/'ALL_SUMMARIES.csv').open('w') as stream:
        flat=[{k:v for k,v in r.items() if k!='gap_counts'} for r in summaries]
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]),lineterminator='\n');writer.writeheader();writer.writerows(flat)
    hashes={**check['input_sha256']}
    for p in OUT.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:hashes[str(p.relative_to(ROOT))]=sha(p)
    (OUT/'FINAL_VERIFICATION.json').write_text(json.dumps(dict(passed=True,native_changes=False,verified_events=check['verified_events'],
        verified_groups=48,original_case_gate_unchanged=True,input_sha256=hashes),indent=2)+'\n')
    print('REENTRY REPORT VERIFIED',len(result['examples']),'direct native examples',flush=True)

if __name__=='__main__':main()
