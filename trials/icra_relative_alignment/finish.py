"""Recompute the frozen comparisons and bind the complete study artifacts."""
from pathlib import Path
import csv
import json
import math
from common import OUT,ROOT,ARMS,sha,write_new,frozen

LABELS={ARMS[0]:'GCE',ARMS[1]:'No-age'}
DATA={'v2v_development':'V2V4Real 已见 9 段','v2x_val':'V2X-Real 已见 5 段'}

def main():
    cfg=frozen();assert not (OUT/'FINAL_VERIFICATION.json').exists()
    feature=json.loads((OUT/'FEATURES.json').read_text());est=json.loads((OUT/'ESTIMATION.json').read_text())
    parity=json.loads((OUT/'audit_alignment_parity.json').read_text());audit=json.loads((OUT/'audit_alignment_corrected.json').read_text())
    execution=json.loads((OUT/'FEATURE_EXECUTION.json').read_text())
    freeze=sha(OUT/'FREEZE.json')
    assert all(r['passed'] for r in [feature,est,parity,audit]) and execution['completed'] and execution['returncode']==0
    assert all(r['freeze_sha256']==freeze for r in [feature,est,parity,audit,execution])
    assert feature['raw_clouds']==5224 and est['packets_decoded']==5224 and est['paired_frames']==2612
    assert est['checked_overlap_counts']==4390772
    assert parity['native_runs']==4 and parity['robot_frames']==1148 and len(parity['parity'])==4
    assert audit['native_runs']==28 and audit['robot_frames']==10448 and not audit['parity']
    assert est['feature_sha256']==sha(OUT/'FEATURES.json') and est['feature_execution_sha256']==sha(OUT/'FEATURE_EXECUTION.json')
    assert est['table_sha256']==sha(OUT/'TRANSLATIONS.csv')
    for r in [feature,est,parity,audit]:
        for name,digest in r['artifacts'].items():assert sha(ROOT/name)==digest,name
    rows=audit['rows']+audit['references'];assert len(rows)==56
    with (OUT/'SCORES_alignment_corrected.csv').open(newline='') as stream:csvrows=list(csv.DictReader(stream))
    assert len(csvrows)==56
    for a,b in zip(rows,csvrows):
        for key in ['ospa','gospa','loc2','miss2','false2','total_wire_bytes']:
            assert float(b[key])==a[key]
    groups=[];gates=[]
    metrics=['ospa','gospa','loc2','miss2','false2','total_wire_bytes','raw_bytes','alignment_payload_bytes','alignment_compute_seconds']
    for dataset in DATA:
        for arm in ARMS:
            values={}
            for mode in ['original','occupancy']:
                group=[r for r in rows if (r['dataset'],r['arm'],r['mode'])==(dataset,arm,mode)]
                assert len(group)==(9 if dataset=='v2v_development' else 5)
                summary=dict(dataset=dataset,arm=arm,mode=mode,sequences=len(group),
                             **{key:math.fsum(r[key] for r in group)/len(group) for key in metrics})
                groups.append(summary);values[mode]=summary
            for metric in ['ospa','gospa']:
                reference=values['original'][metric];candidate=values['occupancy'][metric]
                passed=candidate<=reference if dataset=='v2v_development' else candidate<=.99*reference
                gates.append(dict(dataset=dataset,arm=arm,metric=metric,reference=reference,candidate=candidate,
                                  difference=candidate-reference,relative_change_percent=(candidate/reference-1)*100,passed=passed))
    advance=all(g['passed'] for g in gates);assert len(gates)==8
    source_comparisons=[]
    for dataset in DATA:
        candidates={r['arm']:r for r in groups if r['dataset']==dataset and r['mode']=='occupancy'}
        source_comparisons.append(dict(dataset=dataset,gce_minus_noage={m:candidates[ARMS[0]][m]-candidates[ARMS[1]][m] for m in ['ospa','gospa']}))
    summary=dict(passed=True,advance=advance,gates=gates,groups=groups,corrected_method_comparisons=source_comparisons,
                 native_runs=32,new_corrected_robot_frames=10448,zero_parity_robot_frames=1148,
                 freeze_sha256=freeze,corrected_audit_sha256=sha(OUT/'audit_alignment_corrected.json'))
    write_new(OUT/'SUMMARY.json',summary)
    verdict=f"八项固定比较通过 {sum(g['passed'] for g in gates)}/8；"+('允许继续研究该空间校正方向。' if advance else '该版本停止扩展，不调整网格、搜索窗口或挑选片段补救。')
    lines=['# 当前点云相对平移：完整递推实验','',verdict,'',
           '这是新的共同空间误差实验。每帧由当前两车点云独立估计一份平移，同时用于 GCE 和 No-age 的输入。没有真值配准、未来帧平滑、标签挑选或融合系数搜索。全部 14 段均已见，可靠链路结果不代表未见数据或间歇链路。','',
           '## 完整结果','',
           '每段等权。OSPA/GOSPA 为 m；后三项为 GOSPA 平方定位、漏检、虚假代价。原始输入和校正输入使用相同评分区域、真值及检测集合。','',
           '| 数据 | 方法 | 输入 | OSPA | GOSPA | 定位 | 漏检 | 虚假 |','|---|---|---|---:|---:|---:|---:|---:|']
    for r in groups:
        lines.append(f"| {DATA[r['dataset']]} | {LABELS[r['arm']]} | {'原始' if r['mode']=='original' else '当帧校正'} | {r['ospa']:.9f} | {r['gospa']:.9f} | {r['loc2']:.6f} | {r['miss2']:.6f} | {r['false2']:.6f} |")
    lines+=['','## 预先固定的八项比较','',
            '两种方法分别要求：V2X OSPA、GOSPA 各下降至少 1%，V2V 两项均不增加。全部比较都保留；校正对所有方法共同使用，不能把共同输入收益归因于 GCE。','',
            '| 数据 | 方法 | 指标 | 相对变化 | 通过 |','|---|---|---|---:|---|']
    for g in gates:
        lines.append(f"| {DATA[g['dataset']]} | {LABELS[g['arm']]} | {g['metric'].upper()} | {g['relative_change_percent']:+.4f}% | {'是' if g['passed'] else '否'} |")
    lines+=['','## 资源开销与解释边界','',
            '当前方案在本地滤波前增加一轮可靠通信：每车发送 4,000 字节位图和 32 字节头。两份消息每帧共 8,064 字节，按原 16 KiB 分配和消息控制费用，额外线上成本为 33,024 字节/帧。该费用包含在下面的总通信量里。','',
            '| 数据 | 方法 | 原始线上字节/段 | 校正后总线上字节/段 | 变化 |','|---|---|---:|---:|---:|']
    for dataset in DATA:
        for arm in ARMS:
            a=next(r for r in groups if (r['dataset'],r['arm'],r['mode'])==(dataset,arm,'original'))
            b=next(r for r in groups if (r['dataset'],r['arm'],r['mode'])==(dataset,arm,'occupancy'))
            lines.append(f"| {DATA[dataset]} | {LABELS[arm]} | {a['total_wire_bytes']:.0f} | {b['total_wire_bytes']:.0f} | {(b['total_wire_bytes']/a['total_wire_bytes']-1)*100:+.2f}% |")
    grid_seconds=math.fsum(r['grid_seconds'] for r in est['groups']);solve_seconds=math.fsum(r['both_receivers_solve_seconds'] for r in est['groups'])
    lines += ['',f'2,612 帧的网格生成合计 {grid_seconds:.3f} 秒，两个接收端求解合计 {solve_seconds:.3f} 秒；文件下载、解码和独立核验另计。这是当前机器的实测计算量，不包含真实无线往返等待，也不证明实时运行。','',
              '先前发现的相对偏移不等于已经查明位姿误差。移动物体、时间偏差、两侧采样差异和源标注坐标问题均可能影响这个粗配准。原检测与评分裁剪没有重做，因此该实验也不等于用校正位姿重新运行完整检测器。','',
              '## 完成和核验','',
              '4 条零平移递推与原输出完全一致；28 条校正递推均实际退出并完成独立评分，共 11,596 个新机器人帧。5,224 份原始点云经原压缩包 CRC 或既定 SHA-256 校验，全部网格另用索引法复算；2,612 帧的 4,390,772 个整数平移交集值均与独立位运算实现完全一致。GCE 的完整高斯分布、存在率、报文费用、输出提取和普通指标分别核验。','',
              '逐帧平移见 [TRANSLATIONS.csv](TRANSLATIONS.csv)，全部新旧逐段指标见 [SCORES_alignment_corrected.csv](SCORES_alignment_corrected.csv)，原始数据来源及紧凑缓存见 FEATURES.json，完整预注册见 [PROTOCOL.md](PROTOCOL.md)。','']
    report=OUT/'RESULTS_CN.md';assert not report.exists();report.write_text('\n'.join(lines))
    artifacts=cfg['sources'].copy()
    for item in [feature,est,parity,audit]:artifacts.update(item['artifacts'])
    for p in OUT.iterdir():
        if p.is_file():artifacts[str(p.relative_to(ROOT))]=sha(p)
    for p in (ROOT/'RUN/ICRA_RELATIVE_ALIGNMENT').rglob('*.log'):artifacts[str(p.relative_to(ROOT))]=sha(p)
    for unit in cfg['units']:
        for reference in unit['references'].values():artifacts[reference['path']]=reference['sha256']
    for name,digest in artifacts.items():assert sha(ROOT/name)==digest,name
    write_new(OUT/'FINAL_VERIFICATION.json',dict(passed=True,protected_files=len(artifacts),native_runs=32,
        native_robot_frames=11596,raw_clouds=5224,raw_pairs=2612,independent_overlap_values=4390772,
        gate_count=8,gate_passes=sum(g['passed'] for g in gates),advance=advance,artifacts=artifacts,verifier_sha256=sha(Path(__file__))))
    print('RELATIVE ALIGNMENT STUDY VERIFIED',verdict,flush=True)

if __name__=='__main__':main()
