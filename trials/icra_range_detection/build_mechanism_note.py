"""Summarize descriptive case evidence without changing model selection."""
from pathlib import Path
import csv
import hashlib
import json

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence_range';GS='marked_gaussian_evidence_guarded_scalar_range';NOAGE='marked_lineage_range'
NAMES={GCE:'GCE',GS:'Guarded Scalar',NOAGE:'No-age'}

def main():
    trace_path=OUT/'MECHANISM_TRACE.json';cf_path=OUT/'SCALAR_CASE_SUBSTITUTION.json';csv_path=OUT/'MECHANISM_FRAMES.csv'
    trace=json.loads(trace_path.read_text());cf=json.loads(cf_path.read_text())
    assert trace['passed'] and cf['passed'] and trace['csv_sha256']==sha(csv_path)
    rows=list(csv.DictReader(csv_path.open()))
    lines=['# 指定机制案例：候选分化与即时替换检查','',
        '本页只分析预先指定的 `v2xt_0001`、真值 ID 5，不参与距离模型的筛选。三种方法共享同一距离检出模型，分别运行完整递归；所有原生结果均已通过预检审计。','',
        '## 间歇链路的检出分解','',
        '帧 53–96 尚未进入整段通信中断，但仍有随机丢包；帧 97–122 位于既定中断区间。检出使用既定的 2 米真值—输出匹配。','',
        '| 方法 | 53–96 检出 | 97–122 检出 | 合计 |','|---|---:|---:|---:|']
    for arm in [GCE,NOAGE,GS]:
        a=next(r for r in trace['summaries'] if (r['condition'],r['arm'],r['phase'])==('intermittent',arm,'53_96'))
        b=next(r for r in trace['summaries'] if (r['condition'],r['arm'],r['phase'])==('intermittent',arm,'97_122'))
        lines.append(f"| {NAMES[arm]} | {a['detected']}/88 | {b['detected']}/52 | {a['detected']+b['detected']}/140 |")
    lines+=['','## 帧 53 的状态已经不同','',
        '“近邻候选”指均值距该真值不超过 2 米、存在概率大于 0.001 的不同标签分量。它描述候选分布，不等同于输出虚警，也不能仅凭位置确定候选身份。','',
        '| 方法 | 机器人 | 近邻候选数 | 其中最大存在概率 | 是否检出 |','|---|---:|---:|---:|---|']
    for arm in [GCE,NOAGE,GS]:
        for n in ['1','2']:
            r=next(r for r in rows if (r['condition'],r['arm'],r['frame'],r['robot'])==('intermittent',arm,'53',n))
            lines.append(f"| {NAMES[arm]} | {n} | {r['near_active_components']} | {float(r['strongest_near_r']):.6f} | {'是' if r['output_match']=='True' else '否'} |")
    lines+=['','此时 GCE 和 No-age 各有一个近邻候选；Guarded Scalar 的两台机器人各有 136 个。Guarded Scalar 在帧 53–96 的 79 个漏检机器人帧中，都仍存在至少一个活跃近邻候选。因此，不能把这一段的失败简单描述为“附近已经没有任何候选”。','',
        '## 在 Guarded Scalar 访问过的输入上做即时替换','',
        '保持实际历史、当前输入、准入指数与曲率拒绝结果，重建联合高斯乘积的均值和归一化常数，再分别替换均值、归一化项或两者。没有通信的帧保持实际局部输出；每次替换后不向后递归。先复算实际输出，确认与原生 OSPA 和指定目标匹配一致。','',
        '| 固定输入替换 | 指定目标检出 | 窗口平均 OSPA（米） |','|---|---:|---:|']
    names={'actual':'实际 Guarded Scalar','normalizer_only':'仅归一化项','mean_only':'仅均值','joint':'均值与归一化项'}
    for r in cf['summaries']:lines.append(f"| {names[r['alternative']]} | {r['detected']}/140 | {r['mean_ospa']:.9f} |")
    lines+=['','测试的即时替换没有恢复该目标。这与完整 GCE 递归的 118/140 是两个不同实验：在帧 53，二者的候选分布就已经明显不同。该结果提示要检查更早的递归分化，不能把即时替换的结果解释成联合校正没有作用，也不能据此认定某一个门控或丢包事件就是根因。','',
        '目前能确认的是候选分化及所测即时替换不足；尚未定位最初触发分化的事件。No-age 同样达到 118/140，所以本案例仍不能证明 GCE 相对 No-age 的独立增益。','',
        '可复算来源：`MECHANISM_TRACE.json`、`MECHANISM_FRAMES.csv`、`SCALAR_CASE_SUBSTITUTION.json`，对应脚本为 `diagnose_mechanism.py` 与 `diagnose_scalar_case.py`。','']
    destination=OUT/'MECHANISM_NOTES_CN.md';assert not destination.exists();destination.write_text('\n'.join(lines))
    (OUT/'MECHANISM_NOTE_BUILD.json').write_text(json.dumps(dict(trace_sha256=sha(trace_path),substitution_sha256=sha(cf_path),
        csv_sha256=sha(csv_path),builder_sha256=sha(Path(__file__)),note_sha256=sha(destination)),indent=2)+'\n')
    print('BUILT BOUNDED MECHANISM NOTE')

if __name__=='__main__':main()
