"""Build the Chinese report directly from all audited, frozen screen rows."""
from pathlib import Path
import hashlib
import json
import csv

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
GCE='marked_gaussian_evidence'
NAMES={GCE:'原始 GCE',GCE+'_range':'距离模型 GCE',GCE+'_constant':'固定拟合概率 GCE',
    'marked_lineage_range':'距离模型 No-age',GCE+'_guarded_scalar_range':'距离模型 Guarded Scalar',
    'marked_lineage_constant':'固定拟合概率 No-age',GCE+'_guarded_scalar_constant':'固定拟合概率 Guarded Scalar',
    'marked_lineage':'原始 No-age',GCE+'_guarded_scalar':'原始 Guarded Scalar'}
COHORTS={'v2v_development':'V2V（9 段）','v2x_val':'V2X（5 段）'}

def main():
    path=OUT/'SCREEN_SELECTION.json';s=json.loads(path.read_text())
    verify_path=OUT/'SELECTION_VERIFICATION.json';v=json.loads(verify_path.read_text())
    assert s['passed'] and v['passed'] and v['selection_sha256']==sha(path)
    summaries={(r['dataset'],r['condition'],r['arm']):r for r in s['summaries']}
    lines=['# 共享距离检出模型：完整递归对照','']
    if s['advance']:
        lines+=['**本轮通过预先声明的筛选门槛，可以进入剩余完整数据对照；尚不能写成跨数据集稳定优势。**','']
    else:
        failed=[g for g in s['gates'] if not g['passes']]
        lines += [f"**本轮未通过预先声明的筛选门槛：8 项比较中有 {len(failed)} 项失败。关闭这一组距离模型实验，不继续调整曲线或扩大该候选实验。**",'']
    lines+=['本实验回答：原始 GCE 的漏检惩罚是否部分来自固定检出概率 0.9 的失配，以及共享同一距离模型后，GCE 是否仍优于 No-age 与 Guarded Scalar。传感器模型带来的改善不能单独算成 GCE 的融合贡献。','',
        '## 固定模型与比较方式','',
        '采用上一轮冻结的单调 logistic 检出曲线。V2V 使用整段录制排除后的拟合参数；V2X 原样迁移全部 9 段 V2V 的拟合。所有局部更新和既有负证据支持均使用同一个质量接口。近距离概率可超过 0.9；不裁剪、不利用 V2X 重拟合，也不改位姿、真值、检测、空间归一化、曲率保护、关联、通信或提取规则。','',
        '三种方法各自运行完整递归，并各配距离模型和拟合常数模型。筛选只使用 9 段 V2V 与 5 段 V2X；`v2xt_0001` 仅作为事先指定的机制案例。全部数据都已暴露，不构成新的独立泛化证据。','',
        '## 全部筛选结果','',
        'OSPA 单位为米；先对每段全部机器人帧求均值，再对序列等权平均。Reliable 与 Intermittent 分列，数值越低越好。','',
        '| 方法 | V2V Reliable | V2V Intermittent | V2X Reliable | V2X Intermittent |',
        '|---|---:|---:|---:|---:|']
    arms=[GCE,GCE+'_range','marked_lineage_range',GCE+'_guarded_scalar_range',GCE+'_constant','marked_lineage_constant',GCE+'_guarded_scalar_constant']
    for arm in arms:
        values=[summaries[d,c,arm]['sequence_macro']['ospa'] for d in COHORTS for c in ['reliable','intermittent']]
        lines.append('| '+NAMES[arm]+' | '+' | '.join(f'{x:.6f}' for x in values)+' |')
    lines+=['','距离 GCE 必须在两个数据集各自平均两种链路后，分别优于四个对照。下表是距离 GCE 减去对照的 OSPA，负值表示改善。','',
        '| 数据集 | 对照 | 平均差值（米） | 通过 |','|---|---|---:|---|']
    for g in s['gates']:lines.append(f"| {COHORTS[g['dataset']]} | {NAMES[g['reference']]} | {g['mean_delta']:+.9f} | {'是' if g['passes'] else '否'} |")
    lines+=['','## 机制案例','',
        '目标为 `v2xt_0001` 的真值 ID 5、帧 53–122。用既定 2 米匹配统计两台机器人输出，分母为 140 个机器人帧；该统计不参与筛选。','',
        '| 方法 | Reliable 检出帧 | Intermittent 检出帧 |','|---|---:|---:|']
    mechanism={(r['condition'],r['arm']):r for r in s['mechanism']}
    for arm in [GCE,'marked_lineage',GCE+'_guarded_scalar']+arms[1:]:
        values=[mechanism[c,arm]['detected_target_robot_frames_53_122'] for c in ['reliable','intermittent']]
        lines.append(f'| {NAMES[arm]} | {values[0]}/140 | {values[1]}/140 |')
    lines+=['','## GOSPA 分解与通信','',
        '以下对每个序列和两种链路等权平均。定位、漏检和虚警为 GOSPA 的平方代价分量；通信列为每段平均字节数，实际报文长度随各自递归保留的分量数变化。','',
        '| 数据集 | 方法 | GOSPA | 定位² | 漏检² | 虚警² | 原始字节 | 送达原始字节 | 填充后发送字节 |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for dataset in COHORTS:
        for arm in arms:
            values={k:sum(summaries[dataset,c,arm]['sequence_macro'][k] for c in ['reliable','intermittent'])/2
                for k in ['gospa','loc2','miss2','false2','raw_bytes','delivered_raw_bytes','wire_bytes']}
            lines.append('| '+COHORTS[dataset]+' | '+NAMES[arm]+' | '+' | '.join(f'{values[k]:.6f}' if k in ['gospa','loc2','miss2','false2'] else f'{values[k]:.1f}' for k in values)+' |')
    lines+=['','## 验证范围与限制','',
        f"- 192 个新增完整递归结果；核验 {v['native_robot_frames']:,} 个机器人帧。每个原生进程均有退出状态、完成标记和保存结果。",
        '- 两个预检序列、两种链路、三种原始方法，共 12 个结果与冻结参考逐字段精确复现（不比较运行时间）。',
        '- 解析局部更新、全部预测分量的实际检出概率、局部存在概率、负证据支持、正证据标记、全局标签匹配、完整 Gaussian 融合、输出与通信均由保存结果重建核验。No-age 增加的来源与完整协方差日志仅供审计，不增加通信载荷。',
        f"- 独立选择脚本以标准库求和及原生 OSPA 数组重算全部门槛，验证 {v['protected_files']:,} 个文件哈希。这里的独立指计算重建，不指独立团队或新的人工标注验证。",
        '- 检出曲线拟合的是既定二维匹配召回，包含遮挡、检测遗漏和配准偏差等影响；不能解释为真实可见性概率。机制案例改善也不能替代完整序列比较。',
        '- 局部更新沿用原实现，在高斯分量的预测均值处计算检出概率；本实验没有验证对空间可见性函数作完整积分的观测模型。',
        '- 既有论文、闭合实验、检测器、地图与真值均未修改；本轮结果不自动产生论文中的新贡献声明。','',
        '全部分段与链路结果见 `ALL_SCREEN_SCORES.csv`，原始审计见 `audit_range_detection_preflight.json`、`audit_range_detection_screen.json`；筛选与复算见 `SCREEN_SELECTION.json`、`SELECTION_VERIFICATION.json`。','']
    destination=OUT/'RESULTS_CN.md';assert not destination.exists();destination.write_text('\n'.join(lines))
    rows=[r for r in s['rows'] if r['dataset'] in COHORTS and r['arm'] in arms]
    assert len(rows)==196
    csvpath=OUT/'ALL_SCREEN_SCORES.csv'
    with csvpath.open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
    receipt=dict(selection_sha256=sha(path),verification_sha256=sha(verify_path),builder_sha256=sha(Path(__file__)),
        report_sha256=sha(destination),all_screen_scores_sha256=sha(csvpath))
    (OUT/'REPORT_BUILD.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('BUILT COMPLETE RANGE DETECTION REPORT; advance',s['advance'])

if __name__=='__main__':main()
