"""Present the complete frozen screen and its independent verification."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
NAMES={'original':'原门控','joint_mark':'联合概率 × 原标记','conditional':'条件关联，无额外标记','joint':'联合关联，无额外标记（主）'}
DATA={'v2v_development':'V2V4Real 原开发 9 段','v2x_val':'V2X-Real 已见 5 段'}


def main():
    report_path=OUT/'SCREEN_RESULTS.json';verification_path=OUT/'SCREEN_VERIFICATION.json'
    report=json.loads(report_path.read_text());verification=json.loads(verification_path.read_text())
    assert report['passed'] and verification['passed'] and verification['report_sha256']==sha(report_path)
    assert verification['advance_to_recursion']==report['advance_to_recursion']
    arithmetic_path=OUT/'WEIGHT_ARITHMETIC_VERIFICATION.json'
    arithmetic=json.loads(arithmetic_path.read_text())
    assert arithmetic['passed'] and arithmetic['report_sha256']==sha(report_path)
    passed=sum(g['passed'] for g in report['gates'])
    verdict=('四项继续比较全部通过，允许按固定协议构建并验证完整递推实验。'
             if report['advance_to_recursion'] else
             f'四项继续比较通过 {passed} 项；停止这一个联合准入规则，不进入完整递推或调整系数。')
    lines=['# 联合当前检测准入：固定输入检验','',verdict,'',
        '本轮是一次融合输入上的替换诊断。替代输出没有进入下一帧，因此下表不代表新方法的完整跟踪结果。所有数据已经参与此前开发，不能称为独立测试。','',
        '唯一主规则是正准入量 `r_plus × a`，其中 `a` 是以目标存在为条件的当前检测关联概率，`r_plus` 是本地更新后的存在率。正检测分数已经进入本地似然；主规则不再额外乘一次正标记系数。负准入、名义 pD=0.9、曲率保护、历史权重及提取规则保持既定值。','',
        '这个概率解释不保证跟踪更准确。后验存在率含有历史信念，LBP 是近似关联，错误而高置信的标签仍可通过门控。','',
        '## 原 GCE 状态上的完整比较','',
        '每段等权，两个链路单独列出。OSPA 单位为 m；漏检、虚假和定位为 GOSPA 平方代价，单位 m²。','',
        '| 数据 | 准入规则 | 可靠 OSPA | 间歇 OSPA | 两链路均值 | 漏检代价 | 虚假代价 | 定位代价 | 输出数量 |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for backend in ['GCE','Guarded Scalar']:
        if backend!='GCE':
            lines+=['','## Guarded Scalar 状态上的诊断对照','',
                '这些源状态来自 Guarded Scalar 自己已完成的递推。它们用于检查同一准入替换的状态依赖，不参与主规则选择，也不构成同一输入上的 GCE/Scalar 归因实验。','',
                '| 数据 | 准入规则 | 可靠 OSPA | 间歇 OSPA | 两链路均值 | 漏检代价 | 虚假代价 | 定位代价 | 输出数量 |',
                '|---|---|---:|---:|---:|---:|---:|---:|---:|']
        for row in report['aggregate']:
            if row['backend']!=backend:continue
            cond=row['conditions'];mean=lambda key:sum(v[key] for v in cond.values())/2
            lines.append(f"| {DATA[row['dataset']]} | {NAMES[row['rule']]} | {cond['reliable']['ospa']:.6f} | {cond['intermittent']['ospa']:.6f} | {row['selection_mean_ospa']:.6f} | {mean('miss2'):.3f} | {mean('false2'):.3f} | {mean('loc2'):.3f} | {mean('outputCount'):.3f} |")
    lines+=['','## 预先固定的四项继续条件','',
        '只使用原 GCE 访问过的状态。差值为主规则减参照；必须四项都小于零，且不能用次要规则替换失败主规则。','',
        '| 数据 | 参照 | ΔOSPA | 通过 |','|---|---|---:|---|']
    for gate in report['gates']:
        lines.append(f"| {DATA[gate['dataset']]} | {NAMES[gate['reference']]} | {gate['difference']:+.9f} | {'是' if gate['passed'] else '否'} |")
    W=[d['raw_weights'] for d in report['diagnostics'] if d['raw_weights']['available']]
    raw_rows=sum(d['local_rows_with_W'] for d in W);raw_values=sum(d['values'] for d in W)
    columns=max(d['maximum_joint_detection_column_sum'] for d in W)
    lines+=['','## 检验范围与边界','',
        f"56 个已完成的源运行全部保留：14 段 × 两链路 × 两种源方法。四组替换共 {report['robot_frames']:,} 个机器人帧、224 行序列评分。原始位置集合、数量及 OSPA 在 {report['original_parity_robot_frames']:,} 个机器人帧上通过复现。",'',
        f"生产诊断从本地预测/后验矩重建高斯比值。独立验证从已保存的归一化融合密度出发，减去旧传输残差再加入新残差，用 Cholesky 积分重算 {verification['fusion_distributions']:,} 个完整分布；另写基数递推和 OSPA/GOSPA 计算，核对全部替代输出及四项判断。它是同一工作流程内的独立实现复算，不是第三方复现。",'',
        f"有原始 W 的 {len(W)} 个源运行核查了 {raw_rows:,} 条非空关联记录、{raw_values:,} 个权重值。其每检测联合关联概率列和最大为 {columns:.12f}，原值保留，未裁剪。其余 {56-len(W)} 个较早源运行没有保存 W，只能沿用已审计的本地关联质量记录；本报告明确保留这一信息边界。六个有限状态穷举夹具核对了联合事件的概率恒等式。",'',
        f"原始 W 审计中的一次 NumPy 矩阵乘法报告了运行时警告，原日志完整保留。另以逐元素乘积和 `math.fsum` 检查了全部 {arithmetic['checked_rows']:,} 条可用记录，确认输入、乘积及和都有限，并复现原标记支持和联合概率。没有改变冻结代码、数据、容差或判据；警告的底层原因未确定。详见 [WEIGHT_ARITHMETIC_VERIFICATION.json](WEIGHT_ARITHMETIC_VERIFICATION.json)。",'',
        '完整分布替换保留源曲率拒绝和不可积回退；原递推源、数据、分数模型及先前结果均以散列保护。固定输入替换没有产生新的递推进程或通信轨迹，因此不能据此宣称新的字节成本、运行速度或长期跟踪收益。','',
        '完整数据见 [ALL_SCREEN_SCORES.csv](ALL_SCREEN_SCORES.csv)，判据与范围见 [PROTOCOL.md](PROTOCOL.md)，复算证据见 [SCREEN_VERIFICATION.json](SCREEN_VERIFICATION.json)。','']
    target=OUT/'RESULTS_CN.md';assert not target.exists();target.write_text('\n'.join(lines))
    receipt=OUT/'REPORT_BUILD.json';assert not receipt.exists()
    receipt.write_text(json.dumps(dict(passed=True,report_sha256=sha(target),screen_sha256=sha(report_path),
        verification_sha256=sha(verification_path),arithmetic_sha256=sha(arithmetic_path),
        builder_sha256=sha(Path(__file__))),indent=2)+'\n')
    print(verdict,flush=True)


if __name__=='__main__':main()
