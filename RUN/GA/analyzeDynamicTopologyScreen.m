function [findingsPath, findings] = ...
    analyzeDynamicTopologyScreen(matPath, findingsPath)
% ANALYZEDYNAMICTOPOLOGYSCREEN Paired audit for the exploratory D12 screen.
%
% This post-processor deliberately treats N<=3 as preliminary evidence. It
% reports per-seed direction consistency, practical effect sizes, attempted
% byte matching, topology feasibility, and the residual oracle gap.

if nargin < 1 || isempty(matPath)
    matPath = findLatestScreenMat(fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'full_n3'));
end
loaded = load(matPath);
if ~isfield(loaded, 'summary')
    error('The MAT file does not contain a summary struct: %s', matPath);
end
summary = loaded.summary;
if nargin < 2 || isempty(findingsPath)
    findingsPath = fullfile(fileparts(matPath), ...
        'DYNAMIC_TOPOLOGY_FINDINGS_CN.md');
end

staticIdx = findArm(summary, 'robust-static');
discrepancyIdx = findArm(summary, 'discrepancy');
consensusOracleIdx = findArm(summary, 'oracle-consensus');
truthOracleIdx = findArm(summary, 'oracle-truth');
required = [staticIdx, discrepancyIdx, ...
    consensusOracleIdx, truthOracleIdx];
if numel(required) ~= 4 || any(required < 1)
    error('The result does not contain the four registered D12 arms.');
end

findings = struct();
findings.sourceMat = matPath;
findings.seedCount = numel(summary.seeds);
findings.seeds = summary.seeds;
findings.focusWindowName = ...
    summary.records(1, 1).focusWindowName;
findings.focusWindow = summary.records(1, 1).focusWindow;
findings.baselineBoundary = [ ...
    'The robust-static arm is selected only by all-time geometry and ', ...
    'link distance. It is not an exhaustive offline performance sweep ', ...
    'over the 48 fixed D12 candidates.'];
findings.discrepancyVsStatic = pairedComparison( ...
    summary, staticIdx, discrepancyIdx);
findings.consensusOracleVsDiscrepancy = pairedComparison( ...
    summary, discrepancyIdx, consensusOracleIdx);
findings.truthOracleVsDiscrepancy = pairedComparison( ...
    summary, discrepancyIdx, truthOracleIdx);
findings.classification = classifyFindings(findings, summary);
findings.generatedAt = datestr(now, 31);

writeFindings(findingsPath, findings, summary, ...
    staticIdx, discrepancyIdx, consensusOracleIdx, truthOracleIdx);
end

function comparison = pairedComparison(summary, referenceIdx, candidateIdx)
reference = summary.records(:, referenceIdx);
candidate = summary.records(:, candidateIdx);
referenceFocusEospa = reshape([reference.focusEospa], 1, []);
candidateFocusEospa = reshape([candidate.focusEospa], 1, []);
referenceFocusConsensus = reshape( ...
    [reference.focusPosteriorConsensus], 1, []);
candidateFocusConsensus = reshape( ...
    [candidate.focusPosteriorConsensus], 1, []);
referenceBytes = reshape([reference.attemptedBytes], 1, []);
candidateBytes = reshape([candidate.attemptedBytes], 1, []);

comparison.referenceMode = reference(1).armMode;
comparison.candidateMode = candidate(1).armMode;
comparison.focusEospaImprovementPercent = percentImprovement( ...
    referenceFocusEospa, candidateFocusEospa);
comparison.focusConsensusImprovementPercent = percentImprovement( ...
    referenceFocusConsensus, candidateFocusConsensus);
comparison.attemptedByteMismatchPercent = 100 * ...
    abs(candidateBytes - referenceBytes) ./ max(referenceBytes, 1);
comparison.trackingWinCount = sum( ...
    comparison.focusEospaImprovementPercent > 0);
comparison.consensusWinCount = sum( ...
    comparison.focusConsensusImprovementPercent > 0);
comparison.byteMatchedCount = sum( ...
    comparison.attemptedByteMismatchPercent <= 2);
comparison.meanFocusEospaImprovementPercent = mean( ...
    comparison.focusEospaImprovementPercent);
comparison.meanFocusConsensusImprovementPercent = mean( ...
    comparison.focusConsensusImprovementPercent);
comparison.meanAttemptedByteMismatchPercent = mean( ...
    comparison.attemptedByteMismatchPercent);
comparison.maxAttemptedByteMismatchPercent = max( ...
    comparison.attemptedByteMismatchPercent);
comparison.topologyInfeasibleRate = max( ...
    reshape([candidate.topologyInfeasibleRate], 1, []));
comparison.meanTopologyChurnRate = meanOptionalField( ...
    candidate, 'topologyChurnRate');
comparison.meanDistinctCandidateCount = meanOptionalField( ...
    candidate, 'distinctCandidateCount');
comparison.maxDistinctCandidateCount = maxOptionalField( ...
    candidate, 'distinctCandidateCount');
end

function classification = classifyFindings(findings, summary)
comparison = findings.discrepancyVsStatic;
residualConsensus = findings.consensusOracleVsDiscrepancy;
residualTracking = findings.truthOracleVsDiscrepancy;
classification = struct();
classification.status = 'preliminary-inconclusive';
classification.reason = [ ...
    '三次 trial 不能建立论文级效果量或置信区间。'];
if findings.seedCount > 3
    error('This exploratory analyzer is scoped to at most three trials.');
end
if comparison.topologyInfeasibleRate > 0
    classification.status = 'invalid-topology';
    classification.reason = ...
        '至少一个被选择的拓扑被标记为不可行。';
    return;
end
if comparison.byteMatchedCount < findings.seedCount
    classification.status = 'cost-mismatch';
    classification.reason = [ ...
        '至少一个配对 trial 超过 attempted bytes 的 2% 匹配容差。'];
    return;
end

trackingSignal = ...
    comparison.meanFocusEospaImprovementPercent >= 5 && ...
    comparison.trackingWinCount == findings.seedCount;
consensusSignal = ...
    comparison.meanFocusConsensusImprovementPercent >= 10 && ...
    comparison.consensusWinCount == findings.seedCount;
if ~(trackingSignal || consensusSignal)
    classification.status = 'no-practical-dynamic-signal';
    classification.reason = [ ...
        '可部署的 discrepancy 策略没有在全部 seeds 上稳定达到 5% ', ...
        'tracking 或 10% posterior-consensus 的探索门槛。'];
    return;
end

consensusOracleDominated = ...
    residualConsensus.trackingWinCount == 0 && ...
    residualConsensus.consensusWinCount == 0;
truthOracleDominated = ...
    residualTracking.trackingWinCount == 0 && ...
    residualTracking.consensusWinCount == 0;
residualIsMaterial = ...
    residualTracking.meanFocusEospaImprovementPercent >= 5 || ...
    residualConsensus.meanFocusConsensusImprovementPercent >= 10;
if consensusOracleDominated && truthOracleDominated
    classification.status = 'diagnostic-oracle-dominated';
    classification.reason = [ ...
        '相对 geometry-static 存在候选动态信号，但两个一步诊断 ', ...
        'oracle 在每个 seed 的 tracking 与 posterior 指标上都被 ', ...
        'discrepancy 支配。它们不能充当上界或 GNN 教师，也不能据此', ...
        '推出解析策略已经充分；应暂停 GNN，先重做短时闭环价值定义并', ...
        '补齐最强固定图基线。'];
elseif residualIsMaterial
    classification.status = 'residual-oracle-gap';
    classification.reason = [ ...
        '相对 geometry-static 存在候选动态信号，且诊断 oracle ', ...
        '保留了有实际意义的剩余差距；在补齐最强固定图基线后，', ...
        '学习型边价值预测仍值得研究。'];
else
    classification.status = 'no-registered-residual-gap';
    classification.reason = [ ...
        '相对 geometry-static 存在候选动态信号，但当前一步诊断策略 ', ...
        '没有在 discrepancy 之上留下预注册的剩余差距。当前只支持 ', ...
        '优先保留解析策略，不支持解析策略已经最优的结论。'];
end

if numel(summary.seeds) < 10
    classification.reason = [classification.reason, ...
        ' 这是 N<=3 的方向信号，不是推断性证据。'];
end
end

function writeFindings(path, findings, summary, ...
    staticIdx, discrepancyIdx, consensusOracleIdx, truthOracleIdx)
fid = fopen(path, 'w');
if fid < 0
    error('Could not open findings path: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# D12 动态拓扑三次配对筛查\n\n');
fprintf(fid, '- 数据：`%s`\n', findings.sourceMat);
recordsCsvPath = fullfile(fileparts(path), ...
    'DYNAMIC_TOPOLOGY_RECORDS.csv');
if exist(recordsCsvPath, 'file')
    fprintf(fid, '- 可追踪逐臂记录：`%s`\n', recordsCsvPath);
end
fprintf(fid, '- Seeds：`%s`\n', mat2str(findings.seeds));
fprintf(fid, '- 重点窗口：%s，`%s`\n', ...
    findings.focusWindowName, mat2str(findings.focusWindow));
fprintf(fid, '- 自动分类：`%s`\n', findings.classification.status);
fprintf(fid, '- 解释：%s\n\n', findings.classification.reason);

fprintf(fid, '## 重点窗口的主比较\n\n');
fprintf(fid, ['| Seed | Geometry-static focus E-OSPA | ', ...
    'Discrepancy focus E-OSPA | ', ...
    'Tracking 改善 | Static posterior 分歧 | ', ...
    'Discrepancy posterior 分歧 | Consensus 改善 | Bytes 偏差 |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|--:|\n');
comparison = findings.discrepancyVsStatic;
for seedIdx = 1:findings.seedCount
    static = summary.records(seedIdx, staticIdx);
    discrepancy = summary.records(seedIdx, discrepancyIdx);
    fprintf(fid, ...
        '| %d | %.4f | %.4f | %.2f%% | %.4f | %.4f | %.2f%% | %.2f%% |\n', ...
        findings.seeds(seedIdx), ...
        static.focusEospa, discrepancy.focusEospa, ...
        comparison.focusEospaImprovementPercent(seedIdx), ...
        static.focusPosteriorConsensus, ...
        discrepancy.focusPosteriorConsensus, ...
        comparison.focusConsensusImprovementPercent(seedIdx), ...
        comparison.attemptedByteMismatchPercent(seedIdx));
end

fprintf(fid, '\n## 每个 seed 的完整四臂记录\n\n');
fprintf(fid, ['| Seed | Arm | Focus E-OSPA | Focus posterior 分歧 | ', ...
    'Attempted bytes | 相对 static bytes | Churn | 不同候选数 |\n']);
fprintf(fid, '|--:|:--|--:|--:|--:|--:|--:|--:|\n');
indices = [staticIdx, discrepancyIdx, consensusOracleIdx, truthOracleIdx];
for seedIdx = 1:findings.seedCount
    staticBytes = summary.records(seedIdx, staticIdx).attemptedBytes;
    for armIdx = indices
        arm = summary.records(seedIdx, armIdx);
        byteMismatch = 100 * abs(arm.attemptedBytes - staticBytes) / ...
            max(staticBytes, 1);
        fprintf(fid, ...
            '| %d | %s | %.4f | %.4f | %.0f | %.2f%% | %.4f | %.0f |\n', ...
            findings.seeds(seedIdx), displayArmName(arm), ...
            arm.focusEospa, arm.focusPosteriorConsensus, ...
            arm.attemptedBytes, byteMismatch, ...
            optionalField(arm, 'topologyChurnRate', NaN), ...
            optionalField(arm, 'distinctCandidateCount', NaN));
    end
end

fprintf(fid, '\n## 配对汇总\n\n');
writeComparison(fid, 'Posterior-discrepancy vs static', ...
    findings.discrepancyVsStatic);
writeComparison(fid, 'Consensus oracle vs posterior-discrepancy', ...
    findings.consensusOracleVsDiscrepancy);
writeComparison(fid, 'Truth oracle vs posterior-discrepancy', ...
    findings.truthOracleVsDiscrepancy);

fprintf(fid, '\n## 四个 arm 的均值\n\n');
fprintf(fid, ['| Arm | Focus E-OSPA | Focus posterior 分歧 | ', ...
    'Attempted bytes | Churn | 不同候选数 | 不可行率 | 总时间 s |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|\n');
for armIdx = indices
    arm = summary.aggregate(armIdx);
    fprintf(fid, ...
        '| %s | %.4f | %.4f | %.0f | %.4f | %.1f | %.4f | %.2f |\n', ...
        displayArmName(arm), arm.focusEospa, arm.focusPosteriorConsensus, ...
        arm.attemptedBytes, ...
        optionalField(arm, 'topologyChurnRate', NaN), ...
        optionalField(arm, 'distinctCandidateCount', NaN), ...
        arm.topologyInfeasibleRate, arm.elapsedSeconds);
end

fprintf(fid, '\n## 证据边界\n\n');
fprintf(fid, ['- 三次 trial 只用于判断方向，不提供可靠置信区间，', ...
    '也不能支持论文级显著性主张。\n']);
fprintf(fid, ['- “Exact one-step oracle”只是在每一步精确枚举动作，', ...
    '不是整段闭环跟踪的全局性能上界。\n']);
fprintf(fid, ['- 当前 static 是按全时段几何距离选择的可行固定图，', ...
    '尚未穷举 48 个固定候选的 held-out tracking 表现；因此当前结果', ...
    '不能证明动态策略击败了最强静态基线。\n']);
fprintf(fid, ['- 两个一步诊断策略只访问了很少的不同候选图；当它们被', ...
    '可部署启发式支配时，不能把“负 oracle gap”解释成解析策略最优。\n']);
fprintf(fid, ['- 融合使用 componentwise powered-GM 近似；它保留多模态，', ...
    '但不是任意 Gaussian mixture 幂的精确闭式实现。\n']);
fprintf(fid, ['- attempted bytes 仅包含 payload；控制、ACK 与拓扑协商', ...
    '开销仍需在后续系统实验中单列。\n']);
end

function writeComparison(fid, label, comparison)
fprintf(fid, '- %s：\n', label);
fprintf(fid, '  - focus E-OSPA 平均改善：%.2f%%，胜出 %d/%d seeds；\n', ...
    comparison.meanFocusEospaImprovementPercent, ...
    comparison.trackingWinCount, ...
    numel(comparison.focusEospaImprovementPercent));
fprintf(fid, '  - focus posterior 分歧平均改善：%.2f%%，胜出 %d/%d seeds；\n', ...
    comparison.meanFocusConsensusImprovementPercent, ...
    comparison.consensusWinCount, ...
    numel(comparison.focusConsensusImprovementPercent));
fprintf(fid, '  - attempted-byte 平均/最大偏差：%.2f%% / %.2f%%；\n', ...
    comparison.meanAttemptedByteMismatchPercent, ...
    comparison.maxAttemptedByteMismatchPercent);
fprintf(fid, '  - topology 不可行率上界：%.4f。\n', ...
    comparison.topologyInfeasibleRate);
fprintf(fid, '  - 候选策略平均 churn / 不同候选数：%.4f / %.1f。\n', ...
    comparison.meanTopologyChurnRate, ...
    comparison.meanDistinctCandidateCount);
end

function value = meanOptionalField(records, fieldName)
if ~isfield(records, fieldName)
    value = NaN;
    return;
end
value = mean(reshape([records.(fieldName)], 1, []));
end

function value = maxOptionalField(records, fieldName)
if ~isfield(records, fieldName)
    value = NaN;
    return;
end
value = max(reshape([records.(fieldName)], 1, []));
end

function value = optionalField(record, fieldName, fallback)
if isfield(record, fieldName)
    value = record.(fieldName);
else
    value = fallback;
end
end

function name = displayArmName(record)
if isfield(record, 'mode')
    mode = record.mode;
else
    mode = optionalField(record, 'armMode', '');
end
if strcmp(mode, 'robust-static')
    name = 'All-time geometry static';
elseif isfield(record, 'name')
    name = record.name;
else
    name = record.armName;
end
end

function values = percentImprovement(reference, candidate)
values = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function idx = findArm(summary, mode)
idx = find(strcmp({summary.aggregate.mode}, mode), 1);
if isempty(idx)
    idx = 0;
end
end

function path = findLatestScreenMat(directory)
files = dir(fullfile(directory, ...
    'DYNAMIC_TOPOLOGY_ORACLE_GAP_*.mat'));
if isempty(files)
    error('No dynamic-topology MAT result found in %s.', directory);
end
[~, order] = sort([files.datenum], 'descend');
path = fullfile(directory, files(order(1)).name);
end
