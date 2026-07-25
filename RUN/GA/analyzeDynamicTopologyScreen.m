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
end

function classification = classifyFindings(findings, summary)
comparison = findings.discrepancyVsStatic;
residualConsensus = findings.consensusOracleVsDiscrepancy;
residualTracking = findings.truthOracleVsDiscrepancy;
classification = struct();
classification.status = 'preliminary-inconclusive';
classification.reason = [ ...
    'Three trials cannot establish a paper-level effect or confidence ', ...
    'interval.'];
if findings.seedCount > 3
    error('This exploratory analyzer is scoped to at most three trials.');
end
if comparison.topologyInfeasibleRate > 0
    classification.status = 'invalid-topology';
    classification.reason = ...
        'At least one selected topology was marked infeasible.';
    return;
end
if comparison.byteMatchedCount < findings.seedCount
    classification.status = 'cost-mismatch';
    classification.reason = [ ...
        'At least one paired trial exceeds the 2 percent attempted-byte ', ...
        'matching tolerance.'];
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
        'The deployable discrepancy policy does not meet either the ', ...
        '5 percent tracking or 10 percent posterior-consensus ', ...
        'exploratory threshold consistently across all seeds.'];
    return;
end

residualIsMaterial = ...
    residualTracking.meanFocusEospaImprovementPercent >= 5 || ...
    residualConsensus.meanFocusConsensusImprovementPercent >= 10;
if residualIsMaterial
    classification.status = 'residual-oracle-gap';
    classification.reason = [ ...
        'A deployable dynamic signal exists and the diagnostic oracle ', ...
        'retains a practical residual gap; learned edge value prediction ', ...
        'remains worth investigating.'];
else
    classification.status = 'analytic-policy-sufficient';
    classification.reason = [ ...
        'A deployable dynamic signal exists, but the one-step diagnostic ', ...
        'oracles do not retain the registered residual margin over the ', ...
        'posterior-discrepancy heuristic. Prefer the analytic policy ', ...
        'before introducing a GNN.'];
end

if numel(summary.seeds) < 10
    classification.reason = [classification.reason, ...
        ' This is an N<=3 direction signal, not inferential evidence.'];
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
fprintf(fid, '- Seeds：`%s`\n', mat2str(findings.seeds));
fprintf(fid, '- 重点窗口：%s，`%s`\n', ...
    findings.focusWindowName, mat2str(findings.focusWindow));
fprintf(fid, '- 自动分类：`%s`\n', findings.classification.status);
fprintf(fid, '- 解释：%s\n\n', findings.classification.reason);

fprintf(fid, ['| Seed | Static focus E-OSPA | Discrepancy focus E-OSPA | ', ...
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

fprintf(fid, '\n## 配对汇总\n\n');
writeComparison(fid, 'Posterior-discrepancy vs static', ...
    findings.discrepancyVsStatic);
writeComparison(fid, 'Consensus oracle vs posterior-discrepancy', ...
    findings.consensusOracleVsDiscrepancy);
writeComparison(fid, 'Truth oracle vs posterior-discrepancy', ...
    findings.truthOracleVsDiscrepancy);

fprintf(fid, '\n## 四个 arm 的均值\n\n');
fprintf(fid, ['| Arm | Focus E-OSPA | Focus posterior 分歧 | ', ...
    'Attempted bytes | 不可行率 | 总时间 s |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|\n');
indices = [staticIdx, discrepancyIdx, consensusOracleIdx, truthOracleIdx];
for armIdx = indices
    arm = summary.aggregate(armIdx);
    fprintf(fid, '| %s | %.4f | %.4f | %.0f | %.4f | %.2f |\n', ...
        arm.name, arm.focusEospa, arm.focusPosteriorConsensus, ...
        arm.attemptedBytes, arm.topologyInfeasibleRate, ...
        arm.elapsedSeconds);
end

fprintf(fid, '\n## 证据边界\n\n');
fprintf(fid, ['- 三次 trial 只用于判断方向，不提供可靠置信区间，', ...
    '也不能支持论文级显著性主张。\n']);
fprintf(fid, ['- “Exact one-step oracle”只是在每一步精确枚举动作，', ...
    '不是整段闭环跟踪的全局性能上界。\n']);
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
