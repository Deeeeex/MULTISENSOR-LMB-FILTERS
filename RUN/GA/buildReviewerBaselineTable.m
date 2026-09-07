function buildReviewerBaselineTable(corePath, externalPath, outputDir, enforceCoreValues, ...
    paperNetworkRowsPath, paperLocalRowsPath)
% BUILDREVIEWERBASELINETABLE Combine matched core and literature-adaptation runs.
% The output keeps the literature rows explicitly labeled as adaptations.
% Paper row paths are optional so synthetic tests cannot overwrite the
% manuscript; the formal validation pipeline supplies them explicitly.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
if nargin < 1 || isempty(corePath)
    corePath = fullfile(projectRoot, 'RUN', 'GA', 'gospa_validation', ...
        'gospa_core_n50_summary.mat');
end
if nargin < 2 || isempty(externalPath)
    externalPath = fullfile(projectRoot, 'RUN', 'GA', ...
        'reviewer_baselines_validation', 'reviewer_baselines_n50_summary.mat');
end
if nargin < 3 || isempty(outputDir)
    outputDir = fullfile(projectRoot, 'RUN', 'GA', 'reviewer_baselines_validation');
end
if nargin < 4 || isempty(enforceCoreValues)
    enforceCoreValues = true;
end
if nargin < 5
    paperNetworkRowsPath = '';
end
if nargin < 6
    paperLocalRowsPath = '';
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

coreData = load(corePath, 'summary');
externalData = load(externalPath, 'aggregate');
core = coreData.summary;
external = externalData.aggregate;

expectedCore = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
expectedExternal = {'Zheng-style subdensity GA-LMB adaptation', ...
    'Gao-style local-trust GA-LMB adaptation'};
assert(isequal(core.armNames, expectedCore), 'Unexpected core arm order.');
assert(isequal(external.armNames, expectedExternal), ...
    'Unexpected reviewer-baseline arm order.');
assert(isequal(core.trialSeeds, external.trialSeeds), 'Trial seeds do not match.');
assert(isfield(core, 'commConfig') && isfield(external, 'commConfig') && ...
    isequaln(core.commConfig, external.commConfig), ...
    'Core and adaptation communication configurations do not match.');
validateAdaptationConfigurations(external.arms);
assert(max(abs(core.pDropBySensorTrials(:) - external.pDropBySensorTrials(:))) < 1e-12, ...
    'Communication realizations do not match.');

combined.armNames = {'Fixed Metropolis', ...
    'Zheng-style subdensity GA-LMB adaptation', ...
    'Gao-style local-trust GA-LMB adaptation', ...
    'Balanced mode', 'Cardinality-critical mode'};
combined.trialSeeds = core.trialSeeds;
combined.pDropBySensorTrials = core.pDropBySensorTrials;
combined.consensusTrials.ospa = joinNetworkMetric(core.consensusTrials.ospa, ...
    external.consensusTrials.ospa);
combined.consensusTrials.gospa = joinNetworkMetric(core.consensusTrials.gospa, ...
    external.consensusTrials.gospa);
combined.consensusTrials.pos = joinNetworkMetric(core.consensusTrials.pos, ...
    external.consensusTrials.pos);
combined.consensusTrials.card = joinNetworkMetric(core.consensusTrials.card, ...
    external.consensusTrials.card);
combined.localTrials.eOspa = joinLocalMetric(core.localTrials.eOspa, ...
    external.localTrials.eOspa);
combined.localTrials.rmse = joinLocalMetric(core.localTrials.rmse, ...
    external.localTrials.rmse);
combined.localTrials.cardErr = joinLocalMetric(core.localTrials.cardErr, ...
    external.localTrials.cardErr);
combined.localPerTrial.eOspa = collapseSensors(combined.localTrials.eOspa, false);
combined.localPerTrial.rmse = collapseSensors(combined.localTrials.rmse, true);
combined.localPerTrial.cardErr = collapseSensors(combined.localTrials.cardErr, false);
combined.runtime.filterSeconds = joinNetworkMetric(core.runtime.filterSeconds, ...
    external.runtime.filterSeconds);

if enforceCoreValues
    validateCoreValues(combined);
end

combinedPath = fullfile(outputDir, 'reviewer_baseline_table_n50_combined.mat');
summaryCsvPath = fullfile(outputDir, 'reviewer_baseline_table_n50.csv');
trialCsvPath = fullfile(outputDir, 'reviewer_baseline_table_n50_per_trial.csv');
pairwiseCsvPath = fullfile(outputDir, ...
    'reviewer_baseline_proposed_vs_adaptations.csv');
reportPath = fullfile(outputDir, 'reviewer_baseline_table_n50.md');
networkLatexRowsPath = fullfile(outputDir, ...
    'reviewer_baseline_table_n50_network_rows.tex');
localLatexRowsPath = fullfile(outputDir, ...
    'reviewer_baseline_table_n50_local_rows.tex');
save('-mat7-binary', combinedPath, 'combined');
writeSummaryCsv(summaryCsvPath, combined);
writeTrialCsv(trialCsvPath, combined);
writeProposedVsAdaptationCsv(pairwiseCsvPath, combined);
writeReport(reportPath, combined, corePath, externalPath);
writeNetworkLatexRows(networkLatexRowsPath, combined);
writeLocalLatexRows(localLatexRowsPath, combined);
if ~isempty(paperNetworkRowsPath)
    ensureParentDirectory(paperNetworkRowsPath);
    writeNetworkLatexRows(paperNetworkRowsPath, combined);
end
if ~isempty(paperLocalRowsPath)
    ensureParentDirectory(paperLocalRowsPath);
    writeLocalLatexRows(paperLocalRowsPath, combined);
end

fprintf('REVIEWER_BASELINE_TABLE_COMPLETE\n');
fprintf('Combined: %s\n', combinedPath);
fprintf('Summary CSV: %s\n', summaryCsvPath);
fprintf('Per-trial CSV: %s\n', trialCsvPath);
fprintf('Proposed-vs-adaptation CSV: %s\n', pairwiseCsvPath);
fprintf('Report: %s\n', reportPath);
fprintf('Network LaTeX rows: %s\n', networkLatexRowsPath);
fprintf('Local LaTeX rows: %s\n', localLatexRowsPath);
if ~isempty(paperNetworkRowsPath)
    fprintf('Paper network LaTeX rows: %s\n', paperNetworkRowsPath);
end
if ~isempty(paperLocalRowsPath)
    fprintf('Paper local LaTeX rows: %s\n', paperLocalRowsPath);
end
end

function ensureParentDirectory(path)
parentDir = fileparts(path);
if ~isempty(parentDir) && ~exist(parentDir, 'dir')
    mkdir(parentDir);
end
end

function validateAdaptationConfigurations(arms)
assert(numel(arms) == 2, ...
    'Unexpected number of adaptation arm configurations.');
zheng = arms(1).adaptiveFusion;
assert(strcmpi(zheng.method, 'zhengStyleLmb') && zheng.enabled && ...
    zheng.zhengSpatialPrecisionPower == 1 && ...
    zheng.zhengExistenceMarginPower == 1 && ...
    zheng.zhengExistenceMinScore == 0.05, ...
    'Unexpected Zheng-style adaptation configuration.');
gao = arms(2).adaptiveFusion;
assert(strcmpi(gao.method, 'gaoStyleResilient') && gao.enabled && ...
    gao.gaoDistanceScale == 2 && gao.gaoMinCredibility == 0.05 && ...
    gao.gaoSpatialDistancePower == 1 && ...
    gao.gaoExistenceDistancePower == 1 && ~gao.gaoHardReject, ...
    'Unexpected Gao-style adaptation configuration.');
end

function joined = joinNetworkMetric(coreValues, externalValues)
assert(size(coreValues, 1) == size(externalValues, 1), 'Trial counts do not match.');
assert(size(coreValues, 2) == 4, 'Core metric must contain four arms.');
assert(size(externalValues, 2) == 2, 'External metric must contain two arms.');
joined = [coreValues(:, 1), externalValues(:, 1:2), coreValues(:, 3:4)];
end

function joined = joinLocalMetric(coreValues, externalValues)
assert(size(coreValues, 1) == size(externalValues, 1), 'Trial counts do not match.');
assert(size(coreValues, 2) == size(externalValues, 2), 'Sensor counts do not match.');
assert(size(coreValues, 3) == 4, 'Core local metric must contain four arms.');
assert(size(externalValues, 3) == 2, 'External local metric must contain two arms.');
joined = cat(3, coreValues(:, :, 1), externalValues(:, :, 1:2), coreValues(:, :, 3:4));
end

function collapsed = collapseSensors(values, omitNan)
if omitNan
    collapsed = squeeze(mean(values, 2, 'omitnan'));
else
    collapsed = squeeze(mean(values, 2));
end
end

function validateCoreValues(combined)
% The reused core rows must reproduce the canonical N=50 report, not just
% its rounded manuscript OSPA values.  This protects both the primary
% network metrics and the local safeguards used beside the adaptations.
coreIdx = [1, 4, 5];
tolerance = 5.01e-7;
expectedNetwork = [ ...
    2.468973, 1.779218, 1.677524; ... % OSPA
    2.326034, 1.522191, 1.546107; ... % localization
    0.716025, 0.187675, 0.063200];    % cardinality
expectedLocal = [ ...
    2.862938, 2.334915, 2.019842; ... % E-OSPA
    1.649569, 1.605910, 1.720931; ... % RMSE
    1.455125, 0.578775, 0.223700];    % cardinality error

actualNetwork = [ ...
    mean(combined.consensusTrials.ospa(:, coreIdx), 1); ...
    mean(combined.consensusTrials.pos(:, coreIdx), 1, 'omitnan'); ...
    mean(combined.consensusTrials.card(:, coreIdx), 1)];
actualLocal = [ ...
    mean(combined.localPerTrial.eOspa(:, coreIdx), 1); ...
    mean(combined.localPerTrial.rmse(:, coreIdx), 1, 'omitnan'); ...
    mean(combined.localPerTrial.cardErr(:, coreIdx), 1)];

assert(max(abs(actualNetwork(:) - expectedNetwork(:))) <= tolerance, ...
    'Reused core network metrics do not reproduce the canonical N=50 report.');
assert(max(abs(actualLocal(:) - expectedLocal(:))) <= tolerance, ...
    'Reused core local metrics do not reproduce the canonical N=50 report.');
end

function writeSummaryCsv(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, ['arm,ospa_mean,ospa_std,gospa_mean,gospa_std,' ...
    'localization_mean,localization_std,cardinality_mean,cardinality_std,' ...
    'eospa_mean,eospa_std,rmse_mean,rmse_std,carderr_mean,carderr_std,' ...
    'runtime_mean,runtime_std,runtime_cohort\n']);
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    eOspa = combined.localPerTrial.eOspa(:, armIdx);
    rmse = combined.localPerTrial.rmse(:, armIdx);
    cardErr = combined.localPerTrial.cardErr(:, armIdx);
    runtime = combined.runtime.filterSeconds(:, armIdx);
    fprintf(fid, ['%s,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%s\n'], ...
        combined.armNames{armIdx}, mean(ospa), std(ospa), mean(gospa), std(gospa), ...
        mean(pos, 'omitnan'), std(pos), mean(card), std(card), ...
        mean(eOspa), std(eOspa), mean(rmse, 'omitnan'), std(rmse), ...
        mean(cardErr), std(cardErr), mean(runtime), std(runtime), ...
        runtimeCohort(armIdx));
end
end

function writeTrialCsv(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'trial,seed,arm,ospa,gospa,localization,cardinality,eospa,rmse,carderr,runtime_seconds\n');
for trialIdx = 1:numel(combined.trialSeeds)
    for armIdx = 1:numel(combined.armNames)
        fprintf(fid, '%d,%d,%s,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n', ...
            trialIdx, combined.trialSeeds(trialIdx), combined.armNames{armIdx}, ...
            combined.consensusTrials.ospa(trialIdx, armIdx), ...
            combined.consensusTrials.gospa(trialIdx, armIdx), ...
            combined.consensusTrials.pos(trialIdx, armIdx), ...
            combined.consensusTrials.card(trialIdx, armIdx), ...
            combined.localPerTrial.eOspa(trialIdx, armIdx), ...
            combined.localPerTrial.rmse(trialIdx, armIdx), ...
            combined.localPerTrial.cardErr(trialIdx, armIdx), ...
            combined.runtime.filterSeconds(trialIdx, armIdx));
    end
end
end

function writeProposedVsAdaptationCsv(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, ['proposed_arm,adaptation_arm,metric,mean_paired_delta,' ...
    'improved_trials,non_tied_trials,exact_two_sided_sign_p\n']);
metricNames = {'OSPA', 'GOSPA', 'Localization', 'Cardinality'};
metricValues = {combined.consensusTrials.ospa, combined.consensusTrials.gospa, ...
    combined.consensusTrials.pos, combined.consensusTrials.card};
for proposedIdx = [4, 5]
    for adaptationIdx = [2, 3]
        for metricIdx = 1:numel(metricNames)
            delta = metricValues{metricIdx}(:, proposedIdx) - ...
                metricValues{metricIdx}(:, adaptationIdx);
            validDelta = delta(isfinite(delta));
            nonTied = validDelta(validDelta ~= 0);
            fprintf(fid, '%s,%s,%s,%.9f,%d,%d,%.12g\n', ...
                combined.armNames{proposedIdx}, combined.armNames{adaptationIdx}, ...
                metricNames{metricIdx}, mean(validDelta), sum(validDelta < 0), ...
                numel(nonTied), exactTwoSidedSignP(validDelta));
        end
    end
end
end

function writeReport(path, combined, corePath, externalPath)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '# Reviewer-requested baseline comparison\n\n');
fprintf(fid, '- Seeds: 2--51 (50 deterministic paired trials)\n');
fprintf(fid, '- OSPA/GOSPA: c=5, p=2; GOSPA alpha=2\n');
fprintf(fid, '- Ground space: complete extracted kinematic state vector\n');
fprintf(fid, '- Core source: `%s`\n', corePath);
fprintf(fid, '- Adaptation source: `%s`\n', externalPath);
fprintf(fid, '- Zheng/Gao rows are adaptations, not exact source implementations.\n\n');
fprintf(fid, '| Arm | OSPA | GOSPA | Loc. disag. | Card. disp. | E-OSPA | RMSE | CardErr |\n');
fprintf(fid, '|:----|-----:|------:|------------:|------------:|-------:|-----:|--------:|\n');
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    eOspa = combined.localPerTrial.eOspa(:, armIdx);
    rmse = combined.localPerTrial.rmse(:, armIdx);
    cardErr = combined.localPerTrial.cardErr(:, armIdx);
    fprintf(fid, ['| %s | %.3f +/- %.3f | %.3f +/- %.3f | %.3f +/- %.3f | ' ...
        '%.3f +/- %.3f | %.3f +/- %.3f | %.3f +/- %.3f | %.3f +/- %.3f |\n'], ...
        combined.armNames{armIdx}, mean(ospa), std(ospa), mean(gospa), std(gospa), ...
        mean(pos, 'omitnan'), std(pos), mean(card), std(card), ...
        mean(eOspa), std(eOspa), mean(rmse, 'omitnan'), std(rmse), ...
        mean(cardErr), std(cardErr));
end

fprintf(fid, '\n## Paired network-metric sign tests versus Fixed Metropolis\n\n');
fprintf(fid, '| Arm | Metric | Improved trials | Exact two-sided p |\n');
fprintf(fid, '|:----|:-------|----------------:|------------------:|\n');
metricNames = {'OSPA', 'GOSPA', 'Loc. disag.', 'Card. disp.'};
metricValues = {combined.consensusTrials.ospa, combined.consensusTrials.gospa, ...
    combined.consensusTrials.pos, combined.consensusTrials.card};
for armIdx = 2:numel(combined.armNames)
    for metricIdx = 1:numel(metricNames)
        delta = metricValues{metricIdx}(:, armIdx) - metricValues{metricIdx}(:, 1);
        fprintf(fid, '| %s | %s | %d/50 | %.3g |\n', ...
            combined.armNames{armIdx}, metricNames{metricIdx}, ...
            sum(delta < 0), exactTwoSidedSignP(delta));
    end
end

fprintf(fid, '\n## Paired proposed-mode tests versus literature adaptations\n\n');
fprintf(fid, '| Proposed mode | Adaptation | Metric | Mean paired delta | Improved trials | Exact two-sided p |\n');
fprintf(fid, '|:--------------|:-----------|:-------|------------------:|----------------:|------------------:|\n');
for proposedIdx = [4, 5]
    for adaptationIdx = [2, 3]
        for metricIdx = 1:numel(metricNames)
            delta = metricValues{metricIdx}(:, proposedIdx) - ...
                metricValues{metricIdx}(:, adaptationIdx);
            fprintf(fid, '| %s | %s | %s | %.3f | %d/50 | %.3g |\n', ...
                combined.armNames{proposedIdx}, combined.armNames{adaptationIdx}, ...
                metricNames{metricIdx}, mean(delta, 'omitnan'), ...
                sum(delta < 0), exactTwoSidedSignP(delta));
        end
    end
end
end

function writeNetworkLatexRows(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%% AUTO-GENERATED by buildReviewerBaselineTable.m; do not edit.\n');
fprintf(fid, '%% Seeds 2--51; values are mean \\pm standard deviation.\n');
fprintf(fid, '\\newcommand{\\ReviewerBaselineNetworkRows}{%%\n');
metricMeans = [ ...
    mean(combined.consensusTrials.ospa, 1); ...
    mean(combined.consensusTrials.gospa, 1); ...
    mean(combined.consensusTrials.pos, 1, 'omitnan'); ...
    mean(combined.consensusTrials.card, 1)];
bestMeans = min(metricMeans, [], 2);
for armIdx = 1:numel(combined.armNames)
    ospa = combined.consensusTrials.ospa(:, armIdx);
    gospa = combined.consensusTrials.gospa(:, armIdx);
    pos = combined.consensusTrials.pos(:, armIdx);
    card = combined.consensusTrials.card(:, armIdx);
    values = {ospa, gospa, pos, card};
    cells = cell(1, numel(values));
    for metricIdx = 1:numel(values)
        cells{metricIdx} = formatLatexMetric(values{metricIdx}, ...
            abs(metricMeans(metricIdx, armIdx) - bestMeans(metricIdx)) < 1e-12);
    end
    fprintf(fid, '%s & %s & %s & %s & %s \\\\\n', ...
        latexArmName(combined.armNames{armIdx}), ...
        cells{1}, cells{2}, cells{3}, cells{4});
end
fprintf(fid, '}\n');
end

function writeLocalLatexRows(path, combined)
fid = fopen(path, 'w');
assert(fid >= 0, 'Unable to write %s', path);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%% AUTO-GENERATED by buildReviewerBaselineTable.m; do not edit.\n');
fprintf(fid, '%% Seeds 2--51; values are mean \\pm standard deviation.\n');
fprintf(fid, '\\newcommand{\\ReviewerBaselineLocalRows}{%%\n');
metricMeans = [ ...
    mean(combined.localPerTrial.eOspa, 1); ...
    mean(combined.localPerTrial.rmse, 1, 'omitnan'); ...
    mean(combined.localPerTrial.cardErr, 1)];
bestMeans = min(metricMeans, [], 2);
for armIdx = 1:numel(combined.armNames)
    eOspa = combined.localPerTrial.eOspa(:, armIdx);
    rmse = combined.localPerTrial.rmse(:, armIdx);
    cardErr = combined.localPerTrial.cardErr(:, armIdx);
    values = {eOspa, rmse, cardErr};
    cells = cell(1, numel(values));
    for metricIdx = 1:numel(values)
        cells{metricIdx} = formatLatexMetric(values{metricIdx}, ...
            abs(metricMeans(metricIdx, armIdx) - bestMeans(metricIdx)) < 1e-12);
    end
    fprintf(fid, '%s & %s & %s & %s \\\\\n', ...
        latexArmName(combined.armNames{armIdx}), ...
        cells{1}, cells{2}, cells{3});
end
fprintf(fid, '}\n');
end

function name = latexArmName(fullName)
switch fullName
    case 'Zheng-style subdensity GA-LMB adaptation'
        name = 'Zheng-style adaptation';
    case 'Gao-style local-trust GA-LMB adaptation'
        name = 'Gao-style adaptation';
    otherwise
        name = fullName;
end
end

function text = formatLatexMetric(values, isBest)
values = values(isfinite(values));
metricMean = mean(values);
metricStd = std(values);
if isBest
    text = sprintf('$\\mathbf{%.3f} \\pm %.3f$', metricMean, metricStd);
else
    text = sprintf('$%.3f \\pm %.3f$', metricMean, metricStd);
end
end

function cohort = runtimeCohort(armIdx)
if any(armIdx == [2, 3])
    cohort = 'separate_adaptation_run';
else
    cohort = 'core_run';
end
end

function pValue = exactTwoSidedSignP(delta)
delta = delta(isfinite(delta) & delta ~= 0);
n = numel(delta);
if n == 0
    pValue = 1;
    return;
end
k = min(sum(delta < 0), sum(delta > 0));
tail = 0;
for idx = 0:k
    tail = tail + nchoosek(n, idx) * 0.5^n;
end
pValue = min(1, 2 * tail);
end
