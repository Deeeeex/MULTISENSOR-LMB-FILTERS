function [reportPath, result] = ...
    runOutputAlignedLabelEffectiveOracleV150(options)
% RUNOUTPUTALIGNEDLABELEFFECTIVEORACLEV150 H=8 label-action headroom.
%
% Phase 1 replays bounded singleton omissions and scores their final
% recursive tracking outputs.  Phase 2 composes prefixes of the
% outcome-ranked positive singletons and independently replays those
% bundles.  This is a privileged development oracle, not an online policy.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getOutputAlignedLabelEffectiveOracleV150Protocol();
presetName = char(getField(options, ...
    'presetName', protocol.cases(1).presetName));
caseEntry = resolveCase(protocol, presetName);
cacheRoot = char(getField(options, 'cacheRoot', ''));
if isempty(cacheRoot) || exist(cacheRoot, 'dir') ~= 7
    error('OutputAlignedLabelOracleV150:MissingCacheRoot', ...
        ['A cacheRoot containing the registered opened continuation ', ...
         'must be supplied.']);
end
maximumSingletonActions = round(getField(options, ...
    'maximumSingletonActions', 12));
bundleSizes = reshape(getField(options, ...
    'bundleSizes', [2, 4, 8]), 1, []);
if ~isscalar(maximumSingletonActions) || ...
        ~isfinite(maximumSingletonActions) || ...
        maximumSingletonActions < 1 || ...
        any(~isfinite(bundleSizes)) || ...
        any(bundleSizes ~= round(bundleSizes)) || ...
        any(bundleSizes < 2)
    error('OutputAlignedLabelOracleV150:InvalidSearchOptions', ...
        'The bounded singleton or bundle search options are invalid.');
end
maximumSingletonActions = round(maximumSingletonActions);
bundleSizes = unique(round(bundleSizes), 'stable');
allowDirtySource = logical(getField(options, ...
    'allowDirtySource', false));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v150', ...
    'output_aligned_label_effective_oracle', ...
    lower(caseEntry.scaleName))));
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

bankOptions = struct( ...
    'maximumSingletonActions', maximumSingletonActions);
screenOptions = commonScreenOptions( ...
    protocol, caseEntry, cacheRoot, allowDirtySource);
screenOptions.outputDirectory = fullfile(outputRoot, 'singletons');
screenOptions.outputAlignedLabelOracleBankOptions = bankOptions;
[singletonReportPath, singletonScreen] = ...
    runFormationModeOpenedReturnScreen( ...
        caseEntry.presetName, caseEntry.seed, ...
        caseEntry.currentTime, screenOptions);

[rankedCandidateIndices, rankedRecordIndices] = ...
    rankPositiveSingletons(singletonScreen);
bundleSizes = bundleSizes(bundleSizes <= numel(rankedCandidateIndices));
bundleScreen = struct();
bundleReportPath = '';
if ~isempty(bundleSizes)
    selections = cell(1, 1 + numel(bundleSizes));
    names = cell(1, 1 + numel(bundleSizes));
    selections{1} = zeros(1, 0);
    names{1} = 'reference-full-payload';
    for bundleIdx = 1:numel(bundleSizes)
        bundleSize = bundleSizes(bundleIdx);
        selections{1 + bundleIdx} = ...
            rankedCandidateIndices(1:bundleSize);
        names{1 + bundleIdx} = sprintf( ...
            'output-ranked-positive-top-%d', bundleSize);
    end
    bundleBankOptions = bankOptions;
    bundleBankOptions.actionCandidateSelections = selections;
    bundleBankOptions.actionNames = names;
    bundleOptions = screenOptions;
    bundleOptions.outputDirectory = fullfile(outputRoot, 'bundles');
    bundleOptions.outputAlignedLabelOracleBankOptions = ...
        bundleBankOptions;
    bundleOptions.referenceOutcomeScreenPath = ...
        singletonScreen.matPath;
    [bundleReportPath, bundleScreen] = ...
        runFormationModeOpenedReturnScreen( ...
            caseEntry.presetName, caseEntry.seed, ...
            caseEntry.currentTime, bundleOptions);
end

[bestSource, bestRecord, bestOutcome] = ...
    selectBestRealizedAction(singletonScreen, bundleScreen);
gate = evaluateGate(bestRecord, protocol);
result = struct();
result.contractVersion = ...
    'output-aligned-label-effective-oracle-v150-result-v1';
result.generatedAt = datestr(now, 31);
result.protocol = protocol;
result.caseEntry = caseEntry;
result.cacheRoot = cacheRoot;
result.maximumSingletonActions = maximumSingletonActions;
result.bundleSizesEvaluated = bundleSizes;
result.singletonReportPath = singletonReportPath;
result.bundleReportPath = bundleReportPath;
result.rankedPositiveSingletonCandidateIndices = ...
    rankedCandidateIndices;
result.rankedPositiveSingletonRecordIndices = rankedRecordIndices;
result.bestActionSource = bestSource;
result.bestRecord = bestRecord;
result.bestOutcome = bestOutcome;
result.gate = gate;
result.futureOutcomesUsedForBundleComposition = true;
result.developmentEvidenceOnly = true;

matPath = fullfile(outputRoot, ...
    'OUTPUT_ALIGNED_LABEL_EFFECTIVE_ORACLE_V150_RESULT.mat');
reportPath = fullfile(outputRoot, ...
    'OUTPUT_ALIGNED_LABEL_EFFECTIVE_ORACLE_V150_RESULT.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result', ...
    'singletonScreen', 'bundleScreen');
writeResultReport(reportPath, result, singletonScreen, bundleScreen);
fprintf('V150 output-aligned oracle result: %s\n', reportPath);
end

function options = commonScreenOptions( ...
        protocol, caseEntry, cacheRoot, allowDirtySource)
options = struct();
options.horizonSteps = caseEntry.horizonSteps;
options.interventionDurationSteps = 1;
options.filterSeedOffset = protocol.filterSeedOffset;
options.cacheRoot = cacheRoot;
options.interventionBankType = protocol.interventionBankType;
options.trackingAlignedExecutionProfile = protocol.executionProfile;
options.receiverMode = 'support-renormalized';
options.allowDirtySource = allowDirtySource;
options.writeReport = true;
end

function [candidateIndices, recordIndices] = ...
        rankPositiveSingletons(screen)
referenceIdx = screen.referenceSubsetIndex;
indices = setdiff(1:numel(screen.records), referenceIdx, 'stable');
keep = false(size(indices));
for idx = 1:numel(indices)
    record = screen.records(indices(idx));
    keep(idx) = record.meanGainPercent > 0 && ...
        record.attemptedByteSavingPercent >= -1e-12 && ...
        numel(screen.bank.actionCandidateIndices{ ...
            record.actionIndex}) == 1;
end
indices = indices(keep);
if isempty(indices)
    candidateIndices = zeros(1, 0);
    recordIndices = zeros(1, 0);
    return;
end
ranking = [-reshape([screen.records(indices).meanGainPercent], [], 1), ...
    -reshape([screen.records(indices).minimumFormationGainPercent], [], 1), ...
    reshape(indices, [], 1)];
[~, order] = sortrows(ranking, [1, 2, 3]);
recordIndices = reshape(indices(order), 1, []);
candidateIndices = zeros(1, numel(recordIndices));
for idx = 1:numel(recordIndices)
    actionIdx = screen.records(recordIndices(idx)).actionIndex;
    candidateIndices(idx) = ...
        screen.bank.actionCandidateIndices{actionIdx};
end
end

function [source, record, outcome] = ...
        selectBestRealizedAction(singletonScreen, bundleScreen)
source = 'singleton';
[record, outcome] = bestFromScreen(singletonScreen);
if ~isempty(fieldnames(bundleScreen))
    [bundleRecord, bundleOutcome] = bestFromScreen(bundleScreen);
    if bundleRecord.meanGainPercent > record.meanGainPercent
        source = 'bundle';
        record = bundleRecord;
        outcome = bundleOutcome;
    end
end
end

function [record, outcome] = bestFromScreen(screen)
indices = setdiff(1:numel(screen.records), ...
    screen.referenceSubsetIndex, 'stable');
if isempty(indices)
    error('OutputAlignedLabelOracleV150:MissingCandidateOutcome', ...
        'The oracle screen contains no nonreference action.');
end
byteSafe = [screen.records(indices).attemptedByteSavingPercent] >= -1e-12;
if any(byteSafe)
    indices = indices(byteSafe);
end
[~, localIdx] = max([screen.records(indices).meanGainPercent]);
recordIdx = indices(localIdx);
record = screen.records(recordIdx);
outcome = screen.outcomes(recordIdx);
end

function gate = evaluateGate(record, protocol)
gate = struct();
gate.meanGainPassed = record.meanGainPercent >= ...
    protocol.minimumMeanGainPercent - 1e-12;
gate.worstSensorPassed = record.worstGainPercent >= ...
    protocol.minimumWorstSensorGainPercent - 1e-12;
gate.formationPassed = record.minimumFormationGainPercent >= ...
    protocol.minimumFormationGainPercent - 1e-12;
gate.consensusPassed = record.consensusGainPercent >= ...
    protocol.minimumConsensusGainPercent - 1e-12;
gate.terminalConsensusPassed = ...
    record.terminalConsensusGainPercent >= ...
    protocol.minimumTerminalConsensusGainPercent - 1e-12;
gate.bytePassed = record.attemptedByteSavingPercent >= ...
    protocol.minimumAttemptedByteSavingPercent - 1e-12;
gate.allPassed = gate.meanGainPassed && gate.worstSensorPassed && ...
    gate.formationPassed && gate.consensusPassed && ...
    gate.terminalConsensusPassed && gate.bytePassed;
end

function writeResultReport(path, result, singletonScreen, bundleScreen)
fid = fopen(path, 'w');
if fid < 0
    error('OutputAlignedLabelOracleV150:ReportOpenFailed', ...
        'Could not open the V150 report path.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
record = result.bestRecord;
fprintf(fid, '# V150 output-aligned label-effective oracle\n\n');
fprintf(fid, '- Case: `%s / seed %d / t=%d / H=%d`\n', ...
    result.caseEntry.presetName, result.caseEntry.seed, ...
    result.caseEntry.currentTime, result.caseEntry.horizonSteps);
fprintf(fid, '- Candidate bank: `%d bounded singleton omissions`\n', ...
    result.maximumSingletonActions);
fprintf(fid, '- Positive singleton count: `%d`\n', ...
    numel(result.rankedPositiveSingletonCandidateIndices));
fprintf(fid, '- Bundle sizes evaluated: `%s`\n', ...
    mat2str(result.bundleSizesEvaluated));
fprintf(fid, '- Best source / action: `%s / %s`\n', ...
    result.bestActionSource, record.actionName);
fprintf(fid, '- Development-only: `true`\n\n');
fprintf(fid, '## Best realized result\n\n');
fprintf(fid, '| Metric | Gain | Gate |\n');
fprintf(fid, '|:--|--:|:--:|\n');
fprintf(fid, '| Mean E-OSPA | %+.3f%% | %s |\n', ...
    record.meanGainPercent, passText(result.gate.meanGainPassed));
fprintf(fid, '| Worst sensor E-OSPA | %+.3f%% | %s |\n', ...
    record.worstGainPercent, passText(result.gate.worstSensorPassed));
fprintf(fid, '| Minimum formation E-OSPA | %+.3f%% | %s |\n', ...
    record.minimumFormationGainPercent, ...
    passText(result.gate.formationPassed));
fprintf(fid, '| Window consensus | %+.3f%% | %s |\n', ...
    record.consensusGainPercent, passText(result.gate.consensusPassed));
fprintf(fid, '| Terminal consensus | %+.3f%% | %s |\n', ...
    record.terminalConsensusGainPercent, ...
    passText(result.gate.terminalConsensusPassed));
fprintf(fid, '| Attempted bytes saved | %+.3f%% | %s |\n', ...
    record.attemptedByteSavingPercent, passText(result.gate.bytePassed));
fprintf(fid, '\nOverall registered gate: `%s`\n\n', ...
    passText(result.gate.allPassed));
fprintf(fid, '## Singleton outcomes\n\n');
writeScreenTable(fid, singletonScreen);
if ~isempty(fieldnames(bundleScreen))
    fprintf(fid, '\n## Outcome-ranked bundle outcomes\n\n');
    writeScreenTable(fid, bundleScreen);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.protocol.evidenceBoundary);
end

function writeScreenTable(fid, screen)
fprintf(fid, '| Action | Mean gain | Worst gain | Min formation | Bytes saved |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|\n');
for idx = 1:numel(screen.records)
    record = screen.records(idx);
    fprintf(fid, '| `%s` | %+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% |\n', ...
        record.actionName, record.meanGainPercent, ...
        record.worstGainPercent, ...
        record.minimumFormationGainPercent, ...
        record.attemptedByteSavingPercent);
end
end

function value = passText(passed)
if passed
    value = 'PASS';
else
    value = 'FAIL';
end
end

function entry = resolveCase(protocol, presetName)
matches = strcmp({protocol.cases.presetName}, presetName);
if nnz(matches) ~= 1
    error('OutputAlignedLabelOracleV150:UnregisteredCase', ...
        'The requested preset is not registered for V150.');
end
entry = protocol.cases(matches);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
