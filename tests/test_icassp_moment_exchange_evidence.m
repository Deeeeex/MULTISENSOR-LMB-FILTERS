function test_icassp_moment_exchange_evidence()
% TEST_ICASSP_MOMENT_EXCHANGE_EVIDENCE Verify publish and corruption gates.

[~, ~, ~, ~, config] = ...
    runFusionSufficientMomentExchangeConfirmatory(false);
config.numberOfTrials = 5;
config.baseSeed = 1000;
config.firstSeed = 1001;
config.lastSeed = 1005;
config.isSmoke = true;
config.artifactStem = ...
    'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N5_SEEDS1001_1005';
outputDirectory = tempname();
[ok, message] = mkdir(outputDirectory);
assert(ok, message);
cleanup = onCleanup(@() removeTestDirectory(outputDirectory)); %#ok<NASGU>
config.outputDirectory = outputDirectory;
config.regenerationCommand = 'synthetic evidence test';
summary = makeSyntheticSummary(config);

[reportPath, csvPath, summaryPath] = ...
    writeFusionSufficientEvidence(summary, config);
validation = validateFusionSufficientEvidence( ...
    summaryPath, csvPath, reportPath);
assert(validation.valid);
assert(validation.numberOfTrials == 5);
reportText = fileread(reportPath);
assert(isempty(strfind(reportText, '/Users/'))); %#ok<STREMP>
assert(~isempty(strfind(reportText, ...
    ['RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_', ...
    'N5_SEEDS1001_1005.mat']))); %#ok<STREMP>
assertThrows(@() writeFusionSufficientEvidence(summary, config));

corruptCsvPath = fullfile(outputDirectory, 'corrupted.csv');
corruptText = fileread(csvPath);
corruptText = strrep(corruptText, sprintf('\n1001,'), ...
    sprintf('\n9999,'));
fid = fopen(corruptCsvPath, 'w');
assert(fid >= 0);
fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', corruptText);
clear fileCleanup;
assertThrows(@() validateFusionSufficientEvidence( ...
    summaryPath, corruptCsvPath, reportPath));

corruptReportPath = fullfile(outputDirectory, 'corrupted.md');
corruptReportText = strrep(reportText, '**60.000000%**', ...
    '**61.000000%**');
assert(~strcmp(corruptReportText, reportText));
writeTestText(corruptReportPath, corruptReportText);
assertThrows(@() validateFusionSufficientEvidence( ...
    summaryPath, csvPath, corruptReportPath));

invalidSummary = summary;
invalidSummary.trials.attemptedPayloadBytes(1, 1) = NaN;
assertWriterRejects(invalidSummary, config, outputDirectory, 'nan-bytes');
invalidSummary = summary;
invalidSummary.trials.deliveredPayloadBytes(1, 2) = -1;
assertWriterRejects(invalidSummary, config, outputDirectory, ...
    'negative-bytes');
invalidConfig = config;
invalidConfig.isSmoke = 1;
assertWriterRejects(summary, invalidConfig, outputDirectory, ...
    'numeric-smoke-flag');
invalidSummary = summary;
invalidSummary.trials.attemptedMask{1, 2} = ...
    double(invalidSummary.trials.attemptedMask{1, 2});
assertWriterRejects(invalidSummary, config, outputDirectory, ...
    'nonlogical-mask');
invalidSummary = summary;
invalidSummary.trials.posteriorSnapshotCount(1, 2) = 799;
invalidSummary.equivalence.snapshotCount(2) = ...
    invalidSummary.equivalence.snapshotCount(2) - 1;
assertWriterRejects(invalidSummary, config, outputDirectory, ...
    'short-snapshots');
invalidSummary = summary;
invalidSummary.arms(2).triggerConfig.hiddenConfound = 1;
assertWriterRejects(invalidSummary, config, outputDirectory, ...
    'trigger-confound');
testAtomicRollback(outputDirectory);
fprintf('test_icassp_moment_exchange_evidence passed\n');
end

function summary = makeSyntheticSummary(config)
n = config.numberOfTrials;
summary = struct();
summary.armNames = config.armSelection;
summary.numberOfTrials = n;
summary.trialSeeds = config.firstSeed:config.lastSeed;
summary.scenarioConfig = struct( ...
    'simulationLength', 100, 'numberOfSensors', 8);
trigger = struct( ...
    'capturePosteriorSnapshots', true, ...
    'linkGateEnabled', false, ...
    'forceInitialHeavy', false, ...
    'forceLabelChangeHeavy', false, ...
    'forceStaleHeavy', false, ...
    'useStaleNeighborCache', false, ...
    'labelHeartbeatEnabled', false, ...
    'mixedPayloadEnabled', false, ...
    'mixedPayloadLightForAllActiveLabels', false, ...
    'dynamicTopologyEnabled', false, ...
    'modeAwareFusionWeights', false, ...
    'lightCovarianceInflationEnabled', false, ...
    'lightFusionWeightFactor', 1, ...
    'heavyFusionWeightFactor', 1);
fullTrigger = trigger;
fullTrigger.eventPolicy = 'alwaysHeavy';
momentTrigger = trigger;
momentTrigger.eventPolicy = 'alwaysLight';
summary.arms = [ ...
    struct('name', config.armSelection{1}, 'purpose', 'synthetic', ...
        'triggerConfig', fullTrigger), ...
    struct('name', config.armSelection{2}, 'purpose', 'synthetic', ...
        'triggerConfig', momentTrigger)];

summary.trials.localEOspa = zeros(n, 2, 2);
summary.trials.consensusOspa = zeros(n, 2);
summary.trials.consensusPosition = zeros(n, 2);
summary.trials.consensusCardinality = zeros(n, 2);
for trialIdx = 1:n
    localValue = 1 + trialIdx / 100;
    summary.trials.localEOspa(trialIdx, :, :) = localValue;
    summary.trials.consensusOspa(trialIdx, :) = 2 + trialIdx / 100;
    summary.trials.consensusPosition(trialIdx, :) = ...
        0.5 + trialIdx / 100;
    summary.trials.consensusCardinality(trialIdx, :) = ...
        0.25 + trialIdx / 100;
end
fullAttempted = (1000:1000:5000)';
momentAttempted = (400:400:2000)';
summary.trials.attemptedPayloadBytes = ...
    [fullAttempted, momentAttempted];
summary.trials.deliveredPayloadBytes = ...
    [0.8 * fullAttempted, 0.8 * momentAttempted];
summary.trials.attemptedMask = cell(n, 2);
summary.trials.deliveredMask = cell(n, 2);
for trialIdx = 1:n
    attempted = logical(mod((1:(8 * 8 * 100)) + trialIdx, 2));
    attempted = reshape(attempted, [8, 8, 100]);
    delivered = attempted;
    delivered(1) = false;
    summary.trials.attemptedMask{trialIdx, 1} = attempted;
    summary.trials.attemptedMask{trialIdx, 2} = attempted;
    summary.trials.deliveredMask{trialIdx, 1} = delivered;
    summary.trials.deliveredMask{trialIdx, 2} = delivered;
end
summary.trials.posteriorMissingSnapshotCount = zeros(n, 2);
summary.trials.posteriorLabelSetMismatchCount = zeros(n, 2);
summary.trials.posteriorMissingLabelCount = zeros(n, 2);
summary.trials.posteriorComparisonCount = 10 * ones(n, 2);
summary.trials.posteriorSnapshotCount = 800 * ones(n, 2);
summary.trials.posteriorMaxAbsR = zeros(n, 2);
summary.trials.posteriorMaxAbsMu = zeros(n, 2);
summary.trials.posteriorMaxAbsSigma = zeros(n, 2);
summary.trials.posteriorExactMatch = true(n, 2);
summary.equivalence = struct( ...
    'captured', true, ...
    'baselineArm', config.armSelection{1}, ...
    'missingSnapshotCount', [0, 0], ...
    'labelSetMismatchCount', [0, 0], ...
    'missingLabelCount', [0, 0], ...
    'comparisonCount', [10 * n, 10 * n], ...
    'snapshotCount', [800 * n, 800 * n], ...
    'maxAbsR', [0, 0], ...
    'maxAbsMu', [0, 0], ...
    'maxAbsSigma', [0, 0], ...
    'exactMatch', [true, true]);
end

function assertWriterRejects(summary, config, parentDirectory, name)
directory = fullfile(parentDirectory, name);
[ok, message] = mkdir(directory);
assert(ok, message);
config.outputDirectory = directory;
assertThrows(@() writeFusionSufficientEvidence(summary, config));
assert(isempty(dir(fullfile(directory, '*.mat'))));
assert(isempty(dir(fullfile(directory, '*.csv'))));
assert(isempty(dir(fullfile(directory, '*.md'))));
end

function writeTestText(path, text)
fid = fopen(path, 'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
count = fwrite(fid, uint8(text), 'uint8');
assert(count == numel(uint8(text)));
end

function testAtomicRollback(parentDirectory)
directory = fullfile(parentDirectory, 'atomic-rollback');
[ok, message] = mkdir(directory);
assert(ok, message);
temporaryPaths = { ...
    fullfile(directory, 'one.tmp'), ...
    fullfile(directory, 'two.tmp'), ...
    fullfile(directory, 'three.tmp')};
for pathIdx = 1:numel(temporaryPaths)
    writeTestText(temporaryPaths{pathIdx}, sprintf('%d', pathIdx));
end
finalPaths = { ...
    fullfile(directory, 'one.final'), ...
    fullfile(directory, 'two.final'), ...
    fullfile(directory, 'three.final')};
writeTestText(finalPaths{2}, 'preexisting');
assertThrows(@() publishFusionSufficientEvidenceBundle( ...
    temporaryPaths, finalPaths, @() true));
assert(exist(finalPaths{1}, 'file') ~= 2);
assert(exist(finalPaths{2}, 'file') == 2);
assert(strcmp(fileread(finalPaths{2}), 'preexisting'));
assert(exist(finalPaths{3}, 'file') ~= 2);

temporaryPaths = { ...
    fullfile(directory, 'four.tmp'), ...
    fullfile(directory, 'five.tmp'), ...
    fullfile(directory, 'six.tmp')};
finalPaths = { ...
    fullfile(directory, 'four.final'), ...
    fullfile(directory, 'five.final'), ...
    fullfile(directory, 'six.final')};
for pathIdx = 1:numel(temporaryPaths)
    writeTestText(temporaryPaths{pathIdx}, sprintf('%d', pathIdx));
end
assertThrows(@() publishFusionSufficientEvidenceBundle( ...
    temporaryPaths, finalPaths, @() failFinalValidation()));
for pathIdx = 1:numel(finalPaths)
    assert(exist(finalPaths{pathIdx}, 'file') ~= 2);
end
end

function failFinalValidation()
error('Synthetic final validation failure.');
end

function assertThrows(functionHandle)
didThrow = false;
try
    functionHandle();
catch
    didThrow = true;
end
assert(didThrow);
end

function removeTestDirectory(path)
if exist(path, 'dir') == 7
    rmdir(path, 's');
end
end
