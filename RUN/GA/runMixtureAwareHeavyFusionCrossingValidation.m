function [reportPath, summary] = ...
    runMixtureAwareHeavyFusionCrossingValidation(seedList, writeReport)
% RUNMIXTUREAWAREHEAVYFUSIONCROSSINGVALIDATION Multi-seed crossing stress.
%
% This runner validates whether mixture-aware heavy fusion improves over
% light payloads and legacy heavy payloads in a small high-clutter crossing
% scenario. It intentionally reuses runMixtureAwareHeavyFusionCrossingSmoke
% so single-seed debugging and multi-seed validation stay aligned.

if nargin < 1 || isempty(seedList)
    seedList = 7:16;
end
if nargin < 2 || isempty(writeReport)
    writeReport = true;
end

seedList = reshape(seedList, 1, []);
numberOfTrials = numel(seedList);
mixtureOverrides = struct( ...
    'mixtureAwareMinExistence', 0.90, ...
    'mixtureAwareMinEntropy', 0.20, ...
    'mixtureAwareMinAssociationAmbiguity', 0.00, ...
    'mixtureAwareMinDetectionAssociationMass', 0.00, ...
    'mixtureAwareMaxFusedEntropy', 1.00, ...
    'mixtureAwareMinFusedDominance', 0.55, ...
    'mixtureAwareTopComponents', 2, ...
    'mixtureAwareMaxFusedComponents', 6, ...
    'mixtureAwareMaxComponentTuples', 128, ...
    'mixtureAwareStateExtractionEnabled', true, ...
    'mixtureAwareExtractionMinSeparation', 4, ...
    'mixtureAwareJointExtractionEnabled', true, ...
    'mixtureAwareJointExtractionTopComponents', 3, ...
    'mixtureAwareJointExtractionMinSeparation', 4, ...
    'mixtureAwareJointExtractionPairwiseScale', 1.0, ...
    'mixtureAwarePredictionConsistencyEnabled', true, ...
    'mixtureAwarePredictionConsistencyStrength', 0.2);

trialSummaries = cell(1, numberOfTrials);
for trialIdx = 1:numberOfTrials
    seed = seedList(trialIdx);
    fprintf('\n=== Mixture-aware crossing validation seed %d (%d/%d) ===\n', ...
        seed, trialIdx, numberOfTrials);
    [~, trialSummaries{trialIdx}] = ...
        runMixtureAwareHeavyFusionCrossingSmoke( ...
        seed, false, mixtureOverrides);
end

summary = aggregateTrialSummaries(seedList, trialSummaries);
summary.config.seedList = seedList;
summary.config.mixtureOverrides = mixtureOverrides;

fprintf('\nMixture-aware heavy crossing validation summary\n');
printSummaryLine('Mean E-OSPA', summary.meanEospa);
printSummaryLine('Crossing E-OSPA', summary.crossingEospa);
fprintf('Mixture vs light crossing change: mean %.2f%%, p90 %.2f%%, wins %d/%d\n', ...
    summary.relative.mixtureVsLightCrossingMeanPercent, ...
    summary.relative.mixtureVsLightCrossingP90Percent, ...
    summary.relative.mixtureVsLightCrossingWins, numberOfTrials);
fprintf('Mixture vs legacy-heavy crossing change: mean %.2f%%, p90 %.2f%%, wins %d/%d\n', ...
    summary.relative.mixtureVsLegacyCrossingMeanPercent, ...
    summary.relative.mixtureVsLegacyCrossingP90Percent, ...
    summary.relative.mixtureVsLegacyCrossingWins, numberOfTrials);

reportPath = '';
if writeReport
    reportPath = fullfile('RUN', 'GA', sprintf( ...
        'MIXTURE_AWARE_HEAVY_CROSSING_VALIDATION_N%d_SEEDS%d_%d_%s.md', ...
        numberOfTrials, seedList(1), seedList(end), ...
        datestr(now, 'yyyymmdd_HHMMSS')));
    writeValidationReport(reportPath, summary);
end
end

function summary = aggregateTrialSummaries(seedList, trialSummaries)
numberOfTrials = numel(trialSummaries);
armNames = trialSummaries{1}.armNames;
numberOfArms = numel(armNames);

meanEospa = zeros(numberOfTrials, numberOfArms);
crossingEospa = zeros(numberOfTrials, numberOfArms);
meanCardinality = zeros(numberOfTrials, numberOfArms);
crossingCardinality = zeros(numberOfTrials, numberOfArms);
crossingCardinalityError = zeros(numberOfTrials, numberOfArms);
payloadBytes = zeros(numberOfTrials, numberOfArms);
componentCounts = zeros(numberOfTrials, numberOfArms);
for trialIdx = 1:numberOfTrials
    trialSummary = trialSummaries{trialIdx};
    meanEospa(trialIdx, :) = trialSummary.meanEospa;
    crossingEospa(trialIdx, :) = trialSummary.crossingEospa;
    meanCardinality(trialIdx, :) = trialSummary.meanCardinality;
    crossingCardinality(trialIdx, :) = trialSummary.crossingCardinality;
    crossingCardinalityError(trialIdx, :) = ...
        trialSummary.crossingCardinalityError;
    payloadBytes(trialIdx, :) = trialSummary.payloadBytes;
    componentCounts(trialIdx, :) = trialSummary.componentCounts;
end

lightIdx = 1;
legacyIdx = 2;
mixtureIdx = 3;
mixtureVsLightCrossing = percentChangeVector( ...
    crossingEospa(:, lightIdx), crossingEospa(:, mixtureIdx));
mixtureVsLegacyCrossing = percentChangeVector( ...
    crossingEospa(:, legacyIdx), crossingEospa(:, mixtureIdx));
mixtureVsLightMean = percentChangeVector( ...
    meanEospa(:, lightIdx), meanEospa(:, mixtureIdx));
mixtureVsLegacyMean = percentChangeVector( ...
    meanEospa(:, legacyIdx), meanEospa(:, mixtureIdx));

summary = struct();
summary.armNames = armNames;
summary.trials.seedList = seedList;
summary.trials.meanEospa = meanEospa;
summary.trials.crossingEospa = crossingEospa;
summary.trials.meanCardinality = meanCardinality;
summary.trials.crossingCardinality = crossingCardinality;
summary.trials.crossingCardinalityError = crossingCardinalityError;
summary.trials.payloadBytes = payloadBytes;
summary.trials.componentCounts = componentCounts;
summary.trials.mixtureVsLightCrossingPercent = ...
    mixtureVsLightCrossing;
summary.trials.mixtureVsLegacyCrossingPercent = ...
    mixtureVsLegacyCrossing;
summary.trials.mixtureVsLightMeanPercent = mixtureVsLightMean;
summary.trials.mixtureVsLegacyMeanPercent = mixtureVsLegacyMean;

summary.meanEospa = summarizeMatrix(meanEospa);
summary.crossingEospa = summarizeMatrix(crossingEospa);
summary.meanCardinality = summarizeMatrix(meanCardinality);
summary.crossingCardinality = summarizeMatrix(crossingCardinality);
summary.crossingCardinalityError = summarizeMatrix( ...
    crossingCardinalityError);
summary.payloadBytes = summarizeMatrix(payloadBytes);
summary.componentCounts = summarizeMatrix(componentCounts);

summary.relative.mixtureVsLightCrossingMeanPercent = ...
    mean(mixtureVsLightCrossing);
summary.relative.mixtureVsLightCrossingP90Percent = ...
    percentileValue(mixtureVsLightCrossing, 0.90);
summary.relative.mixtureVsLightCrossingWins = ...
    sum(mixtureVsLightCrossing < 0);
summary.relative.mixtureVsLegacyCrossingMeanPercent = ...
    mean(mixtureVsLegacyCrossing);
summary.relative.mixtureVsLegacyCrossingP90Percent = ...
    percentileValue(mixtureVsLegacyCrossing, 0.90);
summary.relative.mixtureVsLegacyCrossingWins = ...
    sum(mixtureVsLegacyCrossing < 0);
summary.relative.mixtureVsLightMeanPercent = mean(mixtureVsLightMean);
summary.relative.mixtureVsLegacyMeanPercent = mean(mixtureVsLegacyMean);
end

function stats = summarizeMatrix(values)
stats.mean = mean(values, 1);
stats.std = std(values, 0, 1);
stats.p90 = zeros(1, size(values, 2));
stats.worst = max(values, [], 1);
for armIdx = 1:size(values, 2)
    stats.p90(armIdx) = percentileValue(values(:, armIdx), 0.90);
end
end

function changes = percentChangeVector(baseline, candidate)
changes = 100 * (candidate - baseline) ./ max(abs(baseline), eps);
end

function value = percentileValue(values, probability)
values = sort(reshape(values(isfinite(values)), 1, []));
if isempty(values)
    value = NaN;
    return;
end
position = 1 + min(max(probability, 0), 1) * (numel(values) - 1);
lowerIdx = floor(position);
upperIdx = ceil(position);
if lowerIdx == upperIdx
    value = values(lowerIdx);
else
    fraction = position - lowerIdx;
    value = values(lowerIdx) * (1 - fraction) + ...
        values(upperIdx) * fraction;
end
end

function printSummaryLine(name, stats)
fprintf('%s mean: %s\n', name, mat2str(stats.mean, 4));
fprintf('%s p90:  %s\n', name, mat2str(stats.p90, 4));
fprintf('%s worst:%s\n', name, mat2str(stats.worst, 4));
end

function writeValidationReport(reportPath, summary)
fid = fopen(reportPath, 'w');
if fid < 0
    error('Could not open report for writing: %s', reportPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Mixture-aware heavy crossing validation\n\n');
fprintf(fid, '- Seeds: %s\n', mat2str(summary.config.seedList));
fprintf(fid, '- Scenario: two crossing targets, high clutter, periodic delivery.\n');
fprintf(fid, ['- Mixture-aware config: `minExistence=%.2f`, ', ...
    '`minEntropy=%.2f`, `minAssociationAmbiguity=%.2f`, ', ...
    '`minDetectionMass=%.2f`, `maxFusedEntropy=%.2f`, ', ...
    '`minFusedDominance=%.2f`, `topComponents=%d`, ', ...
    '`predictionConsistency=%.2f`.\n\n'], ...
    summary.config.mixtureOverrides.mixtureAwareMinExistence, ...
    summary.config.mixtureOverrides.mixtureAwareMinEntropy, ...
    summary.config.mixtureOverrides.mixtureAwareMinAssociationAmbiguity, ...
    summary.config.mixtureOverrides.mixtureAwareMinDetectionAssociationMass, ...
    summary.config.mixtureOverrides.mixtureAwareMaxFusedEntropy, ...
    summary.config.mixtureOverrides.mixtureAwareMinFusedDominance, ...
    summary.config.mixtureOverrides.mixtureAwareTopComponents, ...
    summary.config.mixtureOverrides.mixtureAwarePredictionConsistencyStrength);

writeMetricTable(fid, 'Mean E-OSPA', summary.armNames, summary.meanEospa);
writeMetricTable(fid, 'Crossing-window E-OSPA', ...
    summary.armNames, summary.crossingEospa);
writeMetricTable(fid, 'Crossing-window cardinality error', ...
    summary.armNames, summary.crossingCardinalityError);
writeMetricTable(fid, 'Payload bytes', summary.armNames, ...
    summary.payloadBytes);

fprintf(fid, '## Relative crossing result\n\n');
fprintf(fid, '| Comparison | Mean change | P90 change | Wins |\n');
fprintf(fid, '|:--|--:|--:|--:|\n');
fprintf(fid, '| Mixture-aware heavy vs light | %.2f%% | %.2f%% | %d/%d |\n', ...
    summary.relative.mixtureVsLightCrossingMeanPercent, ...
    summary.relative.mixtureVsLightCrossingP90Percent, ...
    summary.relative.mixtureVsLightCrossingWins, ...
    numel(summary.config.seedList));
fprintf(fid, '| Mixture-aware heavy vs legacy heavy | %.2f%% | %.2f%% | %d/%d |\n\n', ...
    summary.relative.mixtureVsLegacyCrossingMeanPercent, ...
    summary.relative.mixtureVsLegacyCrossingP90Percent, ...
    summary.relative.mixtureVsLegacyCrossingWins, ...
    numel(summary.config.seedList));

fprintf(fid, 'Negative percentages mean mixture-aware heavy has lower E-OSPA.\n');
end

function writeMetricTable(fid, titleText, armNames, stats)
fprintf(fid, '## %s\n\n', titleText);
fprintf(fid, '| Arm | Mean | Std | P90 | Worst |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|\n');
for armIdx = 1:numel(armNames)
    fprintf(fid, '| %s | %.4f | %.4f | %.4f | %.4f |\n', ...
        armNames{armIdx}, stats.mean(armIdx), stats.std(armIdx), ...
        stats.p90(armIdx), stats.worst(armIdx));
end
fprintf(fid, '\n');
end
