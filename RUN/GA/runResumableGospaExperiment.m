function [aggregate, runMetadata] = runResumableGospaExperiment( ...
    outputDir, filePrefix, trialSeeds, runMissingTrials, experimentMode, ...
    armSelection, expectedArmNames)
% RUNRESUMABLEGOSPAEXPERIMENT Execute deterministic trials with atomic saves.
% Each seed is a standalone one-trial run.  A validated seed file is never
% recomputed, so an interrupted long experiment can resume without losing
% completed filtering work.

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
trialDir = fullfile(outputDir, [filePrefix, '_trials']);
if ~exist(trialDir, 'dir')
    mkdir(trialDir);
end

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
cd(projectRoot);
addpath(projectRoot);
addpath(fullfile(projectRoot, 'RUN', 'GA'));
setPath;

protocol = struct( ...
    'version', 1, ...
    'experimentMode', experimentMode, ...
    'armSelection', reshape(armSelection, 1, []), ...
    'expectedArmNames', {expectedArmNames}, ...
    'gospaC', 5, ...
    'gospaP', 2, ...
    'gospaAlpha', 2, ...
    'gospaGroundSpace', 'complete_extracted_kinematic_state_vector');

runMetadata.startedAt = datestr(now, 31);
runMetadata.trialDir = trialDir;
runMetadata.skippedSeeds = [];
runMetadata.executedSeeds = [];

for trialIdx = 1:numel(trialSeeds)
    trialSeed = trialSeeds(trialIdx);
    trialPath = fullfile(trialDir, sprintf('%s_seed_%03d.mat', ...
        filePrefix, trialSeed));
    if exist(trialPath, 'file')
        validateTrialFile(trialPath, trialSeed, protocol, expectedArmNames);
        runMetadata.skippedSeeds(end + 1) = trialSeed; %#ok<AGROW>
        fprintf('Skipping completed seed %d (%d/%d).\n', ...
            trialSeed, trialIdx, numel(trialSeeds));
        continue;
    end
    if ~runMissingTrials
        error('Missing trial file while runMissingTrials=false: %s', trialPath);
    end

    fprintf('\n%s seed %d (%d/%d)\n', filePrefix, ...
        trialSeed, trialIdx, numel(trialSeeds));
    trialStartedAt = datestr(now, 31);
    [~, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        1, trialSeed - 1, true, struct(), false, experimentMode, struct(), armSelection);
    trialCompletedAt = datestr(now, 31);
    validateTrialSummary(summary, trialSeed, expectedArmNames);

    partialPath = fullfile(trialDir, sprintf('%s_seed_%03d.partial.mat', ...
        filePrefix, trialSeed));
    save('-mat7-binary', partialPath, 'summary', 'trialSeed', 'protocol', ...
        'trialStartedAt', 'trialCompletedAt');
    [moved, message] = movefile(partialPath, trialPath, 'f');
    if ~moved
        error('Unable to finalize %s: %s', trialPath, message);
    end
    runMetadata.executedSeeds(end + 1) = trialSeed; %#ok<AGROW>
end

aggregate = collectTrials(trialDir, filePrefix, trialSeeds, protocol, expectedArmNames);
runMetadata.completedAt = datestr(now, 31);
aggregate.metricProtocol = protocol;
aggregate.resumableTrialDirectory = trialDir;
end

function validateTrialFile(path, trialSeed, protocol, expectedArmNames)
saved = load(path, 'trialSeed', 'protocol', 'summary');
assert(isfield(saved, 'trialSeed') && saved.trialSeed == trialSeed, ...
    'Seed mismatch in %s', path);
assert(isfield(saved, 'protocol') && isequal(saved.protocol, protocol), ...
    'Metric protocol mismatch in %s', path);
assert(isfield(saved, 'summary'), 'Missing summary in %s', path);
validateTrialSummary(saved.summary, trialSeed, expectedArmNames);
end

function validateTrialSummary(summary, trialSeed, expectedArmNames)
assert(isequal(summary.trialSeeds, trialSeed), 'Trial seed mismatch.');
assert(isequal(summary.armNames, expectedArmNames), 'Unexpected arm order.');
requiredConsensus = {'ospa', 'gospa', 'pos', 'card'};
for fieldIdx = 1:numel(requiredConsensus)
    fieldName = requiredConsensus{fieldIdx};
    assert(isfield(summary.consensusTrials, fieldName), ...
        'Missing consensus metric: %s', fieldName);
    values = summary.consensusTrials.(fieldName);
    assert(size(values, 1) == 1 && size(values, 2) == numel(expectedArmNames), ...
        'Unexpected %s shape.', fieldName);
    assert(all(isfinite(values(:))), 'Non-finite %s value.', fieldName);
end
assert(isfield(summary, 'pDropBySensorTrials') && ...
    size(summary.pDropBySensorTrials, 1) == 1, ...
    'Missing one-trial communication realization.');
assert(isfield(summary, 'commConfig') && isstruct(summary.commConfig), ...
    'Missing communication configuration.');
validateMainScenarioCommunication(summary.commConfig);
assert(isfield(summary, 'arms') && numel(summary.arms) == numel(expectedArmNames), ...
    'Missing or malformed arm configuration.');
assert(isequal({summary.arms.name}, expectedArmNames), ...
    'Arm configuration names do not match the selected arms.');
numSensors = size(summary.pDropBySensorTrials, 2);
requiredLocal = {'eOspa', 'rmse', 'cardErr'};
for fieldIdx = 1:numel(requiredLocal)
    fieldName = requiredLocal{fieldIdx};
    assert(isfield(summary.localTrials, fieldName), ...
        'Missing local metric: %s', fieldName);
    values = summary.localTrials.(fieldName);
    assert(size(values, 1) == 1 && size(values, 2) == numSensors && ...
        size(values, 3) == numel(expectedArmNames), ...
        'Unexpected local %s shape.', fieldName);
    if strcmp(fieldName, 'rmse')
        assert(~any(isinf(values(:))), 'Infinite local RMSE value.');
        for armIdx = 1:numel(expectedArmNames)
            armValues = values(:, :, armIdx);
            assert(any(isfinite(armValues(:))), ...
                'Local RMSE is entirely missing for arm %d.', armIdx);
        end
    else
        assert(all(isfinite(values(:))), ...
            'Non-finite local %s value.', fieldName);
    end
end
assert(isfield(summary, 'runtime') && ...
    isfield(summary.runtime, 'filterSeconds'), ...
    'Missing filter runtime.');
runtime = summary.runtime.filterSeconds;
assert(size(runtime, 1) == 1 && size(runtime, 2) == numel(expectedArmNames), ...
    'Unexpected filter runtime shape.');
assert(all(isfinite(runtime(:))) && all(runtime(:) >= 0), ...
    'Invalid filter runtime value.');
end

function aggregate = collectTrials(trialDir, filePrefix, trialSeeds, protocol, expectedArmNames)
aggregate = struct();
for trialIdx = 1:numel(trialSeeds)
    trialSeed = trialSeeds(trialIdx);
    trialPath = fullfile(trialDir, sprintf('%s_seed_%03d.mat', ...
        filePrefix, trialSeed));
    validateTrialFile(trialPath, trialSeed, protocol, expectedArmNames);
    saved = load(trialPath, 'summary');
    current = saved.summary;

    if trialIdx == 1
        numTrials = numel(trialSeeds);
        numArms = numel(expectedArmNames);
        numSensors = size(current.localTrials.eOspa, 2);
        aggregate.armNames = current.armNames;
        aggregate.commConfig = current.commConfig;
        aggregate.arms = current.arms;
        aggregate.consensusTrials.ospa = zeros(numTrials, numArms);
        aggregate.consensusTrials.gospa = zeros(numTrials, numArms);
        aggregate.consensusTrials.pos = zeros(numTrials, numArms);
        aggregate.consensusTrials.card = zeros(numTrials, numArms);
        aggregate.localTrials.eOspa = zeros(numTrials, numSensors, numArms);
        aggregate.localTrials.hOspa = zeros(numTrials, numSensors, numArms);
        aggregate.localTrials.rmse = zeros(numTrials, numSensors, numArms);
        aggregate.localTrials.cardErr = zeros(numTrials, numSensors, numArms);
        aggregate.runtime.filterSeconds = zeros(numTrials, numArms);
        aggregate.pDropBySensorTrials = zeros(numTrials, numSensors);
    end

    assert(isequaln(current.commConfig, aggregate.commConfig), ...
        'Communication configuration drift at seed %d.', trialSeed);
    assert(isequaln(current.arms, aggregate.arms), ...
        'Arm configuration drift at seed %d.', trialSeed);

    aggregate.consensusTrials.ospa(trialIdx, :) = current.consensusTrials.ospa;
    aggregate.consensusTrials.gospa(trialIdx, :) = current.consensusTrials.gospa;
    aggregate.consensusTrials.pos(trialIdx, :) = current.consensusTrials.pos;
    aggregate.consensusTrials.card(trialIdx, :) = current.consensusTrials.card;
    aggregate.localTrials.eOspa(trialIdx, :, :) = current.localTrials.eOspa;
    aggregate.localTrials.hOspa(trialIdx, :, :) = current.localTrials.hOspa;
    aggregate.localTrials.rmse(trialIdx, :, :) = current.localTrials.rmse;
    aggregate.localTrials.cardErr(trialIdx, :, :) = current.localTrials.cardErr;
    aggregate.runtime.filterSeconds(trialIdx, :) = current.runtime.filterSeconds;
    aggregate.pDropBySensorTrials(trialIdx, :) = current.pDropBySensorTrials;
end

aggregate.trialSeeds = reshape(trialSeeds, 1, []);
aggregate.consensus.ospa = mean(aggregate.consensusTrials.ospa, 1);
aggregate.consensus.gospa = mean(aggregate.consensusTrials.gospa, 1);
aggregate.consensus.pos = mean(aggregate.consensusTrials.pos, 1, 'omitnan');
aggregate.consensus.card = mean(aggregate.consensusTrials.card, 1);
aggregate.local.meanAcrossSensors.eOspa = perArmMean(aggregate.localTrials.eOspa, false);
aggregate.local.meanAcrossSensors.hOspa = perArmMean(aggregate.localTrials.hOspa, false);
aggregate.local.meanAcrossSensors.rmse = perArmMean(aggregate.localTrials.rmse, true);
aggregate.local.meanAcrossSensors.cardErr = perArmMean(aggregate.localTrials.cardErr, false);
aggregate.runtime.meanFilterSeconds = mean(aggregate.runtime.filterSeconds, 1);
aggregate.runtime.stdFilterSeconds = std(aggregate.runtime.filterSeconds, 0, 1);
end

function validateMainScenarioCommunication(commConfig)
assert(commConfig.level == 2, 'Unexpected communication level.');
assert(commConfig.globalMaxMeasurementsPerStep == 80, ...
    'Unexpected communication bandwidth cap.');
assert(isequal(commConfig.sensorWeights, ones(1, 8) / 8), ...
    'Unexpected communication sensor weights.');
assert(strcmp(commConfig.priorityPolicy, 'weightedPriority'), ...
    'Unexpected communication priority policy.');
assert(strcmp(commConfig.measurementSelectionPolicy, 'random'), ...
    'Unexpected measurement-selection policy.');
assert(strcmp(commConfig.linkModel, 'fixed'), ...
    'Unexpected link model.');
assert(abs(commConfig.pDrop - 0.2) < 1e-12, ...
    'Unexpected nominal drop probability.');
assert(isequal(commConfig.pDropLevels, [0, 0.1, 0.2, 0.5]), ...
    'Unexpected packet-drop levels.');
assert(isequal(commConfig.pDropLevelCounts, [1, 4, 1, 2]), ...
    'Unexpected packet-drop level counts.');
assert(commConfig.maxOutageNodes == 1, ...
    'Unexpected outage-node setting.');
end

function values = perArmMean(metricValues, omitNan)
numArms = size(metricValues, 3);
values = zeros(1, numArms);
for armIdx = 1:numArms
    armValues = metricValues(:, :, armIdx);
    if omitNan
        values(armIdx) = mean(armValues(:), 'omitnan');
    else
        values(armIdx) = mean(armValues(:));
    end
end
end
