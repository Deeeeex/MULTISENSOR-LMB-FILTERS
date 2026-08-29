function summary = ...
    summarizeSafeGraphOptionDwellV154RawSummary(raw)
% SUMMARIZESAFEGRAPHOPTIONDWELLV154RAWSUMMARY Apply option headroom gate.

protocol = getSafeGraphOptionDwellV154Protocol();
presetIdx = find(strcmp(protocol.presetNames, raw.presetName));
if numel(presetIdx) ~= 1 || numel(raw.seeds) ~= 1 || ...
        ~isfield(raw, 'records') || ...
        ~isfield(raw, 'scenarioConfigSnapshot')
    error('V154 raw summary is outside the frozen single-seed pilot.');
end
records = reshape(raw.records, 1, []);
staticMask = ismember({records.armMode}, protocol.staticArmModes);
optionMask = startsWithCompat( ...
    {records.armMode}, protocol.optionArmPrefix);
if nnz(staticMask) ~= 2 || nnz(optionMask) ~= protocol.optionCount
    error('V154 raw pilot lacks the complete frozen option bank.');
end
staticRecords = records(staticMask);
[~, bestStaticLocalIdx] = min([staticRecords.meanEospa]);
reference = staticRecords(bestStaticLocalIdx);
candidateRecords = records(optionMask);

retained = false(1, numel(candidateRecords));
meanGain = nan(1, numel(candidateRecords));
worstSensorGain = nan(1, numel(candidateRecords));
minimumFormationGain = nan(1, numel(candidateRecords));
consensusGain = nan(1, numel(candidateRecords));
attemptedByteSaving = nan(1, numel(candidateRecords));
structurePass = false(1, numel(candidateRecords));
optionSchedulePass = false(1, numel(candidateRecords));
routeChangeCount = nan(1, numel(candidateRecords));
for idx = 1:numel(candidateRecords)
    candidate = candidateRecords(idx);
    meanGain(idx) = relativeGain( ...
        reference.meanEospa, candidate.meanEospa);
    worstSensorGain(idx) = relativeGain( ...
        reference.worstSensorEospa, candidate.worstSensorEospa);
    minimumFormationGain(idx) = minimumGroupGain( ...
        reference, candidate, ...
        raw.scenarioConfigSnapshot.sensorGroupIds);
    consensusGain(idx) = relativeGain( ...
        reference.consensusOspa, candidate.consensusOspa);
    attemptedByteSaving(idx) = relativeGain( ...
        reference.attemptedBytes, candidate.attemptedBytes);
    [optionSchedulePass(idx), routeChangeCount(idx)] = ...
        verifyOptionSchedule(candidate, protocol, presetIdx);
    structurePass(idx) = ...
        candidate.selectedRollingB3SensorStrongFraction == 1 && ...
        candidate.selectedRollingB3FormationStrongFraction == 1 && ...
        abs(candidate.meanDirectedMessageCount - ...
            protocol.exactSelectedMessageCount(presetIdx)) <= 1e-12 && ...
        optionSchedulePass(idx);
    retained(idx) = ...
        meanGain(idx) > 0 && ...
        worstSensorGain(idx) >= ...
            -protocol.maximumWorstSensorRegressionPercent - 1e-10 && ...
        minimumFormationGain(idx) >= ...
            -protocol.maximumMinimumFormationRegressionPercent - 1e-10 && ...
        consensusGain(idx) >= ...
            -protocol.maximumConsensusRegressionPercent - 1e-10 && ...
        attemptedByteSaving(idx) >= ...
            -protocol.maximumAttemptedByteIncreasePercent - 1e-10 && ...
        structurePass(idx);
end

retainedIndices = find(retained);
selectedDynamicIdx = NaN;
if isempty(retainedIndices)
    selected = reference;
    selectedIsDynamic = false;
else
    [~, localBest] = min( ...
        [candidateRecords(retainedIndices).meanEospa]);
    selectedDynamicIdx = retainedIndices(localBest);
    selected = candidateRecords(selectedDynamicIdx);
    selectedIsDynamic = true;
end

summary = struct();
summary.protocolId = protocol.id;
summary.contractVersion = protocol.contractVersion;
summary.generatedAt = datestr(now, 31);
summary.presetName = raw.presetName;
summary.seed = raw.seeds;
summary.analysisWindow = reference.analysisWindow;
summary.optionDwellPages = protocol.optionDwellPages(presetIdx);
summary.referenceArmMode = reference.armMode;
summary.referenceMeanEospa = reference.meanEospa;
summary.selectedArmMode = selected.armMode;
summary.selectedMeanEospa = selected.meanEospa;
summary.selectedIsDynamic = selectedIsDynamic;
summary.selectedDynamicIndex = selectedDynamicIdx;
summary.oracleGainPercent = relativeGain( ...
    reference.meanEospa, selected.meanEospa);
summary.worstSensorGainPercent = relativeGain( ...
    reference.worstSensorEospa, selected.worstSensorEospa);
summary.minimumFormationGainPercent = minimumGroupGain( ...
    reference, selected, raw.scenarioConfigSnapshot.sensorGroupIds);
summary.consensusGainPercent = relativeGain( ...
    reference.consensusOspa, selected.consensusOspa);
summary.attemptedByteSavingPercent = relativeGain( ...
    reference.attemptedBytes, selected.attemptedBytes);
summary.retainedOptionCount = nnz(retained);
summary.optionCount = numel(candidateRecords);
summary.pilotMeanGatePassed = selectedIsDynamic && ...
    summary.oracleGainPercent >= protocol.minimumMeanGainPercent;
summary.codebook = records;
summary.optionRetained = retained;
summary.optionMeanGainPercent = meanGain;
summary.optionWorstSensorGainPercent = worstSensorGain;
summary.optionMinimumFormationGainPercent = minimumFormationGain;
summary.optionConsensusGainPercent = consensusGain;
summary.optionAttemptedByteSavingPercent = attemptedByteSaving;
summary.optionStructurePass = structurePass;
summary.optionSchedulePass = optionSchedulePass;
summary.optionRouteChangeCount = routeChangeCount;
summary.maximumAttemptedByteIncreasePercent = ...
    protocol.maximumAttemptedByteIncreasePercent;
summary.claimBoundary = protocol.claimBoundary;
end

function [passed, changeCount] = ...
    verifyOptionSchedule(candidate, protocol, presetIdx)
history = logical(candidate.focusSelectedDirectedEdgeHistory);
if size(history, 3) ~= protocol.horizon
    passed = false;
    changeCount = NaN;
    return;
end
changed = false(1, protocol.horizon - 1);
for pageIdx = 2:protocol.horizon
    changed(pageIdx - 1) = any(any( ...
        history(:, :, pageIdx) ~= history(:, :, pageIdx - 1)));
end
pageTimes = protocol.anchorTimes(presetIdx) + ...
    (1:(protocol.horizon - 1));
allowed = mod(pageTimes - protocol.anchorTimes(presetIdx), ...
    protocol.optionDwellPages(presetIdx)) == 0;
passed = ~any(changed & ~allowed);
changeCount = nnz(changed);
end

function gain = minimumGroupGain(reference, candidate, groupIds)
groups = unique(groupIds, 'stable');
gain = inf;
for groupIdx = 1:numel(groups)
    mask = groupIds == groups(groupIdx);
    gain = min(gain, relativeGain( ...
        mean(reference.sensorMeanEospa(mask)), ...
        mean(candidate.sensorMeanEospa(mask))));
end
end

function result = startsWithCompat(values, prefix)
result = cellfun(@(value) strncmp(value, prefix, numel(prefix)), values);
end

function gain = relativeGain(referenceValue, candidateValue)
gain = 100 * (referenceValue - candidateValue) / ...
    max(abs(referenceValue), eps);
end
