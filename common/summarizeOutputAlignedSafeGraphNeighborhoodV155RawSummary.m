function summary = ...
    summarizeOutputAlignedSafeGraphNeighborhoodV155RawSummary(raw)
% SUMMARIZEOUTPUTALIGNEDSAFEGRAPHNEIGHBORHOODV155RAWSUMMARY Apply gate.

protocol = getOutputAlignedSafeGraphNeighborhoodV155Protocol();
presetIdx = find(strcmp(protocol.presetNames, raw.presetName));
if numel(presetIdx) ~= 1 || ...
        ~strcmp(raw.presetName, protocol.stageAPresetName) || ...
        numel(raw.seeds) ~= 1 || raw.seeds ~= protocol.stageASeed || ...
        ~isfield(raw, 'records') || ...
        ~isfield(raw, 'scenarioConfigSnapshot')
    error('V155 raw summary is outside the frozen Stage-A case.');
end
records = reshape(raw.records, 1, []);
staticMask = ismember({records.armMode}, protocol.staticArmModes);
candidateMask = startsWithCompat( ...
    {records.armMode}, protocol.candidateArmPrefix);
if nnz(staticMask) ~= 2 || ...
        nnz(candidateMask) ~= protocol.candidateCounts(presetIdx)
    error('V155 raw pilot lacks the complete frozen bank.');
end
staticRecords = records(staticMask);
[~, bestStaticLocalIdx] = min([staticRecords.meanEospa]);
reference = staticRecords(bestStaticLocalIdx);
candidates = records(candidateMask);

candidateCount = numel(candidates);
retained = false(1, candidateCount);
meanGain = nan(1, candidateCount);
worstSensorGain = nan(1, candidateCount);
minimumFormationGain = nan(1, candidateCount);
consensusGain = nan(1, candidateCount);
attemptedByteSaving = nan(1, candidateCount);
structurePass = false(1, candidateCount);
heldAllPages = false(1, candidateCount);
routeChangeCount = nan(1, candidateCount);
for idx = 1:candidateCount
    candidate = candidates(idx);
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
    [heldAllPages(idx), routeChangeCount(idx)] = ...
        verifyHeldCandidate(candidate, reference, protocol);
    structurePass(idx) = heldAllPages(idx) && ...
        candidate.selectedRollingB3SensorStrongFraction == 1 && ...
        candidate.selectedRollingB3FormationStrongFraction == 1 && ...
        abs(candidate.meanDirectedMessageCount - ...
            protocol.exactSelectedMessageCount(presetIdx)) <= 1e-12;
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
selectedCandidateIdx = NaN;
if isempty(retainedIndices)
    selected = reference;
    selectedIsDynamic = false;
else
    [~, localBest] = min([candidates(retainedIndices).meanEospa]);
    selectedCandidateIdx = retainedIndices(localBest);
    selected = candidates(selectedCandidateIdx);
    selectedIsDynamic = true;
end

summary = struct();
summary.protocolId = protocol.id;
summary.contractVersion = protocol.contractVersion;
summary.generatedAt = datestr(now, 31);
summary.presetName = raw.presetName;
summary.seed = raw.seeds;
summary.analysisWindow = reference.analysisWindow;
summary.referenceArmMode = reference.armMode;
summary.referenceMeanEospa = reference.meanEospa;
summary.selectedArmMode = selected.armMode;
summary.selectedMeanEospa = selected.meanEospa;
summary.selectedIsDynamic = selectedIsDynamic;
summary.selectedCandidateOrdinal = selectedCandidateIdx;
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
summary.retainedCandidateCount = nnz(retained);
summary.candidateCount = candidateCount;
summary.stageAMeanGatePassed = selectedIsDynamic && ...
    summary.oracleGainPercent >= protocol.minimumMeanGainPercent;
summary.radiusTwoAuthorized = selectedIsDynamic && ...
    ~summary.stageAMeanGatePassed;
summary.graphOnlyLocalRouteClosed = ~selectedIsDynamic;
summary.candidateMeanGainPercent = meanGain;
summary.candidateWorstSensorGainPercent = worstSensorGain;
summary.candidateMinimumFormationGainPercent = minimumFormationGain;
summary.candidateConsensusGainPercent = consensusGain;
summary.candidateAttemptedByteSavingPercent = attemptedByteSaving;
summary.candidateStructurePass = structurePass;
summary.candidateHeldAllPages = heldAllPages;
summary.candidateRouteChangeCount = routeChangeCount;
summary.candidateRetained = retained;
summary.maximumAttemptedByteIncreasePercent = ...
    protocol.maximumAttemptedByteIncreasePercent;
summary.codebook = records;
summary.claimBoundary = protocol.claimBoundary;
end

function [passed, changeCount] = ...
    verifyHeldCandidate(candidate, reference, protocol)
history = logical(candidate.focusSelectedDirectedEdgeHistory);
referenceHistory = logical(reference.focusSelectedDirectedEdgeHistory);
if size(history, 3) ~= protocol.horizon || ...
        size(referenceHistory, 3) ~= protocol.horizon
    passed = false;
    changeCount = NaN;
    return;
end
changed = false(1, protocol.horizon - 1);
for pageIdx = 2:protocol.horizon
    changed(pageIdx - 1) = any(any( ...
        history(:, :, pageIdx) ~= history(:, :, pageIdx - 1)));
end
changeCount = nnz(changed);
passed = changeCount == 0 && ...
    any(any(history(:, :, 1) ~= referenceHistory(:, :, 1)));
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
