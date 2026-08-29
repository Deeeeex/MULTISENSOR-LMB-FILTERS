function summary = ...
    summarizeSafeGraphCodebookOracleV152RawSummary(raw)
% SUMMARIZESAFEGRAPHCODEBOOKORACLEV152RAWSUMMARY Apply output-aligned gate.

protocol = getSafeGraphCodebookOracleV152Protocol();
presetIdx = find(strcmp(protocol.presetNames, raw.presetName));
if numel(presetIdx) ~= 1 || numel(raw.seeds) ~= 1 || ...
        ~isfield(raw, 'records') || ...
        ~isfield(raw, 'scenarioConfigSnapshot')
    error('V152 raw summary is outside the frozen single-seed pilot.');
end
records = reshape(raw.records, 1, []);
staticMask = ismember({records.armMode}, protocol.staticArmModes);
dynamicMask = startsWithCompat( ...
    {records.armMode}, protocol.proposalArmPrefix);
if nnz(staticMask) ~= 2 || ...
        nnz(dynamicMask) ~= protocol.proposalCount
    error('V152 raw pilot lacks the complete frozen codebook.');
end
staticRecords = records(staticMask);
[~, bestStaticLocalIdx] = min([staticRecords.meanEospa]);
reference = staticRecords(bestStaticLocalIdx);
candidateRecords = records(dynamicMask);
admissible = false(1, numel(candidateRecords));
for idx = 1:numel(candidateRecords)
    candidate = candidateRecords(idx);
    formationGain = minimumFormationGain( ...
        reference, candidate, ...
        raw.scenarioConfigSnapshot.sensorGroupIds);
    admissible(idx) = ...
        candidate.meanEospa < reference.meanEospa && ...
        relativeGain(reference.worstSensorEospa, ...
            candidate.worstSensorEospa) >= ...
                -protocol.maximumWorstSensorRegressionPercent - 1e-10 && ...
        formationGain >= ...
            -protocol.maximumMinimumFormationRegressionPercent - 1e-10 && ...
        relativeGain(reference.consensusOspa, ...
            candidate.consensusOspa) >= ...
                -protocol.maximumConsensusRegressionPercent - 1e-10 && ...
        relativeSaving(reference.attemptedBytes, ...
            candidate.attemptedBytes) >= ...
                -protocol.maximumAttemptedByteIncreasePercent - 1e-10 && ...
        candidate.selectedRollingB3SensorStrongFraction == 1 && ...
        candidate.selectedRollingB3FormationStrongFraction == 1 && ...
        abs(candidate.meanDirectedMessageCount - ...
            protocol.exactSelectedMessageCount(presetIdx)) <= 1e-12;
end
admissibleIndices = find(admissible);
if isempty(admissibleIndices)
    selected = reference;
    selectedIsDynamic = false;
else
    [~, localBest] = min( ...
        [candidateRecords(admissibleIndices).meanEospa]);
    selected = candidateRecords(admissibleIndices(localBest));
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
summary.oracleGainPercent = relativeGain( ...
    reference.meanEospa, selected.meanEospa);
summary.worstSensorGainPercent = relativeGain( ...
    reference.worstSensorEospa, selected.worstSensorEospa);
summary.minimumFormationGainPercent = minimumFormationGain( ...
    reference, selected, raw.scenarioConfigSnapshot.sensorGroupIds);
summary.consensusGainPercent = relativeGain( ...
    reference.consensusOspa, selected.consensusOspa);
summary.attemptedByteSavingPercent = relativeSaving( ...
    reference.attemptedBytes, selected.attemptedBytes);
summary.admissibleDynamicCount = nnz(admissible);
summary.dynamicArmCount = numel(candidateRecords);
summary.pilotMeanGatePassed = ...
    summary.oracleGainPercent >= ...
        protocol.minimumMeanOracleGainPercent;
summary.codebook = records;
summary.dynamicAdmissible = admissible;
summary.claimBoundary = protocol.claimBoundary;
end

function gain = minimumFormationGain(reference, candidate, groupIds)
groups = unique(groupIds, 'stable');
gain = inf;
for groupIdx = 1:numel(groups)
    mask = groupIds == groups(groupIdx);
    referenceMean = mean(reference.sensorMeanEospa(mask));
    candidateMean = mean(candidate.sensorMeanEospa(mask));
    gain = min(gain, relativeGain(referenceMean, candidateMean));
end
end

function result = startsWithCompat(values, prefix)
result = cellfun(@(value) strncmp(value, prefix, numel(prefix)), values);
end

function gain = relativeGain(referenceValue, candidateValue)
gain = 100 * (referenceValue - candidateValue) / ...
    max(abs(referenceValue), eps);
end

function saving = relativeSaving(referenceValue, candidateValue)
saving = 100 * (referenceValue - candidateValue) / ...
    max(abs(referenceValue), eps);
end
