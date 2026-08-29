function summary = summarizeSafeGraphParetoOracleV153RawSummary(raw)
% SUMMARIZESAFEGRAPHPARETOORACLEV153RAWSUMMARY Apply frozen Pareto gate.

protocol = getSafeGraphParetoOracleV153Protocol();
generatorSummary = summarizeSafeGraphCodebookOracleV152RawSummary(raw);
if ~strcmp(generatorSummary.protocolId, ...
        protocol.actionGeneratorProtocolId)
    error('V153 input does not come from the frozen V152 graph generator.');
end

candidateMask = startsWithCompat( ...
    {generatorSummary.codebook.armMode}, ...
    protocol.proposalArmPrefix);
candidateRecords = generatorSummary.codebook(candidateMask);
if numel(candidateRecords) ~= protocol.proposalCount
    error('V153 raw pilot lacks the complete frozen graph codebook.');
end

admissible = ...
    generatorSummary.dynamicMeanGainPercent > 0 & ...
    generatorSummary.dynamicWorstSensorGainPercent >= ...
        -protocol.maximumWorstSensorRegressionPercent - 1e-10 & ...
    generatorSummary.dynamicMinimumFormationGainPercent >= ...
        -protocol.maximumMinimumFormationRegressionPercent - 1e-10 & ...
    generatorSummary.dynamicConsensusGainPercent >= ...
        -protocol.maximumConsensusRegressionPercent - 1e-10 & ...
    generatorSummary.dynamicAttemptedByteSavingPercent >= ...
        -protocol.maximumAttemptedByteIncreasePercent - 1e-10 & ...
    generatorSummary.dynamicStructurePass;

admissibleIndices = find(admissible);
selectedDynamicIdx = NaN;
if isempty(admissibleIndices)
    selectedIsDynamic = false;
    selectedArmMode = generatorSummary.referenceArmMode;
    selectedMeanEospa = generatorSummary.referenceMeanEospa;
    oracleGainPercent = 0;
    worstSensorGainPercent = 0;
    minimumFormationGainPercent = 0;
    consensusGainPercent = 0;
    attemptedByteSavingPercent = 0;
else
    [~, localBest] = min( ...
        [candidateRecords(admissibleIndices).meanEospa]);
    selectedDynamicIdx = admissibleIndices(localBest);
    selected = candidateRecords(selectedDynamicIdx);
    selectedIsDynamic = true;
    selectedArmMode = selected.armMode;
    selectedMeanEospa = selected.meanEospa;
    oracleGainPercent = ...
        generatorSummary.dynamicMeanGainPercent(selectedDynamicIdx);
    worstSensorGainPercent = ...
        generatorSummary.dynamicWorstSensorGainPercent(selectedDynamicIdx);
    minimumFormationGainPercent = ...
        generatorSummary.dynamicMinimumFormationGainPercent( ...
            selectedDynamicIdx);
    consensusGainPercent = ...
        generatorSummary.dynamicConsensusGainPercent(selectedDynamicIdx);
    attemptedByteSavingPercent = ...
        generatorSummary.dynamicAttemptedByteSavingPercent( ...
            selectedDynamicIdx);
end

summary = generatorSummary;
summary.protocolId = protocol.id;
summary.contractVersion = protocol.contractVersion;
summary.generatedAt = datestr(now, 31);
summary.actionGeneratorProtocolId = ...
    protocol.actionGeneratorProtocolId;
summary.actionGeneratorContractVersion = ...
    protocol.actionGeneratorContractVersion;
summary.selectedArmMode = selectedArmMode;
summary.selectedMeanEospa = selectedMeanEospa;
summary.selectedIsDynamic = selectedIsDynamic;
summary.selectedDynamicIndex = selectedDynamicIdx;
summary.oracleGainPercent = oracleGainPercent;
summary.worstSensorGainPercent = worstSensorGainPercent;
summary.minimumFormationGainPercent = minimumFormationGainPercent;
summary.consensusGainPercent = consensusGainPercent;
summary.attemptedByteSavingPercent = attemptedByteSavingPercent;
summary.strictV152DynamicAdmissible = ...
    generatorSummary.dynamicAdmissible;
summary.dynamicAdmissible = admissible;
summary.admissibleDynamicCount = nnz(admissible);
summary.maximumAttemptedByteIncreasePercent = ...
    protocol.maximumAttemptedByteIncreasePercent;
summary.pilotMeanGatePassed = selectedIsDynamic && ...
    oracleGainPercent >= protocol.minimumMeanGainPercent;
summary.claimBoundary = protocol.claimBoundary;
end

function result = startsWithCompat(values, prefix)
result = cellfun(@(value) strncmp(value, prefix, numel(prefix)), values);
end
