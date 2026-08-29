function protocol = getSafeGraphCodebookOracleV152Protocol()
% GETSAFEGRAPHCODEBOOKORACLEV152PROTOCOL Frozen V152 headroom contract.

labelProtocol = getLabelSetSimulatorPolicyProtocol();

protocol = struct();
protocol.id = 'safe-graph-codebook-value-routing-v152';
protocol.contractVersion = 'safe-graph-codebook-oracle-v152-v1';
protocol.evidenceSplit = 'development';
protocol.presetNames = { ...
    'm24-formation-fov', ...
    'x36-formation-fov'};
protocol.nodeCounts = [24, 36];
protocol.formationCounts = [4, 6];
protocol.sensorsPerFormation = 6;
protocol.openedDevelopmentSeeds = [83, 89, 97, 101, 103];
protocol.pilotSeeds = 83;
protocol.anchorTimes = [70, 60];
protocol.horizon = 8;
protocol.filterSeedOffset = 100000;
protocol.staticArmModes = { ...
    'backbone-residual-spliced-cycle-cw-a70-e05', ...
    'backbone-residual-spliced-cycle-ccw-a70-e05'};
protocol.proposalCount = 6;
protocol.proposalRanks = 1:protocol.proposalCount;
protocol.proposalArmPrefix = 'v152-safe-graph-rank';
protocol.dominantWeight = labelProtocol.dominantWeight;
protocol.residualWeight = labelProtocol.localResidualWeight;
protocol.labelSetProtocolId = labelProtocol.id;
protocol.modelPath = labelProtocol.messagePassingInitializationPath;
protocol.modelSha256 = labelProtocol.messagePassingInitializationSha256;
protocol.modelContractVersion = ...
    labelProtocol.messagePassingModelContractVersion;
protocol.exactSelectedMessageCount = ...
    2 * protocol.nodeCounts - 2 * protocol.formationCounts;
protocol.minimumMeanOracleGainPercent = 5;
protocol.minimumPositiveSeedCount = 4;
protocol.maximumWorstSensorRegressionPercent = 0;
protocol.maximumMinimumFormationRegressionPercent = 0;
protocol.maximumConsensusRegressionPercent = 0;
protocol.maximumAttemptedByteIncreasePercent = 0;
protocol.requireSelectedSensorRollingB3 = true;
protocol.requireSelectedFormationRollingB3 = true;
protocol.truthUsedForProposal = false;
protocol.futureOutcomeUsedForProposal = false;
protocol.truthUsedForOracleScoring = true;
protocol.gnnTrainingAuthorized = false;
protocol.x36ProposalGenerationAuthorized = true;
protocol.claimBoundary = [ ...
    'Opened-development action-space headroom only. The pinned M24 ', ...
    'score generates scale-shared feasible graph diversity; it is not ', ...
    'an X36 value predictor. Truth and future outcomes select the ', ...
    'offline oracle only.'];

if protocol.proposalCount < 2 || ...
        numel(protocol.proposalRanks) ~= protocol.proposalCount || ...
        numel(protocol.presetNames) ~= numel(protocol.nodeCounts) || ...
        numel(protocol.nodeCounts) ~= numel(protocol.formationCounts) || ...
        numel(protocol.anchorTimes) ~= numel(protocol.nodeCounts) || ...
        protocol.filterSeedOffset ~= 100000 || ...
        any(protocol.exactSelectedMessageCount ~= ...
            2 * protocol.nodeCounts - 2 * protocol.formationCounts) || ...
        protocol.gnnTrainingAuthorized || ...
        protocol.truthUsedForProposal || ...
        protocol.futureOutcomeUsedForProposal
    error('V152 safe graph-codebook protocol is inconsistent.');
end
end
