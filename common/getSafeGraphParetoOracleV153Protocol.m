function protocol = getSafeGraphParetoOracleV153Protocol()
% GETSAFEGRAPHPARETOORACLEV153PROTOCOL Frozen two-cost headroom contract.

generator = getSafeGraphCodebookOracleV152Protocol();

protocol = struct();
protocol.id = 'safe-graph-pareto-value-routing-v153';
protocol.contractVersion = 'safe-graph-pareto-oracle-v153-v1';
protocol.evidenceSplit = generator.evidenceSplit;
protocol.actionGeneratorProtocolId = generator.id;
protocol.actionGeneratorContractVersion = generator.contractVersion;
protocol.presetNames = generator.presetNames;
protocol.nodeCounts = generator.nodeCounts;
protocol.formationCounts = generator.formationCounts;
protocol.sensorsPerFormation = generator.sensorsPerFormation;
protocol.openedDevelopmentSeeds = generator.openedDevelopmentSeeds;
protocol.pilotSeeds = generator.pilotSeeds;
protocol.anchorTimes = generator.anchorTimes;
protocol.horizon = generator.horizon;
protocol.staticArmModes = generator.staticArmModes;
protocol.proposalArmPrefix = generator.proposalArmPrefix;
protocol.proposalRanks = generator.proposalRanks;
protocol.proposalCount = generator.proposalCount;
protocol.exactSelectedMessageCount = ...
    generator.exactSelectedMessageCount;

% Transmission opportunities are the hard action budget.  Realized bytes
% are an endogenous second cost because graph choice changes both selected
% senders and later posterior complexity.
protocol.minimumMeanGainPercent = 5;
protocol.maximumWorstSensorRegressionPercent = 0;
protocol.maximumMinimumFormationRegressionPercent = 0;
protocol.maximumConsensusRegressionPercent = 0;
protocol.maximumAttemptedByteIncreasePercent = 5;
protocol.minimumPositiveSeedCount = 4;
protocol.requireSelectedSensorRollingB3 = true;
protocol.requireSelectedFormationRollingB3 = true;
protocol.gnnTrainingAuthorized = false;
protocol.claimBoundary = [ ...
    'Opened-development tracking-byte Pareto headroom only. The V152 ', ...
    'generator supplies complete projector-safe graphs at an exact ', ...
    'transmission-opportunity budget. V153 changes only the prospective ', ...
    'evaluation rule; truth and future outcomes remain offline oracle ', ...
    'signals and are forbidden to a deployed selector.'];

if ~strcmp(protocol.actionGeneratorProtocolId, generator.id) || ...
        protocol.maximumAttemptedByteIncreasePercent ~= 5 || ...
        protocol.minimumMeanGainPercent ~= 5 || ...
        protocol.gnnTrainingAuthorized || ...
        any(protocol.exactSelectedMessageCount ~= ...
            2 * protocol.nodeCounts - 2 * protocol.formationCounts)
    error('V153 safe graph Pareto protocol is inconsistent.');
end
end
