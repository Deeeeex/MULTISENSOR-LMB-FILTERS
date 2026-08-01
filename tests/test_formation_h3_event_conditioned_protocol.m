function test_formation_h3_event_conditioned_protocol()
% TEST_FORMATION_H3_EVENT_CONDITIONED_PROTOCOL Truth-free timing test.

protocol = getFormationH3EventConditionedProtocol();
assert(strcmp(protocol.contractVersion, ...
    'formation-h3-event-conditioned-protocol-v1'));
assert(isequal(protocol.presets, {'m24-formation-fov'}));
assert(isequal(protocol.trainingSeeds, [211, 223]));
assert(isequal(protocol.developmentSeeds, 227));
assert(isequal(protocol.candidateTimes, 40:4:136));
assert(protocol.selectedStateCount == 2);
assert(protocol.minimumSelectedTimeSeparation == 16);
assert(protocol.expectedAugmentedActionCounts == 19);
assert(~protocol.eventSelectionUsesTruth);
assert(~protocol.eventSelectionUsesFutureMeasurements);
assert(isempty(intersect(protocol.allSeeds, ...
    protocol.finalValidationSeedsReserved)));
assert(nargin('selectFormationH3EventConditionedStates') == 3);
assert(nargin('runFormationH3EventConditionedTeacherShard') == 3);

jointProtocol = getFormationH3ProjectedJointProbeProtocol();
assert(strcmp(jointProtocol.contractVersion, ...
    'formation-h3-projected-joint-probe-protocol-v1'));
assert(jointProtocol.seed == 211);
assert(isequal(jointProtocol.snapshotTimes, [60, 72, 104, 124]));
assert(strcmp(jointProtocol.cacheGenerationGitCommit, ...
    'c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53'));
assert(jointProtocol.openedTrainingMechanismProbeOnly);
assert(~jointProtocol.validationClaimAllowed);
assert(nargin('runFormationProjectedJointH3OpenedReturnScreen') == 4);
assert(nargin('runFormationH3ProjectedJointProbe') == 1);

durationProtocol = getFormationH3DurationProbeProtocol();
assert(strcmp(durationProtocol.contractVersion, ...
    'formation-h3-duration-probe-protocol-v1'));
assert(isequal(durationProtocol.snapshotTimes, [60, 72, 104]));
assert(isequal(durationProtocol.actionIndices, [2, 7, 12]));
assert(durationProtocol.interventionDurationSteps == 3);
assert(durationProtocol.actionSelectionUsesTruth);
assert(durationProtocol.openedTrainingMechanismProbeOnly);
assert(~durationProtocol.validationClaimAllowed);
assert(nargin('runFormationH3DurationProbe') == 1);

sequenceProtocol = ...
    getFormationH3ComplementarySequenceProbeProtocol();
assert(strcmp(sequenceProtocol.contractVersion, ...
    'formation-h3-complementary-sequence-protocol-v1'));
assert(sequenceProtocol.seed == 211);
assert(sequenceProtocol.currentTime == 72);
assert(sequenceProtocol.exploitActionIndex == 9);
assert(isequal(sequenceProtocol.repairActionIndices, ...
    [1, 2, 3, 4, 5, 6, 7, 11, 12, 13]));
assert(sequenceProtocol.beamWidth == 4);
assert(sequenceProtocol.actionSelectionUsesTruth);
assert(sequenceProtocol.openedTrainingMechanismProbeOnly);
assert(~sequenceProtocol.validationClaimAllowed);
assert(nargin('runFormationH3ComplementarySequenceProbe') == 1);

beamTargets = [ ...
    1, 0, 0, 0, 0, 0; ...
    5, 0, 0, -10, 0, 0; ...
    2, 0, 0, 5, 0, 0; ...
    2, 2, -3, 0, 0, 0; ...
    1, 1, 1, 1, 1, 1];
[beamActions, beamDetails] = ...
    selectFormationH3ComplementarySequenceBeams( ...
        [1, 2, 3, 4, 5], beamTargets, 4);
assert(isequal(beamActions, [2, 3, 4, 5]));
assert(isequal(beamDetails.selectionCriteria, ...
    sequenceProtocol.beamCriteria));
assert(beamDetails.targetsUseTruth);
assert(~beamDetails.validationClaimAllowed);

pairRepairProtocol = getFormationH3PairRepairProbeProtocol();
assert(strcmp(pairRepairProtocol.contractVersion, ...
    'formation-h3-pair-repair-sequence-protocol-v1'));
assert(isequal(pairRepairProtocol.prefixActionIndices, [9, 13]));
assert(isequal(pairRepairProtocol.pairRepairActionIndices, 14:19));
assert(strcmp(pairRepairProtocol.interventionBankType, ...
    'formation-single-plus-pair'));
assert(pairRepairProtocol.actionSelectionUsesTruth);
assert(pairRepairProtocol.openedTrainingMechanismProbeOnly);
assert(~pairRepairProtocol.validationClaimAllowed);
assert(nargin('runFormationH3PairRepairSequenceProbe') == 1);

groupIds = [1, 1, 2, 2];
context = syntheticContext(groupIds);
metrics = computeFormationH3ObservableEventScore( ...
    context, groupIds, struct());
assert(strcmp(metrics.contractVersion, ...
    protocol.eventScoreContractVersion));
assert(metrics.score > 0);
assert(metrics.posteriorContrast > 0);
assert(abs(metrics.robustSelectedCrossLinkStress - 0.4) < 1e-12);
assert(~metrics.truthUsed && ~metrics.futureMeasurementsUsed);
assert(~metrics.futureLinkOutcomesUsed);

permutation = [2, 1, 4, 3];
permuted = context;
permuted.localPosteriorBySensor = ...
    context.localPosteriorBySensor(permutation);
permuted.previousAdjacency = ...
    context.previousAdjacency(permutation, permutation);
permuted.commConfig.pDropByEdge = ...
    context.commConfig.pDropByEdge(permutation, permutation);
permutedMetrics = computeFormationH3ObservableEventScore( ...
    permuted, groupIds(permutation), struct());
assert(abs(metrics.score - permutedMetrics.score) < 1e-12);
relabelledMetrics = computeFormationH3ObservableEventScore( ...
    context, [9, 9, 3, 3], struct());
assert(abs(metrics.score - relabelledMetrics.score) < 1e-12);

identical = context;
identical.localPosteriorBySensor = repmat( ...
    context.localPosteriorBySensor(1), 1, 4);
zeroMetrics = computeFormationH3ObservableEventScore( ...
    identical, groupIds, struct());
assert(zeroMetrics.score == 0);

invalid = context;
invalid.commConfig.linkUniforms = zeros(4);
failed = false;
try
    computeFormationH3ObservableEventScore( ...
        invalid, groupIds, struct());
catch
    failed = true;
end
assert(failed);

candidateTimes = 40:4:80;
scores = [0, 1, 5, 0, 4, 0, 0, 0, 3, 0, 0];
[selectedTimes, selectedIndices] = ...
    selectFormationH3EventTimes( ...
        candidateTimes, scores, 2, 16);
assert(isequal(selectedTimes, [48, 72]));
assert(isequal(selectedIndices, [3, 9]));
[tieTimes, ~] = selectFormationH3EventTimes( ...
    [40, 56, 72], [1, 1, 0], 1, 8);
assert(isequal(tieTimes, 40));

audit = computeFormationH3StrictOracleAudit( ...
    syntheticScreen('local'), syntheticScreen('pair'));
assert(audit.actionCount == 4);
assert(audit.feasibleActionCount == 3);
assert(strcmp(audit.oracleActionName, 'pair-safe'));
assert(abs(audit.oracleGainPercent - 4) < 1e-12);
assert(isequal(audit.oracleTargets, [4, 0.1, 0.2, 0.3, 1, 10]));
fprintf('test_formation_h3_event_conditioned_protocol passed\n');
end

function context = syntheticContext(groupIds)
nodeCount = numel(groupIds);
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    if groupIds(sensorIdx) == groupIds(1)
        position = [0; 0];
    else
        position = [4; 0];
    end
    posteriors{sensorIdx} = syntheticObject(position);
end
previous = logical([ ...
    0, 1, 1, 0; ...
    1, 0, 0, 0; ...
    1, 0, 0, 1; ...
    0, 0, 1, 0]);
pDrop = zeros(nodeCount);
pDrop(3, 1) = 0.4;
pDrop(1, 3) = 0.4;
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = struct('xDimension', 4);
context.commConfig = struct('pDropByEdge', pDrop);
context.previousAdjacency = previous;
end

function object = syntheticObject(position)
meanVector = [position; 0; 0];
object = struct( ...
    'birthTime', 1, 'birthLocation', 1, ...
    'r', 0.9, 'numberOfGmComponents', 1, ...
    'w', 1, 'mu', {{meanVector}}, ...
    'Sigma', {{eye(4)}});
end

function screen = syntheticScreen(kind)
reference = record('reference', 0, 0, 0, 0, 0);
if strcmp(kind, 'local')
    records = [ ...
        reference, ...
        record('local-safe', 2, 0, 0, 0, 0.5), ...
        record('local-tail-loss', 3, -1, 0, 0, 1)];
    outcomes = [ ...
        outcome(1000), outcome(950), outcome(940)];
elseif strcmp(kind, 'pair')
    records = [ ...
        reference, ...
        record('pair-safe', 4, 0.1, 0.2, 0.3, 1)];
    outcomes = [outcome(1000), outcome(900)];
else
    error('Unknown synthetic screen kind.');
end
screen = struct( ...
    'records', records, 'outcomes', outcomes, ...
    'referenceSubsetIndex', 1, ...
    'presetName', 'm24-formation-fov', ...
    'seed', 211, 'currentTime', 60, ...
    'generationGitCommit', 'test-commit', ...
    'cacheSha256', 'test-cache');
end

function value = record(name, meanGain, formationGain, ...
        worstGain, consensusGain, attemptedSaving)
value = struct( ...
    'actionName', name, ...
    'meanGainPercent', meanGain, ...
    'minimumFormationGainPercent', formationGain, ...
    'worstGainPercent', worstGain, ...
    'consensusGainPercent', consensusGain, ...
    'attemptedByteSavingPercent', attemptedSaving);
end

function value = outcome(deliveredBytes)
value = struct( ...
    'meanEospa', 10, 'worstSensorEospa', 20, ...
    'consensusOspa', 5, 'attemptedBytes', 1100, ...
    'deliveredBytes', deliveredBytes);
end
