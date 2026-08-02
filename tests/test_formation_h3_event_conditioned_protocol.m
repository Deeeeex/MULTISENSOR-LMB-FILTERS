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

coordinatedProtocol = ...
    getFormationH3CoordinatedSubsetRepairProbeProtocol();
assert(strcmp(coordinatedProtocol.contractVersion, ...
    ['formation-h3-coordinated-subset-repair-', ...
     'sequence-protocol-v1']));
assert(isequal(coordinatedProtocol.prefixActionIndices, [9, 13]));
assert(isequal(coordinatedProtocol.coordinatedRepairActionIndices, ...
    20:24));
assert(isequal(coordinatedProtocol.subsetOrders, [3, 4]));
assert(coordinatedProtocol.subsetTrustWeight == 0.30);
assert(coordinatedProtocol.expectedExpandedActionCount == 24);
assert(strcmp(coordinatedProtocol.interventionBankType, ...
    'formation-local-pair-coordinated-subset'));
assert(coordinatedProtocol.actionSelectionUsesTruth);
assert(coordinatedProtocol.openedTrainingMechanismProbeOnly);
assert(~coordinatedProtocol.validationClaimAllowed);
assert(nargin('runFormationH3CoordinatedSubsetRepairProbe') == 1);

modeVectorProtocol = ...
    getFormationH3HeterogeneousModeVectorProbeProtocol();
assert(strcmp(modeVectorProtocol.contractVersion, ...
    'formation-h3-heterogeneous-mode-vector-probe-protocol-v1'));
assert(isequal(modeVectorProtocol.prefixModeVectors, [ ...
    1, 1, 3, 1; 1, 1, 1, 4]));
assert(isequal(modeVectorProtocol.prefixActionIndices, [9, 4]));
assert(isequal(modeVectorProtocol.centerModeVector, [1, 2, 2, 2]));
assert(modeVectorProtocol.centerActionIndex == 22);
assert(modeVectorProtocol.maximumHammingDistance == 2);
assert(modeVectorProtocol.expectedCandidateCount == 67);
assert(modeVectorProtocol.expectedActionCount == 256);
assert(strcmp(modeVectorProtocol.interventionBankType, ...
    'formation-exhaustive-mode-vector'));
assert(modeVectorProtocol.actionSelectionUsesTruth);
assert(modeVectorProtocol.openedTrainingMechanismProbeOnly);
assert(~modeVectorProtocol.validationClaimAllowed);
assert(nargin('runFormationH3HeterogeneousModeVectorProbe') == 1);

firstStepProtocol = getFormationH3FirstStepModeVectorProbeProtocol();
assert(strcmp(firstStepProtocol.contractVersion, ...
    'formation-h3-first-step-mode-vector-probe-protocol-v1'));
assert(isequal(firstStepProtocol.centerModeVector, [1, 1, 1, 1]));
assert(firstStepProtocol.maximumHammingDistance == 2);
assert(firstStepProtocol.expectedCandidateCount == 67);
assert(firstStepProtocol.expectedActionCount == 256);
assert(isequal(firstStepProtocol.reproductionModeVector, ...
    [1, 1, 3, 1]));
assert(firstStepProtocol.reproductionActionIndex == 9);
assert(strcmp(firstStepProtocol.interventionBankType, ...
    'formation-exhaustive-mode-vector'));
assert(firstStepProtocol.actionSelectionUsesTruth);
assert(firstStepProtocol.openedTrainingMechanismProbeOnly);
assert(~firstStepProtocol.validationClaimAllowed);
assert(nargin('runFormationH3FirstStepModeVectorProbe') == 1);

debtRepairProtocol = ...
    getFormationH3TwoStepDebtRepairProbeProtocol();
assert(strcmp(debtRepairProtocol.contractVersion, ...
    'formation-h3-two-step-debt-repair-probe-protocol-v1'));
assert(isequal(debtRepairProtocol.prefixModeVectors, [ ...
    1, 1, 1, 1; 1, 4, 1, 1; 1, 1, 2, 2; ...
    1, 1, 2, 4; 1, 1, 3, 4; 1, 4, 3, 1]));
assert(isequal(debtRepairProtocol.repairModeChoices, ...
    {1, [1, 4], [1, 2, 4], [1, 2, 4]}));
assert(debtRepairProtocol.expectedPrefixCount == 6);
assert(debtRepairProtocol.expectedRepairCount == 18);
assert(debtRepairProtocol.expectedSequenceCount == 108);
assert(debtRepairProtocol.expectedActionCount == 256);
assert(strcmp(debtRepairProtocol.interventionBankType, ...
    'formation-exhaustive-mode-vector'));
assert(debtRepairProtocol.actionSelectionUsesTruth);
assert(debtRepairProtocol.openedTrainingMechanismProbeOnly);
assert(~debtRepairProtocol.validationClaimAllowed);
assert(nargin('runFormationH3TwoStepDebtRepairProbe') == 1);

terminalRepairProtocol = ...
    getFormationH3TerminalDebtRepairProbeProtocol();
assert(strcmp(terminalRepairProtocol.contractVersion, ...
    'formation-h3-terminal-debt-repair-probe-protocol-v1'));
assert(isequal(terminalRepairProtocol.prefixFirstModeVectors, [ ...
    1, 1, 2, 2; 1, 4, 3, 1; 1, 4, 3, 1; 1, 4, 3, 1]));
assert(isequal(terminalRepairProtocol.prefixSecondModeVectors, [ ...
    1, 4, 1, 1; 1, 1, 4, 4; 1, 1, 2, 4; 1, 1, 1, 4]));
assert(isequal(terminalRepairProtocol.savedControlModeVectors, [ ...
    1, 1, 3, 1; 1, 1, 1, 4; 1, 4, 4, 2]));
assert(isequal(terminalRepairProtocol.repairModeChoices, ...
    {1, [1, 4], [1, 2, 4], [1, 2, 4]}));
assert(terminalRepairProtocol.expectedPrefixCount == 4);
assert(terminalRepairProtocol.expectedRepairCount == 18);
assert(terminalRepairProtocol.expectedCandidateCount == 72);
assert(terminalRepairProtocol.expectedSequenceCount == 74);
assert(terminalRepairProtocol.expectedActionCount == 256);
assert(strcmp(terminalRepairProtocol.interventionBankType, ...
    'formation-exhaustive-mode-vector'));
assert(terminalRepairProtocol.actionSelectionUsesTruth);
assert(terminalRepairProtocol.openedTrainingMechanismProbeOnly);
assert(~terminalRepairProtocol.validationClaimAllowed);
assert(nargin('runFormationH3TerminalDebtRepairProbe') == 1);

recoveryProtocol = ...
    getFormationReferenceRecoveryHorizonProbeProtocol();
assert(strcmp(recoveryProtocol.contractVersion, ...
    'formation-reference-recovery-horizon-probe-protocol-v1'));
assert(recoveryProtocol.activePrefixSteps == 3);
assert(isequal(recoveryProtocol.horizonSteps, [4, 5]));
assert(recoveryProtocol.expectedCandidateCount == 4);
assert(recoveryProtocol.expectedSequenceCountPerHorizon == 5);
assert(recoveryProtocol.expectedActionCount == 256);
assert(numel(recoveryProtocol.candidateModeSequences) == 4);
assert(isequal(recoveryProtocol.candidateModeSequences{1}, [ ...
    1, 1, 3, 1; 1, 1, 1, 4; 1, 4, 4, 2]));
assert(isequal(recoveryProtocol.candidateModeSequences{2}, [ ...
    1, 4, 3, 1; 1, 1, 1, 4; 1, 1, 4, 2]));
assert(isequal(size(recoveryProtocol.expectedCandidateH3Targets), ...
    [4, 6]));
assert(strcmp(recoveryProtocol.targetNames{4}, ...
    'window-average-consensus-gain'));
assert(strcmp(recoveryProtocol.targetNames{7}, ...
    'final-step-consensus-gain'));
assert(recoveryProtocol.actionSelectionUsesTruth);
assert(recoveryProtocol.openedTrainingMechanismProbeOnly);
assert(~recoveryProtocol.teacherModelTrainingAuthorized);
assert(~recoveryProtocol.validationClaimAllowed);
assert(nargin('runFormationModeOpenedReturnScreen') == 4);
assert(nargin('runFormationReferenceRecoveryHorizonProbe') == 1);

failed = false;
try
    runFormationModeH3OpenedReturnScreen( ...
        'm24-formation-fov', 211, 72, ...
        struct('horizonSteps', 4));
catch
    failed = true;
end
assert(failed);

failed = false;
try
    runFormationModeOpenedReturnScreen( ...
        'm24-formation-fov', 211, 72, ...
        struct('horizonSteps', 2));
catch
    failed = true;
end
assert(failed);

mixingAuditProtocol = ...
    getFormationReferenceRecoveryMixingAuditProtocol();
assert(strcmp(mixingAuditProtocol.contractVersion, ...
    'formation-reference-recovery-mixing-audit-protocol-v1'));
assert(mixingAuditProtocol.expectedHorizonSteps == 5);
assert(mixingAuditProtocol.expectedArmCount == 5);
assert(mixingAuditProtocol.expectedActionCount == 256);
assert(mixingAuditProtocol.expectedNodeCount == 24);
assert(mixingAuditProtocol.expectedFormationCount == 4);
assert(isequal(mixingAuditProtocol.analysisSteps, ...
    [1, 3, 5, 7, 10, 20, 30]));
assert(mixingAuditProtocol.posthocDiagnosticOnly);
assert(mixingAuditProtocol.authorizesActionRedesign);
assert(~mixingAuditProtocol.authorizesGnnTraining);
assert(~mixingAuditProtocol.validationClaimAllowed);
assert(nargin('auditFormationReferenceRecoveryMixing') == 1);

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
