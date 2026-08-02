function test_formation_staggered_recovery_control()
% TEST_FORMATIONSTAGGEREDRECOVERYCONTROL Structural v35 contracts.

protocol = getFormationStaggeredRecoveryProbeProtocol();
debt = [0.010305317, 0.052297669, 0.012370445, 0.049228190];
schedule = buildFormationDebtCoverageReleaseSchedule( ...
    1:4, [2, 3, 4], debt, [0, 1, 1, 1], ...
    protocol.minimumSuspensionAgeSteps, ...
    protocol.minimumRetainedDebtCoverageFraction);
assert(strcmp(schedule.contractVersion, ...
    'formation-debt-coverage-release-schedule-v1'));
assert(isequal(schedule.matureFormationIds, [2, 3, 4]));
assert(isequal(schedule.releaseOrder, [3, 4, 2]));
assert(schedule.candidateCount == 1);
assert(isequal(schedule.candidateReleasedFormationIds{1}, 3));
assert(isequal(schedule.candidateRetainedFormationIds{1}, [2, 4]));
assert(abs(schedule.candidateRetainedDebtCoverage(1) - ...
    0.89138853069654445) < 1e-7);
assert(~schedule.truthUsed && ~schedule.futureOutcomeUsed);

noMature = buildFormationDebtCoverageReleaseSchedule( ...
    1:4, [2, 3, 4], debt, zeros(1, 4), 1, 0.8);
assert(noMature.candidateCount == 0);
assert(isempty(noMature.releaseOrder));

forcedReference = buildFormationDebtCoverageReleaseSchedule( ...
    1:4, [], [0.0075, NaN, NaN, NaN], [0, 2, 2, 2], 1, 0.8);
assert(forcedReference.candidateCount == 0);
assert(isempty(forcedReference.releaseOrder));
assert(forcedReference.totalSelectedPositiveDebt == 0);

context = buildContext(4, 6);
control = buildFormationStaggeredRecoveryControl(context);
assert(strcmp(control.contractVersion, ...
    'formation-staggered-recovery-control-v1'));
assert(control.nodeCount == 24);
assert(control.formationCount == 4);
assert(control.referenceFallbackUsed);
assert(control.baseFallbackUsed);
assert(~control.staggeredReleaseUsed);
assert(isempty(control.incumbentFormationIds));
assert(isempty(control.selectedFormationIds));
assert(control.releaseSchedule.candidateCount == 0);
assert(control.selectedScore.safe);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(control.selectionUsesSelectedTopologyHistory);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

initial = buildTemporalCrossEdgeSuspensionActionBank( ...
    context, struct('scorePosteriorSafety', false));
context.currentTime = 2;
[adjacency, details] = ...
    selectFormationStaggeredRecoveryRuntimePolicy( ...
        context, 1, initial, [1, 1, 1]);
assert(isequal(adjacency, control.referenceAdjacency));
assert(strcmp(details.contractVersion, ...
    'formation-staggered-recovery-runtime-policy-v1'));
assert(strcmp(details.backboneMode, 'v35-reference-baseline'));
assert(~details.posteriorUsed);
assert(~details.truthUsed && ~details.futureOutcomeUsed);

assert(strcmp(protocol.contractVersion, ...
    'formation-staggered-recovery-probe-protocol-v1'));
assert(protocol.minimumSuspensionAgeSteps == 1);
assert(protocol.minimumRetainedDebtCoverageFraction == 0.8);
assert(protocol.minimumIncumbentDisagreementImprovementFraction == ...
    0.0025);
assert(protocol.maximumControlRouteEvaluations == 25);
assert(isequal(protocol.expectedSelectedFormationIds, [2, 4]));
assert(isequal(protocol.expectedRuntimeSelectedFormationIdsByTime, ...
    {[2, 3, 4], [2, 4], 3}));
assert(isequal(protocol.expectedRuntimeStaggeredReleaseMask, ...
    logical([0, 1, 0])));
assert(abs(protocol.expectedRetainedDebtCoverageFraction - ...
    0.89138853069654445) < 1e-15);
assert(abs(protocol.expectedDisagreementImprovementFraction - ...
    0.0074679654535776407) < 1e-15);
assert(protocol.trackingOutcomeRerunAuthorized);
assert(~protocol.gnnTrainingAuthorized);
assert(~protocol.x36OutcomeOpeningAuthorized);
assert(~protocol.validationClaimAllowed);
assert(nargin('auditFormationStaggeredRecoveryV35Preflight') == 1);
assert(nargin('runFormationStaggeredRecoveryOpenedScreen') == 1);
assert(nargin('replayFormationStaggeredRecoveryV35SourceTrajectory') == 3);

fprintf('PASS: formation staggered-recovery control tests\n');
end

function context = buildContext(formationCount, sensorsPerFormation)
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds));
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    object = model.birthParameters(1);
    object.birthTime = 1;
    object.birthLocation = 1;
    object.r = 0.9;
    object.numberOfGmComponents = 1;
    object.w = 1;
    object.mu = {zeros(model.xDimension, 1)};
    object.Sigma = {eye(model.xDimension)};
    posteriors{sensorIdx} = object;
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(nodeCount), ...
    'outageSchedule', []);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.currentTime = 1;
context.physicalAdjacency = ...
    logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = context.physicalAdjacency;
context.edgeBudget = nodeCount;
context.directedMessageBudget = 2 * nodeCount;
context.positions = zeros(2, nodeCount);
[referenceAdjacency, ~] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
context.previousAdjacency = referenceAdjacency;
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
context.previousAdjacencyHistoryCount = 2;
context.previousAdjacencyHistoryTimes = [-1, 0];
end
