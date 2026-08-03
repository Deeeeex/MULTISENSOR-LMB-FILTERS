function test_formation_backbone_bundle_staggered_recovery_control()
% TEST_FORMATIONBACKBONEBUNDLESTAGGEREDRECOVERYCONTROL v38 contracts.

policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
schedule = buildFormationProtectionCoverageReleaseSchedule( ...
    1:4, [2, 3, 4], [0.01, 0.05, 0.012, 0.049], ...
    [0, 1, 1, 1], policy.minimumSuspensionAgeSteps, ...
    policy.minimumRetainedProtectionCoverageFraction);
assert(strcmp(schedule.contractVersion, ...
    'formation-protection-coverage-release-schedule-v1'));
assert(isequal(schedule.releaseOrder, [3, 4, 2]));
assert(schedule.candidateCount == 1);
assert(isequal(schedule.candidateReleasedFormationIds{1}, 3));
assert(isequal(schedule.candidateRetainedFormationIds{1}, [2, 4]));

context = buildContext();
base = buildFormationBackboneBundleProtectionControl(context);
control = buildFormationBackboneBundleStaggeredRecoveryControl(context);
assert(strcmp(base.contractVersion, ...
    'formation-backbone-bundle-protection-control-v1'));
assert(strcmp(control.contractVersion, ...
    'formation-backbone-bundle-staggered-recovery-control-v1'));
assert(strcmp(control.policyConfigSha256, policy.canonicalSha256));
assert(control.nodeCount == 24 && control.formationCount == 4);
assert(control.referenceFallbackUsed);
assert(isempty(control.selectedFormationIds));
assert(control.selectedScore.safe);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

[adjacency, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy(context);
assert(isequal(adjacency, control.selectedAdjacency));
assert(strcmp(details.contractVersion, ...
    'formation-backbone-bundle-staggered-recovery-runtime-policy-v1'));
assert(isfield(details, 'protectionScoreByFormation'));
assert(isequaln(details.protectionScoreByFormation, ...
    details.retentionDebtFractionByFormation));
assert(details.posteriorUsed && details.currentLinkReliabilityUsed);
assert(~details.truthUsed && ~details.futureOutcomeUsed);
assert(isfinite(details.selectionSeconds) && ...
    details.selectionSeconds >= 0);
assert(isfinite(details.objective) && ...
    isfinite(details.taskRisk) && ...
    isfinite(details.baselineTaskRisk) && ...
    isfinite(details.taskAdvantage));
assert(~details.oneStepJointProjectionUsed);
assert(strcmp(details.validCandidateCountSemantics, ...
    'safe-route-evaluations-not-deduplicated'));

observable = buildObservableTopologyPolicyContext(context);
[observableAdjacency, observableDetails] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy(observable);
assert(isequal(observableAdjacency, adjacency));
assert(isequaln(observableDetails.protectionScoreByFormation, ...
    details.protectionScoreByFormation));
assert(observable.observableInputContract.passed);

assertJointProjectionAndRuntimeMetadata();
assertStaggeredRelease();
assertRejectsRepeatedB3Violation(context);
fprintf('PASS: formation-backbone input-bundle recovery tests\n');
end

function assertJointProjectionAndRuntimeMetadata()
ringEdges = [1, 2; 2, 3; 3, 4; 1, 4];
context = buildContext(4, ringEdges, 4, ...
    [0.011, 0.011, 0.30, 0.30]);
[referenceAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
[jointAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [3, 4]);
historyPage = jointAdjacency;
groupIds = context.model.dynamicTopologyScenario.config.sensorGroupIds;
[receivers, senders] = find(referenceAdjacency);
restoredMask = ...
    (groupIds(receivers) == 3 & groupIds(senders) == 4) | ...
    (groupIds(receivers) == 4 & groupIds(senders) == 3);
for edgeIdx = reshape(find(restoredMask), 1, [])
    historyPage(receivers(edgeIdx), senders(edgeIdx)) = true;
end
context.previousAdjacency = historyPage;
context.previousAdjacencyHistory = repmat( ...
    historyPage, 1, 1, 2);

base = buildFormationBackboneBundleProtectionControl(context);
assert(all(base.formationProtectionScore(3:4) > 0.02));
assert(all(base.cardinalityEvidenceMask(3:4)));
assert(all(base.singleActionSafetyMask(3:4)));
assert(isequal(base.requestedFormationIds, [3, 4]));
assert(isequal(base.selectedFormationIds, 4));
assert(isequal(base.projectionRemovalOrder, 3));
assert(~base.referenceFallbackUsed);
assert(base.evaluatedRouteCount == 7);

[adjacency, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy( ...
        context);
assert(isequal(adjacency, base.selectedAdjacency));
assert(isequal(details.protectionRequestedFormationIds, [3, 4]));
assert(isequal(details.protectionBaseSelectedFormationIds, 4));
assert(isequal(details.protectionSelectedFormationIds, 4));
assert(isequal(details.protectionBaseProjectionRemovalOrder, 3));
assert(isempty(details.protectionReleasedFormationIds));
assert(isequal(details.retentionDebtRequestedFormationIds, [3, 4]));
assert(isequal(details.retentionDebtProjectionRemovalOrder, 3));
assert(isfinite(details.selectionSeconds));
assert(details.oneStepJointProjectionUsed);
end

function assertStaggeredRelease()
pathEdges = [1, 2; 2, 3; 3, 4];
context = buildContext(6, pathEdges, 4, ...
    [0.011, 0.30, 0.011, 0.82]);
[referenceAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
[incumbentAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [2, 4]);
context.previousAdjacency = incumbentAdjacency;
context.previousAdjacencyHistory = cat( ...
    3, referenceAdjacency, incumbentAdjacency);

control = buildFormationBackboneBundleStaggeredRecoveryControl( ...
    context);
assert(isequal(control.incumbentFormationIds, [2, 4]));
assert(control.staggeredReleaseUsed);
assert(isequal(control.releaseSchedule.releaseOrder, [4, 2]));
assert(control.releaseSchedule.candidateCount == 1);
assert(control.candidateSafetyMask(1));
assert(control.candidateEligibilityMask(1));
assert(control.selectedProtectionCoverageFraction >= 0.80);
assert(control.selectedDisagreementImprovementFraction >= 0.0025);
assert(isequal(control.selectedFormationIds, 2));
assert(isequal(control.releasedFormationIds, 4));

[~, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy( ...
        context);
assert(isequal(details.protectionRequestedFormationIds, [2, 4]));
assert(isequal(details.protectionBaseSelectedFormationIds, [2, 4]));
assert(isequal(details.protectionSelectedFormationIds, 2));
assert(isempty(details.protectionBaseProjectionRemovalOrder));
assert(isequal(details.protectionReleasedFormationIds, 4));
assert(details.reserveSchedule.staggeredReleaseUsed);
end

function assertRejectsRepeatedB3Violation(context)
[suspendedAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
context.previousAdjacency = suspendedAdjacency;
context.previousAdjacencyHistory = repmat( ...
    suspendedAdjacency, 1, 1, 2);
failed = false;
try
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBundleSuspension:Infeasible');
end
assert(failed);
end

function context = buildContext( ...
        sensorsPerFormation, backboneEdges, labelCount, ...
        existenceByFormation)
formationCount = 4;
if nargin < 1
    sensorsPerFormation = 6;
end
if nargin < 2
    backboneEdges = [1, 2; 2, 3; 3, 4];
end
if nargin < 3
    labelCount = 1;
end
if nargin < 4
    existenceByFormation = 0.9 * ones(1, formationCount);
end
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
staticAdjacency = buildStatic(groupIds, backboneEdges);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', staticAdjacency);
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    objects = repmat(model.birthParameters(1), 1, labelCount);
    for objectIdx = 1:labelCount
        objects(objectIdx).birthTime = 1;
        objects(objectIdx).birthLocation = objectIdx;
        objects(objectIdx).r = ...
            existenceByFormation(groupIds(sensorIdx));
        objects(objectIdx).numberOfGmComponents = 1;
        objects(objectIdx).w = 1;
        objects(objectIdx).mu = {zeros(model.xDimension, 1)};
        objects(objectIdx).Sigma = {eye(model.xDimension)};
    end
    posteriors{sensorIdx} = objects;
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(nodeCount), ...
    'outageSchedule', []);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.currentTime = 10;
context.physicalAdjacency = logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = staticAdjacency;
context.edgeBudget = nodeCount;
context.directedMessageBudget = 2 * nodeCount;
context.positions = zeros(2, nodeCount);
[referenceAdjacency, ~] = ...
    selectFormationBackboneResidualTourPolicy(context);
context.previousAdjacency = referenceAdjacency;
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
context.previousAdjacencyHistoryCount = 2;
context.previousAdjacencyHistoryTimes = [8, 9];
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
context.previousAdjacencyHistorySource = 'synthetic-test-history';
context.edgeScores = [];
context.localInnovationHistory = [];
context.localAssociationConfidenceHistory = [];
context.localNisNormHistory = [];
context.localNisDeviationHistory = [];
context.localUpdateHistoryTimes = zeros(1, 0);
context.nodeCount = nodeCount;
end

function adjacency = buildStatic(groupIds, treeEdges)
nodeCount = numel(groupIds);
adjacency = false(nodeCount);
for left = 1:nodeCount-1
    for right = left+1:nodeCount
        pair = sort([groupIds(left), groupIds(right)]);
        same = pair(1) == pair(2);
        linked = any(all(treeEdges == pair, 2));
        if same || linked
            adjacency(left, right) = true;
            adjacency(right, left) = true;
        end
    end
end
end
