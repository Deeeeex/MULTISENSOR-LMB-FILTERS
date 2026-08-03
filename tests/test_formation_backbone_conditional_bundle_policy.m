function test_formation_backbone_conditional_bundle_policy()
% TEST_FORMATIONBACKBONECONDITIONALBUNDLEPOLICY V40 counterfactual.

policy = getFormationBackboneConditionalBundlePolicyConfig();
assert(strcmp(policy.id, ...
    'formation-backbone-conditional-bundle-v40-v1'));
assert(strcmp(policy.suspensionReweightingMode, ...
    'renormalize-remaining-reference-row'));
assert(policy.referenceMissingInputEquivalent);
assert(~policy.windowContractionCertificateClaimed);
assert(~policy.trackingBenefitClaimed);

context = buildContext();
[referenceAdjacency, referenceDetails] = ...
    selectTemporalFormationBackboneConditionalInputBundleSuspensionPolicy( ...
        context, []);
[candidateAdjacency, candidateDetails] = ...
    selectTemporalFormationBackboneConditionalInputBundleSuspensionPolicy( ...
        context, 2);
[legacyAdjacency, legacyDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
assert(isequal(candidateAdjacency, legacyAdjacency));
assert(nnz(referenceAdjacency) - nnz(candidateAdjacency) == ...
    numel(candidateDetails.suspendedReceivers));
assert(candidateDetails.referenceMissingInputEquivalent);
assert(candidateDetails.entryDisagreementDiagnosticOnly);
assert(~candidateDetails.entryDisagreementGateApplied);
assert(strcmp(candidateDetails.actionName, ...
    'conditional-suspend-input-bundle-f2'));

referenceWeights = referenceDetails.fusionWeightMatrix;
candidateWeights = candidateDetails.fusionWeightMatrix;
legacyWeights = legacyDetails.fusionWeightMatrix;
for edgeIdx = 1:numel(candidateDetails.suspendedReceivers)
    receiver = candidateDetails.suspendedReceivers(edgeIdx);
    sender = candidateDetails.suspendedSenders(edgeIdx);
    expected = referenceWeights(receiver, :);
    removedWeight = expected(sender);
    expected(sender) = 0;
    expected = expected / sum(expected);
    assert(isequaln(candidateWeights(receiver, :), expected));
    assert(abs(candidateDetails. ...
        remainingReferenceMassByReceiver(receiver) - ...
        (1 - removedWeight)) < 1e-12);
    assert(abs(candidateDetails. ...
        renormalizationScaleByReceiver(receiver) - ...
        1 / (1 - removedWeight)) < 1e-12);
    assert(legacyWeights(receiver, receiver) > ...
        candidateWeights(receiver, receiver));
    assert(legacyWeights(receiver, sender) == 0);
end
unchanged = setdiff(1:context.nodeCount, ...
    candidateDetails.suspendedReceivers);
assert(isequaln(candidateWeights(unchanged, :), ...
    referenceWeights(unchanged, :)));

basePolicy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
scoreOptions = struct( ...
    'maximumIncomingCount', ...
        basePolicy.maximumIncomingCountForOutcomeEnumeration);
[~, referenceNetworkDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, referenceAdjacency, referenceWeights, scoreOptions);
[~, candidateNetworkDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, candidateWeights, scoreOptions);
assertConditionalOutcomeEquivalence( ...
    referenceNetworkDetails, candidateNetworkDetails, ...
    candidateDetails.suspendedReceivers, ...
    candidateDetails.suspendedSenders);

probe = buildFormationBackboneConditionalBundleProbe(context);
assert(strcmp(probe.contractVersion, ...
    'formation-backbone-conditional-bundle-mechanism-probe-v1'));
assert(probe.comparableSingleCount == 4);
assert(probe.conditionalAvailableCount == 4);
assert(probe.legacyAvailableCount == 4);
assert(probe.conditionalPreservingInvariantPassed);
assert(~probe.runtimeActionSelected);
assert(~probe.entryDisagreementGateApplied);
assert(~probe.trackingOutcomeScored);
assert(~probe.truthUsed && ~probe.futureOutcomeUsed);

assertErrorId(@() ...
    selectTemporalFormationBackboneConditionalInputBundleSuspensionPolicy( ...
        context, 2, struct('unknown', true)), ...
    'FormationConditionalBundle:InvalidOptions');
changedPolicy = policy;
changedPolicy.referenceMissingInputEquivalent = false;
assertErrorId(@() ...
    selectTemporalFormationBackboneConditionalInputBundleSuspensionPolicy( ...
        context, 2, struct('policyConfig', changedPolicy)), ...
    'FormationConditionalBundle:InvalidPolicyConfig');

fprintf('PASS: formation-backbone conditional bundle policy tests\n');
end

function assertConditionalOutcomeEquivalence( ...
        referenceDetails, candidateDetails, receivers, removedSenders)
for edgeIdx = 1:numel(receivers)
    receiver = receivers(edgeIdx);
    removedSender = removedSenders(edgeIdx);
    reference = referenceDetails.receiverDistributions{receiver};
    candidate = candidateDetails.receiverDistributions{receiver};
    removedColumn = find( ...
        reference.senderIndices == removedSender, 1, 'first');
    assert(~isempty(removedColumn));
    assert(~ismember(removedSender, candidate.senderIndices));
    for candidateOutcome = 1:size(candidate.deliveryMask, 1)
        referenceMask = false(1, numel(reference.senderIndices));
        for senderCursor = 1:numel(candidate.senderIndices)
            referenceColumn = find(reference.senderIndices == ...
                candidate.senderIndices(senderCursor), 1, 'first');
            assert(~isempty(referenceColumn));
            referenceMask(referenceColumn) = ...
                candidate.deliveryMask(candidateOutcome, senderCursor);
        end
        referenceMask(removedColumn) = false;
        referenceOutcome = find(all( ...
            reference.deliveryMask == referenceMask, 2), 1, 'first');
        assert(~isempty(referenceOutcome));
        assert(isequaln( ...
            candidate.effectiveWeights{candidateOutcome}, ...
            reference.effectiveWeights{referenceOutcome}));
        assert(isequaln(candidate.summary{candidateOutcome}, ...
            reference.summary{referenceOutcome}));
    end
end
end

function assertErrorId(callback, expectedIdentifier)
failed = false;
try
    callback();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expectedIdentifier);
end
assert(failed);
end

function context = buildContext()
sensorsPerFormation = 3;
formationCount = 4;
nodeCount = sensorsPerFormation * formationCount;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
backboneEdges = [1, 2; 2, 3; 3, 4; 1, 4];
staticAdjacency = buildStatic(groupIds, backboneEdges);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', staticAdjacency);
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    object = model.birthParameters(1);
    object.birthTime = 1;
    object.birthLocation = 1;
    object.r = 0.58 + 0.08 * groupIds(sensorIdx);
    object.numberOfGmComponents = 1;
    object.w = 1;
    meanVector = zeros(model.xDimension, 1);
    meanVector(1) = 5 * groupIds(sensorIdx);
    object.mu = {meanVector};
    object.Sigma = {eye(model.xDimension)};
    posteriors{sensorIdx} = object;
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct( ...
    'forceDelivery', false, ...
    'pDropByEdge', 0.2 * (ones(nodeCount) - eye(nodeCount)), ...
    'outageSchedule', []);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.triggerConfig.missingNeighborWeightMode = 'renormalize';
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
        if pair(1) == pair(2) || ...
                any(all(treeEdges == pair, 2))
            adjacency(left, right) = true;
            adjacency(right, left) = true;
        end
    end
end
end
