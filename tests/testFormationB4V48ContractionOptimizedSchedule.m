function testFormationB4V48ContractionOptimizedSchedule()
% Exact budget, no-worse fallback, control cost, and index equivariance.

nodeCount = 8;
[context, reference] = buildSyntheticContext(nodeCount);
[adjacency, weights, details] = ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        context, reference, struct('maximumCoordinateSweeps', 2));
assert(isequal(size(adjacency), [nodeCount, nodeCount, 4]));
assert(isequal(size(weights), [nodeCount, nodeCount, 4]));
assert(sum(details.messageCountByPhase) == 5 * nodeCount);
assert(details.referenceMessagesPerPeriod == 8 * nodeCount);
assert(abs(details.posteriorMessageSavingFraction - 3 / 8) < 1e-12);
assert(details.periodUnionEqualsReference);
assert(details.periodUnionSensorStrongConnected);
assert(details.periodUnionFormationStrongConnected);
assert(details.synchronizedCandidateIncluded);
assert(details.selectedNoWorseThanBestSynchronizedProxy);
assert(details.selectedSquaredFactor <= ...
    details.bestSynchronizedB4SquaredFactor + 1e-12);
assert(details.minimumRelativeImprovementVsBestSynchronized == 0.01);
assert(details.nonSynchronizedExecutionRequiresMaterialImprovement);
assert(~details.posteriorUsed && ...
    ~details.deliveryAcknowledgmentUsed && ...
    ~details.truthUsed && ~details.measurementUsed && ...
    ~details.futureGeometryUsed && ~details.futureOutcomeUsed && ...
    ~details.realizedDeliveryUniformsUsed);

cost = estimateFormationB4V48ScheduleControlPlaneCost(8, 4, 3);
assert(cost.globalPlanWireBytes == 58);
assert(cost.residualPullWireBytes == 44);
assert(cost.attemptedPlanMessagesPerWindow == 7);
assert(cost.attemptedPlanBytesPerWindow == 406);
assert(cost.attemptedPullMessagesPerWindow == 8);
assert(cost.attemptedPullBytesPerWindow == 352);
assert(cost.attemptedControlMessages == 45);
assert(cost.attemptedControlBytesPerWindow == 758);
assert(cost.attemptedControlBytes == 2274);
assert(~cost.posteriorSummaryRequired && ...
    ~cost.deliveryAcknowledgmentRequired);
assert(cost.globalTwoPhaseCommitRequired && ...
    ~cost.globalTwoPhaseCommitBytesIncluded && ...
    cost.splitPlanExecutionForbidden);
assert(cost.idealizedIncrementalLowerBoundOnly);
assert(~cost.incrementalScheduleCostClaimAllowed);
assert(~cost.sameTotalByteComparisonClaimAllowed);
assert(~cost.endToEndNetworkCostClaimAllowed);
assert(estimateFormationB4V48ScheduleControlPlaneCost( ...
    24, 4, 1).attemptedControlBytes == 2482);
assert(estimateFormationB4V48ScheduleControlPlaneCost( ...
    36, 6, 1).attemptedControlBytes == 3859);

order = [5, 2, 8, 1, 7, 3, 6, 4];
permutedContext = permuteContext(context, order);
permutedReference = permuteReference(reference, order);
[permutedAdjacency, permutedWeights, permutedDetails] = ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        permutedContext, permutedReference, ...
        struct('maximumCoordinateSweeps', 2));
assert(isequal(permutedAdjacency, ...
    adjacency(order, order, :)));
expectedWeights = weights(order, order, :);
assert(max(abs(permutedWeights(:) - expectedWeights(:))) < 1e-12);
assert(abs(permutedDetails.selectedSquaredFactor - ...
    details.selectedSquaredFactor) < 1e-12);
assert(isequal(permutedDetails.sensorPhysicalUids, ...
    details.sensorPhysicalUids));
assert(isequal(permutedDetails.selectedPhaseByPhysicalUid, ...
    details.selectedPhaseByPhysicalUid));
assert(strcmp(permutedDetails. ...
    selectedPhysicalUidOrderCertificateCanonicalSha256, ...
    details.selectedPhysicalUidOrderCertificateCanonicalSha256));

withPosteriorSideChannel = context;
withPosteriorSideChannel.localPosteriorBySensor = ...
    repmat({struct('futureTruth', 1)}, 1, nodeCount);
[sanitized, contract] = ...
    buildFormationB4V48GraphOnlyContext(withPosteriorSideChannel);
assert(contract.passed && ~contract.posteriorPresent);
assert(~isfield(sanitized, 'localPosteriorBySensor'));
[sanitizedAdjacency, sanitizedWeights] = ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        sanitized, reference, ...
        struct('maximumCoordinateSweeps', 2));
assert(isequal(sanitizedAdjacency, adjacency));
assert(isequaln(sanitizedWeights, weights));
assertErrorId(@() ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        withPosteriorSideChannel, reference, ...
        struct('maximumCoordinateSweeps', 2)), ...
    'FormationB4V48:InvalidContext');

[~, ~, forcedFallback] = ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        context, reference, struct( ...
            'maximumCoordinateSweeps', 2, ...
            'minimumRelativeImprovementVsBestSynchronized', 1));
assert(~forcedFallback.unconstrainedNonSynchronizedSchedule || ...
    forcedFallback.materialityGateTriggered || ...
    forcedFallback.unconstrainedSelectedSquaredFactor == 0);
if forcedFallback.materialityGateTriggered
    assert(numel(unique( ...
        forcedFallback.selectedPhaseByPhysicalUid)) == 1);
    assert(abs(forcedFallback.selectedSquaredFactor - ...
        forcedFallback.bestSynchronizedB4SquaredFactor) < 1e-12);
end

futurePages = context;
futurePages.commConfig.pDropByEdge = repmat( ...
    context.commConfig.pDropByEdge, 1, 1, 2);
assertErrorId(@() buildFormationB4V48GraphOnlyContext(futurePages), ...
    'FormationB4V48GraphContext:FuturePageOrInvalidLinkState');

invalid = context;
invalid.commConfig.pDropByEdge(1, 2) = 1.1;
assertErrorId(@() ...
    buildFormationB4V48ContractionOptimizedSchedule( ...
        invalid, reference), 'FormationB4V48:InvalidReference');

fprintf('PASS: FormationB4V48 contraction-optimized schedule tests\n');
end

function [context, reference] = buildSyntheticContext(nodeCount)
formationUids = repelem(1:4, 2);
dominant = false(nodeCount);
residual = false(nodeCount);
for formationIdx = 1:4
    members = find(formationUids == formationIdx);
    dominant(members(1), members(2)) = true;
    dominant(members(2), members(1)) = true;
    senderFormation = mod(formationIdx - 2, 4) + 1;
    senders = find(formationUids == senderFormation);
    residual(members(1), senders(1)) = true;
    residual(members(2), senders(2)) = true;
end
referenceAdjacency = dominant | residual;
referenceWeights = zeros(nodeCount);
referenceWeights(dominant) = 0.70;
referenceWeights(residual) = 0.05;
referenceWeights(1:nodeCount+1:end) = ...
    1 - sum(referenceWeights, 2)';
reference = struct( ...
    'dominantAdjacency', dominant, ...
    'residualAdjacency', residual, ...
    'referenceAdjacency', referenceAdjacency, ...
    'referenceFusionWeights', referenceWeights);

context = struct();
context.currentTime = 1;
context.commConfig = struct('pDropByEdge', ...
    0.05 + 0.20 * reshape(mod(1:nodeCount^2, 7), ...
        nodeCount, nodeCount) / 6);
context.physicalAdjacency = ~eye(nodeCount);
context.sensorPhysicalUids = 100 + (1:nodeCount);
context.formationPhysicalUidsBySensor = formationUids;
end

function context = permuteContext(context, order)
context.commConfig.pDropByEdge = ...
    context.commConfig.pDropByEdge(order, order);
context.physicalAdjacency = ...
    context.physicalAdjacency(order, order);
context.sensorPhysicalUids = context.sensorPhysicalUids(order);
context.formationPhysicalUidsBySensor = ...
    context.formationPhysicalUidsBySensor(order);
end

function reference = permuteReference(reference, order)
fields = {'dominantAdjacency', 'residualAdjacency', ...
    'referenceAdjacency', 'referenceFusionWeights'};
for fieldIdx = 1:numel(fields)
    name = fields{fieldIdx};
    reference.(name) = reference.(name)(order, order);
end
end

function assertErrorId(callback, expectedIdentifier)
actual = '';
try
    callback();
catch errorInfo
    actual = errorInfo.identifier;
end
assert(strcmp(actual, expectedIdentifier), ...
    'Expected %s, received %s.', expectedIdentifier, actual);
end
