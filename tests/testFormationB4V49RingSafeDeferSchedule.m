function testFormationB4V49RingSafeDeferSchedule()
% Cycle partition, budget, materiality fallback, and UID permutation.

[context, reference] = buildSyntheticContext();
[adjacency, weights, details] = ...
    buildFormationB4V49RingSafeDeferSchedule( ...
        context, reference, struct( ...
            'minimumRelativeImprovementVsBestSynchronized', 0));
nodeCount = numel(context.sensorPhysicalUids);
assert(isequal(size(adjacency), [nodeCount, nodeCount, 4]));
assert(isequal(size(weights), [nodeCount, nodeCount, 4]));
assert(details.candidateCount == 1 + 2 * (2^4 - 1));
assert(details.allCandidateBurstSensorStrongConnected);
assert(details.allCandidateBurstFormationStrongConnected);
assert(details.allCandidateSamePosteriorMessageBudget);
assert(details.burstSensorStrongConnected);
assert(details.burstFormationStrongConnected);
assert(details.periodUnionEqualsReference);
assert(details.messagesPerPeriod == 5 * nodeCount);
assert(details.referenceMessagesPerPeriod == 8 * nodeCount);
assert(abs(details.posteriorMessageSavingFraction - 3 / 8) < 1e-12);
assert(details.selectedSquaredFactor <= ...
    details.bestSynchronizedB4SquaredFactor + 1e-12);
assert(details.localResidualsRemainAtBurst);
assert(details.onlyOppositeCycleEdgesDeferred);
assert(details.mandatoryDirectedFormationCycleAtBurst);
assert(details.deferPulsePhaseFrozenToComparator);
assert(details.bestSynchronizedPulsePhase == 1);
assert(~details.candidatePulsePhaseReoptimized);
assert(~details.posteriorUsed && ~details.truthUsed && ...
    ~details.measurementUsed && ~details.futurePageUsed && ...
    ~details.realizedDeliveryUniformsUsed);

order = [5, 2, 8, 1, 7, 3, 6, 4];
permutedContext = permuteContext(context, order);
permutedReference = permuteReference(reference, order);
[permutedAdjacency, permutedWeights, permutedDetails] = ...
    buildFormationB4V49RingSafeDeferSchedule( ...
        permutedContext, permutedReference, struct( ...
            'minimumRelativeImprovementVsBestSynchronized', 0));
assert(isequal(permutedAdjacency, adjacency(order, order, :)));
expectedWeights = weights(order, order, :);
assert(max(abs(permutedWeights(:) - expectedWeights(:))) < 1e-12);
assert(strcmp(permutedDetails. ...
    selectedPhysicalUidOrderCertificateCanonicalSha256, ...
    details.selectedPhysicalUidOrderCertificateCanonicalSha256));
assert(isequal(permutedDetails.selectedDeferredCrossEdgesPhysicalUid, ...
    details.selectedDeferredCrossEdgesPhysicalUid));

[~, ~, gated] = buildFormationB4V49RingSafeDeferSchedule( ...
    context, reference, struct( ...
        'minimumRelativeImprovementVsBestSynchronized', 1));
if gated.unconstrainedSelectedDeferredCrossEdgeCount > 0 && ...
        gated.unconstrainedSelectedSquaredFactor > 0
    assert(gated.materialityGateTriggered);
    assert(gated.selectedDeferredCrossEdgeCount == 0);
    assert(abs(gated.selectedSquaredFactor - ...
        gated.bestSynchronizedB4SquaredFactor) < 1e-12);
end

invalidReference = reference;
crossRows = find(context.formationPhysicalUidsBySensor == 1);
replacementSenders = find( ...
    context.formationPhysicalUidsBySensor == 2);
invalidReference.residualAdjacency(crossRows(1), :) = false;
invalidReference.residualAdjacency( ...
    crossRows(1), replacementSenders(1)) = true;
invalidReference.referenceAdjacency = ...
    invalidReference.dominantAdjacency | ...
        invalidReference.residualAdjacency;
assertErrorId(@() buildFormationB4V49RingSafeDeferSchedule( ...
    context, invalidReference), ...
    'FormationB4V49:NoBidirectionalFormationCycle');

fprintf('PASS: FormationB4V49 ring-safe defer schedule tests\n');
end

function [context, reference] = buildSyntheticContext()
formationUids = repelem(1:4, 2);
nodeCount = numel(formationUids);
dominant = false(nodeCount);
residual = false(nodeCount);
cycleOrder = [1, 2, 3, 4];
for formationIdx = 1:4
    members = find(formationUids == formationIdx);
    dominant(members(1), members(2)) = true;
    dominant(members(2), members(1)) = true;
    previousFormation = cycleOrder( ...
        mod(find(cycleOrder == formationIdx) - 2, 4) + 1);
    nextFormation = cycleOrder( ...
        mod(find(cycleOrder == formationIdx), 4) + 1);
    previousMembers = find(formationUids == previousFormation);
    nextMembers = find(formationUids == nextFormation);
    residual(members(1), previousMembers(1)) = true;
    residual(members(2), nextMembers(2)) = true;
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
    0.03 + 0.30 * reshape(mod(1:nodeCount^2, 11), ...
        nodeCount, nodeCount) / 10);
context.physicalAdjacency = ~eye(nodeCount);
context.sensorPhysicalUids = 100 + (1:nodeCount);
context.formationPhysicalUidsBySensor = formationUids;
end

function context = permuteContext(context, order)
context.commConfig.pDropByEdge = ...
    context.commConfig.pDropByEdge(order, order);
context.physicalAdjacency = context.physicalAdjacency(order, order);
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
