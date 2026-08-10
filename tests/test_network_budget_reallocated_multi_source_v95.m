function test_network_budget_reallocated_multi_source_v95()
% TEST_NETWORKBUDGETREALLOCATEDMULTISOURCEV95 Frozen sender-budget invariant.

protocol = getNetworkBudgetReallocatedMultiSourceV95Protocol();
assert(strcmp(protocol.receiverMode, 'fov-aware-censored'));
assert(numel(protocol.cases) == 2);
assert(isequal([protocol.cases.requiredTokenCount], [2, 3]));
assert(isequal([protocol.cases.horizonSteps], [5, 7]));

nodeCount = 6;
groupIds = [1, 1, 2, 2, 3, 3];
referenceAdjacency = false(nodeCount);
referenceWeights = zeros(nodeCount);
dominant = [2, 1, 4, 3, 6, 5];
residual = [3, 4, 5, 6, 1, 2];
for receiver = 1:nodeCount
    referenceAdjacency(receiver, dominant(receiver)) = true;
    referenceAdjacency(receiver, residual(receiver)) = true;
    referenceWeights(receiver, receiver) = 0.25;
    referenceWeights(receiver, dominant(receiver)) = 0.70;
    referenceWeights(receiver, residual(receiver)) = 0.05;
end
physical = referenceAdjacency | referenceAdjacency';
physical(1, 4) = true;
physical(4, 1) = true;
physical(3, 1) = true;
physical(1, 3) = true;
candidateTemplate = struct( ...
    'targetReceiverIdx', NaN, 'donorReceiverIdx', NaN, ...
    'senderIdx', NaN, 'targetFormationId', NaN, ...
    'sourceFormationId', NaN, 'targetNoveltyFraction', NaN, ...
    'donorUniqueFraction', NaN, 'netUtilityFraction', NaN);
candidates = repmat(candidateTemplate, 1, 2);
candidates(1) = struct( ...
    'targetReceiverIdx', 1, 'donorReceiverIdx', 2, ...
    'senderIdx', 4, 'targetFormationId', 1, ...
    'sourceFormationId', 2, 'targetNoveltyFraction', 0.08, ...
    'donorUniqueFraction', 0.01, 'netUtilityFraction', 0.07);
candidates(2) = struct( ...
    'targetReceiverIdx', 3, 'donorReceiverIdx', 5, ...
    'senderIdx', 1, 'targetFormationId', 2, ...
    'sourceFormationId', 1, 'targetNoveltyFraction', 0.06, ...
    'donorUniqueFraction', 0.01, 'netUtilityFraction', 0.05);
projection = projectSenderBudgetReallocationV95( ...
    candidates, referenceAdjacency, referenceWeights, physical, ...
    groupIds, struct('requiredTokenCount', 2));
assert(projection.feasible);
assert(projection.selectedTokenCount == 2);
assert(projection.referenceMessageCount == 12);
assert(projection.donorOnlyMessageCount == 10);
assert(projection.candidateMessageCount == 12);
assert(projection.senderMessageParityPassed);
assert(isequal(sum(projection.candidateAdjacency, 1), ...
    sum(referenceAdjacency, 1)));
assert(all(abs(sum(projection.candidateWeights, 2) - 1) < 1e-12));
assert(all(abs(sum(projection.donorOnlyWeights, 2) - 1) < 1e-12));
assert(projection.candidateWeights(1, 1) == 0.20);
assert(projection.candidateWeights(2, 2) == 0.30);
end
