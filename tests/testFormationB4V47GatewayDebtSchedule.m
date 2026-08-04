function testFormationB4V47GatewayDebtSchedule()
% Exact-budget, rolling-service and physical-index equivariance checks.

protocol = getFormationGatewayDebtV47Protocol();
nodeCount = 8;
[context, reference] = buildSyntheticContext(nodeCount, protocol);
selectedHistory = false(nodeCount, nodeCount, 0);
deliveryHistory = false(nodeCount, nodeCount, 0);
times = zeros(1, 0);
records = cell(1, protocol.period);
for currentTime = 1:protocol.period
    context.currentTime = currentTime;
    context.previousAdjacencyHistory = selectedHistory;
    context.previousAdjacencyHistoryTimes = times;
    context.previousDeliveryHistory = deliveryHistory;
    context.previousDeliveryHistoryTimes = times;
    [adjacency, weights, details] = ...
        buildFormationB4V47GatewayDebtSchedule(context, reference);
    assert(nnz(adjacency) == nodeCount + nodeCount / protocol.period);
    assert(details.residualQuota == nodeCount / protocol.period);
    assert(all(details.dominantAdjacency(:) <= adjacency(:)));
    assert(~details.referenceFallbackUsed);
    assert(all(abs(sum(weights, 2) - 1) < 1e-12));
    records{currentTime} = details;
    selectedHistory(:, :, end + 1) = adjacency; %#ok<AGROW>
    delivered = adjacency;
    if currentTime == 1
        firstReceiver = details.selectedCrossReceivers(1);
        delivered(firstReceiver, :) = false;
    end
    deliveryHistory(:, :, end + 1) = delivered; %#ok<AGROW>
    times(end + 1) = currentTime; %#ok<AGROW>
end
assert(isequal(any(selectedHistory, 3), reference.referenceAdjacency));
assert(records{end}.rollingCrossServiceMature);
assert(records{end}.rollingFormationStrong);
assert(sum(cellfun(@(x) x.currentMessageCount, records)) == ...
    5 * nodeCount);
assert(abs((8 * nodeCount - 5 * nodeCount) / ...
    (8 * nodeCount) - protocol. ...
        exactNoFallbackMessageSavingFraction) < 1e-12);

context.currentTime = protocol.period + 1;
context.previousAdjacencyHistory = selectedHistory;
context.previousAdjacencyHistoryTimes = times;
context.previousDeliveryHistory = deliveryHistory;
context.previousDeliveryHistoryTimes = times;
[adjacency, weights, details] = ...
    buildFormationB4V47GatewayDebtSchedule(context, reference);
permutation = [5, 2, 8, 1, 7, 3, 6, 4];
permutedContext = permuteContext(context, permutation);
permutedReference = permuteReference(reference, permutation);
[permutedAdjacency, permutedWeights, permutedDetails] = ...
    buildFormationB4V47GatewayDebtSchedule( ...
        permutedContext, permutedReference);
assert(isequal(permutedAdjacency, ...
    adjacency(permutation, permutation)));
expectedPermutedWeights = weights(permutation, permutation);
assert(max(abs(permutedWeights(:) - ...
    expectedPermutedWeights(:))) < 1e-12);
assert(isequal(sort(permutedDetails.selectedReceiverPhysicalUids), ...
    sort(details.selectedReceiverPhysicalUids)));

invalid = context;
invalid.observableInputContract.contractVersion = ...
    'topology-policy-observable-input-v3-physical-uid';
assertErrorId(@() buildFormationB4V47GatewayDebtSchedule( ...
    invalid, reference), 'FormationGatewayDebtV47:InvalidContext');

fprintf('PASS: FormationB4V47 gateway-debt schedule tests\n');
end

function [context, reference] = buildSyntheticContext(nodeCount, protocol)
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
referenceWeights(dominant) = protocol.dominantWeight;
referenceWeights(residual) = protocol.referenceResidualWeight;
referenceWeights(1:nodeCount+1:end) = ...
    1 - sum(referenceWeights, 2)';
reference = struct( ...
    'dominantAdjacency', dominant, ...
    'residualAdjacency', residual, ...
    'referenceAdjacency', referenceAdjacency, ...
    'referenceFusionWeights', referenceWeights);

context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = struct('xDimension', 4);
context.commConfig = struct('pDropByEdge', ...
    0.05 + 0.20 * reshape(mod(1:nodeCount^2, 7), ...
        nodeCount, nodeCount) / 6);
context.physicalAdjacency = ~eye(nodeCount);
context.sensorPhysicalUids = 100 + (1:nodeCount);
context.formationPhysicalUidsBySensor = formationUids;
context.previousAdjacencyHistory = false(nodeCount, nodeCount, 0);
context.previousAdjacencyHistoryTimes = zeros(1, 0);
context.previousDeliveryHistory = false(nodeCount, nodeCount, 0);
context.previousDeliveryHistoryTimes = zeros(1, 0);
context.observableInputContract = struct( ...
    'contractVersion', protocol.observableContractVersion, ...
    'passed', true, 'deliveryHistoryPresent', true, ...
    'pastDeliveryHistoryOnly', true, ...
    'deliveryHistoryAlignedToTopologyHistory', true, ...
    'deliveryHistoryAttemptSubset', true);
end

function context = permuteContext(context, order)
matrixFields = {'physicalAdjacency'};
for fieldIdx = 1:numel(matrixFields)
    name = matrixFields{fieldIdx};
    context.(name) = context.(name)(order, order);
end
context.commConfig.pDropByEdge = ...
    context.commConfig.pDropByEdge(order, order);
context.localPosteriorBySensor = ...
    context.localPosteriorBySensor(order);
context.sensorPhysicalUids = context.sensorPhysicalUids(order);
context.formationPhysicalUidsBySensor = ...
    context.formationPhysicalUidsBySensor(order);
context.previousAdjacencyHistory = ...
    context.previousAdjacencyHistory(order, order, :);
context.previousDeliveryHistory = ...
    context.previousDeliveryHistory(order, order, :);
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
