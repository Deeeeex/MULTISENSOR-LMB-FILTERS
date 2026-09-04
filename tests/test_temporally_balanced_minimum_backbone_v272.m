function test_temporally_balanced_minimum_backbone_v272()
% Focused invariant test for the V272 route policy.

protocol = getTemporallyBalancedMinimumBackboneV272Protocol();
config = buildDynamicTopologyScenarioConfig( ...
    'm24-formation-fov-temporal-coupled-formation-braid');
runtimeConfig = ...
    buildTemporallyBalancedMinimumBackboneV272Config(config);
assert(runtimeConfig.topologyPolicyHistoryDepth == protocol.historyDepth);
assert(runtimeConfig.topologyDirectedMessageBudget == ...
    config.numberOfSensors + 2 * (config.formationCount - 1));

[sensors, ~] = generateMultiFormationTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
nodeCount = config.numberOfSensors;
messageCount = nodeCount + 2 * (config.formationCount - 1);
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
changed = false;

for currentTime = 1:8
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageCount);
    [adjacency, details] = ...
        selectTemporallyBalancedMinimumBackboneV272Policy(context);
    assert(nnz(adjacency) == messageCount);
    assert(~any(adjacency(:) & ...
        ~logical(context.physicalAdjacency(:))));
    assert(all(abs(sum(details.fusionWeightMatrix, 2) - 1) <= 1e-12));
    assert(details.qualityFloorPassed);
    assert(details.realizedMinimumReliabilityRatio >= ...
        protocol.minimumReliabilityRatio - 1e-12);
    assert(details.realizedMaximumReliabilityDrop <= ...
        protocol.maximumReliabilityDrop + 1e-12);
    assert(~details.posteriorUsed && ~details.measurementUsed && ...
        ~details.truthUsed && ~details.futureOutcomeUsed);
    if currentTime == 1
        assert(isequal(adjacency, details.referenceAdjacency));
    else
        changed = changed || ~isequal(adjacency, history(:, :, end));
    end
    history = cat(3, history, adjacency);
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
end
assert(changed);
fprintf('PASS: V272 temporal minimum-backbone invariants\n');
end

function context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageCount)
nodeCount = config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', config, ...
    'staticAdjacency', logical(graph.staticAdjacency)));
context.baseAdjacency = logical(graph.staticAdjacency);
context.physicalAdjacency = logical( ...
    graph.physicalAdjacency(:, :, currentTime));
context.positions = graph.positions(:, :, currentTime);
context.currentTime = currentTime;
context.commConfig = struct('pDropByEdge', pDrop(:, :, currentTime));
context.directedMessageBudget = messageCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.previousAdjacencyHistory = history;
context.previousAdjacencyHistoryCount = size(history, 3);
context.previousAdjacencyHistoryTimes = historyTimes;
end
