function test_registered_formation_backbone_residual_tour()
% TEST_REGISTEREDFORMATIONBACKBONERESIDUALTOUR Structural contracts.

presets = { ...
    'm24-formation-fov', ...
    'm24-formation-fov-convoy', ...
    'm24-formation-fov-relay', ...
    'x36-formation-fov-convoy', ...
    'x36-formation-fov-relay'};
times = 1:160;
for presetIdx = 1:numel(presets)
    config = buildDynamicTopologyScenarioConfig(presets{presetIdx});
    [trajectories, ~] = generateMultiFormationTrajectories(config);
    graph = buildDynamicTopologyGraphs(config, trajectories);
    for timeIdx = times
        [adjacency, details] = ...
            buildRegisteredFormationBackboneResidualTour( ...
                config.sensorGroupIds, graph.staticAdjacency, ...
                graph.physicalAdjacency(:, :, timeIdx));
        assert(strcmp(details.contractVersion, ...
            'registered-formation-backbone-residual-tour-v1'));
        assert(details.nodeCount == config.numberOfSensors);
        assert(details.formationCount == config.formationCount);
        assert(size(details.backboneEdges, 1) == ...
            nnz(triu(details.registeredFormationAdjacency, 1)));
        assert(details.crossFormationMessageCount == ...
            nnz(details.registeredFormationAdjacency));
        assert(all(sum(adjacency, 2) == 1));
        assert(all(sum(adjacency, 1) == 1));
        currentPhysical = graph.physicalAdjacency(:, :, timeIdx);
        assert(~any(adjacency(:) & ~currentPhysical(:)));
        assert(isequal(sort(details.cycleOrder), ...
            1:config.numberOfSensors));
        assert(all(cellfun(@numel, ...
            details.incomingCrossReceiversByFormation) == ...
                details.backboneDegrees));
        assert(details.maximumCrossSourceLoad == 1);
        assert(details.maximumCrossReceiverLoad == 1);
        for edgeIdx = 1:size(details.crossFormationPairs, 1)
            pair = details.crossFormationPairs(edgeIdx, :);
            assert(details.registeredFormationAdjacency( ...
                pair(1), pair(2)));
        end
        for backboneEdgeIdx = 1:size(details.backboneEdges, 1)
            pair = details.backboneEdges(backboneEdgeIdx, :);
            forward = all(details.crossFormationPairs == pair, 2);
            reverse = all(details.crossFormationPairs == fliplr(pair), 2);
            assert(nnz(forward) == 1 && nnz(reverse) == 1);
        end
        if strcmp(presets{presetIdx}, 'm24-formation-fov')
            assert(any(all(details.backboneEdges == [3, 4], 2)));
            assert(nnz(all(details.crossFormationPairs == [3, 4], 2)) == 1);
            assert(nnz(all(details.crossFormationPairs == [4, 3], 2)) == 1);
        end
    end
end

context = buildPolicyContext();
[adjacency, details] = ...
    selectFormationBackboneResidualTourPolicy(context);
assert(strcmp(details.contractVersion, ...
    'formation-backbone-residual-tour-policy-v1'));
assert(strcmp(details.backboneMode, ...
    'fixed-index-plus-registered-formation-backbone-residual-tour'));
assert(all(abs(sum(details.fusionWeightMatrix, 2) - 1) < 1e-12));
assert(~any(adjacency(:) & ~context.physicalAdjacency(:)));
assert(details.crossFormationMessageCount == 6);
assert(details.messageCount == 48);
assert(details.maximumMessagesPerReceiver == 2);
assert(details.duplicateSourceFraction == 0);
assert(all(details.dominantSourcesByReceiver ~= ...
    details.residualSourcesByReceiver));
assert(~details.payloadConstraintEnforced);
assert(isnan(details.payloadLimitPassed));
assert(~details.sensorWindowMature && ...
    isnan(details.sensorWindowStrongConnected));
assert(~details.formationWindowMature && ...
    isnan(details.formationWindowStrongConnected));
assert(~details.posteriorUsed && ~details.truthUsed);
expectedWeights = details.fusionWeightMatrix;
for receiverIdx = 1:24
    assert(abs(expectedWeights(receiverIdx, receiverIdx) - 0.25) < 1e-12);
    assert(abs(expectedWeights(receiverIdx, ...
        details.dominantSourcesByReceiver(receiverIdx)) - 0.70) < 1e-12);
    assert(abs(expectedWeights(receiverIdx, ...
        details.residualSourcesByReceiver(receiverIdx)) - 0.05) < 1e-12);
end
[repeatAdjacency, repeatDetails] = ...
    selectFormationBackboneResidualTourPolicy(context);
assert(isequal(repeatAdjacency, adjacency));
assert(strcmp(computeCanonicalValueSha256(details.tour), ...
    computeCanonicalValueSha256(repeatDetails.tour)));

assertFailsInsufficientSensors();
assertFailsNonphysicalTour();
assertFailsAsymmetricRegisteredGraph();
assertFailsTooManyFormations();
assertEightFormationPath();
fprintf('PASS: registered formation-backbone residual-tour tests\n');
end

function context = buildPolicyContext()
formationCount = 4;
sensorsPerFormation = 6;
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
registered = completeWithinAndTree(groupIds, [1, 2; 2, 3; 3, 4]);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', registered);
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = model;
context.physicalAdjacency = logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = registered;
context.directedMessageBudget = 2 * nodeCount;
end

function adjacency = completeWithinAndTree(groupIds, treeEdges)
nodeCount = numel(groupIds);
adjacency = false(nodeCount);
for left = 1:nodeCount-1
    for right = left+1:nodeCount
        same = groupIds(left) == groupIds(right);
        pair = sort([groupIds(left), groupIds(right)]);
        allowedPair = any(all(treeEdges == pair, 2));
        if same || allowedPair
            adjacency(left, right) = true;
            adjacency(right, left) = true;
        end
    end
end
end

function assertFailsInsufficientSensors()
groupIds = repelem(1:5, 3);
registered = completeWithinAndTree( ...
    groupIds, [1, 2; 1, 3; 1, 4; 1, 5]);
physical = logical(ones(15) - eye(15));
failed = false;
try
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registered, physical);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBackboneTour:InsufficientSensors');
end
assert(failed);
end

function assertFailsNonphysicalTour()
groupIds = repelem(1:3, 3);
registered = completeWithinAndTree(groupIds, [1, 2; 2, 3]);
physical = registered;
physical(1, 5) = false;
physical(5, 1) = false;
failed = false;
try
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registered, physical);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBackboneTour:NonphysicalTour');
end
assert(failed);
end

function assertEightFormationPath()
groupIds = repelem(1:8, 6);
treeEdges = [(1:7)', (2:8)'];
registered = completeWithinAndTree(groupIds, treeEdges);
physical = logical(ones(48) - eye(48));
[adjacency, details] = ...
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registered, physical);
assert(all(sum(adjacency, 1) == 1));
assert(all(sum(adjacency, 2) == 1));
assert(details.crossFormationMessageCount == 14);
assert(size(details.backboneEdges, 1) == 7);
end

function assertFailsAsymmetricRegisteredGraph()
groupIds = repelem(1:2, 3);
registered = completeWithinAndTree(groupIds, [1, 2]);
registered(1, 4) = false;
physical = logical(ones(6) - eye(6));
failed = false;
try
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registered, physical);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBackboneTour:InvalidContract');
end
assert(failed);
end

function assertFailsTooManyFormations()
groupIds = repelem(1:9, 3);
treeEdges = [(1:8)', (2:9)'];
registered = completeWithinAndTree(groupIds, treeEdges);
physical = logical(ones(27) - eye(27));
failed = false;
try
    buildRegisteredFormationBackboneResidualTour( ...
        groupIds, registered, physical);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBackboneTour:InvalidContract');
end
assert(failed);
end
