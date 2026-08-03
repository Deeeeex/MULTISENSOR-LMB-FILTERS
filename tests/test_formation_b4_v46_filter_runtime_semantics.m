function test_formation_b4_v46_filter_runtime_semantics()
% End-to-end V46 repair propagation through real filter diagnostics.

global formationB4V46FilterRuntimeTestState;
cleanup = onCleanup(@clearTestState); %#ok<NASGU>
[model, measurements, sensorTrajectories, neighborMap, ...
    commConfig] = buildSyntheticCase();
nodeCount = model.numberOfSensors;

referenceConfig = buildFormationB4V46FixedTriggerConfig( ...
    'v46-repaired-reference-a70-e05', nodeCount);
referenceConfig.topologyPolicyFcn = @captureReferencePolicy;
referenceConfig.topologyPolicyName = ...
    'captureFormationB4V46ReferenceRuntimePolicy';
formationB4V46FilterRuntimeTestState = emptyCaptureState();
runAndAssertArm(model, measurements, sensorTrajectories, ...
    neighborMap, commConfig, referenceConfig, ...
    [2 * nodeCount, 2 * nodeCount], true);

syncConfig = buildFormationB4V46FixedTriggerConfig( ...
    'v46-repaired-sync-all-b4-e20-mc', nodeCount);
syncConfig.topologyPolicyFcn = @captureSynchronizedPolicy;
syncConfig.topologyPolicyName = ...
    'captureFormationB4V46SynchronizedRuntimePolicy';
formationB4V46FilterRuntimeTestState = emptyCaptureState();
runAndAssertArm(model, measurements, sensorTrajectories, ...
    neighborMap, commConfig, syncConfig, ...
    [2 * nodeCount, nodeCount], false);

undirectedConfig = syncConfig;
undirectedConfig.topologyDirectedEnabled = false;
assertErrorId(@() runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, undirectedConfig), ...
    'FormationB4V45:InvalidDirectedRuntime');

fprintf('PASS: FormationB4V46 filter runtime semantics tests\n');
end

function [model, measurements, sensorTrajectories, neighborMap, ...
        commConfig] = buildSyntheticCase()
formationCount = 3;
sensorsPerFormation = 3;
nodeCount = formationCount * sensorsPerFormation;
timeCount = 2;
model = generateMultisensorModel( ...
    nodeCount, zeros(1, nodeCount), ...
    0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
model.simulationLength = timeCount;
model.sensorCommRange = 60;

sceneConfig = struct();
sceneConfig.formationCount = formationCount;
sceneConfig.sensorsPerFormation = sensorsPerFormation;
sceneConfig.numberOfSensors = nodeCount;
sceneConfig.sensorCenterWaypoints = { ...
    [-50; 0], [0; 0], [50; 0]};
sceneConfig.sensorGroupIds = repelem( ...
    1:formationCount, sensorsPerFormation);
identity = buildDynamicTopologyPhysicalIdentityRegistry(sceneConfig);
sceneConfig.sensorPhysicalUids = identity.sensorPhysicalUids;
sceneConfig.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
sceneConfig.sensorLocalRoleUidsByFormation = ...
    identity.sensorLocalRoleUidsByFormation;
sceneConfig.physicalIdentityRegistryCanonicalSha256 = ...
    identity.canonicalSha256;
sensorMetadata = struct( ...
    'sensorPhysicalUids', identity.sensorPhysicalUids, ...
    'formationPhysicalUidsBySensor', ...
        identity.formationPhysicalUidsBySensor, ...
    'physicalIdentityRegistryCanonicalSha256', ...
        identity.canonicalSha256);
model.dynamicTopologyScenario = struct( ...
    'config', sceneConfig, 'sensor', sensorMetadata);

centersByTime = zeros(2, formationCount, timeCount);
centersByTime(:, :, 1) = [-50, 0, 50; 0, 0, 0];
centersByTime(:, :, 2) = [0, -50, 50; 0, 0, 0];
localOffsets = [-4, 2, 2; 0, 3, -3];
sensorTrajectories = cell(1, nodeCount);
for formationIdx = 1:formationCount
    for localIdx = 1:sensorsPerFormation
        sensorIdx = (formationIdx - 1) * ...
            sensorsPerFormation + localIdx;
        positions = squeeze(centersByTime(:, formationIdx, :));
        positions = positions + repmat( ...
            localOffsets(:, localIdx), 1, timeCount);
        sensorTrajectories{sensorIdx} = [ ...
            positions; zeros(2, timeCount)];
    end
end
measurements = repmat({{}}, nodeCount, timeCount);

baseAdjacency = false(nodeCount);
for formationIdx = 1:formationCount
    members = (formationIdx - 1) * sensorsPerFormation + ...
        (1:sensorsPerFormation);
    for localIdx = 1:sensorsPerFormation
        left = members(localIdx);
        right = members(mod(localIdx, sensorsPerFormation) + 1);
        baseAdjacency(left, right) = true;
        baseAdjacency(right, left) = true;
    end
end
crossPlaceholders = [1, 4; 4, 7];
for edgeIdx = 1:size(crossPlaceholders, 1)
    left = crossPlaceholders(edgeIdx, 1);
    right = crossPlaceholders(edgeIdx, 2);
    baseAdjacency(left, right) = true;
    baseAdjacency(right, left) = true;
end
neighborMap = cell(1, nodeCount);
for receiverIdx = 1:nodeCount
    neighborMap{receiverIdx} = unique([receiverIdx, ...
        find(baseAdjacency(receiverIdx, :))]);
end

commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(nodeCount, nodeCount, timeCount), ...
    'linkUniforms', ...
        0.5 * ones(nodeCount, nodeCount, timeCount));
end

function runAndAssertArm(model, measurements, sensorTrajectories, ...
        neighborMap, commConfig, triggerConfig, expectedCounts, ...
        finiteObjectiveRequired)
global formationB4V46FilterRuntimeTestState;
nodeCount = model.numberOfSensors;
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, triggerConfig);

assert(isequal(reshape( ...
    diagnostics.topologyDirectedMessageCount, 1, []), ...
    expectedCounts));
assert(isequal(reshape( ...
    diagnostics.topologyPolicyRepairTriggered, 1, []), ...
    [0, 1]));

for currentTime = 1:2
    capturedAdjacency = formationB4V46FilterRuntimeTestState. ...
        adjacency{currentTime};
    details = formationB4V46FilterRuntimeTestState. ...
        details{currentTime};
    policyAdjacency = logical(diagnostics. ...
        topologyActiveEdge(:, :, currentTime)');
    weights = diagnostics. ...
        topologyPolicyFusionWeightMatrix{currentTime};
    positions = zeros(2, nodeCount);
    for sensorIdx = 1:nodeCount
        positions(:, sensorIdx) = ...
            sensorTrajectories{sensorIdx}(1:2, currentTime);
    end
    physical = buildPhysicalAdjacency( ...
        positions, model.sensorCommRange);
    positive = weights > 1e-12;
    positive(1:nodeCount+1:end) = false;

    assert(isequal(policyAdjacency, capturedAdjacency));
    assert(nnz(policyAdjacency) == expectedCounts(currentTime));
    assert(~any(policyAdjacency(:) & ~physical(:)));
    assert(nnz(details.referenceAdjacency) == 2 * nodeCount);
    assert(~any(details.referenceAdjacency(:) & ~physical(:)));
    assert(isStronglyConnected(details.referenceAdjacency));
    if expectedCounts(currentTime) == 2 * nodeCount
        assert(isStronglyConnected(policyAdjacency));
    end
    assert(isequal(positive, policyAdjacency));
    assert(all(abs(sum(weights, 2) - 1) <= 1e-12));
    assert(max(abs(weights(:) - ...
        details.fusionWeightMatrix(:))) <= 1e-12);
    assert(details.repairTriggered == (currentTime == 2));
    assert(diagnostics.topologyPolicyRepairTriggered(currentTime) == ...
        details.repairTriggered);
    assert(isscalar(diagnostics.topologyPolicyObjective(currentTime)));
    assert(isequaln(diagnostics.topologyPolicyObjective(currentTime), ...
        details.objective));
    if finiteObjectiveRequired
        assert(isfinite(details.objective));
        assert(details.objective == ...
            min(details.parentRouteObjectiveVector));
    else
        assert(isnan(details.objective));
        assert(isempty(details.parentRouteObjectiveVector));
    end
    assert(nnz(diagnostics.attempted(:, :, currentTime)) == ...
        expectedCounts(currentTime));
end
end

function [adjacency, details] = captureReferencePolicy(context)
global formationB4V46FilterRuntimeTestState;
[adjacency, details] = ...
    selectFormationB4V46ReferenceRuntimePolicy(context);
formationB4V46FilterRuntimeTestState.adjacency{context.currentTime} = ...
    adjacency;
formationB4V46FilterRuntimeTestState.details{context.currentTime} = ...
    details;
end

function [adjacency, details] = captureSynchronizedPolicy(context)
global formationB4V46FilterRuntimeTestState;
[adjacency, details] = ...
    selectFormationB4V46SynchronizedRuntimePolicy(context);
formationB4V46FilterRuntimeTestState.adjacency{context.currentTime} = ...
    adjacency;
formationB4V46FilterRuntimeTestState.details{context.currentTime} = ...
    details;
end

function state = emptyCaptureState()
state = struct('adjacency', {cell(1, 2)}, ...
    'details', {cell(1, 2)});
end

function adjacency = buildPhysicalAdjacency(positions, range)
nodeCount = size(positions, 2);
adjacency = false(nodeCount);
for left = 1:nodeCount-1
    for right = left+1:nodeCount
        if norm(positions(:, left) - positions(:, right)) <= range
            adjacency(left, right) = true;
            adjacency(right, left) = true;
        end
    end
end
end

function connected = isStronglyConnected(adjacency)
connected = reachableAll(adjacency) && reachableAll(adjacency');
end

function passed = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end

function assertErrorId(callable, expectedId)
thrown = false;
try
    callable();
catch errorInfo
    thrown = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Unexpected error identifier: %s', errorInfo.identifier);
end
assert(thrown, 'Expected error was not thrown.');
end

function clearTestState()
global formationB4V46FilterRuntimeTestState;
formationB4V46FilterRuntimeTestState = [];
end
