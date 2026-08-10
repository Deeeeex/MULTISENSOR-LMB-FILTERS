function test_static_vs_dynamic_routing_v91()
% TEST_STATIC_VS_DYNAMIC_ROUTING_V91 Focused fixed-route contract test.

groupIds = repelem(1:4, 3);
nodeCount = numel(groupIds);
registered = false(nodeCount);
for groupIdx = 1:4
    members = find(groupIds == groupIdx);
    for memberIdx = 1:numel(members)
        left = members(memberIdx);
        right = members(1 + mod(memberIdx, numel(members)));
        registered(left, right) = true;
        registered(right, left) = true;
    end
end
for groupIdx = 1:3
    left = find(groupIds == groupIdx, 1);
    right = find(groupIds == groupIdx + 1, 1);
    registered(left, right) = true;
    registered(right, left) = true;
end
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', registered));
context.baseAdjacency = registered;
context.directedMessageBudget = 2 * nodeCount;
context.currentTime = 1;
physical = false(nodeCount);
for groupIdx = 1:4
    members = find(groupIds == groupIdx);
    physical(members, members) = true;
end
for groupIdx = 1:3
    left = find(groupIds == groupIdx);
    right = find(groupIds == groupIdx + 1);
    physical(left, right) = true;
    physical(right, left) = true;
end
physical(1:nodeCount+1:end) = false;
context.physicalAdjacency = physical;
[firstAdjacency, firstDetails] = ...
    selectStaticVsDynamicRoutingV91StaticPolicy(context);

context.currentTime = 17;
context.physicalAdjacency = firstAdjacency | firstAdjacency';
[laterAdjacency, laterDetails] = ...
    selectStaticVsDynamicRoutingV91StaticPolicy(context);
assert(isequal(firstAdjacency, laterAdjacency));
assert(isequal(firstDetails.fusionWeightMatrix, ...
    laterDetails.fusionWeightMatrix));
assert(nnz(firstAdjacency) == 2 * nodeCount);
assert(all(sum(firstAdjacency, 2) == 2));
assert(~firstDetails.currentPhysicalActionSetUsed);
assert(firstDetails.currentPhysicalFeasibilityChecked);

preflightInputs = struct();
preflightInputs.config = struct( ...
    'presetName', 'synthetic-v91', ...
    'numberOfSensors', nodeCount, 'simulationLength', 5);
preflightInputs.model = context.model;
preflightInputs.graphData = struct( ...
    'staticAdjacency', registered, ...
    'physicalAdjacency', repmat(physical, 1, 1, 5));
preflight = preflightStaticVsDynamicRoutingV91(preflightInputs);
assert(preflight.initialRouteAndWeightMatched);
assert(preflight.dynamicEquivalentToFrozenStatic);
assert(preflight.staticUniqueTopologyCount == 1);
assert(preflight.dynamicUniqueTopologyCount == 1);
certificate = certifyPhysicalTreeReferenceStaticV91(preflightInputs);
assert(certificate.referenceFunctionallyStatic);
assert(certificate.physicalGraphUniqueCount == 1);
assert(certificate.referenceRouteUniqueCount == 1);
assert(certificate.referenceWeightUniqueCount == 1);

protocol = getStaticVsDynamicRoutingV91Protocol();
config = buildStaticVsDynamicRoutingV91TriggerConfig( ...
    protocol.staticArmId, protocol.expectedNodeCounts(1));
assert(strcmp(config.missingLabelFusionMode, ...
    'support-renormalized'));
assert(config.topologyDirectedMessageBudget == ...
    2 * protocol.expectedNodeCounts(1));
assert(strcmp(func2str(config.topologyPolicyFcn), ...
    'selectStaticVsDynamicRoutingV91StaticPolicy'));
fprintf('PASS: V91 static-vs-dynamic routing contract\n');
end
