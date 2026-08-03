function test_formation_reliable_kla_context_block_permutation()
% Pure coordinate changes must permute every node-indexed input together.

nodeCount = 9;
groupIds = [10, 10, 10, 20, 20, 20, 30, 30, 30];
base = reshape(1:nodeCount^2, nodeCount, nodeCount);
physical = reshape(101:100 + nodeCount^2, nodeCount, nodeCount);
history = cat(3, base > 20, base > 35, base > 50);
pDrop = reshape(1:nodeCount^2, nodeCount, nodeCount) / 100;

context = struct();
context.localPosteriorBySensor = num2cell(101:109);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', base + 1000));
context.baseAdjacency = base;
context.physicalAdjacency = physical;
context.previousAdjacencyHistory = history;
context.currentTime = 17;
context.commConfig = struct( ...
    'pDropByEdge', pDrop, 'forceDelivery', false, ...
    'deliveryIndependenceAttested', true);
context.triggerConfig = struct('sentinel', 7);

formationOrder = [30, 10, 20];
expectedNewToOld = [7:9, 1:3, 4:6];
[permuted, details] = ...
    permuteFormationReliableKlaContextByBlockOrder( ...
        context, formationOrder);

assert(isequal(details.originalFormationOrder, [10, 20, 30]));
assert(isequal(details.permutedFormationOrder, formationOrder));
assert(isequal(details.newToOldNodeOrder, expectedNewToOld));
assert(isequal(details.oldToNewNodeOrder(expectedNewToOld), 1:nodeCount));
assert(numel(details.canonicalSha256) == 64);
assert(details.pureNodeCoordinatePermutation);
assert(~details.formationLabelsChanged);
assert(~details.physicalMembershipChanged);
assert(isequal(permuted.localPosteriorBySensor, ...
    context.localPosteriorBySensor(expectedNewToOld)));
assert(isequal(permuted.baseAdjacency, ...
    base(expectedNewToOld, expectedNewToOld)));
assert(isequal(permuted.physicalAdjacency, ...
    physical(expectedNewToOld, expectedNewToOld)));
assert(isequal(permuted.previousAdjacencyHistory, ...
    history(expectedNewToOld, expectedNewToOld, :)));
assert(isequal(permuted.commConfig.pDropByEdge, ...
    pDrop(expectedNewToOld, expectedNewToOld)));
assert(isequal(permuted.model.dynamicTopologyScenario. ...
    staticAdjacency, ...
    context.model.dynamicTopologyScenario. ...
        staticAdjacency(expectedNewToOld, expectedNewToOld)));
assert(isequal(permuted.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, groupIds(expectedNewToOld)));
assert(permuted.currentTime == context.currentTime);
assert(isequal(permuted.triggerConfig, context.triggerConfig));

restored = zeros(nodeCount);
restored(expectedNewToOld, expectedNewToOld) = ...
    permuted.baseAdjacency;
assert(isequal(restored, context.baseAdjacency));
restoredDrop = zeros(nodeCount);
restoredDrop(expectedNewToOld, expectedNewToOld) = ...
    permuted.commConfig.pDropByEdge;
assert(isequal(restoredDrop, context.commConfig.pDropByEdge));

assertErrorId(@() permuteFormationReliableKlaContextByBlockOrder( ...
    context, [30, 30, 20]), ...
    'FormationKlaBlockPermutation:InvalidContext');
assertErrorId(@() permuteFormationReliableKlaContextByBlockOrder( ...
    context, [30, 10, 40]), ...
    'FormationKlaBlockPermutation:InvalidContext');
bad = context;
bad.commConfig.pDropByEdge = zeros(nodeCount - 1);
assertErrorId(@() permuteFormationReliableKlaContextByBlockOrder( ...
    bad, formationOrder), ...
    'FormationKlaBlockPermutation:InvalidContext');

fprintf('Formation reliable-KLA context block-permutation tests passed.\n');
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
