function test_direct_graph_formation_encoding_v219()
% TEST_DIRECT_GRAPH_FORMATION_ENCODING_V219 Permutation equivariance.

snapshot = syntheticSnapshot();
first = encodeDirectGraphFormationStateV219(snapshot, 2);
order = [3, 1, 2];
permuted = snapshot;
permuted.formationIds = snapshot.formationIds(order);
permuted.nodeFeatures = snapshot.nodeFeatures(order, :);
permuted.receiverRowSenderColumnAdjacency = ...
    snapshot.receiverRowSenderColumnAdjacency(order, order);
permuted.edgeFeatures = snapshot.edgeFeatures(order, order, :);
targetRow = find(order == 2);
second = encodeDirectGraphFormationStateV219(permuted, targetRow);
assert(isequal(first.featureNames, second.featureNames));
assert(max(abs(first.featureVector - second.featureVector)) < 1e-12);
fprintf('test_direct_graph_formation_encoding_v219 passed\n');
end

function snapshot = syntheticSnapshot()
snapshot = struct();
snapshot.contractVersion = ...
    'direct-graph-formation-learning-snapshot-v218-v1';
snapshot.formationIds = [1, 2, 3];
snapshot.formationCount = 3;
snapshot.nodeFeatureNames = {'x', 'y'};
snapshot.nodeFeatures = [1, 2; 3, 4; 5, 6];
snapshot.edgeFeatureNames = {'e'};
snapshot.receiverRowSenderColumnAdjacency = logical( ...
    [0, 1, 0; 0, 0, 1; 1, 0, 0]);
snapshot.edgeFeatures = zeros(3, 3, 1);
snapshot.edgeFeatures(:, :, 1) = [0, 2, 0; 0, 0, 4; 6, 0, 0];
snapshot.numericFormationIdentifiersUsedAsFeatures = false;
snapshot.truthUsed = false;
snapshot.futureInformationUsed = false;
end
