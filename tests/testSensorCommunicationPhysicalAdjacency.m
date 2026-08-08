function testSensorCommunicationPhysicalAdjacency()
% Freeze the posterior-independent geometry/range action-set boundary.

positions = [0, 3, 0, 10; 0, 4, 5, 0];
expected = logical([ ...
    0, 1, 1, 0; ...
    1, 0, 1, 0; ...
    1, 1, 0, 0; ...
    0, 0, 0, 0]);
actual = buildSensorCommunicationPhysicalAdjacency(positions, 5);
assert(isequal(actual, expected));
assert(isequal( ...
    buildSensorCommunicationPhysicalAdjacency(positions, inf), ...
    ~eye(4)));
assertErrorId(@() buildSensorCommunicationPhysicalAdjacency( ...
    [positions(:, 1:3), [NaN; 0]], 5), ...
    'SensorCommunicationPhysicalAdjacency:InvalidInput');
fprintf('PASS: sensor communication physical-adjacency tests\n');
end

function assertErrorId(action, expectedId)
didFail = false;
try
    action();
catch errorInfo
    didFail = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Expected %s, received %s.', expectedId, errorInfo.identifier);
end
assert(didFail, 'Expected %s to be raised.', expectedId);
end
