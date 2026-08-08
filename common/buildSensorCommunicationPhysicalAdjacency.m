function adjacency = ...
    buildSensorCommunicationPhysicalAdjacency(positions, commRange)
% BUILDSENSORCOMMUNICATIONPHYSICALADJACENCY Geometry-only action set.

% The physical communication graph is deliberately independent of any
% posterior-derived topology score.  A pair is feasible exactly when its
% current Euclidean separation does not exceed the communication range.

if nargin ~= 2 || ~isa(positions, 'double') || ...
        ~isreal(positions) || size(positions, 1) ~= 2 || ...
        any(~isfinite(positions(:))) || ...
        ~isa(commRange, 'double') || ~isreal(commRange) || ...
        ~isscalar(commRange) || isnan(commRange) || commRange < 0
    error('SensorCommunicationPhysicalAdjacency:InvalidInput', ...
        'Finite 2-by-N positions and a nonnegative range are required.');
end

nodeCount = size(positions, 2);
adjacency = false(nodeCount);
for leftIdx = 1:nodeCount-1
    for rightIdx = leftIdx+1:nodeCount
        feasible = isinf(commRange) || ...
            norm(positions(:, leftIdx) - ...
                positions(:, rightIdx)) <= commRange;
        adjacency(leftIdx, rightIdx) = feasible;
        adjacency(rightIdx, leftIdx) = feasible;
    end
end
end
