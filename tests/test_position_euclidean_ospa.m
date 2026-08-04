function test_position_euclidean_ospa()
% Position OSPA must be unit-consistent and velocity-invariant.

cutoff = 100.0;
order = 2.0;
positionIndices = [1.0, 2.0];
truth = {[0; 0; 1; 2], [10; 0; -2; 3]};
samePositionsDifferentVelocity = { ...
    [0; 0; 1e6; -1e6], [10; 0; -1e6; 1e6]};
components = computePositionEuclideanOspa( ...
    truth, samePositionsDifferentVelocity, cutoff, order, positionIndices);
assert(isequal(components, zeros(3, 1)));

shifted = {[3; 4; 0; 0], [10; 0; 0; 0]};
forward = computePositionEuclideanOspa( ...
    truth, shifted, cutoff, order, positionIndices);
reverse = computePositionEuclideanOspa( ...
    shifted, truth, cutoff, order, positionIndices);
assert(abs(forward(1) - sqrt(25 / 2)) < 1e-12);
assert(abs(forward(2) - sqrt(25 / 2)) < 1e-12);
assert(forward(3) == 0);
assert(max(abs(forward - reverse)) < 1e-12);

missing = computePositionEuclideanOspa( ...
    truth, truth(1), cutoff, order, positionIndices);
assert(abs(missing(1) - sqrt(5000)) < 1e-12);
assert(missing(2) == 0);
assert(abs(missing(3) - sqrt(5000)) < 1e-12);

left2d = {[0; 0], [5; 7], [200; -30]};
right2d = {[2; 1], [8; 8]};
covLeft = repmat({eye(2)}, 1, numel(left2d));
covRight = repmat({eye(2)}, 1, numel(right2d));
[legacyOnPositionSubspace, ~] = ospa( ...
    left2d, left2d, covLeft, right2d, covRight, ...
    struct('eC', cutoff, 'eP', order, 'hC', 0.5, 'hP', 2));
positionOnly = computePositionEuclideanOspa( ...
    left2d, right2d, cutoff, order, positionIndices);
assert(max(abs(positionOnly - legacyOnPositionSubspace)) < 1e-12);

assert(isequal(computePositionEuclideanOspa( ...
    {}, {}, cutoff, order, positionIndices), ...
    zeros(3, 1)));
assert(isequal(computePositionEuclideanOspa( ...
    {}, truth, cutoff, order, positionIndices), ...
    [cutoff; 0; cutoff]));

rowStates = {[0, 0, 9, 9], [10, 0, 9, 9]};
assert(isequal(computePositionEuclideanOspa( ...
    truth, rowStates, cutoff, order, positionIndices), zeros(3, 1)));

assertThrows(@() computePositionEuclideanOspa( ...
    {[0; NaN; 0; 0]}, truth, cutoff, order, positionIndices), ...
    'PositionEuclideanOspa:InvalidState');
assertThrows(@() computePositionEuclideanOspa( ...
    truth, truth, 0, order, positionIndices), ...
    'PositionEuclideanOspa:InvalidCutoff');
assertThrows(@() computePositionEuclideanOspa( ...
    truth, truth, cutoff, 0.5, positionIndices), ...
    'PositionEuclideanOspa:InvalidOrder');
fprintf('PASS: position-only Euclidean OSPA tests\n');
end

function assertThrows(callback, expectedIdentifier)
threw = false;
try
    callback();
catch err
    threw = true;
    assert(strcmp(err.identifier, expectedIdentifier));
end
assert(threw);
end
