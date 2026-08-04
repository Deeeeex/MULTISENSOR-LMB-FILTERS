function components = computePositionEuclideanOspa( ...
    leftStates, rightStates, cutoff, order, positionIndices)
% COMPUTEPOSITIONEUCLIDEANOSPA Position-only Euclidean OSPA components.
%
% The repository state is [x; y; vx; vy].  This helper deliberately
% selects the registered position coordinates before computing OSPA, so
% the base distance has one physical unit (metres).  It returns
% [total; localization; cardinality].

validateStateSet(leftStates, positionIndices, 'leftStates');
validateStateSet(rightStates, positionIndices, 'rightStates');
if ~isa(cutoff, 'double') || ~isreal(cutoff) || ~isscalar(cutoff) || ...
        ~isfinite(cutoff) || cutoff <= 0
    error('PositionEuclideanOspa:InvalidCutoff', ...
        'cutoff must be one finite positive double scalar.');
end
if ~isa(order, 'double') || ~isreal(order) || ~isscalar(order) || ...
        ~isfinite(order) || order < 1
    error('PositionEuclideanOspa:InvalidOrder', ...
        'order must be one finite double scalar greater than or equal to one.');
end

leftCount = numel(leftStates);
rightCount = numel(rightStates);
if leftCount == 0 && rightCount == 0
    components = zeros(3, 1);
    return;
end
if leftCount == 0 || rightCount == 0
    components = [cutoff; 0; cutoff];
    return;
end

cost = zeros(leftCount, rightCount);
for leftIdx = 1:leftCount
    leftPosition = reshape( ...
        leftStates{leftIdx}(positionIndices), [], 1);
    for rightIdx = 1:rightCount
        rightPosition = reshape( ...
            rightStates{rightIdx}(positionIndices), [], 1);
        distance = norm(leftPosition - rightPosition);
        cost(leftIdx, rightIdx) = min(cutoff, distance) ^ order;
    end
end
[~, assignmentCost] = Hungarian(cost);
if ~isa(assignmentCost, 'double') || ~isreal(assignmentCost) || ...
        ~isscalar(assignmentCost) || ~isfinite(assignmentCost) || ...
        assignmentCost < 0
    error('PositionEuclideanOspa:InvalidAssignmentCost', ...
        'Hungarian returned an invalid assignment cost.');
end

normalizer = max(leftCount, rightCount);
cardinalityGap = abs(leftCount - rightCount);
localizationPower = assignmentCost / normalizer;
cardinalityPower = cutoff ^ order * cardinalityGap / normalizer;
components = [ ...
    (localizationPower + cardinalityPower) ^ (1 / order); ...
    localizationPower ^ (1 / order); ...
    cardinalityPower ^ (1 / order)];
end

function validateStateSet(states, positionIndices, argumentName)
if ~iscell(states) || (~isempty(states) && ~isvector(states))
    error('PositionEuclideanOspa:InvalidStateSet', ...
        '%s must be a cell vector.', argumentName);
end
if ~isa(positionIndices, 'double') || ~isreal(positionIndices) || ...
        ~isvector(positionIndices) || isempty(positionIndices) || ...
        any(~isfinite(positionIndices)) || ...
        any(positionIndices < 1) || ...
        any(positionIndices ~= floor(positionIndices)) || ...
        numel(unique(positionIndices)) ~= numel(positionIndices)
    error('PositionEuclideanOspa:InvalidPositionIndices', ...
        'positionIndices must contain unique positive integer doubles.');
end
maximumIndex = max(positionIndices);
for stateIdx = 1:numel(states)
    state = states{stateIdx};
    if ~isnumeric(state) || ~isreal(state) || ~isvector(state) || ...
            numel(state) < maximumIndex || any(~isfinite(state(:)))
        error('PositionEuclideanOspa:InvalidState', ...
            '%s{%d} is not a finite real state vector.', ...
            argumentName, stateIdx);
    end
end
end
