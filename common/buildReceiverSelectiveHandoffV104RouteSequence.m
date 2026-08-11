function route = buildReceiverSelectiveHandoffV104RouteSequence( ...
        referenceAdjacency, referenceWeights, v103Route, ...
        selectedReceiverIdsByTime, physicalAdjacency, groupIds, options)
% BUILDRECEIVERSELECTIVEHANDOFFV104ROUTESEQUENCE Subset V103 receiver rows.

if nargin < 7 || isempty(options)
    options = struct();
end
referenceAdjacency = logical(referenceAdjacency);
physicalAdjacency = logical(physicalAdjacency);
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
horizonSteps = numel(selectedReceiverIdsByTime);
validV103 = isstruct(v103Route) && isscalar(v103Route) && ...
    isfield(v103Route, 'referenceAdjacency') && ...
    isfield(v103Route, 'referenceFusionWeights') && ...
    isfield(v103Route, 'adjacencyByTime') && ...
    isfield(v103Route, 'fusionWeightsByTime') && ...
    isfield(v103Route, 'changedReceiverIndicesByTime') && ...
    isfield(v103Route, 'gatewayIndicesByFormation');
if ~validV103 || horizonSteps < 1 || ...
        ~isequal(size(referenceAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(referenceWeights), [nodeCount, nodeCount]) || ...
        ~isequal(size(physicalAdjacency), [nodeCount, nodeCount]) || ...
        ~isequal(logical(v103Route.referenceAdjacency), ...
            referenceAdjacency) || ...
        max(abs(v103Route.referenceFusionWeights(:) - ...
            referenceWeights(:))) > 1e-12 || ...
        size(v103Route.adjacencyByTime, 3) ~= horizonSteps || ...
        size(v103Route.fusionWeightsByTime, 3) ~= horizonSteps
    error('ReceiverSelectiveV104:InvalidRouteInput', ...
        'The V103 route or receiver schedule is invalid.');
end

adjacencyByTime = repmat(referenceAdjacency, 1, 1, horizonSteps);
weightsByTime = repmat(referenceWeights, 1, 1, horizonSteps);
selected = cell(1, horizonSteps);
for timeIdx = 1:horizonSteps
    receiverIds = unique(reshape( ...
        selectedReceiverIdsByTime{timeIdx}, 1, []), 'stable');
    available = reshape( ...
        v103Route.changedReceiverIndicesByTime{timeIdx}, 1, []);
    if any(~ismember(receiverIds, available)) || ...
            any(~isfinite(receiverIds)) || ...
            any(receiverIds ~= round(receiverIds)) || ...
            any(receiverIds < 1) || any(receiverIds > nodeCount)
        error('ReceiverSelectiveV104:ReceiverOutsideV103', ...
            'A selected V104 receiver is not changed by V103 on that page.');
    end
    for receiverIdx = receiverIds
        adjacencyByTime(receiverIdx, :, timeIdx) = ...
            v103Route.adjacencyByTime(receiverIdx, :, timeIdx);
        weightsByTime(receiverIdx, :, timeIdx) = ...
            v103Route.fusionWeightsByTime(receiverIdx, :, timeIdx);
    end
    validatePage(adjacencyByTime(:, :, timeIdx), ...
        weightsByTime(:, :, timeIdx), referenceAdjacency, ...
        referenceWeights, physicalAdjacency);
    selected{timeIdx} = receiverIds;
end

route = struct();
route.contractVersion = ...
    'receiver-selective-handoff-v104-route-sequence-v1';
route.referenceAdjacency = referenceAdjacency;
route.referenceFusionWeights = referenceWeights;
route.adjacencyByTime = adjacencyByTime;
route.fusionWeightsByTime = weightsByTime;
route.gatewayIndicesByFormation = ...
    v103Route.gatewayIndicesByFormation;
route.changedReceiverIndicesByTime = selected;
route.selectedReceiverIdsByTime = selected;
previousHistory = getField(options, ...
    'previousAdjacencyHistory', false(nodeCount, nodeCount, 0));
if size(previousHistory, 3) >= 2
    [route.rollingB3SensorPass, route.rollingB3FormationPass] = ...
        rollingB3Pass(previousHistory, adjacencyByTime, groupIds);
    if ~all(route.rollingB3SensorPass) || ...
            ~all(route.rollingB3FormationPass)
        error('ReceiverSelectiveV104:RollingB3Failure', ...
            'The frozen V104 receiver sequence violates rolling-B3.');
    end
else
    route.rollingB3SensorPass = false(1, 0);
    route.rollingB3FormationPass = false(1, 0);
end
route.truthUsed = true;
route.futureOutcomeUsed = true;
end

function validatePage(adjacency, weights, referenceAdjacency, ...
        referenceWeights, physical)
support = adjacency | logical(eye(size(adjacency, 1)));
valid = ~any(adjacency(:) & ~physical(:)) && ...
    nnz(adjacency) == nnz(referenceAdjacency) && ...
    isequal(sum(adjacency, 2), sum(referenceAdjacency, 2)) && ...
    rowWeightMultisetParity(weights, referenceWeights) && ...
    ~any(weights(:) > 1e-12 & ~support(:)) && ...
    all(isfinite(weights(:))) && all(weights(:) >= -1e-12) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12);
if ~valid
    error('ReceiverSelectiveV104:RouteParityFailure', ...
        'A selected receiver row violates the frozen route contract.');
end
end

function pass = rowWeightMultisetParity(first, second)
pass = true;
for receiverIdx = 1:size(first, 1)
    a = sort(first(receiverIdx, first(receiverIdx, :) > 1e-12));
    b = sort(second(receiverIdx, second(receiverIdx, :) > 1e-12));
    if numel(a) ~= numel(b) || any(abs(a - b) > 1e-12)
        pass = false;
        return;
    end
end
end

function [sensorPass, formationPass] = ...
        rollingB3Pass(previousHistory, sequence, groupIds)
history = logical(previousHistory(:, :, end-1:end));
sensorPass = false(1, size(sequence, 3));
formationPass = false(1, size(sequence, 3));
for stepIdx = 1:size(sequence, 3)
    pages = cat(3, history, sequence(:, :, stepIdx));
    window = any(pages(:, :, max(1, end-2):end), 3);
    sensorPass(stepIdx) = isStronglyConnected(window);
    formationPass(stepIdx) = isStronglyConnected( ...
        collapseToFormations(window, groupIds));
    history(:, :, end + 1) = sequence(:, :, stepIdx); %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
    end
end
end

function formation = collapseToFormations(adjacency, groupIds)
groups = unique(reshape(groupIds, 1, []), 'stable');
formation = false(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = groupIds == groups(receiverGroupIdx);
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        formation(receiverGroupIdx, senderGroupIdx) = ...
            any(any(adjacency(receivers, senders)));
    end
end
formation(1:numel(groups)+1:end) = false;
end

function connected = isStronglyConnected(adjacency)
senderAdjacency = logical(adjacency');
connected = reachableAll(senderAdjacency) && ...
    reachableAll(senderAdjacency');
end

function connected = reachableAll(adjacency)
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
connected = all(visited);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
