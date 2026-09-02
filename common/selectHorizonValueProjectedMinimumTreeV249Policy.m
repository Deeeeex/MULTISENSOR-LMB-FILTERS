function [adjacency, details] = ...
        selectHorizonValueProjectedMinimumTreeV249Policy( ...
            context, requestedFormationUidPairs)
% SELECTHORIZONVALUEPROJECTEDMINIMUMTREEV249POLICY Force one tree safely.
%
% This policy is used only by the offline H=3 oracle.  It encodes the
% requested formation tree as the incumbent seen by V242.  V240/V242 then
% perform the normal distinct-gateway assignment, physical projection,
% strong-connectivity construction and KLA-row construction.  If the
% requested tree becomes infeasible inside the H=3 window, the inherited
% causal repair chooses a feasible minimum-edit replacement.

protocol = getHorizonValueProjectedMinimumTreeV249Protocol();
[pairs, formationUids, members] = normalizeRequestedTree( ...
    context, requestedFormationUidPairs);
nodeCount = numel(context.localPosteriorBySensor);
forcedHistory = false(nodeCount);
for pairIdx = 1:size(pairs, 1)
    left = find(formationUids == pairs(pairIdx, 1), 1);
    right = find(formationUids == pairs(pairIdx, 2), 1);
    leftSensor = members{left}(1);
    rightSensor = members{right}(1);
    forcedHistory(leftSensor, rightSensor) = true;
    forcedHistory(rightSensor, leftSensor) = true;
end

forcedContext = context;
forcedContext.previousAdjacency = forcedHistory;
forcedContext.previousAdjacencyHistory = forcedHistory;
forcedContext.previousAdjacencyHistoryCount = 1;
forcedContext.previousAdjacencyHistoryTimes = ...
    max(0, context.currentTime - 1);
[adjacency, details] = ...
    selectCausalMinimumFormationBackboneV242Policy(forcedContext);

appliedPairs = normalizePairRows( ...
    details.currentFormationTreePairs);
requestedApplied = isequal(appliedPairs, pairs);
details.contractVersion = ...
    'horizon-value-projected-minimum-tree-v249-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'offline-forced-tree-with-safe-v242-projection';
details.backboneMode = details.mode;
details.requestedFormationUidPairs = pairs;
details.appliedFormationUidPairs = appliedPairs;
details.requestedTreeApplied = requestedApplied;
details.requestedTreeFallbackUsed = ~requestedApplied;
details.offlineOracleCandidate = true;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'horizon-value-projected-minimum-tree-v249-schedule-v1';
schedule.phase = 'v249-h3-forced-tree';
schedule.requestedFormationUidPairs = pairs;
schedule.appliedFormationUidPairs = appliedPairs;
schedule.requestedTreeApplied = requestedApplied;
schedule.requestedTreeFallbackUsed = ~requestedApplied;
schedule.offlineOracleCandidate = true;
details.scheduleCertificate = schedule;
end

function [pairs, formationUids, members] = ...
        normalizeRequestedTree(context, requested)
required = {'localPosteriorBySensor', 'model', ...
    'formationPhysicalUidsBySensor', 'currentTime'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('HorizonValueProjectedMinimumTreeV249:InvalidContext', ...
        'The forced-tree policy context is incomplete.');
end
nodeCount = numel(context.localPosteriorBySensor);
formationBySensor = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
if numel(formationBySensor) ~= nodeCount || ...
        any(~isfinite(formationBySensor))
    error('HorizonValueProjectedMinimumTreeV249:InvalidIdentity', ...
        'Formation physical identities are malformed.');
end
formationUids = sort(unique(formationBySensor));
formationCount = numel(formationUids);
members = cell(1, formationCount);
for formationIdx = 1:formationCount
    members{formationIdx} = find( ...
        formationBySensor == formationUids(formationIdx));
end
pairs = normalizePairRows(requested);
if ~isnumeric(pairs) || size(pairs, 2) ~= 2 || ...
        size(pairs, 1) ~= formationCount - 1 || ...
        any(~ismember(pairs(:), formationUids)) || ...
        any(pairs(:, 1) == pairs(:, 2)) || ...
        size(unique(pairs, 'rows'), 1) ~= size(pairs, 1)
    error('HorizonValueProjectedMinimumTreeV249:InvalidTree', ...
        'The requested physical-UID pairs do not define a tree.');
end
tree = false(formationCount);
for pairIdx = 1:size(pairs, 1)
    left = find(formationUids == pairs(pairIdx, 1), 1);
    right = find(formationUids == pairs(pairIdx, 2), 1);
    tree(left, right) = true;
    tree(right, left) = true;
end
if ~isConnected(tree)
    error('HorizonValueProjectedMinimumTreeV249:InvalidTree', ...
        'The requested physical-UID pairs are disconnected.');
end
end

function pairs = normalizePairRows(pairs)
if isempty(pairs)
    pairs = zeros(0, 2);
    return;
end
if ~isnumeric(pairs) || size(pairs, 2) ~= 2
    error('HorizonValueProjectedMinimumTreeV249:InvalidPairs', ...
        'Formation pairs must be a numeric K-by-2 matrix.');
end
pairs = sort(pairs, 2);
pairs = sortrows(pairs, [1, 2]);
end

function connected = isConnected(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end
