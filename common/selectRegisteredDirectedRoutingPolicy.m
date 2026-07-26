function [adjacency, details] = ...
    selectRegisteredDirectedRoutingPolicy(context, mode, options)
% SELECTREGISTEREDDIRECTEDROUTINGPOLICY Deterministic directed controls.
%
% Every receiver uses exactly one sender. The first three controls stay
% within the receiver's formation; physical-round-robin cycles through the
% current physical sender set, including cross-formation senders when they
% are available. All controls are posterior- and truth-independent.
%
% Supported modes:
%   fixed-index-star       first role sends to all other local roles
%   fixed-balanced-cycle   one fixed cyclic shift per formation
%   round-robin-balanced   cycle through all non-self local roles
%   physical-round-robin   cycle through the current physical action set

if nargin < 2 || isempty(mode)
    mode = 'fixed-index-star';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
sourceWeight = getField(options, 'sourceWeight', 0.50);
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('sourceWeight must be a finite scalar strictly between 0 and 1.');
end

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;
groupIds = resolveGroupIds(context.model, nodeCount);
selectedSources = nan(1, nodeCount);

if strcmp(mode, 'physical-round-robin')
    anchorTime = round(getField(options, 'anchorTime', 1));
    phase = round(getField(options, 'phase', 1));
    for receiverIdx = 1:nodeCount
        senders = reshape(find(physical(receiverIdx, :)), 1, []);
        if isempty(senders)
            continue;
        end
        senderCursor = 1 + mod( ...
            context.currentTime - anchorTime + phase - 1, ...
            numel(senders));
        selectedSources(receiverIdx) = senders(senderCursor);
    end
else
    groups = unique(groupIds, 'stable');
    for groupCursor = 1:numel(groups)
        members = reshape(find(groupIds == groups(groupCursor)), 1, []);
        groupSize = numel(members);
        if groupSize < 2
            continue;
        end
        switch mode
            case 'fixed-index-star'
                selectedSources(members(1)) = members(2);
                selectedSources(members(2:end)) = members(1);
            case 'fixed-balanced-cycle'
                shift = normalizeShift( ...
                    getField(options, 'phase', 1), groupSize);
                selectedSources(members) = ...
                    circshift(members, [0, -shift]);
            case 'round-robin-balanced'
                anchorTime = round(getField(options, 'anchorTime', 1));
                phase = round(getField(options, 'phase', 1));
                shift = 1 + mod( ...
                    context.currentTime - anchorTime + phase - 1, ...
                    groupSize - 1);
                selectedSources(members) = ...
                    circshift(members, [0, -shift]);
            otherwise
                error( ...
                    'Unknown registered directed-routing mode: %s', ...
                    mode);
        end
    end
end

selectedReceivers = reshape(find(isfinite(selectedSources)), 1, []);
messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if numel(selectedReceivers) > messageBudget
    error([ ...
        'Registered directed control needs one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
for receiverIdx = selectedReceivers
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error([ ...
            'Registered directed control selected a non-physical route ', ...
            'at t=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
end

adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = selectedReceivers
    senderIdx = selectedSources(receiverIdx);
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

details = struct();
details.mode = ['registered-directed-', mode];
details.objective = NaN;
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = nnz(adjacency);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
details.posteriorUsed = false;
details.truthUsed = false;
details.currentLinkReliabilityUsed = false;
details.currentPhysicalActionSetUsed = ...
    strcmp(mode, 'physical-round-robin');
end

function shift = normalizeShift(phase, groupSize)
shift = 1 + mod(round(phase) - 1, groupSize - 1);
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error([ ...
        'Registered same-formation directed controls require explicit ', ...
        'sensorGroupIds metadata.']);
end
groupIds = reshape( ...
    model.dynamicTopologyScenario.config.sensorGroupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || any(groupIds < 1)
    error('sensorGroupIds must contain one valid group ID per sensor.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
