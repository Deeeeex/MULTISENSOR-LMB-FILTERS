function [adjacency, details] = ...
    selectRegisteredGatewayRoutingPolicy(context, mode, options)
% SELECTREGISTEREDGATEWAYROUTINGPOLICY Posterior-independent controls.
%
% All modes start from balanced intra-formation round-robin routing and
% replace at most one receiver per formation with a cross-formation route.
%
%   fixed-ring     fixed lexicographic gateway toward the next formation
%   rotating-ring  cyclic gateway role toward the next formation
%   link-aware     highest-current-reliability cross route

if nargin < 2 || isempty(mode)
    mode = 'fixed-ring';
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
sourceWeight = getField(options, 'sourceWeight', 0.50);
baselinePhase = round(getField(options, 'baselinePhase', 1));
phase = round(getField(options, 'phase', 1));
anchorTime = round(getField(options, 'anchorTime', 1));
maxCrossPerFormation = max(1, floor(getField( ...
    options, 'maxCrossPerFormation', 1)));
maxCrossSourceLoad = max(1, floor(getField( ...
    options, 'maxCrossSourceLoad', 1)));
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('sourceWeight must be strictly between zero and one.');
end

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;
groupIds = resolveGroupIds(context.model, nodeCount);
groups = unique(groupIds, 'stable');

baseOptions = struct( ...
    'sourceWeight', sourceWeight, ...
    'phase', baselinePhase, ...
    'anchorTime', anchorTime);
[~, baseDetails] = selectRegisteredDirectedRoutingPolicy( ...
    context, 'round-robin-balanced', baseOptions);
selectedSources = baseDetails.selectedSourcesByReceiver;
proposals = repmat(struct( ...
    'receiver', NaN, 'sender', NaN, ...
    'formation', NaN, 'priority', -inf), 1, 0);

switch mode
    case {'fixed-ring', 'rotating-ring'}
        for groupCursor = 1:numel(groups)
            receiverGroup = groups(groupCursor);
            senderGroup = groups(1 + mod(groupCursor, numel(groups)));
            receivers = reshape(find(groupIds == receiverGroup), 1, []);
            senders = reshape(find(groupIds == senderGroup), 1, []);
            roleCursor = 1 + mod(phase - 1, ...
                min(numel(receivers), numel(senders)));
            if strcmp(mode, 'rotating-ring')
                roleCursor = 1 + mod( ...
                    context.currentTime - anchorTime + phase - 1, ...
                    min(numel(receivers), numel(senders)));
            end
            receiverIdx = receivers(roleCursor);
            senderIdx = senders(roleCursor);
            if ~physical(receiverIdx, senderIdx)
                continue;
            end
            proposals(end + 1) = struct( ... %#ok<AGROW>
                'receiver', receiverIdx, ...
                'sender', senderIdx, ...
                'formation', receiverGroup, ...
                'priority', 1);
        end
    case 'link-aware'
        reliability = currentReliability( ...
            context, physical, context.currentTime);
        for groupCursor = 1:numel(groups)
            receiverGroup = groups(groupCursor);
            receivers = find(groupIds == receiverGroup);
            crossMask = false(nodeCount);
            crossMask(receivers, :) = true;
            crossMask(:, groupIds == receiverGroup) = false;
            crossMask = crossMask & physical;
            indices = find(crossMask);
            if isempty(indices)
                continue;
            end
            [~, bestCursor] = max(reliability(indices));
            [receiverIdx, senderIdx] = ind2sub( ...
                [nodeCount, nodeCount], indices(bestCursor));
            proposals(end + 1) = struct( ... %#ok<AGROW>
                'receiver', receiverIdx, ...
                'sender', senderIdx, ...
                'formation', receiverGroup, ...
                'priority', reliability(receiverIdx, senderIdx));
        end
    otherwise
        error('Unknown registered gateway mode: %s', mode);
end

overrideMask = false(1, nodeCount);
if ~isempty(proposals)
    [~, order] = sort([proposals.priority], 'descend');
    formationLoad = zeros(1, max(groupIds));
    sourceLoad = zeros(1, nodeCount);
    for cursor = reshape(order, 1, [])
        proposal = proposals(cursor);
        if formationLoad(proposal.formation) >= ...
                maxCrossPerFormation || ...
                sourceLoad(proposal.sender) >= maxCrossSourceLoad
            continue;
        end
        selectedSources(proposal.receiver) = proposal.sender;
        overrideMask(proposal.receiver) = true;
        formationLoad(proposal.formation) = ...
            formationLoad(proposal.formation) + 1;
        sourceLoad(proposal.sender) = sourceLoad(proposal.sender) + 1;
    end
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
if nodeCount > messageBudget
    error([ ...
        'Registered gateway routing needs one message per receiver; ', ...
        'the directed-message budget is too small.']);
end
adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senderIdx = selectedSources(receiverIdx);
    if ~physical(receiverIdx, senderIdx)
        error([ ...
            'Registered gateway selected a non-physical route at ', ...
            't=%d: sender %d -> receiver %d.'], ...
            context.currentTime, senderIdx, receiverIdx);
    end
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

details = struct();
details.mode = ['registered-gateway-', mode];
details.objective = NaN;
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = numel(proposals);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.baselineSourcesByReceiver = ...
    baseDetails.selectedSourcesByReceiver;
details.overrideMask = overrideMask;
details.overrideFraction = mean(overrideMask);
details.supportedCandidateFraction = mean(overrideMask);
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
details.posteriorUsed = false;
details.truthUsed = false;
details.currentLinkReliabilityUsed = strcmp(mode, 'link-aware');
details.currentPhysicalActionSetUsed = true;
end

function reliability = currentReliability( ...
    context, physical, currentTime)
reliability = double(physical);
if isfield(context, 'commConfig') && ...
        isfield(context.commConfig, 'pDropByEdge') && ...
        ~isempty(context.commConfig.pDropByEdge)
    pDrop = context.commConfig.pDropByEdge;
    if ndims(pDrop) >= 3
        timeIdx = min(max(1, round(currentTime)), size(pDrop, 3));
        pDrop = pDrop(:, :, timeIdx);
    end
    if ~isequal(size(pDrop), size(physical))
        error('pDropByEdge must be S-by-S or S-by-S-by-T.');
    end
    % Communication storage is pDrop(sender, receiver); policy matrices
    % are adjacency(receiver, sender).
    reliability = 1 - pDrop';
end
reliability = min(max(reliability, 0), 1);
reliability(~physical) = NaN;
end

function groupIds = resolveGroupIds(model, nodeCount)
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isfield(model.dynamicTopologyScenario, 'config') || ...
        ~isfield(model.dynamicTopologyScenario.config, 'sensorGroupIds')
    error('Registered gateway routing requires sensorGroupIds metadata.');
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
