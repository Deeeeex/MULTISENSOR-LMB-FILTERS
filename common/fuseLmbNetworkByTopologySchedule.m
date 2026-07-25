function [posteriors, details] = ...
    fuseLmbNetworkByTopologySchedule( ...
        initialPosteriors, adjacencySchedule, model, ...
        triggerConfig, options)
% FUSELMBNETWORKBYTOPOLOGYSCHEDULE Synchronous multi-round LMB fusion.
%
% adjacencySchedule(receiver, sender, round) indicates that the receiver
% consumes the sender's posterior in that round. A symmetric slice is an
% ordinary bidirectional topology; asymmetric slices support later
% time-expanded routing experiments. Every round consumes the previous
% round's fused posteriors, rather than repeatedly reading the initial
% local posterior.
%
% This helper is an offline architecture diagnostic. Communication delivery
% can be represented by expected-reliability weight scaling, but the helper
% does not sample packet loss or mutate the online communication caches.

if nargin < 5 || isempty(options)
    options = struct();
end
if nargin < 4 || isempty(triggerConfig)
    triggerConfig = buildMixtureAwareKlaReferenceConfig();
end
if ~iscell(initialPosteriors)
    error('initialPosteriors must be a cell array with one entry per node.');
end
nodeCount = numel(initialPosteriors);
if ndims(adjacencySchedule) == 2
    adjacencySchedule = reshape( ...
        adjacencySchedule, nodeCount, nodeCount, 1);
end
if size(adjacencySchedule, 1) ~= nodeCount || ...
        size(adjacencySchedule, 2) ~= nodeCount
    error('adjacencySchedule must be S-by-S-by-R.');
end
if any(~isfinite(adjacencySchedule(:)))
    error('adjacencySchedule contains a non-finite value.');
end
adjacencySchedule = logical(adjacencySchedule);
for roundIdx = 1:size(adjacencySchedule, 3)
    roundAdjacency = adjacencySchedule(:, :, roundIdx);
    roundAdjacency(1:nodeCount+1:end) = false;
    adjacencySchedule(:, :, roundIdx) = roundAdjacency;
end

commConfig = getField(options, 'commConfig', struct());
currentTime = max(1, round(getField(options, 'currentTime', 1)));
expectedDeliveryWeighting = getField( ...
    options, 'expectedDeliveryWeighting', true);

posteriors = reshape(initialPosteriors, 1, []);
roundCount = size(adjacencySchedule, 3);
roundWeights = cell(1, roundCount);
attemptedBytesByRound = zeros(1, roundCount);
nodePayloadBytesByRound = cell(1, roundCount);
for roundIdx = 1:roundCount
    adjacency = adjacencySchedule(:, :, roundIdx);
    nodePayloadBytes = zeros(1, nodeCount);
    for senderIdx = 1:nodeCount
        payloadStats = estimateLmbPayloadSize( ...
            posteriors{senderIdx}, model, 2);
        nodePayloadBytes(senderIdx) = payloadStats.estimatedBytes;
    end
    nodePayloadBytesByRound{roundIdx} = nodePayloadBytes;
    attemptedBytesByRound(roundIdx) = sum( ...
        sum(adjacency, 1) .* nodePayloadBytes);
    topologyWeights = metropolisWeights(adjacency);
    nextPosteriors = cell(1, nodeCount);
    effectiveWeights = zeros(nodeCount);
    for receiverIdx = 1:nodeCount
        sources = [receiverIdx, find(adjacency(receiverIdx, :))];
        localWeights = topologyWeights(receiverIdx, sources);
        if expectedDeliveryWeighting
            for sourceCursor = 2:numel(sources)
                senderIdx = sources(sourceCursor);
                reliability = 1 - edgeDrop( ...
                    commConfig, senderIdx, receiverIdx, currentTime);
                localWeights(sourceCursor) = ...
                    localWeights(sourceCursor) * reliability;
            end
        end
        positive = localWeights > eps;
        sources = sources(positive);
        localWeights = localWeights(positive);
        localWeights = localWeights / max(sum(localWeights), eps);
        effectiveWeights(receiverIdx, sources) = localWeights;
        fusionDetails = struct( ...
            'eventType', [0, 2 * ones(1, numel(sources) - 1)]);
        nextPosteriors{receiverIdx} = fuseLmbPosteriorsByLabel( ...
            posteriors(sources), localWeights, model, localWeights, ...
            fusionDetails, triggerConfig);
    end
    posteriors = nextPosteriors;
    roundWeights{roundIdx} = effectiveWeights;
end

details = struct();
details.roundCount = roundCount;
details.directedTransmissionCountByRound = reshape(sum(sum( ...
    adjacencySchedule, 1), 2), 1, []);
details.totalDirectedTransmissionCount = sum( ...
    details.directedTransmissionCountByRound);
details.attemptedBytesByRound = attemptedBytesByRound;
details.totalAttemptedBytes = sum(attemptedBytesByRound);
details.nodePayloadBytesByRound = nodePayloadBytesByRound;
details.effectiveWeightsByRound = roundWeights;
end

function weights = metropolisWeights(adjacency)
nodeCount = size(adjacency, 1);
degreesIncludingSelf = sum(adjacency, 2)' + 1;
weights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    senders = find(adjacency(receiverIdx, :));
    for senderIdx = senders
        weights(receiverIdx, senderIdx) = 1 / (1 + max( ...
            degreesIncludingSelf(receiverIdx), ...
            degreesIncludingSelf(senderIdx)));
    end
    weights(receiverIdx, receiverIdx) = ...
        max(0, 1 - sum(weights(receiverIdx, :)));
    weights(receiverIdx, :) = weights(receiverIdx, :) / ...
        max(sum(weights(receiverIdx, :)), eps);
end
end

function probability = edgeDrop( ...
    config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
