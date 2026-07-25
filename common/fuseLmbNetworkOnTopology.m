function [fusedBySensor, details] = fuseLmbNetworkOnTopology( ...
    localPosteriors, adjacency, model, commConfig, currentTime, ...
    triggerConfig, options)
% FUSELMBNETWORKONTOPOLOGY Deterministic one-round network LMB fusion.
%
% The helper mirrors the receiver-side Metropolis weighting used by the
% distributed filter. Optional expected-delivery weighting multiplies each
% directed neighbor weight by its current link reliability and renormalizes.
% It never samples future link uniforms.

if nargin < 7 || isempty(options)
    options = struct();
end
expectedDeliveryWeighting = getField( ...
    options, 'expectedDeliveryWeighting', true);
adjacency = logical(adjacency);
adjacency = adjacency | adjacency';
nodeCount = size(adjacency, 1);
adjacency(1:nodeCount+1:end) = false;
if numel(localPosteriors) ~= nodeCount
    error('localPosteriors must contain one entry per topology node.');
end

weights = metropolisWeights(adjacency);
fusedBySensor = cell(1, nodeCount);
effectiveWeights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    neighbors = find(adjacency(receiverIdx, :));
    sources = [receiverIdx, neighbors];
    localWeights = weights(receiverIdx, sources);
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
    fusionInputs = localPosteriors(sources);
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones(1, numel(sources) - 1)]);
    fusedBySensor{receiverIdx} = fuseLmbPosteriorsByLabel( ...
        fusionInputs, localWeights, model, localWeights, ...
        fusionDetails, triggerConfig);
end
details = struct( ...
    'metropolisWeights', weights, ...
    'effectiveWeights', effectiveWeights, ...
    'expectedDeliveryWeighting', expectedDeliveryWeighting);
end

function weights = metropolisWeights(adjacency)
nodeCount = size(adjacency, 1);
degreesIncludingSelf = sum(adjacency, 2)' + 1;
weights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    neighbors = find(adjacency(receiverIdx, :));
    for senderIdx = neighbors
        weights(receiverIdx, senderIdx) = 1 / (1 + max( ...
            degreesIncludingSelf(receiverIdx), ...
            degreesIncludingSelf(senderIdx)));
    end
    weights(receiverIdx, receiverIdx) = ...
        1 - sum(weights(receiverIdx, :));
    weights(receiverIdx, :) = weights(receiverIdx, :) / ...
        max(sum(weights(receiverIdx, :)), eps);
end
end

function probability = edgeDrop(config, senderIdx, receiverIdx, currentTime)
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
