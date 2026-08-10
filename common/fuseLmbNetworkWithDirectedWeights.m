function [fusedBySensor, details] = ...
    fuseLmbNetworkWithDirectedWeights( ...
        localPosteriors, adjacency, fusionWeights, model, ...
        commConfig, currentTime, triggerConfig, options)
% FUSELMBNETWORKWITHDIRECTEDWEIGHTS One deterministic virtual KLA round.
%
% Unlike fuseLmbNetworkOnTopology, this helper preserves the registered
% directed row weights.  Optional current-link reliability scaling is a
% deterministic expected-weight approximation; no delivery draw is sampled.

if nargin < 8 || isempty(options)
    options = struct();
end
if nargin < 7 || isempty(triggerConfig)
    triggerConfig = buildMixtureAwareKlaReferenceConfig();
end
expectedDeliveryWeighting = getField( ...
    options, 'expectedDeliveryWeighting', true);
nodeCount = numel(localPosteriors);
adjacency = logical(adjacency);
if ~isequal(size(adjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(fusionWeights), [nodeCount, nodeCount]) || ...
        any(~isfinite(fusionWeights(:))) || ...
        any(fusionWeights(:) < 0) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12)
    error('DirectedWeightedFusion:InvalidInput', ...
        'Directed topology or fusion weights are invalid.');
end
adjacency(1:nodeCount+1:end) = false;
allowed = adjacency | logical(eye(nodeCount));
if any(fusionWeights(:) > 0 & ~allowed(:)) || ...
        any(diag(fusionWeights) <= 0)
    error('DirectedWeightedFusion:InvalidSupport', ...
        'Positive weights do not match the directed topology.');
end

fusedBySensor = cell(1, nodeCount);
effectiveWeights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    sources = [receiverIdx, reshape(find( ...
        adjacency(receiverIdx, :)), 1, [])];
    localWeights = fusionWeights(receiverIdx, sources);
    if expectedDeliveryWeighting
        for sourceCursor = 2:numel(sources)
            senderIdx = sources(sourceCursor);
            localWeights(sourceCursor) = ...
                localWeights(sourceCursor) * (1 - edgeDrop( ...
                    commConfig, senderIdx, receiverIdx, currentTime));
        end
    end
    positive = localWeights > eps;
    sources = sources(positive);
    localWeights = localWeights(positive);
    localWeights = localWeights / sum(localWeights);
    effectiveWeights(receiverIdx, sources) = localWeights;
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones(1, numel(sources) - 1)]);
    fusedBySensor{receiverIdx} = fuseLmbPosteriorsByLabel( ...
        localPosteriors(sources), localWeights, model, ...
        localWeights, fusionDetails, triggerConfig);
end
details = struct( ...
    'contractVersion', 'directed-weighted-network-fusion-v1', ...
    'effectiveWeights', effectiveWeights, ...
    'expectedDeliveryWeighting', expectedDeliveryWeighting, ...
    'deliveryDrawSampled', false, ...
    'truthUsed', false, ...
    'futureLinkUsed', false);
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

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
