function [networkRisk, details] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, adjacency, fusionWeights, options)
% COMPUTEEXPECTEDLMBNETWORKDISAGREEMENTFORDIRECTEDROUTING
% Exact one-round posterior-summary disagreement under link uncertainty.
%
% For each receiver, this function enumerates every independent delivery
% outcome on its selected incoming edges, executes the same heavy-payload
% LMB fusion and missing-message weight normalization as the controlled
% runtime, and caches the resulting label-wise posterior moments. It then
% integrates every receiver-pair disagreement over the two independent
% receiver outcome distributions.
%
% The result is exact for the registered one-round fusion-summary model.
% It is not a recursive tracking guarantee and it requires centralized
% access to the current posteriors used by the topology controller.

if nargin < 4 || isempty(options)
    options = struct();
end
aggregationMode = lower(strrep(char(getField( ...
    options, 'aggregationMode', 'mean')), '_', '-'));
tailFraction = getField(options, 'tailFraction', 0.25);
if ~ismember(aggregationMode, {'mean', 'tail'}) || ...
        ~isscalar(tailFraction) || ~isfinite(tailFraction) || ...
        tailFraction <= 0 || tailFraction > 1
    error('Expected-disagreement aggregation options are invalid.');
end
if ~isfield(context, 'localPosteriorBySensor') || ...
        ~iscell(context.localPosteriorBySensor) || ...
        ~isfield(context, 'model') || ...
        ~isfield(context, 'commConfig') || ...
        ~isfield(context, 'currentTime')
    error('Expected-disagreement context is incomplete.');
end
nodeCount = numel(context.localPosteriorBySensor);
adjacency = logical(adjacency);
if nodeCount < 2 || ...
        ~isequal(size(adjacency), [nodeCount, nodeCount]) || ...
        ~isequal(size(fusionWeights), [nodeCount, nodeCount]) || ...
        any(~isfinite(fusionWeights(:))) || ...
        any(fusionWeights(:) < 0)
    error('Expected-disagreement routing inputs are invalid.');
end
adjacency(1:nodeCount+1:end) = false;
allowedWeights = adjacency | logical(eye(nodeCount));
if any(fusionWeights(:) > 0 & ~allowedWeights(:)) || ...
        any(abs(sum(fusionWeights, 2) - 1) > 1e-12) || ...
        any(diag(fusionWeights) <= 0)
    error('Expected-disagreement fusion weights violate the route.');
end

triggerConfig = getField(context, ...
    'triggerConfig', buildMixtureAwareKlaReferenceConfig());
validateRuntimeContract(triggerConfig);
maximumIncomingCount = round(getField( ...
    options, 'maximumIncomingCount', 4));
if ~isscalar(maximumIncomingCount) || ...
        ~isfinite(maximumIncomingCount) || ...
        maximumIncomingCount < 1
    error('maximumIncomingCount must be a positive integer.');
end

receiverDistributions = cell(1, nodeCount);
receiverOutcomeCounts = zeros(1, nodeCount);
for receiverIdx = 1:nodeCount
    senders = reshape(find(adjacency(receiverIdx, :)), 1, []);
    if numel(senders) > maximumIncomingCount
        error(['Expected-disagreement route exceeds the registered ', ...
            'incoming-edge enumeration limit.']);
    end
    receiverDistributions{receiverIdx} = ...
        enumerateReceiverOutcomes( ...
            context, receiverIdx, senders, ...
            fusionWeights(receiverIdx, :), triggerConfig);
    receiverOutcomeCounts(receiverIdx) = numel( ...
        receiverDistributions{receiverIdx}.probability);
end

expectedPairRisk = zeros(nodeCount);
for leftIdx = 1:nodeCount-1
    left = receiverDistributions{leftIdx};
    for rightIdx = leftIdx+1:nodeCount
        right = receiverDistributions{rightIdx};
        value = 0;
        for leftOutcome = 1:numel(left.probability)
            for rightOutcome = 1:numel(right.probability)
                pair = computeLmbPosteriorSummaryDisagreement( ...
                    left.summary{leftOutcome}, ...
                    right.summary{rightOutcome});
                value = value + ...
                    left.probability(leftOutcome) * ...
                    right.probability(rightOutcome) * ...
                    pair.combined;
            end
        end
        expectedPairRisk(leftIdx, rightIdx) = value;
        expectedPairRisk(rightIdx, leftIdx) = value;
    end
end
networkRisk = aggregatePairMatrix( ...
    expectedPairRisk, aggregationMode, tailFraction);
if ~isfinite(networkRisk)
    error('Expected one-step network disagreement is non-finite.');
end

details = struct( ...
    'contractVersion', ...
        'directed-lmb-one-step-expected-disagreement-v1', ...
    'aggregationMode', aggregationMode, ...
    'tailFraction', tailFraction, ...
    'networkRisk', networkRisk, ...
    'expectedPairRisk', expectedPairRisk, ...
    'receiverOutcomeCounts', receiverOutcomeCounts, ...
    'receiverDistributions', {receiverDistributions}, ...
    'maximumIncomingCount', maximumIncomingCount, ...
    'deliveryModel', ...
        'independent-bernoulli-current-link-reliability', ...
    'fusionModel', ...
        'runtime-heavy-payload-with-missing-weight-normalization', ...
    'singleRoundExact', true, ...
    'recursiveSafetyClaimed', false, ...
    'centralizedCurrentPosteriorRequired', true, ...
    'posteriorUsed', true, ...
    'currentLinkReliabilityUsed', true, ...
    'truthUsed', false, ...
    'groundTruthUsed', false, ...
    'futureMeasurementUsed', false, ...
    'futureOutcomeUsed', false);
end

function distribution = enumerateReceiverOutcomes( ...
        context, receiverIdx, senders, routeWeights, triggerConfig)
senderCount = numel(senders);
reliabilities = zeros(1, senderCount);
for senderCursor = 1:senderCount
    reliabilities(senderCursor) = edgeReliability( ...
        context.commConfig, senders(senderCursor), ...
        receiverIdx, context.currentTime);
end
outcomeCount = 2^senderCount;
probability = zeros(1, outcomeCount);
summary = cell(1, outcomeCount);
deliveryMask = false(outcomeCount, senderCount);
effectiveWeights = cell(1, outcomeCount);
for outcomeIdx = 0:(outcomeCount - 1)
    delivered = logical(bitget(outcomeIdx, 1:senderCount));
    deliveryMask(outcomeIdx + 1, :) = delivered;
    probability(outcomeIdx + 1) = ...
        prod(reliabilities(delivered)) * ...
        prod(1 - reliabilities(~delivered));
    deliveredSenders = senders(delivered);
    weights = [ ...
        routeWeights(receiverIdx), ...
        routeWeights(deliveredSenders)];
    if strcmpi(getField(triggerConfig, ...
            'missingNeighborWeightMode', 'renormalize'), 'self')
        weights(1) = weights(1) + ...
            sum(routeWeights(senders(~delivered)));
    end
    weights = weights / max(sum(weights), eps);
    inputs = cell(1, 1 + numel(deliveredSenders));
    inputs{1} = context.localPosteriorBySensor{receiverIdx};
    for senderCursor = 1:numel(deliveredSenders)
        senderIdx = deliveredSenders(senderCursor);
        inputs{senderCursor + 1} = selectActiveObjects( ...
            context.localPosteriorBySensor{senderIdx}, ...
            triggerConfig.payloadExistenceThreshold);
    end
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones( ...
            1, numel(deliveredSenders))]);
    fused = fuseLmbPosteriorsByLabel( ...
        inputs, weights, context.model, weights, ...
        fusionDetails, triggerConfig);
    fused = selectActiveObjects( ...
        fused, context.model.existenceThreshold);
    summary{outcomeIdx + 1} = ...
        summarizeLmbPosteriorForDisagreement( ...
            fused, context.model);
    effectiveWeights{outcomeIdx + 1} = weights;
end
if abs(sum(probability) - 1) > 1e-12
    error('Receiver delivery-outcome probabilities do not sum to one.');
end
distribution = struct( ...
    'receiverIndex', receiverIdx, ...
    'senderIndices', senders, ...
    'senderReliabilities', reliabilities, ...
    'deliveryMask', deliveryMask, ...
    'probability', probability, ...
    'effectiveWeights', {effectiveWeights}, ...
    'summary', {summary});
end

function selected = selectActiveObjects(objects, threshold)
if isempty(objects)
    selected = objects;
    return;
end
active = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
selected = objects(active);
end

function reliability = edgeReliability( ...
        config, senderIdx, receiverIdx, currentTime)
if sensorOutageAtCurrentTime(config, senderIdx, currentTime)
    reliability = 0;
    return;
end
if getField(config, 'forceDelivery', false)
    reliability = 1;
    return;
end
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(max(1, currentTime), ...
            size(config.pDropByEdge, 3));
        dropProbability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        dropProbability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    dropProbability = config.pDropBySensor(senderIdx);
else
    dropProbability = getField(config, 'pDrop', 0);
end
reliability = 1 - min(max(dropProbability, 0), 1);
end

function active = sensorOutageAtCurrentTime( ...
        config, sensorIdx, currentTime)
active = false;
schedule = getField(config, 'outageSchedule', []);
for windowIdx = 1:numel(schedule)
    if schedule(windowIdx).sensor == sensorIdx && ...
            currentTime >= schedule(windowIdx).start && ...
            currentTime <= schedule(windowIdx).end
        active = true;
        return;
    end
end
end

function risk = aggregatePairMatrix( ...
        pairRisk, aggregationMode, tailFraction)
values = pairRisk(triu(true(size(pairRisk)), 1));
if isempty(values)
    risk = 0;
elseif strcmp(aggregationMode, 'mean')
    risk = mean(values);
else
    values = sort(values, 'descend');
    tailCount = max(1, ceil(tailFraction * numel(values)));
    risk = mean(values(1:tailCount));
end
end

function validateRuntimeContract(config)
if ~strcmpi(getField(config, 'eventPolicy', ''), 'alwaysHeavy')
    unsupported('eventPolicy must be alwaysHeavy');
end
if getField(config, 'linkGateEnabled', true)
    unsupported('linkGateEnabled must be false');
end
if getField(config, 'useStaleNeighborCache', false)
    unsupported('stale-cache fusion is not modeled');
end
if getField(config, 'mixedPayloadEnabled', false)
    unsupported('mixed payloads are not modeled');
end
if getField(config, 'modeAwareFusionWeights', false)
    unsupported('mode-aware fusion weights are not modeled');
end
if abs(getField(config, ...
        'nonStaticFusionWeightFactor', 1) - 1) > 1e-12
    unsupported('non-static fusion reweighting is not modeled');
end
if isfinite(getField(config, 'maxSelfFusionWeight', inf))
    unsupported('self-weight capping is not modeled');
end
if isfinite(getField(config, 'maxNonStaticFusionWeight', inf))
    unsupported('non-static weight capping is not modeled');
end
missingMode = lower(char(getField(config, ...
    'missingNeighborWeightMode', 'renormalize')));
if ~ismember(missingMode, {'renormalize', 'self'})
    unsupported('missingNeighborWeightMode must be renormalize or self');
end
end

function unsupported(message)
error('ExpectedDirectedDisagreement:UnsupportedRuntime', ...
    'Unsupported expected-disagreement runtime: %s.', message);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
