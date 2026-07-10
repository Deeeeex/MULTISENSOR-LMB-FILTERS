function [stateEstimatesBySensor, diagnostics] = ...
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, triggerConfig)
% RUNEVENTTRIGGEREDDISTRIBUTEDLMBFILTER Edge-triggered GA-LMB prototype.
%
% Each node performs its own LMB prediction and local measurement update.
% Directed edges then independently choose no message, a moment-matched LMB
% summary, or a full GM-LMB posterior. By default receivers fuse only messages
% delivered at the current step. Optional bounded stale-cache fusion predicts
% the latest received neighbor posterior forward and applies an age-decayed
% topology weight when the current edge is silent.

if nargin < 3
    sensorTrajectories = [];
end
if nargin < 4 || isempty(neighborMap)
    neighborMap = buildRangeNeighborMap(model, sensorTrajectories);
end
if nargin < 5 || isempty(commConfig)
    commConfig = struct();
end
if nargin < 6 || isempty(triggerConfig)
    triggerConfig = struct();
end

numberOfSensors = model.numberOfSensors;
simulationLength = size(measurements, 2);
if ~isempty(sensorTrajectories)
    model.sensorTrajectories = sensorTrajectories;
end

commConfig = resolveCommunicationConfig( ...
    commConfig, numberOfSensors, simulationLength);
triggerConfig = resolveTriggerConfig(triggerConfig, model);
topologyWeights = computeMetropolisWeightMatrix(neighborMap);
baseDirectedEdgeMask = buildDirectedEdgeMask(neighborMap, numberOfSensors);

objectsBySensor = repmat({model.object}, 1, numberOfSensors);
stateEstimatesBySensor = cell(1, numberOfSensors);
for sensorIdx = 1:numberOfSensors
    stateEstimatesBySensor{sensorIdx} = initializeStateEstimate( ...
        simulationLength, model.object);
end

referenceCache = cell(numberOfSensors);
receivedCache = cell(numberOfSensors);
currentMessages = cell(numberOfSensors);
currentMessagePresent = false(numberOfSensors);
lastSuccessfulTime = zeros(numberOfSensors);
labelRefreshCache = cell(numberOfSensors);
attemptHistory = cell(numberOfSensors);
wasOutage = false(numberOfSensors);
previousDirectedEdgeMask = false(numberOfSensors);

diagnostics = initializeDiagnostics( ...
    numberOfSensors, simulationLength, baseDirectedEdgeMask, ...
    commConfig, triggerConfig);

for currentTime = 1:simulationLength
    predictedBySensor = cell(1, numberOfSensors);
    localPosteriorBySensor = cell(1, numberOfSensors);
    updateDiagnostics = cell(1, numberOfSensors);

    for sensorIdx = 1:numberOfSensors
        predictedBySensor{sensorIdx} = lmbPredictionStep( ...
            objectsBySensor{sensorIdx}, model, currentTime);
        isScheduled = resolveSampleMask( ...
            commConfig, sensorIdx, currentTime);
        [localPosteriorBySensor{sensorIdx}, updateDiagnostics{sensorIdx}] = ...
            updateLmbWithSensorMeasurement( ...
                predictedBySensor{sensorIdx}, ...
                measurements{sensorIdx, currentTime}, ...
                model, sensorIdx, currentTime, isScheduled);
        diagnostics.localInnovation(sensorIdx, currentTime) = ...
            updateDiagnostics{sensorIdx}.innovationNovelty;
        diagnostics.localAssociationConfidence(sensorIdx, currentTime) = ...
            updateDiagnostics{sensorIdx}.associationConfidence;
    end

    [currentNeighborMap, currentTopologyWeights, topologyDetails] = ...
        resolveActiveTopology( ...
            neighborMap, topologyWeights, localPosteriorBySensor, ...
            model, sensorTrajectories, commConfig, triggerConfig, ...
            currentTime);
    currentDirectedEdgeMask = buildDirectedEdgeMask( ...
        currentNeighborMap, numberOfSensors);
    diagnostics.topologyActiveEdge(:, :, currentTime) = ...
        currentDirectedEdgeMask;
    diagnostics.topologyAlgebraicConnectivity(currentTime) = ...
        topologyDetails.algebraicConnectivity;
    diagnostics.topologyUndirectedEdgeCount(currentTime) = ...
        topologyDetails.undirectedEdgeCount;
    newEdgeMask = currentDirectedEdgeMask & ~previousDirectedEdgeMask;
    diagnostics.newEdgeActivated(:, :, currentTime) = newEdgeMask;

    currentMessages(:) = {[]};
    currentMessagePresent(:) = false;
    currentMessageTypes = zeros(numberOfSensors);
    for receiverIdx = 1:numberOfSensors
        senders = reshape(currentNeighborMap{receiverIdx}, 1, []);
        senders = senders(senders ~= receiverIdx);
        for senderIdx = senders
            referenceAge = resolveReferenceAge( ...
                lastSuccessfulTime(senderIdx, receiverIdx), currentTime);
            [utility, utilityDetails] = computeLmbEventUtility( ...
                localPosteriorBySensor{senderIdx}, ...
                referenceCache{senderIdx, receiverIdx}, ...
                receivedCache{senderIdx, receiverIdx}, ...
                predictedBySensor{senderIdx}, ...
                updateDiagnostics{senderIdx}, model, triggerConfig);

            isOutage = isSensorOutage( ...
                commConfig.outageSchedule, senderIdx, currentTime);
            recoveredLink = wasOutage(senderIdx, receiverIdx) && ~isOutage;
            edgeTriggerConfig = triggerConfig;
            edgeTriggerConfig.forceStaleHeavy = recoveredLink;
            [rawEventType, classifyDetails] = ...
                classifyLmbCommunicationEvent( ...
                    utility, utilityDetails, edgeTriggerConfig, referenceAge);
            forceHandshake = shouldForceNewEdgeHandshake( ...
                newEdgeMask(senderIdx, receiverIdx), ...
                localPosteriorBySensor{senderIdx}, triggerConfig);
            if forceHandshake && ~strcmpi(triggerConfig.eventPolicy, 'none')
                rawEventType = 2;
                classifyDetails.forceHeavy = true;
            end
            [heartbeatEventType, heartbeatDetails] = ...
                computeLabelHeartbeatEvent( ...
                    localPosteriorBySensor{senderIdx}, ...
                    labelRefreshCache{senderIdx, receiverIdx}, ...
                    currentTime, triggerConfig);
            if heartbeatEventType > rawEventType && ...
                    ~strcmpi(triggerConfig.eventPolicy, 'none')
                rawEventType = heartbeatEventType;
            end

            nominalReliability = 1 - resolveEdgeDropProbability( ...
                commConfig, senderIdx, receiverIdx);
            recentSuccessRate = computeRecentSuccessRate( ...
                attemptHistory{senderIdx, receiverIdx});
            [eventType, gateDetails] = applyLmbEventLinkGate( ...
                rawEventType, nominalReliability, recentSuccessRate, ...
                isOutage, triggerConfig);
            if forceHandshake && ...
                    triggerConfig.newEdgeHandshakeBypassLinkGate && ...
                    ~isOutage && rawEventType > 0
                eventType = 2;
                gateDetails.wasDowngraded = false;
                gateDetails.reason = 'new-edge-handshake';
            end

            diagnostics.utility(senderIdx, receiverIdx, currentTime) = utility;
            diagnostics.informationGain(senderIdx, receiverIdx, currentTime) = ...
                utilityDetails.maxInformationGain;
            diagnostics.rawEventType(senderIdx, receiverIdx, currentTime) = ...
                rawEventType;
            diagnostics.eventType(senderIdx, receiverIdx, currentTime) = ...
                eventType;
            diagnostics.linkQuality(senderIdx, receiverIdx, currentTime) = ...
                gateDetails.linkQuality;
            diagnostics.referenceAge(senderIdx, receiverIdx, currentTime) = ...
                referenceAge;
            diagnostics.forcedHeavy(senderIdx, receiverIdx, currentTime) = ...
                classifyDetails.forceHeavy;
            diagnostics.labelHeartbeat(senderIdx, receiverIdx, currentTime) = ...
                heartbeatEventType > 0;
            diagnostics.labelHeartbeatAge(senderIdx, receiverIdx, currentTime) = ...
                heartbeatDetails.maxAge;
            diagnostics.downgraded(senderIdx, receiverIdx, currentTime) = ...
                gateDetails.wasDowngraded;
            diagnostics.newEdgeHandshake(senderIdx, receiverIdx, ...
                currentTime) = forceHandshake;
            diagnostics.newEdgeNoHandshake(senderIdx, receiverIdx, ...
                currentTime) = newEdgeMask(senderIdx, receiverIdx) && ...
                eventType < 2;

            if eventType > 0
                diagnostics.attempted(senderIdx, receiverIdx, currentTime) = true;
                payloadObjects = buildPayload( ...
                    localPosteriorBySensor{senderIdx}, model, ...
                    eventType, triggerConfig, utilityDetails, ...
                    updateDiagnostics{senderIdx});
                wireMetadata = struct( ...
                    'eventType', eventType, ...
                    'sender', senderIdx, ...
                    'receiver', receiverIdx, ...
                    'timeIndex', currentTime);
                [encodedPayload, wireStats] = encodeLmbWireMessage( ...
                    payloadObjects, wireMetadata, model);
                encodedByteCount = numel(encodedPayload);
                if wireStats.encodedBytes ~= encodedByteCount
                    error(['runEventTriggeredDistributedLmbFilter:' ...
                        'WireLengthMismatch'], ...
                        'Codec statistics do not match the encoded bytes.');
                end
                legacyScalarCount = computeLegacyPayloadScalarCount( ...
                    wireStats, model, updateDiagnostics{senderIdx});
                diagnostics.attemptedPayloadBytes( ...
                    senderIdx, receiverIdx, currentTime) = encodedByteCount;
                delivered = simulateDelivery( ...
                    commConfig, senderIdx, receiverIdx, currentTime, isOutage);
                diagnostics.delivered(senderIdx, receiverIdx, currentTime) = ...
                    delivered;
                attemptHistory{senderIdx, receiverIdx} = appendAttempt( ...
                    attemptHistory{senderIdx, receiverIdx}, delivered, ...
                    triggerConfig.recentWindow);

                if delivered
                    [decodedPayload, decodedMetadata] = ...
                        decodeLmbWireMessage(encodedPayload, model);
                    validateDecodedWireMetadata( ...
                        decodedMetadata, senderIdx, receiverIdx, ...
                        currentTime, eventType, numel(decodedPayload));
                    diagnostics.wireMetadataValidated( ...
                        senderIdx, receiverIdx, currentTime) = true;
                    currentMessages{receiverIdx, senderIdx} = decodedPayload;
                    currentMessagePresent(receiverIdx, senderIdx) = true;
                    currentMessageTypes(receiverIdx, senderIdx) = eventType;
                    diagnostics.deliveredEventType( ...
                        senderIdx, receiverIdx, currentTime) = eventType;
                    referenceCache{senderIdx, receiverIdx} = ...
                        selectActiveObjects( ...
                            localPosteriorBySensor{senderIdx}, ...
                            triggerConfig.payloadExistenceThreshold);
                    receivedCache{receiverIdx, senderIdx} = decodedPayload;
                    labelRefreshCache{senderIdx, receiverIdx} = ...
                        updateLabelRefreshCache( ...
                            labelRefreshCache{senderIdx, receiverIdx}, ...
                            localPosteriorBySensor{senderIdx}, ...
                            currentTime, triggerConfig);
                    lastSuccessfulTime(senderIdx, receiverIdx) = currentTime;
                    diagnostics.payloadScalars( ...
                        senderIdx, receiverIdx, currentTime) = ...
                        legacyScalarCount;
                    diagnostics.deliveredPayloadBytes( ...
                        senderIdx, receiverIdx, currentTime) = ...
                        encodedByteCount;
                    diagnostics.payloadBytes( ...
                        senderIdx, receiverIdx, currentTime) = ...
                        encodedByteCount;
                end
            end
            wasOutage(senderIdx, receiverIdx) = isOutage;
        end
    end
    diagnostics = recordCommunicationGraphDiagnostics( ...
        diagnostics, localPosteriorBySensor, labelRefreshCache, ...
        currentNeighborMap, triggerConfig, currentTime);

    for receiverIdx = 1:numberOfSensors
        [fusionInputs, topologyFusionWeights, fusionDetails] = ...
            collectCurrentFusionInputs( ...
            receiverIdx, localPosteriorBySensor{receiverIdx}, ...
            currentMessages, currentMessagePresent, ...
            currentMessageTypes, receivedCache, ...
            lastSuccessfulTime, ...
            currentNeighborMap, currentTopologyWeights, model, ...
            triggerConfig, currentTime);
        diagnostics = recordStaleFusionDiagnostics( ...
            diagnostics, fusionDetails, receiverIdx, currentTime);
        diagnostics = recordFusionWeightDiagnostics( ...
            diagnostics, fusionDetails, receiverIdx, currentTime);
        [spatialFusionWeights, existenceFusionWeights] = ...
            resolveFusionWeights( ...
                fusionInputs, topologyFusionWeights, model, triggerConfig);
        fusedObjects = fuseLmbPosteriorsByLabel( ...
            fusionInputs, spatialFusionWeights, model, ...
            existenceFusionWeights);
        [objectsBySensor{receiverIdx}, stateEstimatesBySensor{receiverIdx}] = ...
            finalizeNodeStep( ...
                fusedObjects, stateEstimatesBySensor{receiverIdx}, ...
                model, currentTime);
    end
    diagnostics.effectiveWeightAlgebraicConnectivity(currentTime) = ...
        computeWeightedAlgebraicConnectivity( ...
            diagnostics.fusionWeight(:, :, currentTime));
    previousDirectedEdgeMask = currentDirectedEdgeMask;
end

for sensorIdx = 1:numberOfSensors
    surviving = objectsBySensor{sensorIdx};
    if ~isempty(surviving)
        keep = [surviving.trajectoryLength] > model.minimumTrajectoryLength;
        stateEstimatesBySensor{sensorIdx}.objects = [ ...
            stateEstimatesBySensor{sensorIdx}.objects, surviving(keep)];
    end
end
diagnostics = summarizeDiagnostics(diagnostics, baseDirectedEdgeMask);
end

function config = resolveTriggerConfig(config, model)
config.eventPolicy = getField(config, 'eventPolicy', 'dual');
config.criterionMode = getField(config, 'criterionMode', 'multi');
config.thresholdLow = getField(config, 'thresholdLow', 0.25);
config.thresholdHigh = getField(config, 'thresholdHigh', 0.60);
config.criterionWeights = getField( ...
    config, 'criterionWeights', [0.20, 0.20, 0.35, 0.25]);
config.linkGateEnabled = getField(config, 'linkGateEnabled', true);
config.poorLinkThreshold = getField(config, 'poorLinkThreshold', 0.35);
config.moderateLinkThreshold = getField( ...
    config, 'moderateLinkThreshold', 0.70);
config.recentWindow = max(1, round(getField( ...
    config, 'recentWindow', 5)));
config.maxReferenceAge = max(1, round(getField( ...
    config, 'maxReferenceAge', 10)));
config.forceInitialHeavy = getField(config, 'forceInitialHeavy', true);
config.forceLabelChangeHeavy = getField( ...
    config, 'forceLabelChangeHeavy', true);
config.forceLabelExistenceThreshold = getField( ...
    config, 'forceLabelExistenceThreshold', 0.5);
config.forceStaleHeavy = getField(config, 'forceStaleHeavy', false);
config.forceNewEdgeHandshakeHeavy = getField( ...
    config, 'forceNewEdgeHandshakeHeavy', false);
config.newEdgeHandshakeBypassLinkGate = getField( ...
    config, 'newEdgeHandshakeBypassLinkGate', false);
config.newEdgeHandshakeExistenceThreshold = getField( ...
    config, 'newEdgeHandshakeExistenceThreshold', ...
    config.forceLabelExistenceThreshold);
config.activeExistenceThreshold = getField( ...
    config, 'activeExistenceThreshold', model.existenceThreshold);
config.payloadExistenceThreshold = getField( ...
    config, 'payloadExistenceThreshold', model.existenceThreshold);
config.useStaleNeighborCache = getField( ...
    config, 'useStaleNeighborCache', false);
config.maxStaleFusionAge = max(0, round(getField( ...
    config, 'maxStaleFusionAge', 0)));
config.staleFusionWeightDecay = min(max(getField( ...
    config, 'staleFusionWeightDecay', 0.70), 0), 1);
config.labelHeartbeatEnabled = getField( ...
    config, 'labelHeartbeatEnabled', false);
config.labelHeartbeatMaxAge = max(1, round(getField( ...
    config, 'labelHeartbeatMaxAge', config.maxReferenceAge)));
config.labelHeartbeatEventType = min(max(round(getField( ...
    config, 'labelHeartbeatEventType', 1)), 1), 2);
config.labelHeartbeatExistenceThreshold = getField( ...
    config, 'labelHeartbeatExistenceThreshold', ...
    config.forceLabelExistenceThreshold);
config.missingNeighborWeightMode = getField( ...
    config, 'missingNeighborWeightMode', 'renormalize');
config.dynamicTopologyEnabled = getField( ...
    config, 'dynamicTopologyEnabled', false);
config.dynamicTopologyEdgeBudget = getField( ...
    config, 'dynamicTopologyEdgeBudget', []);
config.topologyReliabilityWeight = getField( ...
    config, 'topologyReliabilityWeight', 0.45);
config.topologyOverlapWeight = getField( ...
    config, 'topologyOverlapWeight', 0.35);
config.topologyComplementarityWeight = getField( ...
    config, 'topologyComplementarityWeight', 0.20);
config.topologyScoreMode = getField( ...
    config, 'topologyScoreMode', 'legacy');
config.topologyExpectedFusionWeight = getField( ...
    config, 'topologyExpectedFusionWeight', 0.35);
config.topologyPredictedTriggerWeight = getField( ...
    config, 'topologyPredictedTriggerWeight', 0.25);
config.topologyConnectivityRepairWeight = getField( ...
    config, 'topologyConnectivityRepairWeight', 0.20);
config.topologyCoverageRepairWeight = getField( ...
    config, 'topologyCoverageRepairWeight', 0.15);
config.topologyBytePenaltyWeight = getField( ...
    config, 'topologyBytePenaltyWeight', 0.05);
config.topologyStaticEdgeBonus = getField( ...
    config, 'topologyStaticEdgeBonus', 0);
config.topologyActiveExistenceThreshold = getField( ...
    config, 'topologyActiveExistenceThreshold', ...
    config.forceLabelExistenceThreshold);
config.topologyMinAlgebraicConnectivity = getField( ...
    config, 'topologyMinAlgebraicConnectivity', 0);
config.topologyFallbackToBaseOnConnectivityFailure = getField( ...
    config, 'topologyFallbackToBaseOnConnectivityFailure', false);
config.effectiveGraphWindow = max(1, round(getField( ...
    config, 'effectiveGraphWindow', 5)));
config.effectiveGraphExistenceThreshold = getField( ...
    config, 'effectiveGraphExistenceThreshold', ...
    config.forceLabelExistenceThreshold);
config.localQualityGateEnabled = getField( ...
    config, 'localQualityGateEnabled', true);
config.modeAwareFusionWeights = getField( ...
    config, 'modeAwareFusionWeights', false);
config.lightFusionWeightFactor = min(max(getField( ...
    config, 'lightFusionWeightFactor', 0.55), 0), 1);
config.heavyFusionWeightFactor = min(max(getField( ...
    config, 'heavyFusionWeightFactor', 1.0), 0), 1);
config.staleFusionWeightFactor = min(max(getField( ...
    config, 'staleFusionWeightFactor', 0.20), 0), 1);
config.maxSelfFusionWeight = getField( ...
    config, 'maxSelfFusionWeight', inf);
config.mixedPayloadEnabled = getField( ...
    config, 'mixedPayloadEnabled', false);
config.mixedPayloadLightForAllActiveLabels = getField( ...
    config, 'mixedPayloadLightForAllActiveLabels', true);
config.mixedPayloadLightThreshold = getField( ...
    config, 'mixedPayloadLightThreshold', config.thresholdLow);
config.mixedPayloadHeavyThreshold = getField( ...
    config, 'mixedPayloadHeavyThreshold', config.thresholdHigh);
config.lightCovarianceInflationEnabled = getField( ...
    config, 'lightCovarianceInflationEnabled', false);
config.lightCovarianceInflationBase = getField( ...
    config, 'lightCovarianceInflationBase', 0);
config.lightCovarianceAssociationScale = getField( ...
    config, 'lightCovarianceAssociationScale', 0);
config.lightCovarianceMixtureScale = getField( ...
    config, 'lightCovarianceMixtureScale', 0);
config.fusionWeightMode = getField( ...
    config, 'fusionWeightMode', 'metropolis');
config.adaptiveFusionConfig = getField( ...
    config, 'adaptiveFusionConfig', struct());
end

function config = resolveCommunicationConfig(config, numberOfSensors, simulationLength)
config.pDrop = min(max(getField(config, 'pDrop', 0), 0), 1);
config.pDropBySensor = getField(config, 'pDropBySensor', []);
config.pDropByEdge = getField(config, 'pDropByEdge', []);
config.forceDelivery = getField(config, 'forceDelivery', false);
config.outageSchedule = getField(config, 'outageSchedule', []);
config.sensorSampleMask = getField(config, 'sensorSampleMask', []);
config.linkUniforms = getField(config, 'linkUniforms', []);
if isempty(config.pDropBySensor)
    levels = getField(config, 'pDropLevels', []);
    counts = getField(config, 'pDropLevelCounts', []);
    config.pDropBySensor = resolveTieredDropRates( ...
        levels, counts, numberOfSensors, config.pDrop);
end
if numel(config.pDropBySensor) ~= numberOfSensors
    config.pDropBySensor = config.pDrop * ones(1, numberOfSensors);
end
config.pDropBySensor = min(max( ...
    reshape(config.pDropBySensor, 1, []), 0), 1);
if ~isempty(config.pDropByEdge) && ...
        ~isequal(size(config.pDropByEdge), ...
            [numberOfSensors, numberOfSensors])
    error('commConfig.pDropByEdge must be S-by-S.');
end
if ~isempty(config.linkUniforms) && ...
        ~isequal(size(config.linkUniforms), ...
            [numberOfSensors, numberOfSensors, simulationLength])
    error('commConfig.linkUniforms must be S-by-S-by-T.');
end
end

function rates = resolveTieredDropRates(levels, counts, numberOfSensors, fallback)
rates = fallback * ones(1, numberOfSensors);
if isempty(levels) || isempty(counts)
    return;
end
levels = reshape(levels, 1, []);
counts = round(reshape(counts, 1, []));
if numel(levels) ~= numel(counts) || ...
        any(counts < 0) || sum(counts) ~= numberOfSensors
    return;
end
assigned = zeros(1, numberOfSensors);
cursor = 1;
for idx = 1:numel(levels)
    count = counts(idx);
    if count > 0
        assigned(cursor:cursor + count - 1) = levels(idx);
        cursor = cursor + count;
    end
end
rates = assigned(randperm(numberOfSensors));
end

function edgeMask = buildDirectedEdgeMask(neighborMap, numberOfSensors)
edgeMask = false(numberOfSensors);
for receiverIdx = 1:numberOfSensors
    senders = neighborMap{receiverIdx};
    senders = senders(senders ~= receiverIdx);
    edgeMask(senders, receiverIdx) = true;
end
end

function weights = computeMetropolisWeightMatrix(neighborMap)
numberOfSensors = numel(neighborMap);
degrees = cellfun(@numel, neighborMap);
weights = zeros(numberOfSensors);
for receiverIdx = 1:numberOfSensors
    neighbors = reshape(neighborMap{receiverIdx}, 1, []);
    offDiagonalSum = 0;
    for senderIdx = neighbors
        if senderIdx == receiverIdx
            continue;
        end
        value = 1 / (1 + max( ...
            degrees(receiverIdx), degrees(senderIdx)));
        weights(receiverIdx, senderIdx) = value;
        offDiagonalSum = offDiagonalSum + value;
    end
    weights(receiverIdx, receiverIdx) = max(0, 1 - offDiagonalSum);
    rowSum = sum(weights(receiverIdx, :));
    if rowSum <= 0
        weights(receiverIdx, receiverIdx) = 1;
    else
        weights(receiverIdx, :) = weights(receiverIdx, :) / rowSum;
    end
end
end

function [currentNeighborMap, currentTopologyWeights, details] = ...
    resolveActiveTopology(baseNeighborMap, baseTopologyWeights, ...
        localPosteriorBySensor, model, sensorTrajectories, commConfig, ...
        triggerConfig, currentTime)
details = struct( ...
    'algebraicConnectivity', computeAlgebraicConnectivity(baseNeighborMap), ...
    'undirectedEdgeCount', countUndirectedEdges(baseNeighborMap));
if ~triggerConfig.dynamicTopologyEnabled
    currentNeighborMap = baseNeighborMap;
    currentTopologyWeights = baseTopologyWeights;
    return;
end

numberOfSensors = numel(baseNeighborMap);
edgeBudget = triggerConfig.dynamicTopologyEdgeBudget;
if isempty(edgeBudget)
    edgeBudget = countUndirectedEdges(baseNeighborMap);
end
edgeBudget = min(max(round(edgeBudget), numberOfSensors - 1), ...
    numberOfSensors * (numberOfSensors - 1) / 2);

edgeScores = computeTopologyBenefitMatrix( ...
    localPosteriorBySensor, model, sensorTrajectories, commConfig, ...
    triggerConfig, currentTime);
baseAdjacency = neighborMapToAdjacency(baseNeighborMap);
staticEdgeBonus = triggerConfig.topologyStaticEdgeBonus;
if staticEdgeBonus > 0
    edgeScores(baseAdjacency) = edgeScores(baseAdjacency) + staticEdgeBonus;
end
adjacency = selectBudgetedConnectedTopology(edgeScores, edgeBudget);
minConnectivity = triggerConfig.topologyMinAlgebraicConnectivity;
if minConnectivity < 0
    minConnectivity = computeAlgebraicConnectivity(baseNeighborMap);
end
if minConnectivity > 0
    adjacency = repairTopologyConnectivity( ...
        adjacency, edgeScores, minConnectivity);
end
if triggerConfig.topologyFallbackToBaseOnConnectivityFailure
    repairedConnectivity = computeAlgebraicConnectivity( ...
        adjacencyToNeighborMap(adjacency));
    if repairedConnectivity + 1e-9 < minConnectivity
        adjacency = baseAdjacency;
    end
end
currentNeighborMap = adjacencyToNeighborMap(adjacency);
currentTopologyWeights = computeMetropolisWeightMatrix(currentNeighborMap);
details.algebraicConnectivity = computeAlgebraicConnectivity(currentNeighborMap);
details.undirectedEdgeCount = countUndirectedEdges(currentNeighborMap);
end

function scores = computeTopologyBenefitMatrix( ...
    localPosteriorBySensor, model, sensorTrajectories, commConfig, ...
    triggerConfig, currentTime)
numberOfSensors = numel(localPosteriorBySensor);
scores = -inf(numberOfSensors);
positions = resolveSensorPositions(model, sensorTrajectories, currentTime);
commRange = getField(model, 'sensorCommRange', inf);
reliabilityWeight = triggerConfig.topologyReliabilityWeight;
overlapWeight = triggerConfig.topologyOverlapWeight;
complementarityWeight = triggerConfig.topologyComplementarityWeight;
weightSum = max(reliabilityWeight + overlapWeight + ...
    complementarityWeight, eps);

for leftIdx = 1:numberOfSensors-1
    for rightIdx = leftIdx+1:numberOfSensors
        distance = norm(positions(:, leftIdx) - positions(:, rightIdx));
        if isfinite(commRange) && distance > commRange
            continue;
        end
        reliability = 1 - 0.5 * ( ...
            resolveEdgeDropProbability(commConfig, leftIdx, rightIdx) + ...
            resolveEdgeDropProbability(commConfig, rightIdx, leftIdx));
        overlap = computeLabelOverlapScore( ...
            localPosteriorBySensor{leftIdx}, ...
            localPosteriorBySensor{rightIdx}, ...
            triggerConfig.topologyActiveExistenceThreshold);
        complementarity = computeGeometricComplementarity( ...
            distance, commRange);
        if strcmpi(triggerConfig.topologyScoreMode, 'crosslayer')
            score = computeCrossLayerTopologyScore( ...
                localPosteriorBySensor{leftIdx}, ...
                localPosteriorBySensor{rightIdx}, model, ...
                triggerConfig, reliability, overlap, complementarity);
        else
            score = (reliabilityWeight * reliability + ...
                overlapWeight * overlap + ...
                complementarityWeight * complementarity) / weightSum;
        end
        scores(leftIdx, rightIdx) = score;
        scores(rightIdx, leftIdx) = score;
    end
end
end

function score = computeCrossLayerTopologyScore( ...
    leftObjects, rightObjects, model, triggerConfig, reliability, ...
    overlap, complementarity)
fusionValue = computePairwiseExpectedFusionValue( ...
    leftObjects, rightObjects, model, triggerConfig);
predictedTrigger = computePredictedTriggerScore( ...
    fusionValue, triggerConfig);
expectedFusionWeight = reliability * predictedTrigger * fusionValue;
coverageRepair = computeCoverageRepairScore( ...
    leftObjects, rightObjects, triggerConfig.topologyActiveExistenceThreshold);
bytePenalty = estimatePairwiseBytePenalty( ...
    leftObjects, rightObjects, model, triggerConfig);
rawScore = ...
    triggerConfig.topologyExpectedFusionWeight * expectedFusionWeight + ...
    triggerConfig.topologyPredictedTriggerWeight * predictedTrigger + ...
    triggerConfig.topologyConnectivityRepairWeight * complementarity + ...
    triggerConfig.topologyCoverageRepairWeight * coverageRepair - ...
    triggerConfig.topologyBytePenaltyWeight * bytePenalty;
score = reliability * rawScore + 0.10 * overlap;
score = max(score, 0);
end

function value = computePairwiseExpectedFusionValue( ...
    leftObjects, rightObjects, model, triggerConfig)
threshold = triggerConfig.topologyActiveExistenceThreshold;
labels = unionActiveLabels(leftObjects, rightObjects, threshold);
if isempty(labels)
    value = 0;
    return;
end
scores = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    leftObject = findActiveObject(leftObjects, label, threshold);
    rightObject = findActiveObject(rightObjects, label, threshold);
    scores(labelIdx) = compareTopologyObjects( ...
        leftObject, rightObject, model);
end
value = min(max(mean(scores), 0), 1);
end

function predicted = computePredictedTriggerScore(fusionValue, triggerConfig)
thresholdLow = triggerConfig.thresholdLow;
thresholdHigh = max(triggerConfig.thresholdHigh, thresholdLow + eps);
predicted = (fusionValue - thresholdLow) / (thresholdHigh - thresholdLow);
predicted = min(max(predicted, 0), 1);
end

function score = computeCoverageRepairScore( ...
    leftObjects, rightObjects, threshold)
leftLabels = collectActiveLabels(leftObjects, threshold);
rightLabels = collectActiveLabels(rightObjects, threshold);
if isempty(leftLabels) && isempty(rightLabels)
    score = 0;
    return;
end
leftOnly = countLabelDifference(leftLabels, rightLabels);
rightOnly = countLabelDifference(rightLabels, leftLabels);
unionCount = size(leftLabels, 2) + size(rightLabels, 2) - ...
    (size(leftLabels, 2) - leftOnly);
score = (leftOnly + rightOnly) / max(unionCount, 1);
score = min(max(score, 0), 1);
end

function penalty = estimatePairwiseBytePenalty( ...
    leftObjects, rightObjects, model, triggerConfig)
threshold = triggerConfig.topologyActiveExistenceThreshold;
leftCost = estimateActivePayloadCost(leftObjects, model, threshold);
rightCost = estimateActivePayloadCost(rightObjects, model, threshold);
stateDimension = model.xDimension;
heavyUnit = 3 + 3 * (1 + stateDimension + stateDimension * stateDimension);
normalizer = max(2 * heavyUnit, 1);
penalty = min((leftCost + rightCost) / normalizer, 1);
if triggerConfig.mixedPayloadEnabled
    penalty = 0.75 * penalty;
end
end

function cost = estimateActivePayloadCost(objects, model, threshold)
cost = 0;
if isempty(objects)
    return;
end
stateDimension = model.xDimension;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).r <= threshold || ...
            objects(objectIdx).numberOfGmComponents <= 0
        continue;
    end
    componentCount = max(1, objects(objectIdx).numberOfGmComponents);
    cost = cost + 3 + componentCount * ...
        (1 + stateDimension + stateDimension * stateDimension);
end
end

function count = countLabelDifference(leftLabels, rightLabels)
count = 0;
for labelIdx = 1:size(leftLabels, 2)
    if isempty(rightLabels) || ...
            ~any(all(rightLabels == leftLabels(:, labelIdx), 1))
        count = count + 1;
    end
end
end

function labels = unionActiveLabels(leftObjects, rightObjects, threshold)
labels = collectActiveLabels(leftObjects, threshold);
rightLabels = collectActiveLabels(rightObjects, threshold);
for labelIdx = 1:size(rightLabels, 2)
    label = rightLabels(:, labelIdx);
    if isempty(labels) || ~any(all(labels == label, 1))
        labels(:, end+1) = label; %#ok<AGROW>
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function object = findActiveObject(objects, label, threshold)
object = [];
if isempty(objects)
    return;
end
for objectIdx = 1:numel(objects)
    if objects(objectIdx).r > threshold && ...
            objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        object = objects(objectIdx);
        return;
    end
end
end

function score = compareTopologyObjects(leftObject, rightObject, model)
if isempty(leftObject) && isempty(rightObject)
    score = 0;
    return;
end
if isempty(leftObject) || isempty(rightObject)
    score = 1;
    return;
end
[leftMu, leftCov] = topologyMomentMatch(leftObject, model.xDimension);
[rightMu, rightCov] = topologyMomentMatch(rightObject, model.xDimension);
positionDimension = min(2, model.xDimension);
delta = leftMu(1:positionDimension) - rightMu(1:positionDimension);
scaleCov = leftCov(1:positionDimension, 1:positionDimension) + ...
    rightCov(1:positionDimension, 1:positionDimension);
scaleCov = regularizeTopologyCovariance(scaleCov);
mahalanobis = delta' * (scaleCov \ delta);
positionScore = 1 - exp(-0.5 * max(mahalanobis, 0));
existenceScore = abs(leftObject.r - rightObject.r);
traceScore = abs(log((trace(leftCov) + eps) / ...
    (trace(rightCov) + eps)));
traceScore = 1 - exp(-traceScore);
score = min(max(0.50 * positionScore + ...
    0.30 * existenceScore + 0.20 * traceScore, 0), 1);
end

function [mu, covariance] = topologyMomentMatch(object, stateDimension)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if sum(weights) <= 0
    weights = ones(size(weights)) / max(numel(weights), 1);
else
    weights = weights / sum(weights);
end
mu = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    mu = mu + weights(componentIdx) * object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - mu;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeTopologyCovariance(covariance);
end

function covariance = regularizeTopologyCovariance(covariance)
covariance = (covariance + covariance') / 2;
if rcond(covariance) < 1e-12
    covariance = covariance + 1e-9 * eye(size(covariance));
end
end

function positions = resolveSensorPositions(model, sensorTrajectories, currentTime)
numberOfSensors = model.numberOfSensors;
positions = zeros(2, numberOfSensors);
if ~isempty(sensorTrajectories)
    for sensorIdx = 1:numberOfSensors
        timeIdx = min(currentTime, size(sensorTrajectories{sensorIdx}, 2));
        positions(:, sensorIdx) = sensorTrajectories{sensorIdx}(1:2, timeIdx);
    end
elseif isfield(model, 'sensorInitialStates')
    for sensorIdx = 1:numberOfSensors
        positions(:, sensorIdx) = model.sensorInitialStates{sensorIdx}(1:2);
    end
end
end

function score = computeLabelOverlapScore(leftObjects, rightObjects, threshold)
leftLabels = collectActiveLabels(leftObjects, threshold);
rightLabels = collectActiveLabels(rightObjects, threshold);
if isempty(leftLabels) && isempty(rightLabels)
    score = 0;
    return;
end
intersectionCount = 0;
for labelIdx = 1:size(leftLabels, 2)
    if any(all(rightLabels == leftLabels(:, labelIdx), 1))
        intersectionCount = intersectionCount + 1;
    end
end
unionCount = size(leftLabels, 2) + size(rightLabels, 2) - intersectionCount;
score = intersectionCount / max(unionCount, 1);
end

function score = computeGeometricComplementarity(distance, commRange)
if ~isfinite(commRange) || commRange <= eps
    score = 0.5;
    return;
end
normalized = min(max(distance / commRange, 0), 1);
score = 4 * normalized * (1 - normalized);
end

function adjacency = selectBudgetedConnectedTopology(scores, edgeBudget)
numberOfSensors = size(scores, 1);
adjacency = false(numberOfSensors);
edges = sortedCandidateEdges(scores);
if isempty(edges)
    adjacency = true(numberOfSensors) & ~eye(numberOfSensors);
    return;
end

parent = 1:numberOfSensors;
edgeCount = 0;
for edgeIdx = 1:size(edges, 1)
    leftIdx = edges(edgeIdx, 1);
    rightIdx = edges(edgeIdx, 2);
    [parent, leftRoot] = findDisjointSet(parent, leftIdx);
    [parent, rightRoot] = findDisjointSet(parent, rightIdx);
    if leftRoot ~= rightRoot
        parent(rightRoot) = leftRoot;
        adjacency(leftIdx, rightIdx) = true;
        adjacency(rightIdx, leftIdx) = true;
        edgeCount = edgeCount + 1;
        if edgeCount >= numberOfSensors - 1
            break;
        end
    end
end

for edgeIdx = 1:size(edges, 1)
    if edgeCount >= edgeBudget
        break;
    end
    leftIdx = edges(edgeIdx, 1);
    rightIdx = edges(edgeIdx, 2);
    if ~adjacency(leftIdx, rightIdx)
        adjacency(leftIdx, rightIdx) = true;
        adjacency(rightIdx, leftIdx) = true;
        edgeCount = edgeCount + 1;
    end
end
end

function adjacency = neighborMapToAdjacency(neighborMap)
numberOfSensors = numel(neighborMap);
adjacency = false(numberOfSensors);
for sensorIdx = 1:numberOfSensors
    neighbors = neighborMap{sensorIdx};
    neighbors = neighbors(neighbors ~= sensorIdx);
    adjacency(sensorIdx, neighbors) = true;
end
adjacency = adjacency | adjacency';
adjacency(1:numberOfSensors+1:end) = false;
end

function adjacency = repairTopologyConnectivity( ...
    adjacency, scores, minConnectivity)
currentConnectivity = computeAlgebraicConnectivity( ...
    adjacencyToNeighborMap(adjacency));
if currentConnectivity >= minConnectivity
    return;
end

numberOfSensors = size(adjacency, 1);
for iterationIdx = 1:20
    bestAdjacency = adjacency;
    bestConnectivity = currentConnectivity;
    selected = find(triu(adjacency, 1));
    candidates = find(triu(~adjacency & isfinite(scores), 1));
    for removeCursor = 1:numel(selected)
        [removeLeft, removeRight] = ind2sub( ...
            [numberOfSensors, numberOfSensors], selected(removeCursor));
        for addCursor = 1:numel(candidates)
            [addLeft, addRight] = ind2sub( ...
                [numberOfSensors, numberOfSensors], candidates(addCursor));
            trialAdjacency = adjacency;
            trialAdjacency(removeLeft, removeRight) = false;
            trialAdjacency(removeRight, removeLeft) = false;
            trialAdjacency(addLeft, addRight) = true;
            trialAdjacency(addRight, addLeft) = true;
            trialConnectivity = computeAlgebraicConnectivity( ...
                adjacencyToNeighborMap(trialAdjacency));
            if trialConnectivity > bestConnectivity + 1e-9
                bestConnectivity = trialConnectivity;
                bestAdjacency = trialAdjacency;
            end
        end
    end
    if bestConnectivity <= currentConnectivity + 1e-9
        break;
    end
    adjacency = bestAdjacency;
    currentConnectivity = bestConnectivity;
    if currentConnectivity >= minConnectivity
        break;
    end
end
end

function edges = sortedCandidateEdges(scores)
numberOfSensors = size(scores, 1);
edges = zeros(0, 3);
for leftIdx = 1:numberOfSensors-1
    for rightIdx = leftIdx+1:numberOfSensors
        score = scores(leftIdx, rightIdx);
        if isfinite(score)
            edges(end+1, :) = [leftIdx, rightIdx, score]; %#ok<AGROW>
        end
    end
end
if ~isempty(edges)
    [~, order] = sort(edges(:, 3), 'descend');
    edges = edges(order, :);
end
end

function [parent, root] = findDisjointSet(parent, nodeIdx)
root = nodeIdx;
while parent(root) ~= root
    root = parent(root);
end
while parent(nodeIdx) ~= nodeIdx
    nextIdx = parent(nodeIdx);
    parent(nodeIdx) = root;
    nodeIdx = nextIdx;
end
end

function neighborMap = adjacencyToNeighborMap(adjacency)
numberOfSensors = size(adjacency, 1);
neighborMap = cell(1, numberOfSensors);
for sensorIdx = 1:numberOfSensors
    neighborMap{sensorIdx} = find(adjacency(sensorIdx, :));
    neighborMap{sensorIdx} = unique([sensorIdx, neighborMap{sensorIdx}]);
end
end

function value = computeAlgebraicConnectivity(neighborMap)
numberOfSensors = numel(neighborMap);
adjacency = false(numberOfSensors);
for sensorIdx = 1:numberOfSensors
    neighbors = neighborMap{sensorIdx};
    neighbors = neighbors(neighbors ~= sensorIdx);
    adjacency(sensorIdx, neighbors) = true;
end
adjacency = adjacency | adjacency';
laplacian = diag(sum(adjacency, 2)) - double(adjacency);
eigenvalues = sort(real(eig(laplacian)));
if numel(eigenvalues) < 2
    value = 0;
else
    value = eigenvalues(2);
end
end

function count = countUndirectedEdges(neighborMap)
numberOfSensors = numel(neighborMap);
adjacency = false(numberOfSensors);
for sensorIdx = 1:numberOfSensors
    neighbors = neighborMap{sensorIdx};
    neighbors = neighbors(neighbors ~= sensorIdx);
    adjacency(sensorIdx, neighbors) = true;
end
adjacency = adjacency | adjacency';
count = nnz(triu(adjacency, 1));
end

function tf = shouldForceNewEdgeHandshake( ...
    isNewEdge, senderObjects, triggerConfig)
tf = false;
if ~isNewEdge || ~triggerConfig.forceNewEdgeHandshakeHeavy
    return;
end
labels = collectActiveLabels( ...
    senderObjects, triggerConfig.newEdgeHandshakeExistenceThreshold);
tf = ~isempty(labels);
end

function diagnostics = recordCommunicationGraphDiagnostics( ...
    diagnostics, localPosteriorBySensor, labelRefreshCache, ...
    currentNeighborMap, triggerConfig, currentTime)
attemptedMask = diagnostics.attempted(:, :, currentTime);
deliveredMask = diagnostics.delivered(:, :, currentTime);
diagnostics.attemptedAlgebraicConnectivity(currentTime) = ...
    computeDirectedMaskAlgebraicConnectivity(attemptedMask);
diagnostics.deliveredAlgebraicConnectivity(currentTime) = ...
    computeDirectedMaskAlgebraicConnectivity(deliveredMask);

windowStart = max(1, currentTime - triggerConfig.effectiveGraphWindow + 1);
windowDeliveredMask = any( ...
    diagnostics.delivered(:, :, windowStart:currentTime), 3);
diagnostics.windowDeliveredAlgebraicConnectivity(currentTime) = ...
    computeDirectedMaskAlgebraicConnectivity(windowDeliveredMask);

[currentViolation, windowViolation, staleP90, staleP95] = ...
    computePerLabelEffectiveGraphDiagnostics( ...
        localPosteriorBySensor, deliveredMask, windowDeliveredMask, ...
        labelRefreshCache, currentNeighborMap, triggerConfig, currentTime);
diagnostics.perLabelConnectivityViolationRate(currentTime) = ...
    currentViolation;
diagnostics.perLabelWindowConnectivityViolationRate(currentTime) = ...
    windowViolation;
diagnostics.perLabelStaleAgeP90(currentTime) = staleP90;
diagnostics.perLabelStaleAgeP95(currentTime) = staleP95;
end

function [currentViolation, windowViolation, staleP90, staleP95] = ...
    computePerLabelEffectiveGraphDiagnostics( ...
        localPosteriorBySensor, deliveredMask, windowDeliveredMask, ...
        labelRefreshCache, currentNeighborMap, triggerConfig, currentTime)
threshold = triggerConfig.effectiveGraphExistenceThreshold;
labels = collectActiveLabelsAcrossSensors(localPosteriorBySensor, threshold);
if isempty(labels)
    currentViolation = 0;
    windowViolation = 0;
    staleP90 = 0;
    staleP95 = 0;
    return;
end

currentViolations = 0;
windowViolations = 0;
eligibleLabelCount = 0;
staleAges = [];
numberOfSensors = numel(localPosteriorBySensor);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    vertices = false(1, numberOfSensors);
    for sensorIdx = 1:numberOfSensors
        vertices(sensorIdx) = hasActiveLabel( ...
            localPosteriorBySensor{sensorIdx}, label, threshold);
    end
    if nnz(vertices) < 2
        continue;
    end
    eligibleLabelCount = eligibleLabelCount + 1;
    labelCurrentMask = false(numberOfSensors);
    labelWindowMask = false(numberOfSensors);
    for senderIdx = 1:numberOfSensors
        if ~vertices(senderIdx)
            continue;
        end
        for receiverIdx = find(vertices)
            if senderIdx == receiverIdx
                continue;
            end
            if deliveredMask(senderIdx, receiverIdx)
                labelCurrentMask(senderIdx, receiverIdx) = true;
            end
            if windowDeliveredMask(senderIdx, receiverIdx)
                labelWindowMask(senderIdx, receiverIdx) = true;
            end
        end
        topologyReceivers = reshape(currentNeighborMap{senderIdx}, 1, []);
        topologyReceivers = topologyReceivers(topologyReceivers ~= senderIdx);
        for receiverIdx = topologyReceivers
            age = resolveLabelRefreshAge( ...
                labelRefreshCache{senderIdx, receiverIdx}, ...
                label, currentTime);
            staleAges(end+1) = age; %#ok<AGROW>
        end
    end
    if computeDirectedMaskAlgebraicConnectivity(labelCurrentMask) <= 1e-9
        currentViolations = currentViolations + 1;
    end
    if computeDirectedMaskAlgebraicConnectivity(labelWindowMask) <= 1e-9
        windowViolations = windowViolations + 1;
    end
end
if eligibleLabelCount <= 0
    currentViolation = 0;
    windowViolation = 0;
else
    currentViolation = currentViolations / eligibleLabelCount;
    windowViolation = windowViolations / eligibleLabelCount;
end
if isempty(staleAges)
    staleP90 = 0;
    staleP95 = 0;
else
    staleP90 = percentileValue(staleAges, 0.90);
    staleP95 = percentileValue(staleAges, 0.95);
end
end

function labels = collectActiveLabelsAcrossSensors(objectsBySensor, threshold)
labels = zeros(2, 0);
for sensorIdx = 1:numel(objectsBySensor)
    sensorLabels = collectActiveLabels(objectsBySensor{sensorIdx}, threshold);
    for labelIdx = 1:size(sensorLabels, 2)
        label = sensorLabels(:, labelIdx);
        if isempty(labels) || ~any(all(labels == label, 1))
            labels(:, end+1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function tf = hasActiveLabel(objects, label, threshold)
tf = false;
if isempty(objects)
    return;
end
for objectIdx = 1:numel(objects)
    if objects(objectIdx).r > threshold && ...
            objects(objectIdx).numberOfGmComponents > 0 && ...
            objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        tf = true;
        return;
    end
end
end

function age = resolveLabelRefreshAge(refreshCache, label, currentTime)
lastRefresh = findLabelRefreshTime(refreshCache, label);
if lastRefresh <= 0
    age = currentTime;
else
    age = currentTime - lastRefresh;
end
end

function value = computeDirectedMaskAlgebraicConnectivity(mask)
if isempty(mask)
    value = 0;
    return;
end
adjacency = logical(mask) | logical(mask');
adjacency(1:size(adjacency, 1)+1:end) = false;
value = computeAdjacencyAlgebraicConnectivity(double(adjacency));
end

function value = computeWeightedAlgebraicConnectivity(weightMatrix)
if isempty(weightMatrix)
    value = 0;
    return;
end
weights = max(weightMatrix, weightMatrix');
weights(1:size(weights, 1)+1:end) = 0;
value = computeAdjacencyAlgebraicConnectivity(weights);
end

function value = computeAdjacencyAlgebraicConnectivity(adjacency)
numberOfSensors = size(adjacency, 1);
if numberOfSensors < 2
    value = 0;
    return;
end
laplacian = diag(sum(adjacency, 2)) - adjacency;
eigenvalues = sort(real(eig(laplacian)));
if numel(eigenvalues) < 2
    value = 0;
else
    value = eigenvalues(2);
end
end

function value = percentileValue(values, probability)
values = sort(reshape(values(isfinite(values)), 1, []));
if isempty(values)
    value = 0;
    return;
end
position = 1 + min(max(probability, 0), 1) * (numel(values) - 1);
lowerIdx = floor(position);
upperIdx = ceil(position);
if lowerIdx == upperIdx
    value = values(lowerIdx);
else
    fraction = position - lowerIdx;
    value = values(lowerIdx) * (1 - fraction) + ...
        values(upperIdx) * fraction;
end
end

function [inputs, weights, details] = collectCurrentFusionInputs( ...
    receiverIdx, localPosterior, currentMessages, currentMessagePresent, ...
    currentMessageTypes, receivedCache, lastSuccessfulTime, neighborMap, ...
    topologyWeights, model, triggerConfig, currentTime)
inputs = {localPosterior};
sourceIndices = receiverIdx;
rawWeights = topologyWeights(receiverIdx, receiverIdx);
missingWeightMass = 0;
details = struct( ...
    'sourceIndices', receiverIdx, ...
    'isStale', false, ...
    'age', 0, ...
    'eventType', 2, ...
    'weights', 1);
senders = reshape(neighborMap{receiverIdx}, 1, []);
senders = senders(senders ~= receiverIdx);
for senderIdx = senders
    if currentMessagePresent(receiverIdx, senderIdx)
        inputs{end+1} = currentMessages{receiverIdx, senderIdx}; %#ok<AGROW>
        sourceIndices(end+1) = senderIdx; %#ok<AGROW>
        rawWeights(end+1) = topologyWeights(receiverIdx, senderIdx); %#ok<AGROW>
        details.sourceIndices(end+1) = senderIdx; %#ok<AGROW>
        details.isStale(end+1) = false; %#ok<AGROW>
        details.age(end+1) = 0; %#ok<AGROW>
        details.eventType(end+1) = ...
            currentMessageTypes(receiverIdx, senderIdx); %#ok<AGROW>
    else
        usedFallback = false;
        if triggerConfig.useStaleNeighborCache
        staleAge = resolveReferenceAge( ...
            lastSuccessfulTime(senderIdx, receiverIdx), currentTime);
        if isfinite(staleAge) && staleAge <= triggerConfig.maxStaleFusionAge && ...
                ~isempty(receivedCache{receiverIdx, senderIdx})
            stalePosterior = predictCachedPosterior( ...
                receivedCache{receiverIdx, senderIdx}, model, staleAge, ...
                triggerConfig.payloadExistenceThreshold);
            if ~isempty(stalePosterior)
                inputs{end+1} = stalePosterior; %#ok<AGROW>
                sourceIndices(end+1) = senderIdx; %#ok<AGROW>
                staleWeight = topologyWeights(receiverIdx, senderIdx) * ...
                    triggerConfig.staleFusionWeightDecay^staleAge;
                rawWeights(end+1) = staleWeight; %#ok<AGROW>
                details.sourceIndices(end+1) = senderIdx; %#ok<AGROW>
                details.isStale(end+1) = true; %#ok<AGROW>
                details.age(end+1) = staleAge; %#ok<AGROW>
                details.eventType(end+1) = 0; %#ok<AGROW>
                usedFallback = true;
            end
        end
        end
        if ~usedFallback || strcmpi( ...
                triggerConfig.missingNeighborWeightMode, 'self')
            originalWeight = topologyWeights(receiverIdx, senderIdx);
            if usedFallback
                originalWeight = max(originalWeight - rawWeights(end), 0);
            end
            missingWeightMass = missingWeightMass + originalWeight;
        end
    end
end
weights = rawWeights;
if strcmpi(triggerConfig.missingNeighborWeightMode, 'self')
    weights(1) = weights(1) + missingWeightMass;
end
weights = applyModeAwareFusionWeightFactors( ...
    weights, details, triggerConfig);
weights = normalizeFusionInputWeights(weights);
weights = capSelfFusionWeight(weights, triggerConfig);
details.weights = weights;
end

function weights = applyModeAwareFusionWeightFactors( ...
    weights, details, triggerConfig)
if ~triggerConfig.modeAwareFusionWeights
    return;
end
factors = ones(size(weights));
for sourceIdx = 2:numel(weights)
    if details.isStale(sourceIdx)
        factors(sourceIdx) = triggerConfig.staleFusionWeightFactor;
    elseif details.eventType(sourceIdx) == 1
        factors(sourceIdx) = triggerConfig.lightFusionWeightFactor;
    elseif details.eventType(sourceIdx) >= 2
        factors(sourceIdx) = triggerConfig.heavyFusionWeightFactor;
    else
        factors(sourceIdx) = 0;
    end
end
weights = weights .* factors;
end

function weights = normalizeFusionInputWeights(weights)
if sum(weights) <= 0
    weights = ones(size(weights)) / numel(weights);
else
    weights = weights / sum(weights);
end
end

function weights = capSelfFusionWeight(weights, triggerConfig)
maxSelfWeight = triggerConfig.maxSelfFusionWeight;
if numel(weights) <= 1 || ~isfinite(maxSelfWeight) || ...
        maxSelfWeight <= 0 || maxSelfWeight >= 1 || ...
        weights(1) <= maxSelfWeight
    return;
end
neighborMass = sum(weights(2:end));
if neighborMass <= eps
    return;
end
excess = weights(1) - maxSelfWeight;
weights(1) = maxSelfWeight;
weights(2:end) = weights(2:end) + excess * weights(2:end) / neighborMass;
weights = weights / sum(weights);
end

function diagnostics = recordStaleFusionDiagnostics( ...
    diagnostics, fusionDetails, receiverIdx, currentTime)
for sourceIdx = 1:numel(fusionDetails.sourceIndices)
    senderIdx = fusionDetails.sourceIndices(sourceIdx);
    if senderIdx == receiverIdx || ~fusionDetails.isStale(sourceIdx)
        continue;
    end
    diagnostics.staleFusionUsed(senderIdx, receiverIdx, currentTime) = true;
    diagnostics.staleFusionAge(senderIdx, receiverIdx, currentTime) = ...
        fusionDetails.age(sourceIdx);
    diagnostics.staleFusionWeight(senderIdx, receiverIdx, currentTime) = ...
        fusionDetails.weights(sourceIdx);
end
end

function diagnostics = recordFusionWeightDiagnostics( ...
    diagnostics, fusionDetails, receiverIdx, currentTime)
weights = reshape(fusionDetails.weights, 1, []);
for sourceIdx = 1:numel(fusionDetails.sourceIndices)
    senderIdx = fusionDetails.sourceIndices(sourceIdx);
    weight = weights(sourceIdx);
    if senderIdx == receiverIdx
        diagnostics.fusionSelfWeight(receiverIdx, currentTime) = weight;
    else
        diagnostics.fusionWeight(senderIdx, receiverIdx, currentTime) = weight;
        diagnostics.fusionMode(senderIdx, receiverIdx, currentTime) = ...
            fusionDetails.eventType(sourceIdx);
        diagnostics.fusionNeighborWeight(receiverIdx, currentTime) = ...
            diagnostics.fusionNeighborWeight(receiverIdx, currentTime) + weight;
        if fusionDetails.isStale(sourceIdx)
            diagnostics.fusionStaleWeight(receiverIdx, currentTime) = ...
                diagnostics.fusionStaleWeight(receiverIdx, currentTime) + weight;
        elseif fusionDetails.eventType(sourceIdx) == 1
            diagnostics.fusionLightWeight(receiverIdx, currentTime) = ...
                diagnostics.fusionLightWeight(receiverIdx, currentTime) + weight;
        elseif fusionDetails.eventType(sourceIdx) >= 2
            diagnostics.fusionHeavyWeight(receiverIdx, currentTime) = ...
                diagnostics.fusionHeavyWeight(receiverIdx, currentTime) + weight;
        end
    end
end
positiveWeights = weights(weights > 0);
if isempty(positiveWeights)
    diagnostics.fusionWeightEntropy(receiverIdx, currentTime) = 0;
else
    diagnostics.fusionWeightEntropy(receiverIdx, currentTime) = ...
        -sum(positiveWeights .* log(positiveWeights));
end
end

function predictedObjects = predictCachedPosterior( ...
    objects, model, age, existenceThreshold)
predictedObjects = objects;
if isempty(predictedObjects)
    return;
end
for stepIdx = 1:max(0, round(age))
    for objectIdx = 1:numel(predictedObjects)
        predictedObjects(objectIdx).r = ...
            model.survivalProbability * predictedObjects(objectIdx).r;
        for componentIdx = 1:predictedObjects(objectIdx).numberOfGmComponents
            predictedObjects(objectIdx).mu{componentIdx} = ...
                model.A * predictedObjects(objectIdx).mu{componentIdx} + ...
                model.u;
            predictedObjects(objectIdx).Sigma{componentIdx} = ...
                model.A * predictedObjects(objectIdx).Sigma{componentIdx} * ...
                model.A' + model.R;
        end
    end
end
predictedObjects = selectActiveObjects(predictedObjects, existenceThreshold);
end

function [spatialWeights, existenceWeights] = resolveFusionWeights( ...
    fusionInputs, topologyWeights, model, triggerConfig)
spatialWeights = topologyWeights;
existenceWeights = topologyWeights;
if ~strcmpi(triggerConfig.fusionWeightMode, 'balanced') || ...
        numel(fusionInputs) <= 1
    return;
end

localModel = model;
sourceCount = numel(fusionInputs);
localModel.numberOfSensors = sourceCount;
localModel.gaSensorWeights = topologyWeights;
localModel.aaSensorWeights = topologyWeights;
localModel.gaSpatialWeights = topologyWeights;
localModel.aaSpatialWeights = topologyWeights;
localModel.gaExistenceWeights = topologyWeights;
localModel.aaExistenceWeights = topologyWeights;
localModel.gaTopologyWeights = topologyWeights;
localModel.aaTopologyWeights = topologyWeights;
localModel.gaSpatialStructurePrior = ones(1, sourceCount);
localModel.aaSpatialStructurePrior = ones(1, sourceCount);
localModel.gaExistenceStructurePrior = ones(1, sourceCount);
localModel.aaExistenceStructurePrior = ones(1, sourceCount);
localModel.adaptiveFusion = triggerConfig.adaptiveFusionConfig;
localModel.adaptiveFusion.enabled = true;

dummyMeasurements = cell(sourceCount, 1);
for sourceIdx = 1:sourceCount
    dummyMeasurements{sourceIdx, 1} = {0};
end
dummyCommStats = struct( ...
    'droppedByBandwidth', zeros(sourceCount, 1), ...
    'droppedByLink', zeros(sourceCount, 1), ...
    'droppedByOutage', zeros(sourceCount, 1), ...
    'fusionMask', ones(sourceCount, 1));
previousWeights = struct( ...
    'ga', topologyWeights, ...
    'aa', topologyWeights, ...
    'gaSpatial', topologyWeights, ...
    'aaSpatial', topologyWeights, ...
    'gaExistence', topologyWeights, ...
    'aaExistence', topologyWeights);
[~, ~, debug] = computeAdaptiveFusionWeights( ...
    fusionInputs, dummyMeasurements, localModel, 1, ...
    dummyCommStats, previousWeights);
spatialWeights = getField(debug, 'gaSpatialWeights', topologyWeights);
existenceWeights = getField(debug, 'gaExistenceWeights', topologyWeights);
end

function payload = buildPayload( ...
    objects, model, eventType, triggerConfig, utilityDetails, diagnostics)
if eventType == 1
    payload = compressLmbPosterior( ...
        objects, model, triggerConfig.payloadExistenceThreshold, ...
        triggerConfig, diagnostics);
elseif eventType == 2 && triggerConfig.mixedPayloadEnabled
    payload = buildMixedLmbPayload( ...
        objects, model, triggerConfig, utilityDetails, diagnostics);
else
    payload = selectActiveObjects( ...
        objects, triggerConfig.payloadExistenceThreshold);
end
end

function validateDecodedWireMetadata( ...
    metadata, senderIdx, receiverIdx, currentTime, eventType, objectCount)
actual = [double(metadata.sender), double(metadata.receiver), ...
    double(metadata.timeIndex), double(metadata.eventType)];
expected = double([senderIdx, receiverIdx, currentTime, eventType]);
if ~isequal(actual, expected)
    error('runEventTriggeredDistributedLmbFilter:WireMetadataMismatch', ...
        ['Decoded sender, receiver, time index, or event type does not ' ...
         'match the routed edge event.']);
end
if double(metadata.objectCount) ~= objectCount
    error('runEventTriggeredDistributedLmbFilter:WireObjectCountMismatch', ...
        'Decoded object count does not match the wire header.');
end
end

function scalarCount = computeLegacyPayloadScalarCount( ...
    wireStats, model, diagnostics)
% Compatibility-only scalar count; bytes always come from the wire codec.
if wireStats.objectCount == 0
    scalarCount = 0;
    return;
end
diagnosticCount = 0;
diagnosticFields = {'associationConfidence', 'innovationScore', ...
    'innovationNovelty', 'nisAgg', 'nisNorm', 'nisDeviation'};
for fieldIdx = 1:numel(diagnosticFields)
    if isstruct(diagnostics) && ...
            isfield(diagnostics, diagnosticFields{fieldIdx})
        diagnosticCount = diagnosticCount + ...
            numel(diagnostics.(diagnosticFields{fieldIdx}));
    end
end
stateDimension = model.xDimension;
scalarCount = 4 + diagnosticCount + 3 * wireStats.objectCount + ...
    wireStats.componentCount * ...
    (1 + stateDimension + stateDimension * stateDimension);
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

function [eventType, details] = computeLabelHeartbeatEvent( ...
    currentObjects, refreshCache, currentTime, triggerConfig)
details = struct('maxAge', 0, 'staleCount', 0);
eventType = 0;
if ~triggerConfig.labelHeartbeatEnabled
    return;
end
labels = collectActiveLabels( ...
    currentObjects, triggerConfig.labelHeartbeatExistenceThreshold);
if isempty(labels)
    return;
end

ages = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    lastRefresh = findLabelRefreshTime(refreshCache, labels(:, labelIdx));
    if lastRefresh <= 0
        ages(labelIdx) = inf;
    else
        ages(labelIdx) = currentTime - lastRefresh;
    end
end
stale = ages >= triggerConfig.labelHeartbeatMaxAge;
details.maxAge = max(ages);
details.staleCount = sum(stale);
if any(stale)
    eventType = triggerConfig.labelHeartbeatEventType;
end
end

function refreshCache = updateLabelRefreshCache( ...
    refreshCache, currentObjects, currentTime, triggerConfig)
labels = collectActiveLabels( ...
    currentObjects, triggerConfig.labelHeartbeatExistenceThreshold);
if isempty(refreshCache)
    refreshCache = struct('labels', zeros(2, 0), 'times', zeros(1, 0));
end
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    existingIdx = findLabelColumn(refreshCache.labels, label);
    if existingIdx > 0
        refreshCache.times(existingIdx) = currentTime;
    else
        refreshCache.labels(:, end+1) = label; %#ok<AGROW>
        refreshCache.times(end+1) = currentTime; %#ok<AGROW>
    end
end
end

function labels = collectActiveLabels(objects, threshold)
labels = zeros(2, 0);
if isempty(objects)
    return;
end
active = [objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0;
objects = objects(active);
for objectIdx = 1:numel(objects)
    label = [objects(objectIdx).birthTime; ...
        objects(objectIdx).birthLocation];
    if isempty(labels) || ~any(all(labels == label, 1))
        labels(:, end+1) = label; %#ok<AGROW>
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function lastRefresh = findLabelRefreshTime(refreshCache, label)
lastRefresh = 0;
if isempty(refreshCache) || ~isfield(refreshCache, 'labels')
    return;
end
labelIdx = findLabelColumn(refreshCache.labels, label);
if labelIdx > 0 && numel(refreshCache.times) >= labelIdx
    lastRefresh = refreshCache.times(labelIdx);
end
end

function labelIdx = findLabelColumn(labels, label)
labelIdx = 0;
if isempty(labels)
    return;
end
matches = find(all(labels == label, 1), 1);
if ~isempty(matches)
    labelIdx = matches;
end
end

function [objects, stateEstimate] = finalizeNodeStep( ...
    fusedObjects, stateEstimate, model, currentTime)
objects = fusedObjects;
if isempty(objects)
    stateEstimate.labels{currentTime} = zeros(2, 0);
    stateEstimate.mu{currentTime} = cell(1, 0);
    stateEstimate.Sigma{currentTime} = cell(1, 0);
    return;
end

likely = [objects.r] > model.existenceThreshold;
discarded = objects(~likely & ...
    [objects.trajectoryLength] > model.minimumTrajectoryLength);
stateEstimate.objects = [stateEstimate.objects, discarded];
objects = objects(likely);
if isempty(objects)
    stateEstimate.labels{currentTime} = zeros(2, 0);
    stateEstimate.mu{currentTime} = cell(1, 0);
    stateEstimate.Sigma{currentTime} = cell(1, 0);
    return;
end

[mapCardinality, mapIndices] = lmbMapCardinalityEstimate([objects.r]);
stateEstimate.labels{currentTime} = zeros(2, mapCardinality);
stateEstimate.mu{currentTime} = cell(1, mapCardinality);
stateEstimate.Sigma{currentTime} = cell(1, mapCardinality);
for mapIdx = 1:mapCardinality
    objectIdx = mapIndices(mapIdx);
    stateEstimate.labels{currentTime}(:, mapIdx) = [ ...
        objects(objectIdx).birthTime; objects(objectIdx).birthLocation];
    stateEstimate.mu{currentTime}{mapIdx} = objects(objectIdx).mu{1};
    stateEstimate.Sigma{currentTime}{mapIdx} = ...
        objects(objectIdx).Sigma{1};
end

for objectIdx = 1:numel(objects)
    trajectoryIdx = objects(objectIdx).trajectoryLength + 1;
    objects(objectIdx).trajectoryLength = trajectoryIdx;
    objects(objectIdx).trajectory(:, trajectoryIdx) = ...
        objects(objectIdx).mu{1};
    objects(objectIdx).timestamps(trajectoryIdx) = currentTime;
end
end

function stateEstimate = initializeStateEstimate(simulationLength, emptyObjects)
stateEstimate = struct();
stateEstimate.labels = cell(simulationLength, 1);
stateEstimate.mu = cell(simulationLength, 1);
stateEstimate.Sigma = cell(simulationLength, 1);
stateEstimate.objects = emptyObjects;
end

function diagnostics = initializeDiagnostics( ...
    numberOfSensors, simulationLength, edgeMask, commConfig, triggerConfig)
shape = [numberOfSensors, numberOfSensors, simulationLength];
diagnostics = struct();
diagnostics.edgeMask = edgeMask;
diagnostics.rawEventType = zeros(shape);
diagnostics.eventType = zeros(shape);
diagnostics.attempted = false(shape);
diagnostics.delivered = false(shape);
diagnostics.downgraded = false(shape);
diagnostics.forcedHeavy = false(shape);
diagnostics.labelHeartbeat = false(shape);
diagnostics.labelHeartbeatAge = zeros(shape);
diagnostics.staleFusionUsed = false(shape);
diagnostics.staleFusionAge = inf(shape);
diagnostics.staleFusionWeight = zeros(shape);
diagnostics.topologyActiveEdge = false(shape);
diagnostics.newEdgeActivated = false(shape);
diagnostics.newEdgeHandshake = false(shape);
diagnostics.newEdgeNoHandshake = false(shape);
diagnostics.topologyAlgebraicConnectivity = zeros(1, simulationLength);
diagnostics.topologyUndirectedEdgeCount = zeros(1, simulationLength);
diagnostics.attemptedAlgebraicConnectivity = zeros(1, simulationLength);
diagnostics.deliveredAlgebraicConnectivity = zeros(1, simulationLength);
diagnostics.windowDeliveredAlgebraicConnectivity = zeros(1, simulationLength);
diagnostics.effectiveWeightAlgebraicConnectivity = zeros(1, simulationLength);
diagnostics.perLabelConnectivityViolationRate = zeros(1, simulationLength);
diagnostics.perLabelWindowConnectivityViolationRate = ...
    zeros(1, simulationLength);
diagnostics.perLabelStaleAgeP90 = zeros(1, simulationLength);
diagnostics.perLabelStaleAgeP95 = zeros(1, simulationLength);
diagnostics.utility = zeros(shape);
diagnostics.informationGain = zeros(shape);
diagnostics.linkQuality = zeros(shape);
diagnostics.referenceAge = inf(shape);
diagnostics.payloadScalars = zeros(shape);
diagnostics.attemptedPayloadBytes = zeros(shape);
diagnostics.deliveredPayloadBytes = zeros(shape);
diagnostics.payloadBytes = zeros(shape);
diagnostics.wireMetadataValidated = false(shape);
diagnostics.deliveredEventType = zeros(shape);
diagnostics.fusionWeight = zeros(shape);
diagnostics.fusionMode = zeros(shape);
diagnostics.fusionSelfWeight = zeros(numberOfSensors, simulationLength);
diagnostics.fusionNeighborWeight = zeros(numberOfSensors, simulationLength);
diagnostics.fusionLightWeight = zeros(numberOfSensors, simulationLength);
diagnostics.fusionHeavyWeight = zeros(numberOfSensors, simulationLength);
diagnostics.fusionStaleWeight = zeros(numberOfSensors, simulationLength);
diagnostics.fusionWeightEntropy = zeros(numberOfSensors, simulationLength);
diagnostics.localInnovation = zeros(numberOfSensors, simulationLength);
diagnostics.localAssociationConfidence = ...
    ones(numberOfSensors, simulationLength);
diagnostics.commConfig = commConfig;
diagnostics.triggerConfig = triggerConfig;
end

function diagnostics = summarizeDiagnostics(diagnostics, edgeMask)
timeCount = size(diagnostics.eventType, 3);
if any(diagnostics.topologyActiveEdge(:))
    edgeTimeMask = diagnostics.topologyActiveEdge;
else
    edgeTimeMask = repmat(edgeMask, 1, 1, timeCount);
end
rawEvents = diagnostics.rawEventType(edgeTimeMask);
events = diagnostics.eventType(edgeTimeMask);
attempted = diagnostics.attempted(edgeTimeMask);
delivered = diagnostics.delivered(edgeTimeMask);
staleFusionUsed = diagnostics.staleFusionUsed(edgeTimeMask);
staleFusionAge = diagnostics.staleFusionAge(edgeTimeMask);
diagnostics.summary = struct();
diagnostics.summary.edgeCount = sum(edgeMask(:));
diagnostics.summary.edgeTimeCount = sum(edgeTimeMask(:));
diagnostics.summary.rawTriggerRate = mean(rawEvents > 0);
diagnostics.summary.triggerRate = mean(events > 0);
diagnostics.summary.lightRate = mean(events == 1);
diagnostics.summary.heavyRate = mean(events == 2);
diagnostics.summary.attemptCount = sum(attempted);
diagnostics.summary.deliveryCount = sum(delivered);
diagnostics.summary.deliveryRate = sum(delivered) / max(sum(attempted), 1);
diagnostics.summary.downgradeCount = sum( ...
    diagnostics.downgraded(edgeTimeMask));
diagnostics.summary.labelHeartbeatCount = sum( ...
    diagnostics.labelHeartbeat(edgeTimeMask));
diagnostics.summary.staleFusionCount = sum(staleFusionUsed);
if any(staleFusionUsed)
    diagnostics.summary.meanStaleFusionAge = mean( ...
        staleFusionAge(staleFusionUsed));
else
    diagnostics.summary.meanStaleFusionAge = 0;
end
diagnostics.summary.staleFusionWeight = sum( ...
    diagnostics.staleFusionWeight(edgeTimeMask));
diagnostics.summary.meanAlgebraicConnectivity = mean( ...
    diagnostics.topologyAlgebraicConnectivity);
diagnostics.summary.meanUndirectedEdgeCount = mean( ...
    diagnostics.topologyUndirectedEdgeCount);
diagnostics.summary.meanAttemptedConnectivity = mean( ...
    diagnostics.attemptedAlgebraicConnectivity);
diagnostics.summary.meanDeliveredConnectivity = mean( ...
    diagnostics.deliveredAlgebraicConnectivity);
diagnostics.summary.meanWindowDeliveredConnectivity = mean( ...
    diagnostics.windowDeliveredAlgebraicConnectivity);
diagnostics.summary.meanEffectiveWeightConnectivity = mean( ...
    diagnostics.effectiveWeightAlgebraicConnectivity);
diagnostics.summary.perLabelConnectivityViolationRate = mean( ...
    diagnostics.perLabelConnectivityViolationRate);
diagnostics.summary.perLabelWindowConnectivityViolationRate = mean( ...
    diagnostics.perLabelWindowConnectivityViolationRate);
diagnostics.summary.perLabelStaleAgeP90 = mean( ...
    diagnostics.perLabelStaleAgeP90);
diagnostics.summary.perLabelStaleAgeP95 = mean( ...
    diagnostics.perLabelStaleAgeP95);
newEdges = diagnostics.newEdgeActivated(edgeTimeMask);
diagnostics.summary.newEdgeActivationCount = sum(newEdges);
diagnostics.summary.newEdgeHandshakeCount = sum( ...
    diagnostics.newEdgeHandshake(edgeTimeMask));
if any(newEdges)
    diagnostics.summary.newEdgeNoHandshakeRate = sum( ...
        diagnostics.newEdgeNoHandshake(edgeTimeMask) & newEdges) / ...
        sum(newEdges);
else
    diagnostics.summary.newEdgeNoHandshakeRate = 0;
end
handshakeDeliveryMask = edgeTimeMask & diagnostics.newEdgeHandshake & ...
    diagnostics.delivered;
diagnostics.summary.newEdgeHandshakeBytes = sum( ...
    diagnostics.payloadBytes(handshakeDeliveryMask));
diagnostics.summary.meanSelfWeightMass = mean( ...
    diagnostics.fusionSelfWeight(:));
diagnostics.summary.meanNeighborWeightMass = mean( ...
    diagnostics.fusionNeighborWeight(:));
diagnostics.summary.meanLightWeightMass = mean( ...
    diagnostics.fusionLightWeight(:));
diagnostics.summary.meanHeavyWeightMass = mean( ...
    diagnostics.fusionHeavyWeight(:));
diagnostics.summary.meanStaleWeightMass = mean( ...
    diagnostics.fusionStaleWeight(:));
diagnostics.summary.meanFusionWeightEntropy = mean( ...
    diagnostics.fusionWeightEntropy(:));
diagnostics.summary.payloadScalars = sum( ...
    diagnostics.payloadScalars(edgeTimeMask));
diagnostics.summary.attemptedPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(edgeTimeMask));
diagnostics.summary.deliveredPayloadBytes = sum( ...
    diagnostics.deliveredPayloadBytes(edgeTimeMask));
diagnostics.summary.payloadBytes = ...
    diagnostics.summary.deliveredPayloadBytes;
if diagnostics.summary.attemptedPayloadBytes > 0
    diagnostics.summary.payloadDeliveryRatio = ...
        diagnostics.summary.deliveredPayloadBytes / ...
        diagnostics.summary.attemptedPayloadBytes;
else
    diagnostics.summary.payloadDeliveryRatio = 0;
end
diagnostics.summary.newEdgeHandshakeByteShare = ...
    diagnostics.summary.newEdgeHandshakeBytes / ...
    max(diagnostics.summary.payloadBytes, 1);
diagnostics.summary.topologyChurnRate = computeTopologyChurnRate( ...
    diagnostics.topologyActiveEdge, edgeMask);
diagnostics.summary.meanUtility = mean( ...
    diagnostics.utility(edgeTimeMask));
diagnostics.summary.utilitySamples = ...
    reshape(diagnostics.utility(edgeTimeMask), 1, []);
diagnostics.summary.informationGainSamples = ...
    reshape(diagnostics.informationGain(edgeTimeMask), 1, []);
end

function churnRate = computeTopologyChurnRate(activeEdges, fallbackEdgeMask)
timeCount = size(activeEdges, 3);
if timeCount < 2
    churnRate = 0;
    return;
end
if ~any(activeEdges(:))
    activeEdges = repmat(fallbackEdgeMask, 1, 1, timeCount);
end
previousEdges = activeEdges(:, :, 1:(timeCount - 1));
nextEdges = activeEdges(:, :, 2:timeCount);
changedEdges = xor(previousEdges, nextEdges);
edgeUnion = previousEdges | nextEdges;
churnRate = sum(changedEdges(:)) / max(sum(edgeUnion(:)), 1);
end

function delivered = simulateDelivery( ...
    commConfig, senderIdx, receiverIdx, currentTime, isOutage)
if isOutage
    delivered = false;
    return;
end
if commConfig.forceDelivery
    delivered = true;
    return;
end
pDrop = resolveEdgeDropProbability( ...
    commConfig, senderIdx, receiverIdx);
if isempty(commConfig.linkUniforms)
    uniformValue = rand;
else
    uniformValue = commConfig.linkUniforms( ...
        senderIdx, receiverIdx, currentTime);
end
delivered = uniformValue >= pDrop;
end

function pDrop = resolveEdgeDropProbability(config, senderIdx, receiverIdx)
if ~isempty(config.pDropByEdge)
    pDrop = config.pDropByEdge(senderIdx, receiverIdx);
else
    pDrop = config.pDropBySensor(senderIdx);
end
pDrop = min(max(pDrop, 0), 1);
end

function isOutage = isSensorOutage(schedule, sensorIdx, currentTime)
isOutage = false;
if isempty(schedule)
    return;
end
for idx = 1:numel(schedule)
    if schedule(idx).sensor == sensorIdx && ...
            currentTime >= schedule(idx).start && ...
            currentTime <= schedule(idx).end
        isOutage = true;
        return;
    end
end
end

function history = appendAttempt(history, delivered, windowLength)
history = [reshape(history, 1, []), logical(delivered)];
if numel(history) > windowLength
    history = history(end-windowLength+1:end);
end
end

function rate = computeRecentSuccessRate(history)
if isempty(history)
    rate = NaN;
else
    rate = mean(history);
end
end

function age = resolveReferenceAge(lastSuccessfulTime, currentTime)
if lastSuccessfulTime <= 0
    age = inf;
else
    age = currentTime - lastSuccessfulTime;
end
end

function isScheduled = resolveSampleMask(config, sensorIdx, currentTime)
isScheduled = true;
if ~isempty(config.sensorSampleMask) && ...
        size(config.sensorSampleMask, 1) >= sensorIdx && ...
        size(config.sensorSampleMask, 2) >= currentTime
    isScheduled = config.sensorSampleMask(sensorIdx, currentTime) > 0;
end
end

function neighborMap = buildRangeNeighborMap(model, sensorTrajectories)
numberOfSensors = model.numberOfSensors;
neighborMap = cell(1, numberOfSensors);
range = getField(model, 'sensorCommRange', inf);
positions = zeros(2, numberOfSensors);
if ~isempty(sensorTrajectories)
    for sensorIdx = 1:numberOfSensors
        positions(:, sensorIdx) = sensorTrajectories{sensorIdx}(1:2, 1);
    end
elseif isfield(model, 'sensorInitialStates')
    for sensorIdx = 1:numberOfSensors
        positions(:, sensorIdx) = model.sensorInitialStates{sensorIdx}(1:2);
    end
end
for sensorIdx = 1:numberOfSensors
    delta = positions - positions(:, sensorIdx);
    distances = sqrt(sum(delta .^ 2, 1));
    neighborMap{sensorIdx} = find(distances <= range);
    if isempty(neighborMap{sensorIdx})
        neighborMap{sensorIdx} = sensorIdx;
    end
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
