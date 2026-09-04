function [adjacency, details] = ...
        selectRiskTriggeredFormationResidualPulseV260Policy( ...
            context, pulseWeight)
% SELECTRISKTRIGGEREDFORMATIONRESIDUALPULSEV260POLICY Pulse one formation.

protocol = getRiskTriggeredFormationResidualPulseV260Protocol();
if nargin < 2 || ~isscalar(pulseWeight) || ...
        ~ismember(pulseWeight, protocol.pulseWeights)
    error('RiskTriggeredPulseV260:InvalidPulseWeight', ...
        'V260 accepts only its two registered pulse weights.');
end
[minimumAdjacency, minimum] = ...
    selectCausalMinimumFormationBackboneV242Policy(context);
nodeCount = size(minimumAdjacency, 1);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
summary = summarizeFormationLmbRiskModesV259( ...
    context.localPosteriorBySensor, groupIds, context.model);
localization = reshape( ...
    summary.localizationTailRiskByFormation, 1, []);
formationMedian = median(localization);
relative = localization / max(formationMedian, eps);
eligible = localization >= ...
    protocol.localizationAbsoluteThreshold & ...
    relative >= protocol.localizationRelativeThreshold;
selectedFormationPosition = 0;
if any(eligible)
    candidates = find(eligible);
    [~, localIdx] = max(localization(candidates));
    selectedFormationPosition = candidates(localIdx);
end

minimumMessageCount = nodeCount + ...
    2 * (numel(unique(groupIds)) - 1);
previousPulse = false;
if isfield(context, 'previousAdjacencyHistory') && ...
        ~isempty(context.previousAdjacencyHistory)
    previousPulse = nnz(context.previousAdjacencyHistory(:, :, end)) > ...
        minimumMessageCount;
end
pulseRequested = selectedFormationPosition > 0;
pulseApplied = pulseRequested && ~previousPulse;
selectedResidual = false(nodeCount);
weights = minimum.fusionWeightMatrix;
if pulseApplied
    formationId = summary.formationIds(selectedFormationPosition);
    members = find(groupIds == formationId);
    selectedResidual(members, members) = ...
        minimum.localResidualAdjacency(members, members);
    [receivers, senders] = find(selectedResidual);
    if isempty(receivers) || ...
            numel(receivers) > protocol.maximumFormationResidualEdges || ...
            numel(unique(receivers)) ~= numel(receivers)
        error('RiskTriggeredPulseV260:InvalidResidualBundle', ...
            'The selected formation residual bundle is malformed.');
    end
    for edgeIdx = 1:numel(receivers)
        receiver = receivers(edgeIdx);
        sender = senders(edgeIdx);
        if weights(receiver, receiver) < pulseWeight - 1e-12 || ...
                weights(receiver, sender) > 0
            error('RiskTriggeredPulseV260:InsufficientSelfMass', ...
                'A pulse receiver cannot fund the registered weight.');
        end
        weights(receiver, receiver) = ...
            weights(receiver, receiver) - pulseWeight;
        weights(receiver, sender) = pulseWeight;
    end
end
adjacency = logical(minimumAdjacency | selectedResidual);
positiveAllowed = adjacency | logical(eye(nodeCount));
hardGate = ...
    nnz(adjacency) >= minimumMessageCount && ...
    nnz(adjacency) <= minimumMessageCount + ...
        protocol.maximumFormationResidualEdges && ...
    ~any(adjacency(:) & ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(adjacency) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('RiskTriggeredPulseV260:ProjectionFailed', ...
        'The V260 residual pulse violates a hard graph or weight gate.');
end
if isfield(context, 'directedMessageBudget') && ...
        nnz(adjacency) > context.directedMessageBudget
    error('RiskTriggeredPulseV260:BudgetExceeded', ...
        'The V260 pulse exceeds its registered message budget.');
end

details = minimum;
details.contractVersion = ...
    'risk-triggered-formation-residual-pulse-v260-policy-v1';
details.protocolId = protocol.id;
details.armId = sprintf('%s-w%02d', ...
    protocol.armIdPrefix, round(100 * pulseWeight));
details.mode = 'risk-triggered-formation-local-residual-pulse';
details.backboneMode = details.mode;
details.fusionWeightMatrix = weights;
details.addedResidualAdjacency = selectedResidual;
details.localizationTailRiskByFormation = localization;
details.localizationRelativeRiskByFormation = relative;
details.localizationRiskFormationMedian = formationMedian;
details.localizationRiskGateByFormation = eligible;
details.selectedLocalizationFormationPosition = ...
    selectedFormationPosition;
details.selectedLocalizationFormationId = 0;
if selectedFormationPosition > 0
    details.selectedLocalizationFormationId = ...
        summary.formationIds(selectedFormationPosition);
end
details.pulseWeight = pulseWeight;
details.pulseRequested = pulseRequested;
details.previousPagePulseDetected = previousPulse;
details.pulseApplied = pulseApplied;
details.selectedResidualCount = nnz(selectedResidual);
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = minimumMessageCount;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = false;
details.currentNetworkPosteriorSynopsisUsed = true;
details.distributedControlSynopsisCostIncluded = false;
details.centralizedDevelopmentController = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.trackingOutcomeScored = false;
schedule = minimum.scheduleCertificate;
schedule.contractVersion = ...
    'risk-triggered-formation-residual-pulse-v260-schedule-v1';
schedule.phase = conditionalValue(pulseApplied, ...
    'v260-local-residual-pulse', 'v260-minimum-backbone');
schedule.localizationTailRiskByFormation = localization;
schedule.localizationRelativeRiskByFormation = relative;
schedule.localizationRiskGateByFormation = eligible;
schedule.selectedLocalizationFormationId = ...
    details.selectedLocalizationFormationId;
schedule.pulseWeight = pulseWeight;
schedule.pulseRequested = pulseRequested;
schedule.previousPagePulseDetected = previousPulse;
schedule.pulseApplied = pulseApplied;
schedule.selectedResidualCount = nnz(selectedResidual);
schedule.currentMessageCount = nnz(adjacency);
schedule.centralizedDevelopmentController = true;
schedule.distributedControlSynopsisCostIncluded = false;
details.scheduleCertificate = schedule;
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; ...
        %#ok<AGROW>
end
passed = all(visited);
end

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
end
