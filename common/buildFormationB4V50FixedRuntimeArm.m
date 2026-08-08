function [adjacency, details] = ...
    buildFormationB4V50FixedRuntimeArm(context, armId)
% BUILDFORMATIONB4V50FIXEDRUNTIMEARM Execute one posterior-aware V50 page.

timerId = tic;
protocol = getFormationB4V50RuntimeProtocol();
if nargin ~= 2 || ~strcmp(armId, protocol.candidateArmId)
    error('FormationB4V50Runtime:InvalidArm', ...
        'Only the V50 posterior-aware cycle arm is accepted.');
end
validateContext(context, protocol);

routeContext = buildFormationB4V49GraphOnlyRouteContext(context);
nodeCount = numel(context.localPosteriorBySensor);
incumbentContext = buildIncumbentContext( ...
    context, routeContext, nodeCount);
[incumbentAdjacency, incumbentDetails] = ...
    buildFormationB4V46FixedRuntimeArm( ...
        incumbentContext, protocol.referenceArmId);
incumbent = struct( ...
    'dominantAdjacency', logical(incumbentDetails.dominantAdjacency), ...
    'residualAdjacency', logical(incumbentDetails.residualAdjacency), ...
    'referenceAdjacency', logical(incumbentDetails.referenceAdjacency), ...
    'referenceFusionWeights', ...
        incumbentDetails.referenceFusionWeights);

phase = mod(context.currentTime - 1, protocol.period) + 1;
selectionPerformed = phase == 1;
selected = incumbent;
selectionDetails = struct([]);
selectedIsCycle = false;
fallbackReason = 'not-a-burst-page';
objective = 0;
if selectionPerformed
    [selected, selectionDetails] = ...
        selectFormationB4V50PosteriorAwareCycleReference( ...
            context, incumbent, protocol);
    selectedIsCycle = selectionDetails. ...
        selectedIsCyclePreservingReference;
    fallbackReason = selectionDetails.selectedFallbackReason;
    objective = selectionDetails.selectedPosteriorObjective;
end

[adjacency, fusionWeights] = materializePage( ...
    selected, phase, protocol);
validatePage(adjacency, fusionWeights, selected, incumbent, ...
    context, phase, protocol);

schedule = struct();
schedule.contractVersion = ...
    'formation-b4-v50-runtime-schedule-v1';
schedule.currentTime = context.currentTime;
schedule.currentAbsolutePhase = phase;
schedule.period = protocol.period;
schedule.cycleSelectionPerformed = selectionPerformed;
schedule.cycleSelected = selectedIsCycle;
schedule.referenceFallbackUsed = ...
    selectionPerformed && ~selectedIsCycle;
schedule.fallbackReason = fallbackReason;
schedule.selectedPosteriorObjective = objective;
schedule.selectionDetails = selectionDetails;
schedule.currentScheduledDirectedEdgeCount = nnz(adjacency);
schedule.expectedScheduledDirectedEdgeCount = ...
    nodeCount * (1 + double(phase == 1));
schedule.scheduledDirectedEdgeCountsByPhase = ...
    nodeCount * [2, 1, 1, 1];
schedule.sameScheduledDirectedEdgeCountVsV46 = true;
schedule.posteriorUsedForRoutingOnly = selectionPerformed;
schedule.truthUsed = false;
schedule.futureOutcomeUsed = false;

formationUids = reshape( ...
    context.formationPhysicalUidsBySensor, 1, []);
crossMask = formationUids(:) ~= formationUids(:)';
details = struct();
details.contractVersion = ...
    'formation-b4-v50-fixed-runtime-policy-v1';
details.mode = 'formation-b4-v50-posterior-aware-cycle-runtime';
details.armId = protocol.candidateArmId;
details.armOrdinal = 2;
details.actionName = protocol.candidateArmId;
details.currentTime = context.currentTime;
details.currentAbsolutePhase = phase;
details.period = protocol.period;
details.objective = objective;
details.objectiveDefinition = ...
    'posterior-residual-utility-formation-tail-improvement-vs-v46';
details.objectiveSource = ...
    'current-observable-lmb-posterior-and-link-page';
details.candidateIndex = NaN;
if selectionPerformed && selectedIsCycle
    details.candidateIndex = ...
        selectionDetails.selectedCycleCandidateIndex;
end
details.fusionWeightMatrix = fusionWeights;
details.referenceFusionWeights = selected.referenceFusionWeights;
details.dominantAdjacency = logical(selected.dominantAdjacency);
details.residualAdjacency = logical(selected.residualAdjacency);
details.referenceAdjacency = logical(selected.referenceAdjacency);
details.scheduleCertificate = schedule;
details.protocolId = protocol.id;
details.observableInputContractVersion = ...
    protocol.observableContractVersion;
details.referenceFallbackUsed = schedule.referenceFallbackUsed;
details.fullV43RouteCompositionPassed = true;
details.routeReceivesCurrentPageOnly = true;
details.projectionReceivesCurrentPageOnly = true;
details.observableContextOnly = true;
details.posteriorUsed = selectionPerformed;
details.currentPosteriorUsed = selectionPerformed;
details.posteriorUsedForRoutingOnly = selectionPerformed;
details.fullPosteriorFusionUnchanged = true;
details.posteriorUsedByProjection = false;
details.truthUsedByProjection = false;
details.measurementUsedByProjection = false;
details.futurePageUsedByProjection = false;
details.realizedDeliveryUniformsUsedByProjection = false;
details.currentLinkReliabilityUsed = selectionPerformed;
details.currentPhysicalActionSetUsed = true;
details.truthUsed = false;
details.groundTruthUsed = false;
details.measurementUsed = false;
details.futurePageUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.pastSuccessfulDeliveryUsed = false;
details.crossResidualCount = nnz( ...
    selected.residualAdjacency & crossMask);
details.localResidualCount = nnz( ...
    selected.residualAdjacency & ~crossMask);
details.currentScheduledDirectedEdgeCount = nnz(adjacency);
details.referenceScheduledDirectedEdgeCount = 2 * nodeCount;
details.scheduledDirectedEdgeSavingFraction = ...
    (2 * nodeCount - nnz(adjacency)) / (2 * nodeCount);
details.currentMessageCount = nnz(adjacency);
details.referenceMessageCount = 2 * nodeCount;
details.messageSavingFraction = ...
    details.scheduledDirectedEdgeSavingFraction;
details.sameScheduledDirectedEdgeCountVsV46 = true;
details.routeDisseminationImplemented = false;
details.atomicCommitImplemented = false;
details.sameTotalByteClaimAllowed = false;
details.trackingOutcomeScored = false;
details.validationClaimAllowed = false;
details.developmentEvidenceOnly = true;
details.incumbentRuntimeAdjacency = logical(incumbentAdjacency);
details.selectionSeconds = toc(timerId);
end

function contextOut = buildIncumbentContext( ...
        context, routeContext, nodeCount)
contextOut = struct();
contextOut.localPosteriorBySensor = cell(1, nodeCount);
contextOut.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', routeContext.sensorGroupIds)));
contextOut.baseAdjacency = logical(context.baseAdjacency);
contextOut.physicalAdjacency = logical(routeContext.physicalAdjacency);
contextOut.positions = routeContext.positions;
contextOut.sensorPhysicalUids = routeContext.sensorPhysicalUids;
contextOut.formationPhysicalUidsBySensor = ...
    routeContext.formationPhysicalUidsBySensor;
contextOut.physicalIdentityRegistryCanonicalSha256 = ...
    context.physicalIdentityRegistryCanonicalSha256;
contextOut.commConfig = struct('pDropByEdge', routeContext.pDropByEdge);
contextOut.currentTime = routeContext.currentTime;
contextOut.directedMessageBudget = 2 * nodeCount;
contextOut.triggerConfig = struct( ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount);
contextOut.observableInputContract = context.observableInputContract;
end

function [adjacency, weights] = materializePage(reference, phase, protocol)
nodeCount = size(reference.referenceAdjacency, 1);
residual = false(nodeCount);
if phase == 1
    residual = logical(reference.residualAdjacency);
end
adjacency = logical(reference.dominantAdjacency) | residual;
weights = zeros(nodeCount);
weights(reference.dominantAdjacency) = protocol.dominantWeight;
weights(residual) = protocol.activeResidualWeight;
weights(1:nodeCount+1:end) = 1 - sum(weights, 2)';
end

function validateContext(context, protocol)
required = {'localPosteriorBySensor', 'model', 'baseAdjacency', ...
    'physicalAdjacency', 'positions', 'sensorPhysicalUids', ...
    'formationPhysicalUidsBySensor', ...
    'physicalIdentityRegistryCanonicalSha256', 'commConfig', ...
    'triggerConfig', 'observableInputContract', 'currentTime'};
if ~isstruct(context) || ~isscalar(context) || ...
        ~all(isfield(context, required)) || ...
        ~iscell(context.localPosteriorBySensor) || ...
        ~isstruct(context.observableInputContract) || ...
        ~context.observableInputContract.passed || ...
        ~strcmp(context.observableInputContract.contractVersion, ...
            protocol.observableContractVersion)
    error('FormationB4V50Runtime:InvalidContext', ...
        'The V50 observable runtime context is incomplete.');
end
end

function validatePage(adjacency, weights, selected, incumbent, ...
        context, phase, protocol)
nodeCount = numel(context.sensorPhysicalUids);
expectedCount = nodeCount * (1 + double(phase == 1));
if ~isequal(selected.dominantAdjacency, ...
        incumbent.dominantAdjacency) || ...
        nnz(selected.referenceAdjacency) ~= 2 * nodeCount || ...
        nnz(adjacency) ~= expectedCount || ...
        any(adjacency(:) & ~logical(context.physicalAdjacency(:))) || ...
        any(abs(sum(weights, 2) - 1) > 1e-12) || ...
        any(weights(:) < -1e-12)
    error('FormationB4V50Runtime:InvalidAction', ...
        'The V50 page violates the fixed route or budget contract.');
end
end
