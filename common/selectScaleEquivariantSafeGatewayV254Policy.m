function [adjacency, details] = ...
        selectScaleEquivariantSafeGatewayV254Policy(context, model)
% SELECTSCALEEQUIVARIANTSAFEGATEWAYV254POLICY Score, project, abstain.

started = tic;
protocol = getScaleEquivariantSafeGatewayV254Protocol();
requiredModel = {'activationThreshold', 'calibrationMargin', ...
    'switchingPenalty', 'tieTolerance'};
if ~isstruct(model) || ~isscalar(model) || ...
        ~all(isfield(model, requiredModel)) || ...
        ~isscalar(model.activationThreshold) || ...
        ~isscalar(model.calibrationMargin) || ...
        ~isscalar(model.switchingPenalty) || ...
        ~isscalar(model.tieTolerance) || ...
        any(~isfinite([model.activationThreshold, ...
            model.calibrationMargin, model.switchingPenalty, ...
            model.tieTolerance])) || ...
        any([model.activationThreshold, model.calibrationMargin, ...
            model.switchingPenalty, model.tieTolerance] < 0)
    error('ScaleEquivariantGatewayV254:InvalidPolicyModel', ...
        'The V254 policy model lacks finite nonnegative gate parameters.');
end

[referenceAdjacency, referenceDetails] = ...
    selectCausalGatewayEmbeddingV250Policy(context, []);
referenceAssignment = referenceDetails.referenceGatewayAssignment;
[edgeValues, scoreDetails] = ...
    scoreScaleEquivariantGatewayEdgesV254( ...
        context, referenceAssignment, model);
[projectedAssignment, projection] = ...
    projectScaleEquivariantGatewayAssignmentV254( ...
        context, edgeValues, referenceAssignment, struct( ...
            'switchingPenalty', model.switchingPenalty, ...
            'tieTolerance', model.tieTolerance));

requiredAdvantage = model.activationThreshold + model.calibrationMargin;
candidateDiffers = ~isequal( ...
    projection.candidateAssignment, referenceAssignment);
applyCandidate = ~projection.projectionFallbackUsed && ...
    candidateDiffers && ...
    projection.candidatePredictedAdvantage > requiredAdvantage;
if applyCandidate
    requestedAssignment = projectedAssignment;
    fallbackReason = '';
elseif projection.projectionFallbackUsed
    requestedAssignment = referenceAssignment;
    fallbackReason = projection.projectionFallbackReason;
elseif ~candidateDiffers
    requestedAssignment = referenceAssignment;
    fallbackReason = 'projected-reference-is-optimal';
else
    requestedAssignment = referenceAssignment;
    fallbackReason = 'predicted-advantage-below-calibrated-threshold';
end

[adjacency, applied] = selectCausalGatewayEmbeddingV250Policy( ...
    context, requestedAssignment);
if ~applied.requestedGatewayAssignmentApplied
    error('ScaleEquivariantGatewayV254:SafeProjectionRejected', ...
        'The V254 exact assignment was rejected by the ordinary route projection.');
end

details = applied;
details.contractVersion = ...
    'scale-equivariant-safe-gateway-v254-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'learned-edge-value-exact-gateway-projection';
details.backboneMode = details.mode;
details.referenceAdjacency = referenceAdjacency;
details.referenceGatewayAssignment = referenceAssignment;
details.projectedGatewayAssignment = projection.candidateAssignment;
details.requestedGatewayAssignment = requestedAssignment;
details.appliedGatewayAssignment = applied.appliedGatewayAssignment;
details.learnedGatewayApplied = applyCandidate;
details.gatewayFallbackUsed = ~applyCandidate;
details.gatewayFallbackReason = fallbackReason;
details.predictedCandidateAdvantage = ...
    projection.candidatePredictedAdvantage;
details.requiredPredictedAdvantage = requiredAdvantage;
details.activationThreshold = model.activationThreshold;
details.calibrationMargin = model.calibrationMargin;
details.edgeScoreDetails = scoreDetails;
details.gatewayProjectionDetails = projection;
details.objective = projection.candidatePredictedAdvantage;
details.taskAdvantage = projection.candidatePredictedAdvantage;
details.selectionSeconds = toc(started);
details.referenceFallbackUsed = ~applyCandidate;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.realizedDeliveryUniformsUsed = false;
details.trackingOutcomeScored = false;
schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'scale-equivariant-safe-gateway-v254-schedule-v1';
schedule.phase = 'v254-learned-gateway-projection';
schedule.referenceGatewayAssignment = referenceAssignment;
schedule.projectedGatewayAssignment = projection.candidateAssignment;
schedule.appliedGatewayAssignment = details.appliedGatewayAssignment;
schedule.learnedGatewayApplied = applyCandidate;
schedule.gatewayFallbackUsed = ~applyCandidate;
schedule.gatewayFallbackReason = fallbackReason;
schedule.predictedCandidateAdvantage = ...
    projection.candidatePredictedAdvantage;
schedule.requiredPredictedAdvantage = requiredAdvantage;
details.scheduleCertificate = schedule;
end
