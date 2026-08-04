function cost = ...
    estimateFormationB4V48ScheduleControlPlaneCost( ...
        nodeCount, formationCount, windowCount)
% ESTIMATEFORMATIONB4V48SCHEDULECONTROLPLANECOST
% Idealized lower bound for one full-plan tree dissemination plus one pull
% per residual opportunity. Shared route-discovery traffic is not silently
% assigned to V48; it must be reported for every compared arm.

if nargin < 3
    windowCount = 1;
end
values = [nodeCount, formationCount, windowCount];
if ~isa(values, 'double') || ~isreal(values) || ...
        any(~isfinite(values)) || any(values ~= round(values)) || ...
        nodeCount < 2 || formationCount < 1 || ...
        formationCount > nodeCount || windowCount < 1
    error('FormationB4V48ControlPlane:InvalidInput', ...
        'Node, formation, and window counts must be positive integers.');
end

protocol = getFormationB4V48ControlPlaneProtocol();
headerBytes = protocol.wireHeaderBytes;
phaseBitsPerSensor = protocol.phaseBitsPerSensor;
packedPhaseBytes = ceil(phaseBitsPerSensor * nodeCount / 8);
packetBytes = headerBytes + ...
    protocol.globalPlanFixedPayloadBytes + packedPhaseBytes;
spanningTreeDirectedTransmissions = nodeCount - 1;
planBytesPerWindow = ...
    packetBytes * spanningTreeDirectedTransmissions;
pullMessagesPerWindow = nodeCount;
pullBytesPerWindow = ...
    pullMessagesPerWindow * protocol.residualPullWireBytes;
attemptedBytesPerWindow = planBytesPerWindow + pullBytesPerWindow;

payload = struct();
payload.contractVersion = ...
    'formation-b4-v48-schedule-control-plane-cost-v1';
payload.nodeCount = nodeCount;
payload.formationCount = formationCount;
payload.windowCount = windowCount;
payload.protocolId = protocol.id;
payload.protocolCanonicalSha256 = protocol.canonicalSha256;
payload.windowLength = protocol.windowLength;
payload.headerBytes = headerBytes;
payload.globalPlanFixedPayloadBytes = ...
    protocol.globalPlanFixedPayloadBytes;
payload.headerDefinition = ...
    'version-type-flags-epoch-uids-sequence-fragments-length-crcs';
payload.phaseBitsPerSensor = phaseBitsPerSensor;
payload.packedPhaseBytes = packedPhaseBytes;
payload.globalPlanWireBytes = packetBytes;
payload.residualPullWireBytes = protocol.residualPullWireBytes;
payload.disseminationModel = ...
    ['full-plan-on-every-edge-of-one-rooted-sensor-', ...
     'arborescence-plus-one-pull-per-residual-opportunity-', ...
     'excluding-required-two-phase-commit'];
payload.spanningTreeDirectedTransmissionsPerWindow = ...
    spanningTreeDirectedTransmissions;
payload.attemptedPlanMessagesPerWindow = ...
    spanningTreeDirectedTransmissions;
payload.attemptedPlanBytesPerWindow = planBytesPerWindow;
payload.attemptedPullMessagesPerWindow = pullMessagesPerWindow;
payload.attemptedPullBytesPerWindow = pullBytesPerWindow;
payload.attemptedControlMessagesPerWindow = ...
    spanningTreeDirectedTransmissions + pullMessagesPerWindow;
payload.attemptedControlMessages = ...
    payload.attemptedControlMessagesPerWindow * windowCount;
payload.attemptedControlBytesPerWindow = attemptedBytesPerWindow;
payload.attemptedControlBytes = ...
    attemptedBytesPerWindow * windowCount;
payload.averageAttemptedControlBytesPerTrackingStep = ...
    attemptedBytesPerWindow / payload.windowLength;
payload.posteriorSummaryCollectionMessages = 0;
payload.posteriorSummaryCollectionBytes = 0;
payload.deliveryAcknowledgmentCollectionMessages = 0;
payload.deliveryAcknowledgmentCollectionBytes = 0;
payload.posteriorSummaryRequired = false;
payload.deliveryAcknowledgmentRequired = false;
payload.globalTwoPhaseCommitRequired = ...
    protocol.globalTwoPhaseCommitRequired;
payload.globalTwoPhaseCommitBytesIncluded = false;
payload.splitPlanExecutionForbidden = ...
    protocol.mixedPlannedAndFallbackExecutionForbidden;
payload.sharedRouteLayerCurrentGraphRequired = true;
payload.sharedRouteLayerCurrentReliabilityRequired = true;
payload.sharedRouteLayerCostIncluded = false;
payload.sharedRouteLayerMustBeReportedForEveryArm = true;
payload.reliableControlDeliveryAssumed = true;
payload.controlLossAndLatencyModeled = false;
payload.idealizedIncrementalLowerBoundOnly = true;
payload.incrementalScheduleCostClaimAllowed = false;
payload.sameTotalByteComparisonClaimAllowed = false;
payload.endToEndNetworkCostClaimAllowed = false;
payload.developmentEvidenceOnly = true;
payload.canonicalSha256 = computeCanonicalValueSha256(payload);
cost = payload;
end
