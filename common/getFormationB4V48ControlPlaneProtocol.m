function protocol = getFormationB4V48ControlPlaneProtocol()
% GETFORMATIONB4V48CONTROLPLANEPROTOCOL Frozen wire-level MVP contract.

payload = struct();
payload.id = 'formation-b4-v48-control-plane-development-v1';
payload.contractVersion = ...
    'formation-b4-v48-control-plane-protocol-v1';
payload.windowLength = 4;
payload.phaseBitsPerSensor = 2;
payload.wireHeaderBytes = 32;
payload.globalPlanFixedPayloadBytes = 24;
payload.residualPullPayloadBytes = 12;
payload.residualPullWireBytes = ...
    payload.wireHeaderBytes + payload.residualPullPayloadBytes;
payload.globalPlanWireBytesRule = ...
    'header32-plus-fixed24-plus-ceil-two-bits-per-sensor';
payload.controlTreeRule = ...
    'uid-canonical-rooted-arborescence-on-previous-page-reference';
payload.controlTreeFullPlanForwardedOnEveryEdge = true;
payload.controlPlanSnapshotLag = 1;
payload.firstWindowFallbackMode = ...
    'registered-absolute-phase-synchronized-b4';
payload.missingGlobalCommitFallbackMode = ...
    'registered-absolute-phase-synchronized-b4';
payload.receiverLocalFallbackAllowed = false;
payload.globalTwoPhaseCommitRequired = true;
payload.globalTwoPhaseCommitImplemented = false;
payload.mixedPlannedAndFallbackExecutionForbidden = true;
payload.posteriorAttemptRule = ...
    ['global-plan-committed-and-receiver-active-and-pull-delivered-', ...
     'and-current-physical-support'];
payload.planDeliveryAndGlobalCommitRequiredForSelectedPhase = true;
payload.residualPullRequiredForPosteriorAttempt = true;
payload.posteriorSummaryMessageDefined = false;
payload.deliveryAcknowledgmentMessageDefined = false;
payload.currentDeliveryDrawVisibleToScheduler = false;
payload.controlAndPosteriorRandomStreamsIndependentRequired = true;
payload.physicalUidKeyedPairedRandomStreamsRequired = true;
payload.controlLossModeled = false;
payload.controlLatencyModeled = false;
payload.controlTreeImplemented = false;
payload.executedAdjacencyBoundToControlDelivery = false;
payload.sameTotalByteComparisonClaimAllowed = false;
payload.endToEndNetworkCostClaimAllowed = false;
payload.developmentEvidenceOnly = true;
payload.canonicalSha256 = computeCanonicalValueSha256(payload);
protocol = payload;
end
