function analysis = buildReceiverFusionAlignedProjectionV73( ...
        context, groupIds, options)
% BUILDRECEIVERFUSIONALIGNEDPROJECTIONV73 Shared source-only V73 pipeline.

if nargin < 3 || isempty(options)
    options = struct();
end
v70 = getReceiverDomainNormalizedOpportunityV70Protocol();
v71 = getReceiverDomainTransportProjectionV71Protocol();
fusionConfig = getField(options, ...
    'fusionConfig', buildMixtureAwareKlaReferenceConfig());
[~, control] = computeFormationRoutingLeverageSignature( ...
    context, groupIds, struct('referenceMode', 'current-physical-tree'));
base = computeNetworkAdditiveFormationRiskV65( ...
    control, context, groupIds, v70.positiveSupportThreshold);
influence = computeInfluenceAwareDecisionBreadthV66( ...
    control, context, groupIds, struct( ...
        'positiveSupportThreshold', v70.positiveSupportThreshold, ...
        'horizonSteps', v70.horizonSteps));
transportOptions = getSignedMergeSplitOpportunityV69Protocol();
transportOptions.fusionConfig = fusionConfig;
transport = computeAlternativeTransportHeadroomV68( ...
    context, control, groupIds, transportOptions);
local = computeReceiverDomainNormalizedOpportunityV70( ...
    base, influence, control, transport, groupIds, v70);
projection = buildReceiverDomainTransportProjectionV71( ...
    context, control, transport, local, groupIds, v71);

analysis = struct();
analysis.contractVersion = ...
    'receiver-fusion-aligned-projection-v73-analysis-v1';
analysis.control = control;
analysis.base = base;
analysis.influence = influence;
analysis.transport = transport;
analysis.local = local;
analysis.projection = projection;
analysis.receiverFusionMode = transport.receiverFusionMode;
analysis.truthUsed = false;
analysis.futureMeasurementsUsed = false;
analysis.futureOutcomesUsed = false;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
