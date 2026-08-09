function bank = buildTrackingAlignedLayeredLabelActionBank(context, options)
% BUILDTRACKINGALIGNEDLAYEREDLABELACTIONBANK V62 positive-control modes.

if nargin < 2 || isempty(options)
    options = struct();
end
if ~isstruct(context) || ~isfield(context, 'model')
    error('LayeredLabelBank:InvalidContext', ...
        'A current observable topology context is required.');
end
protocol = getTrackingAlignedLayeredLabelHeadroomV62Protocol();
policyProtocol = getFormationIsolateReconnectProbeProtocol();
policyOptions = struct( ...
    'dominantWeight', policyProtocol.dominantWeight, ...
    'residualWeight', policyProtocol.residualWeight);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', policyOptions);
referenceWeights = referenceDetails.fusionWeightMatrix;

actionNames = { ...
    'reference-full-payload', ...
    'scheduled-control-only', ...
    'scheduled-sender-supported-only', ...
    'scheduled-sender-supported-or-r50', ...
    'scheduled-receiver-need-aware'};
payloadModes = { ...
    'reference-full-payload', ...
    'control-only', ...
    'sender-supported-only', ...
    'sender-supported-or-high-existence', ...
    'receiver-need-aware'};
actionCount = numel(actionNames);

bank = struct();
bank.contractVersion = ...
    'tracking-aligned-layered-label-action-bank-v62-v1';
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionPayloadModes = payloadModes;
bank.actionAdjacency = repmat( ...
    logical(referenceAdjacency), 1, 1, actionCount);
bank.actionFusionWeights = repmat( ...
    referenceWeights, 1, 1, actionCount);
bank.actionPosteriorObjective = [0, 1, 2, 3, 4];
bank.actionPosteriorProxyAllowed = true(1, actionCount);
bank.actionWithinReferencePayload = true(1, actionCount);
bank.actionPredictedNetSavingBytes = NaN(1, actionCount);
bank.formationConditionedTimes = protocol.scheduleTimes;
bank.formationConditionedSchedule = protocol.formationSchedule;
bank.positiveSupportThreshold = getField(options, ...
    'positiveSupportThreshold', protocol.positiveSupportThreshold);
bank.highExistenceThreshold = getField(options, ...
    'highExistenceThreshold', protocol.highExistenceThreshold);
bank.referenceAdjacency = logical(referenceAdjacency);
bank.referenceFusionWeights = referenceWeights;
bank.referencePolicyDetails = referenceDetails;
bank.truthUsed = false;
bank.futureMeasurementsUsed = false;
bank.futureOutcomesUsed = false;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
