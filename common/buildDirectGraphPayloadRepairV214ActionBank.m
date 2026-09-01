function bank = buildDirectGraphPayloadRepairV214ActionBank( ...
        context, groupIds, options)
% BUILDDIRECTGRAPHPAYLOADREPAIRV214ACTIONBANK Cheap single-page actions.
%
% Every formation receives one causal withholding candidate.  The route
% and fusion weights remain the current-physical full-payload reference;
% only cross-formation posterior consumption for the selected receiver
% formation is suspended for the current page.  Following pages return to
% the reference, so the H=3 teacher measures delayed value without letting
% an action silently become a persistent disconnection.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getDirectGraphPayloadRepairControllerV214Protocol();
horizonSteps = round(getField(options, ...
    'horizonSteps', protocol.horizonSteps));
if ~isstruct(context) || ~isscalar(context) || ...
        ~iscell(context.localPosteriorBySensor) || ...
        ~isnumeric(groupIds) || ...
        numel(groupIds) ~= numel(context.localPosteriorBySensor) || ...
        ~isscalar(horizonSteps) || ~isfinite(horizonSteps) || ...
        horizonSteps ~= protocol.horizonSteps
    error('DirectGraphPayloadRepairV214:InvalidActionBankInput', ...
        'V214 requires one truth-free H=3 observable state.');
end
groupIds = reshape(groupIds, 1, []);
[referenceAdjacency, referenceDetails] = ...
    selectDirectGraphPayloadRepairV214ReferencePolicy(context);
formationIds = reshape( ...
    referenceDetails.hierarchicalFormationIds, 1, []);
formationNeed = reshape( ...
    referenceDetails.hierarchicalFormationNeedByFormation, 1, []);
if numel(formationIds) ~= numel(formationNeed) || ...
        ~isequal(formationIds, unique(groupIds, 'stable')) || ...
        any(~isfinite(formationNeed))
    error('DirectGraphPayloadRepairV214:FormationGraphDrift', ...
        'The observable formation graph changed during bank creation.');
end

requestedFormationIds = reshape(getField( ...
    options, 'formationIds', formationIds), 1, []);
if isempty(requestedFormationIds) || ...
        any(~ismember(requestedFormationIds, formationIds)) || ...
        numel(unique(requestedFormationIds)) ~= ...
            numel(requestedFormationIds)
    error('DirectGraphPayloadRepairV214:InvalidFormationSubset', ...
        'The requested withholding formations are invalid.');
end

actionCount = 1 + numel(requestedFormationIds);
actionNames = cell(1, actionCount);
actionNames{1} = 'reference-full-payload';
payloadModes = [{'reference-full-payload'}, repmat( ...
    {protocol.actions.withholdingPayloadMode}, ...
    1, actionCount - 1)];
schedules = cell(1, actionCount);
schedules{1} = repmat({zeros(1, 0)}, 1, horizonSteps);
actionFormationIds = cell(1, actionCount);
actionFormationIds{1} = zeros(1, 0);
actionObjective = zeros(1, actionCount);
for actionIdx = 2:actionCount
    formationId = requestedFormationIds(actionIdx - 1);
    schedule = repmat({zeros(1, 0)}, 1, horizonSteps);
    schedule{1} = formationId;
    schedules{actionIdx} = schedule;
    actionFormationIds{actionIdx} = formationId;
    actionNames{actionIdx} = sprintf( ...
        'v214-withhold-full-posterior-f%d-one-page', formationId);
    needIdx = find(formationIds == formationId, 1);
    actionObjective(actionIdx) = -formationNeed(needIdx);
end

referenceWeights = referenceDetails.fusionWeightMatrix;
layered = getTrackingAlignedLayeredLabelHeadroomV62Protocol();
online = getOnlinePositiveNetAddressablePayloadV99Protocol();
bank = struct();
bank.contractVersion = ...
    'direct-graph-payload-repair-action-bank-v214-v1';
bank.bankVariant = 'direct-graph-payload-repair-v214';
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionPayloadModes = payloadModes;
bank.actionAdjacency = repmat( ...
    logical(referenceAdjacency), 1, 1, actionCount);
bank.actionFusionWeights = repmat( ...
    referenceWeights, 1, 1, actionCount);
bank.actionPolicyDetails = repmat( ...
    {referenceDetails}, 1, actionCount);
bank.actionPosteriorObjective = actionObjective;
bank.actionPosteriorProxyAllowed = true(1, actionCount);
bank.actionWithinReferencePayload = true(1, actionCount);
bank.actionPredictedNetSavingBytes = nan(1, actionCount);
bank.actionFormationIds = actionFormationIds;
bank.formationConditionedTimes = ...
    context.currentTime + (0:(horizonSteps - 1));
bank.actionFormationConditionedSchedule = schedules;
bank.actionCompleteLabelExceptionsByTime = cell(1, actionCount);
bank.completeLabelExceptionsByTime = cell(1, 0);
bank.positiveSupportThreshold = online.positiveSupportThreshold;
bank.highExistenceThreshold = layered.highExistenceThreshold;
bank.referenceAdjacency = logical(referenceAdjacency);
bank.referenceFusionWeights = referenceWeights;
bank.referencePolicyDetails = referenceDetails;
bank.maximumLabelEdits = 1;
bank.formationIds = formationIds;
bank.formationNeed = formationNeed;
bank.withholdingDurationPages = ...
    protocol.actions.maximumWithholdingDurationPages;
bank.sameFormationCooldownPages = ...
    protocol.actions.sameFormationCooldownPages;
bank.effectiveLabelGraphRecoveryWindowPages = ...
    protocol.actions.effectiveLabelGraphRecoveryWindowPages;
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
