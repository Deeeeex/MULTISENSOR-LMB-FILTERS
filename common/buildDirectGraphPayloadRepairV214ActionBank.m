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
actionMode = char(getField(options, ...
    'actionMode', 'withholding-only'));
referenceAttemptedBytes = getField( ...
    options, 'referenceAttemptedBytes', NaN);
repairSourceWeight = getField(options, 'repairSourceWeight', 0.5);
if ~isstruct(context) || ~isscalar(context) || ...
        ~iscell(context.localPosteriorBySensor) || ...
        ~isnumeric(groupIds) || ...
        numel(groupIds) ~= numel(context.localPosteriorBySensor) || ...
        ~isscalar(horizonSteps) || ~isfinite(horizonSteps) || ...
        horizonSteps ~= protocol.horizonSteps || ...
        ~ismember(actionMode, { ...
            'withholding-only', ...
            'withholding-plus-causal-label-kla'})
    error('DirectGraphPayloadRepairV214:InvalidActionBankInput', ...
        'V214 requires one truth-free H=3 observable state.');
end
creditRepairMode = strcmp( ...
    actionMode, 'withholding-plus-causal-label-kla');
if creditRepairMode && ...
        (~isscalar(referenceAttemptedBytes) || ...
         ~isnumeric(referenceAttemptedBytes) || ...
         ~isfinite(referenceAttemptedBytes) || ...
         referenceAttemptedBytes <= 0 || ...
         ~isscalar(repairSourceWeight) || ...
         ~isnumeric(repairSourceWeight) || ...
         ~isfinite(repairSourceWeight) || ...
         repairSourceWeight <= 0 || repairSourceWeight >= 1)
    error('DirectGraphPayloadRepairV214:InvalidCreditRepairOptions', ...
        ['The causal credit-repair bank requires one positive reference ', ...
         'page byte count and a KLA source weight in (0,1).']);
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
actionRepairPlans = repmat({struct()}, 1, actionCount);
actionObjective = zeros(1, actionCount);
featureNames = referenceDetails. ...
    hierarchicalWithholdingFeatureNames;
formationFeatures = referenceDetails. ...
    hierarchicalWithholdingFeaturesByFormation;
actionFeatures = zeros(actionCount, numel(featureNames));
for actionIdx = 2:actionCount
    formationId = requestedFormationIds(actionIdx - 1);
    schedule = repmat({zeros(1, 0)}, 1, horizonSteps);
    schedule{1} = formationId;
    schedules{actionIdx} = schedule;
    actionFormationIds{actionIdx} = formationId;
    if creditRepairMode
        actionNames{actionIdx} = sprintf( ...
            'v216-withhold-f%d-plus-causal-label-kla', formationId);
        actionRepairPlans{actionIdx} = buildCausalRepairPlan( ...
            context.currentTime, formationId, referenceAttemptedBytes, ...
            repairSourceWeight);
    else
        actionNames{actionIdx} = sprintf( ...
            'v214-withhold-full-posterior-f%d-one-page', formationId);
    end
    needIdx = find(formationIds == formationId, 1);
    actionObjective(actionIdx) = -formationNeed(needIdx);
    actionFeatures(actionIdx, :) = formationFeatures(needIdx, :);
end

referenceWeights = referenceDetails.fusionWeightMatrix;
layered = getTrackingAlignedLayeredLabelHeadroomV62Protocol();
online = getOnlinePositiveNetAddressablePayloadV99Protocol();
bank = struct();
bank.contractVersion = ...
    'direct-graph-payload-repair-action-bank-v214-v2';
bank.bankVariant = 'direct-graph-payload-repair-v214';
bank.actionMode = actionMode;
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
bank.actionFeatureContractVersion = referenceDetails. ...
    hierarchicalWithholdingFeatureContractVersion;
bank.actionFeatureNames = featureNames;
bank.actionFeatures = actionFeatures;
bank.actionFeaturePresentMask = ...
    (1:actionCount) ~= bank.referenceActionIndex;
bank.actionFeaturesUseTruth = false;
bank.actionFeaturesUseFutureInformation = false;
bank.actionPosteriorProxyAllowed = true(1, actionCount);
bank.actionWithinReferencePayload = true(1, actionCount);
bank.actionPredictedNetSavingBytes = nan(1, actionCount);
bank.actionFormationIds = actionFormationIds;
bank.actionBudgetRecycledFormationRepairPlans = actionRepairPlans;
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

function plan = buildCausalRepairPlan( ...
        currentTime, targetFormationId, referenceAttemptedBytes, ...
        sourceWeight)
plan = struct();
plan.times = currentTime;
plan.referenceAttemptedBytes = referenceAttemptedBytes;
plan.idealDeliveryTeacherMode = false;
plan.maximumActions = 1;
plan.forcedFormationId = 0;
plan.forcedSourceId = 0;
plan.forcedLabel = zeros(2, 1);
plan.targetFormationId = targetFormationId;
plan.updateMode = 'causal-label-kla';
plan.sourceWeight = sourceWeight;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
