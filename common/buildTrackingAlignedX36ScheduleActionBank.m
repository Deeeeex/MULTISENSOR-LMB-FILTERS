function bank = buildTrackingAlignedX36ScheduleActionBank( ...
        context, groupIds, options)
% BUILDTRACKINGALIGNEDX36SCHEDULEACTIONBANK Frozen observable H=3 bank.
%
% The physical reference graph is identical for every arm.  Nonreference
% arms differ only in which formations temporarily stop consuming complete
% cross-formation posteriors.  Formation ranking uses current posterior
% association support and never reads truth, future measurements, or future
% outcomes.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getTrackingAlignedX36ScheduleHeadroomV63Protocol();
groupIds = reshape(groupIds, 1, []);
if ~isstruct(context) || ~isfield(context, 'model') || ...
        ~isfield(context, 'currentTime') || ...
        ~ismember(context.currentTime, protocol.anchorTimes) || ...
        numel(groupIds) ~= protocol.expectedNodeCount || ...
        numel(unique(groupIds, 'stable')) ~= ...
            protocol.expectedFormationCount
    error('TrackingAlignedV63:InvalidActionBankInput', ...
        'V63 requires a registered current X36 observable state.');
end

policyProtocol = getFormationIsolateReconnectProbeProtocol();
policyOptions = struct( ...
    'dominantWeight', policyProtocol.dominantWeight, ...
    'residualWeight', policyProtocol.residualWeight);
[referenceAdjacency, referenceDetails] = ...
    selectBackboneResidualSplicedCyclePolicy( ...
        context, 'fixed-counter-clockwise', policyOptions);
referenceWeights = referenceDetails.fusionWeightMatrix;

[signature, control] = ...
    computeFormationRoutingLeverageSignature(context, groupIds);
positiveSupportThreshold = getField(options, ...
    'positiveSupportThreshold', protocol.positiveSupportThreshold);
support = computeObservationSupportedRoutingLeverage( ...
    control, context, groupIds, positiveSupportThreshold);

groups = reshape(control.groups, 1, []);
scores = reshape( ...
    support.singleSupportWeightedRescuedFractionOfReference, 1, []);
available = reshape(control.singleActionAvailableMask, 1, []) & ...
    isfinite(scores);
if nnz(available) < 3
    error('TrackingAlignedV63:InsufficientObservableRanking', ...
        'V63 needs at least three observable formation interventions.');
end
availableIndices = find(available);
rankingTable = [-scores(availableIndices)', groups(availableIndices)'];
[~, localOrder] = sortrows(rankingTable, [1, 2]);
rankedIndices = availableIndices(localOrder);
rankedFormationIds = groups(rankedIndices);
rankedScores = scores(rankedIndices);

top1 = rankedFormationIds(1);
top2 = rankedFormationIds(1:2);
top3 = rankedFormationIds(1:3);
anchorIdx = find(protocol.anchorTimes == context.currentTime, 1);
legacySchedule = protocol.legacyFormationSchedules{anchorIdx};

schedules = cell(1, 7);
schedules{1} = {[], [], []};
schedules{2} = legacySchedule;
schedules{3} = {top1, top1, top1};
schedules{4} = {top2, top2, top2};
schedules{5} = {top3, top3, top3};
schedules{6} = {top3, top2, top1};
schedules{7} = {top3, top2, []};

actionNames = { ...
    'reference-full-payload', ...
    'legacy-v37-schedule', ...
    sprintf('support-top1-persistent-%s', formatIds(top1)), ...
    sprintf('support-top2-persistent-%s', formatIds(top2)), ...
    sprintf('support-top3-persistent-%s', formatIds(top3)), ...
    sprintf('support-nested-3-2-1-%s', formatIds(top3)), ...
    sprintf('support-nested-3-2-0-%s', formatIds(top3))};
payloadModes = [ ...
    {'reference-full-payload'}, ...
    repmat({'control-only'}, 1, numel(actionNames) - 1)];

actionObjective = zeros(1, numel(actionNames));
actionFormationIds = cell(1, numel(actionNames));
for actionIdx = 1:numel(actionNames)
    actionObjective(actionIdx) = scheduleObjective( ...
        schedules{actionIdx}, groups, scores);
    actionFormationIds{actionIdx} = unique([schedules{actionIdx}{:}], ...
        'stable');
end

bank = struct();
bank.contractVersion = ...
    'tracking-aligned-x36-schedule-action-bank-v63-v1';
bank.actionCount = numel(actionNames);
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionPayloadModes = payloadModes;
bank.actionAdjacency = repmat( ...
    logical(referenceAdjacency), 1, 1, bank.actionCount);
bank.actionFusionWeights = repmat( ...
    referenceWeights, 1, 1, bank.actionCount);
bank.actionPosteriorObjective = actionObjective;
bank.actionPosteriorProxyAllowed = true(1, bank.actionCount);
bank.actionWithinReferencePayload = true(1, bank.actionCount);
bank.actionPredictedNetSavingBytes = nan(1, bank.actionCount);
bank.actionFormationIds = actionFormationIds;
bank.formationConditionedTimes = ...
    context.currentTime + (0:(protocol.horizonSteps - 1));
bank.actionFormationConditionedSchedule = schedules;
bank.positiveSupportThreshold = positiveSupportThreshold;
layeredProtocol = getTrackingAlignedLayeredLabelHeadroomV62Protocol();
bank.highExistenceThreshold = layeredProtocol.highExistenceThreshold;
bank.referenceAdjacency = logical(referenceAdjacency);
bank.referenceFusionWeights = referenceWeights;
bank.referencePolicyDetails = referenceDetails;
bank.routingLeverageSignature = signature;
bank.observationSupportMetrics = support;
bank.rankedFormationIds = rankedFormationIds;
bank.rankedFormationScores = rankedScores;
bank.legacyFormationSchedule = legacySchedule;
bank.truthUsed = false;
bank.futureMeasurementsUsed = false;
bank.futureOutcomesUsed = false;
end

function value = scheduleObjective(schedule, groups, scores)
value = 0;
for stepIdx = 1:numel(schedule)
    formationIds = reshape(schedule{stepIdx}, 1, []);
    for formationId = formationIds
        idx = find(groups == formationId, 1);
        if isempty(idx) || ~isfinite(scores(idx))
            error('TrackingAlignedV63:InvalidFormationSchedule', ...
                'A scheduled formation lacks an observable score.');
        end
        value = value + scores(idx);
    end
end
end

function value = formatIds(ids)
tokens = arrayfun(@(id) sprintf('f%d', id), ...
    reshape(ids, 1, []), 'UniformOutput', false);
value = strjoin(tokens, '-');
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
