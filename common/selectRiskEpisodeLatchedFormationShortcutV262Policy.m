function [adjacency, details] = ...
        selectRiskEpisodeLatchedFormationShortcutV262Policy(context)
% SELECTRISKEPISODELATCHEDFORMATIONSHORTCUTV262POLICY Suppress tree chatter.
%
% V261 greedily selected the lowest-risk donor on every page.  During the
% M24 F4 event that donor alternated between F1 and F2, so the formation tree
% changed on every active page.  V262 latches the first accepted donor,
% target, label and tree while the same posterior-risk episode remains active.
% A latch is released only when its target/label risk clears, its donor no
% longer satisfies the registered support gate, or its tree is no longer
% physically projectable.

protocol = getRiskEpisodeLatchedFormationShortcutV262Protocol();
[instantaneousAdjacency, instantaneous] = ...
    selectRiskTriggeredFormationShortcutV261Policy(context);
previous = latestLatch(context);

adjacency = instantaneousAdjacency;
details = instantaneous;
latchRetained = false;
latchReleased = false;
latchInitiated = false;
latchActive = false;
releaseReason = '';
latchedPairs = zeros(0, 2);
latchedLabel = zeros(2, 0);
latchedTargetId = 0;
latchedDonorId = 0;
latchedTargetRisk = NaN;
latchedDonorRisk = NaN;
latchedReferenceHops = Inf;
latchedPhysicalHops = Inf;
latchedHopReduction = 0;
latchedPathIds = zeros(1, 0);
latchedPathUids = zeros(1, 0);

if previous.valid && previous.active
    [retainAllowed, currentRisk, reason] = ...
        latchStillSupported(context, previous, protocol);
    if retainAllowed
        [projectedAdjacency, projected] = ...
            selectHorizonValueProjectedMinimumTreeV249Policy( ...
                context, previous.treePairs);
        if projected.requestedTreeApplied
            adjacency = projectedAdjacency;
            details = projected;
            latchRetained = true;
            latchActive = true;
            latchedPairs = previous.treePairs;
            latchedLabel = previous.label;
            latchedTargetId = previous.targetId;
            latchedDonorId = previous.donorId;
            latchedTargetRisk = currentRisk.target;
            latchedDonorRisk = currentRisk.donor;
            latchedReferenceHops = previous.referenceHops;
            latchedPhysicalHops = previous.physicalHops;
            latchedHopReduction = previous.hopReduction;
            latchedPathIds = previous.pathIds;
            latchedPathUids = previous.pathUids;
        else
            latchReleased = true;
            releaseReason = 'latched-tree-no-longer-projectable';
        end
    else
        latchReleased = true;
        releaseReason = reason;
    end
end

if ~latchRetained && instantaneous.shortcutInitiated
    latchInitiated = true;
    latchActive = true;
    latchedPairs = normalizePairs( ...
        instantaneous.appliedFormationTreePairs);
    latchedLabel = instantaneous.selectedLocalizationLabel;
    latchedTargetId = ...
        instantaneous.selectedLocalizationFormationId;
    latchedDonorId = instantaneous.selectedDonorFormationId;
    latchedTargetRisk = instantaneous.targetLabelLocalizationRisk;
    latchedDonorRisk = instantaneous.donorLabelLocalizationRisk;
    latchedReferenceHops = ...
        instantaneous.referenceFormationHopDistance;
    latchedPhysicalHops = ...
        instantaneous.physicalFormationHopDistance;
    latchedHopReduction = instantaneous.formationHopReduction;
    latchedPathIds = instantaneous.shortcutPathFormationIds;
    latchedPathUids = instantaneous.shortcutPathFormationUids;
end

nodeCount = size(adjacency, 1);
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
formationCount = numel(unique(groupIds));
weights = details.fusionWeightMatrix;
expectedMessages = nodeCount + 2 * (formationCount - 1);
positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
hardGate = nnz(adjacency) == expectedMessages && ...
    ~any(logical(adjacency(:)) & ...
        ~logical(context.physicalAdjacency(:))) && ...
    isStronglyConnected(logical(adjacency)) && ...
    all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
    all(weights(:) >= -1e-12) && ...
    ~any(weights(:) > 0 & ~positiveAllowed(:));
if ~hardGate
    error('RiskEpisodeLatchV262:ProjectionFailed', ...
        'The latched route violates a graph, weight or budget gate.');
end

details.contractVersion = ...
    'risk-episode-latched-formation-shortcut-v262-policy-v1';
details.protocolId = protocol.id;
details.armId = protocol.armId;
details.mode = 'risk-episode-latched-formation-kla-path-shortcut';
details.backboneMode = details.mode;
details.minimumBackboneAdjacency = logical( ...
    instantaneous.minimumBackboneAdjacency);
details.minimumFormationTreePairs = normalizePairs( ...
    instantaneous.minimumFormationTreePairs);
details.instantaneousProposedFormationTreePairs = normalizePairs( ...
    instantaneous.requestedFormationTreePairs);
details.instantaneousProposedTargetFormationId = ...
    instantaneous.selectedLocalizationFormationId;
details.instantaneousProposedDonorFormationId = ...
    instantaneous.selectedDonorFormationId;
details.instantaneousProposedLocalizationLabel = ...
    instantaneous.selectedLocalizationLabel;
details.localizationTailRiskByFormation = ...
    instantaneous.localizationTailRiskByFormation;
details.localizationRelativeRiskByFormation = ...
    instantaneous.localizationRelativeRiskByFormation;
details.localizationRiskFormationMedian = ...
    instantaneous.localizationRiskFormationMedian;
details.localizationRiskGateByFormation = ...
    instantaneous.localizationRiskGateByFormation;
details.selectedLocalizationFormationId = conditionalValue( ...
    latchActive, latchedTargetId, ...
    instantaneous.selectedLocalizationFormationId);
details.selectedDonorFormationId = conditionalValue( ...
    latchActive, latchedDonorId, ...
    instantaneous.selectedDonorFormationId);
details.selectedLocalizationLabel = conditionalValue( ...
    latchActive, latchedLabel, ...
    instantaneous.selectedLocalizationLabel);
details.targetLabelLocalizationRisk = conditionalValue( ...
    latchActive, latchedTargetRisk, ...
    instantaneous.targetLabelLocalizationRisk);
details.donorLabelLocalizationRisk = conditionalValue( ...
    latchActive, latchedDonorRisk, ...
    instantaneous.donorLabelLocalizationRisk);
details.referenceFormationHopDistance = conditionalValue( ...
    latchActive, latchedReferenceHops, ...
    instantaneous.referenceFormationHopDistance);
details.physicalFormationHopDistance = conditionalValue( ...
    latchActive, latchedPhysicalHops, ...
    instantaneous.physicalFormationHopDistance);
details.formationHopReduction = conditionalValue( ...
    latchActive, latchedHopReduction, ...
    instantaneous.formationHopReduction);
details.shortcutPathFormationIds = conditionalValue( ...
    latchActive, latchedPathIds, ...
    instantaneous.shortcutPathFormationIds);
details.shortcutPathFormationUids = conditionalValue( ...
    latchActive, latchedPathUids, ...
    instantaneous.shortcutPathFormationUids);
details.requestedFormationTreePairs = conditionalValue( ...
    latchActive, latchedPairs, ...
    instantaneous.requestedFormationTreePairs);
details.appliedFormationTreePairs = normalizePairs( ...
    details.currentFormationTreePairs);
details.riskEpisodeLatchInitiated = latchInitiated;
details.riskEpisodeLatchRetained = latchRetained;
details.riskEpisodeLatchReleased = latchReleased;
details.riskEpisodeActive = latchActive;
details.riskEpisodeReleaseReason = releaseReason;
details.latchedFormationTreePairs = latchedPairs;
details.latchedLocalizationLabel = latchedLabel;
details.latchedTargetFormationId = latchedTargetId;
details.latchedDonorFormationId = latchedDonorId;
details.instantaneousDonorSwitchSuppressed = latchRetained && ...
    instantaneous.shortcutRequestAttempted && ...
    instantaneous.selectedDonorFormationId ~= latchedDonorId;
details.shortcutRequestAttempted = ...
    instantaneous.shortcutRequestAttempted;
details.shortcutRequestApplied = latchActive;
details.shortcutInitiated = latchInitiated;
details.shortcutActive = latchActive;
details.currentMessageCount = nnz(adjacency);
details.minimumArchitectureMessageCount = expectedMessages;
details.posteriorUsed = true;
details.posteriorPayloadMetadataUsed = false;
details.currentNetworkPosteriorSynopsisUsed = true;
details.distributedControlSynopsisCostIncluded = false;
details.centralizedDevelopmentController = true;
details.truthUsed = false;
details.measurementUsed = false;
details.futureOutcomeUsed = false;
details.trackingOutcomeScored = false;

schedule = details.scheduleCertificate;
schedule.contractVersion = ...
    'risk-episode-latched-formation-shortcut-v262-schedule-v1';
schedule.phase = latchPhase(latchInitiated, latchRetained, latchReleased);
schedule.riskEpisodeLatchInitiated = latchInitiated;
schedule.riskEpisodeLatchRetained = latchRetained;
schedule.riskEpisodeLatchReleased = latchReleased;
schedule.riskEpisodeActive = latchActive;
schedule.riskEpisodeReleaseReason = releaseReason;
schedule.latchedFormationTreePairs = latchedPairs;
schedule.latchedLocalizationLabel = latchedLabel;
schedule.latchedTargetFormationId = latchedTargetId;
schedule.latchedDonorFormationId = latchedDonorId;
schedule.instantaneousProposedTargetFormationId = ...
    instantaneous.selectedLocalizationFormationId;
schedule.instantaneousProposedDonorFormationId = ...
    instantaneous.selectedDonorFormationId;
schedule.instantaneousDonorSwitchSuppressed = ...
    details.instantaneousDonorSwitchSuppressed;
schedule.selectedLocalizationFormationId = ...
    details.selectedLocalizationFormationId;
schedule.selectedDonorFormationId = ...
    details.selectedDonorFormationId;
schedule.selectedLocalizationLabel = ...
    details.selectedLocalizationLabel;
schedule.targetLabelLocalizationRisk = ...
    details.targetLabelLocalizationRisk;
schedule.donorLabelLocalizationRisk = ...
    details.donorLabelLocalizationRisk;
schedule.referenceFormationHopDistance = ...
    details.referenceFormationHopDistance;
schedule.physicalFormationHopDistance = ...
    details.physicalFormationHopDistance;
schedule.formationHopReduction = details.formationHopReduction;
schedule.shortcutPathFormationIds = ...
    details.shortcutPathFormationIds;
schedule.shortcutPathFormationUids = ...
    details.shortcutPathFormationUids;
schedule.shortcutRequestAttempted = ...
    details.shortcutRequestAttempted;
schedule.shortcutRequestApplied = details.shortcutRequestApplied;
schedule.shortcutInitiated = details.shortcutInitiated;
schedule.shortcutActive = details.shortcutActive;
schedule.currentMessageCount = nnz(adjacency);
schedule.centralizedDevelopmentController = true;
schedule.distributedControlSynopsisCostIncluded = false;
details.scheduleCertificate = schedule;
end

function previous = latestLatch(context)
previous = struct('valid', false, 'active', false, ...
    'treePairs', zeros(0, 2), 'label', zeros(2, 0), ...
    'targetId', 0, 'donorId', 0, 'referenceHops', Inf, ...
    'physicalHops', Inf, 'hopReduction', 0, ...
    'pathIds', zeros(1, 0), 'pathUids', zeros(1, 0));
if ~isfield(context, 'previousPolicyScheduleHistory') || ...
        isempty(context.previousPolicyScheduleHistory)
    return;
end
schedule = context.previousPolicyScheduleHistory{end};
required = {'contractVersion', 'riskEpisodeActive', ...
    'latchedFormationTreePairs', 'latchedLocalizationLabel', ...
    'latchedTargetFormationId', 'latchedDonorFormationId', ...
    'referenceFormationHopDistance', ...
    'physicalFormationHopDistance', 'formationHopReduction', ...
    'shortcutPathFormationIds'};
if ~isstruct(schedule) || ~isscalar(schedule) || ...
        ~all(isfield(schedule, required)) || ...
        ~strcmp(schedule.contractVersion, ...
            'risk-episode-latched-formation-shortcut-v262-schedule-v1')
    return;
end
previous.valid = true;
previous.active = logical(schedule.riskEpisodeActive);
previous.treePairs = normalizePairs( ...
    schedule.latchedFormationTreePairs);
previous.label = schedule.latchedLocalizationLabel;
previous.targetId = schedule.latchedTargetFormationId;
previous.donorId = schedule.latchedDonorFormationId;
previous.referenceHops = schedule.referenceFormationHopDistance;
previous.physicalHops = schedule.physicalFormationHopDistance;
previous.hopReduction = schedule.formationHopReduction;
previous.pathIds = schedule.shortcutPathFormationIds;
previous.pathUids = getField( ...
    schedule, 'shortcutPathFormationUids', zeros(1, 0));
end

function [allowed, risk, reason] = ...
        latchStillSupported(context, previous, protocol)
allowed = false;
risk = struct('target', NaN, 'donor', NaN);
reason = 'latched-risk-no-longer-supported';
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
summary = summarizeFormationLmbRiskModesV259( ...
    context.localPosteriorBySensor, groupIds, context.model);
target = find(summary.formationIds == previous.targetId, 1);
donor = find(summary.formationIds == previous.donorId, 1);
label = find(all(summary.labels == reshape(previous.label, 2, 1), 1), 1);
if isempty(target) || isempty(donor) || isempty(label)
    reason = 'latched-formation-or-label-disappeared';
    return;
end
localization = reshape( ...
    summary.localizationTailRiskByFormation, 1, []);
relative = localization / max(median(localization), eps);
targetGate = localization(target) >= ...
    protocol.localizationAbsoluteThreshold && ...
    relative(target) >= protocol.localizationRelativeThreshold && ...
    summary.highestLocalizationRiskLabelIndexByFormation(target) == label;
risk.target = summary.labelLocalizationRiskByFormation(target, label);
risk.donor = summary.labelLocalizationRiskByFormation(donor, label);
donorGate = isfinite(risk.target) && isfinite(risk.donor) && ...
    summary.formationActiveCoverage(donor, label) >= ...
        protocol.minimumDonorCoverage && ...
    summary.formationMedianExistence(donor, label) >= ...
        protocol.minimumDonorExistence && ...
    risk.donor <= ...
        protocol.maximumDonorToTargetRiskRatio * risk.target;
allowed = targetGate && donorGate;
if ~targetGate
    reason = 'latched-target-risk-cleared';
elseif ~donorGate
    reason = 'latched-donor-support-cleared';
else
    reason = '';
end
end

function value = latchPhase(initiated, retained, released)
if retained
    value = 'v262-risk-episode-latch-retained';
elseif initiated
    value = 'v262-risk-episode-latch-initiated';
elseif released
    value = 'v262-risk-episode-latch-released';
else
    value = 'v262-minimum-backbone';
end
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

function pairs = normalizePairs(pairs)
if isempty(pairs)
    pairs = zeros(0, 2);
else
    pairs = sortrows(sort(pairs, 2), [1, 2]);
end
end

function value = conditionalValue(condition, left, right)
if condition, value = left; else, value = right; end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
