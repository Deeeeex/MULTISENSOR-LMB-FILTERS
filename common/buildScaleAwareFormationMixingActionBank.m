function bank = buildScaleAwareFormationMixingActionBank(context, options)
% BUILDSCALEAWAREFORMATIONMIXINGACTIONBANK Frozen equal-message action set.

if nargin < 2 || isempty(options)
    options = struct();
end
protocol = getFormationScaleAwareMixingProbeProtocol();
actionModes = getField(options, ...
    'actionModes', protocol.actionModes);
crossWeights = reshape(getField(options, ...
    'actionCrossResidualWeights', ...
    protocol.actionCrossResidualWeights), 1, []);
dominantWeight = getField(options, ...
    'dominantWeight', protocol.dominantWeight);
residualWeight = getField(options, ...
    'residualWeight', protocol.residualWeight);
orientation = getField(options, ...
    'orientation', protocol.orientation);
maximumActionCount = getField(options, ...
    'maximumActionCount', protocol.maximumActionCount);
if ~iscell(actionModes) || isempty(actionModes) || ...
        numel(actionModes) ~= numel(crossWeights) || ...
        ~strcmp(normalizeMode(actionModes{1}), ...
            'reference-cycle') || ...
        abs(crossWeights(1) - residualWeight) > 1e-12 || ...
        ~isscalar(maximumActionCount) || ...
        maximumActionCount < 1 || ...
        numel(actionModes) > round(maximumActionCount)
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware action-bank specification is invalid.');
end

groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
nodeCount = numel(context.localPosteriorBySensor);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
if numel(groupIds) ~= nodeCount || formationCount < 2
    error('ScaleMixing:InvalidContract', ...
        'Scale-aware action-bank context is invalid.');
end

actionAdjacency = false(nodeCount, nodeCount, 0);
actionFusionWeights = zeros(nodeCount, nodeCount, 0);
actionFormationMixing = zeros(formationCount, formationCount, 0);
actionNames = cell(1, 0);
realizedModes = cell(1, 0);
realizedCrossWeights = zeros(1, 0);
actionMessageCounts = zeros(1, 0);
actionCrossCounts = zeros(1, 0);
actionLaneCounts = zeros(1, 0);
actionSpectralGapProxy = zeros(1, 0);
actionDetails = cell(1, 0);
keys = cell(1, 0);
attempts = repmat(emptyAttempt(), 1, 0);

for specIdx = 1:numel(actionModes)
    attempt = emptyAttempt();
    attempt.specificationIndex = specIdx;
    attempt.requestedMode = normalizeMode(actionModes{specIdx});
    attempt.crossResidualWeight = crossWeights(specIdx);
    attempt.actionName = buildActionName( ...
        attempt.requestedMode, crossWeights(specIdx));
    try
        [adjacency, details] = ...
            selectScaleAwareFormationMixingPolicy( ...
                context, attempt.requestedMode, struct( ...
                    'orientation', orientation, ...
                    'dominantWeight', dominantWeight, ...
                    'residualWeight', residualWeight, ...
                    'crossResidualWeight', crossWeights(specIdx), ...
                    'fallbackMode', 'error'));
        if details.projectionFallbackUsed || ...
                ~details.messageCountParityWithReference || ...
                ~details.combinedSensorStrongConnected || ...
                ~details.formationStrongConnected || ...
                details.truthUsed || details.futureOutcomeUsed
            error('ScaleMixing:InvalidCandidate', ...
                'Action bank rejected an unsafe or noncausal action.');
        end
        key = [sprintf('%d,', adjacency(:)), '|', ...
            sprintf('%.17g,', details.fusionWeightMatrix(:))];
        duplicateIdx = find(strcmp(keys, key), 1);
        if ~isempty(duplicateIdx)
            attempt.status = 'duplicate';
            attempt.actionIndex = duplicateIdx;
            attempts(end + 1) = attempt; %#ok<AGROW>
            continue;
        end
        actionIdx = size(actionAdjacency, 3) + 1;
        actionAdjacency(:, :, actionIdx) = logical(adjacency);
        actionFusionWeights(:, :, actionIdx) = ...
            details.fusionWeightMatrix;
        actionFormationMixing(:, :, actionIdx) = ...
            details.formationMixingMatrix;
        actionNames{actionIdx} = attempt.actionName;
        realizedModes{actionIdx} = details.realizedMode;
        realizedCrossWeights(actionIdx) = ...
            details.meanCrossResidualWeight;
        actionMessageCounts(actionIdx) = nnz(adjacency);
        actionCrossCounts(actionIdx) = ...
            details.crossFormationMessageCount;
        actionLaneCounts(actionIdx) = details.gatewayLaneCount;
        actionSpectralGapProxy(actionIdx) = ...
            details.formationMixingSpectralGapProxy;
        actionDetails{actionIdx} = details;
        keys{actionIdx} = key;
        attempt.status = 'accepted';
        attempt.actionIndex = actionIdx;
    catch errorInfo
        if ~isUnavailable(errorInfo)
            rethrow(errorInfo);
        end
        attempt.status = 'unavailable';
        attempt.failureIdentifier = ...
            getErrorField(errorInfo, 'identifier');
        attempt.failureMessage = ...
            getErrorField(errorInfo, 'message');
    end
    attempts(end + 1) = attempt; %#ok<AGROW>
end

actionCount = size(actionAdjacency, 3);
if actionCount < 1 || ...
        ~strcmp(realizedModes{1}, 'reference-cycle') || ...
        any(actionMessageCounts ~= actionMessageCounts(1)) || ...
        any(actionMessageCounts > floor(getField( ...
            context, 'directedMessageBudget', inf))) || ...
        actionCount > round(maximumActionCount)
    error('ScaleMixing:InvalidCandidate', ...
        'Scale-aware action bank failed its global invariants.');
end

bank = struct();
bank.contractVersion = ...
    'scale-aware-formation-mixing-action-bank-v1';
bank.protocolId = protocol.id;
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.groupIds = groupIds;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.actionNames = actionNames;
bank.actionModes = realizedModes;
bank.actionCrossResidualWeights = realizedCrossWeights;
bank.actionAdjacency = actionAdjacency;
bank.actionFusionWeights = actionFusionWeights;
bank.actionFormationMixing = actionFormationMixing;
bank.actionMessageCounts = actionMessageCounts;
bank.actionCrossFormationMessageCounts = actionCrossCounts;
bank.actionGatewayLaneCounts = actionLaneCounts;
bank.actionFormationMixingSpectralGapProxy = ...
    actionSpectralGapProxy;
bank.actionDetails = actionDetails;
% Compatibility fields for the generic frozen-action executor.  Payload
% admissibility is evaluated from realized bytes after execution; this
% truth-free structural bank imposes message-count parity before execution.
bank.actionWithinReferencePayload = true(1, actionCount);
bank.actionFormationIndex = zeros(1, actionCount);
bank.actionModeIndex = 1:actionCount;
bank.modeTrustWeights = realizedCrossWeights;
bank.actionPosteriorProxyAllowed = false(1, actionCount);
bank.actionPosteriorObjective = actionSpectralGapProxy;
bank.actionPayloadBytes = nan(1, actionCount);
bank.messageCountParityWithReference = ...
    all(actionMessageCounts == actionMessageCounts(1));
allPhysical = true;
physical = logical(context.physicalAdjacency);
for actionIdx = 1:actionCount
    currentAdjacency = actionAdjacency(:, :, actionIdx);
    allPhysical = allPhysical && ...
        ~any(currentAdjacency(:) & ~physical(:));
end
bank.allPhysical = allPhysical;
bank.allCombinedOneStepStrongConnected = all(cellfun( ...
    @(item) item.combinedSensorStrongConnected, actionDetails));
bank.allFormationStrongConnected = all(cellfun( ...
    @(item) item.formationStrongConnected, actionDetails));
bank.truthUsed = any(cellfun( ...
    @(item) item.truthUsed, actionDetails));
bank.futureOutcomeUsed = any(cellfun( ...
    @(item) item.futureOutcomeUsed, actionDetails));
bank.attempts = attempts;
bank.acceptedSpecificationCount = nnz(strcmp( ...
    {attempts.status}, 'accepted'));
bank.duplicateSpecificationCount = nnz(strcmp( ...
    {attempts.status}, 'duplicate'));
bank.unavailableSpecificationCount = nnz(strcmp( ...
    {attempts.status}, 'unavailable'));
bank.openedDevelopmentEvidenceOnly = true;
bank.validationClaimAllowed = false;
end

function name = buildActionName(mode, weight)
weightText = strrep(sprintf('%.2f', weight), '.', 'p');
name = sprintf('%s-cross-w%s', mode, weightText);
end

function mode = normalizeMode(mode)
mode = lower(strrep(char(mode), '_', '-'));
end

function tf = isUnavailable(errorInfo)
identifier = getErrorField(errorInfo, 'identifier');
tf = strcmp(identifier, 'ScaleMixing:Infeasible');
end

function value = getErrorField(errorInfo, fieldName)
value = '';
try
    value = errorInfo.(fieldName);
catch
    value = '';
end
if isempty(value)
    value = '';
end
end

function attempt = emptyAttempt()
attempt = struct( ...
    'specificationIndex', NaN, ...
    'requestedMode', '', ...
    'crossResidualWeight', NaN, ...
    'actionName', '', ...
    'status', '', ...
    'actionIndex', NaN, ...
    'failureIdentifier', '', ...
    'failureMessage', '');
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
