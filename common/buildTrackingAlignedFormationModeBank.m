function bank = buildTrackingAlignedFormationModeBank( ...
        context, sourceTrustBank, groupIds, options)
% BUILDTRACKINGALIGNEDFORMATIONMODEBANK V56 local routing actions.
%
% Combines the existing formation-local source/trust actions with one
% existence-retention-safe protection action per formation.  Returning to
% the reference after a protected step is the recovery action; it is kept as
% the same reference mode so that route history, rather than a duplicate
% graph, determines whether the step means hold or recover.

if nargin < 4 || isempty(options)
    options = struct();
end
required = { ...
    'nodeCount', 'formationCount', 'modeCount', 'actionCount', ...
    'referenceActionIndex', 'modeTrustWeights', 'actionNames', ...
    'actionModes', 'actionFormationIndex', 'actionModeIndex', ...
    'actionAdjacency', 'actionFusionWeights', ...
    'actionDominantSources', 'actionPosteriorProxyAllowed', ...
    'actionPosteriorObjective', 'actionPayloadBytes', ...
    'referencePayloadBytes', 'truthUsed', 'futureOutcomeUsed'};
if ~isstruct(context) || ~isstruct(sourceTrustBank) || ...
        ~all(isfield(sourceTrustBank, required)) || ...
        sourceTrustBank.referenceActionIndex ~= 1 || ...
        sourceTrustBank.truthUsed || sourceTrustBank.futureOutcomeUsed
    error('Tracking-aligned source/trust bank is incomplete.');
end
groupIds = reshape(groupIds, 1, []);
nodeCount = numel(groupIds);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
sourceModeCount = round(sourceTrustBank.modeCount);
if nodeCount ~= sourceTrustBank.nodeCount || ...
        formationCount ~= sourceTrustBank.formationCount || ...
        sourceModeCount < 2 || ...
        ~isequal(size(sourceTrustBank.actionModes), ...
            [sourceTrustBank.actionCount, formationCount])
    error('Tracking-aligned source/trust dimensions are invalid.');
end

protectionActionIndices = 1 + 2 .^ (0:(formationCount - 1));
temporalOptions = getField(options, ...
    'temporalSuspensionBankOptions', struct());
if ~isstruct(temporalOptions)
    error('Tracking-aligned temporal-bank options are invalid.');
end
if isfield(temporalOptions, 'posteriorSafetyActionIndices') && ...
        ~isequal(reshape( ...
            temporalOptions.posteriorSafetyActionIndices, 1, []), ...
            [1, protectionActionIndices])
    error(['Tracking-aligned protection scoring must cover exactly ', ...
        'the reference and formation-local protection actions.']);
end
temporalOptions.posteriorSafetyActionIndices = ...
    [1, protectionActionIndices];
temporalBank = buildTemporalCrossEdgeSuspensionActionBank( ...
    context, temporalOptions);

referenceAdjacency = logical( ...
    sourceTrustBank.actionAdjacency(:, :, 1));
referenceWeights = sourceTrustBank.actionFusionWeights(:, :, 1);
temporalReferenceWeights = ...
    temporalBank.actionFusionWeights(:, :, 1);
if ~isequal(referenceAdjacency, ...
        logical(temporalBank.actionAdjacency(:, :, 1))) || ...
        max(abs(referenceWeights(:) - ...
            temporalReferenceWeights(:))) > 1e-12
    error(['Source/trust and protection banks do not share the same ', ...
        'registered reference route.']);
end

modeCount = sourceModeCount + 1;
protectModeIndex = modeCount;
actionCount = sourceTrustBank.actionCount + formationCount;
modeNames = cell(1, modeCount);
modeNames{1} = 'reference-or-recover';
for modeIdx = 2:sourceModeCount
    modeNames{modeIdx} = sprintf('source-trust-%.2f', ...
        sourceTrustBank.modeTrustWeights(modeIdx));
end
modeNames{protectModeIndex} = 'protect-cross-input';
modeKindNames = {'reference', 'source-trust', 'protect'};
modeKindIndex = [1, 2 * ones(1, sourceModeCount - 1), 3];
modeTrustWeights = [ ...
    reshape(sourceTrustBank.modeTrustWeights, 1, []), 0];

actionNames = [sourceTrustBank.actionNames, ...
    cell(1, formationCount)];
actionModes = [sourceTrustBank.actionModes; ...
    ones(formationCount, formationCount)];
actionFormationIndex = [ ...
    reshape(sourceTrustBank.actionFormationIndex, [], 1); ...
    (1:formationCount)'];
actionModeIndex = [ ...
    reshape(sourceTrustBank.actionModeIndex, [], 1); ...
    protectModeIndex * ones(formationCount, 1)];
actionAdjacency = false(nodeCount, nodeCount, actionCount);
actionFusionWeights = zeros(nodeCount, nodeCount, actionCount);
actionDominantSources = zeros(actionCount, nodeCount);
actionPosteriorProxyAllowed = false(actionCount, 1);
actionPosteriorObjective = zeros(actionCount, 1);
actionPayloadBytes = zeros(actionCount, 1);
sourceActionCount = sourceTrustBank.actionCount;
actionAdjacency(:, :, 1:sourceActionCount) = ...
    logical(sourceTrustBank.actionAdjacency);
actionFusionWeights(:, :, 1:sourceActionCount) = ...
    sourceTrustBank.actionFusionWeights;
actionDominantSources(1:sourceActionCount, :) = ...
    sourceTrustBank.actionDominantSources;
actionPosteriorProxyAllowed(1:sourceActionCount) = logical( ...
    sourceTrustBank.actionPosteriorProxyAllowed(:));
actionPosteriorObjective(1:sourceActionCount) = ...
    sourceTrustBank.actionPosteriorObjective(:);

referenceSources = reshape( ...
    sourceTrustBank.actionDominantSources(1, :), 1, []);
for formationIdx = 1:formationCount
    actionIdx = sourceActionCount + formationIdx;
    temporalIdx = protectionActionIndices(formationIdx);
    adjacency = logical( ...
        temporalBank.actionAdjacency(:, :, temporalIdx));
    weights = temporalBank.actionFusionWeights(:, :, temporalIdx);
    actionNames{actionIdx} = sprintf( ...
        'formation-%d-protect-cross-input', formationIdx);
    actionModes(actionIdx, formationIdx) = protectModeIndex;
    actionAdjacency(:, :, actionIdx) = adjacency;
    actionFusionWeights(:, :, actionIdx) = weights;
    actionDominantSources(actionIdx, :) = ...
        dominantSourcesFromRoute( ...
            adjacency, weights, referenceSources);
    actionPosteriorProxyAllowed(actionIdx) = ...
        temporalBank.actionPosteriorSafetyMask(temporalIdx);
    actionPosteriorObjective(actionIdx) = ...
        -temporalBank.retentionRisk(temporalIdx);
end

senderPayloadBytes = estimateSenderPayloadBytes(context);
for actionIdx = 1:actionCount
    actionPayloadBytes(actionIdx) = topologyPayloadBytes( ...
        actionAdjacency(:, :, actionIdx), senderPayloadBytes);
end
referencePayloadBytes = actionPayloadBytes(1);
if max(abs(actionPayloadBytes(1:sourceActionCount) - ...
        reshape(sourceTrustBank.actionPayloadBytes, [], 1))) > 1e-9 || ...
        abs(referencePayloadBytes - ...
            sourceTrustBank.referencePayloadBytes) > 1e-9
    error('Tracking-aligned payload ledger differs from source/trust bank.');
end

bank = struct();
bank.contractVersion = ...
    'tracking-aligned-formation-local-mode-bank-v56-v1';
bank.nodeCount = nodeCount;
bank.formationCount = formationCount;
bank.modeCount = modeCount;
bank.actionCount = actionCount;
bank.referenceActionIndex = 1;
bank.modeNames = modeNames;
bank.modeKindNames = modeKindNames;
bank.modeKindIndex = modeKindIndex;
bank.modeTrustWeights = modeTrustWeights;
bank.protectModeIndex = protectModeIndex;
bank.actionNames = actionNames;
bank.actionModes = actionModes;
bank.actionFormationIndex = actionFormationIndex;
bank.actionModeIndex = actionModeIndex;
bank.actionModeKindIndex = modeKindIndex(actionModeIndex)';
bank.actionAdjacency = actionAdjacency;
bank.actionFusionWeights = actionFusionWeights;
bank.actionDominantSources = actionDominantSources;
bank.actionPosteriorProxyAllowed = actionPosteriorProxyAllowed;
bank.actionPosteriorObjective = actionPosteriorObjective;
bank.actionPayloadBytes = actionPayloadBytes;
bank.referencePayloadBytes = referencePayloadBytes;
bank.actionWithinReferencePayload = ...
    actionPayloadBytes <= referencePayloadBytes + 1e-9;
bank.actionAvailableMask = true(actionCount, 1);
bank.protectionTemporalActionIndices = protectionActionIndices;
bank.protectionSafetyMask = logical( ...
    temporalBank.actionPosteriorSafetyMask(protectionActionIndices));
bank.senderPayloadBytes = senderPayloadBytes;
bank.recoverySemantics = ...
    ['Reference is recovery when the previous route history contains ', ...
     'a protected formation; no duplicate reference graph is emitted.'];
bank.sourceTrustBankContractVersion = ...
    sourceTrustBank.contractVersion;
bank.temporalBankContractVersion = temporalBank.contractVersion;
bank.truthUsed = false;
bank.futureOutcomeUsed = false;
bank.trainingTargetStored = false;
end

function sources = dominantSourcesFromRoute( ...
        adjacency, weights, fallbackSources)
nodeCount = size(adjacency, 1);
sources = reshape(fallbackSources, 1, []);
for receiverIdx = 1:nodeCount
    candidates = reshape(find(adjacency(receiverIdx, :)), 1, []);
    if isempty(candidates)
        sources(receiverIdx) = receiverIdx;
        continue;
    end
    candidateWeights = weights(receiverIdx, candidates);
    [~, localIdx] = max(candidateWeights);
    sources(receiverIdx) = candidates(localIdx);
end
end

function senderBytes = estimateSenderPayloadBytes(context)
nodeCount = numel(context.localPosteriorBySensor);
senderBytes = zeros(1, nodeCount);
for senderIdx = 1:nodeCount
    stats = estimateLmbPayloadSize( ...
        context.localPosteriorBySensor{senderIdx}, ...
        context.model, 2, struct());
    senderBytes(senderIdx) = stats.estimatedBytes;
end
end

function bytes = topologyPayloadBytes(adjacency, senderPayloadBytes)
bytes = sum(sum(adjacency, 1) .* senderPayloadBytes);
end

function value = getField(structure, fieldName, fallback)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = fallback;
end
end
