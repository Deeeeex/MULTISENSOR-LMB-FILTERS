function selection = selectReceiverSafeLabelOptionsExact( ...
        labelOptions, options)
% SELECTRECEIVERSAFELABELOPTIONSEXACT Exact label/edge budget projection.
%
% Each label contributes one option. Variable label bytes are additive, but
% an edge header is paid only when that edge first becomes active. Dynamic
% programming therefore keeps the active-edge mask in its state and Pareto
% prunes only states with the same mask.

if nargin < 2 || isempty(options)
    options = struct();
end
if ~iscell(labelOptions)
    error('ReceiverSafeSelector:InvalidOptions', ...
        'labelOptions must be a cell array with one struct array per label.');
end

edgeCount = inferEdgeCount(labelOptions, options);
edgeHeaderBytes = resolveEdgeVector( ...
    options, 'edgeHeaderBytes', zeros(1, edgeCount), edgeCount);
initialActiveEdgeMask = logical(resolveEdgeVector( ...
    options, 'initialActiveEdgeMask', false(1, edgeCount), edgeCount));
requiredFinalEdgeMask = logical(resolveEdgeVector( ...
    options, 'requiredFinalEdgeMask', false(1, edgeCount), edgeCount));
byteBudget = getField(options, 'byteBudget', Inf);
initialCostBytes = getField(options, 'initialCostBytes', 0);
if ~isscalar(byteBudget) || isnan(byteBudget) || byteBudget < 0 || ...
        ~isscalar(initialCostBytes) || ~isfinite(initialCostBytes) || ...
        initialCostBytes < 0 || initialCostBytes > byteBudget || ...
        any(~isfinite(edgeHeaderBytes)) || any(edgeHeaderBytes < 0)
    error('ReceiverSafeSelector:InvalidBudget', ...
        'Byte costs must be nonnegative and within the budget.');
end

states = makeState( ...
    initialActiveEdgeMask, initialCostBytes, 0, zeros(1, 0));
candidateCountBeforePruning = zeros(1, numel(labelOptions));
stateCountAfterPruning = zeros(1, numel(labelOptions));
for labelIdx = 1:numel(labelOptions)
    currentOptions = validateLabelOptions( ...
        labelOptions{labelIdx}, edgeCount, labelIdx);
    nextStates = repmat(makeState(false(1, edgeCount), 0, 0, []), 1, 0);
    for stateIdx = 1:numel(states)
        for optionIdx = 1:numel(currentOptions)
            option = currentOptions(optionIdx);
            if ~option.existenceRetentionSatisfied || ...
                    ~isfinite(option.distortion)
                continue;
            end
            newlyActive = option.activeEdgeMask & ...
                ~states(stateIdx).activeEdgeMask;
            costBytes = states(stateIdx).costBytes + ...
                option.variablePayloadBytes + sum(edgeHeaderBytes(newlyActive));
            if costBytes > byteBudget
                continue;
            end
            activeEdgeMask = states(stateIdx).activeEdgeMask | ...
                option.activeEdgeMask;
            choices = [states(stateIdx).selectedOptionIndices, optionIdx];
            nextStates(end+1) = makeState( ...
                activeEdgeMask, costBytes, ...
                states(stateIdx).distortion + option.distortion, choices); ...
                %#ok<AGROW>
        end
    end
    candidateCountBeforePruning(labelIdx) = numel(nextStates);
    states = pruneSameMaskParetoStates(nextStates);
    stateCountAfterPruning(labelIdx) = numel(states);
    if isempty(states)
        break;
    end
end

feasibleMask = false(1, numel(states));
for stateIdx = 1:numel(states)
    feasibleMask(stateIdx) = all( ...
        states(stateIdx).activeEdgeMask(requiredFinalEdgeMask));
end
states = states(feasibleMask);

selection = struct();
selection.contractVersion = 'receiver-safe-label-option-selector-v1';
selection.isFeasible = ~isempty(states);
selection.isExactForEnumeratedOptions = true;
selection.objectiveIsAdditiveAcrossLabels = true;
selection.edgeHeaderCostIsShared = true;
selection.candidateCountBeforePruning = candidateCountBeforePruning;
selection.stateCountAfterPruning = stateCountAfterPruning;
selection.byteBudget = byteBudget;
selection.edgeHeaderBytes = edgeHeaderBytes;
selection.initialActiveEdgeMask = initialActiveEdgeMask;
selection.requiredFinalEdgeMask = requiredFinalEdgeMask;
if isempty(states)
    selection.selectedOptionIndices = [];
    selection.totalDistortion = Inf;
    selection.totalBytes = Inf;
    selection.activeEdgeMask = false(1, edgeCount);
    return;
end

distortions = [states.distortion];
costs = [states.costBytes];
[~, order] = sortrows([distortions(:), costs(:)], [1, 2]);
best = states(order(1));
selection.selectedOptionIndices = best.selectedOptionIndices;
selection.totalDistortion = best.distortion;
selection.totalBytes = best.costBytes;
selection.activeEdgeMask = best.activeEdgeMask;
end

function options = validateLabelOptions(options, edgeCount, labelIdx)
requiredFields = {'distortion', 'variablePayloadBytes', ...
    'activeEdgeMask', 'existenceRetentionSatisfied'};
if ~isstruct(options) || isempty(options) || ...
        ~all(isfield(options, requiredFields))
    error('ReceiverSafeSelector:InvalidLabelOptions', ...
        'Label %d has no valid option array.', labelIdx);
end
for optionIdx = 1:numel(options)
    if ~isscalar(options(optionIdx).distortion) || ...
            isnan(options(optionIdx).distortion) || ...
            options(optionIdx).distortion < 0 || ...
            ~isscalar(options(optionIdx).variablePayloadBytes) || ...
            ~isfinite(options(optionIdx).variablePayloadBytes) || ...
            options(optionIdx).variablePayloadBytes < 0 || ...
            numel(options(optionIdx).activeEdgeMask) ~= edgeCount || ...
            ~isscalar(options(optionIdx).existenceRetentionSatisfied)
        error('ReceiverSafeSelector:InvalidLabelOption', ...
            ['Label %d option %d violates the option contract ', ...
             '(distortion=%g, bytes=%g, mask=%d/%d, retention=%d).'], ...
            labelIdx, optionIdx, options(optionIdx).distortion, ...
            options(optionIdx).variablePayloadBytes, ...
            numel(options(optionIdx).activeEdgeMask), edgeCount, ...
            numel(options(optionIdx).existenceRetentionSatisfied));
    end
    options(optionIdx).activeEdgeMask = logical( ...
        reshape(options(optionIdx).activeEdgeMask, 1, []));
    options(optionIdx).existenceRetentionSatisfied = logical( ...
        options(optionIdx).existenceRetentionSatisfied);
end
end

function states = pruneSameMaskParetoStates(states)
if isempty(states)
    return;
end
maskKeys = cell(1, numel(states));
for stateIdx = 1:numel(states)
    maskKeys{stateIdx} = char('0' + states(stateIdx).activeEdgeMask);
end
uniqueKeys = unique(maskKeys, 'stable');
kept = false(1, numel(states));
tolerance = 1e-12;
for keyIdx = 1:numel(uniqueKeys)
    indices = find(strcmp(maskKeys, uniqueKeys{keyIdx}));
    table = [[states(indices).costBytes]', [states(indices).distortion]'];
    [~, order] = sortrows(table, [1, 2]);
    bestDistortion = Inf;
    for localIdx = reshape(order, 1, [])
        stateIdx = indices(localIdx);
        if states(stateIdx).distortion < bestDistortion - tolerance
            kept(stateIdx) = true;
            bestDistortion = states(stateIdx).distortion;
        end
    end
end
states = states(kept);
end

function edgeCount = inferEdgeCount(labelOptions, options)
if isstruct(options) && isfield(options, 'edgeCount')
    edgeCount = options.edgeCount;
else
    edgeCount = [];
    for labelIdx = 1:numel(labelOptions)
        if isstruct(labelOptions{labelIdx}) && ...
                ~isempty(labelOptions{labelIdx}) && ...
                isfield(labelOptions{labelIdx}, 'activeEdgeMask')
            edgeCount = numel(labelOptions{labelIdx}(1).activeEdgeMask);
            break;
        end
    end
    if isempty(edgeCount)
        edgeCount = 0;
    end
end
if ~isscalar(edgeCount) || ~isfinite(edgeCount) || ...
        edgeCount < 0 || edgeCount ~= round(edgeCount)
    error('ReceiverSafeSelector:InvalidEdgeCount', ...
        'edgeCount must be a nonnegative integer.');
end
end

function value = resolveEdgeVector(options, fieldName, defaultValue, edgeCount)
value = getField(options, fieldName, defaultValue);
if isscalar(value) && edgeCount > 1
    value = repmat(value, 1, edgeCount);
end
value = reshape(value, 1, []);
if numel(value) ~= edgeCount
    error('ReceiverSafeSelector:InvalidEdgeVector', ...
        '%s must contain one value per edge.', fieldName);
end
end

function state = makeState(activeEdgeMask, costBytes, distortion, choices)
state = struct( ...
    'activeEdgeMask', logical(reshape(activeEdgeMask, 1, [])), ...
    'costBytes', costBytes, ...
    'distortion', distortion, ...
    'selectedOptionIndices', reshape(choices, 1, []));
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
