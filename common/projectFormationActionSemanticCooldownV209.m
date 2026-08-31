function [eligibleMask, details] = ...
        projectFormationActionSemanticCooldownV209( ...
            actions, history, currentTime, options)
% PROJECTFORMATIONACTIONSEMANTICCOOLDOWNV209 Remove redundant repeats.
%
% A semantic label action is keyed by action type, receiver formation and
% label; changing only the source does not make it a new repair.  Formation
% release is keyed by type and receiver formation.  No-op is never blocked.
% HISTORY contains only actions that were actually applied.

if nargin < 4 || isempty(options)
    options = struct();
end
cooldownPages = getField(options, 'cooldownPages', 1);
if ~isstruct(actions) || ...
        (~isempty(actions) && ...
         any(~isfield(actions, ...
            {'actionType', 'receiverFormationId', 'label'}))) || ...
        ~isstruct(history) || ...
        (~isempty(history) && ...
         any(~isfield(history, ...
            {'actionType', 'receiverFormationId', 'label', 'time'}))) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || currentTime < 1 || ...
        ~isscalar(cooldownPages) || ~isfinite(cooldownPages) || ...
        cooldownPages < 0 || cooldownPages ~= round(cooldownPages)
    error('FormationActionCooldownV209:InvalidInput', ...
        'Actions, applied history, time and cooldown must be valid.');
end

actions = reshape(actions, 1, []);
history = reshape(history, 1, []);
eligibleMask = true(1, numel(actions));
matchedHistoryIndex = zeros(1, numel(actions));
for actionIdx = 1:numel(actions)
    validateKey(actions(actionIdx), false);
    if strcmpi(actions(actionIdx).actionType, 'no-op')
        continue;
    end
    for historyIdx = numel(history):-1:1
        validateKey(history(historyIdx), true);
        age = currentTime - history(historyIdx).time;
        if age < 0
            error('FormationActionCooldownV209:FutureHistory', ...
                'Semantic action history cannot contain future entries.');
        end
        if age <= cooldownPages && ...
                sameSemanticKey(actions(actionIdx), history(historyIdx))
            eligibleMask(actionIdx) = false;
            matchedHistoryIndex(actionIdx) = historyIdx;
            break;
        end
    end
end

details = struct();
details.contractVersion = ...
    'formation-action-semantic-cooldown-v209-v1';
details.currentTime = currentTime;
details.cooldownPages = cooldownPages;
details.eligibleMask = eligibleMask;
details.blockedMask = ~eligibleMask;
details.matchedHistoryIndex = matchedHistoryIndex;
details.sourceIdentifierIgnoredInSemanticKey = true;
details.noOpAlwaysEligible = true;
details.truthUsed = false;
details.futureInformationUsed = false;
details.numericIdentifiersUsedAsLearnedFeatures = false;
end

function same = sameSemanticKey(action, previous)
same = strcmpi(action.actionType, previous.actionType) && ...
    action.receiverFormationId == previous.receiverFormationId;
if ~same || strcmpi(action.actionType, 'formation-release')
    return;
end
same = isequal(reshape(action.label, 2, 1), ...
    reshape(previous.label, 2, 1));
end

function validateKey(value, historyEntry)
validType = ischar(value.actionType) && ...
    ~isempty(strtrim(value.actionType));
validFormation = isscalar(value.receiverFormationId) && ...
    isfinite(value.receiverFormationId) && ...
    value.receiverFormationId >= 0 && ...
    value.receiverFormationId == round(value.receiverFormationId);
validLabel = isnumeric(value.label) && numel(value.label) == 2 && ...
    all(isfinite(value.label(:))) && ...
    all(value.label(:) == round(value.label(:)));
validTime = true;
if historyEntry
    validTime = isscalar(value.time) && isfinite(value.time) && ...
        value.time >= 1 && value.time == round(value.time);
end
if ~validType || ~validFormation || ~validLabel || ~validTime
    error('FormationActionCooldownV209:InvalidSemanticKey', ...
        'A semantic action key is malformed.');
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
