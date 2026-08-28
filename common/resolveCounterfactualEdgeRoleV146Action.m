function resolved = resolveCounterfactualEdgeRoleV146Action( ...
        action, formationCount)
% RESOLVECOUNTERFACTUALEDGEROLEV146ACTION Map a scale-free pattern to ranks.

required = { ...
    'index', 'name', 'formationRankPattern', 'workingPageOffsets', ...
    'maximumConsecutiveWorkingPages', 'initialReferencePageRequired', ...
    'onePayloadPerAttemptedEdge', 'crossFormationWorkingAllowed'};
if ~isstruct(action) || ~isscalar(action) || ...
        ~isequal(sort(fieldnames(action)), sort(required(:))) || ...
        ~isscalar(formationCount) || ~isfinite(formationCount) || ...
        formationCount < 2 || formationCount ~= round(formationCount)
    error('CounterfactualEdgeRoleV146:InvalidAction', ...
        'The scale-free role action or formation count is malformed.');
end
switch action.formationRankPattern
    case 'first-rank'
        selected = 1;
    case 'last-rank'
        selected = formationCount;
    case 'odd-ranks'
        selected = 1:2:formationCount;
    case 'even-ranks'
        selected = 2:2:formationCount;
    case 'upper-half'
        selected = 1:(formationCount / 2);
    case 'lower-half'
        selected = (formationCount / 2 + 1):formationCount;
    case 'all-ranks'
        selected = 1:formationCount;
    otherwise
        error('CounterfactualEdgeRoleV146:InvalidAction', ...
            'The role action uses an unregistered rank pattern.');
end
if mod(formationCount, 2) ~= 0 || isempty(selected) || ...
        any(selected < 1) || any(selected > formationCount)
    error('CounterfactualEdgeRoleV146:InvalidAction', ...
        'V146 requires an even formation count and nonempty rank set.');
end
resolved = action;
resolved.contractVersion = ...
    'v146-resolved-counterfactual-edge-role-action-v1';
resolved.formationCount = formationCount;
resolved.selectedFormationRanks = reshape(selected, 1, []);
end
