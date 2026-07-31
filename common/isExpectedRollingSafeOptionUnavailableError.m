function expected = isExpectedRollingSafeOptionUnavailableError( ...
    errorInfo, allowedIdentifiers)
% ISEXPECTEDROLLINGSAFEOPTIONUNAVAILABLEERROR Narrow fail-closed allowlist.

if nargin < 2 || isempty(allowedIdentifiers)
    allowedIdentifiers = { ...
        'RollingMatching:Infeasible', ...
        'ObservableBasis:Unavailable'};
end
expected = isstruct(errorInfo) || isa(errorInfo, 'MException');
if ~expected || ~isfieldOrProperty(errorInfo, 'identifier')
    expected = false;
    return;
end
identifier = errorInfo.identifier;
expected = ischar(identifier) && ...
    any(strcmp(identifier, allowedIdentifiers));
end

function present = isfieldOrProperty(value, name)
if isstruct(value)
    present = isfield(value, name);
else
    present = isprop(value, name);
end
end
