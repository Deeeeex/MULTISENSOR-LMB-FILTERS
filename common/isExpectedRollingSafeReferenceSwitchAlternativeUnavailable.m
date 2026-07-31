function value = ...
        isExpectedRollingSafeReferenceSwitchAlternativeUnavailable( ...
            errorOrIdentifier, actionCode, protocol)
% ISEXPECTEDROLLINGSAFEREFERENCESWITCHALTERNATIVEUNAVAILABLE Arm allowlist.
%
% The outer switch may legitimately report NoNominalFreeAction after both
% arms have been evaluated.  That identifier is never valid as the failure
% of one arm.  RequestedActionUnavailable is likewise restricted to the
% diverse-observable action family that can silently fall back.

if nargin < 3 || isempty(protocol)
    protocol = getLabelSetSimulatorPolicyProtocol();
end
if (isstruct(errorOrIdentifier) || ...
        isa(errorOrIdentifier, 'MException')) && ...
        isfieldOrProperty(errorOrIdentifier, 'identifier')
    identifier = errorOrIdentifier.identifier;
else
    identifier = errorOrIdentifier;
end
if ~ischar(identifier) || ~isscalar(actionCode) || ...
        ~isfinite(actionCode) || actionCode ~= round(actionCode)
    value = false;
    return;
end
baseIdentifiers = { ...
    'RollingMatching:Infeasible', ...
    'ObservableBasis:Unavailable'};
value = any(strcmp(identifier, baseIdentifiers));
if actionCode >= 82 && actionCode <= 87
    value = value || strcmp(identifier, ...
        'RollingSafeReferenceSwitch:RequestedActionUnavailable');
end
if value && ~any(strcmp(identifier, protocol. ...
        rollingSafeReferencePreferredSwitchExpectedUnavailableErrorIdentifiers))
    value = false;
end
end

function present = isfieldOrProperty(value, name)
if isstruct(value)
    present = isfield(value, name);
else
    present = isprop(value, name);
end
end
