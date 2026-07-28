function offline = isOfflineCounterfactualSequenceMode(mode)
% ISOFFLINECOUNTERFACTUALSEQUENCEMODE Identify codes 40:45 by pair.
%
% These truth-free action codes require centralized access to all current
% full posteriors. They are development-only diagnostic teachers, not
% deployment-observable rolling-safe candidates.

mode = lower(strrep(char(mode), '_', '-'));
offline = false;
token = regexp(mode, ...
    ['^rolling-safe-sequence-t[0-9]+-', ...
     's([0-9]+)-w[0-9]+$'], ...
    'tokens', 'once');
if isempty(token) || mod(numel(token{1}), 2) ~= 0
    return;
end
actionCodes = sscanf(token{1}, '%2d')';
offline = any(actionCodes >= 40 & actionCodes <= 45);
end
