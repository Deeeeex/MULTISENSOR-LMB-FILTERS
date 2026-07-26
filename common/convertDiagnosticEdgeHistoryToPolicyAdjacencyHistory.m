function policyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        diagnosticHistory)
% CONVERTDIAGNOSTICEDGEHISTORYTOPOLICYADJACENCYHISTORY Convert SxSxH.
%
% Diagnostic edge masks use sender rows and receiver columns. Policy
% adjacency uses receiver rows and sender columns. History pages remain
% ordered from oldest to newest.

if isempty(diagnosticHistory)
    if ndims(diagnosticHistory) > 3
        error('Diagnostic edge history must be S-by-S-by-H.');
    end
    sensorCount = size(diagnosticHistory, 1);
    policyHistory = false(sensorCount, sensorCount, 0);
    return;
end
if ndims(diagnosticHistory) > 3 || ...
        size(diagnosticHistory, 1) ~= size(diagnosticHistory, 2)
    error('Diagnostic edge history must be S-by-S-by-H.');
end
if ndims(diagnosticHistory) < 3
    diagnosticHistory = reshape( ...
        diagnosticHistory, size(diagnosticHistory, 1), ...
        size(diagnosticHistory, 2), 1);
end
policyHistory = permute(logical(diagnosticHistory), [2, 1, 3]);
sensorCount = size(policyHistory, 1);
for historyIdx = 1:size(policyHistory, 3)
    page = policyHistory(:, :, historyIdx);
    page(1:sensorCount+1:end) = false;
    policyHistory(:, :, historyIdx) = page;
end
end
