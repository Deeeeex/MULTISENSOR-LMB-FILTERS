function [adjacency, details] = ...
    selectTailGuardBackboneResidualSplicedCyclePolicy( ...
        context, options)
% SELECTTAILGUARDBACKBONERESIDUALSPLICEDCYCLEPOLICY Protect current max risk.
%
% Unlike the strict all-receiver guard, this robust projection constrains
% only the receiver with the largest matched-static expected task risk.
% That receiver cannot accept an edge-weight action whose risk exceeds its
% static residual reference.  Other receivers retain the adaptive-trust
% objective, so the global strong cycle remains feasible without discarding
% the established mean-performance headroom.

if nargin < 2 || isempty(options)
    options = struct();
end
options.referenceGuardMode = ...
    'top-risk-receivers';
options.protectedReceiverCount = getField(options, ...
    'protectedReceiverCount', 1);
[adjacency, details] = ...
    selectTailSafeBackboneResidualSplicedCyclePolicy( ...
        context, options);
details.mode = ...
    'backbone-residual-spliced-cycle-tail-guard-adaptive-current';
details.tailGuardObjective = ...
    'protect-current-maximum-matched-static-risk';
details.tailGuardProtectedReceiverCount = ...
    options.protectedReceiverCount;
details.tailGuardPrivilegedDiagnosticOnly = true;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
