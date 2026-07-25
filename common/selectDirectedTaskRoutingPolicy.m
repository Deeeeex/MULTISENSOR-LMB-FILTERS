function [adjacency, details] = ...
    selectDirectedTaskRoutingPolicy(context, options)
% SELECTDIRECTEDTASKROUTINGPOLICY Online adapter for privileged diagnostics.
%
% This adapter intentionally remains an offline teacher because the
% underlying selector reads ground truth. It exposes a directed adjacency
% and receiver-specific KLA weights to the online filter so that realized
% E-OSPA and communication can falsify the architecture before a deployable
% local surrogate is trained.

if nargin < 2 || isempty(options)
    options = getField( ...
        context.triggerConfig, ...
        'topologyDirectedRoutingOptions', struct());
end
options.maxMessagesPerReceiver = getField( ...
    options, 'maxMessagesPerReceiver', 1);
if options.maxMessagesPerReceiver ~= 1
    error(['The online adapter currently supports exactly one directed ', ...
        'message per receiver.']);
end
[~, routeDetails] = selectDirectedTaskRoutingTeacher(context, options);
adjacency = routeDetails.adjacency;
details = routeDetails;
details.objective = routeDetails.meanRiskAfter;
details.candidateIndex = NaN;
details.selectionSeconds = routeDetails.selectionSeconds;
details.taskRisk = routeDetails.meanRiskAfter;
details.baselineTaskRisk = routeDetails.meanRiskBefore;
details.taskAdvantage = ...
    routeDetails.meanRiskBefore - routeDetails.meanRiskAfter;
details.taskRiskSpread = ...
    max(routeDetails.nodeRiskAfter) - min(routeDetails.nodeRiskAfter);
details.validCandidateCount = routeDetails.messageCount;
details.directed = true;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
