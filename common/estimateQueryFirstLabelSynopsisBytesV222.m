function estimate = estimateQueryFirstLabelSynopsisBytesV222( ...
        participantSensorIds, maximumQueriedLabels, options)
% ESTIMATEQUERYFIRSTLABELSYNOPSISBYTESV222 Bound sparse label discovery.
%
% A coordinator first chooses at most K label keys from the beneficiary's
% ordinary posterior.  It sends only those keys to the participating target
% receivers and common sources.  Each remote participant replies with at
% most one risk-only record per requested key.  This replaces the V188
% all-active-label synopsis with an O(K * participants) control exchange.

if nargin < 3 || isempty(options)
    options = struct();
end
participantSensorIds = unique( ...
    reshape(participantSensorIds, 1, []), 'stable');
queryHeaderBytes = getField(options, 'queryHeaderBytes', 16);
queryBytesPerLabelKey = getField(options, ...
    'queryBytesPerLabelKey', 4);
responseHeaderBytes = getField(options, 'responseHeaderBytes', 16);
responseBytesPerLabelRecord = getField(options, ...
    'responseBytesPerLabelRecord', 20);
values = [queryHeaderBytes, queryBytesPerLabelKey, ...
    responseHeaderBytes, responseBytesPerLabelRecord];
if isempty(participantSensorIds) || ...
        any(~isfinite(participantSensorIds)) || ...
        any(participantSensorIds ~= round(participantSensorIds)) || ...
        any(participantSensorIds < 1) || ...
        ~isscalar(maximumQueriedLabels) || ...
        ~isfinite(maximumQueriedLabels) || ...
        maximumQueriedLabels < 1 || ...
        maximumQueriedLabels ~= round(maximumQueriedLabels) || ...
        any(~isfinite(values)) || any(values < 0) || ...
        any(values ~= round(values))
    error('QueryFirstLabelSynopsisV222:InvalidInput', ...
        'The participant set or byte layout is malformed.');
end

remoteParticipantCount = max(numel(participantSensorIds) - 1, 0);
queryBytesPerRemote = queryHeaderBytes + ...
    maximumQueriedLabels * queryBytesPerLabelKey;
responseBytesPerRemote = responseHeaderBytes + ...
    maximumQueriedLabels * responseBytesPerLabelRecord;
attemptedQueryBytes = ...
    remoteParticipantCount * queryBytesPerRemote;
attemptedResponseBytes = ...
    remoteParticipantCount * responseBytesPerRemote;

estimate = struct();
estimate.contractVersion = ...
    'query-first-label-synopsis-byte-estimate-v222-v1';
estimate.participantSensorIds = participantSensorIds;
estimate.participantCount = numel(participantSensorIds);
estimate.remoteParticipantCount = remoteParticipantCount;
estimate.maximumQueriedLabels = maximumQueriedLabels;
estimate.queryHeaderBytes = queryHeaderBytes;
estimate.queryBytesPerLabelKey = queryBytesPerLabelKey;
estimate.responseHeaderBytes = responseHeaderBytes;
estimate.responseBytesPerLabelRecord = responseBytesPerLabelRecord;
estimate.queryBytesPerRemote = queryBytesPerRemote;
estimate.responseBytesPerRemote = responseBytesPerRemote;
estimate.attemptedQueryBytes = attemptedQueryBytes;
estimate.attemptedResponseBytes = attemptedResponseBytes;
estimate.totalAttemptedBytes = ...
    attemptedQueryBytes + attemptedResponseBytes;
estimate.fullCovarianceIncluded = false;
estimate.gmComponentsIncluded = false;
estimate.fixedLabelCap = true;
estimate.truthUsed = false;
estimate.futureInformationUsed = false;
estimate.semanticIdentifiersUsedAsModelFeatures = false;
estimate.evidenceBoundary = [ ...
    'The byte count is a deterministic upper bound for one coordinator ', ...
    'query and one response from every remote participant at the fixed ', ...
    'label cap. It does not prove that an online coordinator selects the ', ...
    'same label keys as an offline teacher.'];
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
