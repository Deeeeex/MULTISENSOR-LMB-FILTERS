function aggregated = aggregateTemporalPosteriorSupportV198( ...
        currentPosteriors, previousPosteriorHistory, historyDepth)
% AGGREGATETEMPORALPOSTERIORSUPPORTV198 Max causal label support.

if nargin < 3 || isempty(historyDepth)
    historyDepth = 1;
end
if ~iscell(currentPosteriors) || ~isrow(currentPosteriors) || ...
        ~iscell(previousPosteriorHistory) || ...
        ~isrow(previousPosteriorHistory) || ...
        ~isscalar(historyDepth) || ~isfinite(historyDepth) || ...
        historyDepth ~= round(historyDepth) || historyDepth < 0
    error('TemporalSetRepairV198:InvalidSupportHistory', ...
        'The causal posterior-support history is malformed.');
end

nodeCount = numel(currentPosteriors);
aggregated = currentPosteriors;
firstPage = max(1, numel(previousPosteriorHistory) - historyDepth + 1);
for pageIdx = firstPage:numel(previousPosteriorHistory)
    page = previousPosteriorHistory{pageIdx};
    if ~iscell(page) || ~isrow(page) || numel(page) ~= nodeCount
        error('TemporalSetRepairV198:InvalidSupportHistory', ...
            'A causal posterior-support page has the wrong shape.');
    end
    for sensorIdx = 1:nodeCount
        aggregated{sensorIdx} = mergeSensorSupport( ...
            aggregated{sensorIdx}, page{sensorIdx});
    end
end
end

function merged = mergeSensorSupport(current, previous)
if ~isstruct(current) || ~isstruct(previous)
    error('TemporalSetRepairV198:InvalidSupportHistory', ...
        'A sensor posterior-support page is malformed.');
end
merged = current;
for previousIdx = 1:numel(previous)
    matchIdx = find(arrayfun(@(object) ...
        object.birthTime == previous(previousIdx).birthTime && ...
        object.birthLocation == previous(previousIdx).birthLocation, ...
        merged), 1);
    if isempty(matchIdx)
        merged(end + 1) = previous(previousIdx); %#ok<AGROW>
        matchIdx = numel(merged);
    end
    merged(matchIdx).detectionAssociationMass = max( ...
        boundedMass(merged(matchIdx)), boundedMass(previous(previousIdx)));
end
end

function value = boundedMass(object)
value = 0;
if isstruct(object) && isfield(object, 'detectionAssociationMass') && ...
        isscalar(object.detectionAssociationMass) && ...
        isfinite(object.detectionAssociationMass)
    value = min(max(object.detectionAssociationMass, 0), 1);
end
end
