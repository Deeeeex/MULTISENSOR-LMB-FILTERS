function windows = resolveDynamicTopologyBlockageWindows(config, graphData)
% RESOLVEDYNAMICTOPOLOGYBLOCKAGEWINDOWS Bind outages to actual backbone.
%
% Versioned multistyle scenes register only outage times.  The affected
% formation pairs are selected deterministically from the static reference
% tree that was constructed using t=1 geometry.  This prevents a numbered
% outage from silently targeting a pair that the reference never uses.

mode = lower(strrep(char(getField( ...
    config, 'blockageScheduleMode', 'explicit')), '_', '-'));
requireDistinctPairs = false;
switch mode
    case {'explicit', 'fixed'}
        windows = getField(config, 'blockageWindows', zeros(0, 4));
    case {'backbone-sequential', 'reference-backbone-sequential'}
        requireDistinctPairs = true;
        if ~isstruct(graphData) || ...
                ~isfield(graphData, 'formationBackbonePairs')
            error(['Backbone-sequential blockages require the resolved ', ...
                'formation reference tree.']);
        end
        pairs = sortrows(unique( ...
            double(graphData.formationBackbonePairs), 'rows'));
        times = double(getField( ...
            config, 'blockageWindowTimes', zeros(0, 2)));
        if size(pairs, 2) ~= 2 || isempty(pairs) || ...
                size(times, 2) ~= 2 || isempty(times) || ...
                size(times, 1) > size(pairs, 1)
            error(['Backbone-sequential blockage times cannot be ', ...
                'matched to distinct reference-tree pairs.']);
        end
        pairIndices = round(linspace( ...
            1, size(pairs, 1), size(times, 1)));
        if numel(unique(pairIndices)) ~= numel(pairIndices)
            error('Backbone-sequential blockage pair selection collapsed.');
        end
        windows = [pairs(pairIndices, :), times];
    otherwise
        error('Unknown blockageScheduleMode: %s', mode);
end

if isempty(windows)
    windows = zeros(0, 4);
    return;
end
if size(windows, 2) ~= 4 || any(~isfinite(windows(:))) || ...
        any(windows(:, 1:2) ~= round(windows(:, 1:2)), 'all') || ...
        any(windows(:, 1:2) < 1, 'all') || ...
        any(windows(:, 1:2) > config.formationCount, 'all') || ...
        any(windows(:, 1) == windows(:, 2)) || ...
        any(windows(:, 3) < 1) || ...
        any(windows(:, 4) < windows(:, 3)) || ...
        any(windows(:, 4) > config.simulationLength)
    error('Resolved dynamic-topology blockage windows are invalid.');
end
windows(:, 1:2) = sort(windows(:, 1:2), 2);
if requireDistinctPairs && ...
        size(unique(windows(:, 1:2), 'rows'), 1) ~= size(windows, 1)
    error('Resolved blockage windows must affect distinct formation pairs.');
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
