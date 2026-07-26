function count = countCanonicalScheduledTreeControlPhases( ...
        modes, indices, rootPhaseCount, endpointPhaseCount)
% COUNTCANONICALSCHEDULEDTREECONTROLPHASES Count unique (root,endpoint).

if nargin < 3 || isempty(rootPhaseCount)
    rootPhaseCount = inf;
end
if nargin < 4 || isempty(endpointPhaseCount)
    endpointPhaseCount = inf;
end
pairs = zeros(0, 2);
for idx = reshape(indices, 1, [])
    if idx < 1 || idx > numel(modes)
        continue;
    end
    token = regexp(char(modes{idx}), ...
        ['^directed-scheduled-tree-r([0-9]+)-', ...
         'e([0-9]+)-w[0-9]+$'], ...
        'tokens', 'once');
    if isempty(token)
        continue;
    end
    rootPhase = str2double(token{1});
    endpointPhase = str2double(token{2});
    if rootPhase < 1 || rootPhase > rootPhaseCount || ...
            endpointPhase < 1 || ...
            endpointPhase > endpointPhaseCount
        continue;
    end
    pairs(end + 1, :) = [ ...
        rootPhase, endpointPhase]; %#ok<AGROW>
end
if isempty(pairs)
    count = 0;
else
    count = size(unique(pairs, 'rows'), 1);
end
end
