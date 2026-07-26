function expandedNames = expandRollingSafeMatchedArmNames( ...
        armNames, scenarioConfig)
% EXPANDROLLINGSAFEMATCHEDARMNAMES Add complete candidate-matched controls.
%
% Each rolling-safe analytic candidate is expanded independently at its
% registered source weight. Existing names are retained once, in order.

if ischar(armNames)
    armNames = {armNames};
end
if ~iscell(armNames)
    error('Topology arm names must be a character vector or cell array.');
end
if ~isstruct(scenarioConfig) || ...
        ~isfield(scenarioConfig, 'sensorGroupIds')
    error('Rolling matched-control expansion needs sensorGroupIds.');
end
groupIds = reshape(scenarioConfig.sensorGroupIds, 1, []);
groups = unique(groupIds, 'stable');
if numel(groups) < 2
    error('Rolling matched controls need at least two formations.');
end

expandedNames = {};
for armIdx = 1:numel(armNames)
    expandedNames = appendStable( ...
        expandedNames, char(armNames{armIdx}));
end
for armIdx = 1:numel(armNames)
    mode = lower(strrep(char(armNames{armIdx}), '_', '-'));
    token = regexp(mode, ...
        '^rolling-safe-analytic-w([0-9]+)$', ...
        'tokens', 'once');
    if isempty(token)
        continue;
    end
    sourceWeight = str2double(token{1}) / 100;
    controls = buildRollingSafeMatchedControlArmNames( ...
        numel(groups), sourceWeight);
    for controlIdx = 1:numel(controls.allArmNames)
        expandedNames = appendStable( ...
            expandedNames, controls.allArmNames{controlIdx});
    end
end
end

function values = appendStable(values, value)
canonicalValue = canonicalMode(value);
canonicalValues = cellfun(@canonicalMode, values, ...
    'UniformOutput', false);
if ~any(strcmp(canonicalValues, canonicalValue))
    values{end + 1} = value;
end
end

function mode = canonicalMode(value)
mode = lower(strrep(char(value), '_', '-'));
end
