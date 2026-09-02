function reports = runIndependentM24GatewayTeacherBatchV252(options)
% RUNINDEPENDENTM24GATEWAYTEACHERBATCHV252 Sequential resumable dataset.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getIndependentM24GatewayTeacherV252Protocol();
seeds = reshape(getField(options, 'seeds', ...
    protocol.allowedSeeds), 1, []);
if isempty(seeds) || any(~ismember(seeds, protocol.allowedSeeds)) || ...
        numel(unique(seeds)) ~= numel(seeds)
    error('IndependentM24GatewayTeacherV252:InvalidBatchSeeds', ...
        'V252 batch seeds must be unique registered teacher seeds.');
end
reports = cell(1, numel(seeds));
for seedIdx = 1:numel(seeds)
    seed = seeds(seedIdx);
    fprintf('V252 dataset seed %d (%d/%d)\n', ...
        seed, seedIdx, numel(seeds));
    reports{seedIdx} = ...
        runIndependentM24GatewayTeacherDatasetV252(struct( ...
            'seed', seed, 'resume', true, 'writeReport', true));
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
