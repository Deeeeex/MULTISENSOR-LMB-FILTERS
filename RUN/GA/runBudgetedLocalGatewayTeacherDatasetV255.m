function [reportPath, result] = ...
        runBudgetedLocalGatewayTeacherDatasetV255(options)
% RUNBUDGETEDLOCALGATEWAYTEACHERDATASETV255 One seed, six local H=3 banks.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getBudgetedLocalGatewayTeacherV255Protocol();
seed = getField(options, 'seed', protocol.additionalTrainingSeeds(1));
if ~ismember(seed, protocol.allowedSeeds)
    error('BudgetedLocalGatewayTeacherV255:InvalidSeed', ...
        'The requested seed is not registered for V255 teacher data.');
end
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    repoRoot, protocol.outputRoot, sprintf('seed%d', seed)));
delegate = struct( ...
    'protocol', protocol, ...
    'presetName', protocol.allowedPresets{1}, ...
    'seed', seed, ...
    'resume', logical(getField(options, 'resume', true)), ...
    'writeReport', logical(getField(options, 'writeReport', true)), ...
    'outputRoot', outputRoot);
[reportPath, result] = ...
    runCausalGatewayEmbeddingV250H3Oracle(delegate);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
