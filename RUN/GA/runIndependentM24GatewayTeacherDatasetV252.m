function [reportPath, result] = ...
        runIndependentM24GatewayTeacherDatasetV252(options)
% RUNINDEPENDENTM24GATEWAYTEACHERDATASETV252 One seed, six H=3 anchors.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getIndependentM24GatewayTeacherV252Protocol();
seed = getField(options, 'seed', protocol.trainingSeeds(1));
if ~ismember(seed, protocol.allowedSeeds)
    error('IndependentM24GatewayTeacherV252:InvalidSeed', ...
        'The requested seed is not registered for V252 teacher data.');
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
