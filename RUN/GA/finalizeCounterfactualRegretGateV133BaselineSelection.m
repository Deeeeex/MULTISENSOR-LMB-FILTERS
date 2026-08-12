function [selectionPath, selection] = ...
    finalizeCounterfactualRegretGateV133BaselineSelection(options)
% FINALIZECOUNTERFACTUALREGRETGATEV133BASELINESELECTION Aggregate shards.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCounterfactualRegretGateV133Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('CounterfactualRegretV133:DirtySource', ...
        'Finalize V133 carrier selection only from clean source.');
end
outputDirectory = getField(options, 'outputDirectory', ...
    fullfile(protocol.outputRoot, 'baseline_selection'));
selectionPath = fullfile(outputDirectory, ...
    'FROZEN_REFERENCE_CARRIER_V133.mat');
reportPath = fullfile(outputDirectory, ...
    'FROZEN_REFERENCE_CARRIER_V133.md');

shards = cell(1, numel(protocol.scaleCases) * ...
    numel(protocol.developmentSeeds) * ...
    numel(protocol.referenceCarrierModes));
cursor = 0;
for caseInfo = protocol.scaleCases
    for seed = protocol.developmentSeeds
        for carrierIdx = 1:numel(protocol.referenceCarrierModes)
            carrierMode = protocol.referenceCarrierModes{carrierIdx};
            cursor = cursor + 1;
            path = fullfile(outputDirectory, 'shards', ...
                strrep(caseInfo.presetName, '-', '_'), sprintf( ...
                'V133_%s_SEED%d_%s.mat', ...
                upper(strrep(caseInfo.presetName, '-', '_')), ...
                seed, upper(strrep(carrierMode, '-', '_'))));
            if exist(path, 'file') ~= 2
                error('CounterfactualRegretV133:IncompleteBaselineShards', ...
                    'Missing V133 baseline shard: %s', path);
            end
            loaded = load(path, 'shard');
            if ~isfield(loaded, 'shard')
                error('CounterfactualRegretV133:InvalidBaselineShard', ...
                    'The baseline shard lacks its registered payload: %s', ...
                    path);
            end
            shards{cursor} = loaded.shard;
        end
    end
end
selection = buildCounterfactualRegretGateV133BaselineSelection( ...
    [shards{:}], protocol, gitState);
if exist(outputDirectory, 'dir') ~= 7
    mkdir(outputDirectory);
end
temporaryPath = [selectionPath, '.partial'];
save('-mat7-binary', temporaryPath, 'selection');
[moved, message] = movefile(temporaryPath, selectionPath, 'f');
if ~moved
    error('CounterfactualRegretV133:BaselineSelectionWriteFailed', ...
        'Could not finalize baseline selection: %s', message);
end
writeReport(reportPath, selection);
selection.reportPath = reportPath;
selection.selectionPath = selectionPath;
fprintf('V133 frozen carrier selection: %s\n', selectionPath);
end

function writeReport(path, selection)
fileId = fopen(path, 'w');
if fileId < 0
    error('CounterfactualRegretV133:ReportWriteFailed', ...
        'Could not open the V133 baseline report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V133 frozen reference carriers\n\n');
fprintf(fileId, ['Selection uses complete development trajectories, ', ...
    'paired inputs, and no per-state direction oracle.\n\n']);
fprintf(fileId, '| Scale | CW E-OSPA | CCW E-OSPA | Selected | Gain |\n');
fprintf(fileId, '|---|---:|---:|---|---:|\n');
for caseInfo = selection.cases
    fprintf(fileId, '| %s | %.4f | %.4f | %s | %.3f%% |\n', ...
        caseInfo.scaleName, caseInfo.meanEospaByCarrier(1), ...
        caseInfo.meanEospaByCarrier(2), ...
        caseInfo.selectedCarrierMode, ...
        caseInfo.selectionGainPercent);
end
fprintf(fileId, ...
    '\nThis is a repository method record, not a main-document result.\n');
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
