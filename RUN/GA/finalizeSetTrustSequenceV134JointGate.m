function [resultPath, joint] = ...
    finalizeSetTrustSequenceV134JointGate(options)
% FINALIZESETTRUSTSEQUENCEV134JOINTGATE Aggregate M24 and X36 pilots.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getSetTrustSequenceV134Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('V134JointGate:DirtySource', ...
        'Finalize the V134 joint gate only from clean source.');
end
root = getField(options, 'outputRoot', protocol.outputRoot);
pilotCells = cell(1, numel(protocol.scaleCases));
for caseIdx = 1:numel(protocol.scaleCases)
    caseInfo = protocol.scaleCases(caseIdx);
    anchorTime = ...
        resolveSetTrustSequenceV134PilotAnchor(caseInfo, protocol);
    path = fullfile(root, 'pilot', ...
        strrep(caseInfo.presetName, '-', '_'), ...
        sprintf('seed%d_t%d', protocol.pilotSeed, anchorTime), ...
        'PILOT_RESULT_V134.mat');
    if exist(path, 'file') ~= 2
        error('V134JointGate:IncompletePilots', ...
            'Missing V134 scale pilot: %s', path);
    end
    loaded = load(path, 'pilot');
    if ~isfield(loaded, 'pilot')
        error('V134JointGate:InvalidPilotFile', ...
            'The scale pilot file lacks its registered payload: %s', path);
    end
    pilotCells{caseIdx} = loaded.pilot;
end
pilots = [pilotCells{:}];

joint = buildSetTrustSequenceV134JointGate( ...
    pilots, protocol, gitState.commit);
outputDirectory = fullfile(root, 'joint_gate');
if exist(outputDirectory, 'dir') ~= 7
    mkdir(outputDirectory);
end
resultPath = fullfile(outputDirectory, ...
    'JOINT_HEADROOM_GATE_V134.mat');
temporaryPath = [resultPath, '.partial'];
save('-mat7-binary', temporaryPath, 'joint');
[moved, message] = movefile(temporaryPath, resultPath, 'f');
if ~moved
    error('V134JointGate:WriteFailed', ...
        'Could not finalize the V134 joint gate: %s', message);
end
reportPath = fullfile(outputDirectory, ...
    'JOINT_HEADROOM_GATE_V134.md');
writeReport(reportPath, joint);
joint.resultPath = resultPath;
joint.reportPath = reportPath;
fprintf('V134 joint headroom gate: %s\n', reportPath);
end

function writeReport(path, joint)
fileId = fopen(path, 'w');
if fileId < 0
    error('V134JointGate:ReportWriteFailed', ...
        'Could not open the V134 joint-gate report.');
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V134 joint M24/X36 headroom gate\n\n');
fprintf(fileId, [ ...
    'Repository-only development evidence. Each scale must pass ', ...
    'independently; cross-scale averaging is forbidden.\n\n']);
fprintf(fileId, '| Scale | Carrier | Passing actions | Best passing action | ');
fprintf(fileId, 'Intervention | Full | Recovery | Bytes | Rho | Pass |\n');
fprintf(fileId, '|---|---|---|---|---:|---:|---:|---:|---:|---:|\n');
for caseInfo = joint.scaleCases
    if caseInfo.scaleGatePassed
        metrics = caseInfo.bestPassingMetrics;
        actionNames = strjoin(caseInfo.passingActionNames, ', ');
        fprintf(fileId, ...
            '| %s | `%s` | `%s` | `%s` | %+.3f%% | %+.3f%% | ', ...
            caseInfo.scaleName, caseInfo.referenceCarrierMode, ...
            actionNames, caseInfo.bestPassingActionName, ...
            100 * metrics.interventionGainFraction, ...
            100 * metrics.meanGainFraction);
        fprintf(fileId, '%+.3f%% | %+.3f%% | %.4f | 1 |\n', ...
            100 * metrics.matureGainFraction, ...
            100 * metrics.attemptedByteSavingFraction, ...
            metrics.recoverySquaredContractionFactor);
    else
        fprintf(fileId, '| %s | `%s` | - | - | - | - | - | - | - | 0 |\n', ...
            caseInfo.scaleName, caseInfo.referenceCarrierMode);
    end
end
fprintf(fileId, '\n- Joint cross-scale gate: `%d`\n', ...
    joint.jointCrossScaleGatePassed);
fprintf(fileId, '- On-policy teacher development authorized: `%d`\n', ...
    joint.onPolicyTeacherDevelopmentAuthorized);
fprintf(fileId, '- GNN authorized: `%d`\n', joint.gnnAuthorized);
fprintf(fileId, '- Validation claim allowed: `%d`\n', ...
    joint.validationClaimAllowed);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
