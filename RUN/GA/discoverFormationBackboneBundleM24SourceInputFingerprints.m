function [manifest, matPath, reportPath] = ...
    discoverFormationBackboneBundleM24SourceInputFingerprints( ...
        outputDirectory)
% DISCOVERFORMATIONBACKBONEBUNDLEM24SOURCEINPUTFINGERPRINTS Truth-free input freeze.

if nargin < 1 || isempty(outputDirectory)
    repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    outputDirectory = fullfile(repoRoot, 'RUN', 'GA', ...
        'dynamic_topology', ...
        'formation_backbone_bundle_m24_input_discovery_v1');
end
pathResolution = ...
    assertFormationBackboneBundleM24CriticalFunctionPaths();
protocol = getFormationBackboneBundleM24DevelopmentProtocol();
if protocol.sourcePosteriorAuthorized || ...
        protocol.executableSourceFrozen || ...
        protocol.sourceInputFingerprintsFrozen
    error('FormationBundleInputDiscovery:ProtocolAlreadyFrozen', ...
        'Input discovery is only valid for the freeze-pending protocol.');
end
assertOutputDirectoryIsNew(outputDirectory);
mkdir(outputDirectory);
outputCleanup = onCleanup(@() removeEmptyDirectory(outputDirectory));
headCommit = resolveHeadCommit();
gitState = validateFormationBackboneBundleM24ExecutableGitState( ...
    headCommit);
sourceManifest = ...
    buildFormationBackboneBundleM24ExecutableSourceManifest();

caseRecordCells = cell(1, protocol.caseCount);
for caseIdx = 1:protocol.caseCount
    caseContract = protocol.cases(caseIdx);
    timerId = tic;
    source = prepareFormationBackboneBundleM24SourceCaseInputs( ...
        caseContract.presetName, caseContract.seed);
    if ~strcmp(source.caseId, caseContract.id) || ...
            ~isequal(source.sourceWindow, caseContract.sourceWindow)
        error('FormationBundleInputDiscovery:CaseDrift', ...
            'Prepared source case order or identity changed.');
    end
    record = struct();
    record.ordinal = caseContract.ordinal;
    record.caseId = caseContract.id;
    record.caseCanonicalSha256 = ...
        caseContract.caseCanonicalSha256;
    record.presetName = caseContract.presetName;
    record.seed = caseContract.seed;
    record.filterSeed = caseContract.seed + ...
        protocol.filterSeedOffset;
    record.sourceWindow = caseContract.sourceWindow;
    record.generatorName = ...
        'generateDynamicTopologyScenarioInputs';
    record.emptyOverrideCanonicalSha256 = ...
        computeCanonicalValueSha256(struct());
    record.inputFingerprint = source.inputFingerprint;
    record.generationSeconds = toc(timerId);
    caseRecordCells{caseIdx} = record;
    fprintf('Input fingerprint %d/%d: %s\n', ...
        caseIdx, protocol.caseCount, caseContract.id);
end
caseRecords = [caseRecordCells{:}];
assertExecutionStateUnchanged( ...
    gitState, sourceManifest, 'after input discovery');

payload = struct();
payload.contractVersion = ...
    'formation-backbone-bundle-m24-input-discovery-manifest-v1';
payload.protocolId = protocol.id;
payload.freezePendingProtocolCanonicalSha256 = ...
    protocol.canonicalSha256;
payload.executableSourceCommit = headCommit;
payload.executableSourceManifestSha256 = ...
    sourceManifest.canonicalSha256;
payload.pathResolutionCanonicalSha256 = ...
    pathResolution.canonicalSha256;
payload.caseOrdering = protocol.caseOrdering;
payload.caseCount = protocol.caseCount;
payload.cases = caseRecords;
payload.trackingOutcomeScoringAuthorized = false;
payload.groundTruthAccessAuthorized = false;
payload.groundTruthPassedToFilter = false;
payload.futureOutcomeAccessAuthorized = false;
payload.validationClaimAllowed = false;
manifest = payload;
manifest.canonicalSha256 = computeCanonicalValueSha256(payload);

matPath = fullfile(outputDirectory, ...
    'formation_backbone_bundle_m24_input_discovery_v1.mat');
reportPath = fullfile(outputDirectory, ...
    'formation_backbone_bundle_m24_input_discovery_v1.md');
save('-mat7-binary', matPath, 'manifest');
writeReport(reportPath, manifest);
clear outputCleanup;
fprintf('M24 input discovery: %s\n', reportPath);
end

function assertOutputDirectoryIsNew(path)
if exist(path, 'dir') ~= 0 || exist(path, 'file') ~= 0
    error('FormationBundleInputDiscovery:OutputExists', ...
        'Input discovery refuses to overwrite an existing path.');
end
end

function removeEmptyDirectory(path)
if exist(path, 'dir') ~= 0
    listing = dir(path);
    names = {listing.name};
    names = names(~ismember(names, {'.', '..'}));
    if isempty(names)
        rmdir(path);
    end
end
end

function headCommit = resolveHeadCommit()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
[status, value] = system(sprintf( ...
    'git -C %s rev-parse HEAD', shellQuote(repoRoot)));
if status ~= 0
    error('FormationBundleInputDiscovery:GitStateFailed', ...
        'The current commit could not be resolved.');
end
headCommit = strtrim(value);
end

function assertExecutionStateUnchanged(initialGitState, ...
        initialManifest, stageLabel)
currentGitState = ...
    validateFormationBackboneBundleM24ExecutableGitState( ...
        initialGitState.expectedExecutableCommit);
currentManifest = ...
    buildFormationBackboneBundleM24ExecutableSourceManifest();
if ~strcmp(currentGitState.headCommit, initialGitState.headCommit) || ...
        ~strcmp(currentManifest.canonicalSha256, ...
            initialManifest.canonicalSha256)
    error('FormationBundleInputDiscovery:SourceChangedDuringRun', ...
        'Executable source changed %s.', stageLabel);
end
end

function writeReport(path, manifest)
fid = fopen(path, 'w');
if fid < 0
    error('FormationBundleInputDiscovery:ReportWriteFailed', ...
        'The input-discovery report could not be created.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# M24 source-input fingerprint discovery\n\n');
fprintf(fid, '- Executable commit: `%s`\n', ...
    manifest.executableSourceCommit);
fprintf(fid, '- Executable manifest SHA-256: `%s`\n', ...
    manifest.executableSourceManifestSha256);
fprintf(fid, '- Discovery manifest SHA-256: `%s`\n\n', ...
    manifest.canonicalSha256);
fprintf(fid, '| # | Preset | Seed | Window | Input SHA-256 | Seconds |\n');
fprintf(fid, '|--:|:--|--:|:--|:--|--:|\n');
for caseIdx = 1:numel(manifest.cases)
    record = manifest.cases(caseIdx);
    fprintf(fid, '| %d | `%s` | %d | `%s` | `%s` | %.2f |\n', ...
        record.ordinal, record.presetName, record.seed, ...
        mat2str(record.sourceWindow), ...
        record.inputFingerprint.canonicalSha256, ...
        record.generationSeconds);
end
fprintf(fid, ['\nThis artifact contains input identities only. ', ...
    'It authorizes no filter run and records no tracking outcome.\n']);
end

function value = shellQuote(value)
value = ['''', strrep(value, '''', '''"''"'''), ''''];
end
