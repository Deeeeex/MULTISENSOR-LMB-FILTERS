function test_icassp_moment_exchange_runner()
% TEST_ICASSP_MOMENT_EXCHANGE_RUNNER Verify the frozen dry-run contract.

[reportPath, csvPath, summaryPath, summary, config] = ...
    runFusionSufficientMomentExchangeConfirmatory(false);
assert(isempty(reportPath));
assert(isempty(csvPath));
assert(isempty(summaryPath));
assert(isstruct(summary) && isempty(fieldnames(summary)));
assert(isequal(config.armSelection, { ...
    'Periodic full posterior', ...
    'Periodic light posterior on static topology'}));
assert(config.numberOfTrials == 50);
assert(config.baseSeed == 81);
assert(config.firstSeed == 82);
assert(config.lastSeed == 131);
assert(config.simulationLength == 100);
assert(~config.lightCovarianceInflationEnabled);
assert(~config.modeAwareFusionWeights);
assert(~config.includeDynamicTopologyVariants);
assert(config.capturePosteriorSnapshots);
assert(config.skipCalibrationForStaticPair);
assert(config.requiredMaxExistenceResidual == 0);
assert(config.requiredMaxMeanResidual == 0);
assert(config.requiredMaxCovarianceResidual == 0);
assert(config.bootstrapSeed == 20270710);
assert(config.bootstrapResamples == 10000);
assert(strcmp(config.evidenceSchema, ...
    'fusion-sufficient-moment-exchange-v1'));
assert(strcmp(config.artifactStem, ...
    'GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131'));
assert(~isempty(regexp(config.gitCommit, '^[0-9a-f]{40}$', 'once')));

assertThrows(@() ...
    runFusionSufficientMomentExchangeConfirmatory(false, 5, 1000));
assertThrows(@() ...
    runFusionSufficientMomentExchangeConfirmatory(true, 1, 1000));
assertThrows(@() ...
    runFusionSufficientMomentExchangeConfirmatory(true, 5, 999));
testGitProvenance();
fprintf('test_icassp_moment_exchange_runner passed\n');
end

function testGitProvenance()
projectRoot = tempname();
remoteRoot = [projectRoot, '-remote.git'];
[ok, message] = mkdir(projectRoot);
assert(ok, message);
cleanup = onCleanup(@() removeGitFixtures( ...
    projectRoot, remoteRoot)); %#ok<NASGU>
runShell(sprintf('git init -b main %s', shellQuote(projectRoot)));
runGit(projectRoot, 'config user.email codex-test@example.invalid');
runGit(projectRoot, 'config user.name Codex-Test');
requiredPaths = { ...
    'RUN/GA/runFusionSufficientMomentExchangeConfirmatory.m', ...
    'RUN/GA/assertFusionSufficientGitProvenance.m', ...
    'RUN/GA/renderFusionSufficientEvidenceReport.m', ...
    'RUN/GA/publishFusionSufficientEvidenceBundle.m', ...
    'RUN/GA/writeFusionSufficientEvidence.m', ...
    'RUN/GA/validateFusionSufficientEvidence.m', ...
    ['RUN/GA/runMultisensorFilters_formation_4plus4_', ...
        'DualThresholdEventTriggerCompare.m'], ...
    'multisensorLmb/runEventTriggeredDistributedLmbFilter.m', ...
    'multisensorLmb/snapshotLmbPosterior.m', ...
    'multisensorLmb/compareLmbPosteriorSnapshots.m', ...
    'multisensorLmb/projectLmbObjectMoments.m', ...
    'multisensorLmb/compressLmbPosterior.m', ...
    'multisensorLmb/fuseLmbPosteriorsByLabel.m', ...
    'multisensorLmb/getLmbWireSchema.m', ...
    'multisensorLmb/encodeLmbWireMessage.m', ...
    'multisensorLmb/decodeLmbWireMessage.m', ...
    'docs/icassp2027_paper/EXPERIMENT_PROTOCOL_CN.md'};
for pathIdx = 1:numel(requiredPaths)
    writeFixtureFile(projectRoot, requiredPaths{pathIdx}, 'fixture\n');
end
runGit(projectRoot, 'add .');
runGit(projectRoot, 'commit -m initial');
runShell(sprintf('git init --bare %s', shellQuote(remoteRoot)));
runGit(projectRoot, sprintf('remote add origin %s', ...
    shellQuote(remoteRoot)));
runGit(projectRoot, 'push -u origin main');

provenance = assertFusionSufficientGitProvenance(projectRoot);
assert(provenance.runtimeClean);
assert(strcmp(provenance.head, provenance.upstream));

writeFixtureFile(projectRoot, requiredPaths{1}, 'dirty\n');
assertThrowsIdentifier(@() ...
    assertFusionSufficientGitProvenance(projectRoot), ...
    'FusionSufficientProvenance:DirtyRuntime');
runGit(projectRoot, sprintf('restore -- %s', ...
    shellQuote(requiredPaths{1})));

untrackedPath = 'RUN/GA/untracked_runtime.m';
writeFixtureFile(projectRoot, untrackedPath, 'untracked\n');
assertThrowsIdentifier(@() ...
    assertFusionSufficientGitProvenance(projectRoot), ...
    'FusionSufficientProvenance:DirtyRuntime');
delete(fullfile(projectRoot, untrackedPath));

writeFixtureFile(projectRoot, 'README.md', 'ahead\n');
runGit(projectRoot, 'add README.md');
runGit(projectRoot, 'commit -m ahead');
assertThrowsIdentifier(@() ...
    assertFusionSufficientGitProvenance(projectRoot), ...
    'FusionSufficientProvenance:UnpublishedHead');
end

function writeFixtureFile(projectRoot, relativePath, text)
path = fullfile(projectRoot, relativePath);
directory = fileparts(path);
if exist(directory, 'dir') ~= 7
    [ok, message] = mkdir(directory);
    assert(ok, message);
end
fid = fopen(path, 'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', text);
end

function runGit(projectRoot, arguments)
runShell(sprintf('git -C %s %s', shellQuote(projectRoot), arguments));
end

function runShell(command)
[status, output] = system(command);
assert(status == 0, output);
end

function assertThrowsIdentifier(functionHandle, expectedIdentifier)
didThrow = false;
try
    functionHandle();
catch exception
    didThrow = true;
    assert(strcmp(exception.identifier, expectedIdentifier), ...
        exception.message);
end
assert(didThrow);
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end

function removeGitFixtures(projectRoot, remoteRoot)
if exist(projectRoot, 'dir') == 7
    rmdir(projectRoot, 's');
end
if exist(remoteRoot, 'dir') == 7
    rmdir(remoteRoot, 's');
end
end

function assertThrows(functionHandle)
didThrow = false;
try
    functionHandle();
catch
    didThrow = true;
end
assert(didThrow);
end
