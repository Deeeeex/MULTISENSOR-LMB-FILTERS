function ownership = acquireFusionSufficientBatchOwnership( ...
    plan, config, testAuthorization)
% ACQUIREFUSIONSUFFICIENTBATCHOWNERSHIP Atomically burn one batch identity.

if nargin < 3
    testAuthorization = struct();
end
testMode = isFusionSufficientInternalTestAuthorization(testAuthorization);
validateExecutionMode(config, testMode);
if ~testMode
    assertFusionSufficientConfigProvenance(config);
end

expectedPlan = buildFusionSufficientParallelPlan(config);
if ~isequaln(orderfields(plan), orderfields(expectedPlan))
    error('FusionSufficientOwnership:PlanMismatch', ...
        'Ownership request differs from deterministic plan.');
end
for pathIdx = 1:numel(plan.finalPaths)
    if exist(plan.finalPaths{pathIdx}, 'file') == 2 || ...
            exist(plan.finalPaths{pathIdx}, 'dir') == 7
        error('FusionSufficientOwnership:FinalArtifactExists', ...
            'Final artifact already exists: %s', plan.finalPaths{pathIdx});
    end
end

runShell(sprintf('/bin/mkdir -p %s', shellQuote(plan.attemptRoot)), ...
    'FusionSufficientOwnership:AttemptRoot');
reservationAcquired = false;
launcherCapability = randomHex256();
launcherCapabilitySha256 = ...
    fusionSufficientSha256Text(launcherCapability);
command = sprintf('/bin/mkdir %s 2>/dev/null', ...
    shellQuote(plan.attemptDirectory));
[status, ~] = system(command);
if status ~= 0
    error('FusionSufficientOwnership:ReservationExists', ...
        ['Batch identity %s is already reserved/burned; refusing any ', ...
        'new plan or worker publication.'], config.batchIdentity);
end
reservationAcquired = true;

try
    writeReservationReceipt( ...
        plan, config, launcherCapabilitySha256);
    workerRoot = fileparts(plan.workerDirectory);
    runShell(sprintf('/bin/mkdir -p %s', shellQuote(workerRoot)), ...
        'FusionSufficientOwnership:WorkerRoot');
    runShell(sprintf('/bin/mkdir %s', ...
        shellQuote(plan.workerDirectory)), ...
        'FusionSufficientOwnership:WorkerDirectory');
    writeLauncherCapability(plan, launcherCapability);
    publishBatchPlan(plan, config);
catch exception
    if reservationAcquired
        try
            publishFusionSufficientBatchFailure( ...
                plan, config, ['ownership-', exception.identifier], ...
                testAuthorization);
        catch
            % The stable reservation directory still burns the identity.
        end
    end
    rethrow(exception);
end
ownership = struct( ...
    'acquired', true, ...
    'batchIdentity', config.batchIdentity, ...
    'reservationPath', plan.attemptDirectory, ...
    'batchPlanPath', plan.batchPlanPath, ...
    'launcherCapabilitySha256', launcherCapabilitySha256);
end

function writeReservationReceipt(plan, config, launcherCapabilitySha256)
temporaryPath = [tempname(plan.attemptDirectory), '.reservation'];
cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
seedText = joinIntegers(plan.workerSeeds);
text = sprintf([ ...
    'reservation_schema=fusion-sufficient-batch-reservation-v2\n', ...
    'batch_identity=%s\n', ...
    'config_sha256=%s\n', ...
    'git_commit=%s\n', ...
    'launcher_capability_sha256=%s\n', ...
    'ordered_seeds=%s\n', ...
    'max_workers=%d\n'], ...
    config.batchIdentity, config.configSha256, config.gitCommit, ...
    launcherCapabilitySha256, ...
    seedText, config.maxWorkers);
writeText(temporaryPath, text);
linkFusionSufficientFileNoClobber( ...
    temporaryPath, plan.reservationReceiptPath);
end

function writeLauncherCapability(plan, capability)
temporaryPath = [tempname(plan.workerDirectory), '.capability'];
cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
writeText(temporaryPath, [capability, sprintf('\n')]);
linkFusionSufficientFileNoClobber( ...
    temporaryPath, plan.launcherCapabilityPath);
[status, output] = system(sprintf('/bin/chmod 600 %s', ...
    shellQuote(plan.launcherCapabilityPath)));
if status ~= 0
    error('FusionSufficientOwnership:CapabilityMode', ...
        'Unable to protect launcher capability: %s', strtrim(output));
end
end

function publishBatchPlan(plan, config)
temporaryPath = [tempname(plan.workerDirectory), '.plan.mat'];
cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
parallelPlan = plan; %#ok<NASGU>
save(temporaryPath, 'config', 'parallelPlan', '-v7');
loaded = load(temporaryPath);
if ~isfield(loaded, 'config') || ~isfield(loaded, 'parallelPlan') || ...
        ~isequaln(orderfields(loaded.config), orderfields(config)) || ...
        ~isequaln(orderfields(loaded.parallelPlan), orderfields(plan))
    error('FusionSufficientOwnership:PlanValidation', ...
        'Temporary batch plan failed round-trip validation.');
end
linkFusionSufficientFileNoClobber(temporaryPath, plan.batchPlanPath);
end

function runShell(command, identifier)
[status, output] = system(command);
if status ~= 0
    error(identifier, 'OS ownership operation failed: %s', ...
        strtrim(output));
end
end

function value = randomHex256()
[status, output] = system('openssl rand -hex 32');
value = lower(strtrim(output));
if status ~= 0 || isempty(regexp(value, '^[0-9a-f]{64}$', 'once'))
    error('FusionSufficientOwnership:RandomCapability', ...
        'Unable to generate launcher capability.');
end
end

function validateExecutionMode(config, testMode)
if testMode
    if ~isfield(config, 'testOnly') || ~isequal(config.testOnly, true) || ...
            ~strcmp(config.evidenceSchema, ...
            'fusion-sufficient-moment-exchange-test-v2') || ...
            ~strcmp(config.workerSchema, ...
            'fusion-sufficient-seed-worker-test-v2') || ...
            ~strncmp(config.artifactStem, 'TEST_', 5) || ...
            ~strncmp(config.batchIdentity, 'test-', 5)
        error('FusionSufficientOwnership:InvalidTestConfig', ...
            'Internal test authorization requires isolated test schemas.');
    end
else
    if ~isfield(config, 'testOnly') || ~isequal(config.testOnly, false) || ...
            ~strcmp(config.evidenceSchema, ...
            'fusion-sufficient-moment-exchange-v2') || ...
            ~strcmp(config.workerSchema, ...
            'fusion-sufficient-seed-worker-v2')
        error('FusionSufficientOwnership:ProductionSchema', ...
            'Production ownership rejects testOnly/test schemas.');
    end
end
end

function text = joinIntegers(values)
parts = arrayfun(@(value) sprintf('%d', value), values, ...
    'UniformOutput', false);
text = strjoin(parts, ',');
end

function writeText(path, value)
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientOwnership:WriteFailed', ...
        'Unable to create ownership receipt temporary file.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = uint8(value);
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientOwnership:WriteFailed', ...
        'Unable to write complete ownership receipt.');
end
end

function deleteIfPresent(path)
if exist(path, 'file') == 2
    delete(path);
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
