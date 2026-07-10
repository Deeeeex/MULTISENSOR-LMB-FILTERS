function publishFusionSufficientBatchFailure( ...
    plan, config, reason, testAuthorization)
% PUBLISHFUSIONSUFFICIENTBATCHFAILURE Persist immutable burned-batch state.

if nargin < 4
    testAuthorization = struct();
end
testMode = isFusionSufficientInternalTestAuthorization(testAuthorization);
if testMode
    validMode = isfield(config, 'testOnly') && ...
        isequal(config.testOnly, true) && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-test-v2') && ...
        strcmp(config.workerSchema, ...
        'fusion-sufficient-seed-worker-test-v2') && ...
        strncmp(config.artifactStem, 'TEST_', 5) && ...
        strncmp(config.batchIdentity, 'test-', 5);
else
    validMode = isfield(config, 'testOnly') && ...
        isequal(config.testOnly, false) && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-v2');
end
if ~validMode
    error('FusionSufficientFailure:ExecutionMode', ...
        'Failure publisher rejects unauthorized production/test schema.');
end

if exist(plan.attemptDirectory, 'dir') ~= 7
    error('FusionSufficientFailure:MissingReservation', ...
        'Cannot publish failure before reservation acquisition.');
end
reason = regexprep(lower(char(reason)), '[^a-z0-9_.-]+', '-');
if isempty(reason)
    reason = 'unspecified-failure';
end
if exist(plan.batchPlanPath, 'file') == 2
    planHash = fusionSufficientSha256File(plan.batchPlanPath);
else
    planHash = 'unpublished';
end
text = sprintf([ ...
    'failure_schema=fusion-sufficient-batch-failed-v2\n', ...
    'batch_identity=%s\n', ...
    'config_sha256=%s\n', ...
    'plan_sha256=%s\n', ...
    'reason=%s\n'], ...
    config.batchIdentity, config.configSha256, planHash, reason);

if exist(plan.attemptFailedTombstonePath, 'file') ~= 2
    temporaryPath = [tempname(plan.attemptDirectory), '.failure'];
    cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
    writeText(temporaryPath, text);
    linkFusionSufficientFileNoClobber( ...
        temporaryPath, plan.attemptFailedTombstonePath);
end
if exist(plan.workerDirectory, 'dir') == 7 && ...
        exist(plan.workerFailedTombstonePath, 'file') ~= 2
    linkFusionSufficientFileNoClobber( ...
        plan.attemptFailedTombstonePath, ...
        plan.workerFailedTombstonePath);
end
end

function writeText(path, value)
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientFailure:WriteFailed', ...
        'Unable to create failure tombstone temporary file.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = uint8(value);
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientFailure:WriteFailed', ...
        'Unable to write complete failure tombstone.');
end
end

function deleteIfPresent(path)
if exist(path, 'file') == 2
    delete(path);
end
end
