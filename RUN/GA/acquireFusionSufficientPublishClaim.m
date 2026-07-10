function claim = acquireFusionSufficientPublishClaim( ...
    plan, config, testAuthorization)
% ACQUIREFUSIONSUFFICIENTPUBLISHCLAIM Atomically elect one publisher.

if nargin < 3
    testAuthorization = struct();
end
testMode = isFusionSufficientInternalTestAuthorization(testAuthorization);
validateExecutionMode(config, testMode);
if ~testMode
    assertFusionSufficientConfigProvenance(config);
end
if ~strcmp(inspectFusionSufficientBatchState(plan).name, ...
        'COMPLETE_WORKERS')
    error('FusionSufficientPublishClaim:WorkerState', ...
        'Publish claim requires COMPLETE_WORKERS.');
end
if exist(plan.publishedReceiptPath, 'file') == 2
    error('FusionSufficientPublishClaim:AlreadyPublished', ...
        'Evidence already has an immutable PUBLISHED receipt.');
end
for pathIdx = 1:numel(plan.finalPaths)
    if exist(plan.finalPaths{pathIdx}, 'file') == 2 || ...
            exist(plan.finalPaths{pathIdx}, 'dir') == 7
        error('FusionSufficientPublishClaim:UnsealedFinalArtifact', ...
            ['An unsealed final artifact exists. Refusing automatic ', ...
            'takeover; audit manually: %s'], plan.finalPaths{pathIdx});
    end
end
command = sprintf('/bin/mkdir %s 2>/dev/null', ...
    shellQuote(plan.publishClaimDirectory));
[status, ~] = system(command);
if status ~= 0
    ownerDescription = 'owner receipt unavailable';
    if exist(plan.publishOwnerReceiptPath, 'file') == 2
        ownerDescription = strtrim(fileread(plan.publishOwnerReceiptPath));
    end
    error('FusionSufficientPublishClaim:ClaimExists', ...
        ['Another publisher or a stale crash claim exists. Refusing ', ...
        'automatic takeover; audit manually. Owner:\n%s'], ...
        ownerDescription);
end

token = randomHex256();
tokenSha256 = fusionSufficientSha256Text(token);
pid = currentPid();
host = currentHost();
temporaryPath = [tempname(plan.publishClaimDirectory), '.owner'];
cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
text = sprintf([ ...
    'claim_schema=fusion-sufficient-publish-claim-v2\n', ...
    'batch_identity=%s\n', ...
    'config_sha256=%s\n', ...
    'claim_token_sha256=%s\n', ...
    'owner_pid=%d\n', ...
    'owner_host=%s\n'], ...
    config.batchIdentity, config.configSha256, tokenSha256, pid, host);
writeText(temporaryPath, text);
linkFusionSufficientFileNoClobber( ...
    temporaryPath, plan.publishOwnerReceiptPath);
claim = struct( ...
    'schema', 'fusion-sufficient-publish-claim-v2', ...
    'batchIdentity', config.batchIdentity, ...
    'configSha256', config.configSha256, ...
    'token', token, ...
    'tokenSha256', tokenSha256, ...
    'claimDirectory', plan.publishClaimDirectory, ...
    'ownerPid', pid, ...
    'ownerHost', host);
end

function validateExecutionMode(config, testMode)
if testMode
    valid = config.testOnly && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-test-v2') && ...
        strcmp(config.workerSchema, ...
        'fusion-sufficient-seed-worker-test-v2') && ...
        strncmp(config.artifactStem, 'TEST_', 5) && ...
        strncmp(config.batchIdentity, 'test-', 5);
else
    valid = ~config.testOnly && ...
        strcmp(config.evidenceSchema, ...
        'fusion-sufficient-moment-exchange-v2');
end
if ~valid
    error('FusionSufficientPublishClaim:ExecutionMode', ...
        'Publish claim rejects mismatched production/test schema.');
end
end

function value = randomHex256()
[status, output] = system('openssl rand -hex 32');
value = lower(strtrim(output));
if status ~= 0 || isempty(regexp(value, '^[0-9a-f]{64}$', 'once'))
    error('FusionSufficientPublishClaim:Random', ...
        'Unable to generate publisher token.');
end
end

function value = currentPid()
try
    value = getpid();
catch
    value = -1;
end
end

function value = currentHost()
value = strtrim(getenv('HOSTNAME'));
if isempty(value)
    [status, output] = system('hostname');
    if status == 0
        value = strtrim(output);
    end
end
if isempty(value)
    value = 'unknown-host';
end
value = regexprep(value, '[^A-Za-z0-9_.-]+', '-');
end

function writeText(path, value)
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientPublishClaim:Write', ...
        'Unable to write publisher owner receipt.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = uint8(value);
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientPublishClaim:Write', ...
        'Unable to write complete publisher owner receipt.');
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
