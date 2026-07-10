function owner = assertFusionSufficientPublishClaim(plan, config, claim)
% ASSERTFUSIONSUFFICIENTPUBLISHCLAIM Verify exclusive publisher ownership.

required = {'schema', 'batchIdentity', 'configSha256', ...
    'token', 'tokenSha256', 'claimDirectory'};
if ~isstruct(claim) || ~isscalar(claim)
    error('FusionSufficientPublishClaim:InvalidClaim', ...
        'Publish claim must be a scalar struct.');
end
for fieldIdx = 1:numel(required)
    if ~isfield(claim, required{fieldIdx})
        error('FusionSufficientPublishClaim:InvalidClaim', ...
            'Publish claim is missing %s.', required{fieldIdx});
    end
end
if ~strcmp(claim.schema, 'fusion-sufficient-publish-claim-v2') || ...
        ~strcmp(claim.batchIdentity, config.batchIdentity) || ...
        ~strcmp(claim.configSha256, config.configSha256) || ...
        ~strcmp(claim.claimDirectory, plan.publishClaimDirectory) || ...
        ~strcmp(fusionSufficientSha256Text(claim.token), ...
        claim.tokenSha256) || ...
        isempty(regexp(claim.token, '^[0-9a-f]{64}$', 'once'))
    error('FusionSufficientPublishClaim:Mismatch', ...
        'Publish claim does not match batch/config/token.');
end
if exist(plan.publishOwnerReceiptPath, 'file') ~= 2
    error('FusionSufficientPublishClaim:MissingOwnerReceipt', ...
        'Publish owner receipt is missing; manual audit is required.');
end
owner = parseReceipt(plan.publishOwnerReceiptPath);
requireValue(owner, 'claim_schema', ...
    'fusion-sufficient-publish-claim-v2');
requireValue(owner, 'batch_identity', config.batchIdentity);
requireValue(owner, 'config_sha256', config.configSha256);
requireValue(owner, 'claim_token_sha256', claim.tokenSha256);
end

function values = parseReceipt(path)
lines = strsplit(strtrim(fileread(path)), '\n');
values = struct();
for lineIdx = 1:numel(lines)
    token = regexp(lines{lineIdx}, '^([a-z0-9_]+)=(.*)$', ...
        'tokens', 'once');
    if isempty(token) || isfield(values, token{1})
        error('FusionSufficientPublishClaim:OwnerSyntax', ...
            'Publish owner receipt is invalid at line %d.', lineIdx);
    end
    values.(token{1}) = token{2};
end
end

function requireValue(values, name, expected)
if ~isfield(values, name) || ~strcmp(values.(name), expected)
    error('FusionSufficientPublishClaim:OwnerValue', ...
        'Publish owner field %s differs from expected.', name);
end
end
