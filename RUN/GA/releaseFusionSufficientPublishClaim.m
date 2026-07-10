function releaseFusionSufficientPublishClaim(plan, config, claim)
% RELEASEFUSIONSUFFICIENTPUBLISHCLAIM Release only the current failed owner.

assertFusionSufficientPublishClaim(plan, config, claim);
if exist(plan.publishedReceiptPath, 'file') == 2
    error('FusionSufficientPublishClaim:Published', ...
        'A successful PUBLISHED claim is immutable and cannot be released.');
end
for pathIdx = 1:numel(plan.finalPaths)
    if exist(plan.finalPaths{pathIdx}, 'file') == 2 || ...
            exist(plan.finalPaths{pathIdx}, 'dir') == 7
        error('FusionSufficientPublishClaim:UnsealedFinalArtifact', ...
            ['Cannot release a claim after final publication started; ', ...
            'manual audit is required: %s'], plan.finalPaths{pathIdx});
    end
end
backupPath = [tempname(plan.attemptDirectory), '.owner-release'];
cleanup = onCleanup(@() deleteIfPresent(backupPath)); %#ok<NASGU>
linkFusionSufficientFileNoClobber( ...
    plan.publishOwnerReceiptPath, backupPath);
delete(plan.publishOwnerReceiptPath);
[status, output] = system(sprintf('/bin/rmdir %s', ...
    shellQuote(plan.publishClaimDirectory)));
if status ~= 0
    if exist(plan.publishOwnerReceiptPath, 'file') ~= 2
        try
            linkFusionSufficientFileNoClobber( ...
                backupPath, plan.publishOwnerReceiptPath);
        catch restoreException
            warning('FusionSufficientPublishClaim:OwnerRestore', ...
                'Unable to restore owner receipt: %s', ...
                restoreException.message);
        end
    end
    error('FusionSufficientPublishClaim:ReleaseFailed', ...
        ['Publish claim contains unexpected files or changed ownership; ', ...
        'manual audit required: %s'], strtrim(output));
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
