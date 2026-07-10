function publishFusionSufficientEvidenceBundle( ...
    temporaryPaths, finalPaths, finalValidator)
% PUBLISHFUSIONSUFFICIENTEVIDENCEBUNDLE No-clobber links with rollback.
% Publication is transactional for this call, but the three links are not
% a single atomic filesystem operation. The caller seals them afterward.

if ~iscell(temporaryPaths) || ~iscell(finalPaths) || ...
        numel(temporaryPaths) ~= numel(finalPaths) || ...
        isempty(temporaryPaths) || ~isa(finalValidator, 'function_handle')
    error('Invalid transactional evidence publication arguments.');
end
publishedPaths = {};
publicationSources = {};
try
    for pathIdx = 1:numel(finalPaths)
        if exist(finalPaths{pathIdx}, 'file') == 2 || ...
                exist(finalPaths{pathIdx}, 'dir') == 7
            error('Refusing to overwrite evidence artifact: %s', ...
                finalPaths{pathIdx});
        end
        linkFusionSufficientFileNoClobber( ...
            temporaryPaths{pathIdx}, finalPaths{pathIdx});
        publishedPaths{end+1} = finalPaths{pathIdx}; %#ok<AGROW>
        publicationSources{end+1} = temporaryPaths{pathIdx}; %#ok<AGROW>
        if ~sameFileIdentity(temporaryPaths{pathIdx}, finalPaths{pathIdx})
            error(['Published evidence target is not the hard-link ', ...
                'identity created by this publisher: %s'], ...
                finalPaths{pathIdx});
        end
    end
    finalValidator();
    for pathIdx = 1:numel(temporaryPaths)
        if exist(temporaryPaths{pathIdx}, 'file') == 2
            delete(temporaryPaths{pathIdx});
        end
    end
catch exception
    for pathIdx = 1:numel(publishedPaths)
        if exist(publishedPaths{pathIdx}, 'file') == 2 && ...
                sameFileIdentity( ...
                publicationSources{pathIdx}, publishedPaths{pathIdx})
            delete(publishedPaths{pathIdx});
        elseif exist(publishedPaths{pathIdx}, 'file') == 2
            warning('FusionSufficientPublish:RollbackOwnership', ...
                ['Rollback preserved %s because it is no longer the ', ...
                'hard-link identity created by this publisher.'], ...
                publishedPaths{pathIdx});
        end
    end
    rethrow(exception);
end
end

function value = sameFileIdentity(sourcePath, finalPath)
if exist(sourcePath, 'file') ~= 2 || exist(finalPath, 'file') ~= 2
    value = false;
    return;
end
[status, ~] = system(sprintf('/bin/test %s -ef %s', ...
    shellQuote(sourcePath), shellQuote(finalPath)));
value = status == 0;
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
