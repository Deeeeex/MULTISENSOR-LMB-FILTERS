function publishFusionSufficientEvidenceBundle( ...
    temporaryPaths, finalPaths, finalValidator)
% PUBLISHFUSIONSUFFICIENTEVIDENCEBUNDLE Rename with rollback on failure.

if ~iscell(temporaryPaths) || ~iscell(finalPaths) || ...
        numel(temporaryPaths) ~= numel(finalPaths) || ...
        isempty(temporaryPaths) || ~isa(finalValidator, 'function_handle')
    error('Invalid atomic evidence publication arguments.');
end
publishedPaths = {};
try
    for pathIdx = 1:numel(finalPaths)
        if exist(finalPaths{pathIdx}, 'file') == 2 || ...
                exist(finalPaths{pathIdx}, 'dir') == 7
            error('Refusing to overwrite evidence artifact: %s', ...
                finalPaths{pathIdx});
        end
        [ok, message] = movefile( ...
            temporaryPaths{pathIdx}, finalPaths{pathIdx});
        if ~ok
            error('Unable to publish %s: %s', ...
                finalPaths{pathIdx}, message);
        end
        publishedPaths{end+1} = finalPaths{pathIdx}; %#ok<AGROW>
    end
    finalValidator();
catch exception
    for pathIdx = 1:numel(publishedPaths)
        if exist(publishedPaths{pathIdx}, 'file') == 2
            delete(publishedPaths{pathIdx});
        end
    end
    rethrow(exception);
end
end
