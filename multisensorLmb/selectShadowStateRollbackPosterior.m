function [posterior, applied, pageIdx] = ...
        selectShadowStateRollbackPosterior( ...
            currentPosterior, config, receiverIdx, currentTime, ...
            localPosterior, independentAnchorPosterior)
% SELECTSHADOWSTATEROLLBACKPOSTERIOR Apply a registered node-time rollback.

if nargin < 5
    localPosterior = [];
end
if nargin < 6
    independentAnchorPosterior = [];
end

posterior = currentPosterior;
applied = false;
pageIdx = 0;
if ~config.shadowStateRollbackEnabled
    return;
end
pageIdx = find(config.shadowStateRollbackTimes == currentTime, 1);
if isempty(pageIdx) || ...
        ~ismember(receiverIdx, ...
            config.shadowStateRollbackSensorIdsByTime{pageIdx})
    pageIdx = 0;
    return;
end
sourceMode = 'external-fused-posterior';
if isfield(config, 'shadowStateRollbackSourceMode')
    sourceMode = lower(strrep(char( ...
        config.shadowStateRollbackSourceMode), '_', '-'));
end
if strcmp(sourceMode, 'current-local-posterior')
    if nargin < 5 || isempty(localPosterior)
        error('StateRollback:MissingLocalPosterior', ...
            'The current local posterior was not supplied.');
    end
    posterior = localPosterior;
elseif ismember(sourceMode, { ...
        'independent-local-anchor', 'light-network-anchor'})
    if nargin < 6 || isempty(independentAnchorPosterior)
        error('StateRollback:MissingAuxiliaryAnchor', ...
            'The requested auxiliary anchor posterior was not supplied.');
    end
    posterior = independentAnchorPosterior;
elseif strcmp(sourceMode, 'external-fused-posterior')
    page = config.shadowStateRollbackFusedPosteriorByTime{pageIdx};
    if ~iscell(page) || numel(page) < receiverIdx
        error('ShadowStateRollbackV126:MissingShadowPosterior', ...
            'The requested node-time shadow posterior is unavailable.');
    end
    posterior = page{receiverIdx};
else
    error('StateRollback:UnsupportedSourceMode', ...
        'The rollback source mode is not registered.');
end
applied = true;
end
