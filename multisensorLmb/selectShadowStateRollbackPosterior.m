function [posterior, applied, pageIdx] = ...
        selectShadowStateRollbackPosterior( ...
            currentPosterior, config, receiverIdx, currentTime)
% SELECTSHADOWSTATEROLLBACKPOSTERIOR Apply a frozen node-time rollback.

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
page = config.shadowStateRollbackFusedPosteriorByTime{pageIdx};
if ~iscell(page) || numel(page) < receiverIdx
    error('ShadowStateRollbackV126:MissingShadowPosterior', ...
        'The requested node-time shadow posterior is unavailable.');
end
posterior = page{receiverIdx};
applied = true;
end
