function grouped = collapseFusionWeightsByGroup(weights, groupIds)
% COLLAPSEFUSIONWEIGHTSBYGROUP Average receiver-to-group fusion mass.

if ~isnumeric(weights) || ndims(weights) ~= 2 || ...
        size(weights, 1) ~= size(weights, 2) || ...
        any(~isfinite(weights(:))) || any(weights(:) < -1e-12)
    error('Fusion-weight matrix must be finite, nonnegative, and square.');
end
nodeCount = size(weights, 1);
groupIds = reshape(groupIds, 1, []);
if numel(groupIds) ~= nodeCount || ...
        any(~isfinite(groupIds)) || isempty(groupIds)
    error('Fusion-weight group identifiers are invalid.');
end
groups = unique(groupIds, 'stable');
grouped = zeros(numel(groups));
for receiverGroupIdx = 1:numel(groups)
    receivers = find(groupIds == groups(receiverGroupIdx));
    for senderGroupIdx = 1:numel(groups)
        senders = groupIds == groups(senderGroupIdx);
        grouped(receiverGroupIdx, senderGroupIdx) = mean(sum( ...
            weights(receivers, senders), 2));
    end
end
end
