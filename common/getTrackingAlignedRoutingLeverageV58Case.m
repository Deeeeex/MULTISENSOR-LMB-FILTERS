function entry = getTrackingAlignedRoutingLeverageV58Case( ...
        presetName, seed)
% GETTRACKINGALIGNEDROUTINGLEVERAGEV58CASE Resolve one exact V58 case.

protocol = getTrackingAlignedRoutingLeverageV58Protocol();
mask = strcmp({protocol.cases.presetName}, presetName) & ...
    [protocol.cases.seed] == seed;
if nnz(mask) ~= 1
    error('TrackingAlignedV58:UnregisteredCase', ...
        'The requested V58 preset/seed pair is not registered.');
end
entry = protocol.cases(find(mask, 1));
end
