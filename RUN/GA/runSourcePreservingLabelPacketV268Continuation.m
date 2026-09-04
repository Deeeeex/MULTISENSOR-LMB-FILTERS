function [reportPath, result] = ...
        runSourcePreservingLabelPacketV268Continuation(options)
% RUNSOURCEPRESERVINGLABELPACKETV268CONTINUATION One two-hop packet arm.

if nargin < 1 || isempty(options), options = struct(); end
options.variant = 'v268';
[reportPath, result] = ...
    runLabelSelectiveRiskShortcutV265Continuation(options);
end
