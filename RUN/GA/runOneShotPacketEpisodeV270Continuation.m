function [reportPath, result] = ...
        runOneShotPacketEpisodeV270Continuation(options)
% RUNONESHOTPACKETEPISODEV270CONTINUATION One temporal causal ablation.

if nargin < 1 || isempty(options), options = struct(); end
options.variant = 'v270';
[reportPath, result] = ...
    runLabelSelectiveRiskShortcutV265Continuation(options);
end
