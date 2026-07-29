function [analysisPath, analysis] = ...
    analyzeIidClusterRewardActionAlignmentM24(options)
% ANALYZEIIDCLUSTERREWARDACTIONALIGNMENTM24 Run the frozen second gate.

if nargin < 1 || isempty(options)
    options = struct();
end
options.rewardProtocol = ...
    getIidClusterRewardActionAlignmentProtocol();
[analysisPath, analysis] = ...
    analyzePredictiveRewardActionAlignmentM24(options);
end
