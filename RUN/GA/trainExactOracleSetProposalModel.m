function [modelPath, result] = trainExactOracleSetProposalModel(options)
% TRAINEXACTORACLESETPROPOSALMODEL Frozen v2 constraint-generation run.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getRollingSafeStructuredProposalProtocol();
options.methodVariant = protocol.exactOracleMethodVariant;
[modelPath, result] = trainStructuredSetProposalModel(options);
end
