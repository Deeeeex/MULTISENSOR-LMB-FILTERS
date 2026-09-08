function [updated,diagnostics,W]=updateMarkedLmb(predicted,Z,model,sensor,t,ratios)
% A mark likelihood ratio modifies detection associations exactly once.
ratios=reshape(ratios,1,[]);
assert(numel(ratios)==numel(Z) && all(isfinite(ratios)) && all(ratios>0));
if isempty(Z)
    [updated,diagnostics,W]=updateLmbWithAssociationWeights(predicted,Z,model,sensor,t);
    return;
end
assert(~(isfield(model,'fovGaussianSplitting') && model.fovGaussianSplitting.enabled), ...
    'Marked control requires the registered unsplit local update.');
if model.sensorMotionEnabled
    [A,P]=generateLmbSensorAssociationMatrices(predicted,Z,model,sensor,t);
else
    [A,P]=generateLmbSensorAssociationMatrices(predicted,Z,model,sensor);
end
if all(ratios==1)
    % Preserve exact baseline diagnostic behavior in the identity fixture.
    [updated,diagnostics,W]=updateLmbWithAssociationWeights(predicted,Z,model,sensor,t);
    return;
end
A.Psi=A.Psi.*ratios;
L=A.L(:,2:end).*ratios;
A.L=[A.eta,L];A.P=L./(L+A.eta);A.C=-log(L);
if strcmp(model.dataAssociationMethod,'LBP')
    [r,W]=loopyBeliefPropagation(A,model.lbpConvergenceTolerance,model.maximumNumberOfLbpIterations);
elseif strcmp(model.dataAssociationMethod,'Gibbs')
    [r,W]=lmbGibbsSampling(A,model.numberOfSamples);
else
    [r,W]=lmbMurtysAlgorithm(A,model.numberOfAssignments);
end
updated=computePosteriorLmbSpatialDistributions(predicted,r,W,P,model);
% This control has no scheduling/trigger decision that consumes diagnostics.
diagnostics=struct('markedMeasurementModel',true,'minimumMarkRatio',min(ratios),'maximumMarkRatio',max(ratios));
end
