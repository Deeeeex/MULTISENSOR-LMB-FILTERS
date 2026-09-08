function [encoded,packed]=gaussianLogRatio(priorMean,priorCovariance,postMean,postCovariance)
% Exact log ratio of the two supplied four-dimensional Gaussian approximations.
assert(numel(priorMean)==4 && numel(postMean)==4);
P0=(priorCovariance+priorCovariance')/2;P1=(postCovariance+postCovariance')/2;
[R0,f0]=chol(P0);[R1,f1]=chol(P1);assert(f0==0 && f1==0);
J0=P0\eye(4);J0=(J0+J0')/2;J1=P1\eye(4);J1=(J1+J1')/2;
h0=J0*priorMean;h1=J1*postMean;deltaJ=J1-J0;
deltaC=-.5*(2*sum(log(diag(R1)))-2*sum(log(diag(R0)))+postMean'*h1-priorMean'*h0);
indices=find(tril(true(4)));
encoded=[deltaJ(indices)',(h1-h0)',deltaC];
packed=[priorMean',P0(indices)',postMean',P1(indices)'];
assert(numel(encoded)==15 && numel(packed)==28 && all(isfinite(encoded)));
end
