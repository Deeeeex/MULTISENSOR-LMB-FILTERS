function values=directObservationSummary(W,measurements,Q,opportunity)
% Current detection geometry conditioned on a detected association.
% [association mass, mean x/y, covariance xx/xy/yy, peak weight, entropy].
values=zeros(numel(opportunity),8);
assert(isequal(size(Q),[2,2]) && size(measurements,1)==2);
if isempty(W) || isempty(measurements),return;end
assert(size(W,1)==numel(opportunity) && size(W,2)==size(measurements,2)+1);
assert(all(isfinite(W),'all') && all(W>=0,'all'));
for j=1:size(W,1)
    total=sum(W(j,:));
    if ~opportunity(j) || total<=0,continue;end
    weights=W(j,2:end)/total;mass=sum(weights);
    if mass<=0,continue;end
    weights=weights/mass;mu=measurements*weights';delta=measurements-mu;
    covariance=Q+(delta.*weights)*delta';covariance=(covariance+covariance')/2;
    positive=weights>0;entropy=-sum(weights(positive).*log(weights(positive)));
    values(j,:)=[mass,mu',covariance(1,1),covariance(2,1),covariance(2,2),max(weights),entropy];
end
assert(all(isfinite(values),'all'));
end
