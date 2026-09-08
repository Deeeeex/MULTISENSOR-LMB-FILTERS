function [aligned,diagnostics]=alignGaussianLmbPair(local,remote,threshold)
% Gao arXiv:1911.01083v1 Eqs. (52)--(56), Gaussian specialization.
% Same known labels remain fixed. Independent labels get private unmatched
% slots; two unmatched costs compete against each proposed pair cost.
if nargin<3, threshold=50; end
aligned=remote;
diagnostics=struct('knownPairs',0,'assignedPairs',0,'unmatchedLocal',numel(local), ...
    'unmatchedRemote',numel(remote),'maximumMatchedCost',0);
if isempty(local) || isempty(remote), return; end
left=[[local.birthTime];[local.birthLocation]];
right=[[remote.birthTime];[remote.birthLocation]];
assert(size(unique(left','rows'),1)==numel(local));
assert(size(unique(right','rows'),1)==numel(remote));
known=false(numel(local),numel(remote));
for a=1:numel(local), known(a,:)=all(right==left(:,a),1); end
freeLeft=find(~any(known,2)); freeRight=find(~any(known,1));
diagnostics.knownPairs=nnz(known);
if ~isempty(freeLeft) && ~isempty(freeRight)
    n=numel(freeLeft); m=numel(freeRight); costs=zeros(n,m);
    for i=1:n
        a=local(freeLeft(i)); assert(a.numberOfGmComponents==1);
        for j=1:m
            b=remote(freeRight(j)); assert(b.numberOfGmComponents==1);
            P=a.Sigma{1}; Q=b.Sigma{1}; delta=a.mu{1}-b.mu{1}; d=numel(delta);
            costs(i,j)=max(0,.25*(trace(P\Q)+trace(Q\P)+delta'*(P\delta+Q\delta)-2*d));
        end
    end
    prohibit=max([1e6;costs(:)])*100;
    augmented=prohibit*ones(n+m,n+m);
    augmented(1:n,1:m)=costs;
    for i=1:n, augmented(i,m+i)=threshold; end
    for j=1:m, augmented(n+j,j)=threshold; end
    augmented(n+1:end,m+1:end)=0;
    matching=Hungarian(augmented); [ii,jj]=find(matching(1:n,1:m)>0);
    for k=1:numel(ii)
        assert(costs(ii(k),jj(k))<=2*threshold+1e-8);
        a=local(freeLeft(ii(k))); j=freeRight(jj(k));
        aligned(j).birthTime=a.birthTime; aligned(j).birthLocation=a.birthLocation;
    end
    diagnostics.assignedPairs=numel(ii);
    if ~isempty(ii), diagnostics.maximumMatchedCost=max(costs(sub2ind([n,m],ii,jj))); end
end
matched=diagnostics.knownPairs+diagnostics.assignedPairs;
diagnostics.unmatchedLocal=numel(local)-matched;
diagnostics.unmatchedRemote=numel(remote)-matched;
keys=[[aligned.birthTime];[aligned.birthLocation]];
assert(size(unique(keys','rows'),1)==numel(aligned));
end
