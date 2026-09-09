function [aligned,d,memory]=alignObservationLmbPair(local,remote,threshold,memory,mode,t)
% Association from current detection geometry and causal same-frame history.
assert(ismember(mode,{'direct','temporal'}) && threshold==50);
left=labelKeys(local);right=labelKeys(remote);n=numel(local);m=numel(remote);
assert(size(unique(left','rows'),1)==n && size(unique(right','rows'),1)==m);
featuresLeft=reshape([local.localDirectObservation],8,[])';
featuresRight=reshape([remote.localDirectObservation],8,[])';
qualityLeft=reshape([local.r],[],1).*featuresLeft(:,1).*featuresLeft(:,7);
qualityRight=reshape([remote.r],[],1).*featuresRight(:,1).*featuresRight(:,7);
qualified=(qualityLeft>=.5) & (qualityRight'>=.5);
distance=zeros(n,m);
if any(qualified,'all')
    dx=featuresLeft(:,2)-featuresRight(:,2)';dy=featuresLeft(:,3)-featuresRight(:,3)';
    xx=featuresLeft(:,4)+featuresRight(:,4)';xy=featuresLeft(:,5)+featuresRight(:,5)';
    yy=featuresLeft(:,6)+featuresRight(:,6)';determinant=xx.*yy-xy.^2;
    assert(all(determinant(qualified)>0));
    distance(qualified)=(yy(qualified).*dx(qualified).^2-2*xy(qualified).*dx(qualified).*dy(qualified)+ ...
        xx(qualified).*dy(qualified).^2)./determinant(qualified);
    distance=max(distance,0);
end
sumDistance=distance;count=double(qualified);
if strcmp(mode,'temporal')
    for h=1:numel(memory)
        old=memory(h);
        if t-old.frame<1 || t-old.frame>2,continue;end
        [hasLeft,indexLeft]=ismember(left',old.left','rows');
        [hasRight,indexRight]=ismember(right',old.right','rows');
        previousQualified=false(n,m);previousDistance=zeros(n,m);
        previousQualified(hasLeft,hasRight)=old.qualified(indexLeft(hasLeft),indexRight(hasRight));
        previousDistance(hasLeft,hasRight)=old.distance(indexLeft(hasLeft),indexRight(hasRight));
        use=qualified & previousQualified;
        count=count+double(use);sumDistance=sumDistance+previousDistance.*use;
    end
end
assert(all(count(:)>=0 & count(:)<=3));
cutoffs=[9.21034037197618,13.2767041359876,16.8118938297709];
currentCost=zeros(n,m);
currentCost(qualified)=sumDistance(qualified)./reshape(cutoffs(count(qualified)),[],1);
snapshot=struct('frame',t,'left',left,'right',right,'qualified',qualified,'distance',distance);
memory=[memory,snapshot];
memory=memory([memory.frame]>=t-2);
memory=memory(max(1,numel(memory)-1):numel(memory));
d=struct('knownPairs',0,'assignedPairs',0,'unmatchedLocal',n,'unmatchedRemote',m, ...
    'maximumMatchedCost',0,'keptRemoteIndices',1:m,'abstainLabels',zeros(2,0), ...
    'reopenedKnown',0,'droppedRemote',0,'qualifiedPairs',nnz(qualified), ...
    'historyPairs',nnz(count>1),'pairs',zeros(0,6),'reopenedRecords',zeros(0,6));
aligned=remote;
if n==0 || m==0,return;end
known=false(n,m);
for i=1:n,known(i,:)=all(right==left(:,i),1);end
reopened=known & qualified & currentCost>1;
locked=known & ~reopened;
d.knownPairs=nnz(locked);d.reopenedKnown=nnz(reopened);
[openedLeft,openedRight]=find(reopened);
for k=1:numel(openedLeft)
    i=openedLeft(k);j=openedRight(k);
    d.reopenedRecords(end+1,:)=[left(:,i)',currentCost(i,j),count(i,j),distance(i,j),sumDistance(i,j)]; %#ok<AGROW>
end
freeLeft=find(~any(locked,2));freeRight=find(~any(locked,1));
chosen=locked;costs=zeros(numel(freeLeft),numel(freeRight));
if ~any(qualified,'all')
    % Preserve exact original arithmetic and assignment in the no-signal case.
    [aligned,old]=alignGaussianLmbPair(local,remote,threshold);
    d.knownPairs=old.knownPairs;d.assignedPairs=old.assignedPairs;
    d.unmatchedLocal=old.unmatchedLocal;d.unmatchedRemote=old.unmatchedRemote;
    d.maximumMatchedCost=old.maximumMatchedCost/100;
    alignedKeys=labelKeys(aligned);
    for i=1:n
        jj=find(all(alignedKeys==left(:,i),1));
        if ~isempty(jj),d.pairs(end+1,:)=[i,jj,known(i,jj),legacyCost(local(i),remote(jj))/100,0,0];end %#ok<AGROW>
    end
    return;
end
if ~isempty(freeLeft) && ~isempty(freeRight)
    for a=1:numel(freeLeft)
        i=freeLeft(a);
        for b=1:numel(freeRight)
            j=freeRight(b);
            if qualified(i,j),costs(a,b)=currentCost(i,j);
            else,costs(a,b)=legacyCost(local(i),remote(j))/(2*threshold);end
        end
    end
    a=numel(freeLeft);b=numel(freeRight);
    augmented=max([1e6;costs(:)])*100*ones(a+b,a+b);
    augmented(1:a,1:b)=costs;
    for i=1:a,augmented(i,b+i)=.5;end
    for j=1:b,augmented(a+j,j)=.5;end
    augmented(a+1:end,b+1:end)=0;
    matching=Hungarian(augmented);[ii,jj]=find(matching(1:a,1:b)>0);
    for k=1:numel(ii)
        assert(costs(ii(k),jj(k))<=1+1e-8);
        chosen(freeLeft(ii(k)),freeRight(jj(k)))=true;
    end
    d.assignedPairs=numel(ii);
    if ~isempty(ii),d.maximumMatchedCost=max(costs(sub2ind([a,b],ii,jj)));end
end
assert(~any(chosen & reopened,'all'));
[ii,jj]=find(chosen);
for k=1:numel(ii)
    i=ii(k);j=jj(k);aligned(j).birthTime=left(1,i);aligned(j).birthLocation=left(2,i);
    if qualified(i,j),cost=currentCost(i,j);else,cost=legacyCost(local(i),remote(j))/100;end
    d.pairs(end+1,:)=[i,j,locked(i,j),cost,count(i,j),distance(i,j)]; %#ok<AGROW>
end
unmatchedRight=~any(chosen,1);
collision=ismember(right',left','rows')';
keep=~(unmatchedRight & collision);
d.keptRemoteIndices=find(keep);d.droppedRemote=nnz(~keep);aligned=aligned(keep);
abstain=any(reopened,2) & ~any(chosen,2);
d.abstainLabels=left(:,abstain);
matched=nnz(chosen);d.unmatchedLocal=n-matched;d.unmatchedRemote=m-matched;
alignedKeys=labelKeys(aligned);
assert(size(unique(alignedKeys','rows'),1)==numel(aligned));
end

function keys=labelKeys(objects)
keys=[reshape([objects.birthTime],1,[]);reshape([objects.birthLocation],1,[])];
end

function value=legacyCost(a,b)
assert(a.numberOfGmComponents==1 && b.numberOfGmComponents==1);
P=a.Sigma{1};Q=b.Sigma{1};delta=a.mu{1}-b.mu{1};d=numel(delta);
value=max(0,.25*(trace(P\Q)+trace(Q\P)+delta'*(P\delta+Q\delta)-2*d));
end
