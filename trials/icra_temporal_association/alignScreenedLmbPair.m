function [aligned,d,state]=alignScreenedLmbPair(local,remote,threshold,state,mode,t,receiver,minimumQuality,cutoffs)
% Original kinematic assignment with persistent, observation-verified conflicts.
assert(ismember(mode,{'reopen','split'}) && threshold==50 && ismember(receiver,[1,2]));
[e,state.snapshots]=observationScreenEvidence(local,remote,state.snapshots,t,minimumQuality,cutoffs);
left=e.left;right=e.right;n=numel(local);m=numel(remote);
known=false(n,m);for i=1:n,known(i,:)=all(right==left(:,i),1);end
eligible=known & e.qualified & e.count>=2;
enter=eligible & e.currentCost>1;clear=eligible & e.currentCost<=1;
enteredLabels=left(:,any(enter,2));clearedLabels=left(:,any(clear,2));
blocked=unique([state.blocked,enteredLabels]','rows')';
if ~isempty(blocked)
    blocked=blocked(:,~ismember(blocked',clearedLabels','rows'));
    blocked=blocked(:,ismember(blocked',[left,right]','rows'));
end
state.blocked=blocked;
localBlocked=ismember(left',blocked','rows');
reopened=known & localBlocked;
locked=known & ~reopened;
d=struct('knownPairs',nnz(locked),'assignedPairs',0,'unmatchedLocal',n,'unmatchedRemote',m, ...
    'maximumMatchedCost',0,'keptRemoteIndices',1:m,'abstainLabels',zeros(2,0), ...
    'remoteOnlyLabels',zeros(2,0),'splitRecords',zeros(0,4),'conflictState',blocked, ...
    'reopenedKnown',nnz(reopened),'droppedRemote',0,'qualifiedPairs',nnz(e.qualified), ...
    'historyPairs',nnz(e.count>1),'pairs',zeros(0,6),'reopenedRecords',zeros(0,6));
[ii,jj]=find(reopened);
for k=1:numel(ii)
    i=ii(k);j=jj(k);
    d.reopenedRecords(end+1,:)=[left(:,i)',e.currentCost(i,j),e.count(i,j),e.distance(i,j),e.sumDistance(i,j)]; %#ok<AGROW>
end
aligned=remote;chosen=false(n,m);
if ~any(reopened,'all')
    [aligned,old]=alignGaussianLmbPair(local,remote,threshold);
    d.knownPairs=old.knownPairs;d.assignedPairs=old.assignedPairs;
    d.maximumMatchedCost=old.maximumMatchedCost/100;
    alignedKeys=labelKeys(aligned);
    for i=1:n
        j=find(all(alignedKeys==left(:,i),1));
        if ~isempty(j),chosen(i,j)=true;end
    end
else
    chosen=locked;freeLeft=find(~any(locked,2));freeRight=find(~any(locked,1));
    a=numel(freeLeft);b=numel(freeRight);costs=zeros(a,b);
    if a>0 && b>0
        for i=1:a
            for j=1:b
                if reopened(freeLeft(i),freeRight(j)),costs(i,j)=1e8;
                else,costs(i,j)=legacyCost(local(freeLeft(i)),remote(freeRight(j)))/100;end
            end
        end
        augmented=max([1e6;costs(:)])*100*ones(a+b,a+b);
        augmented(1:a,1:b)=costs;
        for i=1:a,augmented(i,b+i)=.5;end
        for j=1:b,augmented(a+j,j)=.5;end
        augmented(a+1:end,b+1:end)=0;
        match=Hungarian(augmented);[ii,jj]=find(match(1:a,1:b)>0);
        for k=1:numel(ii)
            assert(costs(ii(k),jj(k))<=1+1e-8);
            chosen(freeLeft(ii(k)),freeRight(jj(k)))=true;
        end
        d.assignedPairs=numel(ii);
        if ~isempty(ii),d.maximumMatchedCost=max(costs(sub2ind([a,b],ii,jj)));end
    end
end
assert(~any(chosen & reopened,'all'));
[ii,jj]=find(chosen);
for k=1:numel(ii)
    i=ii(k);j=jj(k);aligned(j).birthTime=left(1,i);aligned(j).birthLocation=left(2,i);
    d.pairs(end+1,:)=[i,j,locked(i,j),legacyCost(local(i),remote(j))/100,e.count(i,j),e.distance(i,j)]; %#ok<AGROW>
end
unmatchedRight=~any(chosen,1);collision=ismember(right',left','rows')';
keep=true(1,m);
for j=find(unmatchedRight & collision)
    i=find(all(left==right(:,j),1));assert(isscalar(i) && reopened(i,j));
    retained=false;
    if strcmp(mode,'split') && enter(i,j) && right(2,j)<1e6
        alias=[right(1,j);1e9+1e6*receiver+right(2,j)];
        existing=labelKeys(aligned);other=true(1,m);other(j)=false;
        occupied=[left,existing(:,other & keep)];
        if ~any(all(occupied==alias,1))
            aligned(j).birthTime=alias(1);aligned(j).birthLocation=alias(2);
            d.remoteOnlyLabels(:,end+1)=alias;d.splitRecords(end+1,:)=[right(:,j)',alias']; %#ok<AGROW>
            retained=true;
        end
    end
    keep(j)=retained;
end
d.keptRemoteIndices=find(keep);d.droppedRemote=nnz(~keep);aligned=aligned(keep);
d.abstainLabels=left(:,localBlocked & ~any(chosen,2));
matched=nnz(chosen);d.unmatchedLocal=n-matched;d.unmatchedRemote=m-matched;
keys=labelKeys(aligned);assert(size(unique(keys','rows'),1)==numel(aligned));
assert(~any(ismember(d.remoteOnlyLabels',left','rows')));
end

function keys=labelKeys(objects)
keys=[reshape([objects.birthTime],1,[]);reshape([objects.birthLocation],1,[])];
end

function value=legacyCost(a,b)
P=a.Sigma{1};Q=b.Sigma{1};delta=a.mu{1}-b.mu{1};dimension=numel(delta);
value=max(0,.25*(trace(P\Q)+trace(Q\P)+delta'*(P\delta+Q\delta)-2*dimension));
end
