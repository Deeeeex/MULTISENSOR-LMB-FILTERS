function [evidence,memory]=observationPairEvidenceColumn(local,remote,memory,t)
% The unchanged V1 same-frame observation and received-history calculation.
mode='temporal';
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
currentCost(qualified)=reshape(sumDistance(qualified),[],1)./reshape(cutoffs(count(qualified)),[],1);
snapshot=struct('frame',t,'left',left,'right',right,'qualified',qualified,'distance',distance);
memory=[memory,snapshot];
memory=memory([memory.frame]>=t-2);
memory=memory(max(1,numel(memory)-1):numel(memory));
evidence=struct('left',left,'right',right,'qualified',qualified,'distance',distance, ...
    'sumDistance',sumDistance,'count',count,'currentCost',currentCost);
end

function keys=labelKeys(objects)
keys=[reshape([objects.birthTime],1,[]);reshape([objects.birthLocation],1,[])];
end
