function [objects,stats,events]=refineKnownCensor(original,originalStats,inputs,cache,rule,t,n)
% Receiver-only scalar refinement; original eligibility and spatial pool stay.
assert(ismember(rule,{'lineage','gaussian_evidence','gaussian_evidence_guarded_scalar'}));
objects=original;stats=originalStats;events=zeros(0,25);
assert(numel(original)==size(originalStats.records,1) && size(cache,2)==6);
assert(size(unique(cache(:,1:2),'rows'),1)==size(cache,1));
for k=1:numel(original)
    row=originalStats.records(k,:);o=original(k);label=[o.birthTime,o.birthLocation];
    assert(isequal(row(1:4),[t,n,label]));
    own=any([inputs{1}.birthTime]==label(1) & [inputs{1}.birthLocation]==label(2));
    index=find(cache(:,1)==label(1) & cache(:,2)==label(2));assert(numel(index)<=1);
    if own || isempty(index) || cache(index,4)>.001 || ~cache(index,5) || row(14)<=0,continue;end
    known=cache(index,:);assert(known(4)>=0 && known(4)<=.001 && known(6)>0);
    assert(row(18)==.001 && row(15)>0);
    assert(any([inputs{2}.birthTime]==label(1) & [inputs{2}.birthLocation]==label(2)));
    b=row(14:15);q=row(12:13);oldLogit=logit(row(18:19));newLogit=oldLogit;
    newLogit(1)=logit(known(4));oldAge=sum((q-b).*oldLogit);age=sum((q-b).*newLogit);
    if strcmp(rule,'lineage'),oldBeta=b;beta=b;integral=row(11);
    else
        oldBeta=row(29:30);beta=b;if age < -1e-12,beta=q;end
        assert(all(row(53:54)==0) && all(row(32:33)==0));integral=row(57);
        assert(integral==row(11));
    end
    inherited=sum(oldBeta.*oldLogit);refined=sum(beta.*newLogit);
    assert(abs(logistic(inherited+integral)-o.r)<1e-14 && row(7)==o.r && row(10)==o.r);
    if known(4)~=row(18),objects(k).r=logistic(refined+integral);end
    assert(objects(k).r<=o.r+1e-14);
    stats.records(k,[7,10])=objects(k).r;
    events(end+1,:)=[t,n,label,o.r,objects(k).r,known(3:6),row(18:19),b,q,oldBeta,beta,oldAge,age,integral,inherited,refined]; %#ok<AGROW>
end
for k=1:numel(original)
    for f=setdiff(fieldnames(original(k)),{'r'})'
        assert(isequaln(original(k).(f{1}),objects(k).(f{1})));
    end
end
keep=setdiff(1:60,[7,10]);assert(isequaln(stats.records(:,keep),originalStats.records(:,keep)));
assert(all(isfinite(events),'all'));
end

function x=logit(x),x=min(max(x,1e-9),1-1e-9);x=log(x)-log1p(-x);end
function x=logistic(x),if x>=0,x=1/(1+exp(-x));else,e=exp(x);x=e/(1+e);end;end
