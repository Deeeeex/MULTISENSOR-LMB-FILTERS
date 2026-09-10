function [objects,stats,events]=refinePruneInformation(original,originalStats,inputs,reports,rule,t,n)
% Reports are own current rows and delivered current decoded peer rows only.
assert(ismember(rule,{'lineage','gaussian_evidence','gaussian_evidence_guarded_scalar'}));
objects=original;stats=originalStats;events=zeros(0,24);
assert(numel(original)==size(originalStats.records,1) && numel(reports)==2);
for s=1:2,encodePruneInformation(reports{s},n+(3-2*n)*(s==2),t);end
for k=1:numel(original)
    row=originalStats.records(k,:);o=original(k);label=[o.birthTime,o.birthLocation];
    assert(isequal(row(1:4),[t,n,label]));
    present=false(1,2);
    for s=1:2,present(s)=any([inputs{s}.birthTime]==label(1) & [inputs{s}.birthLocation]==label(2));end
    assert(any(present));if all(present),continue;end
    s=find(~present);assert(numel(s)==1);
    if row(13+s)<=0,continue;end
    index=find(reports{s}(:,1)==label(1) & reports{s}(:,2)==label(2));assert(numel(index)<=1);
    if isempty(index),continue;end
    known=reports{s}(index,:);assert(row(17+s)==.001 && row(16-s)>0);
    b=row(14:15);q=row(12:13);oldLogit=logit(row(18:19));newLogit=oldLogit;
    newLogit(s)=logit(known(3));oldAge=sum((q-b).*oldLogit);age=sum((q-b).*newLogit);
    if strcmp(rule,'lineage'),oldBeta=b;beta=b;integral=row(11);
    else
        oldBeta=row(29:30);beta=b;if age < -1e-12,beta=q;end
        assert(all(row(53:54)==0));integral=row(57);assert(integral==row(11));
        assert(all(row(30+2*s:31+2*s)==0));
    end
    inherited=sum(oldBeta.*oldLogit);refined=sum(beta.*newLogit);
    assert(abs(logistic(inherited+integral)-o.r)<1e-14 && row(7)==o.r && row(10)==o.r);
    if known(3)~=row(17+s),objects(k).r=logistic(refined+integral);end
    assert(objects(k).r<=o.r+1e-14);stats.records(k,[7,10])=objects(k).r;
    events(end+1,:)=[t,n,label,o.r,objects(k).r,s,known(3:4),row(18:19),b,q,oldBeta,beta,oldAge,age,integral,inherited,refined]; %#ok<AGROW>
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
