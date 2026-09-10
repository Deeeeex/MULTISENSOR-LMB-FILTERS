function checkPruneInformation()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');o=model.birthParameters(1);
o.birthTime=2;o.birthLocation=100001;o.mu={[1;2;3;4]};o.Sigma={diag([2,3,4,5])};
o.w=1;o.numberOfGmComponents=1;o.lastDirectOpportunity=19;o.positiveConfirmation=false;count=0;
for arm={'lineage','gaussian_evidence','gaussian_evidence_guarded_scalar'}
    rule=arm{1};
    for side=1:2
        for peerR=[.002,.02,.8]
            for q={[.5,.5],[.7,.3],[.3,.7]}
                row=zeros(1,60);row(1:6)=[20,1,2,100001,1,2];row(12:13)=q{1};row(14:15)=[.5,.5];
                row(18:19)=peerR;row(17+side)=.001;row(11)=-.02;row(57)=row(11);
                logits=logit(row(18:19));age=sum((row(12:13)-row(14:15)).*logits);
                beta=row(14:15);if ~strcmp(rule,'lineage') && age < -1e-12,beta=row(12:13);end
                row(29:30)=beta;row(30+2*(3-side):31+2*(3-side))=[2,100001];
                o.r=logistic(sum(beta.*logits)+row(11));row([7,10])=o.r;stats=struct('records',row,'weightChange',.4);
                inputs={o,o};inputs{side}=o([]);
                for actual=[0,1e-12,.0002,.001]
                    reports={zeros(0,4),zeros(0,4)};reports{side}=[2,100001,actual,.9];
                    packet=encodePruneInformation(reports{side},side,20);reports{side}=decodePruneInformation(packet,side,20);
                    [new,logged,e]=refinePruneInformation(o,stats,inputs,reports,rule,20,1);
                    lo=logits;lo(side)=logit(actual);b=[.5,.5];newAge=sum((q{1}-b).*lo);
                    if ~strcmp(rule,'lineage') && newAge < -1e-12,b=q{1};end
                    assert(abs(new.r-logistic(sum(b.*lo)-.02))<1e-14 && size(e,1)==1 && e(7)==side && e(8)==actual);
                    assert(logged.records(7)==new.r && logged.records(10)==new.r);
                    if actual==.001,assert(isequaln(new,o) && isequaln(logged,stats));else,assert(new.r<o.r);end
                    if side==1
                        [old,oldStats]=refineKnownCensor(o,stats,inputs,[2,100001,.003,actual,1,.9],rule,20,1);
                        assert(isequaln(new,old) && isequaln(logged,oldStats));
                    end
                    count=count+1;
                end
                for kind=1:4
                    reports={zeros(0,4),zeros(0,4)};reports{side}=[2,100001,.0002,.9];st=stats;in=inputs;
                    if kind==1,reports{side}=zeros(0,4);
                    elseif kind==2,reports{side}(1)=3;
                    elseif kind==3,in={o,o};
                    else,st.records(13+side)=0;end
                    [new,logged,e]=refinePruneInformation(o,st,in,reports,rule,20,1);
                    assert(isequaln(new,o) && isequaln(logged,st) && isempty(e));count=count+1;
                end
            end
        end
    end
end
for rows={zeros(0,4),[2,100001,0,.9;3,200002,.001,.1]}
    p=encodePruneInformation(rows{1},2,20);assert(isequal(rows{1},decodePruneInformation(p,2,20)));count=count+1;
end
good=encodePruneInformation([2,100001,.0002,.9],2,20);
for kind=1:13
    p=good;sender=2;t=20;
    if kind==1,p=p(1:end-1);
    elseif kind==2,sender=1;
    elseif kind==3,t=19;
    else
        v=typecast(p,'double');
        if kind==4,v(1)=0;elseif kind==5,v(4)=2;elseif kind==6,v(5)=2.5;
        elseif kind==7,v(5)=21;elseif kind==8,v(7)=.002;elseif kind==9,v(8)=0;
        elseif kind==10,v(7)=NaN;elseif kind==11,v(7)=-1;
        elseif kind==12,v(6)=0;else,v=[v,v(5:8)];v(4)=2;end
        p=typecast(v,'uint8');
    end
    failed=false;try,decodePruneInformation(p,sender,t);catch,failed=true;end;assert(failed);count=count+1;
end
[objects,stats,e]=refinePruneInformation(o([]),struct('records',zeros(0,60)),{o([]),o([])}, ...
    {zeros(0,4),zeros(0,4)},'lineage',20,1);assert(isempty(objects) && isempty(stats.records) && isempty(e));count=count+1;
fprintf('PRUNE INFORMATION CHECK PASSED: %d fixtures; byte parser, both sides, three backends, receiver-only parity and unchanged spatial fields.\n',count);
end

function x=logit(x),x=min(max(x,1e-9),1-1e-9);x=log(x)-log1p(-x);end
function x=logistic(x),if x>=0,x=1/(1+exp(-x));else,e=exp(x);x=e/(1+e);end;end
