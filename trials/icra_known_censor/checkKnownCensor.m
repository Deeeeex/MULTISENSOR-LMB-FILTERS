function checkKnownCensor()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');o=model.birthParameters(1);
o.birthTime=2;o.birthLocation=100001;o.mu={[1;2;3;4]};o.Sigma={diag([2,3,4,5])};
o.w=1;o.numberOfGmComponents=1;o.lastDirectOpportunity=19;o.positiveConfirmation=false;
count=0;
for arm={'lineage','gaussian_evidence','gaussian_evidence_guarded_scalar'}
    rule=arm{1};
    for remoteR=[.0008,.02,.8]
        for q={[.5,.5],[.7,.3]}
            row=zeros(1,60);row(1:6)=[20,1,2,100001,1,2];row(12:13)=q{1};row(14:15)=[.5,.5];
            row(18:19)=[.001,remoteR];row(11)=-.02;row(57)=row(11);
            logits=logit(row(18:19));age=sum((row(12:13)-row(14:15)).*logits);
            beta=row(14:15);if ~strcmp(rule,'lineage') && age < -1e-12,beta=row(12:13);end
            row(29:30)=beta;row(34:35)=[2,100001];o.r=logistic(sum(beta.*logits)+row(11));
            row([7,10])=o.r;stats=struct('records',row,'weightChange',.4);
            for actual=[0,1e-12,.0002,.001]
                cache=[2,100001,.003,actual,1,.9];
                [new,logged,e]=refineKnownCensor(o,stats,{o([]),o},cache,rule,20,1);
                lo=logit([actual,remoteR]);b=[.5,.5];newAge=sum((q{1}-b).*lo);
                if ~strcmp(rule,'lineage') && newAge < -1e-12,b=q{1};end
                assert(abs(new.r-logistic(sum(b.*lo)-.02))<1e-14 && size(e,1)==1);
                assert(logged.records(7)==new.r && logged.records(10)==new.r && e(8)==actual);
                if actual==.001,assert(isequaln(new,o) && isequaln(logged,stats));else,assert(new.r<o.r);end
                count=count+1;
            end
            cache=[2,100001,.003,.0002,1,.9];
            for kind=1:5
                candidate=cache;st=stats;inputs={o([]),o};
                if kind==1,candidate=zeros(0,6);
                elseif kind==2,candidate(5:6)=0;
                elseif kind==3,inputs={o,o};
                elseif kind==4,st.records(14:15)=[0,1];
                else,candidate(4)=.002;end
                [new,logged,e]=refineKnownCensor(o,st,inputs,candidate,rule,20,1);
                assert(isequaln(new,o) && isequaln(logged,st) && isempty(e));count=count+1;
            end
        end
    end
end
[new,logged,e]=refineKnownCensor(o([]),struct('records',zeros(0,60)),{o([]),o([])},zeros(0,6),'lineage',20,1);
assert(isempty(new) && isempty(logged.records) && isempty(e));
fprintf('KNOWN CENSOR CHECK PASSED: %d fixtures; eligibility, all three backends, history branch, clipping, exact identity and spatial/metadata preservation.\n',count+1);
end

function x=logit(x),x=min(max(x,1e-9),1-1e-9);x=log(x)-log1p(-x);end
function x=logistic(x),if x>=0,x=1/(1+exp(-x));else,e=exp(x);x=e/(1+e);end;end
