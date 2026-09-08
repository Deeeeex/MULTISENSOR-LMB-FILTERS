function checkGaussianEvidence()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'trials','icra_asymmetric_evidence'));
checkAsymmetricEvidence();
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.2;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={diag([25,25,4,4])};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=0;o.positiveInnovationSupport=1;o.negativeInnovationSupport=1;
o.localSpatialLogRatio=zeros(1,15);model.birthParameters=o;model.object=o([]);prior=o;
cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
H=[eye(2),zeros(2)];Q=eye(2);scale=1000;
for ypair={[0,0;0,0],[1,2;-.5,1],[-9,9;0,0]}
    ys=ypair{1};objects=cell(1,2);
    for j=1:2
        m=prior.mu{1};P=prior.Sigma{1};S=H*P*H'+Q;K=P*H'/S;
        one=prior;one.mu={m+K*(ys(:,j)-H*m)};one.Sigma={P-K*S*K'};
        delta=log(scale)+logGaussian(ys(:,j),H*m,S);
        one.r=logistic(logit(prior.r)+delta);one.localLogOddsIncrement=delta;
        one.localSpatialLogRatio=gaussianLogRatio(m,P,one.mu{1},one.Sigma{1});objects{j}=one;
    end
    stackH=[H;H];stackY=ys(:);stackQ=blkdiag(Q,Q);
    P0=prior.Sigma{1};m0=prior.mu{1};A=P0\eye(4)+stackH'*(stackQ\stackH);
    expectedP=A\eye(4);expectedM=A\(P0\m0+stackH'*(stackQ\stackY));
    jointDelta=2*log(scale)+logGaussian(stackY,stackH*m0,stackH*P0*stackH'+stackQ);
    expectedR=logistic(logit(prior.r)+jointDelta);
    for arm={'gaussian_evidence','gaussian_evidence_no_curvature','gaussian_evidence_no_history','gaussian_evidence_no_mark'}
        [one,st]=fuseGaussianEvidence(objects,[.5,.5],model,d,cfg,arm{1},20);
        assert(max(abs(one.mu{1}-expectedM))<1e-10 && max(abs(one.Sigma{1}-expectedP),[],'all')<1e-10);
        assert(abs(one.r-expectedR)<1e-10 && abs(log(one.r)-log(expectedR))<1e-9);
        assert(all(st.records(53:54)==.5) && ~st.records(60));
    end
end
% Zero gate retains the original conservative base, including its spatial code.
o=objects{1};p=objects{2};o.positiveInnovationSupport=0;o.negativeInnovationSupport=0;
p.positiveInnovationSupport=0;p.negativeInnovationSupport=0;
[one,~]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[old,~]=fuseAsymmetricEvidence({o,p},[.5,.5],model,d,cfg,'asymmetric',20);
assert(one.r==old.r && isequal(one.mu,old.mu) && isequal(one.Sigma,old.Sigma));
[single,~]=fuseGaussianEvidence({o,p},[1,0],model,d,cfg,'gaussian_evidence',20);
[oldSingle,~]=fuseAsymmetricEvidence({o,p},[1,0],model,d,cfg,'asymmetric',20);
fprintf('SINGLE CHECK differences: r %.17g mu %.17g Sigma %.17g; new/old r %.17g mu %.17g Sigma %.17g\n', ...
    single.r-o.r,max(abs(single.mu{1}-o.mu{1})),max(abs(single.Sigma{1}-o.Sigma{1}),[],'all'), ...
    single.r-oldSingle.r,max(abs(single.mu{1}-oldSingle.mu{1})),max(abs(single.Sigma{1}-oldSingle.Sigma{1}),[],'all'));
assert(abs(single.r-o.r)<1e-12 && max(abs(single.mu{1}-o.mu{1}))<1e-12);
assert(single.r==oldSingle.r && isequal(single.mu,oldSingle.mu) && isequal(single.Sigma,oldSingle.Sigma));
% A widened approximate posterior is not a positive precision increment.
o=prior;o.r=.5;o.Sigma={1.5*prior.Sigma{1}};
o.localSpatialLogRatio=gaussianLogRatio(prior.mu{1},prior.Sigma{1},o.mu{1},o.Sigma{1});p=o;
[one,st]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
assert(all(st.records(53:54)==0) && ~any(st.records(58:59)) && isequal(one.Sigma,o.Sigma));
[wide,st]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence_no_curvature',20);
assert(all(st.records(53:54)==.5) && ~st.records(60) && max(abs(wide.Sigma{1}-3*prior.Sigma{1}),[],'all')<1e-10);
o.Sigma={2*prior.Sigma{1}};o.localSpatialLogRatio=gaussianLogRatio(prior.mu{1},prior.Sigma{1},o.mu{1},o.Sigma{1});p=o;
[one,st]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence_no_curvature',20);
assert(st.records(60) && all(st.records(53:54)==0) && isequal(one.Sigma,o.Sigma));
[decoded,bytes]=gaussianEvidencePacket([o,p],model,1,20);
assert(numel(bytes)==32+352*2 && isequal([decoded.localSpatialLogRatio],[o.localSpatialLogRatio,p.localSpatialLogRatio]));
[decoded,bytes]=gaussianEvidencePacket(model.object,model,1,20);
assert(isempty(decoded) && numel(bytes)==32);
fprintf('GAUSSIAN EVIDENCE CHECK PASSED: centralized Bernoulli Gaussian agreement/conflict, spatial and existence normalization, zero-gate/base and one-source behavior, curvature rejection, aggregate integrability fallback, and native 352 B round trip.\n');
end

function value=logGaussian(x,m,P)
R=chol(P);delta=x-m;value=-.5*(numel(x)*log(2*pi)+2*sum(log(diag(R)))+delta'*(P\delta));
end
function value=logit(p),value=log(p)-log1p(-p);end
function value=logistic(z)
if z>=0,value=1/(1+exp(-z));else,e=exp(z);value=e/(1+e);end
end
