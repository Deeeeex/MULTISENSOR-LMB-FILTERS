function [mu,P,logIntegral,kept,allowed,aggregateFallback,ranks]=applyProjectedEvidence(objects,alpha,kappa)
% Use normalized contracting marginals when the full spatial ratio is rejected.
assert(numel(objects)==numel(alpha) && numel(alpha)==numel(kappa));
assert(all(alpha>=0) && abs(sum(alpha)-1)<1e-12 && all(kappa>=0));
J0=zeros(4);h0=zeros(4,1);c0=0;kept=kappa;allowed=true(size(kappa));ranks=zeros(size(kappa));
changes=cell(1,numel(objects));indices=find(tril(true(4)));
for j=1:numel(objects)
    o=objects{j};
    if isempty(o),assert(alpha(j)==0 && kappa(j)==0);continue;end
    assert(o.numberOfGmComponents==1);
    Q=(o.Sigma{1}+o.Sigma{1}')/2;m=o.mu{1};[R,flag]=chol(Q);assert(flag==0);
    J=Q\eye(4);J=(J+J')/2;h=J*m;
    c=-.5*(4*log(2*pi)+2*sum(log(diag(R)))+m'*h);
    J0=J0+alpha(j)*J;h0=h0+alpha(j)*h;c0=c0+alpha(j)*c;
    encoded=o.localSpatialLogRatio;assert(numel(encoded)==15 && all(isfinite(encoded)));
    dJ=zeros(4);dJ(indices)=encoded(1:10);dJ=dJ+tril(dJ,-1)';
    dh=encoded(11:14)';dc=encoded(15);
    priorJ=J-dJ;priorJ=(priorJ+priorJ')/2;
    tolerance=1e-10*max([1,norm(J,2),norm(priorJ,2)]);
    ranks(j)=4;
    if min(eig(dJ)) < -tolerance
        allowed(j)=false;
        [~,flag]=chol(priorJ);assert(flag==0);
        priorP=priorJ\eye(4);priorP=(priorP+priorP')/2;
        priorMean=priorJ\(h-dh);L=chol(priorP,'lower');
        whiteJ=L'*J*L;whiteJ=(whiteJ+whiteJ')/2;
        [U,values]=eig(whiteJ,'vector');assert(all(values>0));
        select=values>1+1e-10*max([1;abs(values)]);ranks(j)=sum(select);
        if any(select)
            basis=U(:,select);values=values(select);
            whiteH=L'*J*(m-priorMean);oneH=basis'*whiteH;
            whiteA=(basis.*(values'-1))*basis';
            inverse=L\eye(4);dJ=inverse'*whiteA*inverse;dJ=(dJ+dJ')/2;
            shift=inverse'*(basis*oneH);dh=dJ*priorMean+shift;
            marginalC=.5*sum(log(values)-oneH.^2./values);
            dc=marginalC-.5*priorMean'*dJ*priorMean-shift'*priorMean;
        else
            dJ=zeros(4);dh=zeros(4,1);dc=0;kept(j)=0;
        end
    end
    changes{j}={dJ,dh,dc};
end
J=J0;h=h0;c=c0;
for j=1:numel(objects)
    if kept(j)==0,continue;end
    change=changes{j};J=J+kept(j)*change{1};h=h+kept(j)*change{2};c=c+kept(j)*change{3};
end
J=(J+J')/2;
aggregateFallback=any(~isfinite(J),'all') || any(~isfinite(h)) || ~isfinite(c);
if ~aggregateFallback
    [R,flag]=chol(J);aggregateFallback=flag~=0;
    if ~aggregateFallback,aggregateFallback=rcond(J)<1e-12;end
end
if aggregateFallback,J=J0;h=h0;c=c0;kept(:)=0;[R,flag]=chol(J);assert(flag==0);end
P=J\eye(4);P=(P+P')/2;mu=J\h;
logIntegral=c+.5*(4*log(2*pi)-2*sum(log(diag(R)))+h'*mu);
assert(all(isfinite(mu)) && all(isfinite(P),'all') && isfinite(logIntegral));
end
