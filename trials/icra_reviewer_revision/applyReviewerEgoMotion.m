function objects=applyReviewerEgoMotion(objects,transform)
% A predicted state uses the previous ego basis; map it to the current one.
% Velocity is a fixed-frame physical velocity expressed in that local basis.
assert(isequal(size(transform),[3,3]) && all(isfinite(transform),'all'));
assert(norm(transform(3,:)-[0,0,1],inf)<1e-12);
rotation=transform(1:2,1:2);translation=transform(1:2,3);
assert(norm(rotation'*rotation-eye(2),'fro')<1e-10 && abs(det(rotation)-1)<1e-10);
if isequal(transform,eye(3)),return;end
jacobian=blkdiag(rotation,rotation);
for j=1:numel(objects)
    for g=1:objects(j).numberOfGmComponents
        mu=objects(j).mu{g};P=objects(j).Sigma{g};
        assert(isequal(size(mu),[4,1]) && isequal(size(P),[4,4]));
        mu=jacobian*mu;mu(1:2)=mu(1:2)+translation;
        P=jacobian*P*jacobian';P=(P+P')/2;
        [~,flag]=chol(P);assert(flag==0 && all(isfinite(mu)));
        objects(j).mu{g}=mu;objects(j).Sigma{g}=P;
    end
end
end
