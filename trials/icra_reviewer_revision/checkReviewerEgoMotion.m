function checkReviewerEgoMotion()
% Geometric fixtures independent of real tracking truth or fusion outcomes.
o=struct('numberOfGmComponents',1,'mu',{{[10;3;2;-1]}}, ...
    'Sigma',{{[4,.5,.7,0;.5,2,0,.3;.7,0,9,.4;0,.3,.4,5]}},'r',.4);
assert(isequal(applyReviewerEgoMotion(o,eye(3)),o));
R=[0,1;-1,0];T=[R,[-2;4];0,0,1];got=applyReviewerEgoMotion(o,T);
assert(isequal(got.mu{1},[1;-6;-1;-2]) && got.r==o.r);
J=blkdiag(R,R);assert(norm(got.Sigma{1}-J*o.Sigma{1}*J','fro')<1e-12);
restored=applyReviewerEgoMotion(got,inv(T));
assert(norm(restored.mu{1}-o.mu{1})<1e-12 && norm(restored.Sigma{1}-o.Sigma{1},'fro')<1e-12);
% CV in a fixed world frame equals CV in the old ego basis followed by SE(2).
angle=.37;old=[cos(angle),-sin(angle),30;sin(angle),cos(angle),-5;0,0,1];
angle=-.21;current=[cos(angle),-sin(angle),31;sin(angle),cos(angle),-4;0,0,1];
dt=.1;A=[eye(2),dt*eye(2);zeros(2),eye(2)];
for velocity={[0;0],[3;-2]}
    worldPosition=[50;7];worldVelocity=velocity{1};
    localPosition=old\[worldPosition;1];localVelocity=old(1:2,1:2)'*worldVelocity;
    state=o;state.mu={A*[localPosition(1:2);localVelocity]};
    state=applyReviewerEgoMotion(state,current\old);
    targetPosition=current\[worldPosition+dt*worldVelocity;1];
    targetVelocity=current(1:2,1:2)'*worldVelocity;
    assert(norm(state.mu{1}-[targetPosition(1:2);targetVelocity])<1e-11);
end
fprintf('REVIEW EGO MOTION CHECK PASSED: identity, translation/rotation, full covariance, stationary/moving world points.\n');
end
