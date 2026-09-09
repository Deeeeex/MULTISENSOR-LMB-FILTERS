function checkDirectObservationSummary()
W=[.2,.6,.2;.1,.45,.45;1,0,0];z=[0,4;1,1];Q=eye(2);
values=directObservationSummary(W,z,Q,[true,true,true]);
assert(max(abs(values(1,1:7)-[.8,1,1,4,0,1,.75]))<1e-14);
assert(max(abs(values(2,1:7)-[.9,2,1,5,0,1,.5]))<1e-14);
assert(abs(values(2,8)-log(2))<1e-14 && all(values(3,:)==0));
empty=directObservationSummary([],zeros(2,0),Q,[true,true]);assert(all(empty==0,'all'));
disabled=directObservationSummary(W,z,Q,[false,false,false]);assert(all(disabled==0,'all'));
scaled=directObservationSummary(7*W,z,Q,[true,true,true]);assert(max(abs(scaled-values),[],'all')<1e-14);
rotation=[0,-1;1,0];translation=[7;-5];
transformed=directObservationSummary(W,rotation*z+translation,rotation*Q*rotation',[true,true,true]);
for j=1:2
    assert(norm(transformed(j,2:3)'-(rotation*values(j,2:3)'+translation))<1e-12);
    P=[values(j,4),values(j,5);values(j,5),values(j,6)];
    actual=[transformed(j,4),transformed(j,5);transformed(j,5),transformed(j,6)];
    assert(norm(actual-rotation*P*rotation','fro')<1e-12);
end
fprintf('DIRECT OBSERVATION CHECK PASSED\n');
end
