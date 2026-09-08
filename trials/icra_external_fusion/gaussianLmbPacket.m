function [decoded,bytes]=gaussianLmbPacket(objects,model,source,t)
values=[20260909,source,t,numel(objects)];
for k=1:numel(objects)
    o=objects(k); assert(o.numberOfGmComponents==1);
    values=[values,o.birthTime,o.birthLocation,o.r,double(o.hasObservationLineage), ...
        o.lastDirectOpportunity,double(o.positiveConfirmation),o.mu{1}',reshape(o.Sigma{1},1,[])]; %#ok<AGROW>
end
bytes=typecast(double(values),'uint8'); v=typecast(bytes,'double');
assert(v(1)==20260909 && v(2)==source && v(3)==t && v(4)==numel(objects));
decoded=model.object; cursor=5;
for k=1:numel(objects)
    o=model.birthParameters(1); o.birthTime=v(cursor); o.birthLocation=v(cursor+1);
    o.r=v(cursor+2); o.hasObservationLineage=logical(v(cursor+3));
    o.lastDirectOpportunity=v(cursor+4); o.positiveConfirmation=logical(v(cursor+5));
    o.numberOfGmComponents=1; o.w=1; o.mu={v(cursor+6:cursor+9)'};
    o.Sigma={reshape(v(cursor+10:cursor+25),4,4)};cursor=cursor+26;
    decoded(k)=o;
    assert(isequal(o.mu,objects(k).mu) && isequal(o.Sigma,objects(k).Sigma) && o.r==objects(k).r);
    assert(o.birthTime==objects(k).birthTime && o.birthLocation==objects(k).birthLocation);
    assert(o.lastDirectOpportunity<=t);
end
assert(cursor==numel(v)+1);
end
