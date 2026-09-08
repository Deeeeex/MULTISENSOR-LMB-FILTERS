function [decoded,packet]=tcHistoryPacket(history,t,window)
% Explicit float64 history message, with counts and a lossless round trip.
first=max(1,t-window+1);
values=[20260908,history.source_id,t,first,t-first+1];
for k=first:t
    count=size(history.L{k},2);
    assert(size(history.X{k},2)==count);
    values=[values,k,count,reshape([history.L{k};history.X{k}],1,[])]; %#ok<AGROW>
end
packet=typecast(double(values),'uint8');
v=typecast(packet,'double');
assert(v(1)==20260908 && v(3)==t && v(4)==first);
decoded=struct('source_id',v(2),'X',{cell(t,1)},'L',{cell(t,1)},'N',zeros(t,1));
cursor=6;
for k=first:t
    assert(v(cursor)==k); count=v(cursor+1); cursor=cursor+2;
    assert(count>=0 && count==fix(count));
    block=reshape(v(cursor:cursor+6*count-1),6,count); cursor=cursor+6*count;
    decoded.L{k}=block(1:2,:); decoded.X{k}=block(3:6,:); decoded.N(k)=count;
    assert(isequal(decoded.L{k},history.L{k}) && isequal(decoded.X{k},history.X{k}));
end
assert(cursor==numel(v)+1);
end
