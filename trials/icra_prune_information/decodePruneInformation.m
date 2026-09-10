function rows=decodePruneInformation(packet,sender,t)
assert(isa(packet,'uint8') && isvector(packet) && numel(packet)>=32 && mod(numel(packet),32)==0);
[~,~,endian]=computer;assert(endian=='L');
values=reshape(typecast(packet(:),'double'),1,[]);assert(all(isfinite(values)));
assert(values(1)==73190501 && values(2)==sender && values(3)==t);
count=values(4);assert(count>=0 && count==fix(count) && numel(packet)==32+32*count);
rows=reshape(values(5:end),4,count)';
% Re-encoding performs field, identity, probability and uniqueness validation.
assert(isequal(reshape(packet,1,[]),encodePruneInformation(rows,sender,t)));
end
