function [bytes,decoded,restoredBase,tags]=gaussianZeroCodec(base,encoded)
% Losslessly encode 15-double columns, including every signed zero bit.
assert(isa(base,'uint8') && isrow(base) && size(encoded,1)==15 && all(isfinite(encoded),'all'));
n=size(encoded,2);full=any(encoded~=0,1);
bits=reshape(typecast(encoded(:),'uint64'),15,n);
negative=bitget(bits,64);tags=zeros(1,n,'uint16');
for j=1:15,tags=bitor(tags,bitshift(uint16(negative(j,:)),j-1));end
tags(full)=uint16(32768);
payload=encoded(:,full);
bytes=[base,reshape(typecast(tags,'uint8'),1,[]),reshape(typecast(payload(:),'uint8'),1,[])];
restoredBase=bytes(1:numel(base));
receivedTags=reshape(typecast(bytes(numel(base)+(1:2*n)),'uint16'),1,[]);
receivedFull=bitget(receivedTags,16)>0;
assert(all(bitand(receivedTags(receivedFull),uint16(32767))==0));
receivedBits=zeros(15,n,'uint64');
for j=1:15,receivedBits(j,:)=bitshift(uint64(bitget(receivedTags,j)),63);end
decoded=reshape(typecast(receivedBits(:),'double'),15,n);
decoded(:,receivedFull)=reshape(typecast(bytes(numel(base)+2*n+1:end),'double'),15,[]);
assert(isequal(base,restoredBase) && isequal(tags,receivedTags));
assert(isequal(typecast(encoded(:),'uint64'),typecast(decoded(:),'uint64')));
assert(numel(bytes)==numel(base)+2*n+120*nnz(full));
end
