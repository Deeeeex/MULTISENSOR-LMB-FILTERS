function packet=encodePruneInformation(rows,sender,t)
% Fixed little-endian binary trailer; the caller sends the returned bytes.
assert(size(rows,2)==4 && all(isfinite(rows),'all'));
assert(isscalar(sender) && ismember(sender,[1,2]) && isscalar(t) && t>=1 && t==fix(t));
assert(all(rows(:,1)>=1 & rows(:,1)<=t & rows(:,1)==fix(rows(:,1))));
assert(all(rows(:,2)>=1 & rows(:,2)==fix(rows(:,2))));
assert(all(rows(:,3)>=0 & rows(:,3)<=.001 & rows(:,4)>0 & rows(:,4)<=1));
assert(size(unique(rows(:,1:2),'rows'),1)==size(rows,1));
[~,~,endian]=computer;assert(endian=='L','Existing packet codecs require this little-endian host.');
values=[73190501,sender,t,size(rows,1),reshape(rows',1,[])];
packet=typecast(double(values),'uint8');assert(numel(packet)==32+32*size(rows,1));
end
