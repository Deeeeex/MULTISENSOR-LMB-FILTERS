"""Independent voxel indexing and exact integer overlap for every shift."""
import numpy as np

def occupancy(points):
    p=np.asarray(points,dtype=float)
    keep=(p[:,2]>=-1.5)&(p[:,2]<3.)
    xy=np.floor((p[keep,:2]+np.array([50.,40.]))*2.).astype(np.int64)
    xy=xy[(xy[:,0]>=0)&(xy[:,0]<200)&(xy[:,1]>=0)&(xy[:,1]<160)]
    bits=np.zeros(32000,dtype=bool)
    if len(xy):bits[np.unique(xy[:,0]*160+xy[:,1])]=True
    return bits.reshape(200,160)

def all_overlaps(first,second):
    ints=[]
    for grid in (first,second):
        padded=np.zeros((200,200),dtype=np.uint8)
        padded[:,20:180]=np.asarray(grid,dtype=np.uint8)
        ints.append(int.from_bytes(np.packbits(padded.ravel(),bitorder='little').tobytes(),'little'))
    a,b=ints
    values=np.empty((41,41),dtype=np.int32)
    for i,x in enumerate(range(-20,21)):
        for j,y in enumerate(range(-20,21)):
            amount=x*200+y
            translated=b<<amount if amount>=0 else b>>(-amount)
            values[i,j]=(a&translated).bit_count()
    return values

def decision(values):
    ordered=[]
    for i in range(41):
        for j in range(41):
            x,y=i-20,j-20
            ordered.append((-int(values[i,j]),x*x+y*y,x,y))
    _,_,x,y=min(ordered)
    boundary=max(abs(x),abs(y))==20
    return [0.,0.] if boundary else [x/2.,y/2.]

def direct_overlap(first,second,x,y):
    a=first[max(0,x):min(200,200+x),max(0,y):min(160,160+y)]
    b=second[max(0,-x):min(200,200-x),max(0,-y):min(160,160-y)]
    return int(np.count_nonzero(a&b))
