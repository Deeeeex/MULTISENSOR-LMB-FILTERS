"""One fixed occupancy translation estimator; no tracker or truth access."""
import struct
import numpy as np
from scipy.signal import fftconvolve

SHAPE=(200,160)
STEP=.5
LIMIT=20
HEADER=struct.Struct('<4sHHHHddI')
assert HEADER.size==32

def project(points, transform):
    p=np.asarray(points,dtype=float);m=np.asarray(transform,dtype=float)
    assert p.ndim==2 and p.shape[1]==3 and m.shape==(4,4)
    assert np.isfinite(p).all() and np.isfinite(m).all()
    return np.column_stack([p[:,0]*m[i,0]+p[:,1]*m[i,1]+p[:,2]*m[i,2]+m[i,3] for i in range(3)])

def occupancy(points):
    p=np.asarray(points,dtype=float)
    keep=(p[:,2]>=-1.5)&(p[:,2]<3)&(p[:,0]>=-50)&(p[:,0]<50)&(p[:,1]>=-40)&(p[:,1]<40)
    hist,_,_=np.histogram2d(p[keep,0],p[keep,1],bins=[np.arange(-50,50.5,.5),np.arange(-40,40.5,.5)])
    result=hist>0
    assert result.shape==SHAPE
    return result

def packet(grid,source,frame):
    assert np.asarray(grid).shape==SHAPE and source in (1,2) and frame>=1
    body=np.packbits(np.asarray(grid,dtype=np.uint8).ravel(),bitorder='little').tobytes()
    value=HEADER.pack(b'ICAL',1,source,*SHAPE,-50.,-40.,frame)+body
    assert len(value)==4032
    return value

def decode(value,source,frame):
    assert len(value)==4032
    assert HEADER.unpack(value[:32])==(b'ICAL',1,source,*SHAPE,-50.,-40.,frame)
    return np.unpackbits(np.frombuffer(value[32:],dtype=np.uint8),bitorder='little').reshape(SHAPE).astype(bool)

def correlations(first,second):
    a=np.asarray(first,dtype=bool);b=np.asarray(second,dtype=bool)
    assert a.shape==b.shape==SHAPE
    full=fftconvolve(a.astype(float),b[::-1,::-1].astype(float),mode='full')
    center=np.array(SHAPE)-1
    values=full[center[0]-LIMIT:center[0]+LIMIT+1,center[1]-LIMIT:center[1]+LIMIT+1]
    assert np.max(np.abs(values-np.rint(values)))<1e-7
    result=np.rint(values).astype(np.int32)
    assert (result>=0).all() and result[LIMIT,LIMIT]==np.count_nonzero(a&b)
    return result

def choose(values):
    values=np.asarray(values)
    assert values.shape==(41,41)
    shifts=np.argwhere(values==values.max())-LIMIT
    norms=np.sum(shifts*shifts,axis=1);shortest=norms.min()
    sx,sy=min((int(x),int(y)) for (x,y),norm in zip(shifts,norms) if norm==shortest)
    boundary=abs(sx)==LIMIT or abs(sy)==LIMIT
    applied=(0.,0.) if boundary else (sx*STEP,sy*STEP)
    return dict(translation=list(applied),maximizer_cells=[sx,sy],boundary=boundary,
                maximizing_shifts=len(shifts),native_overlap=int(values[LIMIT,LIMIT]),maximum_overlap=int(values.max()))

def estimate(first,second):
    values=correlations(first,second)
    return choose(values),values
