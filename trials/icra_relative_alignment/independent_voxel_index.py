"""Exact half-metre bin indexing without rounding away the sign near zero."""
import numpy as np

def occupancy(points):
    p=np.asarray(points,dtype=float)
    keep=(p[:,2]>=-1.5)&(p[:,2]<3)&(p[:,0]>=-50)&(p[:,0]<50)&(p[:,1]>=-40)&(p[:,1]<40)
    # Multiplication by two is exact here. Adding the integer origin after
    # floor preserves the side of zero, unlike floor(2*(x+50)).
    xy=np.floor(p[keep,:2]*2.).astype(np.int64)+np.array([100,80],dtype=np.int64)
    assert ((xy>=0)&(xy<np.array([200,160]))).all()
    bits=np.zeros(32000,dtype=bool)
    if len(xy):bits[np.unique(xy[:,0]*160+xy[:,1])]=True
    return bits.reshape(200,160)
