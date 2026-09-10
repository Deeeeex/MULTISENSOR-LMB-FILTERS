"""Check shift orientation, border behavior, tie rules and serialization."""
import numpy as np
from alignment_math import occupancy,estimate,packet,decode
import independent_math as independent

def check():
    a=np.zeros((200,160),bool)
    for x,y in [(70,60),(71,65),(75,61),(83,73),(89,55)]:a[x,y]=True
    cases=[]
    for sx,sy in [(0,0),(7,-3),(-11,6),(20,0)]:
        b=np.zeros_like(a)
        points=np.argwhere(a);points+=np.array([sx,sy]);b[points[:,0],points[:,1]]=True
        cases.append((f'shift_{sx}_{sy}',a,b,[0.,0.] if abs(sx)==20 else [-sx*.5,-sy*.5]))
    cases += [('empty',a,np.zeros_like(a),[0.,0.]),('both_empty',np.zeros_like(a),np.zeros_like(a),[0.,0.])]
    tie=np.zeros_like(a);tie[99,80]=tie[101,80]=True
    one=np.zeros_like(a);one[100,80]=True
    cases.append(('symmetric_tie',tie,one,[-.5,0.]))
    reports=[]
    for name,first,second,expected in cases:
        actual,values=estimate(first,second);other=independent.all_overlaps(first,second)
        assert np.array_equal(values,other) and actual['translation']==independent.decision(other)==expected,name
        for x,y in [(-20,-20),(-3,8),(0,0),(17,-9),(20,20)]:
            assert values[x+20,y+20]==independent.direct_overlap(first,second,x,y)
        for source,grid in enumerate((first,second),1):
            encoded=packet(grid,source,7);assert np.array_equal(decode(encoded,source,7),grid)
        reports.append(dict(name=name,**actual,checked_shifts=1681))
    points=np.array([[-50,-40,-1.5],[-49.5,-39.5,2.999],[-50.001,0,0],[50,0,0],[0,40,0],[0,0,3],[0,0,-1.501],[0,0,0]])
    assert np.array_equal(occupancy(points),independent.occupancy(points))
    assert occupancy(points).sum()==3
    return dict(passed=True,cases=reports,packet_bytes=4032,wire_bytes_per_frame=33024,voxel_boundary_cases=len(points))

if __name__=='__main__':print(check())
