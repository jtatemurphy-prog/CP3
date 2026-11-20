import json,math,sys, numpy as np
r=json.load(open(sys.argv[1])); names=sys.argv[2].split(','); st=int(sys.argv[3]) if len(sys.argv)>3 else 25
def R(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fr=r['frames']
for i in range(0,len(fr)-1,st):
    a,b=fr[i],fr[i+1]; dt=b['t']-a['t']; P=a['parts']; Q=b['parts']
    s=f"t={a['t']:5.2f} x{P['deck'][0][0]*1000:5.0f} v{(Q['deck'][0][0]-P['deck'][0][0])/dt*1000:5.0f}"
    for n in names:
        D=R(P[n][1]).T@R(Q[n][1])   # body-frame delta
        W=R(P[n][1])@np.array([D[2,1]-D[1,2],D[0,2]-D[2,0],D[1,0]-D[0,1]])/2/dt   # world angular velocity (small angle)
        s+=f" {n}:{W[1]*60/(2*math.pi):6.0f}"
    print(s)
