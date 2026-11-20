import json,math,sys, numpy as np
r=json.load(open(sys.argv[1])); names=sys.argv[2].split(','); st=int(sys.argv[3]) if len(sys.argv)>3 else 25
def R(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fr=r['frames']
for i in range(0,len(fr)-1,st):
    a,b=fr[i],fr[i+1]; dt=b['t']-a['t']; P=a['parts']
    ro=math.degrees(math.atan2(R(P['deck'][1])[2,1],R(P['deck'][1])[2,2]))
    s=f"t={a['t']:5.2f} x{P['deck'][0][0]*1000:5.0f} z{P['deck'][0][2]*1000:4.0f} v{(b['parts']['deck'][0][0]-P['deck'][0][0])/dt*1000:5.0f}"
    Ra=R(P['deck'][1]); Rb=R(b['parts']['deck'][1])
    for n in names:
        A=Ra.T@R(P[n][1]); B=Rb.T@R(b['parts'][n][1]); D=A.T@B
        ang=math.degrees(math.acos(max(-1,min(1,(np.trace(D)-1)/2))))/dt/360*60
        s+=f" {n}:{ang:4.0f}/z{P[n][0][2]*1000:4.0f}"
    print(s)
