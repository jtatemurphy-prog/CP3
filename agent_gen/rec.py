"""Summarize a record.json: per-frame pose of chosen parts (x,z mm, pitch/roll deg)."""
import json, sys, math
def eul(q):
    x,y,z,w=q
    roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    pitch=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
    yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    return roll,pitch,yaw
r=json.load(open(sys.argv[1])); names=sys.argv[2].split(',') ; step=int(sys.argv[3]) if len(sys.argv)>3 else 4
for i,f in enumerate(r['frames']):
    if i%step: continue
    s=f"t={f['t']:6.2f}"
    for n in names:
        p=f['parts'].get(n)
        if not p: continue
        (x,y,z),q=p; ro,pi,ya=eul(q)
        s+=f" | {n}: x{x*1000:6.0f} y{y*1000:5.0f} z{z*1000:5.0f} r{ro:5.0f} p{pi:5.0f} yw{ya:5.0f}"
    print(s)
