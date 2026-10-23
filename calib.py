from sim import *
def tcourse(line_w=20):
    c = Course(ox=700, line_w=line_w)
    c.seg(0,-250,0,600); c.seg(-400,600,400,600)   # T / + junction bar at y=600
    c.start = (0.0, -230.0, math.pi/2); return c

def axle_offset_at_decide(P=None, robot_kw=None):
    c = tcourse(); fw = FW(P); rb = Robot(c, **(robot_kw or {}))
    t=0.0; nxt=0.0; cl=cr=0; prev=fw.state
    for k in range(30000):
        if t>=nxt:
            nxt+=0.005; cl,cr = fw.tick(rb.sensors(), int(t*1000))
            if prev==FW.ST_JUNCTION and fw.state!=FW.ST_JUNCTION: return rb.y-600, fw.log
            prev=fw.state
        rb.step(cl,cr,0.001); t+=0.001
if __name__=="__main__":
    for creep in (70, 90, 110):
        for adv in (600, 800, 1000, 1200):
            off,_ = axle_offset_at_decide(dict(CREEP=creep, ADVANCE_MS=adv))
            print(f"CREEP={creep} ADVANCE={adv}: axle {off:+.0f} mm past junction centre")
