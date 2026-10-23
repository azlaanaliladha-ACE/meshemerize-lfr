import sys, time
from sim import *
from concurrent.futures import ProcessPoolExecutor

VARIANTS = {
 "nominal":            dict(),
 "pitch 10mm":         dict(rk=dict(pitch=10.0)),
 "pitch 20mm":         dict(rk=dict(pitch=20.0)),
 "line 15mm":          dict(lw=15),
 "line 25mm":          dict(lw=25),
 "slow motors 0.8x":   dict(rk=dict(vmax=440.0)),
 "fast motors 1.2x":   dict(rk=dict(vmax=660.0)),
 "wheel mismatch 5%":  dict(rk=dict(gain_l=0.95)),
 "sensor noise 0.3%":  dict(rk=dict(noise=0.003)),
 "deadband 55":        dict(rk=dict(deadband=55)),
}
def one(args):
    seed, name, P = args
    v = VARIANTS[name]
    c = grid_course(random_edges(seed, extra=0.4), line_w=v.get("lw", 20))
    r = run(c, P=P, robot_kw=dict(seed=seed, **v.get("rk", {})), max_s=300, trace=False)
    return r["ok"], r["t"], r["reason"], seed

def sweep(P=None, seeds=range(40), names=None, verbose=True):
    out = {}
    with ProcessPoolExecutor() as ex:
        for name in (names or VARIANTS):
            rs = list(ex.map(one, [(s, name, P) for s in seeds]))
            ok = [r for r in rs if r[0]]
            out[name] = (len(ok), len(rs), sum(r[1] for r in ok) / max(1, len(ok)), [r[3] for r in rs if not r[0]], [r[2] for r in rs if not r[0]])
            if verbose: print(f"{name:20s} {len(ok):2d}/{len(rs)}  mean {out[name][2]:5.1f}s  fails={out[name][3]} {out[name][4]}", flush=True)
    return out

if __name__ == "__main__":
    t = time.time(); sweep(); print(round(time.time() - t), "s")
