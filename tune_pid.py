import random, json, time
from batch import *
NAMES = ["nominal", "fast motors 1.2x", "line 15mm", "slow motors 0.8x", "wheel mismatch 5%"]
def evaluate(P, seeds=range(14)):
    o = sweep(P, seeds=seeds, names=NAMES, verbose=False)
    ok = sum(v[0] for v in o.values()); n = sum(v[1] for v in o.values())
    tm = sum(v[2] * v[0] for v in o.values()) / max(1, ok)
    return ok, n, tm
if __name__ == "__main__":
    rnd = random.Random(7); res = []
    base = dict()
    cands = [base] + [dict(KP=rnd.choice([16, 20, 24, 30, 36]), KI=rnd.choice([0, 3, 6, 10]), KD=rnd.choice([0.3, 0.6, 0.9, 1.4, 2.0]),
                          CRUISE=rnd.choice([140, 160, 180, 200]), MIN_SPEED=rnd.choice([85, 100]),
                          ERR_EMA=rnd.choice([0.35, 0.5, 0.7])) for _ in range(20)]
    for P in cands:
        ok, n, tm = evaluate(P); res.append((ok, -tm, P))
        print(f"{ok}/{n} {tm:5.1f}s {P}", flush=True)
    res.sort(key=lambda r: (r[0], r[1]), reverse=True)
    print("BEST", res[0])
