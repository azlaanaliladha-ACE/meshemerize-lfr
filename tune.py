import sys, json
from batch import *
NAMES = ["nominal", "fast motors 1.2x", "line 15mm", "pitch 20mm", "slow motors 0.8x"]
def score(P, seeds=range(24), names=NAMES):
    o = sweep(P, seeds=seeds, names=names, verbose=False)
    return sum(v[0] for v in o.values()), sum(v[1] for v in o.values()), {k: f"{v[0]}/{v[1]}" for k, v in o.items()}
if __name__ == "__main__":
    key = sys.argv[1]; vals = [float(v) for v in sys.argv[2:]]
    base = {}
    for v in vals:
        P = dict(base); P[key] = int(v) if key.isupper() and float(v).is_integer() and key not in ("TURN_INNER",) else v
        a, n, d = score(P)
        print(f"{key}={v:g}: {a}/{n}  {d}", flush=True)
