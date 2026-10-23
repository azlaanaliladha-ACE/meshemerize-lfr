import sys
from batch import *
if __name__ == "__main__":
    for v in [float(a) for a in sys.argv[1:]]:
        o = sweep(dict(TURN_INNER=v), seeds=range(40), verbose=False)
        tot = sum(x[0] for x in o.values())
        print(f"TURN_INNER={v}: {tot}/{sum(x[1] for x in o.values())}  " + "  ".join(f"{k}:{x[0]}" for k, x in o.items()), flush=True)
