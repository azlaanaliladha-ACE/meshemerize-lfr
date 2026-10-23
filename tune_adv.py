from batch import *
if __name__ == "__main__":
    for adv in (500, 600, 680, 740, 800, 880, 980):
        o = sweep(dict(ADVANCE_MS=adv), seeds=range(40), names=["nominal", "fast motors 1.2x", "slow motors 0.8x"], verbose=False)
        print(f"ADVANCE_MS={adv}: " + "  ".join(f"{k}:{v[0]}/40" for k, v in o.items()), flush=True)
