from batch import *
def one_cmp(seed):
    c = grid_course(random_edges(seed, extra=0.4))
    a = run(c, fw_cls=OLD, max_s=120, trace=False)
    b = run(c, max_s=300, trace=False)
    return a["reached"], b["ok"], b["t"], a["fw"].events and len(a["fw"].events)
if __name__ == "__main__":
    with ProcessPoolExecutor() as ex: rs = list(ex.map(one_cmp, range(40)))
    print("YOUR PASTED CODE reached end box:", sum(r[0] for r in rs), "/ 40")
    print("NEW FIRMWARE   solved + stopped :", sum(r[1] for r in rs), "/ 40, mean time", round(sum(r[2] for r in rs)/40,1), "s")
