import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
def draw(course, res, path, title=""):
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(1 - course.img, cmap="gray", origin="lower",
              extent=(-course.ox, course.img.shape[1] - course.ox, -course.oy, course.img.shape[0] - course.oy))
    tj = res["traj"]
    if len(tj): ax.plot(tj[:, 0], tj[:, 1], "r-", lw=1.2)
    ax.set_title(f"{title}  ok={res['ok']} t={res['t']:.1f}s log={res['log'][:40]}")
    ax.set_aspect("equal"); fig.savefig(path, dpi=80, bbox_inches="tight"); plt.close(fig)
