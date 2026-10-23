import sys, subprocess, math
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from sim import *

STATE = ["START", "FOLLOW", "JUNCTION", "TURN", "SEARCH", "DONE", "GAP"]
SCOL = {"START": "#888", "FOLLOW": "#2e9e4f", "JUNCTION": "#e0a800", "TURN": "#d9480f", "SEARCH": "#c2255c", "DONE": "#1971c2", "GAP": "#7048e8"}

def render(course, res, t0, dur, out, fps=25, speedup=1.0, P=None, title=""):
    T = res["tele"]; ts = T[:, 0]
    W, H = 1280, 720
    fig = plt.figure(figsize=(W / 80, H / 80), dpi=80, facecolor="#111418")
    gs = fig.add_gridspec(3, 3, left=0.04, right=0.98, top=0.93, bottom=0.07, wspace=0.28, hspace=0.45,
                          width_ratios=[1.55, 1, 1], height_ratios=[1.2, 1, 1])
    axm = fig.add_subplot(gs[:, 0]); axir = fig.add_subplot(gs[0, 1]); axmap = fig.add_subplot(gs[0, 2])
    axe = fig.add_subplot(gs[1, 1:]); axp = fig.add_subplot(gs[2, 1:])
    for a in (axm, axir, axmap, axe, axp):
        a.set_facecolor("#1b1f24")
        for sp in a.spines.values(): sp.set_color("#444")
        a.tick_params(colors="#aaa", labelsize=8)
    ext = (-course.ox, course.img.shape[1] - course.ox, -course.oy, course.img.shape[0] - course.oy)
    axm.imshow(1 - course.img, cmap="gray", origin="lower", extent=ext, vmin=-0.3, vmax=1.0)
    axmap.imshow(1 - course.img, cmap="gray", origin="lower", extent=ext, vmin=-0.3, vmax=1.0)
    axmap.set_xticks([]); axmap.set_yticks([]); axmap.set_title("course map", color="#ccc", fontsize=9)
    pad = 380
    # robot polygons (robot frame: x fwd, y left)
    A, Wb = 100.0, 170.0
    parts = {
        "body": ([(0, -25), (A - 5, -25), (A - 5, 25), (0, 25)], "#4dabf7"),
        "axle": ([(-12, -Wb / 2), (12, -Wb / 2), (12, Wb / 2), (-12, Wb / 2)], "#74c0fc"),
        "wl": ([(-32, Wb / 2), (32, Wb / 2), (32, Wb / 2 + 22), (-32, Wb / 2 + 22)], "#ffd43b"),
        "wr": ([(-32, -Wb / 2), (32, -Wb / 2), (32, -Wb / 2 - 22), (-32, -Wb / 2 - 22)], "#ffd43b"),
        "arr": ([(A - 8, -40), (A + 8, -40), (A + 8, 40), (A - 8, 40)], "#e64980"),
    }
    polys = {k: Polygon(v[0], closed=True, fc=v[1], ec="k", lw=0.6, zorder=5) for k, v in parts.items()}
    for p in polys.values(): axm.add_patch(p)
    spots = axm.scatter([0] * 4, [0] * 4, s=46, zorder=7, ec="w", lw=0.8)
    (trail,) = axm.plot([], [], "-", color="#ff6b6b", lw=1.6, zorder=4)
    (mdot,) = axmap.plot([], [], "o", color="#ff6b6b", ms=5)
    (mtrail,) = axmap.plot([], [], "-", color="#ff6b6b", lw=1)
    ttl = fig.text(0.04, 0.955, title, color="#e9ecef", fontsize=13, weight="bold")
    stxt = axm.text(0.02, 0.97, "", transform=axm.transAxes, color="w", fontsize=13, weight="bold", va="top",
                    bbox=dict(fc="#000a", ec="none", pad=4), zorder=10)
    # IR panel
    axir.set_xlim(0, 4); axir.set_ylim(0, 1.6); axir.axis("off")
    axir.set_title("IR array  R1 R2 R3 R4  (white=line seen)", color="#ccc", fontsize=9)
    irs = [Rectangle((i + .06, .55), .88, .85, fc="#333", ec="#777") for i in range(4)]
    for r in irs: axir.add_patch(r)
    for i in range(4): axir.text(i + .5, .38, f"R{i+1}", color="#aaa", ha="center", fontsize=8)
    bits = axir.text(2, .08, "", color="#74c0fc", ha="center", fontsize=11, family="monospace")
    # scrolling plots
    win = 4.0
    axe.set_title("PID: filtered error (units) & integral term (PWM)", color="#ccc", fontsize=9)
    axe.set_ylim(-3.4, 3.4); axe.axhline(0, color="#555", lw=.6)
    (le,) = axe.plot([], [], color="#ffd43b", lw=1.5, label="error")
    (li,) = axe.plot([], [], color="#69db7c", lw=1.2, label="I term/10")
    axe.legend(loc="upper left", fontsize=7, facecolor="#222", labelcolor="#ddd", framealpha=.6)
    axp.set_title("motor command PWM (L / R)  +  wheel speed", color="#ccc", fontsize=9)
    axp.set_ylim(-140, 260); axp.axhline(0, color="#555", lw=.6)
    (ll,) = axp.plot([], [], color="#4dabf7", lw=1.4, label="L cmd")
    (lr,) = axp.plot([], [], color="#ff8787", lw=1.4, label="R cmd")
    axp.legend(loc="upper left", fontsize=7, facecolor="#222", labelcolor="#ddd", framealpha=.6, ncol=2)
    axp.set_xlabel("time (s)", color="#aaa", fontsize=8)

    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(fps),
           "-i", "-", "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    n = int(dur * fps)
    for f in range(n):
        tt = t0 + f / fps * speedup
        i = min(len(ts) - 1, int(np.searchsorted(ts, tt)))
        t, x, y, th, raw, st, cl, cr, ef, ig, vl, vr = T[i]
        c, s_ = math.cos(th), math.sin(th)
        for k, (pts, _) in parts.items():
            xy = np.array(pts, float)
            wx = x + xy[:, 0] * c - xy[:, 1] * s_; wy = y + xy[:, 0] * s_ + xy[:, 1] * c
            polys[k].set_xy(np.c_[wx, wy])
        sx, sy, sc = [], [], []
        for b in range(4):
            lat = (1.5 - b) * 15.0
            sx.append(x + A * c - lat * s_); sy.append(y + A * s_ + lat * c)
            sc.append("#ffffff" if (int(raw) >> b) & 1 else "#e03131")
        spots.set_offsets(np.c_[sx, sy]); spots.set_facecolor(sc)
        axm.set_xlim(x - pad * 16 / 9 * 0.9, x + pad * 16 / 9 * 0.9); axm.set_ylim(y - pad, y + pad)
        j0 = max(0, int(np.searchsorted(ts, t0)))
        trail.set_data(T[j0:i + 1:5, 1], T[j0:i + 1:5, 2])
        mdot.set_data([x], [y]); mtrail.set_data(T[:i + 1:20, 1], T[:i + 1:20, 2])
        name = STATE[int(st)]
        stxt.set_text(f"t={t:6.2f}s   {name}\nL={int(cl):4d}  R={int(cr):4d}\nv={0.5*(vl+vr):5.0f} mm/s")
        stxt.get_bbox_patch().set_facecolor(SCOL[name])
        for b in range(4):
            irs[b].set_facecolor("#f1f3f5" if (int(raw) >> b) & 1 else "#2b2f36")
        bits.set_text(f"mask {int(raw):04b}   err {ef:+.2f}")
        k0 = int(np.searchsorted(ts, t - win)); sl = slice(k0, i + 1)
        for a in (axe, axp): a.set_xlim(t - win, t)
        le.set_data(ts[sl], T[sl, 8]); li.set_data(ts[sl], T[sl, 9] / 10)
        ll.set_data(ts[sl], T[sl, 6]); lr.set_data(ts[sl], T[sl, 7])
        fig.canvas.draw()
        ff.stdin.write(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].tobytes())
    ff.stdin.close(); ff.wait(); plt.close(fig)

if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    out = sys.argv[2] if len(sys.argv) > 2 else "lfr_run.mp4"
    c = grid_course(random_edges(seed, extra=0.4)); r = run(c, max_s=240)
    T = r["tele"]
    gap = np.where(T[:, 5] == FW.ST_GAP)[0]
    t_gap = T[gap[0], 0] if len(gap) else 20.0
    t0 = max(0.0, t_gap - 4.5)
    print("run", r["ok"], r["log"], "window start", round(t0, 1))
    render(c, r, t0, 10.0, out, title=f"Meshemerize LFR sim - seed {seed}: dead-end U-turn then LHR")
