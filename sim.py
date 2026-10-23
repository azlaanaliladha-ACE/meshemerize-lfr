"""
Physics sim for the Meshemerize line follower.

Geometry (from the robot): T-body, wheels 170 mm apart, 4-ch RGB sensor array
100 mm ahead of the axle. Everything is in mm / seconds.

The firmware logic in class FW is a line-for-line port of meshemerize_lfr.ino
so that what is tuned here is what runs on the ESP32.
"""
import math, random
import numpy as np

# ---------------------------------------------------------------- world
RES = 1  # mm per pixel
LINE_W = 20


class Course:
    def __init__(self, w=2200, h=1900, ox=300, oy=500, line_w=LINE_W):
        self.img = np.zeros((h, w), np.uint8)
        self.ox, self.oy, self.lw = ox, oy, line_w
        self.end_center = None
        self.nodes = set()

    def seg(self, x0, y0, x1, y1):
        hw = self.lw // 2
        xa, xb = sorted((x0, x1)); ya, yb = sorted((y0, y1))
        self.img[self.oy + ya - hw: self.oy + yb + hw, self.ox + xa - hw: self.ox + xb + hw] = 1

    def box(self, cx, cy, size=400):
        h = size // 2
        self.img[self.oy + cy - h: self.oy + cy + h, self.ox + cx - h: self.ox + cx + h] = 1
        self.end_center = (cx, cy, h)

    def black(self, x, y):
        ix, iy = int(round(x)) + self.ox, int(round(y)) + self.oy
        if ix < 0 or iy < 0 or iy >= self.img.shape[0] or ix >= self.img.shape[1]:
            return 0
        return int(self.img[iy, ix])

    def in_end(self, x, y):
        if not self.end_center: return False
        cx, cy, h = self.end_center
        return abs(x - cx) <= h and abs(y - cy) <= h


def grid_course(edges, n=5, pitch=300, end_node=None, line_w=LINE_W, start_stub=250):
    """edges: iterable of ((c,r),(c,r)) between grid nodes. Start is below node (0,0),
    end box attached to the right of end_node (default top-right node)."""
    c = Course(line_w=line_w)
    for (a, b) in edges:
        c.seg(a[0] * pitch, a[1] * pitch, b[0] * pitch, b[1] * pitch)
    c.seg(0, -start_stub, 0, 0)  # start stub
    en = end_node or (n - 1, n - 1)
    ex, ey = en[0] * pitch, en[1] * pitch
    c.seg(ex, ey, ex + 400, ey)
    c.box(ex + 400 + 200, ey, 400)
    c.start = (0.0, -start_stub + 20.0, math.pi / 2)
    c.edges = list(edges); c.n = n; c.pitch = pitch
    return c


def random_edges(seed, n=5, extra=0.35, dead_prune=0.0):
    """Random spanning tree (-> dead ends) + extra edges (-> loops)."""
    rnd = random.Random(seed)
    cells = [(i, j) for i in range(n) for j in range(n)]
    seen = {(0, 0)}; stack = [(0, 0)]; edges = []
    while stack:
        cur = stack[-1]
        nb = [(cur[0] + dx, cur[1] + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 0 <= cur[0] + dx < n and 0 <= cur[1] + dy < n and (cur[0] + dx, cur[1] + dy) not in seen]
        if nb:
            nx = rnd.choice(nb); seen.add(nx); edges.append((cur, nx)); stack.append(nx)
        else:
            stack.pop()
    es = {frozenset(e) for e in edges}
    for a in cells:
        for dx, dy in ((1, 0), (0, 1)):
            b = (a[0] + dx, a[1] + dy)
            if b[0] < n and b[1] < n and frozenset((a, b)) not in es and rnd.random() < extra:
                edges.append((a, b)); es.add(frozenset((a, b)))
    return edges


def demo_course(line_w=LINE_W):
    """Hand-made 5x5 course: loops (incl. a loop LHR laps forever), dead ends, T and + junctions."""
    e = []
    def H(r, c0, c1):
        for c in range(c0, c1): e.append(((c, r), (c + 1, r)))
    def V(c, r0, r1):
        for r in range(r0, r1): e.append(((c, r), (c, r + 1)))
    V(0, 0, 4)          # left spine
    H(0, 0, 2)          # bottom run to (2,0)
    V(2, 0, 2)          # up to (2,2)
    H(2, 0, 4)          # middle bar  -> loop (0,0)-(2,0)-(2,2)-(0,2)  [closed loop]
    H(2, 2, 4)
    V(4, 0, 4)          # right spine
    H(0, 2, 4)          # bottom to right spine
    H(4, 0, 4)          # top bar
    V(1, 2, 3)          # dead-end stub off the middle bar (up to (1,3))
    V(3, 2, 4)          # connect middle bar to top bar -> second loop
    return grid_course(e, line_w=line_w)


# ---------------------------------------------------------------- robot
class Robot:
    def __init__(self, course, wheel_base=170.0, sensor_ahead=100.0, pitch=15.0,
                 vmax=550.0, deadband=35, tau=0.04, gain_l=1.0, gain_r=1.0, noise=0.0, seed=0):
        self.c = course
        self.W, self.A, self.pitch = wheel_base, sensor_ahead, pitch
        self.vmax, self.db, self.tau = vmax, deadband, tau
        self.gl, self.gr = gain_l, gain_r
        self.x, self.y, self.th = course.start
        self.vl = self.vr = 0.0
        self.noise = noise; self.rnd = random.Random(seed)

    def _target(self, cmd, g):
        s = 1 if cmd >= 0 else -1
        m = max(0.0, abs(cmd) - self.db) / (255.0 - self.db)
        return s * m * self.vmax * g

    def step(self, cl, cr, dt):
        tl, tr = self._target(cl, self.gl), self._target(cr, self.gr)
        a = dt / (self.tau + dt)
        self.vl += (tl - self.vl) * a
        self.vr += (tr - self.vr) * a
        v = 0.5 * (self.vl + self.vr)
        w = (self.vr - self.vl) / self.W
        self.th += w * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

    def sensors(self):
        """bit0 = R1 (leftmost) ... bit3 = R4 (rightmost); 1 = black."""
        c, s = math.cos(self.th), math.sin(self.th)
        m = 0
        for i in range(4):
            lat = (1.5 - i) * self.pitch  # +left
            px = self.x + self.A * c - lat * s
            py = self.y + self.A * s + lat * c
            b = self.c.black(px, py)
            if self.noise and self.rnd.random() < self.noise: b ^= 1
            m |= b << i
        return m


# ---------------------------------------------------------------- firmware (port of the .ino)
DEFAULT = dict(
    CRUISE=160, MIN_SPEED=100, CREEP=90, TURN_SPEED=95, TURN_INNER=-0.9,
    KP=20.0, KI=6.0, KD=0.6, I_CLAMP=40.0, ERR_EMA=0.5, MAX_CORR=110.0, SPEED_DROP=0.45,
    GAP_MS=450, END_ROLL_MS=600,
    ADVANCE_MS=740, STRAIGHT_WINDOW_MS=140, END_HOLD_MS=350,
    LOST_CONFIRM_MS=40, LOST_CENTERED=1.6, POST_TURN_SLOW_MS=250,
    TURN_TIMEOUT_MS=3000, UTURN_TIMEOUT_MS=5000, SEARCH_FIRST_MS=1200, SEARCH_TOTAL_MS=3600,
    START_WAIT_MS=200, START_SEEK_MS=3000, LAP_MIN=3, LAP_MAX=8,
)


def popcount(m): return bin(m).count("1")


def trunc(f):  # C-style (int) cast
    return int(f)


def constrain(v, lo, hi): return lo if v < lo else hi if v > hi else v


class FW:
    ST_START, ST_FOLLOW, ST_JUNCTION, ST_TURN, ST_SEARCH, ST_DONE, ST_GAP = range(7)
    POS = (3, 1, -1, -3)

    def __init__(s, P=None):
        s.P = dict(DEFAULT); s.P.update(P or {})
        s.state = s.ST_START; s.since = 0
        s.hist = [0, 0, 0]; s.filt_init = False
        s.errF = 0.0; s.prevErrF = 0.0; s.lastErr = 0.0; s.lastDir = 0; s.integ = 0.0
        s.zero_since = None
        s.log = []; s.guard_start = 0; s.breaks = 0
        s.slow_until = 0
        s.jL = s.jR = s.jS = False; s.ab_since = None
        s.turnDir = 1; s.phase = 0; s.leave = 0; s.acq = 0; s.is_u = False
        s.search_dir = 1; s.search_acq = 0
        s.done_reason = ""
        s.events = []   # (t, text) for plotting/diagnostics
        s.l = s.r = 0

    # --- helpers
    def filt(s, m):
        if not s.filt_init:
            s.hist = [m, m, m]; s.filt_init = True; return m
        s.hist = [s.hist[1], s.hist[2], m]
        a, b, c = s.hist
        return ((a & b) | (a & c) | (b & c)) & 0xF

    def line_error(s, m):
        n = 0; sm = 0
        for i in range(4):
            if m & (1 << i): sm += s.POS[i]; n += 1
        return None if n == 0 else sm / n

    def enter(s, st, now): s.state = st; s.since = now

    def drive(s, l, r): s.l, s.r = int(l), int(r)

    def log_turn(s, ch):
        s.log.append(ch)

    # --- loop guard: last k moves repeat the k before (k=LAP_MIN..LAP_MAX) or 4 same turns
    def lapping(s):
        n = len(s.log) - s.guard_start; L = s.log
        for k in range(s.P["LAP_MIN"], s.P["LAP_MAX"] + 1):
            if n < 2 * k: break
            same = True; has_turn = False
            for i in range(1, k + 1):
                a, b = L[-i], L[-i - k]
                if a != b: same = False; break
                if a in "LR": has_turn = True
            if same and has_turn: return True
        if n >= 4:
            c = L[-1]
            if c in "LR" and L[-2] == c and L[-3] == c and L[-4] == c: return True
        return False

    # --- state entries
    def begin_junction(s, now):
        s.jL = s.jR = s.jS = False; s.ab_since = None; s.integ = 0.0; s.end_until = None
        s.enter(s.ST_JUNCTION, now)
        s.drive(s.P["CREEP"], s.P["CREEP"])

    def begin_turn(s, d, now, is_u=False):
        s.turnDir = d; s.phase = 0; s.leave = 0; s.acq = 0; s.is_u = is_u
        s.enter(s.ST_TURN, now)

    def begin_search(s, d, now):
        s.search_dir = d; s.search_acq = 0
        s.enter(s.ST_SEARCH, now)

    def resume(s, m, now):
        e = s.line_error(m)
        s.errF = s.prevErrF = 0.0 if e is None else e
        s.integ = 0.0
        s.zero_since = None
        s.slow_until = now + s.P["POST_TURN_SLOW_MS"]
        s.enter(s.ST_FOLLOW, now)

    def decide(s, now, mask):
        P = s.P
        order = "LSR"
        if s.lapping():
            s.guard_start = len(s.log)
            order = "SRL" if s.breaks % 2 == 0 else "RSL"
            s.breaks += 1
            s.events.append((now, "loop break"))
        avail = {"L": s.jL, "S": s.jS, "R": s.jR}
        for ch in order:
            if avail[ch]:
                s.log_turn(ch)
                if ch == "L": s.begin_turn(1, now)
                elif ch == "R": s.begin_turn(-1, now)
                else: s.resume(mask, now)
                return
        s.log_turn("U"); s.begin_turn(1, now, True)

    def turn_drive(s, d, speed):
        inner = int(speed * s.P["TURN_INNER"])
        if d > 0: s.drive(inner, speed)
        else: s.drive(speed, inner)

    # --- main tick: returns (left, right) command
    def tick(s, raw_mask, now):
        P = s.P
        m = s.filt(raw_mask)
        st = s.state

        if st == s.ST_DONE:
            s.drive(0, 0)

        elif st == s.ST_START:
            if now < P["START_WAIT_MS"]: s.drive(0, 0)
            elif m != 0: s.resume(m, now)
            elif now - P["START_WAIT_MS"] > P["START_SEEK_MS"]: s.drive(0, 0)
            else: s.drive(P["CREEP"], P["CREEP"])

        elif st == s.ST_FOLLOW:
            if popcount(m) >= 3:
                s.begin_junction(now)
            elif m == 0:
                if s.zero_since is None: s.zero_since = now
                if now - s.zero_since >= P["LOST_CONFIRM_MS"]:
                    s.zero_since = None
                    if abs(s.lastErr) < P["LOST_CENTERED"]:
                        s.enter(s.ST_GAP, now)
                    else:
                        s.begin_search(1 if s.lastErr > 0 else -1, now)
                else:
                    s.drive(P["CREEP"], P["CREEP"])
            else:
                s.zero_since = None
                e = s.line_error(m)
                s.lastErr = e
                s.errF += P["ERR_EMA"] * (e - s.errF)
                d = (s.errF - s.prevErrF) / 0.005
                s.prevErrF = s.errF
                s.integ = constrain(s.integ + s.errF * 0.005 * P["KI"], -P["I_CLAMP"], P["I_CLAMP"])
                corr = constrain(P["KP"] * s.errF + s.integ + P["KD"] * d, -P["MAX_CORR"], P["MAX_CORR"])
                base = P["CRUISE"] - (P["CRUISE"] - P["MIN_SPEED"]) * min(1.0, abs(s.errF) * P["SPEED_DROP"])
                if now < s.slow_until: base = P["MIN_SPEED"]
                base = trunc(base)
                s.drive(base - trunc(corr), base + trunc(corr))

        elif st == s.ST_GAP:
            # line vanished while centred: roll straight; real gap -> line returns, else dead end
            if m != 0:
                s.resume(m, now)
            elif now - s.since >= P["GAP_MS"]:
                s.log_turn("U"); s.begin_turn(1, now, True)
            else:
                s.drive(P["CREEP"], P["CREEP"])

        elif st == s.ST_JUNCTION:
            if s.end_until is not None:      # end box confirmed: roll the axle inside, then stop
                if now >= s.end_until:
                    s.drive(0, 0); s.enter(s.ST_DONE, now); s.done_reason = "END"
                else:
                    s.drive(P["CREEP"], P["CREEP"])
                return s.l, s.r
            if popcount(m) >= 3:             # a real bar crossing; drift of the main line only gives 2
                if m & 0b0001: s.jL = True
                if m & 0b1000: s.jR = True
            if m == 0xF:
                if s.ab_since is None: s.ab_since = now
                elif now - s.ab_since >= P["END_HOLD_MS"]:
                    s.end_until = now + P["END_ROLL_MS"]
                    return s.l, s.r
            else:
                s.ab_since = None
            el = now - s.since
            if el >= P["ADVANCE_MS"] - P["STRAIGHT_WINDOW_MS"] and (m & 0b0110): s.jS = True
            if el >= P["ADVANCE_MS"]:
                s.decide(now, m)
            else:
                s.drive(P["CREEP"], P["CREEP"])

        elif st == s.ST_TURN:
            center = m & 0b0110
            if s.phase == 0:
                s.leave = s.leave + 1 if center == 0 else 0
                if s.leave >= 3: s.phase = 1
            else:
                s.acq = s.acq + 1 if (center != 0 and (m & 0b1001) == 0) else 0
                if s.acq >= 2:
                    s.resume(m, now); s.lastDir = s.turnDir
                    return s.l, s.r
            lim = P["UTURN_TIMEOUT_MS"] if s.is_u else P["TURN_TIMEOUT_MS"]
            if now - s.since >= lim:
                s.begin_search(s.turnDir, now)
                s.events.append((now, "turn timeout"))
            else:
                s.turn_drive(s.turnDir, P["TURN_SPEED"])

        elif st == s.ST_SEARCH:
            s.search_acq = s.search_acq + 1 if m != 0 else 0
            if s.search_acq >= 2:
                s.resume(m, now)
                return s.l, s.r
            el = now - s.since
            if el >= P["SEARCH_TOTAL_MS"]:
                s.drive(0, 0); s.enter(s.ST_DONE, now); s.done_reason = "LOST"
            else:
                d = s.search_dir if el < P["SEARCH_FIRST_MS"] else -s.search_dir
                sp = P["CREEP"] + 10
                s.drive(-sp, sp) if d > 0 else s.drive(sp, -sp)

        return s.l, s.r


# ---------------------------------------------------------------- runner
def simplify(path):
    """LSRB shortcut: removes dead-end U-turns."""
    p = list(path)
    rules = {("L", "R"): "U", ("L", "S"): "R", ("R", "L"): "U", ("S", "L"): "R", ("S", "S"): "U", ("L", "L"): "S"}
    ch = True
    while ch:
        ch = False
        for i in range(1, len(p) - 1):
            if p[i] != "U": continue
            r = rules.get((p[i - 1], p[i + 1]))
            if r is None: continue
            p[i - 1:i + 2] = [r]; ch = True; break
    return p


def run(course, P=None, robot_kw=None, max_s=300.0, fw_cls=None, trace=True):
    fw = (fw_cls or FW)(P)
    rb = Robot(course, **(robot_kw or {}))
    dt = 0.001
    t = 0.0; nxt = 0.0
    cl = cr = 0
    traj = []; tele = []
    off_ms = 0; reached = False
    nsteps = int(max_s / dt)
    for k in range(nsteps):
        now = int(t * 1000)
        if t >= nxt:
            nxt += 0.005
            raw = rb.sensors()
            cl, cr = fw.tick(raw, now)
            if trace: tele.append((t, rb.x, rb.y, rb.th, raw, fw.state, cl, cr, fw.errF, fw.integ, rb.vl, rb.vr))
            if getattr(fw, "state", None) == FW.ST_DONE and fw.done_reason:
                if k % 5 == 0 and trace: traj.append((rb.x, rb.y, rb.th))
                # let it coast to rest
                for _ in range(300): rb.step(0, 0, dt)
                break
        rb.step(cl, cr, dt)
        t += dt
        if not reached and k % 20 == 0 and course.in_end(rb.x, rb.y): reached = True
        if trace and k % 10 == 0: traj.append((rb.x, rb.y, rb.th))
    ok = fw.state == FW.ST_DONE and fw.done_reason == "END" and course.in_end(rb.x, rb.y)
    return dict(ok=ok, t=t, reason=fw.done_reason or "TIMEOUT", log="".join(fw.log), fw=fw, traj=np.array(traj), tele=np.array(tele), rb=rb,
                x=rb.x, y=rb.y, reached=reached)


# ---------------------------------------------------------------- the user's pasted "partial" firmware, ported 1:1 for comparison
class OLD:
    ST_DONE = -1
    F, TL, TR, SL, SR, LOST = range(6)
    def __init__(s, P=None):
        s.state = s.F; s.since = 0; s.hist = [0, 0, 0]; s.init = False
        s.prevErr = 0.0; s.lastDir = 0; s.armed = True; s.edge = 0; s.edgeSince = 0
        s.cs = 0; s.rs = 0; s.reqC = False; s.stimeout = 0
        s.log = []; s.events = []; s.done_reason = ""; s.breaks = 0; s.integ = 0.0; s.errF = 0.0
        s.l = s.r = 0
    def filt(s, m):
        if not s.init: s.hist = [m] * 3; s.init = True; return m
        s.hist = [s.hist[1], s.hist[2], m]; a, b, c = s.hist
        return ((a & b) | (a & c) | (b & c)) & 0xF
    @staticmethod
    def err(m):
        pos = (3, 1, -1, -3); n = 0; sm = 0
        for i in range(4):
            if m & (1 << i): sm += pos[i]; n += 1
        return 999.0 if n == 0 else sm / n
    def drive(s, l, r): s.l, s.r = int(l), int(r)
    def enter(s, st, now): s.state = st; s.since = now; s.cs = 0; s.rs = 0
    def resume(s, m, now):
        s.enter(s.F, now); s.prevErr = s.err(m)
        if s.prevErr == 999.0: s.prevErr = 0.0
        s.armed = (m & 6) == 6; s.edge = 0; s.drive(0, 0)
    def search(s, left, req, to, now):
        s.lastDir = 1 if left else -1; s.armed = False; s.edge = 0; s.reqC = req; s.stimeout = to
        s.enter(s.SL if left else s.SR, now)
    def tick(s, raw, now):
        m = s.filt(raw); st = s.state
        if st == s.F:
            if m == 0:
                s.edge = 0
                if s.lastDir == 0: s.enter(s.LOST, now); s.armed = False; s.drive(0, 0); return s.l, s.r
                s.search(s.lastDir > 0, False, 500, now); return s.l, s.r
            if (m & 6) == 6: s.armed = True; s.edge = 0
            if m == 0xF or m == 0b1001:
                s.edge = 0; s.prevErr = 0.0
                if m == 0xF: s.armed = True
                s.drive(65, 65); return s.l, s.r
            if s.armed and m in (1, 8):
                if s.edge != m: s.edge = m; s.edgeSince = now; s.lastDir = 1 if m == 1 else -1
                elif now - s.edgeSince >= 20:
                    left = (m == 1); s.lastDir = 1 if left else -1; s.armed = False; s.edge = 0
                    s.enter(s.TL if left else s.TR, now)
                    s.drive(-85, 85) if left else s.drive(85, -85); return s.l, s.r
            else: s.edge = 0
            e = s.err(m)
            if e > .1: s.lastDir = 1
            elif e < -.1: s.lastDir = -1
            corr = constrain(25.0 * e + 3.0 * (e - s.prevErr), -95.0, 95.0); s.prevErr = e
            base = constrain(130 - trunc(abs(e) * 10.0), 65, 130)
            s.drive(base - trunc(corr), base + trunc(corr))
        elif st in (s.TL, s.TR):
            s.cs = s.cs + 1 if (m & 6) == 6 else 0
            if s.cs >= 3: s.resume(m, now); s.armed = True; return s.l, s.r
            if now - s.since >= 1200: s.search(st == s.TL, True, 400, now); return s.l, s.r
            s.drive(-85, 85) if st == s.TL else s.drive(85, -85)
        elif st in (s.SL, s.SR):
            if s.reqC:
                s.cs = s.cs + 1 if (m & 6) == 6 else 0
                if s.cs >= 3: s.resume(m, now); s.armed = True; return s.l, s.r
            else:
                s.rs = s.rs + 1 if m != 0 else 0
                if s.rs >= 3: s.resume(m, now); s.armed = (m & 6) == 6; return s.l, s.r
            if now - s.since >= s.stimeout: s.drive(0, 0); s.enter(s.LOST, now); s.armed = False; return s.l, s.r
            sp = 85 if s.reqC else 65
            s.drive(-sp, sp) if st == s.SL else s.drive(sp, -sp)
        elif st == s.LOST:
            s.drive(0, 0); s.rs = s.rs + 1 if m != 0 else 0
            if s.rs >= 3: s.resume(m, now)
        return s.l, s.r
