#!/usr/bin/env python3

import argparse
import csv
import heapq
import math

import numpy as np
import matplotlib.pyplot as plt

SENSOR_OFFSET = 7.0   # cm, wheel axle centre -> colour sensor (MEASURE on your robot)
WHEEL_R = 2.8         # cm, wheel radius (task 1) - only for --heading wheels
TRACK = 11.8          # cm, distance between the wheels (task 1) - only for --heading wheels
STEP = 1.0            # cm, resampling step along the sensor path
SMOOTH = 9            # samples, moving average used for measuring edge lengths
PASS_R = 7.0          # cm, two pieces of path closer than this are "the same place"
ARC_GAP = 20.0        # cm, own path within this distance along the trace is ignored
FOLD_MAX = 200.0      # cm, longest out-and-back trip along a dead-end branch
MIN_SPUR = 5.0        # cm, a dead end must stick out at least this far
MIN_RUN = 3           # samples, shorter detections are noise
DRIFT_MIN = 45        # deg, --fix-drift only corrects laps that are further off than this
NODE_R = 10.0         # cm, visits closer than this belong to the same node
MERGE_D = 8.0         # cm, a junction this close to a dead end is the T-bar of a goal
HIT_R = 6.0           # cm, --nodes mode: a pass this close to a labelled node visits it
TAPE = {1: "L", 2: "R"}   # lap 1: tape on robot's left, lap 2: on its right
START_DIR = 0         # deg, direction the robot faced at the start ON THE DRAWING
                      # (0 = right/east, 90 = up/north), only rotates the plot


def disp(x, y):
    """robot frame -> drawing frame (rotated so the plot looks like the board)"""
    a = math.radians(START_DIR)
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


# ---------------------------------------------------------------- loading
def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {c: np.array([float(r[c]) for r in rows]) for c in rows[0]}


def choose_heading(d, source, fix_drift=False):
    """Compare gyro heading with heading from the wheels; optionally use the wheels.
    Each lap starts and ends at the same mark facing the same way, so the true net
    turn per lap is a whole number of turns (+-360). The source closer to that is
    the one to trust."""
    has_wheels = "left_deg" in d
    if has_wheels:
        wheel = np.degrees(np.radians(d["right_deg"] - d["left_deg"]) * WHEEL_R / TRACK)
        wheel -= wheel[0]
    print("HEADING CHECK (net turn per lap, should be close to +-360):")
    for lap in (1, 2):
        idx = np.flatnonzero((d["lap"] == lap) & (d["mode"] == 0))
        if len(idx) < 10:
            continue
        s0, s1 = idx[:30], idx[-30:]          # average a few samples: the robot wiggles
        line = "  lap {}: gyro {:+6.0f} deg".format(lap, d["heading_deg"][s1].mean() - d["heading_deg"][s0].mean())
        if has_wheels:
            line += "   wheels {:+6.0f} deg".format(wheel[s1].mean() - wheel[s0].mean())
        print(line)
    if fix_drift:
        # each lap ends where it started, facing the same way -> the gyro must show a whole
        # number of turns. The rest is gyro drift: remove it, growing linearly with time.
        hd = d["heading_deg"].astype(float).copy()
        offset, applied = np.zeros(len(hd)), 0.0
        for lap in (1, 2):
            idx = np.flatnonzero((d["lap"] == lap) & (d["mode"] == 0))
            if len(idx) < 60:
                continue
            a, b = idx[0], idx[-1]
            net = hd[idx[-30:]].mean() - hd[idx[:30]].mean()
            err = net - 360 * round(net / 360)
            if abs(err) < DRIFT_MIN:          # small: just wiggle at the start/end mark
                print("  lap {}: only {:+.0f} deg off, no drift correction needed".format(lap, err))
                continue
            tt = d["time_ms"]
            offset[a:b + 1] = applied + err * (tt[a:b + 1] - tt[a]) / max(tt[b] - tt[a], 1)
            offset[b + 1:] = applied + err
            applied += err
            print("  lap {}: removed {:+.0f} deg of drift ({:+.2f} deg/s)".format(
                lap, -err, -err / max((tt[b] - tt[a]) / 1000, 1)))
        hd -= offset
        d["heading_deg"] = hd
        ds = np.r_[0, np.diff(d["arc_cm"])]
        d["x_cm"] = np.cumsum(ds * np.cos(np.radians(hd)))
        d["y_cm"] = np.cumsum(ds * np.sin(np.radians(hd)))
    if source == "wheels":
        if not has_wheels:
            raise SystemExit("this log has no wheel angles (old explore version) - use --heading gyro")
        d["heading_deg"] = wheel
        ds = np.r_[0, np.diff(d["arc_cm"])]
        d["x_cm"] = np.cumsum(ds * np.cos(np.radians(wheel)))
        d["y_cm"] = np.cumsum(ds * np.sin(np.radians(wheel)))
    print("  using heading from:", source)
    return d


def prepare_laps(d):
    """Per lap: loop-closure correction, sensor position, resample every STEP cm."""
    laps = []
    for lap in (1, 2):
        m = (d["lap"] == lap) & (d["mode"] == 0)
        if m.sum() < 10:
            continue
        t = d["time_ms"][m]
        enc = d["arc_cm"][m] - d["arc_cm"][m][0]   # wheel-encoder distance
        x, y = d["x_cm"][m].copy(), d["y_cm"][m].copy()
        h = np.radians(d["heading_deg"][m])
        refl = d["reflection"][m]
        # lap starts and ends at the same mark -> spread the end error along the lap
        frac = (t - t[0]) / max(t[-1] - t[0], 1e-9)
        drift = (x[-1] - x[0], y[-1] - y[0])
        x -= drift[0] * frac
        y -= drift[1] * frac
        sx, sy = x + SENSOR_OFFSET * np.cos(h), y + SENSOR_OFFSET * np.sin(h)
        sa = np.concatenate([[0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])
        sa, keep = np.unique(sa, return_index=True)
        g = np.arange(0, sa[-1], STEP)
        gx, gy = np.interp(g, sa, sx[keep]), np.interp(g, sa, sy[keep])
        # edge lengths: arc length of the SMOOTHED sensor path (removes the wiggle of
        # edge following and the swing round corners; close to the tape centre line)
        k = np.ones(SMOOTH) / SMOOTH
        pad = SMOOTH // 2
        mx = np.convolve(np.r_[gx[-pad:], gx, gx[:pad]], k, "valid")
        my = np.convolve(np.r_[gy[-pad:], gy, gy[:pad]], k, "valid")
        seg = np.hypot(np.diff(np.r_[mx, mx[0]]), np.diff(np.r_[my, my[0]]))
        arc = np.r_[0, np.cumsum(seg[:-1])]
        laps.append(dict(
            lap=lap, drift=math.hypot(*drift), sa=g, L=sa[-1], x=gx, y=gy,
            h=np.interp(g, sa, h[keep]), arc=arc, arc_len=seg.sum(),
            enc=np.interp(g, sa, enc[keep]), enc_len=enc[-1],
            refl=np.interp(g, sa, refl[keep])))
    if len(laps) == 2:
        align(laps[1], laps[0])
    return laps


def align(mov, ref, iters=40):
    """Gyro drift slowly rotates the map, so lap 2 ends up a little turned against lap 1.
    Both laps drive along the same loop (from inside and from outside), so rotate and
    shift lap 2 as a whole until its points sit on top of lap 1 (rigid ICP)."""
    Q = np.stack([ref["x"], ref["y"]], 1)
    P = np.stack([mov["x"], mov["y"]], 1)
    total_rot = 0.0
    for it in range(iters):
        thr = 12.0 if it < iters // 2 else 6.0
        d = np.hypot(P[:, None, 0] - Q[None, :, 0], P[:, None, 1] - Q[None, :, 1])
        j = d.argmin(1)
        ok = d[np.arange(len(P)), j] < thr
        if ok.sum() < 20:
            break
        A, B = P[ok], Q[j[ok]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        Rm = (U @ Vt).T
        if np.linalg.det(Rm) < 0:
            Vt[-1] *= -1
            Rm = (U @ Vt).T
        P = (P - ca) @ Rm.T + cb
        total_rot += math.degrees(math.atan2(Rm[1, 0], Rm[0, 0]))
    mov["x"], mov["y"] = P[:, 0], P[:, 1]
    mov["h"] = mov["h"] + math.radians(total_rot)
    mov["aligned_deg"] = total_rot


# ---------------------------------------------------------------- detection
def runs(mask, cyclic=True):
    """Index ranges [a, b) where mask is True (joins a run over the lap seam)."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    out, a = [], idx[0]
    for p, q in zip(idx[:-1], idx[1:]):
        if q != p + 1:
            out.append([a, p + 1])
            a = q
    out.append([a, idx[-1] + 1])
    if cyclic and len(out) > 1 and out[0][0] == 0 and out[-1][1] == len(mask):
        out[0] = [out[-1][0] - len(mask), out[0][1]]
        out.pop()
    return out


def pass_counts(laps):
    X = np.concatenate([l["x"] for l in laps])
    Y = np.concatenate([l["y"] for l in laps])
    lap_id = np.concatenate([np.full(len(l["x"]), k) for k, l in enumerate(laps)])
    sa = np.concatenate([l["sa"] for l in laps])
    Ls = np.array([l["L"] for l in laps])[lap_id]
    starts = np.cumsum([0] + [len(l["x"]) for l in laps])
    D = np.hypot(X[:, None] - X[None, :], Y[:, None] - Y[None, :])
    same = lap_id[:, None] == lap_id[None, :]
    darc = np.abs(sa[:, None] - sa[None, :])
    darc = np.minimum(darc, Ls[:, None] - darc)
    near = (D < PASS_R) & (~same | (darc > ARC_GAP))
    counts = np.ones(len(X), dtype=int)
    for i in range(len(X)):
        for k in range(len(laps)):
            a, b = starts[k], starts[k + 1]
            sel = np.flatnonzero(near[i, a:b])
            if len(sel) == 0:
                continue
            g = 1 + int(np.sum(np.diff(sel) > 2))
            if g > 1 and sel[0] <= 1 and (b - a - 1 - sel[-1]) <= 1:
                g -= 1                       # one pass split by the lap seam
            counts[i] += g
    return [counts[starts[k]:starts[k + 1]] for k in range(len(laps))]


def find_visits(laps, counts):
    visits = []
    for k, (l, c) in enumerate(zip(laps, counts)):
        n = len(l["sa"])
        # junctions: 3 or more separate passes close together
        jmask = np.zeros(n, bool)
        for a, b in runs(c >= 3):
            if b - a < MIN_RUN:
                continue
            ii = np.arange(a, b) % n
            jmask[ii] = True
            visits.append(dict(lap=k, i=int(ii[len(ii) // 2]), kind="J"))
        # dead ends: the path folds back on itself (out along a spur, round the tip,
        # back along the other side). The middle of the shortest fold is the tip.
        P = np.stack([l["x"], l["y"]], 1)
        gap, fmax = int(ARC_GAP / STEP), int(FOLD_MAX / STEP)
        centers = []
        for i in range(n):
            jj = (i + np.arange(gap, min(fmax, n - 1))) % n
            d = np.hypot(*(P[jj] - P[i]).T)
            hit = np.flatnonzero(d < PASS_R)
            if len(hit) == 0:
                continue
            m = hit[0] + gap                      # first return = shortest fold
            inside = (i + np.arange(MIN_RUN, m - MIN_RUN)) % n
            if jmask[inside].any():               # fold contains a junction: not a dead end
                continue
            if np.hypot(*(P[inside] - P[i]).T).max() < MIN_SPUR:
                continue                          # just the sensor swinging round a corner
            centers.append((i + m // 2) % n)
        centers.sort()
        groups = []
        for cidx in centers:
            if groups and cidx - groups[-1][-1] <= 10:
                groups[-1].append(cidx)
            else:
                groups.append([cidx])
        for g in groups:
            if len(g) >= MIN_RUN:
                visits.append(dict(lap=k, i=int(np.median(g)), kind="E"))
    for v in visits:
        l = laps[v["lap"]]
        v.update(x=l["x"][v["i"]], y=l["y"][v["i"]], sa=l["sa"][v["i"]], arc=l["arc"][v["i"]])
    return visits


def cluster(visits):
    """Group visits of the same kind that are close together into nodes."""
    parent = list(range(len(visits)))

    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a in range(len(visits)):
        for b in range(a + 1, len(visits)):
            va, vb = visits[a], visits[b]
            if va["kind"] == vb["kind"] and math.hypot(va["x"] - vb["x"], va["y"] - vb["y"]) < NODE_R:
                parent[root(a)] = root(b)
    groups = {}
    for a in range(len(visits)):
        groups.setdefault(root(a), []).append(a)
    nodes = []
    for members in groups.values():
        vs = [visits[a] for a in members]
        nodes.append(dict(kind=vs[0]["kind"], x=np.mean([v["x"] for v in vs]),
                          y=np.mean([v["y"] for v in vs]), members=members))
    # goal ends have a T-bar: the robot sees a small junction right next to the tip.
    # merge such a junction into the dead end
    for e in [n for n in nodes if n["kind"] == "E"]:
        for j in [n for n in nodes if n["kind"] == "J"]:
            if math.hypot(e["x"] - j["x"], e["y"] - j["y"]) < MERGE_D:
                e["members"] += j["members"]
                nodes.remove(j)
    # readable ids: J1, J2... for junctions, E1, E2... for ends, numbered left to right
    nodes.sort(key=lambda n: (n["kind"] != "J", disp(n["x"], n["y"])[0]))
    cnt = {"J": 0, "E": 0}
    for n in nodes:
        cnt[n["kind"]] += 1
        n["id"] = "{}{}".format(n["kind"], cnt[n["kind"]])
        for a in n["members"]:
            visits[a]["node"] = n["id"]
    return nodes


# ---------------------------------------------------------------- graph
def build_graph(laps, visits):
    edges = {}     # (u, v) sorted -> list of pass dicts
    turns = []     # (node, came_from, went_to, tape_side)
    for k, l in enumerate(laps):
        vs = sorted([v for v in visits if v["lap"] == k], key=lambda v: v["sa"])
        seq = []   # collapse repeated visits of the same node
        for v in vs:
            if seq and seq[-1]["node"] == v["node"]:
                seq[-1]["vs"].append(v)
            else:
                seq.append(dict(node=v["node"], vs=[v]))
        if len(seq) > 1 and seq[0]["node"] == seq[-1]["node"]:
            seq[0]["vs"] = seq[-1]["vs"] + seq[0]["vs"]
            seq.pop()
        for g in seq:                       # node position along the lap = its middle visit
            mid = g["vs"][len(g["vs"]) // 2]
            g["arc"], g["i"] = mid["arc"], mid["i"]
        n = len(seq)
        for j in range(n):
            a, b = seq[j], seq[(j + 1) % n]
            length = b["arc"] - a["arc"]
            enc = l["enc"][b["i"]] - l["enc"][a["i"]]
            if length <= 0:
                length += l["arc_len"]
                enc += l["enc_len"]
            key = tuple(sorted((a["node"], b["node"])))
            if a["node"] != b["node"]:
                edges.setdefault(key, []).append(dict(lap=k, i0=a["i"], i1=b["i"], length=length,
                                                      enc=enc, frm=a["node"], to=b["node"],
                                                      side=TAPE[l["lap"]]))
            prev_node = seq[(j - 1) % n]["node"]
            turns.append((a["node"], prev_node, b["node"], TAPE[l["lap"]]))
    return edges, turns


def exit_order(nodes, edges, turns):
    """sigma[node][a] = exit taken when arriving from a with the tape on the LEFT."""
    nbrs = {n["id"]: set() for n in nodes}
    for u, v in edges:
        nbrs[u].add(v)
        nbrs[v].add(u)
    sigma = {n: {} for n in nbrs}
    for node, came, went, side in turns:
        if came == node or went == node:
            continue
        if side == "L":
            sigma[node][came] = went
        else:
            sigma[node][went] = came
    for node, s in sigma.items():          # degree 3: one known pair gives the whole cycle
        nb = nbrs[node]
        if len(nb) == 3 and s:
            a, b = next(iter(s.items()))
            c = (nb - {a, b}).pop()
            s.update({a: b, b: c, c: a})
    return nbrs, sigma


def dijkstra(edges, start, goal):
    adj = {}
    for (u, v), passes in edges.items():
        w = float(np.mean([p["length"] for p in passes]))
        adj.setdefault(u, []).append((v, w))
        adj.setdefault(v, []).append((u, w))
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, 1e18):
            continue
        for v, w in adj.get(u, []):
            if d + w < dist.get(v, 1e18):
                dist[v], prev[v] = d + w, u
                heapq.heappush(pq, (d + w, v))
    if goal not in dist:
        return None, None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1], dist[goal]


def route_sides(path, edges, sigma):
    out = []
    for k in range(len(path) - 1):
        u, v = path[k], path[k + 1]
        length = float(np.mean([p["length"] for p in edges[tuple(sorted((u, v)))]]))
        if k + 2 < len(path):
            w = path[k + 2]
            if sigma[v].get(u) == w:
                side = "L"
            elif sigma[v].get(w) == u:
                side = "R"
            else:
                raise SystemExit("cannot choose exit {} at {} coming from {}".format(w, v, u))
        else:
            side = out[-1][0] if out else "L"
        # the run counts wheel-encoder distance, so use the encoder distance of the
        # explore pass that drove along the SAME side of this line
        enc = None
        for p in edges[tuple(sorted((u, v)))]:
            phys = p["side"] if (p["frm"], p["to"]) == (u, v) else ("L" if p["side"] == "R" else "R")
            if phys == side:
                enc = p["enc"]
        if enc is None:
            enc = length
        out.append((side, round(enc, 1), u, v))
    return out


# ---------------------------------------------------------------- output
def plot_map(laps, nodes, edges, route, fname):
    fig, ax = plt.subplots(figsize=(11, 7))
    colors = {0: "tab:blue", 1: "tab:orange"}
    for k, l in enumerate(laps):
        side = "left" if TAPE[l["lap"]] == "L" else "right"
        ax.plot(*disp(l["x"], l["y"]), lw=1, color=colors[k],
                label="lap {} (tape on {}), loop-closure drift {:.1f} cm".format(l["lap"], side, l["drift"]))
    if route:
        for side, _, u, v in route:
            p = edges[tuple(sorted((u, v)))][0]
            l = laps[p["lap"]]
            n = len(l["x"])
            ii = np.arange(p["i0"], p["i1"] + (n if p["i1"] < p["i0"] else 0) + 1) % n
            ax.plot(*disp(l["x"][ii], l["y"][ii]), lw=5, color="tab:red", alpha=0.5)
        ax.plot([], [], lw=5, color="tab:red", alpha=0.5, label="planned route")
    for (u, v), passes in edges.items():
        p = passes[0]
        l = laps[p["lap"]]
        n = len(l["x"])
        i1 = p["i1"] + (n if p["i1"] < p["i0"] else 0)
        im = ((p["i0"] + i1) // 2) % n
        ax.annotate("{:.0f}".format(np.mean([q["length"] for q in passes])), disp(l["x"][im], l["y"][im]),
                    fontsize=8, color="dimgray", ha="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    for n in nodes:
        mk = "s" if n["kind"] == "J" else "o"
        ax.plot(*disp(n["x"], n["y"]), mk, ms=10, color="black" if n["kind"] == "J" else "tab:green")
        ax.annotate(n["id"], disp(n["x"], n["y"]), xytext=(6, 6), textcoords="offset points",
                    fontsize=11, weight="bold")
    ax.plot([], [], "s", color="black", label="junction (3 passes)")
    ax.plot([], [], "o", color="tab:green", label="dead end (path folds back)")
    ax.set_aspect("equal")
    ax.set_xlabel("cm")
    ax.set_ylabel("cm")
    ax.set_title("Map built from explore log (sensor path); edge labels = length in cm")
    ax.legend(fontsize=8, loc="best")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(fname, dpi=150)


def plot_signals(laps, counts, visits, fname):
    fig, axes = plt.subplots(3, len(laps), figsize=(6 * len(laps), 7), sharex="col", squeeze=False)
    for k, (l, c) in enumerate(zip(laps, counts)):
        a0, a1, a2 = axes[:, k]
        a0.plot(l["sa"], l["refl"], lw=0.7)
        a0.set_ylabel("reflection (%)")
        a0.set_title("lap {} (tape on {})".format(l["lap"], "left" if TAPE[l["lap"]] == "L" else "right"))
        a1.plot(l["sa"], np.degrees(l["h"]), lw=0.8, color="tab:purple")
        a1.set_ylabel("heading (deg)")
        a2.step(l["sa"], c, where="mid", lw=0.8, color="black")
        a2.set_ylabel("passes nearby")
        a2.set_xlabel("distance along sensor path (cm)")
        a2.set_yticks([1, 2, 3, 4])
        for v in visits:
            if v["lap"] == k:
                for ax in (a0, a1, a2):
                    ax.axvline(v["sa"], color="tab:red" if v["kind"] == "J" else "tab:green", lw=0.6, alpha=0.6)
                a2.annotate(v.get("node", ""), (v["sa"], 3.6), fontsize=7, ha="center")
    fig.suptitle("What the robot saw while exploring (red = junction visit, green = dead end)")
    fig.tight_layout()
    fig.savefig(fname, dpi=150)


def undisp(px, py):
    """drawing frame (map.png) -> robot frame"""
    a = -math.radians(START_DIR)
    return px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a)


def visits_from_file(laps, path):
    """Manual node mode: node positions are read off map.png by hand (name,x,y per line).
    A visit is every pass of the sensor path within HIT_R of a node (closest point of that
    pass). Exits, edge lengths and the route are then measured from the log as usual."""
    nodes, pts = [], []
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line or line.lower().startswith("name"):
                continue
            name, px, py = [s.strip() for s in line.split(",")[:3]]
            x, y = undisp(float(px), float(py))
            pts.append((name, x, y))               # a name may appear more than once (e.g. a wide pad)
    names = []
    for name, _, _ in pts:
        if name not in names:
            names.append(name)
    for name in names:
        xs = [x for nm, x, y in pts if nm == name]
        ys = [y for nm, x, y in pts if nm == name]
        nodes.append(dict(id=name, kind="J", x=float(np.mean(xs)), y=float(np.mean(ys)), members=[]))
    NX = np.array([x for _, x, _ in pts])
    NY = np.array([y for _, _, y in pts])
    owner = np.array([names.index(nm) for nm, _, _ in pts])
    visits = []
    for k, l in enumerate(laps):
        D = np.hypot(l["x"][:, None] - NX[None, :], l["y"][:, None] - NY[None, :])
        hit = D.min(1) <= HIT_R
        near = owner[D.argmin(1)]
        D = np.stack([D[:, owner == q].min(1) for q in range(len(names))], 1)
        near[~hit] = -1                              # not close to any node
        n = len(near)
        i = 0
        while i < n:
            if near[i] < 0:
                i += 1
                continue
            j = i
            while j + 1 < n and near[j + 1] == near[i]:
                j += 1
            seg = np.arange(i, j + 1)
            best = int(seg[np.argmin(D[seg, near[i]])])
            visits.append(dict(lap=k, i=best, kind="J", node=nodes[near[i]]["id"],
                               x=l["x"][best], y=l["y"][best], sa=l["sa"][best], arc=l["arc"][best]))
            nodes[near[i]]["members"].append(len(visits) - 1)
            i = j + 1
    return visits, nodes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--start")
    ap.add_argument("--goal")
    ap.add_argument("--fix-drift", action="store_true",
                    help="remove gyro drift so each lap turns exactly 360 deg")
    ap.add_argument("--nodes", help="file with hand-labelled node positions from map.png (name,x,y)")
    ap.add_argument("--heading", choices=["gyro", "wheels"], default="gyro",
                    help="where the robot's heading comes from (default gyro)")
    args = ap.parse_args()

    laps = prepare_laps(choose_heading(load(args.csv), args.heading, args.fix_drift))
    counts = pass_counts(laps)
    if args.nodes:
        visits, nodes = visits_from_file(laps, args.nodes)
    else:
        visits = find_visits(laps, counts)
        nodes = cluster(visits)
    edges, turns = build_graph(laps, visits)
    # a real junction has 3 exits; a "junction" with only 2 is a corner that looked
    # busy (e.g. next to the start mark) -> drop it and join its two edges
    while not args.nodes:
        deg = {}
        for u, v in edges:
            deg[u] = deg.get(u, 0) + 1
            deg[v] = deg.get(v, 0) + 1
        fake = {n["id"] for n in nodes if n["kind"] == "J" and deg.get(n["id"], 0) == 2}
        if not fake:
            break
        visits = [v for v in visits if v.get("node") not in fake]
        nodes = [n for n in nodes if n["id"] not in fake]
        edges, turns = build_graph(laps, visits)
    nbrs, sigma = exit_order(nodes, edges, turns)
    if args.nodes:                                   # kind from the measured number of exits
        for n in nodes:
            n["kind"] = "E" if len(nbrs[n["id"]]) == 1 else "J"

    print("NODES")
    with open("nodes.csv", "w") as f:
        f.write("node,kind,x_cm,y_cm,degree,visits,exits\n")
        for n in nodes:
            ex = " ".join(sorted(nbrs[n["id"]]))
            px, py = disp(n["x"], n["y"])         # same frame as map.png
            print("  {:4s} {:9s} at ({:6.1f},{:6.1f})  degree {}  exits: {}".format(
                n["id"], "junction" if n["kind"] == "J" else "dead end", px, py,
                len(nbrs[n["id"]]), ex))
            f.write("{},{},{:.1f},{:.1f},{},{},{}\n".format(
                n["id"], n["kind"], px, py, len(nbrs[n["id"]]), len(n["members"]), ex))
    print("EDGES (cm; each line was measured once per side)")
    with open("edges.csv", "w") as f:
        f.write("from,to,length_cm,pass_lengths_cm\n")
        for (u, v), passes in sorted(edges.items()):
            ls = [p["length"] for p in passes]
            print("  {:4s} - {:4s} {:6.1f}   passes: {}".format(u, v, np.mean(ls), ", ".join("%.1f" % q for q in ls)))
            f.write("{},{},{:.1f},{}\n".format(u, v, np.mean(ls), " ".join("%.1f" % q for q in ls)))
    for l in laps:
        print("lap {}: {:.0f} cm, loop-closure drift {:.1f} cm{}".format(l["lap"], l["arc_len"], l["drift"],
              ", rotated {:.1f} deg onto lap 1".format(l["aligned_deg"]) if "aligned_deg" in l else ""))

    route = None
    if args.start and args.goal:
        path, cost = dijkstra(edges, args.start, args.goal)
        if path is None:
            raise SystemExit("no path between {} and {}".format(args.start, args.goal))
        route = route_sides(path, edges, sigma)
        print("ROUTE {}  ({:.1f} cm)".format(" -> ".join(path), cost))
        with open("route.txt", "w") as f:
            f.write("# route {}  total {:.1f} cm\n".format(" -> ".join(path), cost))
            f.write("ROUTE = [   # (tape on robot's L/R side, wheel distance cm on that side)\n")
            for side, length, u, v in route:
                f.write('    ("{}", {}),   # {} -> {}\n'.format(side, length, u, v))
            f.write("]\n")
        print(open("route.txt").read())
        print("Start: robot at {} on the line towards {}, tape on its {}.".format(
            path[0], path[1], "LEFT" if route[0][0] == "L" else "RIGHT"))

    plot_map(laps, nodes, edges, route, "map.png")
    plot_signals(laps, counts, visits, "explore_signals.png")
    print("saved map.png, explore_signals.png, nodes.csv, edges.csv" + (", route.txt" if route else ""))


if __name__ == "__main__":
    main()
