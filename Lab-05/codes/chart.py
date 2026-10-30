#!/usr/bin/env python3

import argparse                                               # import command-line argument parsing functions
import csv                                                    # import CSV reading functions
import heapq                                                  # import priority queue functions for shortest-path search
import math                                                   # import angle conversion, distance and trigonometric functions

import numpy as np                                            # import NumPy for arrays and numeric calculations
import matplotlib.pyplot as plt                               # import Matplotlib plotting functions

SENSOR_OFFSET = 7.0                                           # set axle-center to color-sensor offset to 7 cm; measure on actual robot
WHEEL_R = 2.8                                                 # set wheel radius to 2.8 cm for wheel-based heading calculation
TRACK = 11.8                                                  # set wheel spacing to 11.8 cm for wheel-based heading calculation
STEP = 1.0                                                    # resample sensor trajectory at 1 cm distance intervals
SMOOTH = 9                                                    # use 9 samples for path smoothing when measuring edge lengths
PASS_R = 7.0                                                  # treat separated path passes within 7 cm as spatially near
ARC_GAP = 20.0                                                # ignore same-lap neighbors within 20 cm along recorded path
FOLD_MAX = 200.0                                              # set maximum dead-end out-and-back search length to 200 cm
MIN_SPUR = 5.0                                                # require candidate dead-end spur to extend at least 5 cm
MIN_RUN = 3                                                   # require at least 3 samples for a detection group
DRIFT_MIN = 45                                                # apply optional gyro drift correction only at 45 degrees error or more
NODE_R = 10.0                                                 # group same-kind visits within 10 cm into one node
MERGE_D = 8.0                                                 # merge junction into nearby dead end when separation is under 8 cm
HIT_R = 6.0                                                   # set manual-node visit radius to 6 cm
TAPE = {1: "L", 2: "R"}                                       # record tape on left in lap 1 and right in lap 2
START_DIR = 0                                                 # set drawing rotation to 0 degrees, robot start direction shown right/east
                      # (0 = right/east, 90 = up/north), only rotates the plot


def disp(x, y):                                               # function for rotating robot coordinates into drawing coordinates
    """robot frame -> drawing frame (rotated so the plot looks like the board)"""
    a = math.radians(START_DIR)                               # convert configured drawing rotation from degrees into radians
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)  # return x and y after applying drawing rotation


# ---------------------------------------------------------------- loading
def load(path):                                               # function for loading CSV measurement columns into numeric arrays
    with open(path) as f:                                     # open selected exploration log for reading
        rows = list(csv.DictReader(f))                        # read each CSV row as a dictionary using its header
    return {c: np.array([float(r[c]) for r in rows]) for c in rows[0]}  # convert each column into a NumPy array of numeric values


def choose_heading(d, source, fix_drift=False):               # function for selecting heading source and optionally correcting gyro drift
    """Compare gyro heading with heading from the wheels; optionally use the wheels.
    Each lap starts and ends at the same mark facing the same way, so the true net
    turn per lap is a whole number of turns (+-360). The source closer to that is
    the one to trust."""
    has_wheels = "left_deg" in d                              # check whether log includes left wheel encoder column
    if has_wheels:                                            # calculate wheel-derived heading when wheel data is available
        wheel = np.degrees(np.radians(d["right_deg"] - d["left_deg"]) * WHEEL_R / TRACK)  # convert right-minus-left wheel rotation into robot heading in degrees
        wheel -= wheel[0]                                     # make first wheel-derived heading equal to 0 degrees
    print("HEADING CHECK (net turn per lap, should be close to +-360):")  # print title explaining per-lap heading comparison
    for lap in (1, 2):                                        # check both recorded exploration laps
        idx = np.flatnonzero((d["lap"] == lap) & (d["mode"] == 0))  # select following-mode samples belonging to this lap
        if len(idx) < 10:                                     # check if lap has fewer than 10 selected samples
            continue                                          # skip heading check for insufficient lap data
        s0, s1 = idx[:30], idx[-30:]                          # select up to first and last 30 samples for averaged headings
        line = "  lap {}: gyro {:+6.0f} deg".format(lap, d["heading_deg"][s1].mean() - d["heading_deg"][s0].mean())  # format net gyro heading change between averaged end and start samples
        if has_wheels:                                        # include wheel comparison when encoder data is available
            line += "   wheels {:+6.0f} deg".format(wheel[s1].mean() - wheel[s0].mean())  # append net wheel-derived heading change to diagnostic line
        print(line)                                           # print heading comparison for this lap
    if fix_drift:                                             # check whether optional drift correction was requested
        # each lap ends where it started, facing the same way -> the gyro must show a whole
        # number of turns. The rest is gyro drift: remove it, growing linearly with time.
        hd = d["heading_deg"].astype(float).copy()            # copy logged gyro headings as floating-point values
        offset, applied = np.zeros(len(hd)), 0.0              # create per-sample correction array and cumulative applied correction
        for lap in (1, 2):                                    # process drift correction separately for each lap
            idx = np.flatnonzero((d["lap"] == lap) & (d["mode"] == 0))  # select following-mode samples for current lap
            if len(idx) < 60:                                 # check if fewer than 60 samples are available for correction
                continue                                      # skip drift correction for insufficient lap data
            a, b = idx[0], idx[-1]                            # get first and last selected sample indices
            net = hd[idx[-30:]].mean() - hd[idx[:30]].mean()  # calculate net gyro turn using averaged end and start headings
            err = net - 360 * round(net / 360)                # calculate difference from nearest complete 360-degree rotation
            if abs(err) < DRIFT_MIN:                          # check if heading mismatch is smaller than correction threshold
                print("  lap {}: only {:+.0f} deg off, no drift correction needed".format(lap, err))  # print that this small mismatch is left uncorrected
                continue                                      # skip drift correction for this lap
            tt = d["time_ms"]                                 # read logged times in milliseconds
            offset[a:b + 1] = applied + err * (tt[a:b + 1] - tt[a]) / max(tt[b] - tt[a], 1)  # distribute this lap's heading correction linearly with elapsed time
            offset[b + 1:] = applied + err                    # carry accumulated correction into all later samples
            applied += err                                    # add this lap's heading mismatch to accumulated correction
            print("  lap {}: removed {:+.0f} deg of drift ({:+.2f} deg/s)".format(  # format diagnostic message about applied gyro correction
                lap, -err, -err / max((tt[b] - tt[a]) / 1000, 1)))  # print lap number, signed correction and estimated correction rate
        hd -= offset                                          # subtract correction array from copied gyro headings
        d["heading_deg"] = hd                                 # replace logged heading column with corrected values
        ds = np.r_[0, np.diff(d["arc_cm"])]                   # calculate signed distance increments with initial increment 0
        d["x_cm"] = np.cumsum(ds * np.cos(np.radians(hd)))    # recalculate x coordinates from corrected gyro headings
        d["y_cm"] = np.cumsum(ds * np.sin(np.radians(hd)))    # recalculate y coordinates from corrected gyro headings
    if source == "wheels":                                    # check if user requested wheel-based heading instead of gyro
        if not has_wheels:                                    # check if required wheel encoder data is missing
            raise SystemExit("this log has no wheel angles (old explore version) - use --heading gyro")  # stop with instruction to use gyro heading for this older log
        d["heading_deg"] = wheel                              # replace heading column with wheel-derived angles
        ds = np.r_[0, np.diff(d["arc_cm"])]                   # calculate signed distance increments from logged travel
        d["x_cm"] = np.cumsum(ds * np.cos(np.radians(wheel)))  # recalculate x coordinates using wheel-derived headings
        d["y_cm"] = np.cumsum(ds * np.sin(np.radians(wheel)))  # recalculate y coordinates using wheel-derived headings
    print("  using heading from:", source)                    # print which heading source was selected
    return d                                                  # return measurement dictionary with selected heading and coordinates


def prepare_laps(d):                                          # function for correcting lap closure and preparing sampled sensor paths
    """Per lap: loop-closure correction, sensor position, resample every STEP cm."""
    laps = []                                                 # create empty list for prepared laps
    for lap in (1, 2):                                        # prepare lap 1 and lap 2 separately
        m = (d["lap"] == lap) & (d["mode"] == 0)              # select following-mode rows for current lap
        if m.sum() < 10:                                      # check if fewer than 10 following samples are present
            continue                                          # skip lap if too little data is available
        t = d["time_ms"][m]                                   # read selected timestamps
        enc = d["arc_cm"][m] - d["arc_cm"][m][0]              # make encoder travel distance relative to beginning of lap
        x, y = d["x_cm"][m].copy(), d["y_cm"][m].copy()       # copy estimated x and y positions for current lap
        h = np.radians(d["heading_deg"][m])                   # convert selected heading angles into radians
        refl = d["reflection"][m]                             # read reflected light measurements for current lap
        # lap starts and ends at the same mark -> spread the end error along the lap
        frac = (t - t[0]) / max(t[-1] - t[0], 1e-9)           # calculate elapsed fraction of lap, protecting against zero duration
        drift = (x[-1] - x[0], y[-1] - y[0])                  # measure estimated endpoint offset from lap start
        x -= drift[0] * frac                                  # spread x closing offset linearly across lap samples
        y -= drift[1] * frac                                  # spread y closing offset linearly across lap samples
        sx, sy = x + SENSOR_OFFSET * np.cos(h), y + SENSOR_OFFSET * np.sin(h)  # estimate sensor position by adding forward offset along heading
        sa = np.concatenate([[0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])  # calculate accumulated geometric length along sensor path
        sa, keep = np.unique(sa, return_index=True)           # remove duplicate path distances and retain corresponding sample indices
        g = np.arange(0, sa[-1], STEP)                        # create uniform distance grid using 1 cm step
        gx, gy = np.interp(g, sa, sx[keep]), np.interp(g, sa, sy[keep])  # interpolate sensor x and y coordinates onto distance grid
        # edge lengths: arc length of the SMOOTHED sensor path (removes the wiggle of
        # edge following and the swing round corners; close to the tape centre line)
        k = np.ones(SMOOTH) / SMOOTH                          # create equal-weight 9-sample smoothing kernel
        pad = SMOOTH // 2                                     # calculate number of wraparound padding samples on each side
        mx = np.convolve(np.r_[gx[-pad:], gx, gx[:pad]], k, "valid")  # smooth x coordinates with cyclic padding
        my = np.convolve(np.r_[gy[-pad:], gy, gy[:pad]], k, "valid")  # smooth y coordinates with cyclic padding
        seg = np.hypot(np.diff(np.r_[mx, mx[0]]), np.diff(np.r_[my, my[0]]))  # calculate smoothed segment lengths including last-to-first connection
        arc = np.r_[0, np.cumsum(seg[:-1])]                   # build cumulative smoothed arc distance for each sampled point
        laps.append(dict(                                     # append prepared lap measurements and geometry as a dictionary
            lap=lap, drift=math.hypot(*drift), sa=g, L=sa[-1], x=gx, y=gy,  # store lap label, original closing error, sampling distances and positions
            h=np.interp(g, sa, h[keep]), arc=arc, arc_len=seg.sum(),  # store interpolated headings and smoothed path distances and length
            enc=np.interp(g, sa, enc[keep]), enc_len=enc[-1],  # store interpolated encoder travel and full encoder distance
            refl=np.interp(g, sa, refl[keep])))               # store interpolated reflection readings and finish lap record
    if len(laps) == 2:                                        # check if both laps were prepared successfully
        align(laps[1], laps[0])                               # align second lap onto first lap using rigid transformation
    return laps                                               # return prepared lap records


def align(mov, ref, iters=40):                                # function for rotating and shifting moving lap onto reference lap
    """Gyro drift slowly rotates the map, so lap 2 ends up a little turned against lap 1.
    Both laps drive along the same loop (from inside and from outside), so rotate and
    shift lap 2 as a whole until its points sit on top of lap 1 (rigid ICP)."""
    Q = np.stack([ref["x"], ref["y"]], 1)                     # combine reference x and y coordinates into point pairs
    P = np.stack([mov["x"], mov["y"]], 1)                     # combine moving x and y coordinates into point pairs
    total_rot = 0.0                                           # start total applied rotation at 0 degrees
    for it in range(iters):                                   # repeat alignment for requested number of iterations
        thr = 12.0 if it < iters // 2 else 6.0                # use 12 cm matching radius initially, then tighten to 6 cm
        d = np.hypot(P[:, None, 0] - Q[None, :, 0], P[:, None, 1] - Q[None, :, 1])  # calculate distance from every moving point to every reference point
        j = d.argmin(1)                                       # find nearest reference point for each moving point
        ok = d[np.arange(len(P)), j] < thr                    # keep matched pairs closer than current matching threshold
        if ok.sum() < 20:                                     # check if fewer than 20 close matches remain
            break                                             # stop alignment when too few point pairs are available
        A, B = P[ok], Q[j[ok]]                                # collect accepted moving points and their matched reference points
        ca, cb = A.mean(0), B.mean(0)                         # calculate centers of the two matched point sets
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))       # use singular value decomposition to estimate best rigid rotation
        Rm = (U @ Vt).T                                       # build rotation matrix from decomposition results
        if np.linalg.det(Rm) < 0:                             # check if matrix would reflect rather than only rotate points
            Vt[-1] *= -1                                      # flip final singular-vector sign to remove reflection
            Rm = (U @ Vt).T                                   # rebuild proper rotation matrix
        P = (P - ca) @ Rm.T + cb                              # rotate moving points about their center and shift onto reference center
        total_rot += math.degrees(math.atan2(Rm[1, 0], Rm[0, 0]))  # add applied rotation angle to total alignment rotation
    mov["x"], mov["y"] = P[:, 0], P[:, 1]                     # save aligned moving-lap x and y coordinates
    mov["h"] = mov["h"] + math.radians(total_rot)             # rotate moving-lap headings by same total alignment angle
    mov["aligned_deg"] = total_rot                            # store total applied alignment rotation for output report


# ---------------------------------------------------------------- detection
def runs(mask, cyclic=True):                                  # function for finding consecutive True ranges in a mask
    """Index ranges [a, b) where mask is True (joins a run over the lap seam)."""
    idx = np.flatnonzero(mask)                                # get indices where mask is True
    if len(idx) == 0:                                         # check whether mask has no True samples
        return []                                             # return empty list when no runs exist
    out, a = [], idx[0]                                       # start range list and store first run's initial index
    for p, q in zip(idx[:-1], idx[1:]):                       # compare each pair of consecutive True indices
        if q != p + 1:                                        # check if a gap separates current index from next one
            out.append([a, p + 1])                            # save completed run with exclusive end index
            a = q                                             # start next run at index after the gap
    out.append([a, idx[-1] + 1])                              # save final detected run
    if cyclic and len(out) > 1 and out[0][0] == 0 and out[-1][1] == len(mask):  # check if first and last runs connect across cyclic lap boundary
        out[0] = [out[-1][0] - len(mask), out[0][1]]          # join boundary-spanning run using negative start index
        out.pop()                                             # remove last run because it was joined into first run
    return out                                                # return detected index ranges


def pass_counts(laps):                                        # function for counting separate path passes near each sampled point
    X = np.concatenate([l["x"] for l in laps])                # join x coordinates from all laps
    Y = np.concatenate([l["y"] for l in laps])                # join y coordinates from all laps
    lap_id = np.concatenate([np.full(len(l["x"]), k) for k, l in enumerate(laps)])  # record which lap owns each combined point
    sa = np.concatenate([l["sa"] for l in laps])              # join distance-along-path coordinates from all laps
    Ls = np.array([l["L"] for l in laps])[lap_id]             # assign each combined point the total sensor-path length of its lap
    starts = np.cumsum([0] + [len(l["x"]) for l in laps])     # calculate start and end offsets of each lap in combined arrays
    D = np.hypot(X[:, None] - X[None, :], Y[:, None] - Y[None, :])  # calculate pairwise spatial distances between all path points
    same = lap_id[:, None] == lap_id[None, :]                 # mark point pairs belonging to same lap
    darc = np.abs(sa[:, None] - sa[None, :])                  # calculate pairwise separation along recorded sensor paths
    darc = np.minimum(darc, Ls[:, None] - darc)               # use shorter cyclic path separation for same-lap comparisons
    near = (D < PASS_R) & (~same | (darc > ARC_GAP))          # find spatially near passes while excluding nearby same-path samples
    counts = np.ones(len(X), dtype=int)                       # start every point's pass count at 1 for its own pass
    for i in range(len(X)):                                   # process every combined path point
        for k in range(len(laps)):                            # look for additional passes in each lap
            a, b = starts[k], starts[k + 1]                   # get current lap's section of combined arrays
            sel = np.flatnonzero(near[i, a:b])                # find samples in this lap that pass near current point
            if len(sel) == 0:                                 # check if no nearby samples were found
                continue                                      # skip this lap when it contributes no nearby pass
            g = 1 + int(np.sum(np.diff(sel) > 2))             # count separated groups of nearby samples using gaps above 2 indices
            if g > 1 and sel[0] <= 1 and (b - a - 1 - sel[-1]) <= 1:  # check if one group is split between start and end of cyclic lap
                g -= 1                                        # count that boundary-spanning group only once
            counts[i] += g                                    # add detected nearby pass groups to current point's count
    return [counts[starts[k]:starts[k + 1]] for k in range(len(laps))]  # split combined counts back into one array per lap


def find_visits(laps, counts):                                # function for detecting junction and dead-end visits from recorded paths
    visits = []                                               # create empty list for candidate node visits
    for k, (l, c) in enumerate(zip(laps, counts)):            # process each prepared lap together with its nearby-pass counts
        n = len(l["sa"])                                      # store number of resampled points in current lap
        # junctions: 3 or more separate passes close together
        jmask = np.zeros(n, bool)                             # create empty mask for junction regions
        for a, b in runs(c >= 3):                             # find runs with at least three nearby passes
            if b - a < MIN_RUN:                               # check if candidate junction run is shorter than minimum detection size
                continue                                      # ignore short candidate as noise
            ii = np.arange(a, b) % n                          # wrap candidate run indices around cyclic lap boundary
            jmask[ii] = True                                  # mark these samples as a junction region
            visits.append(dict(lap=k, i=int(ii[len(ii) // 2]), kind="J"))  # record middle sample as a junction visit
        # dead ends: the path folds back on itself (out along a spur, round the tip,
        # back along the other side). The middle of the shortest fold is the tip.
        P = np.stack([l["x"], l["y"]], 1)                     # combine lap x and y coordinates into path points
        gap, fmax = int(ARC_GAP / STEP), int(FOLD_MAX / STEP)  # convert minimum path gap and maximum fold length into sample counts
        centers = []                                          # create list of candidate dead-end center indices
        for i in range(n):                                    # search for an out-and-back fold starting at each path point
            jj = (i + np.arange(gap, min(fmax, n - 1))) % n   # select later cyclic path samples within allowed fold search range
            d = np.hypot(*(P[jj] - P[i]).T)                   # calculate spatial distances from current point to searched samples
            hit = np.flatnonzero(d < PASS_R)                  # find searched samples that return within pass radius
            if len(hit) == 0:                                 # check if no spatial return was found
                continue                                      # skip current point if path does not return nearby
            m = hit[0] + gap                                  # take earliest nearby return as shortest candidate fold
            inside = (i + np.arange(MIN_RUN, m - MIN_RUN)) % n  # select interior samples of candidate out-and-back segment
            if jmask[inside].any():                           # check if fold interior contains a detected junction
                continue                                      # reject fold containing a junction as a dead-end candidate
            if np.hypot(*(P[inside] - P[i]).T).max() < MIN_SPUR:  # check if fold never extends far enough from its start
                continue                                      # ignore small fold as cornering or sensor swing
            centers.append((i + m // 2) % n)                  # save midpoint index of accepted out-and-back fold
        centers.sort()                                        # sort candidate dead-end centers in path order
        groups = []                                           # create empty groups of neighboring dead-end candidates
        for cidx in centers:                                  # process each candidate center index
            if groups and cidx - groups[-1][-1] <= 10:        # check if candidate is within 10 samples of previous group's end
                groups[-1].append(cidx)                       # append nearby candidate to current group
            else:                                             # otherwise begin a separate candidate group
                groups.append([cidx])                         # create new group containing this candidate
        for g in groups:                                      # process each candidate center group
            if len(g) >= MIN_RUN:                             # require at least minimum number of supporting candidates
                visits.append(dict(lap=k, i=int(np.median(g)), kind="E"))  # record group median as a dead-end visit
    for v in visits:                                          # add geometric information to every detected visit
        l = laps[v["lap"]]                                    # get lap record associated with visit
        v.update(x=l["x"][v["i"]], y=l["y"][v["i"]], sa=l["sa"][v["i"]], arc=l["arc"][v["i"]])  # store visit position, raw sensor-path distance and smoothed arc distance
    return visits                                             # return detected junction and dead-end visits


def cluster(visits):                                          # function for grouping nearby same-kind visits into graph nodes
    """Group visits of the same kind that are close together into nodes."""
    parent = list(range(len(visits)))                         # start each visit as its own disjoint-set group

    def root(a):                                              # function for finding representative of a visit group
        while parent[a] != a:                                 # follow parent links until reaching group root
            parent[a] = parent[parent[a]]                     # shorten parent chain by linking to grandparent
            a = parent[a]                                     # move to next parent in shortened chain
        return a                                              # return representative visit index

    for a in range(len(visits)):                              # compare each visit with later visits
        for b in range(a + 1, len(visits)):                   # process each remaining visit pair once
            va, vb = visits[a], visits[b]                     # read current pair of visit records
            if va["kind"] == vb["kind"] and math.hypot(va["x"] - vb["x"], va["y"] - vb["y"]) < NODE_R:  # check if visits share kind and are closer than node grouping radius
                parent[root(a)] = root(b)                     # join their disjoint-set groups
    groups = {}                                               # create dictionary for member lists of grouped visits
    for a in range(len(visits)):                              # process each visit index
        groups.setdefault(root(a), []).append(a)              # append visit to group identified by its root
    nodes = []                                                # create empty list of graph nodes
    for members in groups.values():                           # process each group of matching visits
        vs = [visits[a] for a in members]                     # collect full visit records for this group
        nodes.append(dict(kind=vs[0]["kind"], x=np.mean([v["x"] for v in vs]),  # start node record with shared kind and mean x coordinate
                          y=np.mean([v["y"] for v in vs]), members=members))  # store mean y coordinate and member visit indices
    # goal ends have a T-bar: the robot sees a small junction right next to the tip.
    # merge such a junction into the dead end
    for e in [n for n in nodes if n["kind"] == "E"]:          # process each detected dead-end node
        for j in [n for n in nodes if n["kind"] == "J"]:      # compare with each junction node
            if math.hypot(e["x"] - j["x"], e["y"] - j["y"]) < MERGE_D:  # check if junction lies within dead-end merge distance
                e["members"] += j["members"]                  # add nearby junction's visits to dead-end node
                nodes.remove(j)                               # remove junction node that was absorbed into dead end
    # readable ids: J1, J2... for junctions, E1, E2... for ends, numbered left to right
    nodes.sort(key=lambda n: (n["kind"] != "J", disp(n["x"], n["y"])[0]))  # sort junctions first, then order each kind by drawing x coordinate
    cnt = {"J": 0, "E": 0}                                    # start separate numbering counters for junctions and dead ends
    for n in nodes:                                           # assign readable identifiers to all remaining nodes
        cnt[n["kind"]] += 1                                   # increase counter for current node's kind
        n["id"] = "{}{}".format(n["kind"], cnt[n["kind"]])    # build identifier such as J1 or E1
        for a in n["members"]:                                # process every visit belonging to this node
            visits[a]["node"] = n["id"]                       # write assigned node identifier into each member visit
    return nodes                                              # return grouped graph nodes


# ---------------------------------------------------------------- graph
def build_graph(laps, visits):                                # function for building graph edges and observed turn relationships
    edges = {}                                                # create dictionary of undirected node pairs and their measured passes
    turns = []                                                # create list of arrival and departure relationships at nodes
    for k, l in enumerate(laps):                              # process visits for each exploration lap
        vs = sorted([v for v in visits if v["lap"] == k], key=lambda v: v["sa"])  # sort this lap's visits by distance along sensor path
        seq = []                                              # create sequence with repeated visits of same node grouped together
        for v in vs:                                          # process each ordered visit
            if seq and seq[-1]["node"] == v["node"]:          # check if visit belongs to same node as previous sequence entry
                seq[-1]["vs"].append(v)                       # add visit to current sequence group
            else:                                             # otherwise start a new node group in sequence
                seq.append(dict(node=v["node"], vs=[v]))      # append new node group containing this visit
        if len(seq) > 1 and seq[0]["node"] == seq[-1]["node"]:  # check if first and last groups repeat same node across lap boundary
            seq[0]["vs"] = seq[-1]["vs"] + seq[0]["vs"]       # join final group's visits with first group's visits
            seq.pop()                                         # remove duplicate group at end of cyclic sequence
        for g in seq:                                         # choose representative position for each grouped node visit
            mid = g["vs"][len(g["vs"]) // 2]                  # select group's middle visit
            g["arc"], g["i"] = mid["arc"], mid["i"]           # store representative smoothed arc distance and sample index
        n = len(seq)                                          # store number of node groups in lap sequence
        for j in range(n):                                    # build edges between each pair of consecutive node groups
            a, b = seq[j], seq[(j + 1) % n]                   # select current node and next node with cyclic wraparound
            length = b["arc"] - a["arc"]                      # calculate smoothed path length between representative visits
            enc = l["enc"][b["i"]] - l["enc"][a["i"]]         # calculate corresponding signed wheel-encoder distance
            if length <= 0:                                   # check if edge crosses lap boundary
                length += l["arc_len"]                        # add full smoothed lap length to wrapped edge length
                enc += l["enc_len"]                           # add full encoder lap distance to wrapped encoder distance
            key = tuple(sorted((a["node"], b["node"])))       # sort node identifiers to create undirected edge key
            if a["node"] != b["node"]:                        # exclude edges that begin and end at same node
                edges.setdefault(key, []).append(dict(lap=k, i0=a["i"], i1=b["i"], length=length,  # start storing this observed pass with lap, sample indices and length
                                                      enc=enc, frm=a["node"], to=b["node"],  # store encoder distance and actual direction of traversal
                                                      side=TAPE[l["lap"]]))  # store which side of robot held tape during this pass
            prev_node = seq[(j - 1) % n]["node"]              # read node visited before current node in cyclic sequence
            turns.append((a["node"], prev_node, b["node"], TAPE[l["lap"]]))  # record arrival, departure and tape side at current node
    return edges, turns                                       # return measured edges and observed turn relationships


def exit_order(nodes, edges, turns):                          # function for inferring neighbor sets and left-tape exit relationships
    """sigma[node][a] = exit taken when arriving from a with the tape on the LEFT."""
    nbrs = {n["id"]: set() for n in nodes}                    # create empty neighbor set for every node
    for u, v in edges:                                        # process each undirected edge
        nbrs[u].add(v)                                        # add second endpoint as neighbor of first endpoint
        nbrs[v].add(u)                                        # add first endpoint as neighbor of second endpoint
    sigma = {n: {} for n in nbrs}                             # create empty arrival-to-exit mapping for each node
    for node, came, went, side in turns:                      # process each observed arrival and departure relationship
        if came == node or went == node:                      # check if arrival or departure incorrectly repeats current node
            continue                                          # ignore self-transition when inferring exit order
        if side == "L":                                       # check if observed pass had tape on robot's left
            sigma[node][came] = went                          # save observed arrival-to-exit relationship directly
        else:                                                 # otherwise observation had tape on robot's right
            sigma[node][went] = came                          # reverse relationship to express left-tape ordering
    for node, s in sigma.items():                             # complete exit ordering separately for each node
        nb = nbrs[node]                                       # read node's neighbor set
        if len(nb) == 3 and s:                                # check if three-neighbor junction has at least one known relationship
            a, b = next(iter(s.items()))                      # take one known arrival and exit pair
            c = (nb - {a, b}).pop()                           # find remaining third neighbor
            s.update({a: b, b: c, c: a})                      # complete cyclic ordering among all three exits
    return nbrs, sigma                                        # return neighbor sets and inferred exit ordering


def dijkstra(edges, start, goal):                             # function for finding shortest route using measured graph edge lengths
    adj = {}                                                  # create empty weighted adjacency dictionary
    for (u, v), passes in edges.items():                      # process each edge and its measured traversal passes
        w = float(np.mean([p["length"] for p in passes]))     # use mean smoothed pass length as edge weight in cm
        adj.setdefault(u, []).append((v, w))                  # add connection from first node to second node
        adj.setdefault(v, []).append((u, w))                  # add reverse connection because graph edges are undirected
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]         # initialize start distance, predecessor dictionary and priority queue
    while pq:                                                 # continue until no candidate nodes remain
        d, u = heapq.heappop(pq)                              # take candidate with smallest queued distance
        if u == goal:                                         # check if destination has been reached
            break                                             # stop searching when goal is removed from priority queue
        if d > dist.get(u, 1e18):                             # check if queued distance is outdated
            continue                                          # skip candidate when a shorter distance is already known
        for v, w in adj.get(u, []):                           # process outgoing neighbors and weights of current node
            if d + w < dist.get(v, 1e18):                     # check if route through current node improves neighbor distance
                dist[v], prev[v] = d + w, u                   # save improved distance and predecessor node
                heapq.heappush(pq, (d + w, v))                # push improved neighbor candidate into priority queue
    if goal not in dist:                                      # check whether goal was never reached
        return None, None                                     # return no path and no cost when goal is unreachable
    path = [goal]                                             # begin backward reconstruction from destination node
    while path[-1] != start:                                  # follow predecessors until start node is reached
        path.append(prev[path[-1]])                           # append previous node in shortest route
    return path[::-1], dist[goal]                             # return route in start-to-goal order and its total length


def route_sides(path, edges, sigma):                          # function for choosing tape side and driving distance for each route segment
    out = []                                                  # create empty list of route instructions
    for k in range(len(path) - 1):                            # process each consecutive node pair in planned route
        u, v = path[k], path[k + 1]                           # read current segment's start and end nodes
        length = float(np.mean([p["length"] for p in edges[tuple(sorted((u, v)))]]))  # calculate mean measured geometric length of this edge
        if k + 2 < len(path):                                 # check if another segment follows this one
            w = path[k + 2]                                   # read node after current segment's endpoint
            if sigma[v].get(u) == w:                          # check if left-tape exit ordering leads to required next node
                side = "L"                                    # select tape on robot's left side
            elif sigma[v].get(w) == u:                        # check if reverse exit relationship requires tape on right
                side = "R"                                    # select tape on robot's right side
            else:                                             # otherwise required exit cannot be inferred
                raise SystemExit("cannot choose exit {} at {} coming from {}".format(w, v, u))  # stop with message identifying unresolved route transition
        else:                                                 # otherwise this is final route segment
            side = out[-1][0] if out else "L"                 # reuse previous tape side, or default to left for single segment
        # the run counts wheel-encoder distance, so use the encoder distance of the
        # explore pass that drove along the SAME side of this line
        enc = None                                            # start without a matching encoder distance
        for p in edges[tuple(sorted((u, v)))]:                # inspect measured passes for current undirected edge
            phys = p["side"] if (p["frm"], p["to"]) == (u, v) else ("L" if p["side"] == "R" else "R")  # convert stored pass side to current route's traversal direction
            if phys == side:                                  # check if this pass used selected physical tape side
                enc = p["enc"]                                # use encoder distance recorded for matching-side pass
        if enc is None:                                       # check if no matching-side encoder distance was found
            enc = length                                      # fall back to mean geometric edge length
        out.append((side, round(enc, 1), u, v))               # store tape side, rounded driving distance and node endpoints
    return out                                                # return generated route instructions


# ---------------------------------------------------------------- output
def plot_map(laps, nodes, edges, route, fname):               # function for plotting reconstructed laps, graph nodes and optional route
    fig, ax = plt.subplots(figsize=(11, 7))                   # create map figure with 11 by 7 inch size
    colors = {0: "tab:blue", 1: "tab:orange"}                 # assign blue and orange colors to the two lap indices
    for k, l in enumerate(laps):                              # draw each prepared lap
        side = "left" if TAPE[l["lap"]] == "L" else "right"   # convert recorded tape-side letter to readable label
        ax.plot(*disp(l["x"], l["y"]), lw=1, color=colors[k],  # plot sensor path in drawing coordinates using lap color
                label="lap {} (tape on {}), loop-closure drift {:.1f} cm".format(l["lap"], side, l["drift"]))  # include lap number, tape side and original closing error in legend label
    if route:                                                 # check if a route was planned
        for side, _, u, v in route:                           # process each planned route segment
            p = edges[tuple(sorted((u, v)))][0]               # select first recorded pass for this route edge
            l = laps[p["lap"]]                                # get lap containing selected pass
            n = len(l["x"])                                   # store number of sampled points in that lap
            ii = np.arange(p["i0"], p["i1"] + (n if p["i1"] < p["i0"] else 0) + 1) % n  # select segment indices, wrapping around lap boundary when needed
            ax.plot(*disp(l["x"][ii], l["y"][ii]), lw=5, color="tab:red", alpha=0.5)  # highlight stored path segment in thick translucent red
        ax.plot([], [], lw=5, color="tab:red", alpha=0.5, label="planned route")  # add legend entry explaining highlighted planned route
    for (u, v), passes in edges.items():                      # label each measured graph edge
        p = passes[0]                                         # select first pass for positioning edge label
        l = laps[p["lap"]]                                    # get lap record containing that pass
        n = len(l["x"])                                       # store number of samples in lap
        i1 = p["i1"] + (n if p["i1"] < p["i0"] else 0)        # unwrap endpoint index if pass crosses lap boundary
        im = ((p["i0"] + i1) // 2) % n                        # find midpoint sample for edge label position
        ax.annotate("{:.0f}".format(np.mean([q["length"] for q in passes])), disp(l["x"][im], l["y"][im]),  # place mean edge length label at segment midpoint
                    fontsize=8, color="dimgray", ha="center",  # set label font size, gray color and centered horizontal alignment
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))  # add light rounded background behind length label
    for n in nodes:                                           # draw each detected graph node
        mk = "s" if n["kind"] == "J" else "o"                 # use square marker for junction and circle for dead end
        ax.plot(*disp(n["x"], n["y"]), mk, ms=10, color="black" if n["kind"] == "J" else "tab:green")  # plot node in drawing coordinates with kind-specific color
        ax.annotate(n["id"], disp(n["x"], n["y"]), xytext=(6, 6), textcoords="offset points",  # place node identifier slightly offset from node position
                    fontsize=11, weight="bold")               # use bold text for node identifier
    ax.plot([], [], "s", color="black", label="junction (3 passes)")  # add legend entry for junction markers
    ax.plot([], [], "o", color="tab:green", label="dead end (path folds back)")  # add legend entry for dead-end markers
    ax.set_aspect("equal")                                    # use equal scale on x and y axes
    ax.set_xlabel("cm")                                       # label horizontal coordinate in cm
    ax.set_ylabel("cm")                                       # label vertical coordinate in cm
    ax.set_title("Map built from explore log (sensor path); edge labels = length in cm")  # set map title and explain edge length labels
    ax.legend(fontsize=8, loc="best")                         # show legend using compact font and automatic position
    ax.grid(alpha=0.3)                                        # show lightly shaded grid
    fig.tight_layout()                                        # adjust spacing to keep plot elements inside figure
    fig.savefig(fname, dpi=150)                               # save map image at 150 dpi using requested filename


def plot_signals(laps, counts, visits, fname):                # function for plotting reflection, heading and nearby passes for each lap
    fig, axes = plt.subplots(3, len(laps), figsize=(6 * len(laps), 7), sharex="col", squeeze=False)  # create three plot rows with one column per lap and shared distance axes
    for k, (l, c) in enumerate(zip(laps, counts)):            # process each lap and its nearby-pass counts
        a0, a1, a2 = axes[:, k]                               # get reflection, heading and pass-count axes for this lap
        a0.plot(l["sa"], l["refl"], lw=0.7)                   # plot reflection against distance along sensor path
        a0.set_ylabel("reflection (%)")                       # label reflected light axis in percent
        a0.set_title("lap {} (tape on {})".format(l["lap"], "left" if TAPE[l["lap"]] == "L" else "right"))  # set column title with lap number and tape side
        a1.plot(l["sa"], np.degrees(l["h"]), lw=0.8, color="tab:purple")  # plot prepared heading in degrees using purple line
        a1.set_ylabel("heading (deg)")                        # label heading axis in degrees
        a2.step(l["sa"], c, where="mid", lw=0.8, color="black")  # plot nearby-pass counts as a step graph
        a2.set_ylabel("passes nearby")                        # label nearby-pass count axis
        a2.set_xlabel("distance along sensor path (cm)")      # label horizontal axis with sensor-path distance in cm
        a2.set_yticks([1, 2, 3, 4])                           # show integer pass-count ticks from 1 through 4
        for v in visits:                                      # mark detected node visits on plots
            if v["lap"] == k:                                 # check if visit belongs to this lap
                for ax in (a0, a1, a2):                       # add visit marker to all three signal plots
                    ax.axvline(v["sa"], color="tab:red" if v["kind"] == "J" else "tab:green", lw=0.6, alpha=0.6)  # draw red vertical line for junction or green line for dead end
                a2.annotate(v.get("node", ""), (v["sa"], 3.6), fontsize=7, ha="center")  # label visit with assigned node identifier when available
    fig.suptitle("What the robot saw while exploring (red = junction visit, green = dead end)")  # set figure title explaining visit marker colors
    fig.tight_layout()                                        # adjust spacing of all signal plots
    fig.savefig(fname, dpi=150)                               # save signal figure at 150 dpi using requested filename


def undisp(px, py):                                           # function for converting drawing coordinates back into robot coordinates
    """drawing frame (map.png) -> robot frame"""
    a = -math.radians(START_DIR)                              # negate drawing rotation and convert it into radians
    return px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a)  # return coordinates after inverse drawing rotation


def visits_from_file(laps, path):                             # function for locating path visits near manually labelled node positions
    """Manual node mode: node positions are read off map.png by hand (name,x,y per line).
    A visit is every pass of the sensor path within HIT_R of a node (closest point of that
    pass). Exits, edge lengths and the route are then measured from the log as usual."""
    nodes, pts = [], []                                       # create empty node list and labelled point list
    with open(path) as f:                                     # open manually labelled node coordinate file
        for line in f:                                        # read each line of node file
            line = line.split("#")[0].strip()                 # remove text after hash symbol and trim surrounding whitespace
            if not line or line.lower().startswith("name"):   # check if line is empty or is a column-header line
                continue                                      # skip blank, comment-only or header line
            name, px, py = [s.strip() for s in line.split(",")[:3]]  # read node name and first two comma-separated coordinates
            x, y = undisp(float(px), float(py))               # convert drawing coordinates into robot coordinate frame
            pts.append((name, x, y))                          # store labelled position, allowing multiple points with same node name
    names = []                                                # create list of distinct node names
    for name, _, _ in pts:                                    # process name from each labelled point
        if name not in names:                                 # check if name has not yet been recorded
            names.append(name)                                # add new name while preserving input order
    for name in names:                                        # create one graph node for each distinct name
        xs = [x for nm, x, y in pts if nm == name]            # collect x coordinates labelled with this name
        ys = [y for nm, x, y in pts if nm == name]            # collect y coordinates labelled with this name
        nodes.append(dict(id=name, kind="J", x=float(np.mean(xs)), y=float(np.mean(ys)), members=[]))  # use mean labelled position and initially classify node as junction
    NX = np.array([x for _, x, _ in pts])                     # create array of all labelled x coordinates
    NY = np.array([y for _, _, y in pts])                     # create array of all labelled y coordinates
    owner = np.array([names.index(nm) for nm, _, _ in pts])   # map each labelled point to index of its named node
    visits = []                                               # create empty list of visits to manual nodes
    for k, l in enumerate(laps):                              # process each prepared lap
        D = np.hypot(l["x"][:, None] - NX[None, :], l["y"][:, None] - NY[None, :])  # calculate distances from path samples to labelled points
        hit = D.min(1) <= HIT_R                               # mark path samples within manual-node visit radius
        near = owner[D.argmin(1)]                             # assign nearest labelled point's node to each path sample
        D = np.stack([D[:, owner == q].min(1) for q in range(len(names))], 1)  # calculate closest labelled-point distance separately for each named node
        near[~hit] = -1                                       # mark samples outside all node hit radii with -1
        n = len(near)                                         # store number of sampled path positions
        i = 0                                                 # begin searching from first path sample
        while i < n:                                          # scan through path samples in order
            if near[i] < 0:                                   # check if current sample is outside all labelled nodes
                i += 1                                        # advance to next path sample
                continue                                      # skip sample that does not visit a node
            j = i                                             # start contiguous visit range at current sample
            while j + 1 < n and near[j + 1] == near[i]:       # extend range while next sample is assigned to same node
                j += 1                                        # advance final index of contiguous visit range
            seg = np.arange(i, j + 1)                         # build indices belonging to this visit range
            best = int(seg[np.argmin(D[seg, near[i]])])       # choose sample that passes closest to assigned node
            visits.append(dict(lap=k, i=best, kind="J", node=nodes[near[i]]["id"],  # start visit record with lap, sample index, kind and node identifier
                               x=l["x"][best], y=l["y"][best], sa=l["sa"][best], arc=l["arc"][best]))  # store selected sample's position and path-distance values
            nodes[near[i]]["members"].append(len(visits) - 1)  # attach new visit index to corresponding node's member list
            i = j + 1                                         # resume searching after current visit range
    return visits, nodes                                      # return visits and manually specified graph nodes


def main():                                                   # function for loading a log, building graph and writing mapping outputs
    ap = argparse.ArgumentParser()                            # create command-line argument parser
    ap.add_argument("csv")                                    # require path to exploration CSV log
    ap.add_argument("--start")                                # accept optional start node identifier for route planning
    ap.add_argument("--goal")                                 # accept optional goal node identifier for route planning
    ap.add_argument("--fix-drift", action="store_true",       # add switch for optional gyro drift correction
                    help="remove gyro drift so each lap turns exactly 360 deg")  # show drift-correction explanation in command-line help
    ap.add_argument("--nodes", help="file with hand-labelled node positions from map.png (name,x,y)")  # accept optional file of manually labelled node coordinates
    ap.add_argument("--heading", choices=["gyro", "wheels"], default="gyro",  # allow gyro or wheel heading source, default to gyro
                    help="where the robot's heading comes from (default gyro)")  # explain heading source option in command-line help
    args = ap.parse_args()                                    # read and validate supplied command-line arguments

    laps = prepare_laps(choose_heading(load(args.csv), args.heading, args.fix_drift))  # load CSV, select heading and prepare corrected and aligned lap paths
    counts = pass_counts(laps)                                # calculate nearby-pass counts along prepared paths
    if args.nodes:                                            # check if manual node-coordinate file was supplied
        visits, nodes = visits_from_file(laps, args.nodes)    # detect visits to manually labelled nodes
    else:                                                     # otherwise use automatic node detection
        visits = find_visits(laps, counts)                    # detect junction and dead-end visits from path geometry
        nodes = cluster(visits)                               # group matching visits into graph nodes
    edges, turns = build_graph(laps, visits)                  # build measured edges and observed node transitions
    # a real junction has 3 exits; a "junction" with only 2 is a corner that looked
    # busy (e.g. next to the start mark) -> drop it and join its two edges
    while not args.nodes:                                     # prune false two-exit junctions only in automatic-node mode
        deg = {}                                              # create empty dictionary for node degrees
        for u, v in edges:                                    # count connections from every current graph edge
            deg[u] = deg.get(u, 0) + 1                        # increase degree of first endpoint
            deg[v] = deg.get(v, 0) + 1                        # increase degree of second endpoint
        fake = {n["id"] for n in nodes if n["kind"] == "J" and deg.get(n["id"], 0) == 2}  # find nodes marked as junctions but having only two connections
        if not fake:                                          # check if no false two-exit junction remains
            break                                             # finish pruning when graph no longer needs this correction
        visits = [v for v in visits if v.get("node") not in fake]  # remove visits assigned to false junction nodes
        nodes = [n for n in nodes if n["id"] not in fake]     # remove false junction nodes from node list
        edges, turns = build_graph(laps, visits)              # rebuild graph so paths connect through removed junction positions
    nbrs, sigma = exit_order(nodes, edges, turns)             # infer node neighbors and left-tape exit ordering
    if args.nodes:                                            # check if node positions came from manual labels
        for n in nodes:                                       # classify each manually labelled node using its graph connections
            n["kind"] = "E" if len(nbrs[n["id"]]) == 1 else "J"  # mark one-neighbor node as dead end, otherwise as junction

    print("NODES")                                            # print title for node summary
    with open("nodes.csv", "w") as f:                         # open nodes.csv for writing node information
        f.write("node,kind,x_cm,y_cm,degree,visits,exits\n")  # write node table column names and coordinate units
        for n in nodes:                                       # write each graph node's information
            ex = " ".join(sorted(nbrs[n["id"]]))              # join sorted neighbor identifiers into readable exit list
            px, py = disp(n["x"], n["y"])                     # convert node coordinates into same frame used by map image
            print("  {:4s} {:9s} at ({:6.1f},{:6.1f})  degree {}  exits: {}".format(  # format node identifier, kind, position, degree and exits for terminal
                n["id"], "junction" if n["kind"] == "J" else "dead end", px, py,  # provide node label, readable kind and drawing coordinates
                len(nbrs[n["id"]]), ex))                      # provide neighbor count and exit labels to finish terminal node output
            f.write("{},{},{:.1f},{:.1f},{},{},{}\n".format(  # format one CSV row for current node
                n["id"], n["kind"], px, py, len(nbrs[n["id"]]), len(n["members"]), ex))  # write identifier, kind, position, degree, visit count and exits
    print("EDGES (cm; each line was measured once per side)")  # print title explaining edge lengths and pass measurements
    with open("edges.csv", "w") as f:                         # open edges.csv for writing graph connections
        f.write("from,to,length_cm,pass_lengths_cm\n")        # write edge table column names and length units
        for (u, v), passes in sorted(edges.items()):          # process undirected edges in sorted node-pair order
            ls = [p["length"] for p in passes]                # collect all measured pass lengths for this edge
            print("  {:4s} - {:4s} {:6.1f}   passes: {}".format(u, v, np.mean(ls), ", ".join("%.1f" % q for q in ls)))  # print node pair, mean edge length and individual pass lengths
            f.write("{},{},{:.1f},{}\n".format(u, v, np.mean(ls), " ".join("%.1f" % q for q in ls)))  # write edge endpoints, mean length and individual lengths to CSV
    for l in laps:                                            # print summary for each prepared lap
        print("lap {}: {:.0f} cm, loop-closure drift {:.1f} cm{}".format(l["lap"], l["arc_len"], l["drift"],  # format lap length and original endpoint closing error
              ", rotated {:.1f} deg onto lap 1".format(l["aligned_deg"]) if "aligned_deg" in l else ""))  # append applied alignment angle when this lap was aligned

    route = None                                              # start with no optional planned route
    if args.start and args.goal:                              # check if both start and goal node identifiers were provided
        path, cost = dijkstra(edges, args.start, args.goal)   # find shortest path and total geometric edge length
        if path is None:                                      # check if goal is unreachable from supplied start
            raise SystemExit("no path between {} and {}".format(args.start, args.goal))  # stop with message identifying unreachable start and goal pair
        route = route_sides(path, edges, sigma)               # convert graph path into tape-side and wheel-distance instructions
        print("ROUTE {}  ({:.1f} cm)".format(" -> ".join(path), cost))  # print ordered route nodes and total length in cm
        with open("route.txt", "w") as f:                     # open route.txt for writing planned route instructions
            f.write("# route {}  total {:.1f} cm\n".format(" -> ".join(path), cost))  # write route node sequence and total geometric length as a file comment
            f.write("ROUTE = [   # (tape on robot's L/R side, wheel distance cm on that side)\n")  # write beginning of route list and explain tape-side/distance fields
            for side, length, u, v in route:                  # write one instruction for each route segment
                f.write('    ("{}", {}),   # {} -> {}\n'.format(side, length, u, v))  # write tape side, driving distance and endpoint comment
            f.write("]\n")                                    # close route list in text output
        print(open("route.txt").read())                       # print saved route instructions in terminal
        print("Start: robot at {} on the line towards {}, tape on its {}.".format(  # format physical starting-position and tape-side instructions
            path[0], path[1], "LEFT" if route[0][0] == "L" else "RIGHT"))  # print first edge and selected starting tape side

    plot_map(laps, nodes, edges, route, "map.png")            # save reconstructed map and optional highlighted route
    plot_signals(laps, counts, visits, "explore_signals.png")  # save reflection, heading and node-visit signal plots
    print("saved map.png, explore_signals.png, nodes.csv, edges.csv" + (", route.txt" if route else ""))  # print output filenames and include route file only when one was planned


if __name__ == "__main__":                                    # run command-line program only when this file is launched directly
    main()                                                    # start loading, mapping and optional route planning
