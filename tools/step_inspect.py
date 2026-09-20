#!/usr/bin/env python3
"""Locate parts in a STEP assembly, in robot coordinates.

A companion to tools/onshape_measurements.py, for when a STEP export is what you
have. It reads the assembly graph, resolves every instance to a world transform,
and reports robust bounding boxes for named parts.

WHAT IT IS GOOD FOR

Cross-checking. A STEP file carries no datum saying "this is the robot origin"
and no annotation saying "the lens is here, pointing that way", so it cannot
produce a camera transform the way a named mate connector can. What it does very
well is confirm the things that are easy to get wrong and expensive to discover
late: whether the assembly origin coincides with the FRC robot origin, whether
the frame is where you think, and whether TunerConstants' module positions match
the CAD.

On the Rebuilt robot it established, by three independent routes, that the
assembly origin IS the robot origin: the frame bounding box centres on it in XY,
the four swerve modules are symmetric about it, and the module hardware bottoms
out at exactly Z = 0. It also confirmed the modules sit at about +-0.26 m, which
agrees with TunerConstants' +-10 inches.

TWO TRAPS, BOTH HIT WHILE WRITING THIS

1. THE TRANSFORM DIRECTION IS NOT FIXED BY THE STANDARD. A
   REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION names two representations, and
   which one is the parent varies by exporter. In this Onshape export, rep_1 is
   the CHILD -- the opposite of the common assumption -- so composing
   M(item_1) . M(item_2)^-1 silently produces a plausible but wrong assembly.
   The giveaway was two flywheels on one shooter coming out 90 degrees apart with
   their Y and Z swapped. This script decides the direction per relationship by
   checking which representation belongs to the child product, rather than
   assuming.

2. B-REP GEOMETRY CONTAINS POINTS THAT ARE NOT ON THE PART. Unbounded surface
   and curve definitions carry control points far outside the solid; one bracket
   measured 501 metres across on a first pass. Bounds here are therefore
   percentile-based, not min/max. If a part looks absurdly large, that is why.

Usage:
    python3 tools/step_inspect.py --file Robot.step --find limelight quest turret
    python3 tools/step_inspect.py --file Robot.step --tree          # assembly outline
"""

import argparse
import math
import pickle
import re
import sys
from collections import defaultdict

ENT = re.compile(r"^#(\d+)\s*=\s*(.*)$", re.S)
REFS = re.compile(r"#(\d+)")


def records(path):
    """Yields (id, body) for every STEP entity, handling multi-line records."""
    buf = []
    in_data = False
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            s = line.strip()
            if not in_data:
                if s == "DATA;":
                    in_data = True
                continue
            if s == "ENDSEC;":
                break
            buf.append(s)
            joined = "".join(buf)
            # A record ends at a ';' that is not inside a quoted string.
            if joined.endswith(";") and joined.count("'") % 2 == 0:
                m = ENT.match(joined[:-1])
                if m:
                    yield int(m.group(1)), m.group(2)
                buf = []


def typename(body):
    if body.startswith("("):
        return "COMPLEX"
    i = body.find("(")
    return body[:i].strip() if i > 0 else body.strip()


def args_of(body, kind):
    """Extracts the argument text of `kind` from a simple or complex entity body."""
    idx = body.find(kind + "(")
    if idx < 0:
        return None
    start = idx + len(kind) + 1
    depth, i = 1, start
    while i < len(body) and depth:
        if body[i] == "(":
            depth += 1
        elif body[i] == ")":
            depth -= 1
        i += 1
    return body[start : i - 1]


def split_args(text):
    """Splits a STEP argument list on top-level commas."""
    out, depth, cur, instr = [], 0, [], False
    i = 0
    while i < len(text):
        c = text[i]
        if instr:
            if c == "'":
                if i + 1 < len(text) and text[i + 1] == "'":
                    cur.append("''")
                    i += 2
                    continue
                instr = False
            cur.append(c)
        elif c == "'":
            instr = True
            cur.append(c)
        elif c == "(":
            depth += 1
            cur.append(c)
        elif c == ")":
            depth -= 1
            cur.append(c)
        elif c == "," and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(c)
        i += 1
    out.append("".join(cur).strip())
    return out




def build(path, cache_path=None):
    """Parses the file (or a cached parse) and returns the resolved instance list."""
    ents = None
    if cache_path:
        try:
            with open(cache_path, "rb") as fh:
                ents = pickle.load(fh)
                print(f"  using cached parse: {cache_path}", file=sys.stderr)
        except Exception:
            ents = None
    if ents is None:
        print("  parsing...", file=sys.stderr)
        ents = {eid: b for eid, b in records(path)}
        if cache_path:
            with open(cache_path, "wb") as fh:
                pickle.dump(ents, fh, protocol=4)
    print(f"  {len(ents)} entities", file=sys.stderr)

    body = ents.get
    refs_re = re.compile(r"#(\d+)")

    def dot(a, b): return sum(p * q for p, q in zip(a, b))
    def sub(a, b): return [p - q for p, q in zip(a, b)]
    def mul(a, s): return [p * s for p in a]
    def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

    def unit(a):
        n = math.sqrt(dot(a, a))
        return [p / n for p in a] if n else a

    def matmul(A, B):
        C = [0.0] * 16
        for r in range(4):
            for c in range(4):
                C[r*4+c] = sum(A[r*4+k] * B[k*4+c] for k in range(4))
        return C

    def inv(m):
        rt = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]]
        t = [m[3], m[7], m[11]]
        nt = [-(rt[0]*t[0]+rt[1]*t[1]+rt[2]*t[2]),
              -(rt[3]*t[0]+rt[4]*t[1]+rt[5]*t[2]),
              -(rt[6]*t[0]+rt[7]*t[1]+rt[8]*t[2])]
        return [rt[0],rt[1],rt[2],nt[0], rt[3],rt[4],rt[5],nt[1], rt[6],rt[7],rt[8],nt[2], 0,0,0,1]

    def point(eid):
        a = args_of(body(eid, ""), "CARTESIAN_POINT")
        if a is None:
            return None
        v = split_args(split_args(a)[1].strip("()"))
        return [float(x) for x in v] if len(v) == 3 else None

    def direction(eid):
        a = args_of(body(eid, ""), "DIRECTION")
        if a is None:
            return None
        return [float(x) for x in split_args(split_args(a)[1].strip("()"))]

    def placement(eid):
        a = args_of(body(eid, ""), "AXIS2_PLACEMENT_3D")
        if a is None:
            return None
        p = split_args(a)
        loc = point(int(p[1][1:]))
        axis = direction(int(p[2][1:])) if p[2] != "$" else [0, 0, 1]
        refd = direction(int(p[3][1:])) if p[3] != "$" else [1, 0, 0]
        z = unit(axis)
        x = unit(sub(refd, mul(z, dot(refd, z))))
        y = cross(z, x)
        return [x[0],y[0],z[0],loc[0], x[1],y[1],z[1],loc[1], x[2],y[2],z[2],loc[2], 0,0,0,1]

    pd_name, pd_rep, nauo, pend = {}, {}, {}, {}
    for eid, b in ents.items():
        t = typename(b)
        if t == "PRODUCT_DEFINITION":
            a = split_args(args_of(b, "PRODUCT_DEFINITION"))
            pdf = int(a[2][1:])
            fa = (args_of(body(pdf, ""), "PRODUCT_DEFINITION_FORMATION")
                  or args_of(body(pdf, ""), "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE"))
            pd_name[eid] = split_args(args_of(body(int(split_args(fa)[2][1:]), ""), "PRODUCT"))[0].strip("'")
        elif t == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            a = split_args(args_of(b, "NEXT_ASSEMBLY_USAGE_OCCURRENCE"))
            nauo[eid] = (int(a[3][1:]), int(a[4][1:]))

    for eid, b in ents.items():
        t = typename(b)
        if t == "SHAPE_DEFINITION_REPRESENTATION":
            a = split_args(args_of(b, "SHAPE_DEFINITION_REPRESENTATION"))
            pdsa = args_of(body(int(a[0][1:]), ""), "PRODUCT_DEFINITION_SHAPE")
            if pdsa:
                tgt = split_args(pdsa)[2]
                if tgt.startswith("#"):
                    pd_rep[int(tgt[1:])] = int(a[1][1:])
        elif t == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            a = split_args(args_of(b, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"))
            pdsa = args_of(body(int(a[1][1:]), ""), "PRODUCT_DEFINITION_SHAPE")
            if not pdsa:
                continue
            tgt = split_args(pdsa)[2]
            if not tgt.startswith("#") or int(tgt[1:]) not in nauo:
                continue
            idt_ref = args_of(body(int(a[0][1:]), ""), "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
            if idt_ref is None:
                continue
            ia = split_args(args_of(body(int(split_args(idt_ref)[0][1:]), ""), "ITEM_DEFINED_TRANSFORMATION"))
            rr = split_args(args_of(body(int(a[0][1:]), ""), "REPRESENTATION_RELATIONSHIP"))
            pend[int(tgt[1:])] = (placement(int(ia[2][1:])), placement(int(ia[3][1:])),
                                  int(rr[2][1:]), int(rr[3][1:]))

    # Trap 1: decide the composition direction per relationship, never by assumption.
    child_rep = {nid: pd_rep[c] for nid, (p, c) in nauo.items() if c in pd_rep}
    xf, flipped = {}, 0
    for nid, (m1, m2, r1, r2) in pend.items():
        if not (m1 and m2):
            continue
        if child_rep.get(nid) == r1:
            xf[nid] = matmul(m2, inv(m1)); flipped += 1
        else:
            xf[nid] = matmul(m1, inv(m2))
    print(f"  {len(xf)} assembly transforms ({flipped} with rep_1 = child)", file=sys.stderr)

    children, is_child = defaultdict(list), set()
    for nid, (p, c) in nauo.items():
        children[p].append((nid, c)); is_child.add(c)
    roots = [pd for pd in pd_name if pd not in is_child]

    instances = []
    ident = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

    def walk(pd, w, path, d=0):
        if d > 14:
            return
        for nid, ch in children.get(pd, []):
            m = xf.get(nid)
            ww = matmul(w, m) if m else w
            p = path + "/" + pd_name.get(ch, "?")
            instances.append((p, ch, ww)); walk(ch, ww, p, d + 1)

    for r in roots:
        instances.append(("/" + pd_name[r], r, ident)); walk(r, ident, "/" + pd_name[r])

    GEOM_REPS = ("ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
                 "TESSELLATED_SHAPE_REPRESENTATION",
                 "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION", "SHAPE_REPRESENTATION")
    geom_of = defaultdict(list)
    for eid, b in ents.items():
        if typename(b) == "SHAPE_REPRESENTATION_RELATIONSHIP":
            a = split_args(args_of(b, "SHAPE_REPRESENTATION_RELATIONSHIP"))
            geom_of[int(a[2][1:])].append(int(a[3][1:]))

    cache = {}

    def points_for(pd):
        if pd in cache:
            return cache[pd]
        rep = pd_rep.get(pd)
        out = []
        if rep is not None:
            seeds = []
            for g in [rep] + list(geom_of.get(rep, [])):
                gb = body(g, "")
                if typename(gb) not in GEOM_REPS:
                    continue
                ga = args_of(gb, typename(gb))
                if ga:
                    seeds += [int(x) for x in refs_re.findall(split_args(ga)[1])]
            seen, stack = set(), list(seeds)
            while stack:
                eid = stack.pop()
                if eid in seen:
                    continue
                seen.add(eid)
                b = body(eid)
                if b is None:
                    continue
                if b.startswith("CARTESIAN_POINT"):
                    p = point(eid)
                    if p:
                        out.append(p)
                    continue
                if typename(b) in GEOM_REPS:
                    continue
                for r in refs_re.findall(b):
                    r = int(r)
                    if r not in seen:
                        stack.append(r)
        cache[pd] = out
        return out

    def xform(m, p):
        return (m[0]*p[0]+m[1]*p[1]+m[2]*p[2]+m[3],
                m[4]*p[0]+m[5]*p[1]+m[6]*p[2]+m[7],
                m[8]*p[0]+m[9]*p[1]+m[10]*p[2]+m[11])

    return instances, points_for, xform


def robust_bounds(pts, lo=0.005, hi=0.995):
    """Percentile bounds. Trap 2: min/max would include off-part control points."""
    out = []
    for ax in range(3):
        v = sorted(p[ax] for p in pts)
        n = len(v)
        out.append((v[max(0, int(lo * n))], v[min(n - 1, int(hi * n))]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", required=True, help="path to the STEP file")
    ap.add_argument("--find", nargs="*", default=[], help="case-insensitive substrings of part names")
    ap.add_argument("--tree", action="store_true", help="print the top levels of the assembly")
    ap.add_argument("--cache", help="pickle path to speed up repeated runs")
    ap.add_argument("--min-points", type=int, default=20,
                    help="ignore parts with fewer points than this; a handful usually means a "
                         "datum rather than a solid, and its 'centre' is meaningless")
    args = ap.parse_args()

    instances, points_for, xform = build(args.file, args.cache)
    print(f"  {len(instances)} placed instances", file=sys.stderr)

    if args.tree:
        seen = set()
        for path, pd, w in instances:
            if path.count("/") <= 2 and path not in seen:
                seen.add(path)
                print(f"  {path}")
        return 0

    if not args.find:
        print("nothing to find; pass --find or --tree", file=sys.stderr)
        return 2

    shown = set()
    for path, pd, w in instances:
        leaf = path.rsplit("/", 1)[-1]
        if not any(k.lower() in leaf.lower() for k in args.find):
            continue
        pts = points_for(pd)
        if len(pts) < args.min_points:
            continue
        W = [xform(w, p) for p in pts]
        b = robust_bounds(W)
        c = [(lo + hi) / 2 for lo, hi in b]
        key = (leaf, round(c[0], 4), round(c[1], 4), round(c[2], 4))
        if key in shown:
            continue
        shown.add(key)
        print(f"\n  {leaf}   ({len(pts)} points)")
        print(f"    {path}")
        print(f"    centre ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f})   "
              f"size ({b[0][1]-b[0][0]:.4f} x {b[1][1]-b[1][0]:.4f} x {b[2][1]-b[2][0]:.4f})")
        print(f"    bounds X {b[0][0]:+.4f}..{b[0][1]:+.4f}  Y {b[1][0]:+.4f}..{b[1][1]:+.4f}  "
              f"Z {b[2][0]:+.4f}..{b[2][1]:+.4f}")
        print(f"    part axes in assembly frame: "
              f"X=({w[0]:+.3f},{w[4]:+.3f},{w[8]:+.3f}) "
              f"Y=({w[1]:+.3f},{w[5]:+.3f},{w[9]:+.3f}) "
              f"Z=({w[2]:+.3f},{w[6]:+.3f},{w[10]:+.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
