# -*- coding: utf-8 -*-
"""DRC geometrico + verificacion de conectividad, sin depender de KiCad."""
import math
import design, place as P, routes as R, fp_lib as FL

CL, CL_HV, HV = P.CLEARANCE, P.CLEARANCE_HV, P.HV_NETS
VR, VCL = P.VIA_D/2, P.CLEARANCE

# ---------------------------------------------------------------- geometria
def seg_seg(a, b, c, d):
    def sub(p,q): return (p[0]-q[0], p[1]-q[1])
    def cross(p,q): return p[0]*q[1]-p[1]*q[0]
    r, s = sub(b,a), sub(d,c)
    den = cross(r,s)
    if abs(den) > 1e-12:
        t = cross(sub(c,a), s)/den
        u = cross(sub(c,a), r)/den
        if -1e-9 <= t <= 1+1e-9 and -1e-9 <= u <= 1+1e-9:
            return 0.0
    return min(pt_seg(a,c,d), pt_seg(b,c,d), pt_seg(c,a,b), pt_seg(d,a,b))

def pt_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx-ax, by-ay
    L = dx*dx + dy*dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy)/L))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))

def pt_rect(p, r):
    dx = max(r[0]-p[0], 0, p[0]-r[2])
    dy = max(r[1]-p[1], 0, p[1]-r[3])
    return math.hypot(dx, dy)

def seg_rect(a, b, r):
    """Distancia de un segmento a un rectangulo alineado a los ejes."""
    corners = [(r[0],r[1]),(r[2],r[1]),(r[2],r[3]),(r[0],r[3])]
    edges = [(corners[i], corners[(i+1) % 4]) for i in range(4)]
    if any(seg_seg(a,b,e[0],e[1]) == 0.0 for e in edges): return 0.0
    if r[0] <= a[0] <= r[2] and r[1] <= a[1] <= r[3]: return 0.0
    return min(pt_rect(a,r), pt_rect(b,r),
               *[pt_seg(c,a,b) for c in corners])

def rect_rect(p, q):
    dx = max(p[0]-q[2], q[0]-p[2], 0.0)
    dy = max(p[1]-q[3], q[1]-p[3], 0.0)
    return math.hypot(dx, dy)

def need(n1, n2):
    return CL_HV if (n1 in HV or n2 in HV) else CL

# ---------------------------------------------------------------- objetos
PADNET = {(r,p): n for n, cs in design.NETS.items() for r, p in cs}

def all_pads():
    """(ref, padname, net, rect, layers)"""
    out = []
    for ref, (fp, x, y, rr, _) in P.PLACE.items():
        for pad in fp.pads:
            dx, dy = FL.rot(pad.x, pad.y, rr)
            w, h = (pad.h, pad.w) if rr % 180 == 90 else (pad.w, pad.h)
            rect = (x+dx-w/2, y+dy-h/2, x+dx+w/2, y+dy+h/2)
            lay = {"F.Cu"} if pad.ptype == "smd" else {"F.Cu","B.Cu"}
            net = PADNET.get((ref, pad.name))
            out.append((ref, pad.name, net, rect, lay, pad.ptype))
    return out

PADS = all_pads()

def segments():
    out = []
    for net, layer, pts, w in R.TRACKS:
        for i in range(len(pts)-1):
            if pts[i] != pts[i+1]:
                out.append((net, layer, pts[i], pts[i+1], w))
    for ref, pad, tgt in R.GND_STUBS:
        out.append(("GND", "F.Cu", P.pad_xy(ref, pad), tgt, P.TRACK_PWR))
    return out

SEGS = segments()
VIAS = [(n,(x,y)) for n,x,y in R.VIAS] + [("GND",(x,y)) for x,y in R.GND_VIAS]

# ---------------------------------------------------------------- chequeos
def check_clearance():
    errs = []
    for net, layer, a, b, w in SEGS:
        half = w/2
        for ref, pn, pnet, rect, lay, _ in PADS:
            if pnet == net or layer not in lay: continue
            d = seg_rect(a, b, rect) - half
            req = need(net, pnet) if pnet else CL
            if d < req - 1e-6:
                errs.append(f"pista {net} {layer} {a}-{b}  vs pad {ref}.{pn}[{pnet}]  {d:.3f} < {req}")
        for vnet, vp in VIAS:
            if vnet == net: continue
            d = pt_seg(vp, a, b) - half - VR
            if d < need(net, vnet) - 1e-6:
                errs.append(f"pista {net} {layer} {a}-{b}  vs via {vnet}@{vp}  {d:.3f}")
    for i, (n1, l1, a1, b1, w1) in enumerate(SEGS):
        for n2, l2, a2, b2, w2 in SEGS[i+1:]:
            if n1 == n2 or l1 != l2: continue
            d = seg_seg(a1,b1,a2,b2) - w1/2 - w2/2
            if d < need(n1,n2) - 1e-6:
                errs.append(f"pista {n1} {l1} {a1}-{b1}  vs pista {n2} {a2}-{b2}  {d:.3f}")
    for i, (n1, p1) in enumerate(VIAS):
        for n2, p2 in VIAS[i+1:]:
            if n1 == n2: continue
            d = math.hypot(p1[0]-p2[0], p1[1]-p2[1]) - 2*VR
            if d < need(n1,n2) - 1e-6:
                errs.append(f"via {n1}@{p1} vs via {n2}@{p2}  {d:.3f}")
        for ref, pn, pnet, rect, lay, _ in PADS:
            if pnet == n1: continue
            d = pt_rect(p1, rect) - VR
            if d < need(n1, pnet or "") - 1e-6:
                errs.append(f"via {n1}@{p1} vs pad {ref}.{pn}[{pnet}]  {d:.3f}")
    for i, (r1,p1,n1,rc1,l1,t1) in enumerate(PADS):
        for r2,p2,n2,rc2,l2,t2 in PADS[i+1:]:
            if n1 == n2 or r1 == r2 or not (l1 & l2): continue
            d = rect_rect(rc1, rc2)
            if d < need(n1 or "", n2 or "") - 1e-6:
                errs.append(f"pad {r1}.{p1}[{n1}] vs pad {r2}.{p2}[{n2}]  {d:.3f}")
    return errs

def check_connectivity():
    """Union-find sobre pads, segmentos y vias; el plano de GND une todo B.Cu."""
    items = []
    for ref, pn, net, rect, lay, ty in PADS:
        if net: items.append(("pad", net, lay, rect, f"{ref}.{pn}"))
    for net, layer, a, b, w in SEGS:
        items.append(("seg", net, {layer}, (a,b,w), f"{net} {a}-{b}"))
    for net, vp in VIAS:
        items.append(("via", net, {"F.Cu","B.Cu"}, vp, f"via {net}@{vp}"))
    par = list(range(len(items)))
    def find(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    def uni(i,j):
        a,b = find(i), find(j)
        if a != b: par[a] = b
    def touch(A, B):
        ta, _, la, ga, _ = A; tb, _, lb, gb, _ = B
        if not (la & lb): return False
        if ta == "pad" and tb == "pad": return rect_rect(ga, gb) <= 1e-6
        if ta == "pad" and tb == "seg": return seg_rect(gb[0], gb[1], ga) - gb[2]/2 <= 1e-6
        if ta == "seg" and tb == "pad": return seg_rect(ga[0], ga[1], gb) - ga[2]/2 <= 1e-6
        if ta == "seg" and tb == "seg":
            return seg_seg(ga[0],ga[1],gb[0],gb[1]) - ga[2]/2 - gb[2]/2 <= 1e-6
        if ta == "via" and tb == "pad": return pt_rect(ga, gb) <= VR + 1e-6
        if ta == "pad" and tb == "via": return pt_rect(gb, ga) <= VR + 1e-6
        if ta == "via" and tb == "seg": return pt_seg(ga, gb[0], gb[1]) <= VR + gb[2]/2 + 1e-6
        if ta == "seg" and tb == "via": return pt_seg(gb, ga[0], ga[1]) <= VR + ga[2]/2 + 1e-6
        return math.hypot(ga[0]-gb[0], ga[1]-gb[1]) <= 2*VR + 1e-6
    for i in range(len(items)):
        for j in range(i+1, len(items)):
            if items[i][1] == items[j][1] and touch(items[i], items[j]):
                uni(i,j)
    # el plano de GND en B.Cu une todos los elementos de GND que lo alcanzan
    gnd = [i for i,it in enumerate(items) if it[1] == "GND"]
    for k in gnd[1:]: uni(gnd[0], k)

    errs = []
    for net in design.NETS:
        idx = [i for i,it in enumerate(items) if it[1] == net]
        roots = {find(i) for i in idx}
        if len(roots) > 1:
            groups = {}
            for i in idx: groups.setdefault(find(i), []).append(items[i][4])
            errs.append(f"NET '{net}' partido en {len(roots)} islas:")
            for g in groups.values():
                errs.append("      " + " | ".join(sorted(set(g))[:6]))
    return errs

if __name__ == "__main__":
    ce = check_clearance()
    print(f"--- separaciones: {len(ce)} problemas")
    for e in ce[:40]: print("  " + e)
    co = check_connectivity()
    print(f"--- conectividad: {'OK' if not co else 'FALLOS'}")
    for e in co: print("  " + e)
