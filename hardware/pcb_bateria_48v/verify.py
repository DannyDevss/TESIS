# -*- coding: utf-8 -*-
"""Relee el .kicad_pcb emitido y comprueba geometria y nets contra el modelo."""
import re, math, design, place as P, fp_lib as FL

src = open("bateria48v.kicad_pcb").read()

def toks(s):
    return [float(v) for v in s.split()]

# --- footprints y pads tal como quedaron en el archivo ---------------------
fps = re.findall(r'\(footprint "([^"]+)" \(layer "F\.Cu"\)\s*\n\s*\(tstamp \S+\) '
                 r'\(at ([-\d. ]+)\)(.*?)\n  \)', src, re.S)
parsed = {}
for name, at, body in fps:
    a = toks(at); fx, fy = a[0], a[1]; frot = a[2] if len(a) > 2 else 0.0
    ref = re.search(r'\(property "Reference" "([^"]*)"', body)
    ref = ref.group(1) if ref else None
    pads = []
    for pm in re.finditer(r'\(pad "([^"]*)" (\S+) (\S+) \(at ([-\d. ]+)\) '
                          r'\(size ([-\d. ]+)\).*?\(tstamp [0-9a-f-]+\)\)', body, re.S):
        pn, ptype, shape, pat, size = pm.groups()
        txt = pm.group(0)
        nm = re.search(r'\(net (\d+) "([^"]*)"\)', txt)
        p = toks(pat); dx, dy = FL.rot(p[0], p[1], p[2] if len(p) > 2 else 0.0)
        pads.append(dict(name=pn, type=ptype, gx=fx+dx, gy=fy+dy,
                         size=toks(size), net=(nm.group(2) if nm else None),
                         drill=None))
    parsed[ref] = dict(name=name, at=(fx, fy), rot=frot, pads=pads)

errs = []
# 1) posicion global de cada pad == modelo
for ref, (fp, x, y, rot, val) in P.PLACE.items():
    if ref not in parsed: errs.append(f"falta footprint {ref}"); continue
    got = parsed[ref]["pads"]
    if len(got) != len(fp.pads):
        errs.append(f"{ref}: {len(got)} pads en archivo vs {len(fp.pads)} en modelo")
    for gp, mp in zip(got, fp.pads):
        dx, dy = FL.rot(mp.x, mp.y, rot)
        ex, ey = x+dx, y+dy
        if abs(gp["gx"]-ex) > 1e-3 or abs(gp["gy"]-ey) > 1e-3:
            errs.append(f"{ref}.{mp.name}: archivo ({gp['gx']:.3f},{gp['gy']:.3f}) "
                        f"vs modelo ({ex:.3f},{ey:.3f})")
        want = design.NETS and next((n for n, cs in design.NETS.items()
                                     if (ref, mp.name) in cs), None)
        if (gp["net"] or None) != want:
            errs.append(f"{ref}.{mp.name}: net '{gp['net']}' en archivo vs '{want}' esperado")

# 2) todo pad con net declarado existe en el netlist
declared = {(r, p) for n, cs in design.NETS.items() for r, p in cs}
for ref, d in parsed.items():
    for gp in d["pads"]:
        if gp["net"] and (ref, gp["name"]) not in declared:
            errs.append(f"{ref}.{gp['name']} tiene net '{gp['net']}' no declarado")

# 3) los segmentos del archivo cubren exactamente los del modelo
segs = re.findall(r'\(segment \(start ([-\d. ]+)\) \(end ([-\d. ]+)\) \(width ([\d.]+)\)'
                  r' \(layer "([^"]+)"\) \(net (\d+)\)', src)
netnames = dict(re.findall(r'\(net (\d+) "([^"]*)"\)', src))
got = {(netnames[n], l, tuple(toks(a)), tuple(toks(b))) for a, b, w, l, n in segs}
want = set()
for net, layer, wp, w in __import__("routes").TRACKS:
    for i in range(len(wp)-1):
        if wp[i] != wp[i+1]: want.add((net, layer, wp[i], wp[i+1]))
if got != want:
    errs.append(f"segmentos: {len(got - want)} sobran, {len(want - got)} faltan")

# 4) todo pad cae dentro del contorno de la placa
def inside(px, py):
    n, c = len(P.OUTLINE), False
    for i in range(n):
        x1, y1 = P.OUTLINE[i]; x2, y2 = P.OUTLINE[(i+1) % n]
        if (y1 > py) != (y2 > py) and px < (x2-x1)*(py-y1)/(y2-y1)+x1: c = not c
    return c
for ref, d in parsed.items():
    for gp in d["pads"]:
        if not inside(gp["gx"], gp["gy"]):
            errs.append(f"{ref}.{gp['name']} fuera del contorno en ({gp['gx']:.2f},{gp['gy']:.2f})")

print(f"footprints leidos del archivo: {len(parsed)}")
print(f"pads leidos: {sum(len(d['pads']) for d in parsed.values())}")
print(f"segmentos leidos: {len(segs)}  vias: {src.count('(via ')}")
print("\n".join(errs) if errs else "\nVERIFICACION OK: geometria, nets y contorno coinciden con el modelo")
