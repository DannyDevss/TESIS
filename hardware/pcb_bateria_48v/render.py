# -*- coding: utf-8 -*-
"""Vista previa SVG de la placa (no es un entregable, es para revisarla)."""
import design, place as P, routes as R, fp_lib as FL
S, M = 9.0, 14           # escala px/mm, margen
W, H = P.BOARD_W*S+2*M, P.BOARD_H*S+2*M
def x(v): return v*S+M
def y(v): return v*S+M
o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
     f'viewBox="0 0 {W:.0f} {H:.0f}"><rect width="100%" height="100%" fill="#10151a"/>']
pts = " ".join(f"{x(a):.1f},{y(b):.1f}" for a, b in P.OUTLINE)
o.append(f'<polygon points="{pts}" fill="#0d3b2e" stroke="#7dd3a0" stroke-width="2"/>')
for net, lay, wp, w in R.TRACKS:
    col = "#ff9b3d" if lay == "F.Cu" else "#4fa3ff"
    hv = net in P.HV_NETS
    d = " ".join(("M" if i == 0 else "L")+f"{x(a):.1f} {y(b):.1f}" for i,(a,b) in enumerate(wp))
    o.append(f'<path d="{d}" fill="none" stroke="{"#ff4d4d" if hv else col}" '
             f'stroke-width="{max(w*S,1.4):.1f}" stroke-linecap="round" stroke-linejoin="round" '
             f'opacity="{0.95 if lay=="F.Cu" else 0.7}"/>')
for net, vx, vy in R.VIAS:
    o.append(f'<circle cx="{x(vx):.1f}" cy="{y(vy):.1f}" r="{P.VIA_D/2*S:.1f}" fill="#ffd966"/>')
for vx, vy in R.GND_VIAS:
    o.append(f'<circle cx="{x(vx):.1f}" cy="{y(vy):.1f}" r="{P.VIA_D/2*S:.1f}" fill="#7a8b99"/>')
for ref,(fp,px,py,rot,val) in P.PLACE.items():
    for pad in fp.pads:
        dx,dy = FL.rot(pad.x,pad.y,rot)
        w,h = (pad.h,pad.w) if rot%180==90 else (pad.w,pad.h)
        net = next((n for n,cs in design.NETS.items() if (ref,pad.name) in cs), None)
        col = "#ff4d4d" if net in P.HV_NETS else ("#c9d6e0" if pad.ptype=="smd" else "#ffe08a")
        o.append(f'<rect x="{x(px+dx-w/2):.1f}" y="{y(py+dy-h/2):.1f}" '
                 f'width="{w*S:.1f}" height="{h*S:.1f}" fill="{col}" rx="{min(w,h)*S*0.2:.1f}"/>')
    cy = fp.courtyard
    if cy:
        c=[FL.rot(a,b,rot) for a,b in ((cy[0],cy[1]),(cy[2],cy[1]),(cy[2],cy[3]),(cy[0],cy[3]))]
        pp=" ".join(f"{x(px+a):.1f},{y(py+b):.1f}" for a,b in c)
        o.append(f'<polygon points="{pp}" fill="none" stroke="#5d7285" stroke-width="0.7"/>')
    o.append(f'<text x="{x(px):.1f}" y="{y(py):.1f}" fill="#eaf2f8" font-size="9" '
             f'font-family="monospace" text-anchor="middle" dominant-baseline="middle" '
             f'style="paint-order:stroke;stroke:#10151a;stroke-width:3">{ref}</text>')
for mx,my in P.MOUNT:
    o.append(f'<circle cx="{x(mx):.1f}" cy="{y(my):.1f}" r="{1.6*S:.1f}" fill="#10151a" stroke="#7dd3a0"/>')
a=FL.ANT_KEEPOUT_LOCAL
o.append(f'<rect x="{x(P.U1_AT[0]+a[0]):.1f}" y="{y(P.U1_AT[1]+a[1]):.1f}" '
         f'width="{(a[2]-a[0])*S:.1f}" height="{(a[3]-a[1])*S:.1f}" fill="none" '
         f'stroke="#ff4d4d" stroke-width="1" stroke-dasharray="5 4"/>')
o.append('</svg>')
open("preview.svg","w").write("\n".join(o))
print("preview.svg escrito")
