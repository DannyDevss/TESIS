# -*- coding: utf-8 -*-
"""Chequeo de colisiones de courtyard y de que todo entre en la placa."""
import fp_lib as F, place as P

def courtyard_box(ref):
    fp, x, y, r, _ = P.PLACE[ref]
    if not fp.courtyard: return None
    x1, y1, x2, y2 = fp.courtyard
    pts = [F.rot(a, b, r) for a, b in ((x1,y1),(x2,y1),(x2,y2),(x1,y2))]
    xs = [x+a for a, b in pts]; ys = [y+b for a, b in pts]
    return (min(xs), min(ys), max(xs), max(ys))

def overlap(a, b):
    ix = min(a[2],b[2]) - max(a[0],b[0])
    iy = min(a[3],b[3]) - max(a[1],b[1])
    return (ix, iy) if ix > 0 and iy > 0 else None

def run():
    boxes = {r: courtyard_box(r) for r in P.PLACE}
    refs = sorted(boxes)
    errs = []
    for i, a in enumerate(refs):
        for b in refs[i+1:]:
            ov = overlap(boxes[a], boxes[b])
            if ov:
                errs.append(f"  SOLAPE  {a} <-> {b}   ({ov[0]:.2f} x {ov[1]:.2f} mm)")
    for r, bx in boxes.items():
        if bx[0] < 0.3 or bx[1] < 0.3 or bx[2] > P.BOARD_W-0.3 or bx[3] > P.BOARD_H-0.3:
            errs.append(f"  FUERA DE PLACA  {r}  bbox=({bx[0]:.1f},{bx[1]:.1f})-({bx[2]:.1f},{bx[3]:.1f})")
    # agujeros de montaje vs componentes
    for (mx, my) in P.MOUNT:
        mb = (mx-3.2, my-3.2, mx+3.2, my+3.2)
        for r, bx in boxes.items():
            if overlap(mb, bx):
                errs.append(f"  SOLAPE  agujero M3 ({mx},{my}) <-> {r}")
    # zona de antena: sin componentes
    ax1, ay1, ax2, ay2 = (P.U1_AT[0]+F.ANT_KEEPOUT_LOCAL[0], P.U1_AT[1]+F.ANT_KEEPOUT_LOCAL[1],
                          P.U1_AT[0]+F.ANT_KEEPOUT_LOCAL[2], P.U1_AT[1]+F.ANT_KEEPOUT_LOCAL[3])
    for r, bx in boxes.items():
        if r == "U1": continue
        if overlap((ax1,ay1,ax2,ay2), bx):
            errs.append(f"  {r} invade la zona de antena ({ax1:.1f},{ay1:.1f})-({ax2:.1f},{ay2:.1f})")
    return errs

if __name__ == "__main__":
    e = run()
    print("\n".join(e) if e else "emplazamiento OK: sin solapes, todo dentro de la placa")
    print(f"\n{len(P.PLACE)} componentes + {len(P.MOUNT)} agujeros en {P.BOARD_W}x{P.BOARD_H} mm")
