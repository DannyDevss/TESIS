# -*- coding: utf-8 -*-
"""Emite el .kicad_pcb (formato KiCad 7, lo abren 7/8/9) con footprints embebidos."""
import design, place as P, routes as R, fp_lib as FL
from fp_lib import uid

NETS = ["GND"] + sorted(n for n in design.NETS if n != "GND")
NETNO = {n: i+1 for i, n in enumerate(NETS)}
PADNET = {(r, p): n for n, cs in design.NETS.items() for r, p in cs}

LAYERS = [(0,"F.Cu","signal",None),(31,"B.Cu","signal",None),
    (32,"B.Adhes","user","B.Adhesive"),(33,"F.Adhes","user","F.Adhesive"),
    (34,"B.Paste","user",None),(35,"F.Paste","user",None),
    (36,"B.SilkS","user","B.Silkscreen"),(37,"F.SilkS","user","F.Silkscreen"),
    (38,"B.Mask","user",None),(39,"F.Mask","user",None),
    (40,"Dwgs.User","user","User.Drawings"),(41,"Cmts.User","user","User.Comments"),
    (42,"Eco1.User","user","User.Eco1"),(43,"Eco2.User","user","User.Eco2"),
    (44,"Edge.Cuts","user",None),(45,"Margin","user",None),
    (46,"B.CrtYd","user","B.Courtyard"),(47,"F.CrtYd","user","F.Courtyard"),
    (48,"B.Fab","user",None),(49,"F.Fab","user",None)]

def fnum(v): return f"{v:.4f}".rstrip("0").rstrip(".") or "0"
def XY(x, y): return f"{fnum(x)} {fnum(y)}"

def emit_footprint(ref):
    fp, x, y, rot, val = P.PLACE[ref]
    o, at = [], (f"{XY(x,y)} {fnum(rot)}" if rot else XY(x, y))
    o.append(f'  (footprint "{fp.name}" (layer "F.Cu")')
    o.append(f'    (tstamp {uid("fp"+ref)}) (at {at}) (attr {fp.attr})')
    o.append(f'    (property "Reference" "{ref}") (property "Value" "{val}")')
    ty = (fp.courtyard[1] - 0.6) if fp.courtyard else -1.6
    o.append(f'    (fp_text reference "{ref}" (at 0 {fnum(ty)} {fnum(rot)}) (layer "F.SilkS")'
             f' (tstamp {uid("ref"+ref)}) (effects (font (size 0.7 0.7) (thickness 0.11))))')
    by = (fp.courtyard[3] + 0.6) if fp.courtyard else 1.6
    o.append(f'    (fp_text value "{val}" (at 0 {fnum(by)} {fnum(rot)}) (layer "F.Fab")'
             f' (tstamp {uid("val"+ref)}) (effects (font (size 0.7 0.7) (thickness 0.11))))')
    for i, (x1,y1,x2,y2) in enumerate(fp.silk):
        o.append(f'    (fp_line (start {XY(x1,y1)}) (end {XY(x2,y2)})'
                 f' (stroke (width 0.12) (type solid)) (layer "F.SilkS") (tstamp {uid(f"s{ref}{i}")}))')
    for i, (x1,y1,x2,y2) in enumerate(fp.fab):
        o.append(f'    (fp_line (start {XY(x1,y1)}) (end {XY(x2,y2)})'
                 f' (stroke (width 0.1) (type solid)) (layer "F.Fab") (tstamp {uid(f"f{ref}{i}")}))')
    for i, (cx,cy,r,lay) in enumerate(fp.circles):
        o.append(f'    (fp_circle (center {XY(cx,cy)}) (end {XY(cx+r,cy)})'
                 f' (stroke (width 0.12) (type solid)) (fill none) (layer "{lay}")'
                 f' (tstamp {uid(f"c{ref}{i}")}))')
    if fp.courtyard:
        x1,y1,x2,y2 = fp.courtyard
        for i,(a,b,c,d) in enumerate(FL.rect(x1,y1,x2,y2)):
            o.append(f'    (fp_line (start {XY(a,b)}) (end {XY(c,d)})'
                     f' (stroke (width 0.05) (type solid)) (layer "F.CrtYd") (tstamp {uid(f"y{ref}{i}")}))')
    for i, pad in enumerate(fp.pads):
        net = PADNET.get((ref, pad.name))
        nl = f' (net {NETNO[net]} "{net}")' if net else ""
        pat = f"{XY(pad.x,pad.y)} {fnum(rot)}" if rot else XY(pad.x, pad.y)
        dr = f" (drill {fnum(pad.drill)})" if pad.drill else ""
        rr = ' (roundrect_rratio 0.25)' if pad.shape == "roundrect" else ""
        lays = " ".join(f'"{l}"' for l in pad.layers)
        nm = pad.name if pad.name else ""
        o.append(f'    (pad "{nm}" {pad.ptype} {pad.shape} (at {pat}) (size {XY(pad.w,pad.h)})'
                 f'{dr} (layers {lays}){rr}{nl} (tstamp {uid(f"p{ref}{i}")}))')
    o.append("  )")
    return o

def build():
    o = ['(kicad_pcb (version 20221018) (generator pcbnew)', '',
         '  (general (thickness 1.6))', '  (paper "A4")', '  (layers']
    for n, nm, ty, usr in LAYERS:
        o.append(f'    ({n} "{nm}" {ty}' + (f' "{usr}")' if usr else ')'))
    o += ['  )', '  (setup',
          f'    (pad_to_mask_clearance 0.05)',
          '    (pcbplotparams (layerselection 0x00010fc_ffffffff) (plot_on_all_layers_selection 0x0000000_00000000)',
          '      (disableapertmacros false) (usegerberextensions false) (usegerberattributes true)',
          '      (usegerberadvancedattributes true) (creategerberjobfile true) (dashed_line_dash_ratio 12.000000)',
          '      (dashed_line_gap_ratio 3.000000) (svgprecision 4) (plotframeref false) (viasonmask false)',
          '      (mode 1) (useauxorigin false) (hpglpennumber 1) (hpglpenspeed 20) (hpglpendiameter 15.000000)',
          '      (pdf_front_fp_property_popups true) (pdf_back_fp_property_popups true) (dxfpolygonmode true)',
          '      (dxfimperialunits true) (dxfusepcbnewfont true) (psnegative false) (psa4output false)',
          '      (plotreference true) (plotvalue true) (plotinvisibletext false) (sketchpadsonfab false)',
          '      (subtractmaskfromsilk false) (outputformat 1) (mirror false) (drillshape 1)',
          '      (scaleselection 1) (outputdirectory ""))', '  )', '']
    o.append('  (net 0 "")')
    for n in NETS: o.append(f'  (net {NETNO[n]} "{n}")')
    o.append("")
    for ref in P.PLACE: o += emit_footprint(ref)
    for i, (mx, my) in enumerate(P.MOUNT):
        fp = FL.mounting_hole_m3()
        o += [f'  (footprint "{fp.name}" (layer "F.Cu") (tstamp {uid(f"mh{i}")})',
              f'    (at {XY(mx,my)}) (attr through_hole exclude_from_pos_files exclude_from_bom)',
              f'    (fp_circle (center 0 0) (end 1.9 0) (stroke (width 0.15) (type solid))'
              f' (fill none) (layer "F.SilkS") (tstamp {uid(f"mhc{i}")}))',
              f'    (pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) (drill 3.2)'
              f' (layers "F&B.Cu" "*.Mask") (tstamp {uid(f"mhp{i}")}))', '  )']
    o.append("")
    pts = P.OUTLINE
    for i in range(len(pts)):
        a, b = pts[i], pts[(i+1) % len(pts)]
        o.append(f'  (gr_line (start {XY(*a)}) (end {XY(*b)}) (stroke (width 0.1) (type solid))'
                 f' (layer "Edge.Cuts") (tstamp {uid(f"edge{i}")}))')
    o.append("")
    k = 0
    for net, layer, wp, w in R.TRACKS:
        for i in range(len(wp)-1):
            if wp[i] == wp[i+1]: continue
            o.append(f'  (segment (start {XY(*wp[i])}) (end {XY(*wp[i+1])}) (width {fnum(w)})'
                     f' (layer "{layer}") (net {NETNO[net]}) (tstamp {uid(f"t{k}")}))')
            k += 1
    for i, (net, x, y) in enumerate(R.VIAS):
        o.append(f'  (via (at {XY(x,y)}) (size {fnum(P.VIA_D)}) (drill {fnum(P.VIA_DRILL)})'
                 f' (layers "F.Cu" "B.Cu") (net {NETNO[net]}) (tstamp {uid(f"v{i}")}))')
    for i, (x, y) in enumerate(R.GND_VIAS):
        o.append(f'  (via (at {XY(x,y)}) (size {fnum(P.VIA_D)}) (drill {fnum(P.VIA_DRILL)})'
                 f' (layers "F.Cu" "B.Cu") (net {NETNO["GND"]}) (tstamp {uid(f"gv{i}")}))')
    o.append("")
    poly = " ".join(f"(xy {XY(*p)})" for p in P.OUTLINE)
    for lay in ("F.Cu", "B.Cu"):
        o += [f'  (zone (net {NETNO["GND"]}) (net_name "GND") (layer "{lay}")',
              f'    (tstamp {uid("zone"+lay)}) (hatch edge 0.5) (connect_pads (clearance 0.3))',
              '    (min_thickness 0.25) (filled_areas_thickness no)',
              '    (fill yes (thermal_gap 0.4) (thermal_bridge_width 0.4))',
              f'    (polygon (pts {poly}))', '  )']
    ax1 = P.U1_AT[0]+FL.ANT_KEEPOUT_LOCAL[0]; ay1 = P.U1_AT[1]+FL.ANT_KEEPOUT_LOCAL[1]
    ax2 = P.U1_AT[0]+FL.ANT_KEEPOUT_LOCAL[2]; ay2 = P.U1_AT[1]+FL.ANT_KEEPOUT_LOCAL[3]
    kp = f"(xy {XY(ax1,ay1)}) (xy {XY(ax2,ay1)}) (xy {XY(ax2,ay2)}) (xy {XY(ax1,ay2)})"
    o += [f'  (zone (net 0) (net_name "") (layers "F&B.Cu") (tstamp {uid("antkp")})',
          '    (name "zona de antena - sin cobre") (hatch edge 0.5)',
          '    (connect_pads (clearance 0)) (min_thickness 0.25)',
          '    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed)',
          '      (copperpour not_allowed) (footprints allowed))',
          f'    (polygon (pts {kp}))', '  )']
    o.append(")")
    return "\n".join(o) + "\n"

if __name__ == "__main__":
    open("bateria48v.kicad_pcb", "w").write(build())
    print("bateria48v.kicad_pcb escrito")
