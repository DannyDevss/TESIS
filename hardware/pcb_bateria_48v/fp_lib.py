# -*- coding: utf-8 -*-
"""Footprints embebidos: no requieren ninguna libreria de KiCad instalada."""
import math, hashlib, uuid

def uid(s):
    return str(uuid.UUID(hashlib.md5(("bat48v:"+s).encode()).hexdigest()))

def rot(x, y, deg):
    """Rotacion de KiCad: angulo positivo = antihorario en pantalla (Y hacia abajo)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x*c + y*s, -x*s + y*c)

class Pad:
    def __init__(self, name, x, y, w, h, shape="roundrect", ptype="smd",
                 drill=None, layers=None, rratio=0.25):
        self.name, self.x, self.y, self.w, self.h = name, x, y, w, h
        self.shape, self.ptype, self.drill, self.rratio = shape, ptype, drill, rratio
        self.layers = layers or (["*.Cu", "*.Mask"] if ptype != "smd"
                                 else ["F.Cu", "F.Paste", "F.Mask"])

class FP:
    """Footprint: pads en coordenadas locales + graficos de serigrafia."""
    def __init__(self, name, pads, silk=(), fab=(), courtyard=None, attr="smd",
                 circles=()):
        self.name, self.pads = name, list(pads)
        self.silk, self.fab, self.courtyard = list(silk), list(fab), courtyard
        self.attr, self.circles = attr, list(circles)

# --- helpers de geometria -----------------------------------------------------
def rect(x1, y1, x2, y2):
    return [(x1,y1,x2,y1), (x2,y1,x2,y2), (x2,y2,x1,y2), (x1,y2,x1,y1)]

# --- pasivos 0805 -------------------------------------------------------------
def _0805(name, polarized=False):
    pads = [Pad("1", -0.9375, 0, 1.025, 1.4), Pad("2", 0.9375, 0, 1.025, 1.4)]
    silk = [(-1.6,-0.85, 1.6,-0.85), (-1.6,0.85, 1.6,0.85)]
    if polarized:                      # marca de catodo/pin1
        silk += [(-1.75,-0.85, -1.75,0.85)]
    return FP(name, pads, silk, rect(-1.0,-0.62,1.0,0.62), (-1.9,-1.1,1.9,1.1))

R_0805   = _0805("R_0805_2012Metric")
C_0805   = _0805("C_0805_2012Metric")
LED_0805 = _0805("LED_0805_2012Metric", polarized=True)   # pad1 = anodo

# --- SOD-123 (1N4148W): pad1 = anodo, pad2 = catodo ---------------------------
D_SOD123 = FP("D_SOD123",
    [Pad("1", -1.65, 0, 1.2, 1.2), Pad("2", 1.65, 0, 1.2, 1.2)],
    silk=[(-1.4,-0.9, 1.4,-0.9), (-1.4,0.9, 1.4,0.9), (1.55,-0.9, 1.55,0.9)],
    fab=rect(-1.4,-0.85,1.4,0.85), courtyard=(-2.5,-1.2,2.5,1.2))

# --- SOT-23: 1 y 2 abajo, 3 arriba -------------------------------------------
SOT23 = FP("SOT-23",
    [Pad("1", -0.95, 0.95, 1.0, 1.1), Pad("2", 0.95, 0.95, 1.0, 1.1),
     Pad("3", 0.0, -0.95, 1.0, 1.1)],
    silk=[(-1.6,-1.6, 1.6,-1.6), (-1.6,1.6, 1.6,1.6), (-1.6,-1.6,-1.6,1.6)],
    fab=rect(-1.45,-0.65,1.45,0.65), courtyard=(-2.0,-1.9,2.0,1.9))

# --- SOT-223 (AMS1117): 1,2,3 abajo + lengueta (= pad 2, Vout) arriba ---------
SOT223 = FP("SOT-223-3_TabPin2",
    [Pad("1", -2.3, 3.15, 1.2, 2.2), Pad("2", 0.0, 3.15, 1.2, 2.2),
     Pad("3", 2.3, 3.15, 1.2, 2.2), Pad("2", 0.0, -3.15, 3.8, 2.2)],
    silk=[(-3.5,-2.0, 3.5,-2.0), (-3.5,2.0, 3.5,2.0),
          (-3.5,-2.0,-3.5,2.0), (3.5,-2.0, 3.5,2.0)],
    fab=rect(-3.3,-1.8,3.3,1.8), courtyard=(-3.8,-4.5,3.8,4.5))

# --- pulsador tactil 6x6 SMD (dos contactos, dos pads cada uno) ---------------
TACT_6x6 = FP("SW_SPST_6x6mm_SMD",
    [Pad("1", -3.25, -2.25, 1.55, 1.3), Pad("1", -3.25, 2.25, 1.55, 1.3),
     Pad("2",  3.25, -2.25, 1.55, 1.3), Pad("2",  3.25, 2.25, 1.55, 1.3)],
    silk=[(-3.0,-3.2, 3.0,-3.2), (-3.0,3.2, 3.0,3.2)],
    fab=rect(-3.0,-3.0,3.0,3.0), courtyard=(-4.3,-3.6,4.3,3.6))

# --- conectores pasantes ------------------------------------------------------
def header(n, pitch=2.54, name=None):
    pads, y0 = [], -(n-1)*pitch/2
    for i in range(n):
        pads.append(Pad(str(i+1), 0, y0+i*pitch, 1.7, 1.7,
                        shape=("rect" if i == 0 else "circle"),
                        ptype="thru_hole", drill=1.0))
    h = (n-1)*pitch/2 + 1.27
    return FP(name or f"PinHeader_1x{n:02d}_P2.54mm_Vertical", pads,
              silk=rect(-1.27,-h,1.27,h), fab=rect(-1.27,-h,1.27,h),
              courtyard=(-1.6,-h-0.3,1.6,h+0.3), attr="through_hole")

def terminal_block_2():
    p = 5.08
    pads = [Pad("1", -p/2, 0, 2.6, 2.6, shape="rect", ptype="thru_hole", drill=1.3),
            Pad("2",  p/2, 0, 2.6, 2.6, shape="circle", ptype="thru_hole", drill=1.3)]
    return FP("TerminalBlock_bornier-2_P5.08mm", pads,
              silk=rect(-5.1,-4.4,5.1,4.4), fab=rect(-5.1,-4.4,5.1,4.4),
              courtyard=(-5.4,-4.7,5.4,4.7), attr="through_hole")

def buzzer_12mm():
    p = 7.6
    pads = [Pad("1", -p/2, 0, 1.8, 1.8, shape="rect", ptype="thru_hole", drill=1.1),
            Pad("2",  p/2, 0, 1.8, 1.8, shape="circle", ptype="thru_hole", drill=1.1)]
    return FP("Buzzer_12x9.5RM7.6mm", pads, silk=[], fab=[],
              courtyard=(-6.3,-6.3,6.3,6.3), attr="through_hole",
              circles=[(0,0,6.0,"F.SilkS"), (0,0,6.0,"F.Fab")])

def mounting_hole_m3():
    return FP("MountingHole_3.2mm_M3", 
              [Pad("", 0, 0, 3.2, 3.2, shape="circle", ptype="np_thru_hole",
                   drill=3.2, layers=["*.Cu","*.Mask"])],
              silk=[], fab=[], courtyard=(-3.2,-3.2,3.2,3.2), attr="through_hole",
              circles=[(0,0,3.2,"F.SilkS")])

# =============================================================================
# ESP32-S3-WROOM-1
# -----------------------------------------------------------------------------
# COTAS -- verificar contra el datasheet oficial de Espressif antes de fabricar.
# Si alguna difiere, se cambia aqui y se regenera todo.
# =============================================================================
W_BODY      = 18.00   # ancho del modulo (mm)
L_BODY      = 25.50   # largo del modulo (mm)
W_PITCH     = 1.27    # paso entre pads castellados
W_PAD_W     = 0.90    # ancho del pad (en direccion del paso)
W_PAD_L     = 2.00    # largo del pad (perpendicular al borde)
W_OVERHANG  = 0.50    # cuanto sobresale el pad del borde del modulo
                      # (0.5mm de cobre visible -> menisco inspeccionable a mano)
W_N_SIDE    = 15      # pads por columna lateral
W_N_BOT     = 10      # pads en la fila inferior
W_BOT_GAP   = 1.50    # del pad lateral mas bajo al borde inferior del modulo
W_ANT_LEN   = 6.00    # zona de antena medida desde el borde superior
W_EPAD      = 3.70    # lado del pad termico central
W_EPAD_Y    = 5.00    # desplazamiento del pad termico respecto al centro

def esp32s3_wroom1():
    """Origen en el centro geometrico del modulo. Antena hacia -Y (arriba)."""
    hx, hy = W_BODY/2, L_BODY/2
    cx = hx + W_OVERHANG - W_PAD_L/2      # |x| del centro de los pads laterales
    cy = hy + W_OVERHANG - W_PAD_L/2      # y del centro de los pads inferiores
    y_bot = hy - W_BOT_GAP                # y del pad lateral mas bajo (pin 15 / 26)

    pads = []
    # pads 1..15: columna izquierda, de arriba hacia abajo
    for i in range(W_N_SIDE):
        n = i + 1
        y = y_bot - (W_N_SIDE - 1 - i) * W_PITCH
        pads.append(Pad(str(n), -cx, y, W_PAD_L, W_PAD_W))
    # pads 16..25: fila inferior, de izquierda a derecha
    x0 = -(W_N_BOT - 1) * W_PITCH / 2
    for i in range(W_N_BOT):
        n = W_N_SIDE + 1 + i
        pads.append(Pad(str(n), x0 + i*W_PITCH, cy, W_PAD_W, W_PAD_L))
    # pads 26..40: columna derecha, de abajo hacia arriba
    for i in range(W_N_SIDE):
        n = W_N_SIDE + W_N_BOT + 1 + i
        y = y_bot - i * W_PITCH
        pads.append(Pad(str(n), cx, y, W_PAD_L, W_PAD_W))
    # pad 41: termico central
    pads.append(Pad("41", 0, W_EPAD_Y, W_EPAD, W_EPAD, shape="rect"))

    y_ant = -hy + W_ANT_LEN
    silk = [(-hx, y_ant, hx, y_ant)]                    # limite de la antena
    fab  = rect(-hx, -hy, hx, hy)
    return FP("ESP32-S3-WROOM-1", pads, silk, fab,
              courtyard=(-hx-W_OVERHANG-0.3, -hy-0.3, hx+W_OVERHANG+0.3, hy+0.3))

WROOM1 = esp32s3_wroom1()

# zona de exclusion de cobre bajo/alrededor de la antena, en coords locales
ANT_KEEPOUT_LOCAL = (-W_BODY/2 - 2.0, -L_BODY/2 - 0.5,
                      W_BODY/2 + 2.0, -L_BODY/2 + W_ANT_LEN + 1.0)


# =============================================================================
# Componentes PASANTES (through-hole).  Paso de 2.54 mm y multiplos: todo
# soldable a mano y compatible con protoboard.
# =============================================================================
D_PAD, D_DRILL = 1.8, 0.9          # pad y taladro estandar para patas de 0.6 mm

def _th(name, x, y, shape="circle", pad=D_PAD, drill=D_DRILL):
    return Pad(name, x, y, pad, pad, shape=shape, ptype="thru_hole", drill=drill)

def r_axial(P=10.16, body=3.2):
    """Resistencia axial de 1/4 W, patas dobladas a paso P."""
    pads = [_th("1", -P/2, 0), _th("2", P/2, 0)]
    silk = rect(-body, -1.3, body, 1.3) + [
        (-P/2+1.2, 0, -body, 0), (body, 0, P/2-1.2, 0)]
    return FP(f"R_Axial_DIN0207_P{P:.2f}mm", pads, silk, rect(-body,-1.25,body,1.25),
              courtyard=(-P/2-1.3, -1.9, P/2+1.3, 1.9), attr="through_hole")

def c_disc(P=5.08):
    """Ceramico de disco (100 nF y similares)."""
    pads = [_th("1", -P/2, 0), _th("2", P/2, 0)]
    return FP(f"C_Disc_P{P:.2f}mm", pads, rect(-3.4,-1.7,3.4,1.7), rect(-3.4,-1.7,3.4,1.7),
              courtyard=(-P/2-1.3,-2.3,P/2+1.3,2.3), attr="through_hole")

def cp_radial(D=5.0, P=2.5):
    """Electrolitico radial.  Pad 1 cuadrado = positivo."""
    pads = [_th("1", -P/2, 0, "rect"), _th("2", P/2, 0)]
    return FP(f"CP_Radial_D{D:.1f}mm_P{P:.2f}mm", pads, [], [],
              courtyard=(-D/2-1.3,-D/2-1.3,D/2+1.3,D/2+1.3), attr="through_hole",
              circles=[(0,0,D/2,"F.SilkS")])

def led_5mm(P=2.54):
    """LED de 5 mm.  Pad 1 cuadrado = anodo (pata larga)."""
    pads = [_th("1", -P/2, 0, "rect"), _th("2", P/2, 0)]
    return FP("LED_D5.0mm", pads, [], [],
              courtyard=(-3.1,-3.1,3.1,3.1), attr="through_hole",
              circles=[(0,0,2.6,"F.SilkS")])

def d_do35(P=7.62):
    """Diodo DO-35 (1N4148).  Pad 2 cuadrado = catodo, junto a la banda."""
    pads = [_th("1", -P/2, 0), _th("2", P/2, 0, "rect")]
    silk = rect(-2.0,-1.0,2.0,1.0) + [(1.3,-1.0,1.3,1.0),
        (-P/2+1.2,0,-2.0,0), (2.0,0,P/2-1.2,0)]
    return FP(f"D_DO-35_P{P:.2f}mm", pads, silk, rect(-2.0,-0.95,2.0,0.95),
              courtyard=(-P/2-1.3,-1.6,P/2+1.3,1.6), attr="through_hole")

def to92(P=2.54):
    """TO-92 con patas en linea.  Cara plana hacia -Y.  1=E 2=B 3=C (2N3904)."""
    pads = [_th("1", -P, 0, "rect"), _th("2", 0, 0), _th("3", P, 0)]
    silk = [(-2.4,-1.3, 2.4,-1.3), (-2.4,-1.3,-2.4,1.6), (2.4,-1.3,2.4,1.6),
            (-2.4,1.6, 2.4,1.6)]
    return FP("TO-92_Inline", pads, silk, [],
              courtyard=(-P-1.4,-2.2,P+1.4,2.4), attr="through_hole")

def to220_vertical(P=2.54):
    """TO-220 en vertical (LM1117T).  1=GND 2=VOUT 3=VIN.  Cuerpo hacia -Y."""
    pads = [_th("1", -P, 0, "rect", pad=2.2, drill=1.1), _th("2", 0, 0, pad=2.2, drill=1.1),
            _th("3", P, 0, pad=2.2, drill=1.1)]
    silk = rect(-5.1, -13.0, 5.1, -2.2)
    return FP("TO-220-3_Vertical", pads, silk, rect(-5.1,-13.0,5.1,-2.2),
              courtyard=(-5.4,-13.5,5.4,2.0), attr="through_hole")

def tact_6mm_tht():
    """Pulsador tactil de 6x6 mm, 4 patas pasantes."""
    pads = [_th("1",-3.25,-2.25), _th("1",-3.25,2.25),
            _th("2", 3.25,-2.25), _th("2", 3.25,2.25)]
    return FP("SW_Tactile_6x6mm_THT", pads, rect(-3.0,-3.1,3.0,3.1), rect(-3.0,-3.0,3.0,3.0),
              courtyard=(-4.6,-4.1,4.6,4.1), attr="through_hole")
