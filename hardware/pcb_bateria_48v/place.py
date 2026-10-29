# -*- coding: utf-8 -*-
"""Emplazamiento de componentes y ruteo.  Placa 80 x 55 mm, 2 capas."""
import fp_lib as F

BOARD_W, BOARD_H = 110.0, 80.0
TRACK_W   = 0.40      # ancho de pista de senal (generoso: todo es pasante)
TRACK_PWR = 0.70      # ancho de pista de alimentacion
TRACK_HV  = 0.80      # ancho de pista de 48V
VIA_D, VIA_DRILL = 0.9, 0.5
CLEARANCE = 0.30      # separacion minima general
CLEARANCE_HV = 1.00   # separacion minima para los nets de 48V
HV_NETS = {"BAT+", "BAT_DIV"}

U1_AT = (26.0, 14.0)

# Muesca en el contorno bajo la antena: el modulo queda volado sobre aire.
# Debe terminar por encima del pad 1 (y=7.47, borde superior 7.02).
NOTCH = (16.0, 36.0, 6.5)   # x_izq, x_der, profundidad desde el borde superior

# Contorno de la placa recorrido en sentido horario
OUTLINE = [(0,0), (NOTCH[0],0), (NOTCH[0],NOTCH[2]), (NOTCH[1],NOTCH[2]),
           (NOTCH[1],0), (BOARD_W,0), (BOARD_W,BOARD_H), (0,BOARD_H)]

_R  = F.r_axial()
_CD = F.c_disc()
_CP = F.cp_radial()

# ref -> (footprint, x, y, rotacion, valor)
PLACE = {
    "U1": (F.WROOM1,              *U1_AT, 0,   "ESP32-S3-WROOM-1"),

    # --- franja izquierda: desacople y filtro del ADC ---
    "C5": (_CD,                    8.0, 12.0,   0, "100nF"),
    "C6": (_CP,                    8.0, 20.0,   0, "10uF"),
    "C2": (_CD,                    8.0, 28.0, 180, "100nF"),

    # --- alimentacion (abajo a la izquierda) ---
    "J2": (F.terminal_block_2(),  16.0, 72.0,   0, "5V IN"),
    "U2": (F.to220_vertical(),    10.0, 56.0,   0, "LM1117T-3.3"),
    "C3": (_CP,                   20.0, 56.0,   0, "10uF"),
    "C4": (_CP,                    8.0, 63.0, 270, "22uF"),

    # --- sensado de bateria 48V (abajo al centro) ---
    "J1": (F.terminal_block_2(),  36.0, 71.0,   0, "BAT 48V"),
    "R1": (_R,                    34.0, 58.0,  90, "180k"),
    "R2": (_R,                    42.0, 58.0, 270, "10k"),
    "C1": (_CD,                   48.0, 58.0, 270, "100nF"),
    "R3": (_R,                    34.0, 45.0,  90, "1k"),
    "D5A":(F.d_do35(),            12.0, 34.0,   0, "1N4148"),
    "D5B":(F.d_do35(),            12.0, 39.0, 180, "1N4148"),

    # --- LEDs (a la derecha del modulo) ---
    "R6": (_R,                    50.0, 22.0,   0, "220R"),
    "D3": (F.led_5mm(),           62.0, 22.0,   0, "ROJO"),
    "R5": (_R,                    50.0, 30.0,   0, "220R"),
    "D2": (F.led_5mm(),           62.0, 30.0,   0, "AMBAR"),
    "R4": (_R,                    50.0, 38.0,   0, "220R"),
    "D1": (F.led_5mm(),           62.0, 38.0,   0, "VERDE"),

    # --- botones, pull-ups y header de programacion ---
    "SW1":(F.tact_6mm_tht(),      76.0, 22.0,   0, "RESET"),
    "C7": (_CP,                   76.0, 31.0,   0, "1uF"),
    "SW2":(F.tact_6mm_tht(),      76.0, 40.0,   0, "BOOT"),
    "R8": (_R,                    92.0, 22.0,   0, "10k"),
    "R9": (_R,                    92.0, 40.0,   0, "10k"),
    "J3": (F.header(6),          104.0, 50.0,   0, "PROG"),

    # --- buzzer (abajo a la derecha) ---
    "R7": (_R,                    62.0, 70.0,   0, "1k"),
    "Q1": (F.to92(),              74.0, 70.0,   0, "2N3904"),
    "J4": (F.header(2),           83.0, 70.0,   0, "BUZZER EN"),
    "BZ1":(F.buzzer_12mm(),       95.0, 68.0,   0, "BUZZER"),
    "D4": (F.d_do35(),            88.0, 50.0,   0, "1N4148"),
}

MOUNT = [(5.0, 5.0), (105.0, 5.0), (5.0, 75.0), (105.0, 75.0)]

def pad_xy(ref, padname):
    fp, x, y, r, _ = PLACE[ref]
    for p in fp.pads:
        if p.name == padname:
            dx, dy = F.rot(p.x, p.y, r)
            return (round(x+dx, 4), round(y+dy, 4))
    raise KeyError(f"{ref}.{padname}")

def pad_box(ref, padname):
    """Caja envolvente absoluta del pad (x1,y1,x2,y2), con w/h intercambiados si rota 90/270."""
    fp, x, y, r, _ = PLACE[ref]
    out = []
    for p in fp.pads:
        if p.name != padname: continue
        dx, dy = F.rot(p.x, p.y, r)
        w, h = (p.h, p.w) if r % 180 == 90 else (p.w, p.h)
        out.append((x+dx-w/2, y+dy-h/2, x+dx+w/2, y+dy+h/2))
    return out
