#!/usr/bin/env python3
"""
gen.py — Placa de distribución de 48 V para fresar en la Bungard CCD del laboratorio.

Mismo circuito que ../pcb_distribucion_48v (XT60 -> 8 x XT30 con fusible, salida
auxiliar de 48 V con fusible, sensado), rehecho para una fresadora de aislación:

  * UNA SOLA CARA de cobre (B.Cu, el lado de la soldadura). La CCD no metaliza
    agujeros, y los XT60/XT30 verticales se apoyan sobre la cara de arriba: un pad
    arriba no se podría soldar.
  * Cobre de 35 µm (1 oz), el de las placas vírgenes comunes.
  * GND rellena todo lo que no es otra red: queda un anillo por el borde que toca
    los "-" de todos los conectores. Las demás redes van adentro, sin cruzarse.

    python3 gen.py        # escribe distribucion48v_cnc.kicad_pcb con los planos rellenos

Regenerar SOBRESCRIBE el .kicad_pcb (los arreglos van aquí).
"""
import os

import pcbnew

AQUI = os.path.dirname(os.path.abspath(__file__))
SALIDA = os.path.join(AQUI, 'distribucion48v_cnc.kicad_pcb')
LIB = '/usr/share/kicad/footprints'

# ---------------------------------------------------------------- geometría --
# Mismo tamaño y agujeros que la placa para fabricante. Sin el reductor de 5 V la
# placa se acortó de 130 a 104 mm: cabe con margen en una Eurocard de 160 x 100.
ANCHO, ALTO = 104.0, 84.0
PASO_SALIDAS = 16.0                 # separación entre XT30 de una misma fila
X_PRIMER_MAS = 27.0                 # pad "+" del primer XT30 de cada fila
Y_XT30 = 13.0                       # pads del XT30, desde el borde
Y_CLIP_EXTERIOR = 39.5              # clip del lado del bus (pad 1 exterior), desde el borde
MEDIO_ANCHO_OUT = 3.0               # plano de cada canal: 6 mm de ancho
Y_BUS = (31.5, 52.5)                # bus de 48 V, franja central
X_BUS_FIN = 95.0
SEPARACION = 1.0                    # entre redes; IPC-2221 pide 0.6 mm para 48 V sin barniz
SEPARACION_BORDE = 1.0              # la fresa de contorno no debe tocar cobre
ANCHO_PISTA_SENSADO = 0.8
# Columna derecha, después de los 8 canales: arriba la salida auxiliar de 48 V
# (F10 -> J11), abajo el sensado (F9 -> J10).
X_AUX = X_PRIMER_MAS + 4 * PASO_SALIDAS
Y_BORNERA = 10.6                    # pads de las borneras desde el borde: el cuerpo queda al ras

# Sin metalizado el pad es lo único que sujeta la pata: anillos más anchos y
# menos brocas distintas (la CCD cambia herramienta por cada diámetro).
DIAM_PAD_CLIP = 3.0
DIAM_PAD_PTC, DIAM_TALADRO_PTC = 2.0, 0.8
TALADRO_CLIP = 1.4                  # el mismo de las borneras: una broca menos

FILA_DELANTERA = [(1, 'track_fl'), (5, 'flipper_fl'), (6, 'flipper_fr'), (2, 'track_fr')]
FILA_TRASERA = [(3, 'track_rl'), (7, 'flipper_rl'), (8, 'flipper_rr'), (4, 'track_rr')]

# En los AMASS el pad 1 es el NEGATIVO (lado del chaflán) y el pad 2 el POSITIVO.
FP_XT60 = ('Connector_AMASS', 'AMASS_XT60-M_1x02_P7.20mm_Vertical')
FP_XT30 = ('Connector_AMASS', 'AMASS_XT30U-F_1x02_P5.0mm_Vertical')
FP_PORTAFUSIBLE = ('Fuse', 'Fuseholder_Clip-5x20mm_Littelfuse_445-030_Inline_P20.50x5.20mm_D1.30mm_Horizontal')
VALOR_FUSIBLE = 'T10A 5x20'
VALOR_FUSIBLE_AUX = 'T2A 5x20'
FP_PTC = ('Fuse', 'Fuse_BelFuse_0ZRE0005FF_L8.3mm_W3.8mm')
FP_BORNERA = ('Connector_Phoenix_MSTB', 'PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal')
FP_AGUJERO = ('MountingHole', 'MountingHole_3.2mm_M3')

POS_AGUJEROS = [(5.0, 5.0), (ANCHO - 5.0, 5.0), (5.0, ALTO - 5.0), (ANCHO - 5.0, ALTO - 5.0)]


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


class Placa:
    def __init__(self):
        self.board = pcbnew.NewBoard(SALIDA)
        self.redes = {}

    def red(self, nombre):
        if nombre not in self.redes:
            net = pcbnew.NETINFO_ITEM(self.board, nombre)
            self.board.Add(net)
            self.redes[nombre] = net
        return self.redes[nombre]

    def huella(self, fp_id, ref, valor, x, y, grados=0.0, redes=None):
        lib, nombre = fp_id
        fp = pcbnew.FootprintLoad(os.path.join(LIB, lib + '.pretty'), nombre)
        if fp is None:
            raise RuntimeError(f'no encuentro la huella {lib}:{nombre}')
        fp.SetReference(ref)
        fp.SetValue(valor)
        fp.SetPosition(mm(x, y))
        fp.SetOrientationDegrees(grados)
        for pad in fp.Pads():
            if redes and pad.GetNumber() in redes:
                pad.SetNet(self.red(redes[pad.GetNumber()]))
                pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        self.board.Add(fp)
        return fp

    def pad(self, fp, numero):
        """Centro (mm) del primer pad con ese número."""
        for p in fp.Pads():
            if p.GetNumber() == numero:
                pos = p.GetPosition()
                return pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
        raise KeyError(numero)

    def zona(self, red, puntos, prioridad=0):
        z = pcbnew.ZONE(self.board)
        z.SetLayer(pcbnew.B_Cu)
        z.SetNet(self.red(red))
        contorno = z.Outline()
        contorno.NewOutline()
        for x, y in puntos:
            contorno.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        z.SetLocalClearance(pcbnew.FromMM(SEPARACION))
        # Nada más fino que esto: la fresa de aislación no deja tiras de 0.2 mm.
        z.SetMinThickness(pcbnew.FromMM(0.8))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetAssignedPriority(prioridad)
        self.board.Add(z)
        return z

    def rect(self, red, x0, y0, x1, y1, prioridad):
        return self.zona(red, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], prioridad)

    def pista(self, red, puntos, ancho):
        for a, b in zip(puntos, puntos[1:]):
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(mm(*a))
            t.SetEnd(mm(*b))
            t.SetWidth(pcbnew.FromMM(ancho))
            t.SetLayer(pcbnew.B_Cu)
            t.SetNet(self.red(red))
            self.board.Add(t)

    def texto(self, s, x, y, alto=1.2, grados=0.0, capa=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetPosition(mm(x, y))
        t.SetLayer(capa)
        t.SetTextSize(mm(alto, alto))
        t.SetTextThickness(pcbnew.FromMM(max(alto * 0.15, 0.4 if capa == pcbnew.B_Cu else 0)))
        t.SetTextAngleDegrees(grados)
        t.SetMirrored(capa == pcbnew.B_Cu)
        self.board.Add(t)

    def rotulo_cobre(self, s, x, y, grados=0.0):
        """Texto fresado en el cobre de abajo: lo único que queda marcado en la placa."""
        self.texto(s, x, y, alto=2.5, grados=grados, capa=pcbnew.B_Cu)

    def borde(self):
        s = pcbnew.PCB_SHAPE(self.board)
        s.SetShape(pcbnew.SHAPE_T_RECT)
        s.SetStart(mm(0, 0))
        s.SetEnd(mm(ANCHO, ALTO))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        self.board.Add(s)


def ajustar_pads(fp, diametro=None, taladro=None):
    for p in fp.Pads():
        if diametro:
            p.SetSize(pcbnew.F_Cu, mm(diametro, diametro))
            p.SetSize(pcbnew.B_Cu, mm(diametro, diametro))
        if taladro:
            p.SetDrillSize(mm(taladro, taladro))


def salida(placa, n, motor_id, nombre, x_mas, delantera):
    """Un canal: portafusible F<n> -> XT30 J<n+1>, con su plano OUT<n>."""
    red_out = f'OUT{n}'
    y_xt30 = Y_XT30 if delantera else ALTO - Y_XT30
    xt30 = placa.huella(FP_XT30, f'J{n + 1}', 'XT30U-F', x_mas - 5.0, y_xt30,
                        0, {'1': 'GND', '2': red_out})
    if not delantera:
        xt30.Reference().SetPosition(mm(x_mas - 2.5, y_xt30 + 4.6))

    # Portafusible a lo largo de Y bajo el "+": el clip 2 mira al XT30 y el
    # clip 1 al bus central.
    if delantera:
        fus = placa.huella(FP_PORTAFUSIBLE, f'F{n}', VALOR_FUSIBLE, x_mas, Y_CLIP_EXTERIOR,
                           90, {'1': 'VBAT', '2': red_out})
    else:
        fus = placa.huella(FP_PORTAFUSIBLE, f'F{n}', VALOR_FUSIBLE, x_mas, ALTO - Y_CLIP_EXTERIOR,
                           270, {'1': 'VBAT', '2': red_out})
    ajustar_pads(fus, DIAM_PAD_CLIP, TALADRO_CLIP)

    # Plano del canal: del "+" del XT30 al clip 2. Angosto a propósito, para que
    # GND pase por los lados hasta el "-".
    ys2 = [pcbnew.ToMM(p.GetPosition().y) for p in fus.Pads() if p.GetNumber() == '2']
    if delantera:
        y0, y1 = y_xt30 - 2.5, max(ys2) + DIAM_PAD_CLIP / 2 + 0.5
    else:
        y0, y1 = min(ys2) - DIAM_PAD_CLIP / 2 - 0.5, y_xt30 + 2.5
    placa.rect(red_out, x_mas - MEDIO_ANCHO_OUT, y0, x_mas + MEDIO_ANCHO_OUT, y1, prioridad=2)

    y_rot = 25.0 if delantera else ALTO - 25.0
    placa.texto(f'M{motor_id} {nombre}', x_mas - 6.2, Y_CLIP_EXTERIOR - 11.5 if delantera
                else ALTO - Y_CLIP_EXTERIOR + 11.5, alto=1.1, grados=90)
    placa.rotulo_cobre(f'M{motor_id}', x_mas - 8.0, y_rot, grados=90)
    return fus


def bornera_borde(placa, ref, valor, x_pin1, delantera, red_positiva):
    """Bornera de 2 pines en el borde de delante o de atrás, con la entrada del
    cable hacia afuera. Pin 1 = red_positiva, pin 2 = GND (a 5.08 mm)."""
    y = Y_BORNERA if delantera else ALTO - Y_BORNERA
    fp = placa.huella(FP_BORNERA, ref, valor, x_pin1, y, 180 if delantera else 0,
                      {'1': red_positiva, '2': 'GND'})
    for numero, marca in (('1', '+'), ('2', '-')):
        x, yy = placa.pad(fp, numero)
        placa.texto(marca, x, yy + (4.6 if delantera else -4.6), alto=1.5)
    return fp


def main():
    placa = Placa()
    placa.borde()

    # Entrada de batería: XT60 macho a la izquierda, "-" hacia el borde (anillo
    # de GND) y "+" hacia adentro (bus).
    xt60 = placa.huella(FP_XT60, 'J1', 'XT60-M', 8.0, sum(Y_BUS) / 2, 0,
                        {'1': 'GND', '2': 'VBAT'})
    for numero, marca in (('1', '-'), ('2', '+')):
        x, y = placa.pad(xt60, numero)
        placa.texto(marca, x, y - 6.0, alto=2.0)
    placa.texto('BATERIA 48V', 11.6, sum(Y_BUS) / 2 + 7.0, alto=1.5)

    fusibles = []
    for fila, delantera in ((FILA_DELANTERA, True), (FILA_TRASERA, False)):
        for i, (mid, nombre) in enumerate(fila):
            n = len(fusibles) + 1
            fusibles.append(salida(placa, n, mid, nombre, X_PRIMER_MAS + i * PASO_SALIDAS, delantera))
    x_centro = X_PRIMER_MAS + 1.5 * PASO_SALIDAS - 2.5
    placa.texto('DELANTE', x_centro, 3.0, alto=1.2)
    placa.texto('ATRAS', x_centro, ALTO - 3.0, alto=1.2)

    # Salida auxiliar de 48 V: una columna más en la fila de delante, igual que
    # los canales de los motores, con una bornera en vez del XT30. Es para el
    # reductor de la Raspberry y el pi3hat; si se alimentan con baterías aparte,
    # F10 y J11 no se sueldan.
    j11 = bornera_borde(placa, 'J11', '48V AUX', X_AUX, True, 'AUX48V')
    f10 = placa.huella(FP_PORTAFUSIBLE, 'F10', VALOR_FUSIBLE_AUX, X_AUX, Y_CLIP_EXTERIOR, 90,
                       {'1': 'VBAT', '2': 'AUX48V'})
    ajustar_pads(f10, DIAM_PAD_CLIP, TALADRO_CLIP)
    ys2 = [pcbnew.ToMM(p.GetPosition().y) for p in f10.Pads() if p.GetNumber() == '2']
    placa.rect('AUX48V', X_AUX - MEDIO_ANCHO_OUT, Y_BORNERA - 2.5,
               X_AUX + MEDIO_ANCHO_OUT, max(ys2) + DIAM_PAD_CLIP / 2 + 0.5, prioridad=2)
    placa.texto('48V AUX', X_AUX - 6.2, Y_CLIP_EXTERIOR - 11.5, alto=1.1, grados=90)
    placa.rotulo_cobre('AUX', X_AUX - 9.0, 25.0, grados=90)

    # Sensado: J10 en la fila de atrás, con el pin 1 bajo la columna de J11 y el
    # PTC entre el bus y la bornera. Una pista recta del PTC al pin 1.
    j10 = bornera_borde(placa, 'J10', 'SENSADO', X_AUX - 5.0, False, 'SENSE')
    x_sen, y_sen = placa.pad(j10, '1')
    ptc = placa.huella(FP_PTC, 'F9', '0ZRE0005FF', x_sen + 1.6, Y_BUS[1] - 2.0, 270,
                       {'1': 'VBAT', '2': 'SENSE'})
    ajustar_pads(ptc, DIAM_PAD_PTC, DIAM_TALADRO_PTC)
    placa.pista('SENSE', [placa.pad(ptc, '2'), (x_sen, y_sen)], ANCHO_PISTA_SENSADO)
    placa.texto('SENSADO', X_AUX - 6.2, ALTO - 20.0, alto=1.1, grados=90)
    placa.rotulo_cobre('SEN', X_AUX - 9.0, ALTO - 25.0, grados=90)

    for k, (x, y) in enumerate(POS_AGUJEROS, start=1):
        placa.huella(FP_AGUJERO, f'H{k}', 'M3', x, y)

    # Bus de 48 V: franja central desde el "+" del XT60. Toca el lado 1 de los
    # nueve fusibles y el PTC.
    yb0, yb1 = Y_BUS
    placa.rect('VBAT', 11.0, yb0, X_BUS_FIN, yb1, prioridad=1)

    # GND: toda la cara. Rellena lo que dejan las demás redes.
    m = SEPARACION_BORDE
    placa.rect('GND', m, m, ANCHO - m, ALTO - m, prioridad=0)

    ds = placa.board.GetDesignSettings()
    ds.m_NetSettings.GetDefaultNetclass().SetClearance(pcbnew.FromMM(SEPARACION))
    ds.m_CopperEdgeClearance = pcbnew.FromMM(SEPARACION_BORDE)

    pcbnew.ZONE_FILLER(placa.board).Fill(placa.board.Zones())
    placa.board.Save(SALIDA)
    print('escrito', SALIDA)


if __name__ == '__main__':
    main()
