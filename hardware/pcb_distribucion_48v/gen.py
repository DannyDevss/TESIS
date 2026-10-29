#!/usr/bin/env python3
"""
gen.py — Genera la placa de distribución de 48 V (XT60 -> 8 x XT30 con fusible).

Usa la API de KiCad (pcbnew) y las huellas de la librería oficial, así que las
cotas de conectores y portafusibles son las del fabricante, no reconstruidas.

    python3 gen.py        # escribe distribucion48v.kicad_pcb con los planos rellenos

Todo lo que define la placa está en las constantes de abajo: cambias un número
y regeneras. Regenerar SOBRESCRIBE el .kicad_pcb (los arreglos van aquí).
"""
import os

import pcbnew

AQUI = os.path.dirname(os.path.abspath(__file__))
SALIDA = os.path.join(AQUI, 'distribucion48v.kicad_pcb')
LIB = '/usr/share/kicad/footprints'

# ---------------------------------------------------------------- geometría --
ANCHO, ALTO = 130.0, 84.0           # placa, mm
PASO_SALIDAS = 16.0                 # separación entre XT30 de una misma fila
X_PRIMERA_SALIDA = 20.0             # pad 1 del primer XT30 de cada fila
Y_PAD_XT30 = 7.0                    # pads del XT30 a esta distancia del borde
Y_FUSIBLE = 35.5                    # origen del portafusibles, desde el borde
SEPARACION_48V = 1.0                # entre cobres de distinta red (planos)
ANCHO_PISTA_SENSADO = 0.5
ANCHO_PISTA_DCDC = 1.0              # 48 V al reductor: <1 A
X_BUS_FIN = 103.0                   # donde termina el bus de 48 V (empieza la zona de 5 V)

# Salidas por fila, de izquierda a derecha. Agrupadas por esquina, igual que los
# buses CAN (geometria_robot.yaml): oruga y flipper de una esquina van juntos.
# (id del motor, nombre en el código)
FILA_DELANTERA = [(1, 'track_fl'), (5, 'flipper_fl'), (6, 'flipper_fr'), (2, 'track_fr')]
FILA_TRASERA = [(3, 'track_rl'), (7, 'flipper_rl'), (8, 'flipper_rr'), (4, 'track_rr')]

# Huellas (librería, nombre). Conectores XT de cable montados en vertical: el
# enchufe entra desde arriba. En los AMASS el pad 1 es el NEGATIVO, del lado del
# chaflán, y el pad 2 el POSITIVO: lo marca la serigrafía "-" / "+" del XT30U.
FP_XT60 = ('Connector_AMASS', 'AMASS_XT60-M_1x02_P7.20mm_Vertical')
FP_XT30 = ('Connector_AMASS', 'AMASS_XT30U-F_1x02_P5.0mm_Vertical')
# Fusible cerámico 5x20 mm de 250 V en dos clips de 2 patas (Littelfuse 445-030,
# 10 A: los 0445 0001 / 0030 0210 que se venden sueltos).
FP_PORTAFUSIBLE = ('Fuse', 'Fuseholder_Clip-5x20mm_Littelfuse_445-030_Inline_P20.50x5.20mm_D1.30mm_Horizontal')
VALOR_FUSIBLE = 'T10A 5x20'
VALOR_FUSIBLE_DCDC = 'T2A 5x20'     # 25 W a 48 V son ~0.6 A
FP_PTC = ('Fuse', 'Fuse_BelFuse_0ZRE0005FF_L8.3mm_W3.8mm')
FP_BORNERA = ('Connector_Phoenix_MSTB', 'PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal')
FP_BORNERA_4 = ('Connector_Phoenix_MSTB', 'PhoenixContact_MSTBA_2,5_4-G-5,08_1x04_P5.08mm_Horizontal')
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
                # Conexión sólida al plano: por estos pads pasan decenas de amperios.
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

    def ys_pads(self, fp, numero):
        """Coordenada Y (mm) de todos los pads con ese número."""
        return [pcbnew.ToMM(p.GetPosition().y) for p in fp.Pads() if p.GetNumber() == numero]

    def zona(self, red, capa, puntos, prioridad=0):
        z = pcbnew.ZONE(self.board)
        z.SetLayer(capa)
        z.SetNet(self.red(red))
        contorno = z.Outline()
        contorno.NewOutline()
        for x, y in puntos:
            contorno.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        z.SetLocalClearance(pcbnew.FromMM(SEPARACION_48V))
        z.SetMinThickness(pcbnew.FromMM(0.5))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetAssignedPriority(prioridad)
        self.board.Add(z)
        return z

    def pista(self, red, a, b, ancho, capa=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(self.board)
        t.SetStart(mm(*a))
        t.SetEnd(mm(*b))
        t.SetWidth(pcbnew.FromMM(ancho))
        t.SetLayer(capa)
        t.SetNet(self.red(red))
        self.board.Add(t)

    def texto(self, s, x, y, alto=1.2, grados=0.0, capa=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetPosition(mm(x, y))
        t.SetLayer(capa)
        t.SetTextSize(mm(alto, alto))
        t.SetTextThickness(pcbnew.FromMM(alto * 0.15))
        t.SetTextAngleDegrees(grados)
        self.board.Add(t)

    def borde(self):
        s = pcbnew.PCB_SHAPE(self.board)
        s.SetShape(pcbnew.SHAPE_T_RECT)
        s.SetStart(mm(0, 0))
        s.SetEnd(mm(ANCHO, ALTO))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        self.board.Add(s)


def salida(placa, n, motor_id, nombre, x_pad1, delantera):
    """Un canal: portafusible F<n> -> XT30 J<n+1>, con su plano OUT<n>."""
    red_out = f'OUT{n}'
    # Vertical: la orientación no cambia por dónde entra el enchufe, así que las
    # dos filas quedan iguales, con el "-" a la izquierda.
    y_xt30 = Y_PAD_XT30 if delantera else ALTO - Y_PAD_XT30
    xt30 = placa.huella(FP_XT30, f'J{n + 1}', 'XT30U-F', x_pad1, y_xt30,
                        0, {'1': 'GND', '2': red_out})
    x_mas, y_mas = placa.pad(xt30, '2')
    if not delantera:
        # La referencia va por defecto sobre el conector, donde está el fusible.
        xt30.Reference().SetPosition(mm(x_pad1 + 2.5, y_xt30 + 4.6))

    # Portafusible a lo largo del eje Y, bajo el pad positivo: el clip 2 mira al
    # XT30 y el clip 1 al bus central de 48 V.
    if delantera:
        fus = placa.huella(FP_PORTAFUSIBLE, f'F{n}', VALOR_FUSIBLE, x_mas, Y_FUSIBLE,
                           90, {'1': 'VBAT', '2': red_out})
    else:
        fus = placa.huella(FP_PORTAFUSIBLE, f'F{n}', VALOR_FUSIBLE, x_mas, ALTO - Y_FUSIBLE,
                           270, {'1': 'VBAT', '2': red_out})

    # Plano del canal en F.Cu: desde el XT30 hasta el lado 2 del fusible.
    ys2 = placa.ys_pads(fus, '2')
    x0, x1 = x_mas - 4.5, x_mas + 4.5
    if delantera:
        y0, y1 = y_mas - 3.0, max(ys2) + 2.5
    else:
        y0, y1 = min(ys2) - 2.5, y_mas + 3.0
    placa.zona(red_out, pcbnew.F_Cu, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], prioridad=2)

    # Rótulo al costado del fusible: número de motor y su nombre en el código.
    x_txt = x_mas - 6.2
    placa.texto(f'M{motor_id} {nombre}', x_txt, Y_FUSIBLE - 11.5 if delantera else ALTO - Y_FUSIBLE + 11.5,
                alto=1.1, grados=90)
    return fus


def bornera_lateral(placa, ref, valor, y, red_positiva):
    """Bornera de 2 pines en el borde derecho: pin 1 = red_positiva, pin 2 = GND."""
    fp = placa.huella(FP_BORNERA, ref, valor, ANCHO - 10.6, y, 90,
                      {'1': red_positiva, '2': 'GND'})
    for numero, marca in (('1', '+'), ('2', '-')):
        x, yy = placa.pad(fp, numero)
        placa.texto(marca, x - 4.4, yy, alto=1.5)
    return fp


def main():
    placa = Placa()
    placa.borde()

    # Entrada de batería: XT60 macho a la izquierda, sobre el bus de 48 V.
    # Rotado 90°: el pad 2 (+) queda arriba y el pad 1 (-) abajo.
    xt60 = placa.huella(FP_XT60, 'J1', 'XT60-M', 8.0, ALTO / 2 + 3.6, 90,
                        {'1': 'GND', '2': 'VBAT'})
    # La huella vertical no trae marca de polaridad: se la ponemos.
    for numero, marca in (('2', '+'), ('1', '-')):
        x, y = placa.pad(xt60, numero)
        placa.texto(marca, x + 6.3, y, alto=2.0)
    placa.texto('BATERIA 48V', 8.5, ALTO / 2 + 11.0, alto=1.5)
    placa.texto('COBRE 2 oz', ANCHO - 16.0, ALTO - 2.0, alto=1.2)

    n = 1
    fusibles = []
    for i, (mid, nombre) in enumerate(FILA_DELANTERA):
        fusibles.append(salida(placa, n, mid, nombre, X_PRIMERA_SALIDA + i * PASO_SALIDAS, True))
        n += 1
    for i, (mid, nombre) in enumerate(FILA_TRASERA):
        fusibles.append(salida(placa, n, mid, nombre, X_PRIMERA_SALIDA + i * PASO_SALIDAS, False))
        n += 1
    x_centro = X_PRIMERA_SALIDA + 1.5 * PASO_SALIDAS + 2.5
    placa.texto('DELANTE', x_centro, 1.8, alto=1.2)
    placa.texto('ATRAS', x_centro, ALTO - 1.8, alto=1.2)

    # Borde derecho: tres borneras de 2 pines con la entrada de cable hacia afuera.
    # 5 V a la Raspberry, 5 V al J2 del medidor y sensado al J1 del medidor.
    j12 = bornera_lateral(placa, 'J12', '5V PI', 24.0, '+5V')
    j13 = bornera_lateral(placa, 'J13', '5V MEDIDOR', 42.0, '+5V')
    j10 = bornera_lateral(placa, 'J10', 'SENSADO', 60.0, 'SENSE')
    for fp, nombre in ((j12, '5V PI'), (j13, '5V MEDIDOR')):
        x, y = placa.pad(fp, '1')
        placa.texto(nombre, x - 7.0, y - 2.5, alto=1.1, grados=90)
    x, y = placa.pad(j10, '1')
    placa.texto('SENSADO', x, y + 6.5, alto=1.1)

    # Sensado: PTC en serie para que un corto en ese cable fino no se lleve la
    # batería por delante.
    ptc = placa.huella(FP_PTC, 'F9', '0ZRE0005FF', 104.5, 57.5, 0,
                       {'1': 'VBAT', '2': 'SENSE'})
    a = placa.pad(ptc, '2')
    b = placa.pad(j10, '1')
    placa.pista('SENSE', a, (b[0], a[1]), ANCHO_PISTA_SENSADO)
    placa.pista('SENSE', (b[0], a[1]), b, ANCHO_PISTA_SENSADO)

    # Reductor de 5 V: va FUERA de la placa (cualquier módulo de 60 V o más de
    # entrada y 5 V / 5 A) y se conecta a J11. Rotada 180° para que el cable entre
    # por el borde superior. De izquierda a derecha: IN+, -, 5V, -.
    dcdc = placa.huella(FP_BORNERA_4, 'J11', 'DC-DC', 114.3, 11.04, 180,
                        {'4': 'DCDC_IN', '3': 'GND', '2': '+5V', '1': 'GND'})
    for numero, marca in (('3', '-'), ('2', '5V'), ('1', '-')):
        x, y = placa.pad(dcdc, numero)
        placa.texto(marca, x, y + 4.6, alto=1.2)
    x_in, y_in = placa.pad(dcdc, '4')
    placa.texto('IN+', x_in - 6.0, y_in, alto=1.2)
    placa.texto('DC-DC', 88.0, 7.0, alto=1.2)

    # Fusible del reductor: del bus de 48 V a la entrada del módulo.
    f10 = placa.huella(FP_PORTAFUSIBLE, 'F10', VALOR_FUSIBLE_DCDC, x_in, 36.5, 90,
                       {'1': 'VBAT', '2': 'DCDC_IN'})
    # Una sola pista recta pasa por las dos patas del clip 2 hasta la bornera.
    y_f10 = max(placa.ys_pads(f10, '2'))
    placa.pista('DCDC_IN', (x_in, y_f10), (x_in, y_in), ANCHO_PISTA_DCDC)

    for k, (x, y) in enumerate(POS_AGUJEROS, start=1):
        placa.huella(FP_AGUJERO, f'H{k}', 'M3', x, y)

    # Bus de 48 V en F.Cu: franja central que toca el XT60, los lados 1 de los ocho
    # fusibles y el PTC. Los planos OUT tienen más prioridad y no se pisan con éste.
    y_top = max(y for f in fusibles[:4] for y in placa.ys_pads(f, '2')) + 2.5 + SEPARACION_48V
    y_bot = min(y for f in fusibles[4:] for y in placa.ys_pads(f, '2')) - 2.5 - SEPARACION_48V
    placa.zona('VBAT', pcbnew.F_Cu,
               [(2.0, y_top), (X_BUS_FIN, y_top), (X_BUS_FIN, 50.0), (116.0, 50.0),
                (116.0, y_bot), (2.0, y_bot)], prioridad=1)

    # 5 V en F.Cu: arriba a la derecha, de la salida del módulo a J12 y J13 (5 A).
    placa.zona('+5V', pcbnew.F_Cu, [(X_BUS_FIN + 2.5, 0.5), (ANCHO - 0.5, 0.5),
                                    (ANCHO - 0.5, 47.5), (X_BUS_FIN + 2.5, 47.5)], prioridad=1)

    # GND: toda la cara inferior.
    placa.zona('GND', pcbnew.B_Cu, [(0.5, 0.5), (ANCHO - 0.5, 0.5),
                                   (ANCHO - 0.5, ALTO - 0.5), (0.5, ALTO - 0.5)])

    ds = placa.board.GetDesignSettings()
    ds.m_NetSettings.GetDefaultNetclass().SetClearance(pcbnew.FromMM(0.5))
    ds.m_CopperEdgeClearance = pcbnew.FromMM(0.5)

    pcbnew.ZONE_FILLER(placa.board).Fill(placa.board.Zones())
    placa.board.Save(SALIDA)
    print('escrito', SALIDA)


if __name__ == '__main__':
    main()
