# -*- coding: utf-8 -*-
"""
Definicion electrica de la placa: medidor de bateria 48V + buzzer + 3 LEDs
sobre ESP32-S3-WROOM-1.  Fuente unica de verdad para netlist, BOM y PCB.
"""

# ----------------------------------------------------------------------------
# Pinout del modulo ESP32-S3-WROOM-1 (40 pads castellados + pad termico 41)
# Distribucion fisica: pads 1..15 columna izquierda (arriba->abajo),
#                      pads 16..25 fila inferior (izq->der),
#                      pads 26..40 columna derecha (abajo->arriba),
#                      pad 41 = pad termico central (GND).
# ----------------------------------------------------------------------------
WROOM_PINS = {
     1: "GND",   2: "3V3",   3: "EN",    4: "IO4",   5: "IO5",
     6: "IO6",   7: "IO7",   8: "IO15",  9: "IO16", 10: "IO17",
    11: "IO18", 12: "IO8",  13: "IO19", 14: "IO20", 15: "IO3",
    16: "IO46", 17: "IO9",  18: "IO10", 19: "IO11", 20: "IO12",
    21: "IO13", 22: "IO14", 23: "IO21", 24: "IO47", 25: "IO48",
    26: "IO45", 27: "IO0",  28: "IO35", 29: "IO36", 30: "IO37",
    31: "IO38", 32: "IO39", 33: "IO40", 34: "IO41", 35: "IO42",
    36: "IO44", 37: "IO43", 38: "IO2",  39: "IO1",  40: "GND",
    41: "GND",
}

# Pines que NO se deben usar libremente
RESERVADOS = {
    3:  "EN  - reset",
    15: "IO3  - strapping (JTAG source)",
    16: "IO46 - strapping, solo entrada",
    26: "IO45 - strapping (VDD_SPI)",
    27: "IO0  - strapping (boot)",
    28: "IO35 - PSRAM octal en variantes R8",
    29: "IO36 - PSRAM octal en variantes R8",
    30: "IO37 - PSRAM octal en variantes R8",
    13: "IO19 - USB D-",
    14: "IO20 - USB D+",
    36: "IO44 - RXD0 consola",
    37: "IO43 - TXD0 consola",
}

# ----------------------------------------------------------------------------
# Asignacion de funciones  (pin_modulo -> (net, motivo))
# ----------------------------------------------------------------------------
ASIGNACION = {
     4: ("BAT_ADC",   "GPIO4 = ADC1_CH3. ADC1 sigue funcionando con WiFi activo."),
    31: ("LED_VERDE", "IO38 libre, sin strapping, en la columna derecha."),
    32: ("LED_AMBAR", "IO39 libre (MTCK por defecto, se pierde JTAG por pines)."),
    33: ("LED_ROJO",  "IO40 libre (MTDO por defecto)."),
    23: ("BUZZER_PWM","IO21 libre, en la fila inferior: sale hacia el bloque del buzzer."),
}

# ----------------------------------------------------------------------------
# Netlist: net -> lista de (referencia, pad)
# ----------------------------------------------------------------------------
NETS = {
    # --- alimentacion ---
    "GND": [("U1","1"),("U1","40"),("U1","41"),("J1","2"),("J2","2"),
            ("U2","1"),("C3","2"),("C4","2"),("C5","2"),("C6","2"),
            ("C1","2"),("C2","2"),("R2","2"),("D5A","1"),
            ("D1","2"),("D2","2"),("D3","2"),("Q1","1"),
            ("SW1","2"),("SW2","2"),("J3","6")],
    "+5V":  [("J2","1"),("U2","3"),("C3","1")],
    "BZ_PWR":[("J4","2"),("BZ1","1")],
    "+3V3": [("U1","2"),("U2","2"),("C4","1"),("C5","1"),("C6","1"),
             ("R8","1"),("R9","1"),("D5B","2"),("J3","5"),("J4","1")],

    # --- sensado de bateria 48V ---
    "BAT+":     [("J1","1"),("R1","1")],
    "BAT_DIV":  [("R1","2"),("R2","1"),("C1","1"),("R3","1")],
    "BAT_ADC":  [("R3","2"),("D5A","2"),("D5B","1"),("C2","1"),("U1","4")],

    # --- control ---
    "EN":       [("U1","3"),("R8","2"),("C7","1"),("SW1","1"),("J3","4")],
    "IO0":      [("U1","27"),("R9","2"),("SW2","1"),("J3","3")],
    "TXD0":     [("U1","37"),("J3","1")],
    "RXD0":     [("U1","36"),("J3","2")],

    # --- LEDs ---
    "LED_VERDE":[("U1","31"),("R4","1")],
    "LED_AMBAR":[("U1","32"),("R5","1")],
    "LED_ROJO": [("U1","33"),("R6","1")],
    "AN_VERDE": [("R4","2"),("D1","1")],
    "AN_AMBAR": [("R5","2"),("D2","1")],
    "AN_ROJO":  [("R6","2"),("D3","1")],

    # --- buzzer ---
    "BUZZER_PWM":[("U1","23"),("R7","1")],
    "Q1_BASE":   [("R7","2"),("Q1","2")],
    "BZ_COL":    [("Q1","3"),("BZ1","2"),("D4","1")],
}
# conexiones que cierran nets ya definidas mas arriba
# (catodo del flyback a +5V, retorno de C7 a GND)
NETS["BZ_PWR"].append(("D4","2"))
NETS["GND"].append(("C7","2"))

# ----------------------------------------------------------------------------
# BOM
# ----------------------------------------------------------------------------
BOM = [
    # ref, valor, huella, descripcion, nota
    ("U1", "ESP32-S3-WROOM-1", "modulo 18x25.5mm, 40 pads castellados 1.27mm",
     "MCU + WiFi/BT", "unico componente SMD: el modulo viene asi"),
    ("U2", "LM1117T-3.3 / LD1117V33", "TO-220 vertical, 3 pines 2.54mm",
     "regulador 5V -> 3.3V, 800mA", "1=GND 2=VOUT 3=VIN"),
    ("J1", "BAT SENSE", "bornera 2 pines 5.08mm", "entrada 48V a medir", "solo sensado, no alimenta"),
    ("J2", "5V IN", "bornera 2 pines 5.08mm", "alimentacion 5V", "desde DC-DC 48->5V externo"),
    ("J3", "PROG", "header 1x6 2.54mm", "1:TXD0 2:RXD0 3:IO0 4:EN 5:3V3 6:GND", "adaptador USB-UART 3.3V"),
    ("J4", "BUZZER EN", "jumper 1x2 2.54mm", "habilita el buzzer", "en serie con +3V3; abrir = mudo"),
    ("R1", "180k 1% 1/4W", "axial, paso 10.16mm", "divisor rama alta", "51.7V sobre ella a 54.6V"),
    ("R2", "10k 1% 1/4W", "axial, paso 10.16mm", "divisor rama baja", "usar 1% o mejor"),
    ("R3", "1k 1/4W", "axial, paso 10.16mm", "serie de proteccion al ADC", "limita la corriente al pin"),
    ("R4", "220R 1/4W", "axial, paso 10.16mm", "limitador LED verde", ""),
    ("R5", "220R 1/4W", "axial, paso 10.16mm", "limitador LED ambar", ""),
    ("R6", "220R 1/4W", "axial, paso 10.16mm", "limitador LED rojo", ""),
    ("R7", "1k 1/4W", "axial, paso 10.16mm", "base de Q1", ""),
    ("R8", "10k 1/4W", "axial, paso 10.16mm", "pull-up EN", ""),
    ("R9", "10k 1/4W", "axial, paso 10.16mm", "pull-up IO0", ""),
    ("C1", "100nF ceramico", "disco, paso 5.08mm", "filtro sobre R2", "polo con R1||R2 = 9.47k"),
    ("C2", "100nF ceramico", "disco, paso 5.08mm", "filtro en el pin del ADC", "cerca del pad 4"),
    ("C3", "10uF 25V", "electrolitico radial D5mm", "entrada del regulador", "pata larga = +"),
    ("C4", "22uF 16V", "electrolitico radial D5mm", "salida del regulador", "pata larga = +"),
    ("C5", "100nF ceramico", "disco, paso 5.08mm", "desacople 3V3 del modulo", "cerca del pad 2"),
    ("C6", "10uF 16V", "electrolitico radial D5mm", "reserva para picos de WiFi", "pata larga = +"),
    ("C7", "1uF 16V", "electrolitico radial D5mm", "RC de arranque en EN", "pata larga = + (al pin EN)"),
    ("D1", "LED verde 5mm", "LED 5mm, paso 2.54mm", "estado OK", "pata larga = anodo = pad cuadrado"),
    ("D2", "LED ambar 5mm", "LED 5mm, paso 2.54mm", "aviso bateria media", "pata larga = anodo"),
    ("D3", "LED rojo 5mm", "LED 5mm, paso 2.54mm", "bateria baja", "pata larga = anodo"),
    ("D4", "1N4148", "DO-35, paso 7.62mm", "recirculacion del buzzer", "banda = catodo = pad cuadrado"),
    ("D5A","1N4148", "DO-35, paso 7.62mm", "clamp del ADC contra GND", "anodo a GND, catodo al nodo"),
    ("D5B","1N4148", "DO-35, paso 7.62mm", "clamp del ADC contra 3V3", "anodo al nodo, catodo a 3V3"),
    ("Q1", "2N3904", "TO-92 en linea, paso 2.54mm", "driver del buzzer", "1=E 2=B 3=C, cara plana hacia arriba"),
    ("SW1","RESET", "tactil 6x6mm pasante", "reset (EN)", ""),
    ("SW2","BOOT", "tactil 6x6mm pasante", "modo descarga (IO0)", ""),
    ("BZ1","buzzer pasivo 12mm", "TH 2 pines, paso 7.6mm", "magnetico pasivo, 3.3V", "necesita PWM, no DC"),
]
