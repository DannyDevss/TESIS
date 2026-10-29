# Medidor de batería 48 V — ESP32-S3-WROOM-1

Placa de 2 capas, 110 × 80 mm: divisor de tensión a GPIO4, buzzer pasivo y 3 LEDs.

**Todo es pasante (through-hole)** menos el módulo, que sólo existe en SMD.
Resistencias axiales de 1/4 W a paso 10.16 mm, cerámicos de disco, electrolíticos
radiales, LEDs de 5 mm, TO-92 y TO-220. Soldable a mano sin lupa.

## Archivos

| Archivo | Qué es |
|---|---|
| `bateria48v.kicad_pcb` | La placa. **Footprints embebidos**: abre sin instalar librerías. |
| `bateria48v.kicad_dru` | Reglas DRC propias para la sección de 48 V. |
| `preview.png` / `preview.svg` | Vista previa del ruteo. |
| `bom.csv`, `netlist.txt`, `pinout.txt` | Lista de materiales, netlist y asignación de pines. |
| `design.py`, `place.py`, `routes.py`, `fp_lib.py` | Fuente paramétrica: cambias un número y regeneras. |
| `gen.py`, `drc.py`, `verify.py`, `check_place.py` | Generador y verificadores. |
| `PENDIENTES.md` | Lo que falta por hacer antes de fabricar. |

Regenerar todo: `python3 gen.py && python3 drc.py && python3 verify.py`

## Lo primero que tienes que decidir

**El divisor 180k/10k satura el ADC si tu pack pasa de ~55 V.**

Ganancia = 10k/190k = 1/19. Con atenuación de 12 dB el ADC del S3 llega a ~3.1 V
pero deja de ser lineal por encima de ~2.8 V:

| Batería | V en GPIO4 | Estado |
|---|---|---|
| 13S Li-ion vacía (39 V) | 2.053 V | bien |
| 13S nominal (48.1 V) | 2.532 V | bien |
| **13S llena (54.6 V)** | **2.874 V** | en el borde de la zona lineal |
| 15S LiFePO4 llena (54.75 V) | 2.882 V | en el borde |
| **16S LiFePO4 llena (58.4 V)** | **3.074 V** | **satura, lectura inservible** |

- Si tu pack es **13S Li-ion o 15S LiFePO4**: 180k/10k funciona, con poco margen arriba.
- Si es **16S LiFePO4** o quieres margen: pon **R1 = 220k** (ganancia 1/23 → 58.4 V da 2.54 V).
  La huella es la misma, es cambiar el componente.

Dejé 180k/10k como valor por defecto porque es lo que pediste.

## Antes de mandar a fabricar

1. **Contrasta el footprint del módulo con el datasheet de Espressif.** No tenía KiCad
   ni acceso al PDF, así que reconstruí las cotas (18 × 25.5 mm, paso 1.27 mm,
   pads 0.9 × 2.0 mm, 15 + 10 + 15 castellados + pad térmico) y las dejé como
   constantes al principio de `fp_lib.py`. Si alguna no cuadra, la cambias ahí y
   corres `python3 gen.py`: se regenera la placa entera.
   Es lo único SMD de la placa; el resto son taladros de 0.9 mm con pad de 1.8 mm.
2. Abre el `.kicad_pcb`, pulsa **B** para rellenar los planos de GND y corre el
   **DRC de KiCad**. Mi verificador comprueba separaciones y conectividad, pero no
   sustituye al de KiCad (anillos de vía, islas de cobre aisladas, máscara).
3. Carga `bateria48v.kicad_dru` en *Rules → Custom Rules* para la regla de 1 mm en 48 V.

## Tres piezas que no existen en pasante

Al pasar todo a through-hole hubo que sustituir:

| Antes (SMD) | Ahora (pasante) | Por qué |
|---|---|---|
| BAT54S (doble diodo en SOT-23) | **D5A + D5B**, dos 1N4148 en DO-35 | el doble diodo en serie sólo viene en SOT-23 |
| MMBT2222A (SOT-23) | **2N3904** en TO-92 | mismo transistor, encapsulado pasante |
| AMS1117-3.3 (SOT-223) | **LM1117T-3.3** en TO-220 | vertical, 1=GND 2=VOUT 3=VIN |

Ojo con las patillas: el **2N3904 en TO-92 es E-B-C** mirando la cara plana de frente.
Los pads llevan la marca en serigrafía, pero confírmalo con tu datasheet porque hay
TO-92 con el orden invertido.

En los polarizados el **pad cuadrado es el positivo / ánodo**: electrolíticos (C3, C4,
C6, C7), LEDs (pata larga) y la banda del cátodo en los diodos DO-35.

## Alimentación

La placa **no lleva el convertidor de 48 V a 5 V**. El único cobre que ve los 48 V es
el divisor, confinado en la franja central-inferior con 1 mm de separación.

- **J1** (bornera): sólo sensado de batería. BAT+ y GND.
- **J2** (bornera): entrada de **5 V** desde un DC-DC externo de 48→5 V (necesita uno
  con entrada de 60 V: XL7005, LM5164 o un ladrillo aislado; los mini-360 / MP1584
  no aguantan 48 V).
- **U2** (AMS1117-3.3) genera los 3.3 V a bordo.

Consumo parásito del divisor: 287 µA a 54.6 V ≈ 0.21 mAh/mes. Si la placa va a estar
meses desconectada, conviene un MOSFET que corte el divisor; no lo incluí.

## Conectores

- **J3 — programación** (1×6, 2.54 mm): `1:TXD0  2:RXD0  3:IO0  4:EN  5:3V3  6:GND`.
  Adaptador USB-UART de **3.3 V**. Ese orden está elegido para que las cuatro señales
  lleguen sin cruzarse; si lo cambias, hay que reruear.
- **J4 — jumper del buzzer**: en serie con los 3.3 V. Abierto = mudo, sin tocar firmware.

## Pines usados

| Función | GPIO | Pad | Nota |
|---|---|---|---|
| Sensado de batería | **IO4** | 4 | ADC1_CH3 — ADC1 sigue midiendo con WiFi activo |
| LED verde | IO38 | 31 | |
| LED ámbar | IO39 | 32 | MTCK por defecto |
| LED rojo | IO40 | 33 | MTDO por defecto |
| Buzzer (PWM) | IO21 | 23 | fila inferior, sale hacia el bloque del buzzer |

Usar IO39/IO40 deja sin JTAG por pines, pero el S3 tiene USB-Serial-JTAG integrado
(IO19/IO20), que es la vía habitual, así que no se pierde nada en la práctica.

Los LEDs van **activos en alto** (GPIO → 220 Ω → LED → GND), ~6 mA cada uno.
El buzzer es **pasivo**: necesita PWM (LEDC), no nivel continuo. Lo mueve un
MMBT2222A con diodo de recirculación.

## Firmware — lectura del ADC

```c
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"

#define DIVISOR 19.0f      // (180k + 10k) / 10k

adc_oneshot_unit_handle_t adc;
adc_cali_handle_t cali;

void bat_init(void) {
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&u, &adc);
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(adc, ADC_CHANNEL_3, &c);     // GPIO4 = ADC1_CH3
    adc_cali_curve_fitting_config_t k = {
        .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_3,
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_cali_create_scheme_curve_fitting(&k, &cali);
}

float bat_volts(void) {
    int acc = 0, mv;
    for (int i = 0; i < 64; i++) {            // promediar: el nodo es de alta impedancia
        int raw; adc_oneshot_read(adc, ADC_CHANNEL_3, &raw); acc += raw;
    }
    adc_cali_raw_to_voltage(cali, acc / 64, &mv);
    return mv / 1000.0f * DIVISOR;
}
```

**Calibra con un multímetro.** La tolerancia de dos resistencias al 1 % ya da ±2 %
(±1 V a 48 V), y el ADC del ESP32 suma lo suyo. Mide la tensión real en J1 y la que
reporta el firmware, y ajusta `DIVISOR`. Con una sola constante corriges casi todo.

El filtro RC (R3 = 1 k con C2 = 100 nF) da ~1.6 kHz de corte; con 64 muestras
promediadas la lectura queda estable.

## Lo que no incluí

- Convertidor de 48 V a 5 V a bordo (va fuera, por J2).
- Conector USB-C: se programa por J3. El módulo tiene USB nativo en IO19/IO20 libres
  si lo quieres añadir.
- Corte del divisor por MOSFET para almacenamiento largo.
- Versión SMD: si alguna vez la quieres compacta, `fp_lib.py` conserva las huellas
  0805/SOT-23/SOT-223 de la primera versión.
