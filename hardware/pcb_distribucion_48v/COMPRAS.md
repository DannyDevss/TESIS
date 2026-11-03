# Lista de compras — placa de distribución de 48 V

Cantidades para **una placa**. Entre paréntesis, la referencia en la placa.

## Para soldar en la placa

| ✓ | Qué | Cant. | Especificación | Dónde / nota |
|---|---|---|---|---|
| [ ] | Fusibles cerámicos 5×20 | 1 kit | **Cerámicos, retardados (T)**, 250 V. Del kit se usan: **8 × T10A** (F1–F8) y **1 × T2A** (F10, solo con reductor) | AliExpress: kit de 100 uds "fusibles de cerámica de soplado lento 5x20mm 1A…20A" (~$6.600). **No** los de vidrio ni los "rápidos" |
| [ ] | Clips para fusible 5×20 | 18 | **Littelfuse 0445 0001 / 0030 0210** (serie 445-030), 10 A, 2 patas a 5.2 mm | AliExpress: lote de 100 (~$10.300). Medir uno al llegar (ver PENDIENTES) |
| [ ] | XT60 **macho** (J1) | 1 | El de cable, cuerpo amarillo, se suelda **vertical** | Viene en pares macho+hembra |
| [ ] | XT30 **hembra** (J2–J9) | 8 | El de cable, cuerpo amarillo, se suelda **vertical** | Viene en pares: los machos sirven para los cables de los motores |
| [ ] | PTC Bel Fuse **0ZRE0005FF** (F9) | 1 | Radial, 60 V | DigiKey / Mouser. Confirmar los 60 V en la ficha |
| [ ] | Bornera Phoenix **MSTBA 2,5/2-G-5,08** (J10, J11) | 2 | 2 pines, paso 5.08 mm, horizontal | Con su enchufe **MSTB 2,5/2-ST-5,08** (2) |

## Fuera de la placa

| ✓ | Qué | Cant. | Especificación | Nota |
|---|---|---|---|---|
| [ ] | Reductor para la Pi y el pi3hat **o** batería aparte | 1 | Reductor: entrada **60 V o más** (mejor 72 V); salida 5 V / 5 A si va al GPIO, o la que pida la entrada del pi3hat | **No** LM2596 ni XL4015 (40 V). Pendiente de decidir (README, "Salida auxiliar de 48 V") |
| [ ] | XT30 **macho** para los cables de motor | 8 | Igual que arriba | Salen de los pares de XT30 |
| [ ] | XT30(2+2)-F, lado del driver | 8 | Potencia + CAN en el mismo conector | Solo si los drivers no traen el cable armado |
| [ ] | Cable para los motores | ~8 tramos | 16–18 AWG (1–1.5 mm²), rojo y negro | XT30 de la placa → driver |
| [ ] | Cable J11 → reductor | 1 tramo | 20–22 AWG basta (<1 A) | Con batería aparte, solo el `-` |
| [ ] | Cable reductor → Raspberry | 1 tramo | **18 AWG (0.75 mm²) o más grueso**, corto | Si va al GPIO: 5V (pin 2/4) y GND (pin 6) |
| [ ] | Cable al medidor (J10 → J1, 5 V → J2) | 2 tramos | 22 AWG basta | El 5 V sale del reductor o de la batería de la Pi |
| [ ] | Tornillos M3 + separadores | 4 | | Agujeros a 94 × 74 mm entre centros |

## Fabricar la placa

| ✓ | Qué | Nota |
|---|---|---|
| [ ] | PCB de 2 capas, 104 × 84 mm, **cobre de 2 oz** | Mandar los archivos de `fabricacion/`. **Pedir 2 oz** en el formulario; con 1 oz el bus de 48 V se queda corto |

Antes de mandar a fabricar, cerrar lo que está en `PENDIENTES.md` (sobre todo la
polaridad del XT60 y las medidas de los clips).
