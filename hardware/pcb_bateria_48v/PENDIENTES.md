# Pendientes

Estado al 2026-10-02, tras abrir la placa en KiCad 10.0.6 y correr su DRC
(`kicad-cli pcb drc --refill-zones --severity-all`).

## Bloquea la fabricación

- [ ] **Zona de antena pisa la primera fila de pads del ESP32.** El keepout
      (`ANT_KEEPOUT_LOCAL`, `fp_lib.py:164`) llega hasta y = 8.25 mm porque suma
      `W_ANT_LEN + 1.0`, y los pads 1 y 40 (GND) están en y = 7.47 (borde superior
      7.02). Por eso las colas de GND de `routes.py:76-77` dan 2 errores
      `items_not_allowed`. Ajustar el margen para que el keepout termine antes de
      y ≈ 7.0, contrastando con el datasheet.
- [ ] **Verificar el footprint del ESP32-S3-WROOM-1 contra el datasheet de Espressif.**
      Las cotas de `fp_lib.py` (18 × 25.5 mm, paso 1.27, pads 0.9 × 2.0, zona de
      antena de 6 mm) están reconstruidas, no comprobadas. Esto va antes del punto
      anterior, porque cambia dónde termina la antena.
- [ ] **Decidir R1 según el pack:** con 180k/10k el ADC satura si el pack pasa de
      ~55 V (16S LiFePO4 → usar 220k). Ver README, "Lo primero que tienes que decidir".

- [ ] **Confirmar el pinout del 2N3904 (TO-92) con el datasheet del que se compre.**
      La huella asume E-B-C mirando la cara plana; hay TO-92 con el orden invertido.

## Limpieza (no bloquea)

- [ ] Vía de GND duplicada sobre el pad 6 de J3 (`holes_co_located`, 2 avisos):
      sacarla de `GND_VIAS` en `routes.py`.
- [ ] 32 textos de serigrafía más bajos que el mínimo (`text_height`).
- [ ] 26 casos de serigrafía sobre cobre (`silk_over_copper`).
- [ ] 1 vía suelta (`via_dangling`) que queda tras rellenar planos.

## Documentación

- [ ] El README todavía nombra piezas SMD de la primera versión: dice que el buzzer
      lo mueve un MMBT2222A (ahora es 2N3904) y que U2 es AMS1117-3.3 (ahora
      LM1117T-3.3).

## Notas

- Los 51 "no conectados" que aparecen sin rellenar zonas son todos de GND y
  desaparecen al rellenar los planos (tecla **B**). No son un problema.
- Regenerar con `python3 gen.py` **sobrescribe `bateria48v.kicad_pcb`**: cualquier
  cambio hecho a mano en KiCad se pierde. Los arreglos van en los scripts.
- Tras regenerar: abrir en KiCad, **B**, cargar `bateria48v.kicad_dru` y correr el DRC.
