# Pendientes

Estado al 2026-10-29: generada con `gen.py` y DRC de KiCad 10.0.7
(`kicad-cli pcb drc --refill-zones --severity-all`) con 0 violaciones y 0 sin conectar.

## Bloquea la fabricación

- [ ] **Comprobar la polaridad con los conectores en la mano.** Las redes siguen la
      serigrafía de la huella del XT30U (pad 1 = `-`, del lado del chaflán; pad 2 =
      `+`); el XT60 vertical no trae marca y se asumió el mismo criterio. Antes de
      fabricar, poner un XT60 y un XT30 reales sobre la vista de KiCad y comprobar
      que el `-` moldeado en el cuerpo cae sobre el pad 1: con un canal invertido se
      quema el driver.
- [ ] **Confirmar la tensión del PTC 0ZRE0005FF** (F9) en la hoja de Bel Fuse. La
      serie 0ZRE es de 60 V: alcanza para 13S Li-ion (54.6 V) pero queda justa para
      16S LiFePO4 (58.4 V).
- [ ] **Pedir cobre de 2 oz** en el formulario del fabricante (ver README, "Fabricación").

## Decidir

- [ ] **Elegir el módulo DC-DC.** Entrada de 60 V o más y 5 V a 5 A o más (README,
      "Reductor de 5 V"). Con el módulo en la mano, ver dónde se monta y que sus
      cables lleguen a J11.

- [ ] **Fusibles con el robot andando.** Son T10A 5×20, pensados para motores al
      ~50 % (unos 3 A). Si saltan en los arranques, probar primero con fusibles
      retardados de otra marca; 10 A es el máximo del formato 5×20, así que si no
      alcanza, hay que cambiar a 6.3×32 y agrandar la placa.
- [ ] **Clips Littelfuse 445-030 en la mano.** Antes de fabricar, medir uno: 2 patas
      a 5.2 mm, que entren en agujeros de 1.3 mm. Si son copias con otras medidas,
      hay que ajustar la huella.
- [ ] **Corriente total real.** Medir con el robot andando. Si pasa de ~30 A
      sostenidos, cambiar el XT60 por un XT90 (README, "Corriente").
- [ ] **Encaje mecánico.** Los conectores van verticales, con los cables saliendo
      hacia arriba, y los agujeros M3 a 5 mm de las esquinas. Comprobar que hay
      altura libre sobre la placa y que los agujeros coinciden con el chasis.

## Notas

- El cable de cada motor lleva un XT30 macho de 2 pines del lado de la placa y un
  XT30(2+2)-F del lado del driver, con el CAN entrando por ese mismo conector
  (README, "Polaridad y géneros").
- J10 va pin a pin al J1 del medidor: 1 = BAT+ y 2 = GND en las dos placas.
