# Manual GIM6010-8 rev 2.2 — traducción al español

Traducción por secciones del manual de usuario del micromotor SteadyWin GIM6010-8
(original en chino, `SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf`, 96 páginas).

| PDF | Contenido | Págs. del original |
|---|---|---|
| `00_Preliminares.pdf` | Portada, índice, registro de versiones, precauciones, aviso legal, postventa | 1–4 |
| `01_Especificaciones_motor.pdf` | 1 Parámetros del motor | 5–7 |
| `02_Informacion_driver.pdf` | 2 Información del driver e interfaces | 8–14 |
| `03-1_Guia_puesta_en_marcha.pdf` | 3.1 Guía de puesta en marcha (Motor Wizard y odrivetool) | 15–38 |
| `03-2_Actualizacion_firmware.pdf` | 3.2 Descarga y actualización de firmware | 39–40 |
| `04-1_CAN_Simple.pdf` | 4.1 Protocolo CAN Simple | 41–53 |
| `04-2_CANopen.pdf` | 4.2 Protocolo CANopen | 54–56 |
| `04-3_Modbus_RTU.pdf` | 4.3 RS485 (Modbus RTU) | 56–80 |
| `04-4_Control_PWM.pdf` | 4.4 Control por entrada PWM | 81–82 |
| `04-5_Pulso_Direccion.pdf` | 4.5 Control por pulso/dirección | 83–84 |
| `04-6_SDK.pdf` | 4.6 SDK de C/C++, Python, Arduino y ROS 2 | 85–94 |
| `05_FAQ_y_codigos_error.pdf` | 5 FAQ y tabla de códigos de error | 95–96 |

Notas sobre la traducción:

- Las 42 figuras del original se han extraído del PDF y se han vuelto a insertar en su
  sitio. Las capturas de pantalla del software de PC siguen en chino (son imágenes); los
  pies de figura traducen las etiquetas de la interfaz que hacen falta para seguir el texto.
- El código de ejemplo (Python, Arduino, comandos de `odrivetool`) se reproduce tal cual;
  solo se han traducido los comentarios propios del manual.
- Donde el original tiene una errata o una incoherencia (registros Modbus del ejemplo de
  posición, modo de par en PWM, «control PWM» en la sección de pulso/dirección, referencia
  al apartado 5.2), se ha traducido lo que dice el original y se ha añadido una
  «Nota del traductor».

## Regenerar los PDF

Los fuentes están en `fuente/` (un `.html` por sección, `estilo.css` y `figuras/`).
Para volver a generarlos hace falta WeasyPrint:

```sh
python3 -m venv .venv && .venv/bin/pip install weasyprint
.venv/bin/weasyprint fuente/04-1_CAN_Simple.html 04-1_CAN_Simple.pdf
```

`fuente/render.sh` hace lo mismo para todos los archivos (apunta al venv usado al
generarlos; ajusta la ruta `VP` si lo reinstalas en otro sitio).
