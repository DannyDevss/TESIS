# textures/

Texturas que los meshes COLLADA de `meshes/` referencian internamente:

| Archivo             | Lo pide            | Vía                                          |
|---------------------|--------------------|----------------------------------------------|
| `chassis.png`       | `chassis.dae`      | `<init_from>../textures/chassis.png</init_from>` |
| `flipper_color.png` | `flipper.dae`      | `<init_from>../textures/flipper_color.png</init_from>` |

## Por qué esta carpeta tiene que existir

Los `.dae` siempre declararon estas rutas, pero los archivos no estaban en el
repositorio. **RViz lo tolera**: avisa por consola y dibuja la malla sin textura.
**Foxglove no**: su cargador COLLADA descarta la malla entera si no puede resolver
la imagen, así que el robot desaparecía del panel 3D y sólo quedaban los ejes de TF.
El síntoma en el log de `foxglove_bridge` es:

    Failed to retrieve asset 'package://ugv_bridge/textures/chassis.png'

## Estado actual

Los dos PNG son **marcadores de posición**: color plano 256x256, grafito para el
chasis y naranja para los flippers (contraste a propósito, para distinguir de un
vistazo la orientación de los brazos en el panel 3D).

Cuando existan las texturas reales del CAD, basta con sobreescribir estos dos
archivos conservando el nombre. No hay que tocar los `.dae` ni el URDF.

Se instalan vía `setup.py` (`glob('textures/*')`), que es lo que permite al puente
resolverlos como `package://ugv_bridge/textures/...`.
