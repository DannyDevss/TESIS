# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Qué es

Workspace colcon de **ROS 2 Jazzy** (tesis) para un UGV de rescate con orugas y 4 flippers articulados. Hay **un solo paquete**, `src/ugv_bridge` (Python, `ament_python`). Toda la documentación y el código están en español; sigue esa convención.

Docs de referencia: `README.md` (resumen), `COMO_EJECUTAR.md` (runbook detallado: launch, tópicos, cableado CAN, motores, RL), `AVANCES.txt` (bitácora).

## Entorno

- En el equipo original no hay ROS en el host: se usa el contenedor `rosdev` (`distrobox enter ros2`, Jazzy) o, como alternativa, `micromamba activate ros2` (Humble).
- **En esta máquina (CachyOS) ROS corre en Docker**: contenedor `ros2_jazzy`, imagen `ros2_jazzy_tesis` (Ubuntu 24.04, Python 3.12) construida desde `~/.config/ros2_jazzy/Dockerfile`. Se entra con la función fish `ros_run` (`ros_run --rebuild` reconstruye la imagen, `ros_run --reset` borra el contenedor). El repo se monta en `/home/ubuntu/TESIS`: rutas de modelos, etc. tienen que estar dentro del repo (p. ej. `modelos/`, excluida en `.git/info/exclude`), no en `/tmp` del host. En el host no hay `ros2`/`colcon`.
- Atajo: `compilar_simu [ESCENARIO] [args:=...]` (script `~/.config/ros2_jazzy/compilar_simu`, copiado a la imagen): cierra la simulación anterior, compila y lanza el gemelo (`flipper.launch.py modo:=gemelo`, sin vcan0). Escenarios: `politica_giro` (regenera `modelos/giro.onnx`) y `politica_<nombre>` (usa `modelos/<nombre>.onnx`). Al cambiar el script, copiarlo al contenedor (`docker cp`) o `ros_run --rebuild`.
- `install/` es una copia (sin `--symlink-install`): tras cambiar código hay que `colcon build --packages-select ugv_bridge` dentro del contenedor, o se ejecuta la versión vieja.
- Las rutas de los docs (`~/Proyectos/ugv_ws`, `~/TESIS`) son de otro equipo; en esta máquina el repo es `~/Proyectos/TESIS` (rama `PRINCIPAL`).
- Para las pruebas sin ROS hay un venv en `.venv/` (excluido en `.git/info/exclude`, no en `.gitignore`) con pytest, numpy, onnx, onnxruntime, flake8 y pydocstyle.

## Comandos

```bash
colcon build --packages-select ugv_bridge && source install/setup.bash

# Tests (ament_flake8, ament_pep257, copyright y el ciclo de bus del driver)
colcon test --packages-select ugv_bridge && colcon test-result --verbose
# Tests sin ROS (DESDE src/ugv_bridge: importan `ugv_bridge` por ruta relativa).
# En esta máquina, con el venv: ../../.venv/bin/python -m pytest ...
python3 -m pytest test/test_driver_bus.py test/test_contrato_politica.py test/test_ejecutor_politica.py
python3 -m pytest test/test_driver_bus.py -k <nombre>

# Modelo ONNX falso con su contrato, para probar la política sin el modelo real:
python3 src/ugv_bridge/scripts/modelo_falso.py -s modelos/falso --ganancia 0 --sesgo 0 0 0 0 0.3 0.3
ros2 launch ugv_bridge can_sim.launch.py use_foxglove:=true use_politica:=true modelo_politica:=/home/ubuntu/TESIS/modelos/falso.onnx

# Launch principales (todos levantan foxglove_bridge en ws://localhost:8765)
ros2 launch ugv_bridge view.launch.py                    # gemelo digital, solo posar
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true   # sistema completo
ros2 launch ugv_bridge can_studio.launch.py              # tramas CAN reales en vcan0 + espía del bus
bash src/ugv_bridge/scripts/setup_vcan.sh                # crear vcan0
bash src/ugv_bridge/scripts/limpiar_fantasmas.sh         # matar publicadores huérfanos

# Tras editar config/geometria_robot.yaml, regenerar el URDF plano:
bash src/ugv_bridge/scripts/generar_urdf.sh
```

## Arquitectura

```
Foxglove / politica_flippers ──/cmd_tracks, /cmd_flippers──► flipper_node ──► RMD_Hardware (driver_movimiento.py)
flipper_node ──/joint_states, /imu/data_raw──► track_odometry_node ──/odom──► EKF (robot_localization) ──► /odometry/filtered + TF
                                          └──► robot_state_publisher (URDF xacro) ──► TF de flippers ──► Foxglove
```

- **Motores**: IDs 1–4 = orugas (consigna de **velocidad**, rad/s); IDs 5–8 = flippers (consigna de **posición**, rad, juntas `continuous` sin topes).
- **`driver_movimiento.RMD_Hardware`** es la capa de abstracción de hardware y devuelve diccionarios planos, no mensajes ROS (la conversión vive en `flipper_node`). Tiene tres `modo`s:
  - `gemelo`: modelo cinemático en memoria.
  - `can`: SocketCAN, normalmente `vcan0` con `motor_emulator` al otro lado.
  - `pi3hat`: motores reales vía `pi3hat_backend`.

  `can` y `pi3hat` comparten **exactamente** el mismo código de protocolo (`_estado_via_bus` + `protocolo_can.py`) y solo cambia el transporte. No dupliques lógica entre ellos. La fuente de IMU (`imu_fuente`: `sintetica` | `pi3hat_real`) es un parámetro independiente del modo. La IMU sintética integra la actitud a partir del estado de los motores; no es ruido.
- **El protocolo CAN es ODrive, no RMD** (a pesar de los nombres): `arbitration_id = (node_id << 5) | cmd_id`. Los motores emiten heartbeat y encoder por su cuenta, y hay que **armar** los ejes (closed loop), porque un eje en IDLE acepta consignas sin moverse ni dar error. El driver rearma al arrancar y tras recuperarse un fallo del watchdog.
- **`flipper_node` retiene el último comando** recibido. Por eso Foxglove es la **única** interfaz: no agregues GUIs Qt/RViz ni otros publicadores a `/cmd_*`, porque se pisan entre sí. La velocidad de las orugas no se resetea sola.
- `test/test_driver_bus.py` sustituye el transporte por uno en memoria que usa el mismo `MotorEmulado` y las mismas funciones de protocolo que `motor_emulator`. Así se prueba el empaquetado real de bytes; no lo reemplaces por mocks.

## Fuentes únicas de verdad

- **Geometría**: `src/ugv_bridge/config/geometria_robot.yaml`. La leen el xacro (`xacro.load_yaml`), `track_odometry_node` y el driver (montaje de la IMU). No copies números de ahí a otro sitio. `urdf/ugv.urdf` es un artefacto generado: no lo edites a mano.
- **Política RL**: la entrena un profesor, no nosotros; nuestro trabajo es que el robot ejecute su modelo. Es **autónoma** (flippers y orugas). Su contrato todavía no se conoce, así que **el contrato es un dato**: viaja en el `.json` junto al `.onnx` (`{"contrato": {...}}`) y el robot arma la observación a partir de él. El formato (catálogo de señales, `op`s, destinos y modos de acción, señales `extra/` para sensores futuros) está documentado en `ugv_bridge/contrato_politica.py`. Adaptarse a un modelo nuevo = escribir su `.json` (y, si hace falta, agregar señales al catálogo `senales()`), **no** tocar el nodo. `contrato_provisional()` es solo nuestra propuesta (40 obs, 6 acciones). `FORMATO_CONTRATO` solo se sube si cambia lo que el código sabe interpretar.
- **Frenos de la política** en `ugv_bridge/ejecutor_politica.py` (sin ROS, con tests): arranca desactivada, watchdog de `/joint_states`, topes duros (`vel_max_oruga`, `vel_max_flipper`) independientes del contrato y **parada segura** (orugas a 0, flippers quietos) al desactivarse, al saltar el watchdog o si la red falla. `politica_flippers.py` es solo pegamento ROS: la lógica nueva va en el ejecutor, donde se puede probar. No hay sensor de contacto aún ni `kinematic_guardian` (se eliminó a propósito: los límites los da la política).

## Al agregar archivos

Los nuevos launch, config (`.yaml`/`.json`), meshes, texturas o scripts tienen que coincidir con los `glob` de `data_files` en `setup.py`, o no se instalan. Los `.dae` dependen de `textures/`: sin ellas, Foxglove descarta la malla. Los nodos nuevos se registran en `entry_points`.
