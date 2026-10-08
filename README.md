# TESIS — Workspace ROS 2 del UGV de rescate (RMD)

Workspace de **ROS 2 Jazzy** para un vehículo terrestre no tripulado (UGV) de
rescate con **orugas y flippers articulados**. Incluye el puente con el hardware
de motores, la odometría, la fusión sensorial (EKF), el modelo del robot (URDF)
y la inferencia de la política de flippers entrenada por refuerzo en el PC.

```bash
cd ~/TESIS && git checkout PRINCIPAL
colcon build --packages-select ugv_bridge && source install/setup.bash
ros2 launch ugv_bridge can_studio.launch.py   # simulación CAN + espía + Foxglove
```

Después, conectar Foxglove a `ws://localhost:8765` y cargar el layout
`src/ugv_bridge/config/ugv_control_v4.json`. **Foxglove es la única interfaz**:
no hay RViz ni ventanas Qt en ningún launch.

## Paquete (`src/`)

| Paquete | Rol | Ejecutables |
|---------|-----|-------------|
| **ugv_bridge** | Todo: puente ROS ↔ motores (orugas + flippers), IMU, odometría, EKF, URDF, launch, espía del bus CAN e inferencia de la política | `flipper_node`, `track_odometry_node`, `teleop_flippers`, `can_monitor`, `politica_flippers`, `motor_emulator`, `pi3hat_imu`, `esp32_bridge`, `gui_a_comandos` *(legado)* |

**Un solo paquete, a propósito.** El workspace tenía además `ugv_core`,
`ugv_sim`, `ugv_policy` y `ugv_msgs`, de una versión anterior del proyecto que
modelaba navegación 2D plana: telémetros, `/cmd_vel` y tres acciones discretas.
Esa no es la tarea de la tesis. Este robot tiene orugas y flippers articulados
y se comanda por `/cmd_tracks` y `/cmd_flippers`, y no había nada que conectara
lo uno con lo otro. Se eliminaron en octubre de 2026 para que no quedara código
muerto con apariencia de sistema. Lo único que se rescató de ellos es la
inferencia de la política, que ahora vive en `ugv_bridge`.

## Arquitectura del track de flippers (`ugv_bridge`)

```
Foxglove (paneles Publish y Teleop)  /  política RL
   │  /cmd_tracks (vel. orugas)   /cmd_flippers (pos. flippers)
   ▼
flipper_node  ──►  RMD_Hardware (real pi3hat/CAN  o  gemelo digital)
   │  /joint_states   /imu/data_raw
   ├──► track_odometry_node ──► /odom ─┐
   │                                   ├──► EKF (robot_localization) ──► /odometry/filtered + TF
   └──► robot_state_publisher (URDF) ──► TF de cada flipper ──► Foxglove
```

- **8 motores**: IDs 1–4 orugas (comando de velocidad), IDs 5–8 flippers (comando de posición).
- **Flippers**: juntas `continuous`, giran **360°** sin topes.
- **Sim-to-real**: el mismo nodo sirve para simulación y hardware; solo cambia el
  parámetro `modo_simulacion`. El modo hardware real (pi3hat + CAN + IMU) queda como TODO.

## Geometría del robot: una sola fuente de verdad

Todas las dimensiones físicas (posición de los motores de flipper, largo del
flipper, radio de las ruedas, radio y separación de las orugas, montaje de la IMU)
viven en **[`src/ugv_bridge/config/geometria_robot.yaml`](src/ugv_bridge/config/geometria_robot.yaml)**.
De ahí las leen, sin duplicar un solo número:

```
config/geometria_robot.yaml          <-- editar SOLO aquí
        |
        +--> urdf/ugv.urdf.xacro     (xacro.load_yaml)  --> modelo 3D, colisiones, TF
        +--> track_odometry_node     (radio/ancho de orugas)
        +--> flipper_node / driver   (IMU sintética, montaje de la IMU)
```

Cuando lleguen las medidas reales del chasis:

```bash
# 1. Editar config/geometria_robot.yaml
# 2. Refrescar el URDF plano (para Foxglove y herramientas que no procesan xacro)
bash src/ugv_bridge/scripts/generar_urdf.sh
# 3. Recompilar
colcon build
```

Los launch procesan el `.xacro` en caliente, así que el modelo **nunca** queda
desfasado del YAML. El `.urdf` plano es un artefacto generado: no editarlo a mano.

## Compilar

```bash
cd ~/Proyectos/ugv_ws
colcon build
source install/setup.bash
```

## Uso rápido

> **La única interfaz es Foxglove.** Ningún launch abre RViz ni ventanas Qt: la
> GUI de sliders y la vieja ventana de `can_monitor` publicaban en los mismos
> tópicos de comando que Foxglove y se pisaban entre sí (`flipper_node` retiene
> el último comando que recibe). Todos los launch levantan el puente WebSocket
> en el 8765; conecta con `ws://localhost:8765` y el layout
> `config/ugv_control_v4.json`.

```bash
# Posar el robot (gemelo digital), sin locomoción:
ros2 launch ugv_bridge view.launch.py

# Sistema completo (driver + odometría + EKF + Foxglove):
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true

# Simulación con tramas CAN reales sobre vcan0, y con espía del bus:
ros2 launch ugv_bridge can_view.launch.py
ros2 launch ugv_bridge can_studio.launch.py

# Matar ventanas/publicadores huérfanos de una sesión vieja:
bash src/ugv_bridge/scripts/limpiar_fantasmas.sh

# Mover (un solo mensaje basta; el nodo retiene el último comando):
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [0.78,0.78,0.78,0.78]}"
ros2 topic pub --once /cmd_tracks   std_msgs/msg/Float64MultiArray "{data: [5.0,5.0,5.0,5.0]}"

# Política de flippers, solo observando las señales táctiles (no comanda nada):
ros2 launch ugv_bridge flipper.launch.py use_politica:=true
```

> ⚠️ Usa `--once`, no `-r`. La velocidad de las orugas es persistente: para frenar,
> publica explícitamente `[0,0,0,0]` en `/cmd_tracks`.

## RL: se entrena en el PC, se ejecuta en la Raspberry

El entrenamiento **no** corre en la Pi. En el robot solo va un archivo `.onnx`
ya entrenado, que ejecuta `politica_flippers` con `onnxruntime`. La Pi no
necesita gymnasium, ni stable-baselines3, ni torch.

```
PC de alta gama                          Raspberry Pi
---------------                          ------------
simulador + RL (PPO/SAC)                 politica_flippers
        |                                  lee /joint_states, /imu/data_raw
        v                     .onnx        construye la observación
scripts/exportar_onnx.py  ──────────────▶  ejecuta la red (onnxruntime)
                          .json            publica /cmd_flippers
```

El **contrato** (`ugv_bridge/contrato_politica.py`) es la única fuente de verdad
del vector de observación y de acción, y lo comparten los dos lados. El `.json`
que acompaña al modelo graba su versión, y el nodo se niega a cargar un modelo
cuya versión no sea la suya: si los layouts no coinciden, la red no falla, solo
devuelve ángulos plausibles y equivocados.

Detalle completo del objetivo (percepción táctil sin sensor táctil), de las
señales y de los frenos de seguridad en `COMO_EJECUTAR.md`, sección 3.

## Documentación

- **[COMO_EJECUTAR.md](COMO_EJECUTAR.md)** — runbook detallado (entornos, launch, tópicos, inspección).
- **[DEFENSA_ROS2.txt](DEFENSA_ROS2.txt)** — guía de preguntas/respuestas de ROS 2 para la defensa.
- **[BITACORA_2026-07-11.txt](BITACORA_2026-07-11.txt)** — bitácora de construcción del track de flippers.
