# Cómo ejecutar cada cosa en ROS

Runbook práctico del proyecto (UGV de rescate). Cubre los dos entornos y todos los
nodos/launch. Guía rápida; el detalle de arquitectura está en `tesis/CLAUDE.md`.

## 0. Entornos ROS (no hay ROS en `/opt/ros` del host)

| Entorno | Distro | Cómo entrar |
|---------|--------|-------------|
| **Principal** | Jazzy | comando `rosdev` (= `distrobox enter ros2 -- bash`) o el acceso directo de escritorio "ROS 2 Jazzy" |
| Alternativo | Humble | env micromamba: `micromamba activate ros2` |

Dentro de `rosdev` el `~/.bashrc` ya sourcea Jazzy **y** el workspace `ugv_ws` en cada
terminal nueva. En **fish** (host), para cargar un workspace colcon: `rosws [ruta_ws]`.

> Cierra siempre los nodos con **Ctrl+C** en su terminal (evita procesos huérfanos).
> Si `ros2 node list` muestra nodos que ya cerraste: `ros2 daemon stop && ros2 daemon start`.

## 1. Compilar el workspace

```bash
rosdev                                   # entrar al contenedor Jazzy
cd ~/Proyectos/ugv_ws
colcon build                             # todos los paquetes
colcon build --packages-select ugv_bridge   # uno solo
source install/setup.bash                # tras cada build (en bash; en fish: rosws)
```

## 2. Track de FLIPPERS (paquete `ugv_bridge`)

Hay **dos launch** según lo que quieras hacer:

| Launch | Para qué | Corre |
|--------|----------|-------|
| **`view.launch.py`** | posar/mirar el robot: mover las 4 articulaciones con sliders, robot fijo y centrado | robot_state_publisher + joint_state_publisher_gui + RViz |
| **`flipper.launch.py`** | sistema completo: driver, IMU, odometría, EKF y locomoción | flipper_node + track_odometry_node + robot_state_publisher (+ EKF/RViz opcionales) |

### 2a. Posar los flippers con sliders (sin movimiento)

```bash
ros2 launch ugv_bridge view.launch.py
```

Abre RViz (fixed frame `base_link`, el robot no se desplaza) y una **ventana con un slider
por cada flipper** (`fl/fr/rl/rr`). Las juntas son `continuous` (**giro completo de 360°**),
así que el slider recorre todo el rango. Mueve un
slider y el brazo gira en tiempo real. Aquí `/joint_states` lo publica la GUI, así que **no**
se lanza `flipper_node` (entrarían en conflicto en el mismo tópico).

### 2b. Sistema completo (con locomoción, IMU y EKF)

```bash
# Básico: flipper_node + track_odometry_node + robot_state_publisher
ros2 launch ugv_bridge flipper.launch.py

# Con fusión sensorial EKF (robot_localization) y/o RViz:
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true use_rviz:=true

# Ajustar frecuencia del bucle de control o modo hardware:
ros2 launch ugv_bridge flipper.launch.py frecuencia_hz:=200.0 modo_simulacion:=true
```

Nodos sueltos (para depurar):

```bash
ros2 run ugv_bridge flipper_node
ros2 run ugv_bridge track_odometry_node --ros-args \
    --params-file src/ugv_bridge/config/geometria_robot.yaml
```

> Los parámetros de geometría (`radio_oruga`, `ancho_orugas`, antes
> `wheel_radius`/`track_width`) salen de `config/geometria_robot.yaml`, que es la
> única fuente de verdad y la comparte con el URDF y el guardián de colisiones.

### Cómo mover el robot en la simulación

El robot se controla con dos tópicos `std_msgs/Float64MultiArray` (4 valores, orden
`fl, fr, rl, rr`). `flipper_node` **retiene el último comando recibido** y lo aplica en
cada ciclo del bucle, así que **un solo mensaje basta** y persiste:

```bash
# FLIPPERS: posición en radianes. 0.78 = 45°, 0.0 = plano.  (se quedan ahí)
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [0.78,0.78,0.78,0.78]}"

# ORUGAS: velocidad en rad/s.  5.0 = avanzar (~0.25 m/s con r=0.05 m).
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [5.0,5.0,5.0,5.0]}"

# GIRAR en el sitio: lados opuestos (izq fl/rl vs der fr/rr):
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [5.0,-5.0,5.0,-5.0]}"

# DETENER las orugas (importante: la velocidad NO se resetea sola):
ros2 topic pub --once /cmd_tracks  std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0]}"
```

> ⚠️ **Usa `--once`, no `-r 20`.** Como el nodo retiene el último valor, no hace falta
> publicar en bucle. Un `ros2 topic pub -r 20 ...` deja un **publicador vivo** que sigue
> mandando velocidad para siempre; si lo olvidas (o abres varios), el robot se conduce solo
> indefinidamente y su pose se dispara (lo vimos llegar a ~180 m y salir de cuadro en RViz).
> Si usas `-r`, **ciérralo con Ctrl+C** al terminar. Para ver publicadores olvidados:
> `ros2 topic info /cmd_tracks` (Publisher count) o `pgrep -af "topic pub"`.

> Recordatorio: la velocidad de orugas es persistente. Tras avanzar, manda explícitamente
> `[0,0,0,0]` para frenar; no basta con dejar de publicar.

Inspeccionar:

```bash
ros2 topic echo /joint_states          # 8 motores (4 orugas + 4 flippers)
ros2 topic echo /imu/data_raw          # IMU (cuaternión)
ros2 topic echo /odom                  # odometría de orugas
ros2 topic echo /odometry/filtered     # salida del EKF (solo con use_ekf:=true)
ros2 run tf2_ros tf2_echo base_link flipper_fl_link   # TF de un flipper
ros2 run rqt_plot rqt_plot /joint_states/position[4]  # ángulo de un flipper en el tiempo
```

En RViz: fixed frame `base_link` (sin EKF) u `odom` (con EKF); displays RobotModel + TF.

### Que el modelo 3D se INCLINE en Foxglove (no sólo los flippers)

`robot_state_publisher` dibuja únicamente la geometría **interna** del robot: mueve
los flippers respecto a `base_link`, pero **nunca mueve `base_link` dentro del
mundo**. Por eso, aunque `/imu/data_raw` traiga una orientación perfecta, el modelo
se ve plano: falta quién publique dónde está el chasis.

Eso lo hace el **EKF**, con el TF `odom -> base_link`. Hay que levantarlo:

```bash
ros2 launch ugv_bridge can_sim.launch.py use_ekf:=true
# o, sin emulador CAN (gemelo digital):
ros2 launch ugv_bridge flipper.launch.py use_ekf:=true
```

En el panel 3D de Foxglove, poner **Display frame = `odom`** (con `base_link` el
robot queda fijo y sólo se mueven los flippers, que es justo el síntoma).

Comprobar que la inclinación llega de verdad:

```bash
ros2 run tf2_ros tf2_echo odom base_link      # debe cambiar el RPY al inclinarse
ros2 topic pub --once /cmd_flippers std_msgs/msg/Float64MultiArray "{data: [1.0,1.0,0.0,0.0]}"
# -> pitch NEGATIVO (~-34°): en REP-103 el eje Y apunta a la izquierda, así que
#    morro ARRIBA es pitch negativo (al revés que en convención aeronáutica).
```

Las dos cadenas de TF son complementarias y no se pisan:

```
odom --[EKF]--> base_link --[robot_state_publisher]--> flippers, imu_link
```

`track_odometry_node` emite `odom -> base_link` **sólo cuando el EKF está apagado**
(el launch lo ajusta solo). Si ambos lo publicaran, `base_link` tendría dos padres
y el árbol TF quedaría inválido: en Foxglove el modelo tiembla o desaparece.

## 3. Track de NAVEGACIÓN 2D (simulación + política RL)

```bash
ros2 run ugv_sim sim_node        # avanza RobotEnv 20 Hz: publica /scan, /odom, TF; escucha /cmd_vel
ros2 run ugv_policy policy_node  # decide con el cerebro (heurística/RL) y publica /cmd_vel
#   con modelo RL: ros2 run ugv_policy policy_node --ros-args -p model_path:=/ruta/modelo_robot.zip
ros2 run ugv_gcs gcs_node        # GCS mínima ROS2 (mini-mapa 2D)

# Todo junto (sim + policy + la GUI simu.py de tesis/):
ros2 launch ~/Proyectos/ugv_ws/ugv.launch.py
```

## 4. App / simulación de escritorio (carpeta `tesis/`, venv propio)

No es ROS puro, pero `simu.py` importa `rclpy` (necesita ROS sourceado). Desde el venv de `tesis/`:

```bash
python simu.py             # GCS completa (PySide6 + OpenGL)
python robot_env.py        # demo headless del entorno + heurística (métricas)
python entrenar.py         # entrena PPO -> modelo_robot.zip (requiere gymnasium/sb3/torch)
python entrenar.py eval    # evalúa un modelo entrenado
```

## 5. Instalar dependencias ROS

```bash
# En Jazzy (dentro de rosdev):
sudo apt install ros-jazzy-<paquete>       # ya instalados: robot-localization, rviz2
# En Humble (micromamba):
mamba install ros-humble-<paquete>
```
